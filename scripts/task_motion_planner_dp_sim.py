#!/usr/bin/env python
# -*- coding: utf8 -*-
"""
Solve task planning with dynamic programming (value iteration)
"""
from task_motion_planner_fcfs_sim import *
import operator


class TaskMotionPlannerDPSim(TaskMotionPlannerFCFSSim):
    def __init__(self):
        """
        self.cur_node: the node where robot starts to move, initial at charge (2), type: int
        """
        TaskMotionPlannerFCFSSim.__init__(self)
        self.shortest_path = nx.floyd_warshall_numpy(self.map_graph)

        print '----------------------------------------'
        print self.shortest_path
        print self.shortest_path.shape
        print '----------------------------------------'
        rospy.loginfo('TAMP Initialized!')

    def value_iter(self):
        rospy.loginfo('value_iter, cur_node: {0}, next_node: {1}'.format(self.cur_node, self.next_node))

        # Compare the accumulated reward of neighbor, move to the maximum one.
        neighbor_node = np.where(np.array(self.cur_neighbor) > 0)[0].tolist()
        candidate_steps = dict()  # {neighbor_node: {'reward': float, 'neighbor': np.array}}

        for n in neighbor_node:
            temp_neighbor = self.move_adjacency_node(n)  # assuming moving toward the neighbor node n.
            temp_neighbor_node = np.where(np.array(temp_neighbor) > 0)[0].tolist()
            rospy.logdebug('temp_neighbor_node = {0}'.format(temp_neighbor_node))

            # Calculate accumulated reward of a candidate step from all instructions
            total_reward = 0.0
            for _, instr in self.instr_dict.iteritems():
                if instr.prev_id not in self.instr_dict.keys():
                    # Possible distances from instr to candidate steps
                    temp_dis = np.zeros(len(temp_neighbor_node)).astype(float)
                    for j, temp_n in enumerate(temp_neighbor_node):
                        temp_dis[j] = self.shortest_path[instr.destination, temp_n] + temp_neighbor[temp_n]

                    total_reward += instr.r * pow(instr.b, np.min(temp_dis))

            rospy.loginfo('total reward in candidate node {0}: {1}'.format(n, total_reward))

            candidate_steps[n] = {'reward': total_reward, 'neighbor': temp_neighbor}

        # Pick the node with maximum accumulated reward
        self.next_node = max(candidate_steps, key=lambda x: candidate_steps[x]['reward'])
        return

    def plan_task(self, in_instructions):
        rospy.loginfo('Start task planning!')

        s_time = time.time()

        # Convert InstructionArray into dictionary
        if type(in_instructions) == thesis.msg._InstructionArray.InstructionArray:

            self.save_csv_flag = True

            for instr in in_instructions.data:
                self.instr_dest_dict[instr.destination].add(instr.id)
                self.instr_dict[instr.id] = instr

            # print 'self.instr_dict: ', self.instr_dict

        rospy.logdebug('len(self.instr_dict.keys()): {0}'.format(len(self.instr_dict.keys())))

        if len(self.instr_dict.keys()) > 0:  # if there exists instructions
            if len(self.instr_dest_dict[self.cur_node]) > 0:
                # for two-stage instruction, count the instructions that still have previous instr
                temp_count = 0
                for instr_id in self.instr_dest_dict[self.cur_node]:
                    if self.instr_dict[instr_id].prev_id in self.instr_dict.keys():
                        temp_count += 1

                # all the instructions in cur_node are sequential
                if temp_count == len(self.instr_dest_dict[self.cur_node]):
                    self.value_iter()  # update self.next_node

                # there are instructions exist in cur_node
                else:
                    self.next_node = self.cur_node

                # for single stage instruction
                # self.next_node = self.cur_node

            else:
                self.value_iter()  # update self.next_node

            rospy.loginfo('plan_task result: {0}'.format(self.next_node))
            # self.motion_pub.publish(self.next_node)

            self.plan_time += time.time() - s_time
            rospy.set_param('/thesis/plan_time', self.plan_time)
            rospy.loginfo('plan_time: {0}'.format(self.plan_time))

        return

    def plan_motion_viz(self):
        if self.cur_node == self.next_node:
            rospy.loginfo('Motion: Reach node {0}.'.format(self.next_node))

            if len(self.instr_dest_dict[self.cur_node]) > 0:
                # Create reward_dict = {'id (int)': 'reward (float)'}
                reward_dict = dict()
                for idx in self.instr_dest_dict[self.cur_node]:
                    # Check two-stage instruction
                    if self.instr_dict[idx].prev_id not in self.instr_dict.keys():
                        reward_dict[self.instr_dict[idx].id] = self.instr_dict[idx].r

                # Sort the instructions with the max reward
                for r in sorted(reward_dict.items(), key=operator.itemgetter(1), reverse=True):
                    do_instr = self.instr_dict[r[0]]
                    rospy.loginfo('Do instr {0}: {1}'.format(do_instr.id, do_instr.function))
                    rospy.sleep(do_instr.duration)

                    # for experiment evaluation
                    self.cal_accu_reward(do_instr)  # calculate the accumulative reward
                    self.done_instr.append(do_instr.id)
                    # end

                    del self.instr_dict[r[0]]
                    self.show_instr()

                # Reset the set when all the tasks in the instructions are done.
                self.instr_dest_dict[self.cur_node].clear()
                rospy.logdebug('self.instr_dest_dict: {0}'.format(self.instr_dest_dict))

                # Convert undo_tasks to a list() and publish to /thesis/instruction_buffer
                undo_instr_list = list()
                for key, value in self.instr_dict.iteritems():
                    undo_instr_list.append(value)

                self.task_pub.publish(undo_instr_list)

            else:
                rospy.loginfo('No instructions on task {0}'.format(self.cur_node))
                if len(self.instr_dict) > 0:
                    self.plan_task(self.instr_dict)

                # save the accumulative reward, all
                elif self.save_csv_flag:
                    self.save_done_instr_id()
                    self.save_accu_reward()

        else:
            rospy.loginfo('Motion: from {0} to {1}'.format(self.cur_node, self.next_node))
            self.move_adjacency_node(self.next_node, sim=False, render=True)

        return

    def cal_accu_reward(self, input_instr):
        # calculate obtained reward
        # rospy.set_param('instr_start_time') is in "instruction_constructor.py"
        _temp_step = np.around((time.time() - rospy.get_param('/instr_start_time')) / self.sim_time_step)
        self.accu_r += input_instr.r * (input_instr.b ** _temp_step)
        self.accu_r_list.append(self.accu_r)
        self.time_r_list.append(_temp_step)
        rospy.logdebug('accu reward: {0}'.format(self.accu_r))

        return

    def save_accu_reward(self):
        rospy.loginfo('Saving accumulative reward')
        csv_file = self._pkg_dir + '/experiments/' + os.path.basename(__file__).split('.')[0] + '_reward.csv'
        output_df = pd.DataFrame({'time': self.time_r_list, 'reward': self.accu_r_list})
        output_df.to_csv(csv_file, index=False, columns=['time', 'reward'])
        rospy.loginfo('Done!')
        rospy.sleep(1)
        self.save_csv_flag = False
        return

    def save_done_instr_id(self, id_seq=None):
        rospy.loginfo('Save done instructions')
        if id_seq is None:
            id_seq = self.done_instr
        rospy.loginfo('done_instr: {0}'.format(id_seq))
        file_name = self._pkg_dir + '/experiments/' + os.path.basename(__file__).split('.')[0] + '_done.csv'
        output_df = pd.DataFrame({'done': id_seq})
        output_df.to_csv(file_name, index=False)
        rospy.sleep(1)
        rospy.loginfo('Done!')
        return


if __name__ == '__main__':
    rospy.init_node(os.path.basename(__file__).split('.')[0], log_level=rospy.INFO)
    tamp = TaskMotionPlannerDPSim()
    tamp.run_plan_viz()  # this is for simulation
