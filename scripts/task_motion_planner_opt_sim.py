#!/usr/bin/env python
# -*- coding: utf8 -*-
"""
Solve task planning with dynamic programming (value iteration)
"""
from task_motion_planner_fcfs_sim import *
import itertools


class TaskMotionPlannerOptSim(TaskMotionPlannerFCFSSim):
    def __init__(self):
        """
        self.cur_node: the node where robot starts to move, initial at charge (2), type: int
        """
        TaskMotionPlannerFCFSSim.__init__(self)
        self.shortest_path = nx.floyd_warshall_numpy(self.map_graph)
        self.reward_list = list()
        self.opt_seq = list()
        self.opt_accu_reward_list = list()
        self.opt_time_list = list()
        self.instr_counter = -1
        self.save_opt_flag = True

        rospy.loginfo('TAMP_Opt Initialized!')

    def plan_task(self, in_instructions):
        # This is only for static instruction list
        rospy.loginfo('Start task planning!')

        s_time = time.time()
        _store_time = 0.0

        # Convert InstructionArray into dictionary
        if type(in_instructions) == thesis.msg._InstructionArray.InstructionArray:
            for instr in in_instructions.data:
                self.instr_dest_dict[instr.destination].add(instr.id)
                self.instr_dict[instr.id] = instr

        # rospy.loginfo('self.instr_dict.keys(): {0}'.format(self.instr_dict.keys()))

        if len(self.opt_seq) > 0:

            dest_node = self.instr_dict[self.opt_seq[0]].destination  # destination node
            rospy.loginfo('destination node from task planner: {0}'.format(dest_node))

            # Calculate the shortest path nodes from current node to the goal node
            temp_path = nx.shortest_path(self.map_graph, self.cur_node, dest_node, weight='weight')
            rospy.logdebug('temp_path = {0}'.format(temp_path))
            if len(temp_path) > 1:
                self.next_node = temp_path[1]  # next neighbor node for motion planner
            else:
                self.next_node = self.cur_node

            rospy.loginfo('plan_task result: {0}'.format(self.next_node))

        else:
            rospy.logwarn('self.opt_seq is empty.')
            self.instr_counter = len(self.instr_dict.keys())
            node_seq = list(itertools.permutations(self.instr_dict.keys()))
            rospy.logdebug('len node_seq: {0}'.format(len(node_seq)))

            reward_seq = np.zeros(len(node_seq))
            for s_id, seq in enumerate(node_seq):
                path_len = 0
                temp_node = self.cur_node
                for instr_id in seq:
                    temp_duration = self.shortest_path[self.instr_dict[instr_id].destination, temp_node] + \
                                    np.around(self.instr_dict[instr_id].duration / self.sim_time_step)
                    path_len += temp_duration

                    reward_seq[s_id] += self.instr_dict[instr_id].r * (self.instr_dict[instr_id].b ** path_len)
                    temp_node = self.instr_dict[instr_id].destination

            rospy.loginfo('reward_seq: {0}'.format(reward_seq))
            self.opt_seq = list(node_seq[np.argmax(reward_seq)])  # list of instr_id

            if self.save_opt_flag:
                # store theoretical optimal rewards once!
                _ss_time = time.time()
                rospy.loginfo('Saving optimal reward')
                __temp_len = 0.0
                __temp_reward = 0.0
                __temp_node = self.cur_node
                __temp_t_list = list()
                __temp_r_list = list()

                self.save_done_instr_id(id_seq=self.opt_seq)

                for seq_id, instr_id in enumerate(self.opt_seq):
                    __temp_len += self.shortest_path[self.instr_dict[instr_id].destination, __temp_node] + \
                                  np.around(self.instr_dict[instr_id].duration / self.sim_time_step)

                    __temp_reward += self.instr_dict[instr_id].r * (self.instr_dict[instr_id].b ** __temp_len)
                    __temp_node = self.instr_dict[instr_id].destination

                    __temp_r_list.append(__temp_reward)
                    __temp_t_list.append(__temp_len)

                opt_csv_file = self._pkg_dir + '/config/' + os.path.basename(__file__).split('.')[0] + '_opt_reward.csv'
                output_df = pd.DataFrame({'time': __temp_t_list, 'reward': __temp_r_list})
                output_df.to_csv(opt_csv_file, index=False)

                self.save_opt_flag = False
                rospy.loginfo('Done!')

                _store_time = time.time() - _ss_time

        # evaluate planning time
        self.plan_time += time.time() - s_time - _store_time
        rospy.loginfo('plan time: {0}'.format(self.plan_time))
        rospy.set_param('/thesis/plan_time', self.plan_time)            

        return

    def plan_motion_viz(self):
        if len(self.opt_seq) > 0:
            if self.cur_node == self.next_node:
                rospy.loginfo('Motion: Reach node {0}.'.format(self.next_node))

                if len(self.instr_dest_dict[self.cur_node]) > 0:
                    print 'self.instr_dest_dict[self.cur_node] = ', self.instr_dest_dict[self.cur_node]

                    for idx in self.instr_dest_dict[self.cur_node]:
                        if idx == self.opt_seq[0]:
                            do_instr = self.instr_dict[idx]
                            rospy.loginfo('Do instr {0}: {1}'.format(idx, do_instr.function))
                            rospy.sleep(do_instr.duration)
                            done_time = time.time()

                            # calculate obtained reward
                            # rospy.set_param('instr_start_time') is in "instruction_constructor.py"
                            _temp_step = np.around((done_time-rospy.get_param('/instr_start_time'))/self.sim_time_step)
                            self.accu_r += do_instr.r * (do_instr.b ** _temp_step)

                            self.accu_r_list.append(self.accu_r)
                            self.time_r_list.append(_temp_step)

                            del self.instr_dict[do_instr.id]
                            self.opt_seq.pop(0)
                            self.instr_dest_dict[self.cur_node].remove(do_instr.id)
                            self.instr_counter -= 1

                            break

                    # Convert undo_tasks to a list() and publish to /thesis/instruction_buffer
                    undo_instr_list = list()
                    for key, value in self.instr_dict.iteritems():
                        undo_instr_list.append(value)

                    self.task_pub.publish(undo_instr_list)

                else:
                    rospy.loginfo('No instructions on task {0}'.format(self.cur_node))
                    if len(self.instr_dict) > 0:
                        self.plan_task(self.instr_dict)

            else:
                rospy.loginfo('Motion: from {0} to {1}'.format(self.cur_node, self.next_node))
                self.move_adjacency_node(self.next_node, sim=False, render=True)

        # save the accumulative reward
        else:
            if self.instr_counter == 0:
                rospy.loginfo('Saving reward')
                csv_file = self._pkg_dir + '/config/' + os.path.basename(__file__).split('.')[0] + '.csv'
                output_df = pd.DataFrame({'time': self.time_r_list, 'reward': self.accu_r_list})
                output_df.to_csv(csv_file, index=False, columns=['time', 'reward'])
                rospy.loginfo('Done!')
                self.instr_counter = -1

        return


if __name__ == '__main__':
    rospy.init_node(os.path.basename(__file__).split('.')[0], log_level=rospy.DEBUG)
    tamp = TaskMotionPlannerOptSim()
    tamp.run_plan_viz()
