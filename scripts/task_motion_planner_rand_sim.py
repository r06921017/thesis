#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
Solve task planning with priority first.
self.property_dict: reward in this case
"""

from task_motion_planner_fcfs_sim import *
import random


class TaskMotionPlannerPF(TaskMotionPlannerFCFSSim):
    def __init__(self):
        TaskMotionPlannerFCFSSim.__init__(self)
        self.property_key = -1

    @staticmethod
    def show_property(in_dict):
        for key, instr_id in in_dict.iteritems():
            rospy.loginfo('id: {0}, reward: {1}'.format(key, instr_id))
        return

    def plan_task(self, in_instructions):
        rospy.loginfo('Planning task ...')
        s_time = time.time()  # for task planning time

        # Convert InstructionArray into dictionary
        if type(in_instructions) == thesis.msg._InstructionArray.InstructionArray:

            self.save_csv_flag = True

            for instr in in_instructions.data:
                self.instr_dest_dict[instr.destination].add(instr.id)
                self.instr_dict[instr.id] = instr

        self.show_instr()

        # Fetch the destination from the task
        if len(self.instr_dict.keys()) > 0:
            # Random pick an instruction to complete
            if self.property_key not in self.instr_dict.keys():
                self.property_key = random.choice(self.instr_dict.keys())

            dest_node = self.instr_dict[self.property_key].destination  # destination node
            rospy.loginfo('destination node: {0}'.format(dest_node))

            # Calculate the shortest path nodes from current node to the goal node
            temp_path = nx.shortest_path(self.map_graph, self.cur_node, dest_node, weight='weight')
            rospy.logdebug('temp_path = {0}'.format(temp_path))
            if len(temp_path) > 1:
                self.next_node = temp_path[1]  # next neighbor node for motion planner
            else:
                self.next_node = self.cur_node
            rospy.loginfo('plan_task result: {0}'.format(self.next_node))

            # evaluate planning time
            self.plan_time += time.time() - s_time
            rospy.set_param('/thesis/plan_time', self.plan_time)
            rospy.loginfo('plan_time: {0}'.format(self.plan_time))

        return

    def plan_motion_viz(self):
        if self.cur_node == self.next_node:
            rospy.loginfo('Motion: Reach node {0}.'.format(self.next_node))

            if len(self.instr_dest_dict[self.cur_node]) > 0:

                for idx in self.instr_dest_dict[self.cur_node]:
                    if idx == self.property_key:
                        do_instr = self.instr_dict[idx]
                        rospy.loginfo('Do instr {0}: {1}'.format(idx, do_instr.function))
                        rospy.sleep(do_instr.duration)

                        # for experiment evaluation# for experiment evaluation
                        self.cal_accu_reward(do_instr)
                        self.done_instr.append(do_instr.id)
                        # end

                        del self.instr_dict[do_instr.id]
                        self.property_key = -1
                        self.instr_dest_dict[self.cur_node].remove(do_instr.id)

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

                # for experiment: save the accumulative reward, all
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
        rospy.sleep(2)
        rospy.loginfo('Done!')
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
        rospy.sleep(2)
        rospy.loginfo('Done!')
        return


if __name__ == '__main__':
    rospy.init_node(os.path.basename(__file__).split('.')[0], log_level=rospy.INFO)
    tamp = TaskMotionPlannerPF()
    tamp.run_plan_viz()
