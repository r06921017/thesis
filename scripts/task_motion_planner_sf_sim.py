#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
Solve task planning with current shortest time (do + move) first.
"""

from task_motion_planner_pf_sim import *


class TaskMotionPlannerSF(TaskMotionPlannerPFSim):
    def __init__(self):
        TaskMotionPlannerPFSim.__init__(self)
        self.shortest_path = nx.floyd_warshall_numpy(self.map_graph)

    def plan_task(self, in_instructions):
        rospy.loginfo('Planning task ...')
        s_time = time.time()  # for task planning time
        # self.cur_instr.data = list(instr_list.data)

        # Convert InstructionArray into dictionary
        if type(in_instructions) == thesis.msg._InstructionArray.InstructionArray:

            self.save_csv_flag = True

            for instr in in_instructions.data:
                self.instr_dest_dict[instr.destination].add(instr.id)
                self.instr_dict[instr.id] = instr

        self.show_instr()

        # Fetch the destination from the task
        if len(self.instr_dict.keys()) > 0:
            # Create dictionary = {'id': 'property'}
            _property_dict = dict()

            for _, instr in self.instr_dict.iteritems():
                _property_dict[instr.id] = instr.duration + \
                                           self.shortest_path[instr.destination, self.cur_node]*self.time_step
            self.show_property(_property_dict)

            # Get the key(id) with the min value(r) in self.property_dict
            self.property_key = min(_property_dict.iterkeys(), key=(lambda key: _property_dict[key]))
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
        rospy.loginfo('Done!')
        return


if __name__ == '__main__':
    rospy.init_node(os.path.basename(__file__).split('.')[0], log_level=rospy.INFO)
    tamp = TaskMotionPlannerSF()
    tamp.run_plan_viz()
