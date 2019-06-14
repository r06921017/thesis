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
        # self.cur_instr.data = list(instr_list.data)

        # Convert InstructionArray into dictionary
        if type(in_instructions) == thesis.msg._InstructionArray.InstructionArray:
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

        return


if __name__ == '__main__':
    rospy.init_node(os.path.basename(__file__).split('.')[0], log_level=rospy.INFO)
    tamp = TaskMotionPlannerSF()
    tamp.run_plan_viz()
