#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
Solve task planning with priority first.
self.property_dict: reward in this case
"""

from task_motion_planner_fcfs import *


class TaskMotionPlannerPF(TaskMotionPlannerFCFS):
    def __init__(self):
        TaskMotionPlannerFCFS.__init__(self)
        self.property_key = -1

    @staticmethod
    def show_property(in_dict):
        for key, instr_id in in_dict.iteritems():
            print 'id: {0}, reward: {1}'.format(key, instr_id)
        return

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
            # Create reward dictionary = {'id': 'r'}
            _property_dict = dict()
            self.property_key = -1

            for _, instr in self.instr_dict.iteritems():
                _property_dict[instr.id] = instr.r
            self.show_property(_property_dict)

            # Get the key(id) with the max value(r) in self.property_dict
            self.property_key = max(_property_dict.iterkeys(), key=(lambda key: _property_dict[key]))
            dest_node = self.instr_dict[self.property_key].destination  # destination node
            rospy.loginfo('destination node: {0}'.format(dest_node))

            # Calculate the shortest path nodes from current node to the goal node
            temp_path = nx.shortest_path(self.map_graph, self.cur_node, dest_node, weight='weight')
            print 'temp_path = ', temp_path
            if len(temp_path) > 1:
                self.next_node = temp_path[1]  # next neighbor node for motion planner
            else:
                self.next_node = self.cur_node
        return

    def plan_motion_viz(self):
        if self.cur_node == self.next_node:
            rospy.loginfo('Motion: Reach node {0}.'.format(self.next_node))

            if len(self.instr_dest_dict[self.cur_node]) > 0:
                rospy.loginfo('self.instr_dest_dict[self.cur_node] = '.format(self.instr_dest_dict[self.cur_node]))

                for idx in self.instr_dest_dict[self.cur_node]:
                    if idx == self.property_key:
                        do_instr = self.instr_dict[idx]
                        rospy.loginfo('Do instr {0}: {1}'.format(idx, do_instr.function))
                        rospy.sleep(do_instr.duration)

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

        else:
            rospy.loginfo('Motion: from {0} to {1}'.format(self.cur_node, self.next_node))
            self.move_adjacency_node(self.next_node, sim=False, render=True)

        return


if __name__ == '__main__':
    rospy.init_node(os.path.basename(__file__).split('.')[0], log_level=rospy.INFO)
    tamp = TaskMotionPlannerPF()
    tamp.run_plan_viz()
