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

        rospy.loginfo('TAMP_Opt Initialized!')

    def plan_task(self, in_instructions):
        # This is only for static instruction list
        rospy.loginfo('Start task planning!')

        # Convert InstructionArray into dictionary
        if type(in_instructions) == thesis.msg._InstructionArray.InstructionArray:
            for instr in in_instructions.data:
                self.instr_dest_dict[instr.destination].add(instr.id)
                self.instr_dict[instr.id] = instr

            # print 'self.instr_dict: ', self.instr_dict

        rospy.loginfo('self.instr_dict.keys(): {0}'.format(self.instr_dict.keys()))

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
            s_time = time.time()
            node_seq = list(itertools.permutations(self.instr_dict.keys()))
            rospy.logdebug('len node_seq: {0}'.format(len(node_seq)))

            reward_seq = np.zeros(len(node_seq))
            for s_id, seq in enumerate(node_seq):
                path_len = 0
                temp_node = self.cur_node
                for instr_id in seq:
                    path_len += self.shortest_path[self.instr_dict[instr_id].destination, temp_node]
                    reward_seq[s_id] += self.instr_dict[instr_id].r * (self.instr_dict[instr_id].b ** path_len)
                    temp_node = self.instr_dict[instr_id].destination

            rospy.loginfo('reward_seq: {0}'.format(reward_seq))
            self.opt_seq = list(node_seq[np.argmax(reward_seq)])  # list of instr_id
            rospy.loginfo('opt_seq: {0}'.format(self.opt_seq))

            # evaluate planning time
            self.plan_time = time.time() - s_time
            rospy.loginfo('plan time: {0}')
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

                            del self.instr_dict[do_instr.id]
                            self.opt_seq.pop(0)
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
    rospy.init_node(os.path.basename(__file__).split('.')[0], log_level=rospy.DEBUG)
    tamp = TaskMotionPlannerOptSim()
    tamp.run_plan_viz()
