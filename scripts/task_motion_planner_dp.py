#!/usr/bin/env python
# -*- coding: utf8 -*-
"""
Solve task planning with dynamic programming (value iteration)
"""
from task_motion_planner_fcfs import *


class TaskMotionPlannerDP(TaskMotionPlannerFCFS):
    def __init__(self):
        """
        self.cur_node: the node where robot starts to move, initial at charge (2), type: int
        """
        TaskMotionPlannerFCFS.__init__(self)
        self.shortest_path = nx.floyd_warshall_numpy(self.map_graph)
        print '&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&'
        print self.shortest_path
        print self.shortest_path.shape
        print '&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&'

    def plan_task(self, in_instructions):

        # Convert InstructionArray into dictionary
        if type(in_instructions) == thesis.msg._InstructionArray.InstructionArray:
            print 'in_instructions.data len = ', len(in_instructions.data)
            for instr in in_instructions.data:
                self.instr_dest_dict[instr.destination].add(instr.id)
                self.instr_dict[instr.id] = instr

        self.show_instr()

        if len(self.instr_dict.keys()) > 0:  # if there exists instructions

            # Compare the accumulated reward of neighbor, move to the maximum one.
            neighbor_node = np.where(np.array(self.cur_neighbor) > 0)[0].tolist()
            candidate_steps = dict()  # {neighbor_node: {'reward': int, 'neighbor': np.array}}
            # print 'neighbor_node = ', neighbor_node

            for n in neighbor_node:
                temp_neighbor = self.move_adjacency_node(n)  # assuming moving toward the neighbor node n.
                temp_neighbor_node = np.where(np.array(temp_neighbor) > 0)[0].tolist()
                print 'temp_neighbor_node = ', temp_neighbor_node

                # Calculate accumulated reward of a candidate step from all instructions
                total_reward = 0.0
                for _, instr in self.instr_dict.iteritems():
                    # Possible distances from instr to candidate steps
                    temp_dis = np.zeros(len(temp_neighbor_node)).astype(float)
                    for j, temp_n in enumerate(temp_neighbor_node):
                        # print '****************************************************'
                        # print 'instr.destination = ', instr.destination
                        # print 'n = ', n
                        # print 'shortest path = ', self.shortest_path[instr.destination, n]
                        # print 'temp_neighbor[n] = ', temp_neighbor[n]
                        # print '****************************************************'
                        temp_dis[j] = self.shortest_path[instr.destination, n] + temp_neighbor[n]

                    total_reward += instr.r * pow(instr.b, np.min(temp_dis))
                    # print 'max reward of task {0}: {1}'.format(instr.id, instr.r * pow(instr.b, np.min(temp_dis)))

                print 'total reward in candidate node {0}: {1}'.format(n, total_reward)

                candidate_steps[n] = {'reward': total_reward, 'neighbor': temp_neighbor}
                # print 'candidate_steps[n] = ', candidate_steps[n]

            # Pick the node with maximum accumulated reward
            self.next_node = max(candidate_steps, key=lambda x: candidate_steps[x]['reward'])
            # print 'next_node: {0}, next_neighbor: {1}'.format(self.next_node, candidate_steps[self.next_node])
            rospy.loginfo('plan_task result: {0}'.format(self.next_node))

        return

    def plan_motion_viz(self):
        if self.cur_node == self.next_node:
            rospy.loginfo('Motion: Reach node {0}.'.format(self.next_node))
            if self.cur_node in self.instr_dest_dict.keys():

                # This is for Value Iteration!
                for idx in self.instr_dest_dict[self.cur_node]:
                    do_instr = self.instr_dict[idx]
                    rospy.loginfo('Do instr {0}: {1}'.format(idx, do_instr.function))
                    rospy.sleep(do_instr.duration)
                    del self.instr_dict[idx]
                    self.show_instr()

                del self.instr_dest_dict[self.cur_node]

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
    rospy.init_node('task_motion_planner_dp', anonymous=True, log_level=rospy.INFO)
    tamp_dp = TaskMotionPlannerDP()
    rospy.loginfo('Start task planner dp!')
    tamp_dp.run_plan_viz()
