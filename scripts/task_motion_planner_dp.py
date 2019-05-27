#!/usr/bin/env python
# -*- coding: utf8 -*-
"""
Solve task planning with dynamic programming (value iteration)
"""
from task_motion_planner_fcfs import *
import operator


class TaskMotionPlannerDP(TaskMotionPlannerFCFS):
    def __init__(self):
        """
        self.cur_node: the node where robot starts to move, initial at charge (2), type: int
        """
        TaskMotionPlannerFCFS.__init__(self)
        self.shortest_path = nx.floyd_warshall_numpy(self.map_graph)
        print '----------------------------------------'
        print self.shortest_path
        print self.shortest_path.shape
        print '----------------------------------------'

    def plan_task(self, in_instructions):
        rospy.loginfo('Start task planning!')

        # Convert InstructionArray into dictionary
        if type(in_instructions) == thesis.msg._InstructionArray.InstructionArray:
            for instr in in_instructions.data:
                self.instr_dest_dict[instr.destination].add(instr.id)
                self.instr_dict[instr.id] = instr
                # rospy.sleep(0.001)
            print 'self.instr_dict: ', self.instr_dict
            self.last_id = max(self.instr_dict.keys()) + 1

        rospy.logdebug('len(self.instr_dict.keys()): {0}'.format(len(self.instr_dict.keys())))

        if len(self.instr_dict.keys()) > 0:  # if there exists instructions
            if len(self.instr_dest_dict[self.cur_node]) > 0:
                self.next_node = self.cur_node

            else:
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
                        # Possible distances from instr to candidate steps
                        temp_dis = np.zeros(len(temp_neighbor_node)).astype(float)
                        for j, temp_n in enumerate(temp_neighbor_node):
                            temp_dis[j] = self.shortest_path[instr.destination, temp_n] + temp_neighbor[temp_n]

                        total_reward += instr.r * pow(instr.b, np.min(temp_dis))
                        # print 'max reward of task {0}: {1}'.format(instr.id, instr.r * pow(instr.b, np.min(temp_dis)))

                    rospy.logdebug('total reward in candidate node {0}: {1}'.format(n, total_reward))

                    candidate_steps[n] = {'reward': total_reward, 'neighbor': temp_neighbor}

                # Pick the node with maximum accumulated reward
                self.next_node = max(candidate_steps, key=lambda x: candidate_steps[x]['reward'])

            rospy.loginfo('plan_task result: {0}'.format(self.next_node))

        return

    def plan_motion_viz(self):
        if self.cur_node == self.next_node:
            rospy.loginfo('Motion: Reach node {0}.'.format(self.next_node))

            if len(self.instr_dest_dict[self.cur_node]) > 0:
                # Create reward_dict = {'id (int)': 'reward (float)'}
                reward_dict = dict()
                for idx in self.instr_dest_dict[self.cur_node]:
                    _instr = self.instr_dict[idx]
                    reward_dict[_instr.id] = _instr.r

                # Sort the instructions with the max reward
                for r in sorted(reward_dict.items(), key=operator.itemgetter(1), reverse=True):
                    do_instr = self.instr_dict[r[0]]
                    rospy.loginfo('Do instr {0}: {1}'.format(do_instr.id, do_instr.function))
                    rospy.sleep(do_instr.duration)

                    if do_instr.function == 4:
                        temp_instr = Instruction(id=self.last_id,
                                                 r=do_instr.r,
                                                 b=do_instr.b,
                                                 type=do_instr.type,
                                                 duration=do_instr.duration,
                                                 source=do_instr.source,
                                                 status=do_instr.status,
                                                 function=9,
                                                 target=do_instr.source,
                                                 destination=self.human_dict['name'][do_instr.source].location)
                        self.instr_dict[temp_instr.id] = temp_instr
                        self.instr_dest_dict[temp_instr.destination].add(temp_instr.id)

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

        else:
            rospy.loginfo('Motion: from {0} to {1}'.format(self.cur_node, self.next_node))
            self.move_adjacency_node(self.next_node, sim=False, render=True)

        return


if __name__ == '__main__':
    rospy.init_node(os.path.basename(__file__).split('.')[0], log_level=rospy.INFO)
    tamp = TaskMotionPlannerDP()
    tamp.run_plan_viz()
