#!/usr/bin/env python
# -*- coding: utf8 -*-
"""
Solve task planning with dynamic programming (value iteration)
"""

from human_id import *
from task_planner_fcfs import TaskPlannerFCFS
import numpy as np


class TaskPlannerDP(TaskPlannerFCFS):
    def __init__(self):
        TaskPlannerFCFS.__init__(self)
        self.adjacency_matrix = np.load(self._pkg_dir + '/config/adjacency_matrix.npy')
        self.shortest_path = np.load(self._pkg_dir + '/config/shortest_path.npy')
        self.cur_location = rospy.get_param('/thesis/pepper_location', 2)  # initial at charge
        self.cur_neighbor = self.adjacency_matrix[self.cur_location]  # neighbor of nodes

    def move_adjacency_node(self, dest_neighbor, sim=True, render=False):
        """
        Get the neighbor of next node after moving.
        :param dest_neighbor: int of neighbor node
        :param sim: is it simulation or real move
        :param render: Whether to show on networkx.
        :return: next_neighbor
        """
        print 'cur_neighbor = ', self.cur_neighbor
        neighbor_node = np.where(self.cur_neighbor > 0)

        if dest_neighbor not in neighbor_node:
            rospy.logerr('Invalid destination for planning.')
            exit(1)

        temp_neighbor = np.copy(self.cur_neighbor)

        for n in neighbor_node:
            if n == dest_neighbor:  # moving toward desire neighbor node
                temp_neighbor[n] -= 1

                if temp_neighbor[n] == 0:  # Reach the neighbor node
                    if not sim:
                        self.cur_location = n
                    rospy.set_param('/thesis/pepper_location', n)

                    temp_neighbor = np.copy(self.adjacency_matrix[n])
                    break

            else:
                temp_neighbor[n] += 1
                if temp_neighbor[n] > np.max(self.adjacency_matrix[n]):  # leave the neighbor node
                    temp_neighbor[n] = 0

        if not sim:  # if real move, not simulation
            self.cur_neighbor = np.copy(temp_neighbor)

            if render:
                if self.cur_neighbor == self.adjacency_matrix[self.cur_location]:  # is on neighbor node
                    robot_node = str(self.cur_location)

                else:
                    edge = map(str, sorted([self.cur_location, dest_neighbor]))
                    robot_node = ''
                    for e in edge:
                        robot_node += e
                print robot_node

        rospy.loginfo('Next neighbor: ', temp_neighbor)
        return temp_neighbor

    def plan_task(self, instr_list):
        self.show_task(instr_list)  # print unplanned tasks.

        # compare the accumulated reward of neighbor, move to the maximum one.
        neighbor_node = np.where(self.cur_neighbor > 0)
        candidate_steps = dict()  # {neighbor_node: {'reward': int, 'neighbor': np.array}}

        for n in neighbor_node:
            temp_neighbor = self.move_adjacency_node(n)
            temp_neighbor_node = np.where(temp_neighbor > 0)

            # calculate accumulated reward of a candidate step from all instructions
            total_reward = 0.0
            for instr in instr_list:

                temp_dis = np.zeros(len(temp_neighbor_node))  # possible distances from instr to candidate steps
                for j, temp_n in enumerate(temp_neighbor_node):
                    temp_dis[j] = self.shortest_path[instr.dest][n] + temp_neighbor[n]

                total_reward += instr.r * pow(instr.b, np.min(temp_dis))
                print 'max reward of task {0}: {1}'.format(instr.id, instr.r * pow(instr.b, np.min(temp_dis)))

            print 'total reward in candidate step: ', total_reward

            candidate_steps[n] = {'reward': total_reward, 'neighbor': temp_neighbor}

        # pick the node with maximum accumulated reward
        next_node = max(candidate_steps, key=lambda x: candidate_steps['reward'])
        next_neighbor = candidate_steps[next_node]
        print 'next_node: {0}, next_neighbor: {1}'.format(next_node, next_neighbor)
        self.move_adjacency_node(next_node, sim=False)

        return


if __name__ == '__main__':
    rospy.init_node('task_planner_dp')
    task_planner = TaskPlannerDP()
    rospy.spin()
