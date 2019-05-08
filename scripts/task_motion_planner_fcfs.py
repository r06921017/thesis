#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
Solve task planning with first come first serve
"""

from human_id import *
from thesis.msg import *
import rospkg
from std_msgs.msg import String
from node_viz import create_map_graph
import numpy as np
import networkx as nx


class TaskMotionPlannerFCFS:
    def __init__(self):
        self.sub = rospy.Subscriber('/thesis/instruction_buffer', InstructionArray, self.plan_task, queue_size=1)
        self.task_pub = rospy.Publisher('/thesis/instruction_buffer', InstructionArray, queue_size=1)

        # for TAMP
        self.map_graph = create_map_graph()
        self.adjacency_matrix = nx.convert_matrix.to_numpy_array(self.map_graph)
        self.cur_node = rospy.get_param('/thesis/pepper_location', 2)  # initial at charge, type=int
        self.next_node = rospy.get_param('/thesis/pepper_location', 2)  # initial at charge, type=int
        self.time_step = 1
        self.instr_dict = dict()
        self.instr_dest_dict = {n: set() for n in self.map_graph.nodes}

        # for visualization, including nodes and edges
        self.viz_node_pub = rospy.Publisher('/thesis/robot_node', String, queue_size=2)
        self._pkg_dir = rospkg.RosPack().get_path('thesis')
        self.cur_neighbor = self.adjacency_matrix[self.cur_node].astype(int).tolist()

    def show_instr(self):
        _loc_symbol = {0: 'office',
                       1: 'bedroom',
                       2: 'charge',
                       3: 'alley1',
                       4: 'alley2',
                       5: 'livingroom',
                       6: 'diningroom',
                       7: 'greet',
                       8: 'emergency'}

        print '--------------------------------------------------------------------------'
        for key, instr in self.instr_dict.iteritems():
            print 'instr {0}: dest={1}, function={2}, duration={3}, r={4}'.format(instr.id,
                                                                                  _loc_symbol[instr.destination],
                                                                                  instr.function,
                                                                                  instr.duration,
                                                                                  instr.r)
        print '--------------------------------------------------------------------------'
        return

    def get_pkg_dir(self):
        print 'pkg path', self._pkg_dir
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
            dest_node = self.instr_dict[min(self.instr_dict.keys())].destination  # destination node
            rospy.loginfo('destination node: {0}'.format(dest_node))
            temp_path = nx.shortest_path(self.map_graph, self.cur_node, dest_node, weight='weight')
            print 'temp_path = ', temp_path
            if len(temp_path) > 1:
                self.next_node = temp_path[1]  # next neighbor node for motion planner
            else:
                self.next_node = self.cur_node
        return

    def move_adjacency_node(self, dest_neighbor_node, sim=True, render=False):
        """
        Get the neighbor of next node after moving.
        :param dest_neighbor_node: int of neighbor node
        :param sim: is it simulation or real move
        :param render: Whether to show on networkx.
        :return: next_neighbor, type=list()
        """
        print 'start moving, cur_neighbor = ', self.cur_neighbor
        _neighbor_nodes = np.where(np.array(self.cur_neighbor) > 0)[0].tolist()

        if dest_neighbor_node == self.cur_node:
            rospy.loginfo('dest_node is the same as current node.')
            return self.cur_neighbor

        elif dest_neighbor_node not in _neighbor_nodes:
            rospy.logerr('Invalid destination for planning.')
            exit(1)

        _temp_neighbor = list(self.cur_neighbor)  # copy the list, not changing self.cur_neighbor

        # update neighbor after moving
        for n in _neighbor_nodes:
            if n == dest_neighbor_node:  # moving toward desire neighbor node
                _temp_neighbor[n] -= 1

                if _temp_neighbor[n] == 0:  # robot reaches the desire neighbor node
                    _temp_neighbor = np.copy(self.adjacency_matrix[n]).astype(int).tolist()
                    break

                _temp_neighbor[self.cur_node] += 1

            elif n == self.cur_node:  # robot is currently on the edge.
                continue

            else:
                _temp_neighbor[n] = 0

        if not sim:  # if real move, not checking the candidate steps.
            self.cur_neighbor = list(_temp_neighbor)
            print 'self.cur_neighbor:  ', self.cur_neighbor

            if render:
                # robot reach destination neighbor node
                if self.cur_neighbor == self.adjacency_matrix[dest_neighbor_node].astype(int).tolist():
                    self.cur_node = dest_neighbor_node
                    _robot_node = str(self.cur_node)

                # robot is on edge
                else:
                    _nodes_on_edge = sorted([self.cur_node, dest_neighbor_node])
                    _temp_step = self.cur_neighbor[_nodes_on_edge[0]]
                    _nodes_on_edge = map(str, _nodes_on_edge)
                    _robot_node = ''
                    for e in _nodes_on_edge:
                        _robot_node += e
                    _robot_node += '_'
                    _robot_node += str(_temp_step)
                print 'robot_node = ', _robot_node

                # Publish the robot node in str type
                _viz_node = String()
                _viz_node.data = str(_robot_node)
                self.viz_node_pub.publish(_viz_node)

        print 'neighbor array after moving: ', _temp_neighbor
        return _temp_neighbor

    def plan_motion_viz(self):
        if self.cur_node == self.next_node:
            rospy.loginfo('Motion: Reach node {0}.'.format(self.next_node))
            # if self.cur_node in self.instr_dest_dict.keys():
            if len(self.instr_dest_dict[self.cur_node]) > 0:
                # This is for FCFS!!!
                for idx in self.instr_dest_dict[self.cur_node]:
                    if idx == min(self.instr_dict.keys()):
                        do_instr = self.instr_dict[idx]
                        rospy.loginfo('Do instr {0}: {1}'.format(idx, do_instr.function))
                        rospy.sleep(do_instr.duration)

                        del self.instr_dict[do_instr.id]
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

    def run_plan_viz(self):
        rospy.loginfo('Start TAMP!')

        # Publish the initial position node of the robot to visualization
        rospy.sleep(0.5)
        self.viz_node_pub.publish(String(data=str(self.cur_node)))

        # Start running motion planning and visualization
        rate = rospy.Rate(1.0 / self.time_step)
        while not rospy.is_shutdown():
            self.plan_motion_viz()
            rate.sleep()

        return


if __name__ == '__main__':
    rospy.init_node(os.path.basename(__file__).split('.')[0], log_level=rospy.INFO)
    tamp = TaskMotionPlannerFCFS()
    tamp.run_plan_viz()
