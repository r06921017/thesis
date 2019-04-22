#!/usr/bin/env python
# -*- coding: UTF-8 -*-

"""
Value Iteration for task planner
"""

from human_id import *
import networkx as nx


def task_cb(instr):

    return


if __name__ == '__main__':
    # rospy.init_node('task_planner', log_level=rospy.INFO)
    # rospy.Subscriber('/thesis/instruction_buffer', task_cb, queue_size=1)

    loc_symbol = {0: 'office', 1: 'bedroom', 2: 'charge', 3: 'alley', 4: 'livingroom',
                  5: 'diningroom', 6: 'greet', 7: 'emergency'}

    print loc_symbol.values()

    loc_graph = nx.Graph(loc_symbol.values())
    A = nx.nx_agraph.to_agraph(loc_graph)
    A.draw('/home/rdaneel/loc_graph.png', prog='dot')
