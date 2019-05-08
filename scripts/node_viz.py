#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
1. Create shortest path among nodes with Floyd-Warshall algorithm.
2. Visualization of graph.
"""

import networkx as nx
import numpy as np
import rospkg
import rospy

from std_msgs.msg import String
from thesis.msg import InstructionArray

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import matplotlib.pyplot as plt


robot_pre_node = str(rospy.get_param('/thesis/pepper_location', 2))  # global variable


def get_graph_config(in_node_graph):
    """
    Get color, size, labels of nodes for rendering.
    :param in_node_graph: graph with weighted edge being nodes
    :return: c_map, size_list, pos_node_graph, node_label
    """
    loc_symbol = {0: 'office',
                  1: 'bedroom',
                  2: 'charge',
                  3: 'alley1',
                  4: 'alley2',
                  5: 'livingroom',
                  6: 'diningroom',
                  7: 'greet',
                  8: 'emergency'}

    loc_symbol_val = loc_symbol.values()

    task_node = [n for n in in_node_graph.nodes if in_node_graph.nodes[n]['is_node']]  # nodes for adding tasks
    task_graph = in_node_graph.subgraph(task_node)

    _c = ['orange' if in_node_graph.nodes[n]['task'] else '#6699ff' for n in in_node_graph.nodes]
    c_map = ['yellow' if in_node_graph.nodes[n]['robot'] else _c[i] for i, n in enumerate(in_node_graph.nodes)]
    size_list = [800 if in_node_graph.nodes[n]['is_node'] else 200 for n in in_node_graph.nodes]
    pos_node_graph = nx.get_node_attributes(in_node_graph, 'pos')

    node_label = {}
    for n in task_graph.nodes:
        node_label[n] = loc_symbol_val[int(n)]

    return c_map, size_list, pos_node_graph, node_label


def robot_node_cb(data, args):
    global robot_pre_node

    if len(args) != 4:
        rospy.logerr('Invalid number of arguments, must be 4.')
        exit(1)

    (in_node_graph, gtype, save, in_mem_instr) = args
    try:
        instr_list = rospy.wait_for_message('/thesis/instruction_buffer', InstructionArray, timeout=1)
        in_mem_instr = instr_list

    except rospy.ROSException:
        instr_list = in_mem_instr

    if len(instr_list.data) > 0:
        instr_node_set = set()
        for instr in instr_list.data:
            instr_node_set.add(str(instr.destination))

    else:
        instr_node_set = None

    render_node(data.data, instr_node_set, in_node_graph, gtype, save)

    return


def render_node(r_node, i_node_set, in_node_graph, gtype='node', save=False):
    """
    Render weight-node conversion graph
    :param r_node: type=str, show which node robot is on
    :param i_node_set: type=str, nodes where current task exists
    :param in_node_graph: graph with weighted edge being nodes
    :param gtype: Graph type. 'node' or 'value'
    :param save: Whether to save the rendering graph
    :return:
    """

    global robot_pre_node

    # Reset the task node
    for node in in_node_graph.nodes:
        in_node_graph.nodes[node]['task'] = False

    # Fetch tasks
    if i_node_set is not None:
        for i_node in i_node_set:
            if type(i_node) == str:
                print 'render_node, task exists on node: ', i_node
                in_node_graph.nodes[i_node]['task'] = True
            else:
                rospy.logerr('Invalid task node type, must be str.')

    # Update robot position
    in_node_graph.nodes[robot_pre_node]['robot'] = False
    in_node_graph.nodes[r_node]['robot'] = True
    robot_pre_node = r_node

    c_map, size_list, pos_node_graph, node_label = get_graph_config(in_node_graph)

    nx.draw(in_node_graph, pos_node_graph, with_labels=False, node_size=size_list, node_color=c_map)

    if gtype == 'node':
        nx.draw_networkx_labels(in_node_graph, pos_node_graph, labels=node_label)

    elif gtype == 'value':
        value_labels = nx.get_node_attributes(in_node_graph, 'value')
        nx.draw_networkx_labels(in_node_graph, pos_node_graph, labels=value_labels)

    else:
        rospy.logerr("Invalid gtype. Only 'node' and 'value' are acceptable.")
        exit(1)

    if save:
        if gtype == 'node':
            fname = ['node_graph.png', 'node_graph.pkl']
        elif gtype == 'value':
            fname = ['value_graph.png', 'value_graph.pkl']
        else:
            fname = None
            rospy.logerr("Invalid gtype. Only 'node' and 'value' are accepted.")
            exit(1)

        plt.savefig(config_dir + fname[0])
        nx.write_gpickle(in_node_graph, config_dir + fname[1])

    temp_fig = plt.gcf()
    temp_fig.canvas.draw()
    # print 'temp_fig = ', temp_fig, type(temp_fig)

    # Get the RGBA buffer from the figure
    w, h = temp_fig.canvas.get_width_height()
    buf = np.fromstring(temp_fig.canvas.tostring_argb(), dtype=np.uint8)
    buf.shape = (h, w, 4)

    # canvas.tostring_argb give pixmap in ARGB mode. Roll the ALPHA channel to have it in RGBA mode
    buf = np.roll(buf, 3, axis=2)

    viz_pub.publish(cv_bridge.cv2_to_imgmsg(buf, 'rgba8'))
    temp_fig.clf()

    return


def create_map_graph(save_npy=False):
    map_graph = nx.Graph()
    map_graph.add_node(0, pos=(0, 10))
    map_graph.add_node(1, pos=(12, 10))
    map_graph.add_node(2, pos=(12, 8))
    map_graph.add_node(3, pos=(8, 8))
    map_graph.add_node(4, pos=(8, 2))
    map_graph.add_node(5, pos=(0, 2))
    map_graph.add_node(6, pos=(12, 2))
    map_graph.add_node(7, pos=(0, 0))
    map_graph.add_node(8, pos=(8, 0))

    map_graph.add_edge(0, 1, weight=12)
    map_graph.add_edge(0, 3, weight=9)
    map_graph.add_edge(1, 2, weight=2)
    map_graph.add_edge(1, 3, weight=5)
    map_graph.add_edge(2, 3, weight=4)
    map_graph.add_edge(3, 4, weight=6)
    map_graph.add_edge(4, 5, weight=8)
    map_graph.add_edge(4, 6, weight=4)
    # map_graph.add_edge(4, 7, weight=8)
    map_graph.add_edge(4, 8, weight=2)
    map_graph.add_edge(5, 7, weight=2)
    # map_graph.add_edge(5, 8, weight=8)
    map_graph.add_edge(6, 8, weight=4)
    map_graph.add_edge(7, 8, weight=8)

    if save_npy:
        # Run Floyd Warshall algorithm for shortest path.
        short_path = nx.floyd_warshall_numpy(map_graph)
        np.save(file=config_dir + 'shortest_path.npy', arr=short_path, allow_pickle=True)
        print '=== shortest path matrix ===\n', short_path
        adjacency_matrix = nx.convert_matrix.to_numpy_array(map_graph)
        print '=== adjacency matrix ===\n', adjacency_matrix
        np.save(file=config_dir + 'adjacency_matrix.npy', arr=adjacency_matrix, allow_pickle=True)

        # Draw weighted graph map_graph
        # pos = nx.get_node_attributes(map_graph, 'pos')
        # weight_label = nx.get_edge_attributes(map_graph, 'weight')
        # nx.draw(map_graph, pos, with_labels=True, node_size=300, node_color='orange')
        # nx.draw_networkx_edge_labels(map_graph, pos, edge_labels=weight_label)
        # plt.savefig(config_dir + 'weighted_graph.png')
        # plt.clf()

    return map_graph


def create_node_graph():

    map_graph = create_map_graph()

    # Create main graph for AI center
    node_graph = nx.Graph()
    node_num = map_graph.number_of_nodes()
    node_graph.add_nodes_from([str(idx) for idx in range(node_num)], value=0.0, robot=False, is_node=True, task=False)

    _pos_list = [map_graph.nodes[node]['pos'] for node in map_graph.nodes]

    for node in node_graph.nodes:
        node_graph.nodes[node]['pos'] = _pos_list[int(node)]

    # Convert edge weight to node number
    for e in map_graph.edges.items():
        node1 = str(e[0][0])
        node2 = str(e[0][1])
        weight = e[1]['weight']

        previous_node = node1

        start_node = map_graph.nodes[e[0][0]]['pos']
        end_node = map_graph.nodes[e[0][1]]['pos']

        u_vec = tuple(np.subtract(end_node, start_node) / float(weight))

        for idx in range(1, weight):
            new_pos = tuple(np.add(start_node, tuple(np.multiply(u_vec, idx))))
            temp_node = node1 + node2 + '_' + str(idx)
            node_graph.add_node(temp_node, pos=new_pos, value=0.0, robot=False, is_node=False, task=False)
            node_graph.add_edge(previous_node, temp_node)
            previous_node = temp_node

        node_graph.add_edge(previous_node, node2)

    return node_graph


if __name__ == '__main__':
    # Storing directory
    config_dir = rospkg.RosPack().get_path('thesis') + '/config/'

    # fix plotting on the same position
    # plt.get_current_fig_manager().window.wm_geometry("+600+400")

    cur_node_graph = create_node_graph()
    mem_instr = InstructionArray()
    rospy.init_node('node_viz', anonymous=True, log_level=rospy.INFO)

    rospy.Subscriber(name='/thesis/robot_node',
                     data_class=String,
                     callback=robot_node_cb,
                     callback_args=(cur_node_graph, 'node', False, mem_instr),
                     queue_size=10)

    cv_bridge = CvBridge()
    viz_pub = rospy.Publisher('/thesis/node_viz', Image, queue_size=5)

    rospy.loginfo('Start node_viz!')
    rospy.spin()
