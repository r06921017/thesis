#!/usr/bin/env python
# -*- coding: utf8 -*-
"""
1. Create shortest path among nodes with Floyd-Warshall algorithm.
2. Visualization of graph.
"""

import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import rospkg

if __name__ == '__main__':
    pkg_dir = rospkg.RosPack().get_path('thesis')

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
    map_graph.add_edge(4, 8, weight=2)
    map_graph.add_edge(5, 7, weight=2)
    map_graph.add_edge(6, 8, weight=4)
    map_graph.add_edge(7, 8, weight=8)

    # Run Floyd Warshall algorithm for shortest path.
    # short_path = nx.floyd_warshall_numpy(map_graph)
    # np.save(file=pkg_dir+'/config/shortest_path.npy', arr=short_path, allow_pickle=True)
    # print '=== shortest path matrix ===\n', short_path
    # adjacency_matrix = nx.convert_matrix.to_numpy_array(map_graph)
    # print '=== adjacency matrix ===\n', adjacency_matrix
    # np.save(file=pkg_dir+'/config/adjacency_matrix.npy', arr=adjacency_matrix, allow_pickle=True)

    # Create main graph for AI center
    node_graph = nx.Graph()
    node_num = map_graph.number_of_nodes()
    node_graph.add_nodes_from([str(i) for i in range(0, node_num)], value=0.0, robot=False, task=True)
    node_graph.nodes['2']['robot'] = True

    pos_list = [map_graph.nodes[n]['pos'] for n in map_graph.nodes]

    for n in node_graph.nodes:
        node_graph.nodes[n]['pos'] = pos_list[int(n)]

    # Convert edge weight to node number
    for e in map_graph.edges.items():
        node1 = str(e[0][0])
        node2 = str(e[0][1])
        weight = e[1]['weight']

        previous_node = node1

        start_node = map_graph.nodes[e[0][0]]['pos']
        end_node = map_graph.nodes[e[0][1]]['pos']

        u_vec = tuple(np.subtract(end_node, start_node) / float(weight))

        for i in range(1, weight):
            new_pos = tuple(np.add(start_node, tuple(np.multiply(u_vec, i))))
            temp_node = node1 + node2 + '_' + str(i)
            node_graph.add_node(temp_node, pos=new_pos, value=0.0, robot=False, task=False)
            node_graph.add_edge(previous_node, temp_node)
            previous_node = temp_node

        node_graph.add_edge(previous_node, node2)

    task_node = [n for n in node_graph.nodes if node_graph.nodes[n]['task']]  # nodes for adding tasks
    task_graph = node_graph.subgraph(task_node)

    color_list = ['#6699ff' if node_graph.nodes[n]['task'] else '#6699ff' for n in node_graph.nodes]
    r_color_list = ['yellow' if node_graph.nodes[n]['robot'] else color_list[i] for i, n in enumerate(node_graph.nodes)]
    size_list = [1400 if node_graph.nodes[n]['task'] else 600 for n in node_graph.nodes]
    label_list = [True if node_graph.nodes[n]['task'] else False for n in node_graph.nodes]

    # draw origin node
    plt.figure(num=None, figsize=(12, 8), dpi=80, facecolor='w', edgecolor='k')
    pos_node_graph = nx.get_node_attributes(node_graph, 'pos')
    nx.draw(node_graph, pos_node_graph, with_labels=False, node_size=size_list, node_color=color_list)

    node_label = {}
    for n in task_graph.nodes:
        node_label[n] = loc_symbol_val[int(n)]

    nx.draw_networkx_labels(node_graph, pos_node_graph, labels=node_label)
    plt.savefig(pkg_dir + "/config/node_graph.png")
    plt.clf()

    # Draw weighted graph G
    plt.figure(num=None, figsize=(12, 8), dpi=80, facecolor='w', edgecolor='k')
    pos = nx.get_node_attributes(map_graph, 'pos')
    weight_label = nx.get_edge_attributes(map_graph, 'weight')
    nx.draw(map_graph, pos, with_labels=False, node_size=1400, node_color='#6699ff')
    nx.draw_networkx_edge_labels(map_graph, pos, edge_labels=weight_label, font_size=20)
    nx.draw_networkx_labels(map_graph, pos_node_graph, labels=node_label)

    plt.savefig(pkg_dir + '/config/weighted_graph.png')
    plt.clf()

    # draw value graph
    nx.draw(node_graph, pos_node_graph, with_labels=False, node_size=size_list, node_color=color_list)
    # value_labels = nx.get_node_attributes(node_graph, 'value')
    # nx.draw_networkx_labels(node_graph, pos_node_graph, labels=value_labels)
    plt.savefig(pkg_dir + '/config/value_graph.png')
    plt.clf()
