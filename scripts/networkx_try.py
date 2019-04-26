#!/usr/bin/env python
# -*- coding: utf8 -*-

import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import rospkg

if __name__ == '__main__':
    pkg_dir = rospkg.RosPack().get_path('thesis')

    # G = nx.Graph()
    # G.add_edges_from([(1, 2), (1, 3)])
    # A = nx.nx_agraph.to_agraph(G)
    # A.draw('/home/rdaneel/temp.png', prog='dot')
    #
    # G1 = nx.path_graph(3)
    # A1 = nx.nx_agraph.to_agraph(G1)
    # A.draw('/home/rdaneel/temp1.png', prog='dot')

    loc_symbol = {0: 'office', 1: 'bedroom', 2: 'charge', 3: 'alley', 4: 'livingroom',
                  5: 'diningroom', 6: 'greet', 7: 'emergency'}

    loc_symbol_val = loc_symbol.values()

    G = nx.Graph()
    G.add_node(0, pos=(0, 10))
    G.add_node(1, pos=(12, 10))
    G.add_node(2, pos=(12, 8))
    G.add_node(3, pos=(8, 8))
    G.add_node(4, pos=(8, 2))
    G.add_node(5, pos=(0, 2))
    G.add_node(6, pos=(12, 1))
    G.add_node(7, pos=(0, 0))
    G.add_node(8, pos=(8, 0))

    G.add_edge(0, 1, weight=12)
    G.add_edge(0, 3, weight=9)
    G.add_edge(1, 2, weight=2)
    G.add_edge(1, 3, weight=5)
    G.add_edge(2, 3, weight=4)
    G.add_edge(3, 4, weight=6)
    G.add_edge(4, 5, weight=8)
    G.add_edge(4, 6, weight=4)
    G.add_edge(4, 7, weight=8)
    G.add_edge(4, 8, weight=2)
    G.add_edge(5, 7, weight=2)
    G.add_edge(5, 8, weight=8)
    G.add_edge(6, 8, weight=4)
    G.add_edge(7, 8, weight=8)

    pos = nx.get_node_attributes(G, 'pos')
    weight_label = nx.get_edge_attributes(G, 'weight')
    nx.draw(G, pos, with_labels=True, node_size=300, node_color='orange')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=weight_label)
    plt.savefig(pkg_dir + '/config/temp_graph.png')

    pos_list = [G.nodes[n]['pos'] for n in G.nodes]
    # pos_list = [(0, 9), (12, 9), (12, 7), (8, 1), (0, 1), (12, 1), (0, 0), (8, 0), (8, 7)]

    # main graph for AI center
    G1 = nx.Graph()
    G1.add_nodes_from([str(i) for i in range(0, G.number_of_nodes())], value=0.0, robot=False, task=True)

    for n in G1.nodes:
        G1.nodes[n]['pos'] = pos_list[int(n)]

    # G1.add_node('0', pos=(0, 9), value=0.0, robot=False, task=True)
    # G1.add_node('1', pos=(12, 9), value=0.0, robot=False, task=True)
    # G1.add_node('2', pos=(12, 7), value=0.0, robot=True, task=True)
    # G1.add_node('3', pos=(8, 1), value=0.0, robot=False, task=True)
    # G1.add_node('4', pos=(0, 1), value=0.0, robot=False, task=True)
    # G1.add_node('5', pos=(12, 1), value=0.0, robot=False, task=True)
    # G1.add_node('6', pos=(0, 0), value=0.0, robot=False, task=True)
    # G1.add_node('7', pos=(8, 0), value=0.0, robot=False, task=True)
    # G1.add_node('8', pos=(8, 7), value=0.0, robot=False, task=True)

    for e in G.edges.items():
        node1 = str(e[0][0])
        node2 = str(e[0][1])
        weight = e[1]['weight']

        print 'node = {0}, {1}'.format(node1, node2)
        print 'w = ', weight

        previous_node = node1

        start_node = G.nodes[e[0][0]]['pos']
        end_node = G.nodes[e[0][1]]['pos']

        u_vec = tuple(np.subtract(end_node, start_node) / float(weight))

        print 'unit vector = ', u_vec

        for i in range(1, weight):
            # print 'add = ', np.add(start_node, tuple(np.multiply(u_vec, i)))
            new_pos = tuple(np.add(start_node, tuple(np.multiply(u_vec, i))))
            # print 'new_pos = ', new_pos
            temp_node = node1 + node2 + '_' + str(i)
            G1.add_node(temp_node, pos=new_pos, value=0.0, robot=False, task=False)
            G1.add_edge(previous_node, temp_node)
            previous_node = temp_node

        G1.add_edge(previous_node, node2)

    for n in G1.nodes:
        print 'n = ', n, ' ', G1.nodes[n]['task']

    task_node = [n for n in G1.nodes if G1.nodes[n]['task']]

    G2 = G1.subgraph(task_node)

    color_list = ['orange' if G1.nodes[n]['task'] else 'brown' for n in G1.nodes]
    size_list = [500 if G1.nodes[n]['task'] else 100 for n in G1.nodes]
    label_list = [True if G1.nodes[n]['task'] else False for n in G1.nodes]

    pos = nx.get_node_attributes(G1, 'pos')
    nx.draw(G1, pos, node_size=size_list, node_color=color_list)

    node_label = {}
    for n in G2.nodes:
        node_label[n] = n

    # draw origin node
    nx.draw_networkx_labels(G1, pos, labels=node_label)

    # draw value graph
    # value_labels = nx.get_node_attributes(G2, 'value')
    # nx.draw_networkx_labels(G2, pos, labels=value_labels)

    # G2 = nx.Graph()
    # G2.add_node('0', pos=(0, 9))
    # G2.add_node('1', pos=(12, 9))
    # G2.add_node('2', pos=(12, 7))
    # G2.add_node('3', pos=(8, 1))
    # G2.add_node('4', pos=(0, 1))
    # G2.add_node('5', pos=(12, 1))
    # G2.add_node('6', pos=(0, 0))
    # G2.add_node('7', pos=(8, 0))

    # pos = nx.get_node_attributes(G1, 'pos')
    # nx.draw(G1, pos, with_labels=True, node_size=500, node_color='orange')

    plt.savefig(pkg_dir + "/config/temp_graph3.png")

    # G1.neighbors()
