#!/usr/bin/env python
# -*- coding: utf8 -*-

import networkx as nx
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
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
    G.add_node(0, pos=(0, 9))
    G.add_node(1, pos=(12, 9))
    G.add_node(2, pos=(12, 7))
    G.add_node(3, pos=(8, 1))
    G.add_node(4, pos=(0, 1))
    G.add_node(5, pos=(12, 1))
    G.add_node(6, pos=(0, 0))
    G.add_node(7, pos=(8, 0))
    G.add_node(8, pos=(8, 7))

    G.add_edge(0, 1, weight=12)
    G.add_edge(0, 8, weight=9)
    G.add_edge(8, 3, weight=6)
    G.add_edge(8, 2, weight=4)
    G.add_edge(1, 2, weight=2)
    G.add_edge(3, 4, weight=7)
    G.add_edge(3, 5, weight=4)
    G.add_edge(3, 7, weight=1)
    G.add_edge(4, 6, weight=1)
    G.add_edge(6, 7, weight=8)

    G1 = nx.Graph()
    G1.add_node('0', pos=(0, 9))
    G1.add_node('1', pos=(12, 9))
    G1.add_node('2', pos=(12, 7))
    G1.add_node('3', pos=(8, 1))
    G1.add_node('4', pos=(0, 1))
    G1.add_node('5', pos=(12, 1))
    G1.add_node('6', pos=(0, 0))
    G1.add_node('7', pos=(8, 0))
    G1.add_node('8', pos=(8, 7))

    for e in G.edges.items():
        node1 = str(e[0][0])
        node2 = str(e[0][1])
        weight = e[1]['weight']

        print 'node = {0}, {1}'.format(node1, node2)
        print 'w = ', weight

        previous_node = node1

        start_node = G.node[e[0][0]]['pos']
        end_node = G.node[e[0][1]]['pos']

        u_vec = tuple(np.subtract(end_node, start_node) / float(weight))

        print 'unit vector = ', u_vec

        for i in range(1, weight):
            print 'add = ', np.add(start_node, tuple(np.multiply(u_vec, i)))
            new_pos = tuple(np.add(start_node, tuple(np.multiply(u_vec, i))))
            print 'new_pos = ', new_pos
            temp_node = node1 + node2 + '_' + str(i)
            G1.add_node(temp_node, pos=new_pos)
            G1.add_edge(previous_node, temp_node)
            previous_node = temp_node

        G1.add_edge(previous_node, node2)

    pos = nx.get_node_attributes(G1, 'pos')
    nx.draw(G1, pos, with_labels=False, node_size=100, node_color='brown')
    # labels = nx.get_edge_attributes(G1, 'weight')
    # nx.draw_networkx_edge_labels(G1, pos, edge_labels=labels)

    G2 = nx.Graph()
    G2.add_node('0', pos=(0, 9))
    G2.add_node('1', pos=(12, 9))
    G2.add_node('2', pos=(12, 7))
    G2.add_node('3', pos=(8, 1))
    G2.add_node('4', pos=(0, 1))
    G2.add_node('5', pos=(12, 1))
    G2.add_node('6', pos=(0, 0))
    G2.add_node('7', pos=(8, 0))

    pos = nx.get_node_attributes(G2, 'pos')
    nx.draw(G2, pos, with_labels=True, node_size=500, node_color='orange')

    plt.savefig("/home/rdaneel/temp_graph.png")
