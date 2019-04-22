#!/usr/bin/env python
# -*- coding: utf8 -*-

import networkx as nx
import matplotlib.pyplot as plt

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

    G.add_node(0, pos=(0, 5))
    G.add_node(1, pos=(2, 5))
    G.add_node(2, pos=(2, 4))
    G.add_node(3, pos=(1, 1))
    G.add_node(4, pos=(0, 1))
    G.add_node(5, pos=(2, 1))
    G.add_node(6, pos=(0, 0))
    G.add_node(7, pos=(1, 0))

    G.add_edge(0, 1, weight=12)
    G.add_edge(0, 2, weight=13)
    G.add_edge(0, 3, weight=14)
    G.add_edge(0, 4, weight=15)
    G.add_edge(1, 2, weight=2)
    G.add_edge(2, 3, weight=10)
    G.add_edge(3, 4, weight=7)
    G.add_edge(3, 5, weight=4)
    G.add_edge(3, 7, weight=1)
    G.add_edge(4, 6, weight=1)
    G.add_edge(6, 7, weight=8)

    pos = nx.get_node_attributes(G, 'pos')
    nx.draw(G, pos, with_labels=True, node_size=500, node_color='yellow')
    labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    plt.savefig("/home/rdaneel/temp_123.png")

    # print G.edges
    #
    # for e in G.edges.values():
    #     print e['weight']

    G1 = nx.Graph()

    G1.add_node(0, pos=(0, 5))
    G1.add_node(1, pos=(2, 5))
    G1.add_node(2, pos=(2, 4))
    G1.add_node(3, pos=(1, 1))
    G1.add_node(4, pos=(0, 1))
    G1.add_node(5, pos=(2, 1))
    G1.add_node(6, pos=(0, 0))
    G1.add_node(7, pos=(1, 0))

    for e in G.edges.items():
        node1 = e[0][0]
        node2 = e[0][1]
        weight = e[1]['weight']

        # print e
        # print 'node = {0}, {1}'.format(node1, node2)
        # print 'w = ', weight
