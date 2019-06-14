import networkx as nx
import matplotlib.pyplot as plt


def create_map_graph(save_npy=False):
    config_dir = '/home/rdaneel/'
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
        # short_path = nx.floyd_warshall_numpy(map_graph)
        # np.save(file=config_dir + 'shortest_path.npy', arr=short_path, allow_pickle=True)
        # print '=== shortest path matrix ===\n', short_path
        # adjacency_matrix = nx.convert_matrix.to_numpy_array(map_graph)
        # print '=== adjacency matrix ===\n', adjacency_matrix
        # np.save(file=config_dir + 'adjacency_matrix.npy', arr=adjacency_matrix, allow_pickle=True)

        # Draw weighted graph map_graph
        pos = nx.get_node_attributes(map_graph, 'pos')
        weight_label = nx.get_edge_attributes(map_graph, 'weight')
        nx.draw(map_graph, pos, with_labels=True, node_size=300, node_color='orange')
        nx.draw_networkx_edge_labels(map_graph, pos, edge_labels=weight_label)
        plt.savefig(config_dir + 'weighted_graph.png')
        plt.clf()

    return map_graph


if __name__ == '__main__':
    create_map_graph(save_npy=True)
