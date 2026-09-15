import networkx as nx
import numpy as np

from src.graph_metrics import (
    add_information_edge,
    effective_resistance,
    log_spanning_tree,
    marginal_log_utility,
)


def path_graph(n=8):
    graph = nx.Graph()
    graph.add_nodes_from(range(n))
    for node in range(n - 1):
        graph.add_edge(node, node + 1, information_weight=1.0, edge_type="odom")
    return graph


def test_weighted_path_has_one_spanning_tree_and_zero_log_utility():
    assert abs(log_spanning_tree(path_graph())) < 1e-12


def test_positive_edge_never_decreases_log_tree_utility():
    graph = path_graph()
    before = log_spanning_tree(graph)
    add_information_edge(graph, 0, 7, 2.0)
    assert log_spanning_tree(graph) >= before - 1e-12


def test_rank_one_marginal_matches_direct_slogdet():
    graph = path_graph()
    predicted = marginal_log_utility(graph, 1, 6, 1.7)
    before = log_spanning_tree(graph)
    add_information_edge(graph, 1, 6, 1.7)
    observed = log_spanning_tree(graph) - before
    assert np.isclose(predicted, observed, atol=1e-12)


def test_path_effective_resistance_equals_hop_distance():
    assert np.isclose(effective_resistance(path_graph(), 1, 6), 5.0)
