"""Weighted spanning-tree graph utility and diagnostic resistance metrics."""

from __future__ import annotations

from typing import Iterable, Mapping

import networkx as nx
import numpy as np


def weighted_laplacian(graph: nx.Graph) -> tuple[np.ndarray, list[int]]:
    """Return the weighted Laplacian in a stable node order."""
    nodes = sorted(graph.nodes)
    adjacency = nx.to_numpy_array(
        graph, nodelist=nodes, weight="information_weight", dtype=float
    )
    return np.diag(adjacency.sum(axis=1)) - adjacency, nodes


def log_spanning_tree(graph: nx.Graph, normalized: bool = True) -> float:
    """Compute log weighted spanning-tree count via a reduced Laplacian."""
    if graph.number_of_nodes() < 2:
        return 0.0
    if not nx.is_connected(graph):
        return float("-inf")
    laplacian, _ = weighted_laplacian(graph)
    sign, logdet = np.linalg.slogdet(laplacian[:-1, :-1])
    if sign <= 0:
        raise ValueError("Reduced Laplacian is not positive definite")
    if normalized:
        logdet /= graph.number_of_nodes() - 1
    return float(logdet)


def resistance_context(graph: nx.Graph) -> tuple[np.ndarray, Mapping[int, int], int]:
    """Factor-independent context for repeated effective-resistance queries."""
    if not nx.is_connected(graph):
        raise ValueError("Effective resistance requires a connected graph")
    laplacian, nodes = weighted_laplacian(graph)
    root = nodes[-1]
    inverse = np.linalg.inv(laplacian[:-1, :-1])
    index = {node: idx for idx, node in enumerate(nodes[:-1])}
    return inverse, index, root


def effective_resistance_from_context(
    context: tuple[np.ndarray, Mapping[int, int], int], u: int, v: int
) -> float:
    inverse, index, root = context
    b = np.zeros(inverse.shape[0], dtype=float)
    if u != root:
        b[index[u]] += 1.0
    if v != root:
        b[index[v]] -= 1.0
    return max(0.0, float(b @ inverse @ b))


def effective_resistance(graph: nx.Graph, u: int, v: int) -> float:
    return effective_resistance_from_context(resistance_context(graph), u, v)


def marginal_log_utility(
    graph: nx.Graph, u: int, v: int, weight: float, normalized: bool = True
) -> float:
    """Exact marginal log-tree utility, using the rank-one determinant lemma."""
    if weight <= 0:
        raise ValueError("Edge weight must be positive")
    resistance = effective_resistance(graph, u, v)
    delta = np.log1p(weight * resistance)
    if normalized:
        delta /= graph.number_of_nodes() - 1
    return float(delta)


def batch_marginal_log_utility(
    graph: nx.Graph, edges: Iterable[tuple[str, int, int, float]], normalized: bool = True
) -> dict[str, float]:
    """Compute many exact marginal utilities with one reduced-Laplacian inverse."""
    context = resistance_context(graph)
    divisor = graph.number_of_nodes() - 1 if normalized else 1
    output: dict[str, float] = {}
    for edge_id, u, v, weight in edges:
        if weight <= 0:
            raise ValueError("Edge weight must be positive")
        resistance = effective_resistance_from_context(context, u, v)
        output[edge_id] = float(np.log1p(weight * resistance) / divisor)
    return output


def add_information_edge(
    graph: nx.Graph, u: int, v: int, weight: float, edge_type: str = "loop"
) -> None:
    """Add a non-parallel information edge to a graph."""
    if weight <= 0:
        raise ValueError("Edge weight must be positive")
    if graph.has_edge(u, v):
        graph[u][v]["information_weight"] += float(weight)
        if graph[u][v].get("edge_type") != edge_type:
            graph[u][v]["edge_type"] = "mixed"
    else:
        graph.add_edge(
            u, v, information_weight=float(weight), edge_type=edge_type
        )
