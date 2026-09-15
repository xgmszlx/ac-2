"""Deterministic synthetic trajectory and candidate-loop generation."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import hypot
from typing import Iterable

import networkx as nx
import numpy as np

from .graph_metrics import batch_marginal_log_utility


@dataclass(frozen=True)
class CandidateLoop:
    edge_id: str
    u: int
    v: int
    target_x: float
    target_y: float
    measurement_information: float
    success_probability: float = 1.0

    @property
    def target_location(self) -> tuple[float, float]:
        return self.target_x, self.target_y


def _serpentine(rows: int, cols: int, sx: float, sy: float) -> list[tuple[float, float]]:
    coordinates: list[tuple[float, float]] = []
    for row in range(rows):
        columns: Iterable[int] = range(cols) if row % 2 == 0 else range(cols - 1, -1, -1)
        coordinates.extend((column * sx, row * sy) for column in columns)
    return coordinates


def _irregular_walk(n: int, seed: int) -> list[tuple[float, float]]:
    rng = np.random.default_rng(seed)
    point = np.array([0.0, 0.0])
    heading = rng.uniform(-np.pi, np.pi)
    coordinates = [tuple(point)]
    for step in range(1, n):
        # A bounded persistent walk naturally revisits old regions without adding
        # any latent failure structure.
        heading += rng.normal(0.0, 0.72)
        point = point + rng.uniform(0.75, 1.25) * np.array([np.cos(heading), np.sin(heading)])
        for axis in (0, 1):
            if abs(point[axis]) > 5.5:
                point[axis] = np.sign(point[axis]) * (11.0 - abs(point[axis]))
                heading = np.pi - heading if axis == 0 else -heading
        coordinates.append(tuple(point))
    return coordinates


def _base_graph(coordinates: list[tuple[float, float]], odometry_weight: float) -> nx.Graph:
    graph = nx.Graph()
    for node_id, (x, y) in enumerate(coordinates):
        graph.add_node(node_id, id=node_id, x=float(x), y=float(y))
    for node_id in range(len(coordinates) - 1):
        graph.add_edge(
            node_id,
            node_id + 1,
            information_weight=float(odometry_weight),
            edge_type="odom",
        )
    return graph


def _candidate_pool(
    graph: nx.Graph,
    minimum_temporal_separation: int,
    maximum_spatial_distance: float,
) -> list[tuple[float, int, int]]:
    pool: list[tuple[float, int, int]] = []
    for u in graph.nodes:
        for v in range(u + minimum_temporal_separation, graph.number_of_nodes()):
            if graph.has_edge(u, v):
                continue
            distance = hypot(
                graph.nodes[u]["x"] - graph.nodes[v]["x"],
                graph.nodes[u]["y"] - graph.nodes[v]["y"],
            )
            if distance <= maximum_spatial_distance:
                score = (v - u) / (distance + 0.25)
                pool.append((score, u, v))
    return sorted(pool, key=lambda item: (-item[0], item[1], item[2]))


def _select_diverse_candidates(
    graph: nx.Graph,
    pool: list[tuple[float, int, int]],
    count: int,
    loop_weight: float,
    topology: str,
) -> list[CandidateLoop]:
    chosen: list[tuple[int, int]] = []
    endpoint_degree: dict[int, int] = {}
    # First pass limits endpoint reuse so loops span distinct weak regions.
    for _, u, v in pool:
        if endpoint_degree.get(u, 0) >= 2 or endpoint_degree.get(v, 0) >= 2:
            continue
        chosen.append((u, v))
        endpoint_degree[u] = endpoint_degree.get(u, 0) + 1
        endpoint_degree[v] = endpoint_degree.get(v, 0) + 1
        if len(chosen) == count:
            break
    if len(chosen) < count:
        for _, u, v in pool:
            if (u, v) not in chosen:
                chosen.append((u, v))
                if len(chosen) == count:
                    break
    if len(chosen) < count:
        raise ValueError(f"{topology} produced only {len(chosen)} candidates")
    output: list[CandidateLoop] = []
    for idx, (u, v) in enumerate(sorted(chosen)):
        output.append(
            CandidateLoop(
                edge_id=f"{topology}_e{idx:02d}",
                u=u,
                v=v,
                target_x=float(graph.nodes[v]["x"]),
                target_y=float(graph.nodes[v]["y"]),
                measurement_information=float(loop_weight),
            )
        )
    return output


def generate_scenario(
    topology: str,
    node_count: int,
    graph_seed: int,
    odometry_weight: float,
    loop_weight: float,
    candidate_count: int,
    minimum_temporal_separation: int,
) -> tuple[nx.Graph, list[CandidateLoop]]:
    """Generate T1, T2, or T3 and a fixed candidate set."""
    if node_count != 48:
        raise ValueError("The frozen Phase-0 generators currently require node_count=48")
    if topology == "T1_corridor":
        coordinates = _serpentine(rows=4, cols=12, sx=1.0, sy=1.4)
        spatial_threshold = 1.55
    elif topology == "T2_rooms":
        coordinates = _serpentine(rows=6, cols=8, sx=2.0, sy=2.0)
        spatial_threshold = 2.10
    elif topology == "T3_irregular":
        coordinates = _irregular_walk(node_count, graph_seed)
        spatial_threshold = 1.80
    else:
        raise ValueError(f"Unknown topology: {topology}")
    graph = _base_graph(coordinates, odometry_weight)
    pool = _candidate_pool(graph, minimum_temporal_separation, spatial_threshold)
    candidates = _select_diverse_candidates(
        graph, pool, candidate_count, loop_weight, topology
    )
    return graph, candidates


def assign_reliabilities(
    graph: nx.Graph,
    candidates: list[CandidateLoop],
    reliability_name: str,
    reliability_config: dict,
    seed: int,
) -> list[CandidateLoop]:
    """Apply a preregistered R1, R2, or R3 reliability rule."""
    if reliability_name.startswith("R1_q"):
        q = float(reliability_name[len("R1_q"):])
        probabilities = np.full(len(candidates), q)
    elif reliability_name == "R2_heterogeneous":
        rng = np.random.default_rng(seed)
        cfg = reliability_config["heterogeneous"]
        samples = rng.beta(cfg["alpha"], cfg["beta"], size=len(candidates))
        probabilities = np.clip(samples, cfg["minimum"], cfg["maximum"])
    elif reliability_name == "R3_conflict":
        cfg = reliability_config["conflict"]
        edges = [
            (c.edge_id, c.u, c.v, c.measurement_information) for c in candidates
        ]
        values = batch_marginal_log_utility(graph, edges)
        order = sorted(candidates, key=lambda c: (-values[c.edge_id], c.edge_id))
        probabilities_by_id: dict[str, float] = {}
        for rank, candidate in enumerate(order):
            fraction = rank / max(1, len(order) - 1)
            probabilities_by_id[candidate.edge_id] = (
                cfg["highest_value_q"]
                + fraction * (cfg["lowest_value_q"] - cfg["highest_value_q"])
            )
        probabilities = np.array([probabilities_by_id[c.edge_id] for c in candidates])
    else:
        raise ValueError(f"Unknown reliability scenario: {reliability_name}")
    return [
        replace(candidate, success_probability=float(probability))
        for candidate, probability in zip(candidates, probabilities)
    ]
