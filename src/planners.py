"""Static and event-driven adaptive greedy planners."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Optional

import networkx as nx

from .graph_metrics import add_information_edge, batch_marginal_log_utility
from .scenario_generator import CandidateLoop


@dataclass
class Plan:
    sequence: list[CandidateLoop]
    surrogate_graph: nx.Graph
    total_cost: float


def action_cost(position: tuple[float, float], candidate: CandidateLoop, service_cost: float) -> float:
    return hypot(position[0] - candidate.target_x, position[1] - candidate.target_y) + service_cost


def _best_candidate(
    graph: nx.Graph,
    candidates: list[CandidateLoop],
    position: tuple[float, float],
    remaining_budget: float,
    service_cost: float,
    probability_aware: bool,
) -> Optional[CandidateLoop]:
    feasible = [
        candidate
        for candidate in candidates
        if action_cost(position, candidate, service_cost) <= remaining_budget + 1e-12
    ]
    if not feasible:
        return None
    marginals = batch_marginal_log_utility(
        graph,
        [
            (c.edge_id, c.u, c.v, c.measurement_information)
            for c in feasible
        ],
    )
    scored = []
    for candidate in feasible:
        q = candidate.success_probability if probability_aware else 1.0
        score = q * marginals[candidate.edge_id] / action_cost(position, candidate, service_cost)
        scored.append((score, candidate.edge_id, candidate))
    score, _, best = max(scored, key=lambda item: (item[0], item[1]))
    return best if score > 0.0 else None


def make_static_plan(
    initial_graph: nx.Graph,
    candidates: list[CandidateLoop],
    start_position: tuple[float, float],
    budget: float,
    service_cost: float,
    probability_aware: bool,
) -> Plan:
    """Create a fixed sequence using either full or expected-weight graph updates."""
    surrogate = initial_graph.copy()
    remaining = list(candidates)
    position = start_position
    spent = 0.0
    sequence: list[CandidateLoop] = []
    while remaining:
        candidate = _best_candidate(
            surrogate,
            remaining,
            position,
            budget - spent,
            service_cost,
            probability_aware,
        )
        if candidate is None:
            break
        cost = action_cost(position, candidate, service_cost)
        sequence.append(candidate)
        spent += cost
        position = candidate.target_location
        planning_weight = candidate.measurement_information
        if probability_aware:
            planning_weight *= candidate.success_probability
        if planning_weight > 0:
            add_information_edge(
                surrogate, candidate.u, candidate.v, planning_weight, "loop_surrogate"
            )
        remaining.remove(candidate)
    return Plan(sequence=sequence, surrogate_graph=surrogate, total_cost=spent)


def choose_adaptive_action(
    realized_graph: nx.Graph,
    remaining_candidates: list[CandidateLoop],
    position: tuple[float, float],
    remaining_budget: float,
    service_cost: float,
) -> Optional[CandidateLoop]:
    return _best_candidate(
        realized_graph,
        remaining_candidates,
        position,
        remaining_budget,
        service_cost,
        probability_aware=True,
    )
