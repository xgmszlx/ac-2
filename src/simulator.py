"""Paired-outcome execution of all Phase-0 planners."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Optional

import networkx as nx

from .graph_metrics import add_information_edge, log_spanning_tree
from .planners import Plan, action_cost, choose_adaptive_action, make_static_plan
from .scenario_generator import CandidateLoop


PLANNERS = ("DeterministicStatic", "StaticExpected", "AdaptiveRealized")


@dataclass
class ActionRecord:
    step: int
    edge_id: str
    u: int
    v: int
    cost: float
    latent_u: float
    success: bool


@dataclass
class EpisodeResult:
    planner: str
    final_phi: float
    predicted_phi: Optional[float]
    prediction_gap: Optional[float]
    travel_cost: float
    planned_loops: int
    executed_loops: int
    successful_loops: int
    failed_loops: int
    realized_graph: nx.Graph
    actions: list[ActionRecord]


def _start_position(graph: nx.Graph) -> tuple[float, float]:
    node = max(graph.nodes)
    return float(graph.nodes[node]["x"]), float(graph.nodes[node]["y"])


def run_episode(
    planner: str,
    initial_graph: nx.Graph,
    candidates: list[CandidateLoop],
    outcomes: Mapping[str, float],
    budget: float,
    service_cost: float,
    precomputed_plan: Optional[Plan] = None,
) -> EpisodeResult:
    if planner not in PLANNERS:
        raise ValueError(f"Unknown planner: {planner}")
    if set(outcomes) != {candidate.edge_id for candidate in candidates}:
        raise ValueError("Outcome latent variables must cover the candidate set exactly")

    realized = initial_graph.copy()
    position = _start_position(initial_graph)
    spent = 0.0
    actions: list[ActionRecord] = []
    predicted_phi: Optional[float] = None

    if planner != "AdaptiveRealized":
        plan = precomputed_plan or make_static_plan(
            initial_graph, candidates, position, budget, service_cost,
            probability_aware=(planner == "StaticExpected"),
        )
        sequence = plan.sequence
        predicted_phi = log_spanning_tree(plan.surrogate_graph)
        for step, candidate in enumerate(sequence):
            cost = action_cost(position, candidate, service_cost)
            if spent + cost > budget + 1e-10:
                raise RuntimeError("Static plan exceeded budget")
            success = outcomes[candidate.edge_id] < candidate.success_probability
            if success:
                add_information_edge(
                    realized,
                    candidate.u,
                    candidate.v,
                    candidate.measurement_information,
                    "loop",
                )
            actions.append(
                ActionRecord(
                    step, candidate.edge_id, candidate.u, candidate.v, cost,
                    float(outcomes[candidate.edge_id]), bool(success)
                )
            )
            spent += cost
            position = candidate.target_location
    else:
        remaining = list(candidates)
        while remaining:
            candidate = choose_adaptive_action(
                realized, remaining, position, budget - spent, service_cost
            )
            if candidate is None:
                break
            cost = action_cost(position, candidate, service_cost)
            if spent + cost > budget + 1e-10:
                raise RuntimeError("Adaptive plan exceeded budget")
            success = outcomes[candidate.edge_id] < candidate.success_probability
            if success:
                add_information_edge(
                    realized,
                    candidate.u,
                    candidate.v,
                    candidate.measurement_information,
                    "loop",
                )
            actions.append(
                ActionRecord(
                    len(actions), candidate.edge_id, candidate.u, candidate.v, cost,
                    float(outcomes[candidate.edge_id]), bool(success)
                )
            )
            spent += cost
            position = candidate.target_location
            remaining.remove(candidate)

    final_phi = log_spanning_tree(realized)
    gap = None if predicted_phi is None else predicted_phi - final_phi
    successes = sum(action.success for action in actions)
    return EpisodeResult(
        planner=planner,
        final_phi=final_phi,
        predicted_phi=predicted_phi,
        prediction_gap=gap,
        travel_cost=spent,
        planned_loops=len(actions),
        executed_loops=len(actions),
        successful_loops=successes,
        failed_loops=len(actions) - successes,
        realized_graph=realized,
        actions=actions,
    )
