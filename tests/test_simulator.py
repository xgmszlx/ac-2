from dataclasses import replace

from src.graph_metrics import log_spanning_tree
from src.scenario_generator import generate_scenario
from src.simulator import PLANNERS, run_episode


def scenario():
    return generate_scenario(
        "T1_corridor", node_count=48, graph_seed=1701, odometry_weight=1.0,
        loop_weight=2.0, candidate_count=18, minimum_temporal_separation=8
    )


def test_q0_has_no_false_realized_utility_growth():
    graph, candidates = scenario()
    candidates = [replace(candidate, success_probability=0.0) for candidate in candidates]
    outcomes = {candidate.edge_id: 0.0 for candidate in candidates}
    base = log_spanning_tree(graph)
    for planner in PLANNERS:
        result = run_episode(planner, graph, candidates, outcomes, 36.0, 2.0)
        assert abs(result.final_phi - base) < 1e-12


def test_every_planner_respects_budget():
    graph, candidates = scenario()
    candidates = [replace(candidate, success_probability=0.5) for candidate in candidates]
    outcomes = {candidate.edge_id: 0.4 for candidate in candidates}
    for planner in PLANNERS:
        result = run_episode(planner, graph, candidates, outcomes, 12.0, 2.0)
        assert result.travel_cost <= 12.0 + 1e-10


def test_shared_latent_is_used_for_same_edge_across_planners():
    graph, candidates = scenario()
    candidates = [replace(candidate, success_probability=0.5) for candidate in candidates]
    outcomes = {candidate.edge_id: (index + 0.25) / len(candidates)
                for index, candidate in enumerate(candidates)}
    observed = {}
    for planner in PLANNERS:
        result = run_episode(planner, graph, candidates, outcomes, 36.0, 2.0)
        for action in result.actions:
            observed.setdefault(action.edge_id, []).append(action.latent_u)
    assert all(len(set(values)) == 1 for values in observed.values())


def test_missing_outcome_latent_is_rejected():
    graph, candidates = scenario()
    outcomes = {candidate.edge_id: 0.5 for candidate in candidates[:-1]}
    try:
        run_episode("AdaptiveRealized", graph, candidates, outcomes, 12.0, 2.0)
    except ValueError as error:
        assert "cover the candidate set" in str(error)
    else:
        raise AssertionError("Missing paired latent should fail")
