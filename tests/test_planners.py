from dataclasses import replace

from src.planners import make_static_plan
from src.scenario_generator import generate_scenario
from src.simulator import run_episode


def scenario():
    return generate_scenario(
        "T2_rooms", node_count=48, graph_seed=1701, odometry_weight=1.0,
        loop_weight=2.0, candidate_count=18, minimum_temporal_separation=8
    )


def test_static_plan_respects_budget():
    graph, candidates = scenario()
    last = max(graph.nodes)
    start = (graph.nodes[last]["x"], graph.nodes[last]["y"])
    plan = make_static_plan(graph, candidates, start, 24.0, 2.0, True)
    assert plan.total_cost <= 24.0 + 1e-10


def test_q1_static_expected_and_adaptive_are_identical():
    graph, candidates = scenario()
    candidates = [replace(candidate, success_probability=1.0) for candidate in candidates]
    outcomes = {candidate.edge_id: 0.75 for candidate in candidates}
    static = run_episode("StaticExpected", graph, candidates, outcomes, 36.0, 2.0)
    adaptive = run_episode("AdaptiveRealized", graph, candidates, outcomes, 36.0, 2.0)
    assert [action.edge_id for action in static.actions] == [action.edge_id for action in adaptive.actions]
    assert abs(static.final_phi - adaptive.final_phi) < 1e-12
    assert abs(static.travel_cost - adaptive.travel_cost) < 1e-12


def test_probability_aware_static_uses_expected_weight_surrogate():
    graph, candidates = scenario()
    candidates = [replace(candidate, success_probability=0.5) for candidate in candidates]
    last = max(graph.nodes)
    start = (graph.nodes[last]["x"], graph.nodes[last]["y"])
    plan = make_static_plan(graph, candidates, start, 24.0, 2.0, True)
    for candidate in plan.sequence:
        assert plan.surrogate_graph[candidate.u][candidate.v]["information_weight"] == 1.0
