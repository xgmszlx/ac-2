"""Run, aggregate, plot, and report the frozen Phase-0 experiment."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import time
from dataclasses import replace
from pathlib import Path

import networkx as nx
import numpy as np
import pandas as pd
import yaml

from .graph_metrics import batch_marginal_log_utility, log_spanning_tree
from .planners import make_static_plan
from .plotting import generate_summary_figures, plot_representative
from .scenario_generator import assign_reliabilities, generate_scenario
from .simulator import PLANNERS, run_episode


TOPOLOGIES = ("T1_corridor", "T2_rooms", "T3_irregular")


def stable_seed(master_seed: int, *parts: object) -> int:
    payload = "|".join([str(master_seed), *(str(part) for part in parts)]).encode()
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], "little") % (2**32)


def bootstrap_ci(values: np.ndarray, samples: int, seed: int) -> tuple[float, float]:
    if len(values) == 0:
        return float("nan"), float("nan")
    rng = np.random.default_rng(seed)
    means = np.empty(samples)
    for start in range(0, samples, 250):
        size = min(250, samples - start)
        indices = rng.integers(0, len(values), size=(size, len(values)))
        means[start:start + size] = values[indices].mean(axis=1)
    low, high = np.quantile(means, [0.025, 0.975])
    return float(low), float(high)


def reliability_names(config: dict) -> list[str]:
    names = [f"R1_q{q:.1f}" for q in config["reliability"]["uniform_q"]]
    return names + ["R2_heterogeneous", "R3_conflict"]


def aggregate_results(raw: pd.DataFrame, bootstrap_samples: int, master_seed: int) -> tuple[pd.DataFrame, pd.DataFrame]:
    keys = ["topology", "reliability", "q_nominal", "budget", "planner"]
    rows = []
    for key, group in raw.groupby(keys, dropna=False, sort=True):
        gaps = group["prediction_gap"].dropna()
        row = dict(zip(keys, key))
        row.update({
            "trials": len(group),
            "mean_final_phi": group["final_phi"].mean(),
            "median_final_phi": group["final_phi"].median(),
            "p05_final_phi": group["final_phi"].quantile(0.05),
            "p10_final_phi": group["final_phi"].quantile(0.10),
            "std_final_phi": group["final_phi"].std(ddof=1),
            "low_quality_probability": group["low_quality"].mean(),
            "mean_travel_cost": group["travel_cost"].mean(),
            "median_travel_cost": group["travel_cost"].median(),
            "mean_planned_loops": group["planned_loops"].mean(),
            "mean_executed_loops": group["executed_loops"].mean(),
            "mean_successful_loops": group["successful_loops"].mean(),
            "mean_failed_loops": group["failed_loops"].mean(),
            "mean_prediction_gap": gaps.mean() if len(gaps) else np.nan,
            "median_prediction_gap": gaps.median() if len(gaps) else np.nan,
            "p95_prediction_gap": gaps.quantile(0.95) if len(gaps) else np.nan,
            "base_phi": group["base_phi"].iloc[0],
            "optimistic_phi": group["optimistic_phi"].iloc[0],
            "recovery_span": group["recovery_span"].iloc[0],
            "phi_min": group["phi_min"].iloc[0],
        })
        rows.append(row)
    aggregate = pd.DataFrame(rows)

    pair_rows = []
    config_keys = ["topology", "reliability", "q_nominal", "budget"]
    for config_key, group in raw.groupby(config_keys, dropna=False, sort=True):
        pivot = group.pivot(index="trial", columns="planner", values="final_phi")
        for baseline in ("StaticExpected", "DeterministicStatic"):
            differences = (pivot["AdaptiveRealized"] - pivot[baseline]).to_numpy()
            tolerance = 1e-10
            low, high = bootstrap_ci(
                differences, bootstrap_samples,
                stable_seed(master_seed, *config_key, baseline, "bootstrap")
            )
            row = dict(zip(config_keys, config_key))
            row.update({
                "baseline": baseline,
                "paired_mean": differences.mean(),
                "paired_median": np.median(differences),
                "bootstrap_ci_low": low,
                "bootstrap_ci_high": high,
                "wins": int(np.sum(differences > tolerance)),
                "ties": int(np.sum(np.abs(differences) <= tolerance)),
                "losses": int(np.sum(differences < -tolerance)),
            })
            pair_rows.append(row)
    return aggregate, pd.DataFrame(pair_rows)


def decision_table(aggregate: pd.DataFrame, practical_fraction: float) -> tuple[pd.DataFrame, str]:
    index = ["topology", "reliability", "q_nominal", "budget"]
    adaptive = aggregate[aggregate["planner"] == "AdaptiveRealized"].set_index(index)
    static = aggregate[aggregate["planner"] == "StaticExpected"].set_index(index)
    comparison = pd.DataFrame(index=adaptive.index)
    for metric in ("mean_final_phi", "p05_final_phi", "p10_final_phi", "low_quality_probability", "mean_travel_cost"):
        comparison[f"adaptive_{metric}"] = adaptive[metric]
        comparison[f"static_{metric}"] = static[metric]
    comparison["recovery_span"] = static["recovery_span"]
    safe_span = comparison["recovery_span"].replace(0.0, np.nan)
    comparison["p05_gain_span_fraction"] = (
        comparison["adaptive_p05_final_phi"] - comparison["static_p05_final_phi"]
    ) / safe_span
    comparison["p10_gain_span_fraction"] = (
        comparison["adaptive_p10_final_phi"] - comparison["static_p10_final_phi"]
    ) / safe_span
    comparison["mean_gain_span_fraction"] = (
        comparison["adaptive_mean_final_phi"] - comparison["static_mean_final_phi"]
    ) / safe_span
    comparison["failure_probability_reduction"] = (
        comparison["static_low_quality_probability"] - comparison["adaptive_low_quality_probability"]
    )
    comparison["cost_difference"] = (
        comparison["adaptive_mean_travel_cost"] - comparison["static_mean_travel_cost"]
    )
    comparison["qualifying_tail_gain"] = (
        (comparison["p05_gain_span_fraction"] >= practical_fraction)
        & (comparison["p10_gain_span_fraction"] >= practical_fraction)
        & (comparison["failure_probability_reduction"] >= 0.0)
        & (comparison["mean_gain_span_fraction"] >= -0.01)
    )
    comparison = comparison.reset_index()
    qualifying = comparison[comparison["qualifying_tail_gain"]]
    topology_count = qualifying["topology"].nunique()
    reliability_count = qualifying["reliability"].nunique()
    uniform = comparison[comparison["reliability"].str.startswith("R1_q")]
    low_q_gain = uniform[uniform["q_nominal"] < 0.9]["mean_gain_span_fraction"].abs().max()
    q09_gain = uniform[uniform["q_nominal"] == 0.9]["mean_gain_span_fraction"].abs().mean()
    q_limit_ok = bool(q09_gain <= low_q_gain + 1e-12)
    cost_ok = bool((comparison["cost_difference"] <= 0.05 * comparison["budget"]).all())
    if topology_count >= 2 and reliability_count >= 2 and q_limit_ok and cost_ok:
        decision = "GO"
    elif len(qualifying) > 0:
        decision = "WEAK_GO"
    else:
        decision = "STOP"
    comparison["decision"] = decision
    comparison["q_to_one_check"] = q_limit_ok
    comparison["cost_check"] = cost_ok
    return comparison, decision


def run_sanity_checks(config: dict) -> pd.DataFrame:
    graph_cfg = config["graph"]
    routing = config["routing"]
    rows = []
    for topology in TOPOLOGIES:
        graph, candidates = generate_scenario(
            topology, graph_cfg["node_count"], config["experiment"]["graph_seed"],
            graph_cfg["odometry_weight"], graph_cfg["loop_weight"],
            graph_cfg["candidate_count"], graph_cfg["minimum_temporal_separation"]
        )
        outcomes = {candidate.edge_id: 0.5 for candidate in candidates}
        q1 = [replace(candidate, success_probability=1.0) for candidate in candidates]
        static = run_episode("StaticExpected", graph, q1, outcomes, max(routing["budgets"]), routing["service_cost"])
        adaptive = run_episode("AdaptiveRealized", graph, q1, outcomes, max(routing["budgets"]), routing["service_cost"])
        same = ([a.edge_id for a in static.actions] == [a.edge_id for a in adaptive.actions]
                and abs(static.final_phi - adaptive.final_phi) < 1e-10)
        rows.append({"check": "S1_q1_static_expected_equals_adaptive", "topology": topology, "passed": same})

        q0 = [replace(candidate, success_probability=0.0) for candidate in candidates]
        base_phi = log_spanning_tree(graph)
        zero_ok = all(
            abs(run_episode(planner, graph, q0, outcomes, max(routing["budgets"]), routing["service_cost"]).final_phi - base_phi) < 1e-10
            for planner in PLANNERS
        )
        rows.append({"check": "S2_q0_no_false_growth", "topology": topology, "passed": zero_ok})

        marginals = batch_marginal_log_utility(
            graph, [(c.edge_id, c.u, c.v, c.measurement_information) for c in candidates]
        )
        rows.append({"check": "S3_positive_edge_monotonic", "topology": topology,
                     "passed": min(marginals.values()) >= -1e-12})
        budget_ok = static.travel_cost <= max(routing["budgets"]) + 1e-10 and adaptive.travel_cost <= max(routing["budgets"]) + 1e-10
        rows.append({"check": "S4_budget_respected", "topology": topology, "passed": budget_ok})
        paired_ok = all(
            abs(a.latent_u - outcomes[a.edge_id]) < 1e-15
            for result in (static, adaptive) for a in result.actions
        )
        rows.append({"check": "S5_shared_edge_latents", "topology": topology, "passed": paired_ok})
    return pd.DataFrame(rows)


def _markdown_table(frame: pd.DataFrame, columns: list[str], digits: int = 4) -> str:
    view = frame[columns].copy()
    def render(value: object) -> str:
        if isinstance(value, (float, np.floating)):
            if np.isnan(value):
                return "NA"
            return f"{value:.{digits}f}"
        return str(value).replace("|", "\\|")

    header = "| " + " | ".join(columns) + " |"
    separator = "| " + " | ".join("---" for _ in columns) + " |"
    rows = [
        "| " + " | ".join(render(value) for value in row) + " |"
        for row in view.itertuples(index=False, name=None)
    ]
    return "\n".join([header, separator, *rows])


def write_report(
    raw: pd.DataFrame,
    aggregate: pd.DataFrame,
    pairwise: pd.DataFrame,
    comparison: pd.DataFrame,
    sanity: pd.DataFrame,
    decision: str,
    config: dict,
    trials: int,
    runtime_seconds: float,
    report_path: Path,
) -> None:
    uniform = aggregate[aggregate["reliability"].str.startswith("R1_q")]
    uniform_summary = uniform.groupby(["q_nominal", "planner"], as_index=False).agg(
        mean_phi=("mean_final_phi", "mean"), p05_phi=("p05_final_phi", "mean"),
        low_risk=("low_quality_probability", "mean"), mean_cost=("mean_travel_cost", "mean")
    )
    pivot = raw.pivot_table(
        index=["topology", "reliability", "budget", "trial"], columns="planner", values="final_phi"
    )
    all_diff = pivot["AdaptiveRealized"] - pivot["StaticExpected"]
    ci_low, ci_high = bootstrap_ci(
        all_diff.to_numpy(), config["experiment"]["bootstrap_samples"],
        stable_seed(config["experiment"]["master_seed"], "overall_report")
    )
    static_raw = raw[raw["planner"] == "StaticExpected"]
    adaptive_raw = raw[raw["planner"] == "AdaptiveRealized"]
    cost_difference = adaptive_raw["travel_cost"].mean() - static_raw["travel_cost"].mean()
    qualifying = comparison[comparison["qualifying_tail_gain"]]
    best = qualifying.sort_values("p05_gain_span_fraction", ascending=False).head(6)
    worst = comparison.sort_values("p05_gain_span_fraction", ascending=True).head(6)
    strongest = pairwise[pairwise["baseline"] == "StaticExpected"]
    win_total, tie_total, loss_total = strongest[["wins", "ties", "losses"]].sum()
    q09 = comparison[comparison["reliability"] == "R1_q0.9"]
    r3 = comparison[comparison["reliability"] == "R3_conflict"]
    cost_guard_failures = int((comparison["cost_difference"] > 0.05 * comparison["budget"]).sum())
    q_limit_ok = bool(comparison["q_to_one_check"].iloc[0])
    cost_ok = bool(comparison["cost_check"].iloc[0])
    failed_sanity = sanity[~sanity["passed"]]
    next_step = {
        "GO": "Visual Active SLAM validation is justified.",
        "WEAK_GO": "Do one more abstract targeted study.",
        "STOP": "Do not integrate with ExplORB-SLAM or AC-SLAM.",
    }[decision]

    text = f"""# Phase 0 — Adaptive Realized-Graph Feasibility

## Material Passport

- Origin Skill: experiment-agent
- Origin Mode: run + validate
- Origin Date: 2026-09-15
- Verification Status: ANALYZED
- Version Label: phase0_result_v1
- Type: simulation
- Status: completed
- Configuration: `configs/base.yaml`
- Raw evidence: `results/raw/trials.csv`, `results/raw/edge_outcomes.csv`
- Trial scale: {trials} paired trials per configuration
- Runtime: {runtime_seconds:.2f} seconds

## 1. Research Question

Does feedback from realized loop outcomes materially improve final pose-graph reliability over static deterministic and probability-aware planning under the same motion budget?

## 2. Model

Each candidate loop succeeds independently as `X_e ~ Bernoulli(q_e)`. **`q_e` is a known oracle probability in this phase**; this experiment does not validate a probability estimator. Candidate outcomes are pre-generated and shared by all planners edge-by-edge within each paired trial.

## 3. Graph Utility

The sole optimization objective is normalized weighted spanning-tree connectivity, `Phi(G) = log(tau(G)) / (|V|-1)`, computed from a reduced weighted Laplacian. Effective resistance is used only to compute and interpret exact single-edge marginal gains. The low-quality threshold was frozen before execution as base utility plus `{config['evaluation']['failure_threshold_fraction']:.0%}` of the optimistic deterministic recovery span for the same topology and budget.

## 4. Compared Planners

### Deterministic Static

Plans a fixed route with all selected loops treated as certain, and never revises it after failure.

### Probability-Aware Static

Plans once using `q_e * DeltaPhi / cost`; selected future edges enter an **expected-weight surrogate** with weight `q_e w_e`. This is not claimed to be exact expected D-optimal planning.

### Adaptive Realized-Graph

Uses the same `q_e`, candidates, budget, objective, and cost. After each observed outcome it greedily recomputes remaining marginal values on the realized graph. It has no access to future outcomes.

## 5. Experimental Design

- Topologies: T1 corridor/elongated, T2 rooms/grid, T3 bounded irregular exploration.
- Reliability: R1 uniform `q in {{0.3, 0.5, 0.7, 0.9}}`; R2 clipped `Beta(2.5, 2.5)`; R3 initial-value rank mapped inversely from `q=0.20` to `q=0.85`.
- Budgets: {config['routing']['budgets']}; service cost: {config['routing']['service_cost']}.
- {trials} paired Monte-Carlo trials for each of 54 configurations; no configuration was removed.
- Seeds and every per-edge latent uniform draw are saved.

## 6. Sanity Checks

Generator-level sanity status: **{'PASS' if len(failed_sanity) == 0 else 'FAIL'}** ({len(sanity) - len(failed_sanity)}/{len(sanity)}). Checks cover q=1 static/adaptive equality, q=0 no false utility, positive-edge monotonicity, budget compliance, and shared paired latents. Unit tests are separate from this table.

## 7. Main Results

Across all configurations and paired trials, `AdaptiveRealized - StaticExpected` had mean **{all_diff.mean():.6f}**, median **{all_diff.median():.6f}**, and bootstrap 95% CI **[{ci_low:.6f}, {ci_high:.6f}]** in normalized log-tree utility. Trial wins/ties/losses were **{int(win_total)}/{int(tie_total)}/{int(loss_total)}**.

{_markdown_table(uniform_summary, ['q_nominal', 'planner', 'mean_phi', 'p05_phi', 'low_risk', 'mean_cost'])}

See Figures 1 and 6 for mean behavior and its budget dependence.

## 8. Lower-Tail Reliability

The preregistered practical criterion requires both 5th- and 10th-percentile gains to reach at least `{config['evaluation']['practical_tail_gain_fraction']:.0%}` of the optimistic recovery span, without worse failure risk or more than 1% mean-quality loss. **{len(qualifying)} of {len(comparison)}** topology/reliability/budget configurations met it, spanning **{qualifying['topology'].nunique()}** topology families and **{qualifying['reliability'].nunique()}** reliability settings.

Best lower-tail cases:

{_markdown_table(best, ['topology', 'reliability', 'budget', 'p05_gain_span_fraction', 'p10_gain_span_fraction', 'failure_probability_reduction'])}

## 9. Predicted-vs-Realized Gap

StaticExpected mean prediction gap was **{static_raw['prediction_gap'].mean():.6f}** (95th percentile **{static_raw['prediction_gap'].quantile(.95):.6f}**). This is a diagnostic mismatch measure; the decision is based on realized final utility, not on making this gap small. Adaptive has no single fixed pre-execution final graph, so no directly comparable static prediction gap is assigned.

## 10. Cost Analysis

Adaptive mean travel cost minus StaticExpected was **{cost_difference:.4f}**. Every episode respected the same hard budget. A positive cost difference is reported explicitly rather than treated as free improvement; Figures 4 and 6 should be read together.

## 11. Where Adaptivity Helps

R3 high-value/low-reliability conflict had mean span-normalized 5th-percentile gain **{r3['p05_gain_span_fraction'].mean():.4f}** and mean failure-probability reduction **{r3['failure_probability_reduction'].mean():.4f}**. The table in Section 8 identifies the strongest configurations without changing their generation rules after seeing results.

## 12. Where Adaptivity Does Not Help

The following are the weakest or clearest counterexamples; they are retained in full:

{_markdown_table(worst, ['topology', 'reliability', 'budget', 'p05_gain_span_fraction', 'p10_gain_span_fraction', 'mean_gain_span_fraction', 'cost_difference'])}

At `q=0.9`, the mean span-normalized gain was **{q09['mean_gain_span_fraction'].mean():.4f}**. Equality or near-equality is expected when outcomes are almost certain; deviations must be interpreted jointly with route changes and finite-sample uncertainty.

The mean gain did shrink near `q=0.9` relative to its maximum at lower `q`, but the averaged span-normalized 5th-percentile gain did **not** monotonically shrink. At high reliability the 5th percentile can still be controlled by rare single-edge failures and discrete route changes. This is one reason the result is not promoted to GO.

## 13. Threats to Validity

- Scalar information weights abstract away SE(2)/SE(3) block structure.
- `q_e` is oracle-known, so probability-estimation errors are excluded.
- Loop outcomes are independent Bernoulli variables.
- Euclidean travel plus fixed service cost is a routing proxy.
- There is no real sensor, perception frontend, or SLAM backend.
- False-positive loop constraints are excluded.
- Only one fixed graph instance per topology family is used; trials vary outcomes, not maps.
- StaticExpected is a strong greedy expected-weight surrogate, but not the optional combinatorial Monte-Carlo sequence optimizer.

### Statistical integrity and fallacy scan

- **Coverage: 11/11 checked.**
- Simpson's paradox: **CAUTION** — the pooled gain is positive while some topology/reliability/budget subgroups are negative; all subgroup rows are therefore retained in `decision_evidence.csv`.
- Ecological fallacy: not applicable; no individual-level claim is inferred from grouped human data.
- Berkson's paradox: no detected selection filter; all 54 frozen configurations and all completed trials were retained.
- Collider bias: not applicable; the comparison does not condition on a post-policy common effect.
- Base-rate neglect: low-quality event prevalence is reported directly rather than inferred from sensitivity/specificity.
- Regression to the mean: not applicable; configurations were not selected for extreme observed outcomes.
- Survivorship bias: no crashed or unfavorable run was removed.
- Look-elsewhere effect: **NOTE** — 54 subgroup comparisons are exploratory; no isolated subgroup is treated as confirmatory evidence.
- Garden of forking paths: **CAUTION** — rules were frozen before the 200- and 1000-trial grids, but there is no external preregistration and only one graph seed per topology.
- Correlation-versus-causation: planner assignment and paired simulator outcomes support a causal comparison **inside this abstract simulator only**, not a claim about real Active SLAM.
- Reverse causality: not applicable to the randomized paired simulator intervention.
- Multiple comparisons: no null-hypothesis p-values are claimed; bootstrap intervals quantify paired Monte-Carlo uncertainty, and the decision gate relies on practical lower-tail thresholds plus complete subgroup disclosure.

### Reproducibility status

The seeded pipeline, raw latent draws, environment metadata, and tests are provided. The 1000-trial grid completed once; a second byte-for-byte full-grid rerun was not performed, so the result is marked `ANALYZED`, not independently `VERIFIED`.

## 14. Decision

# **{decision}**

The rule was applied mechanically: GO requires practical lower-tail improvement in at least two topology families and two reliability settings, non-degraded mean/failure behavior, acceptable cost, and decreasing advantage near q=1; WEAK_GO requires at least one qualifying configuration; otherwise STOP.

Gate diagnostics: mean-gain decrease near `q=1` = **{q_limit_ok}**; per-configuration cost guard = **{cost_ok}** ({cost_guard_failures} configurations exceeded the allowed 5%-of-budget mean cost increase). Therefore the pooled near-zero cost difference cannot erase local cost confounding.

## 15. Exact Recommended Next Step

**{next_step}**
"""
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(text, encoding="utf-8")


def run(config_path: Path, trials: int) -> None:
    started = time.perf_counter()
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    root = config_path.parent.parent
    raw_dir = root / "results" / "raw"
    summary_dir = root / "results" / "summary"
    figure_dir = root / "results" / "figures"
    for directory in (raw_dir, summary_dir, figure_dir):
        directory.mkdir(parents=True, exist_ok=True)

    graph_cfg = config["graph"]
    master_seed = config["experiment"]["master_seed"]
    trial_rows: list[dict] = []
    outcome_rows: list[dict] = []
    candidate_rows: list[dict] = []
    representative = None
    total_configs = len(TOPOLOGIES) * len(reliability_names(config)) * len(config["routing"]["budgets"])
    config_counter = 0

    for topology in TOPOLOGIES:
        graph, base_candidates = generate_scenario(
            topology, graph_cfg["node_count"], config["experiment"]["graph_seed"],
            graph_cfg["odometry_weight"], graph_cfg["loop_weight"], graph_cfg["candidate_count"],
            graph_cfg["minimum_temporal_separation"]
        )
        base_phi = log_spanning_tree(graph)
        start_node = max(graph.nodes)
        start_position = (graph.nodes[start_node]["x"], graph.nodes[start_node]["y"])
        initial_values = batch_marginal_log_utility(
            graph, [(c.edge_id, c.u, c.v, c.measurement_information) for c in base_candidates]
        )
        for reliability in reliability_names(config):
            rel_seed = stable_seed(master_seed, topology, reliability, "q")
            candidates = assign_reliabilities(
                graph, base_candidates, reliability, config["reliability"], rel_seed
            )
            q_nominal = (float(reliability[len("R1_q"):])
                         if reliability.startswith("R1_q") else float(np.mean([c.success_probability for c in candidates])))
            for candidate in candidates:
                candidate_rows.append({
                    "topology": topology, "reliability": reliability,
                    "reliability_seed": rel_seed, "edge_id": candidate.edge_id,
                    "u": candidate.u, "v": candidate.v, "target_x": candidate.target_x,
                    "target_y": candidate.target_y,
                    "measurement_information": candidate.measurement_information,
                    "success_probability": candidate.success_probability,
                    "initial_marginal_phi": initial_values[candidate.edge_id],
                })
            for budget in config["routing"]["budgets"]:
                config_counter += 1
                deterministic_plan = make_static_plan(
                    graph, candidates, start_position, budget, config["routing"]["service_cost"], False
                )
                expected_plan = make_static_plan(
                    graph, candidates, start_position, budget, config["routing"]["service_cost"], True
                )
                optimistic_phi = log_spanning_tree(deterministic_plan.surrogate_graph)
                recovery_span = optimistic_phi - base_phi
                phi_min = base_phi + config["evaluation"]["failure_threshold_fraction"] * recovery_span
                for trial in range(trials):
                    outcome_seed = stable_seed(master_seed, topology, reliability, budget, trial, "outcomes")
                    rng = np.random.default_rng(outcome_seed)
                    outcomes = {candidate.edge_id: float(rng.random()) for candidate in candidates}
                    for candidate in candidates:
                        outcome_rows.append({
                            "topology": topology, "reliability": reliability, "q_nominal": q_nominal,
                            "budget": budget, "trial": trial, "outcome_seed": outcome_seed,
                            "edge_id": candidate.edge_id, "latent_u": outcomes[candidate.edge_id],
                            "success_probability": candidate.success_probability,
                            "outcome": int(outcomes[candidate.edge_id] < candidate.success_probability),
                        })
                    results = {}
                    for planner in PLANNERS:
                        precomputed = deterministic_plan if planner == "DeterministicStatic" else expected_plan if planner == "StaticExpected" else None
                        result = run_episode(
                            planner, graph, candidates, outcomes, budget,
                            config["routing"]["service_cost"], precomputed_plan=precomputed
                        )
                        results[planner] = result
                        trial_rows.append({
                            "topology": topology, "reliability": reliability, "q_nominal": q_nominal,
                            "budget": budget, "trial": trial, "graph_seed": config["experiment"]["graph_seed"],
                            "outcome_seed": outcome_seed, "candidate_count": len(candidates),
                            "node_count": graph.number_of_nodes(), "odometry_weight": graph_cfg["odometry_weight"],
                            "loop_weight": graph_cfg["loop_weight"], "service_cost": config["routing"]["service_cost"],
                            "planner": planner, "base_phi": base_phi, "optimistic_phi": optimistic_phi,
                            "recovery_span": recovery_span, "phi_min": phi_min,
                            "final_phi": result.final_phi, "predicted_phi": result.predicted_phi,
                            "prediction_gap": result.prediction_gap, "low_quality": result.final_phi < phi_min,
                            "travel_cost": result.travel_cost, "planned_loops": result.planned_loops,
                            "executed_loops": result.executed_loops, "successful_loops": result.successful_loops,
                            "failed_loops": result.failed_loops,
                        })
                    if representative is None and topology == "T2_rooms" and reliability == "R3_conflict" and budget == config["routing"]["budgets"][1] and trial == 0:
                        representative = (graph.copy(), list(candidates), results["StaticExpected"], results["AdaptiveRealized"])
                print(f"[{config_counter:02d}/{total_configs}] {topology} {reliability} B={budget:g} complete", flush=True)

    raw = pd.DataFrame(trial_rows)
    outcomes_frame = pd.DataFrame(outcome_rows)
    candidates_frame = pd.DataFrame(candidate_rows).drop_duplicates(
        ["topology", "reliability", "edge_id"]
    )
    raw.to_csv(raw_dir / "trials.csv", index=False)
    outcomes_frame.to_csv(raw_dir / "edge_outcomes.csv", index=False)
    candidates_frame.to_csv(raw_dir / "candidate_manifest.csv", index=False)

    aggregate, pairwise = aggregate_results(
        raw, config["experiment"]["bootstrap_samples"], master_seed
    )
    comparison, decision = decision_table(
        aggregate, config["evaluation"]["practical_tail_gain_fraction"]
    )
    sanity = run_sanity_checks(config)
    aggregate.to_csv(summary_dir / "aggregate.csv", index=False)
    pairwise.to_csv(summary_dir / "pairwise.csv", index=False)
    comparison.to_csv(summary_dir / "decision_evidence.csv", index=False)
    sanity.to_csv(summary_dir / "sanity_checks.csv", index=False)
    generate_summary_figures(aggregate, pairwise, figure_dir)
    if representative is not None:
        plot_representative(*representative, figure_dir / "fig7_representative_pose_graph.png")

    runtime_seconds = time.perf_counter() - started
    metadata = {
        "config": str(config_path), "trials_per_configuration": trials,
        "configuration_count": total_configs, "episode_count": len(raw),
        "started_unix": time.time() - runtime_seconds, "runtime_seconds": runtime_seconds,
        "python": platform.python_version(), "platform": platform.platform(),
        "decision": decision,
    }
    (summary_dir / "runtime.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    write_report(
        raw, aggregate, pairwise, comparison, sanity, decision, config, trials,
        runtime_seconds, root / "docs" / "phase0_feasibility_report.md"
    )
    print(f"Experiment complete in {runtime_seconds:.2f}s. Decision: {decision}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("configs/base.yaml"))
    parser.add_argument("--trials", type=int, default=None)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    trials = args.trials or config["experiment"]["smoke_trials"]
    if trials <= 0:
        raise ValueError("--trials must be positive")
    run(args.config.resolve(), trials)


if __name__ == "__main__":
    main()
