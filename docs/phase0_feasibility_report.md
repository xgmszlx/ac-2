# Phase 0 — Adaptive Realized-Graph Feasibility

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
- Trial scale: 1000 paired trials per configuration
- Runtime: 150.19 seconds

## 1. Research Question

Does feedback from realized loop outcomes materially improve final pose-graph reliability over static deterministic and probability-aware planning under the same motion budget?

## 2. Model

Each candidate loop succeeds independently as `X_e ~ Bernoulli(q_e)`. **`q_e` is a known oracle probability in this phase**; this experiment does not validate a probability estimator. Candidate outcomes are pre-generated and shared by all planners edge-by-edge within each paired trial.

## 3. Graph Utility

The sole optimization objective is normalized weighted spanning-tree connectivity, `Phi(G) = log(tau(G)) / (|V|-1)`, computed from a reduced weighted Laplacian. Effective resistance is used only to compute and interpret exact single-edge marginal gains. The low-quality threshold was frozen before execution as base utility plus `25%` of the optimistic deterministic recovery span for the same topology and budget.

## 4. Compared Planners

### Deterministic Static

Plans a fixed route with all selected loops treated as certain, and never revises it after failure.

### Probability-Aware Static

Plans once using `q_e * DeltaPhi / cost`; selected future edges enter an **expected-weight surrogate** with weight `q_e w_e`. This is not claimed to be exact expected D-optimal planning.

### Adaptive Realized-Graph

Uses the same `q_e`, candidates, budget, objective, and cost. After each observed outcome it greedily recomputes remaining marginal values on the realized graph. It has no access to future outcomes.

## 5. Experimental Design

- Topologies: T1 corridor/elongated, T2 rooms/grid, T3 bounded irregular exploration.
- Reliability: R1 uniform `q in {0.3, 0.5, 0.7, 0.9}`; R2 clipped `Beta(2.5, 2.5)`; R3 initial-value rank mapped inversely from `q=0.20` to `q=0.85`.
- Budgets: [12.0, 24.0, 36.0]; service cost: 2.0.
- 1000 paired Monte-Carlo trials for each of 54 configurations; no configuration was removed.
- Seeds and every per-edge latent uniform draw are saved.

## 6. Sanity Checks

Generator-level sanity status: **PASS** (15/15). Checks cover q=1 static/adaptive equality, q=0 no false utility, positive-edge monotonicity, budget compliance, and shared paired latents. Unit tests are separate from this table.

## 7. Main Results

Across all configurations and paired trials, `AdaptiveRealized - StaticExpected` had mean **0.010344**, median **0.000000**, and bootstrap 95% CI **[0.009981, 0.010686]** in normalized log-tree utility. Trial wins/ties/losses were **20375/23471/10154**.

| q_nominal | planner | mean_phi | p05_phi | low_risk | mean_cost |
| --- | --- | --- | --- | --- | --- |
| 0.3000 | AdaptiveRealized | 0.1383 | 0.0275 | 0.2350 | 22.4381 |
| 0.3000 | DeterministicStatic | 0.1188 | 0.0151 | 0.3096 | 22.2830 |
| 0.3000 | StaticExpected | 0.1281 | 0.0170 | 0.2603 | 22.8377 |
| 0.5000 | AdaptiveRealized | 0.2035 | 0.0993 | 0.0568 | 22.4075 |
| 0.5000 | DeterministicStatic | 0.1843 | 0.0671 | 0.0813 | 22.2830 |
| 0.5000 | StaticExpected | 0.1937 | 0.0779 | 0.0628 | 22.5759 |
| 0.7000 | AdaptiveRealized | 0.2561 | 0.1705 | 0.0076 | 22.3922 |
| 0.7000 | DeterministicStatic | 0.2421 | 0.1506 | 0.0136 | 22.2830 |
| 0.7000 | StaticExpected | 0.2418 | 0.1486 | 0.0133 | 22.1910 |
| 0.9000 | AdaptiveRealized | 0.3022 | 0.2508 | 0.0002 | 22.3256 |
| 0.9000 | DeterministicStatic | 0.2969 | 0.2300 | 0.0004 | 22.2830 |
| 0.9000 | StaticExpected | 0.2969 | 0.2300 | 0.0004 | 22.2830 |

See Figures 1 and 6 for mean behavior and its budget dependence.

## 8. Lower-Tail Reliability

The preregistered practical criterion requires both 5th- and 10th-percentile gains to reach at least `5%` of the optimistic recovery span, without worse failure risk or more than 1% mean-quality loss. **13 of 54** topology/reliability/budget configurations met it, spanning **3** topology families and **6** reliability settings.

Best lower-tail cases:

| topology | reliability | budget | p05_gain_span_fraction | p10_gain_span_fraction | failure_probability_reduction |
| --- | --- | --- | --- | --- | --- |
| T3_irregular | R1_q0.3 | 24.0000 | 0.2287 | 0.0572 | 0.1000 |
| T1_corridor | R1_q0.9 | 12.0000 | 0.2099 | 0.2099 | 0.0010 |
| T3_irregular | R1_q0.5 | 24.0000 | 0.1777 | 0.0613 | 0.0170 |
| T2_rooms | R1_q0.9 | 12.0000 | 0.1774 | 0.1774 | 0.0000 |
| T2_rooms | R1_q0.5 | 36.0000 | 0.1605 | 0.0631 | 0.0340 |
| T2_rooms | R3_conflict | 36.0000 | 0.1602 | 0.0802 | 0.0040 |

## 9. Predicted-vs-Realized Gap

StaticExpected mean prediction gap was **0.051564** (95th percentile **0.162637**). This is a diagnostic mismatch measure; the decision is based on realized final utility, not on making this gap small. Adaptive has no single fixed pre-execution final graph, so no directly comparable static prediction gap is assigned.

## 10. Cost Analysis

Adaptive mean travel cost minus StaticExpected was **-0.0022**. Every episode respected the same hard budget. A positive cost difference is reported explicitly rather than treated as free improvement; Figures 4 and 6 should be read together.

## 11. Where Adaptivity Helps

R3 high-value/low-reliability conflict had mean span-normalized 5th-percentile gain **0.0229** and mean failure-probability reduction **0.0007**. The table in Section 8 identifies the strongest configurations without changing their generation rules after seeing results.

## 12. Where Adaptivity Does Not Help

The following are the weakest or clearest counterexamples; they are retained in full:

| topology | reliability | budget | p05_gain_span_fraction | p10_gain_span_fraction | mean_gain_span_fraction | cost_difference |
| --- | --- | --- | --- | --- | --- | --- |
| T3_irregular | R3_conflict | 24.0000 | -0.1931 | 0.0073 | 0.0007 | 0.0785 |
| T2_rooms | R1_q0.7 | 12.0000 | -0.0188 | 0.0000 | 0.0243 | -0.1900 |
| T3_irregular | R3_conflict | 36.0000 | -0.0156 | 0.0047 | 0.0320 | -0.3599 |
| T3_irregular | R3_conflict | 12.0000 | 0.0000 | 0.0000 | -0.0241 | -1.2132 |
| T3_irregular | R1_q0.5 | 12.0000 | 0.0000 | 0.0763 | 0.0162 | -0.6494 |
| T3_irregular | R1_q0.3 | 12.0000 | 0.0000 | 0.0000 | 0.0270 | -1.0066 |

At `q=0.9`, the mean span-normalized gain was **0.0182**. Equality or near-equality is expected when outcomes are almost certain; deviations must be interpreted jointly with route changes and finite-sample uncertainty.

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

# **WEAK_GO**

The rule was applied mechanically: GO requires practical lower-tail improvement in at least two topology families and two reliability settings, non-degraded mean/failure behavior, acceptable cost, and decreasing advantage near q=1; WEAK_GO requires at least one qualifying configuration; otherwise STOP.

Gate diagnostics: mean-gain decrease near `q=1` = **True**; per-configuration cost guard = **False** (2 configurations exceeded the allowed 5%-of-budget mean cost increase). Therefore the pooled near-zero cost difference cannot erase local cost confounding.

## 15. Exact Recommended Next Step

**Do one more abstract targeted study.**
