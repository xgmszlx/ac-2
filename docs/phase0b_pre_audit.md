# Phase 0B — Pre-Experiment Implementation Audit

## Material Passport

- Origin Skill: experiment-agent
- Origin Mode: validate
- Origin Date: 2026-09-15
- Verification Status: ANALYZED
- Version Label: phase0b_pre_audit_v1
- Scope: Phase 0 code and frozen Phase 0 artifacts only

## Audit verdict

**No Phase 0 correctness bug was found that requires stopping and repairing the existing experiment.** The graph objective, realized-edge updates, budget checks, common-random-number pairing, and stated greedy policies agree with the Phase 0 specification.

The audit did find a major design limitation, not a hidden implementation bug: Phase 0 used exactly one graph per topology family. T1 and T2 do not use `graph_seed` at all; only T3 does. Consequently, Phase 0's 54,000 paired repetitions reduce outcome Monte-Carlo noise on three fixed graphs and are not 54,000 independent graph problems. The pooled bootstrap interval is not cross-graph generalization evidence.

Because the supplied Phase 0B instruction file ends mid-sentence at `For T1`, no Phase 0B performance experiment is authorized by this audit document alone.

## 1. Topology instances used by Phase 0

Phase 0 calls `generate_scenario` once per topology, outside the reliability and budget loops (`src/experiment.py:422-451`). The same returned graph and candidate set are reused for all six reliability settings, three budgets, and 1,000 outcome trials.

| Family | Phase 0 graph | Uses `graph_seed`? | Independent graph instances in Phase 0 |
| --- | --- | --- | ---: |
| T1 corridor | Fixed 4 x 12 serpentine coordinates, spacing `(1.0, 1.4)` | No | 1 |
| T2 rooms/grid | Fixed 6 x 8 serpentine coordinates, spacing `(2.0, 2.0)` | No | 1 |
| T3 irregular | Bounded persistent random walk with 48 nodes | Yes, seed 1701 | 1 |

The relevant generator branches are `src/scenario_generator.py:131-159`. A direct audit with seeds 1701, 1702, and 1703 produced one unique graph/candidate signature for T1, one for T2, and three for T3.

## 2. What `graph_seed` actually controls

`graph_seed` is passed to all three branches, but is consumed only by `_irregular_walk` in the T3 branch (`src/scenario_generator.py:38-53,149-151`). It controls T3's initial heading, heading perturbations, step lengths, reflected coordinates, spatial candidate pool, and therefore the selected candidate endpoints and targets.

It does not control:

- T1 or T2 coordinates or candidates;
- R1 probabilities;
- R2 draws, whose seed is derived from master seed, topology, and reliability name;
- R3 mapping endpoints (`q=0.20` and `q=0.85`);
- outcome latents, which have a separate seed.

## 3. Does the candidate set vary with graph seed?

- **T1: no.** Coordinates are deterministic and the candidate-selection sorting and endpoint-degree rule are deterministic.
- **T2: no.** The same applies.
- **T3: yes.** Candidate eligibility depends on graph-seed-controlled spatial distance, followed by deterministic diversity selection (`src/scenario_generator.py:70-128`). Seeds 1701, 1702, and 1703 produced three different 18-edge sets in the direct check.

This means simply looping over new `graph_seed` values with the current T1/T2 generator would falsely label duplicate graph instances as independent. Phase 0B must introduce seed-dependent T1-like and T2-like generators before using graph seed as the statistical unit.

## 4. Exact definition of `StaticExpected`

`StaticExpected` is a fixed, non-adaptive greedy sequence, not an exact stochastic optimum.

At each planning step it considers budget-feasible unattempted candidates and scores

```text
q_e * DeltaPhi(e | G_surrogate) / c(e | p)
```

where `DeltaPhi` is the exact normalized log spanning-tree marginal of adding the full-weight candidate to the current surrogate graph (`src/planners.py:26-54`). After selecting an edge, the surrogate graph receives an edge of weight `q_e * w_e`, the planned robot position moves to its target, and planning continues until no candidate fits (`src/planners.py:57-94`).

The entire sequence is computed before any outcome. Execution never changes it (`src/simulator.py:69-100`). This is correctly described as an **expected-weight surrogate**; it is not exact `E[Phi]` and is not the strong static baseline required by Phase 0B.

Tie breaking is deterministic by lexicographically larger `edge_id` after score comparison (`src/planners.py:53`).

## 5. Exact definition of `AdaptiveRealized`

At every action event, the policy scores each currently budget-feasible remaining edge as

```text
q_e * DeltaPhi(e | G_realized) / c(e | p_current)
```

using the same candidate probabilities, edge weights, cost function, and tie break as `StaticExpected` (`src/planners.py:97-111`). On success, the full information edge is inserted; on failure, the graph is unchanged. The attempted edge is removed either way, the robot moves to its target, and all remaining scores are recomputed (`src/simulator.py:101-134`).

This is a one-step adaptive greedy heuristic. It is neither `ExactAdaptiveOptimal` nor evidence that adaptation itself is harmful when the heuristic loses.

## 6. Travel cost and budget fairness

All planners use exactly

```text
Euclidean(current_position, candidate.target) + service_cost
```

from `src/planners.py:22-23`, start at the last trajectory node (`src/simulator.py:44-46`), move to the selected target after every attempt regardless of success, and check the same remaining budget. Static plans filter infeasible actions while planning and verify the budget again during execution; adaptive planning filters against the current remaining budget and verifies again before execution.

The frozen raw table contains no budget violation (`max(travel_cost - budget) = 0.0`). Different policies may spend different amounts because they select different routes; this is an outcome of the policy, not preferential cost accounting.

## 7. Paired outcomes and bootstrap resampling unit

For each `(topology, reliability, budget, trial)`, Phase 0 derives one `outcome_seed`, generates one latent uniform per candidate, and passes the same edge-keyed mapping to all planners (`src/experiment.py:462-480`). Thus common edges attempted by different planners see exactly the same latent outcome. Latents are not shared across budgets because budget is part of the seed; cross-budget pairing was not claimed.

There are two bootstrap levels in the implementation:

1. `results/summary/pairwise.csv`: within each fixed `(topology, reliability, budget)` configuration, the resampling unit is one of the 1,000 paired **outcome trials** (`src/experiment.py:82-104`).
2. The headline report CI pools all 54,000 paired outcome differences, then resamples individual pooled rows (`src/experiment.py:236-243`). It does not resample graphs and should not be interpreted as a cross-graph CI.

The correct Phase 0B generalization unit is an independently generated graph instance/seed, with outcome repetitions nested inside it.

## 8. R1, R2, and R3 audit

- R1 assigns the requested constant `q` to all 18 candidates.
- R2 draws one clipped `Beta(2.5, 2.5)` vector per topology using a fixed reliability seed and reuses it across budgets/trials.
- R3 ranks candidates by initial full-weight marginal graph utility and maps ranks linearly from `q=0.20` for the highest-value candidate to `q=0.85` for the lowest-value candidate (`src/scenario_generator.py:162-198`). Ties are resolved by edge ID.

These match the stated Phase 0 rules. For a future multi-graph study, the R2 seed should explicitly include graph seed if each graph instance is intended to have an independent reliability realization.

## 9. T3 + R3 + budget 24 exact configuration

| Field | Value |
| --- | --- |
| Master seed | 20260915 |
| Graph seed | 1701 |
| Reliability seed | 185541997 |
| Nodes / odometry edges / candidates | 48 / 47 / 18 |
| Start position | `(1.716724558463, -1.316029199640)` |
| Budget / service cost | 24.0 / 2.0 |
| Odometry / loop information weight | 1.0 / 2.0 |
| Base Phi | 0.0 |
| Optimistic deterministic Phi | 0.345451702185 |
| Frozen low-quality threshold | 0.086362925546 |

Candidate endpoints and probabilities:

| Edge | Endpoints | q |
| --- | --- | ---: |
| e00 | 1-46 | 0.200000000000 |
| e01 | 2-46 | 0.276470588235 |
| e02 | 2-47 | 0.238235294118 |
| e03 | 3-40 | 0.352941176471 |
| e04 | 3-47 | 0.314705882353 |
| e05 | 4-25 | 0.429411764706 |
| e06 | 4-26 | 0.391176470588 |
| e07 | 5-24 | 0.505882352941 |
| e08 | 5-25 | 0.467647058824 |
| e09 | 6-22 | 0.582352941176 |
| e10 | 6-23 | 0.544117647059 |
| e11 | 7-21 | 0.658823529412 |
| e12 | 7-22 | 0.620588235294 |
| e13 | 8-20 | 0.697058823529 |
| e14 | 9-19 | 0.773529411765 |
| e15 | 9-20 | 0.735294117647 |
| e16 | 10-19 | 0.811764705882 |
| e17 | 30-38 | 0.850000000000 |

The fixed `StaticExpected` sequence is `e04, e17, e08, e05, e11, e15`, with cost `22.726474028818` and expected-weight surrogate Phi `0.269146534432`.

Frozen 1,000-trial results:

| Planner | Mean Phi | P05 Phi | P10 Phi | Low-quality probability | Mean cost |
| --- | ---: | ---: | ---: | ---: | ---: |
| AdaptiveRealized | 0.208302 | 0.060281 | 0.133101 | 0.090 | 22.804998 |
| StaticExpected | 0.208057 | 0.126994 | 0.130571 | 0.033 | 22.726474 |
| DeterministicStatic | 0.192350 | 0.073064 | 0.132012 | 0.069 | 23.919865 |

Relative to `StaticExpected`, adaptive has `+0.000708` mean gain as a fraction of the optimistic recovery span, `-0.193117` P05 change, `+0.007323` P10 change, `+5.7` percentage points low-quality probability, and `+0.078524` mean cost. This is compatible with myopic greedy adaptation being harmful on this fixed instance; it does not establish that an optimal adaptive policy is harmful.

## 10. Frozen Phase 0 artifact hashes

These hashes define the publication snapshot. Phase 0 results and report match
the pre-publication audit. The configuration's terminal blank-only line was
normalized for `git diff --check`; no YAML value changed.

| Artifact | SHA-256 |
| --- | --- |
| `configs/base.yaml` | `87f69ddd1de5127de16d21f3fc4da2f1a35ac7a5ac6f8fc6a6fdfeb943d4c01c` |
| `docs/phase0_feasibility_report.md` | `c71b6febde9b061306b138dda58307fabb3746332a8166e21271651785a6c07e` |
| `results/raw/trials.csv` | `a347a8937ad344ee2a72a8b7733abffb0f9c0d1e6c943a48e73d358a23f70582` |
| `results/raw/edge_outcomes.csv` | `e460c3692eece3642302458dd03fcd13ebd93a79d78d51bdd9404e012fa7adb2` |
| `results/raw/candidate_manifest.csv` | `d834272fea95f4214332a3314be62b13eecb2b84dff802da9d3529c4a4351dfb` |
| `results/summary/aggregate.csv` | `7dfaf784374e5d97a4a3a8e9c5709f81e2a8e381c4795bc06178ba23972bc9eb` |
| `results/summary/pairwise.csv` | `d29e367a0aa68e52bbe01d75849fe249a12db6d27a27007d8011bfa477cd10de` |
| `results/summary/decision_evidence.csv` | `0d57e8941c279ae8b00888f3ac3c136c6ad82f5cb8973f00c690b9a90531470e` |
| `results/summary/runtime.json` | `94cfb7b81b708c02af03c401649b61117a7dbb8d3a91cc6aee949582d47ece26` |
| `results/summary/sanity_checks.csv` | `e097f09a3602a0e4bcc69f60a842510d2c968a8d799edd0600c029c6cc4eaef0` |

## 11. Stop/proceed gate

- Correctness-bug stop condition: **not triggered**.
- Cross-graph evidence available from Phase 0: **no**.
- Exact static/adaptive benchmark available from Phase 0: **no**.
- Strong stochastic non-adaptive baseline available from Phase 0: **no**.
- Phase 0B performance execution: **paused because the supplied instruction is truncated before the Part B design and final acceptance criteria**.
