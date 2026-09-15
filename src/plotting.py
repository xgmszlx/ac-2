"""Single-question figures for the Phase-0 experiment."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import pandas as pd
from matplotlib.lines import Line2D

from .scenario_generator import CandidateLoop
from .simulator import EpisodeResult


COLORS = {
    "DeterministicStatic": "#777777",
    "StaticExpected": "#2878b5",
    "AdaptiveRealized": "#d95319",
}


def _uniform_summary(aggregate: pd.DataFrame, metric: str) -> pd.DataFrame:
    data = aggregate[aggregate["reliability"].str.startswith("R1_q")].copy()
    return (
        data.groupby(["planner", "q_nominal"], as_index=False)[metric]
        .mean()
        .sort_values("q_nominal")
    )


def _line_figure(
    aggregate: pd.DataFrame,
    metric: str,
    ylabel: str,
    title: str,
    path: Path,
) -> None:
    summary = _uniform_summary(aggregate, metric)
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    for planner, group in summary.groupby("planner"):
        ax.plot(
            group["q_nominal"], group[metric], marker="o", linewidth=2,
            label=planner, color=COLORS[planner]
        )
    ax.set(xlabel="Loop success probability q", ylabel=ylabel, title=title)
    ax.grid(alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def generate_summary_figures(
    aggregate: pd.DataFrame, pairwise: pd.DataFrame, output_dir: Path
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    _line_figure(aggregate, "mean_final_phi", "Mean normalized log-tree utility",
                 "Mean realized graph quality", output_dir / "fig1_mean_utility_vs_q.png")
    _line_figure(aggregate, "p05_final_phi", "5th percentile utility",
                 "Lower-tail realized graph quality", output_dir / "fig2_p05_utility_vs_q.png")
    _line_figure(aggregate, "low_quality_probability", "P(final utility < fixed threshold)",
                 "Low-quality final-graph risk", output_dir / "fig3_failure_probability_vs_q.png")
    _line_figure(aggregate, "mean_travel_cost", "Mean travel cost",
                 "Travel cost under a shared budget", output_dir / "fig4_travel_cost_vs_q.png")

    gap_data = aggregate[
        aggregate["reliability"].str.startswith("R1_q")
        & aggregate["planner"].isin(["DeterministicStatic", "StaticExpected"])
    ]
    gap_summary = gap_data.groupby(["planner", "q_nominal"], as_index=False)["mean_prediction_gap"].mean()
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    for planner, group in gap_summary.groupby("planner"):
        ax.plot(group["q_nominal"], group["mean_prediction_gap"], marker="o",
                linewidth=2, label=planner, color=COLORS[planner])
    ax.set(xlabel="Loop success probability q", ylabel="Mean predicted - realized utility",
           title="Hypothetical-realized graph mismatch (static planners)")
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.grid(alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(output_dir / "fig5_prediction_gap_vs_q.png", dpi=180)
    plt.close(fig)

    heat = pairwise[
        pairwise["baseline"] == "StaticExpected"
    ].copy()
    heat = heat[heat["reliability"].str.startswith("R1_q")]
    heat = heat.groupby(["budget", "q_nominal"], as_index=False)["paired_mean"].mean()
    matrix = heat.pivot(index="budget", columns="q_nominal", values="paired_mean")
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    image = ax.imshow(matrix.values, aspect="auto", cmap="coolwarm",
                      vmin=-np.max(np.abs(matrix.values)), vmax=np.max(np.abs(matrix.values)))
    ax.set_xticks(range(len(matrix.columns)), [f"{q:.1f}" for q in matrix.columns])
    ax.set_yticks(range(len(matrix.index)), [f"{b:g}" for b in matrix.index])
    ax.set(xlabel="q", ylabel="Budget", title="Adaptive - StaticExpected mean utility")
    for row in range(matrix.shape[0]):
        for col in range(matrix.shape[1]):
            ax.text(col, row, f"{matrix.iloc[row, col]:.4f}", ha="center", va="center", fontsize=8)
    fig.colorbar(image, ax=ax, label="Paired mean gain")
    fig.tight_layout()
    fig.savefig(output_dir / "fig6_adaptive_gain_heatmap.png", dpi=180)
    plt.close(fig)


def _draw_episode(
    ax: plt.Axes,
    initial_graph: nx.Graph,
    candidates: dict[str, CandidateLoop],
    result: Optional[EpisodeResult],
    title: str,
) -> None:
    positions = {node: (initial_graph.nodes[node]["x"], initial_graph.nodes[node]["y"])
                 for node in initial_graph.nodes}
    nx.draw_networkx_nodes(initial_graph, positions, node_size=14, node_color="#333333", ax=ax)
    nx.draw_networkx_edges(initial_graph, positions, width=0.8, edge_color="#aaaaaa", ax=ax)
    if result is not None:
        planned_edges = [
            (candidates[action.edge_id].u, candidates[action.edge_id].v)
            for action in result.actions
        ]
        nx.draw_networkx_edges(
            initial_graph, positions, edgelist=planned_edges,
            edge_color="#2f6fab", style="dotted", width=3.2, alpha=0.65, ax=ax
        )
        for action in result.actions:
            candidate = candidates[action.edge_id]
            color = "#2ca02c" if action.success else "#c62828"
            style = "solid" if action.success else "dashed"
            nx.draw_networkx_edges(
                initial_graph, positions, edgelist=[(candidate.u, candidate.v)],
                edge_color=color, style=style, width=1.8, ax=ax
            )
    ax.set_title(title)
    ax.set_aspect("equal")
    ax.axis("off")


def plot_representative(
    initial_graph: nx.Graph,
    candidates: list[CandidateLoop],
    static_result: EpisodeResult,
    adaptive_result: EpisodeResult,
    output_path: Path,
) -> None:
    lookup = {candidate.edge_id: candidate for candidate in candidates}
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))
    _draw_episode(axes[0], initial_graph, lookup, None, "Initial pose graph")
    _draw_episode(axes[1], initial_graph, lookup, static_result, "StaticExpected realized")
    _draw_episode(axes[2], initial_graph, lookup, adaptive_result, "AdaptiveRealized")
    legend = [
        Line2D([0], [0], color="#aaaaaa", linewidth=1.0, label="odometry"),
        Line2D([0], [0], color="#2f6fab", linewidth=3.0, linestyle="dotted", label="planned/executed loop"),
        Line2D([0], [0], color="#2ca02c", linewidth=2.0, label="successful loop"),
        Line2D([0], [0], color="#c62828", linewidth=2.0, linestyle="dashed", label="failed loop"),
    ]
    fig.legend(handles=legend, loc="upper center", ncol=4, frameon=False)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)
