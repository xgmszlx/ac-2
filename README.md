# Adaptive Realized-Graph Planning — Phase 0

Independent, lightweight proof-of-value experiment for realized pose-graph replanning. It uses only synthetic weighted graphs and does not depend on or modify ROS, Karto, or another SLAM repository.

Run from this directory:

```bash
python -m pytest
python -m src.experiment --config configs/base.yaml --trials 200
# Full frozen grid used by the report:
python -m src.experiment --config configs/base.yaml --trials 1000
```

The frozen configuration is in `configs/base.yaml`. Per-trial evidence, paired edge latents, summaries, figures, runtime metadata, and the feasibility decision are written below `results/` and `docs/`.

The large reproducible files `results/raw/trials.csv` and
`results/raw/edge_outcomes.csv` are intentionally excluded from Git. Recreate
them with the full-grid command above; the compact candidate manifest,
aggregate tables, figures, and reports are versioned.
