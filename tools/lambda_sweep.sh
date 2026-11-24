#!/usr/bin/env bash
# Lambda sensitivity *calibration* for the two visual-constraint weights.
#
# The task book asks for calibration on an independent setup before any evaluation run.
# This is that calibration: same fixed episode as the A/B (so the trajectory shape is
# comparable), only lambda_g=lambda_c=lambda changes. It is NOT an evaluation seed and
# the numbers are reported as a sensitivity table, not as a result.
#
#   lambda_sweep.sh "<l1> <l2> ..."   (default: "5 20 80")
set +u
LINE_ROOT=/home/wcqw/explorb_line_20261121
LAMBDAS=${1:-"5 20 80"}
TABLE=$LINE_ROOT/results/lambda_sweep.csv

export PATH=/home/wcqw/.dsh_heavy:$PATH
export FIXED_GOALS=true
export FIXED_GOALS_CSV=$LINE_ROOT/results/fixed_goals_ab.csv
export WITH_DECISION_MAKER=false
export USE_DYNAMIC_MASK=true

[ -f "$TABLE" ] || echo "task,lambda,reached,path_m,net_m,coverage_m2,ate_rmse_m,teb_ms,n_cmd,visual_edges,covis_edges" > "$TABLE"

for L in $LAMBDAS; do
    TASK="lam_${L}_visual"
    echo "===== lambda=$L task=$TASK ====="
    heavy --line A --note "lambda-$L" --max-wait 2700 --timeout 3600 -- \
        "$LINE_ROOT/run_task.sh" native visual "$TASK" 230 0 \
        "lambda_g:=$L" "lambda_c:=$L" "visual_use_dynamic:=true"
    RC=$?
    if [ "$RC" = "75" ] || [ "$RC" = "76" ] || [ "$RC" = "124" ]; then
        echo "[lambda] lambda=$L NOT RUN (heavy rc=$RC)"
        continue
    fi
    python3 "$LINE_ROOT/tools/eval_run.py" "$LINE_ROOT/results/runs/$TASK" >/dev/null 2>&1
    simlog="$LINE_ROOT/logs/${TASK}_sim.log"
    teb=$(grep -h "teb-solve" "$simlog" 2>/dev/null | tail -1 | sed 's/.*mean=//;s/ ms.*//')
    edg=$(grep -h "visual-constraint" "$simlog" 2>/dev/null | tail -1 | sed 's/.*visual_edges=//')
    python3 - "$TASK" "$L" "$LINE_ROOT/results/runs/$TASK" "$TABLE" "${teb:-}" "${edg:-}" <<'PY'
import json, os, sys, csv
task, lam, rundir, table, teb, edg = sys.argv[1:7]
m = json.load(open(os.path.join(rundir, 'metrics.json'))) if os.path.exists(os.path.join(rundir, 'metrics.json')) else {}
ve, ce = ('', '')
if edg:
    parts = edg.split()
    ve = parts[0] if parts else ''
    ce = parts[1].split('=')[1] if len(parts) > 1 and '=' in parts[1] else ''
row = [task, lam, m.get('n_reached'), m.get('path_len_m'), m.get('robot_moved_m'),
       m.get('coverage_m2'), m.get('ate_rmse_m'), teb, m.get('n_cmd'), ve, ce]
with open(table, 'a', newline='') as f:
    csv.writer(f).writerow(row)
print('[lambda] row:', row)
PY
done
echo "[lambda] table: $TABLE"
cat "$TABLE"
