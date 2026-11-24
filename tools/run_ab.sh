#!/usr/bin/env bash
# 4-arm x N-repeat A/B over the fixed start->end route (task book phase 4).
#
#   run_ab.sh <scenario> <budget_sim_s> <repeats> [arm...]
#
# Every episode is a *separate* heavy-lock job (the brief's rule: one episode per
# acquisition, so the other research line can interleave). Each run writes its own
# results/runs/<task_id>/ and a row in results/ab_table.csv.
#
# Arms (everything except these weights is identical, including the dynamic-pixel mask,
# the costmap, the goal sequence and the recorder):
#   B0            lambda_g=0.0  lambda_c=0.0  visual_use_dynamic=any
#   B1            lambda_g=1.0  lambda_c=1.0  visual_use_dynamic=false
#   Ours          lambda_g=1.0  lambda_c=1.0  visual_use_dynamic=true
#   Ours_no_cov   lambda_g=1.0  lambda_c=0.0  visual_use_dynamic=true
set +u

LINE_ROOT=/home/wcqw/explorb_line_20261121
SCENARIO=${1:-static}
BUDGET=${2:-240}
REPEATS=${3:-1}
shift 3 2>/dev/null || true
ARMS=("$@")
if [ ${#ARMS[@]} -eq 0 ]; then
    ARMS=(B0 B1 Ours Ours_no_cov)
fi

export PATH=/home/wcqw/.dsh_heavy:$PATH
GOALS_CSV=$LINE_ROOT/results/fixed_goals_ab.csv
TABLE=$LINE_ROOT/results/ab_table.csv
GIT_SHA=$(git -C "$LINE_ROOT" rev-parse --short HEAD)
BRANCH=$(git -C "$LINE_ROOT" rev-parse --abbrev-ref HEAD)

[ -f "$TABLE" ] || echo "task,arm,scenario,repeat,git_sha,branch,budget_sim_s,success,n_goal,n_cmd,path_len_m,robot_moved_m,coverage_m2,ate_rmse_m,lost_s,teb_norm,tracking_ok_pct,note" > "$TABLE"

arm_params() {
    case "$1" in
        B0)           echo "0.0 0.0 true"  ;;
        B1)           echo "1.0 1.0 false" ;;
        Ours)         echo "1.0 1.0 true"  ;;
        Ours_no_cov)  echo "1.0 0.0 true"  ;;
        *)            echo "INVALID"        ;;
    esac
}

for arm in "${ARMS[@]}"; do
    read -r LG LC DYN <<< "$(arm_params "$arm")"
    if [ "$LG" = "INVALID" ]; then
        echo "[ab] unknown arm: $arm"; continue
    fi
    for rep in $(seq 1 "$REPEATS"); do
        TASK=$(printf 'ab_%s_%s_r%d' "$arm" "$SCENARIO" "$rep")
        echo "=============================================================="
        echo "[ab] arm=$arm rep=$rep task=$TASK lambda_g=$LG lambda_c=$LC dynamic=$DYN"
        echo "=============================================================="
        export FIXED_GOALS=true
        export FIXED_GOALS_CSV="$GOALS_CSV"
        export WITH_DECISION_MAKER=false
        export USE_DYNAMIC_MASK=true
        heavy --line A --note "ab-$arm-r$rep" --max-wait 2700 --timeout 3600 -- \
            "$LINE_ROOT/run_task.sh" native "$SCENARIO" "$TASK" "$BUDGET" 0 \
            "lambda_g:=$LG" "lambda_c:=$LC" "visual_use_dynamic:=$DYN"
        RC=$?
        if [ "$RC" = "75" ] || [ "$RC" = "76" ] || [ "$RC" = "124" ]; then
            echo "[ab] arm=$arm rep=$rep NOT RUN (heavy rc=$RC) - not an algorithm failure"
            continue
        fi
        # offline metrics (GT only here, never online)
        if [ -d "$LINE_ROOT/results/runs/$TASK" ]; then
            python3 "$LINE_ROOT/tools/eval_run.py" "$LINE_ROOT/results/runs/$TASK" >/dev/null 2>&1
            python3 - "$TASK" "$arm" "$SCENARIO" "$rep" "$GIT_SHA" "$BRANCH" "$BUDGET" "$TABLE" "$LINE_ROOT/results/runs/$TASK" <<'PY'
import json, sys, os, csv
task, arm, scen, rep, sha, branch, budget, table, rundir = sys.argv[1:10]
mpath = os.path.join(rundir, 'metrics.json')
m = json.load(open(mpath)) if os.path.exists(mpath) else {}
# TEB solve-time statistic from the sim log (throttled [teb-solve] lines)
teb_norm = ''
simlog = '/home/wcqw/explorb_line_20261121/logs/%s_sim.log' % task
nx = 0
if os.path.exists(simlog):
    for line in open(simlog, errors='ignore'):
        if '[teb-solve]' in line and 'mean=' in line:
            nx += 1
            try:
                teb_norm = line.split('mean=')[1].split('ms')[0].strip()
            except Exception:
                pass
row = [task, arm, scen, rep, sha, branch, budget,
       m.get('success'), m.get('n_goal'), m.get('n_cmd'), m.get('path_len_m'),
       m.get('robot_moved_m'), m.get('coverage_m2'), m.get('ate_rmse_m'),
       m.get('lost_s'), teb_norm, m.get('tracking_ok_pct'),
       '' if m else 'no metrics.json']
with open(table, 'a', newline='') as f:
    csv.writer(f).writerow(row)
print('[ab] row:', row)
PY
        fi
    done
done

echo "[ab] table: $TABLE"
cat "$TABLE"
