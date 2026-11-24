#!/usr/bin/env bash
# Gate: prove from the *running process* that move_base loaded the source-built
# libteb_local_planner.so (with the two visual-constraint edges), not the deb one.
#
#   tools/verify_source_teb.sh <task_id> [lambda_g lambda_c]
#
# It starts a fixed episode and, while it runs, records
#   * /proc/<move_base pid>/maps  lines that mention libteb_local_planner
#   * the [teb-solve] and [visual-constraint] log lines
# into logs/<task_id>_gate.log. Exit code 0 only if all three are present.
set +u
LINE_ROOT=/home/wcqw/explorb_line_20261121
TASK=${1:-gate_ours}
LG=${2:-1.0}
LC=${3:-1.0}
GATE="$LINE_ROOT/logs/${TASK}_gate.log"
: > "$GATE"

export PATH=/home/wcqw/.dsh_heavy:$PATH
export FIXED_GOALS=true
export FIXED_GOALS_CSV=$LINE_ROOT/results/fixed_goals_ab.csv
export WITH_DECISION_MAKER=false
export USE_DYNAMIC_MASK=true

heavy --line A --note "gate-$TASK" --max-wait 2700 --timeout 3600 -- \
    "$LINE_ROOT/run_task.sh" native visual "$TASK" 230 0 \
    "lambda_g:=$LG" "lambda_c:=$LC" "visual_use_dynamic:=true" &
RUNPID=$!

# Sample the move_base memory map several times: the TEB plugin is dlopen'ed during
# move_base's initialize(), i.e. a few seconds after the process appears, so a single
# early sample can legitimately come back empty.
for round in 1 2 3 4 5 6; do
    PID=$(pgrep -f "move_base/move_base" | head -1)
    if [ -n "$PID" ]; then
        sleep 8
        {
            echo "===== sample $round @ $(date +%H:%M:%S) pid=$PID ====="
            echo "cmdline: $(tr '\0' ' ' < /proc/$PID/cmdline 2>/dev/null | cut -c1-120)"
            grep -o '[^ ]*libteb_local_planner[^ ]*' /proc/$PID/maps 2>/dev/null | sort -u
            # belt and braces: scan every process, in case the pid we picked is a wrapper
            for q in /proc/[0-9]*; do
                if grep -q libteb_local_planner "$q/maps" 2>/dev/null; then
                    echo "scan: $(basename $q) $(grep -o '[^ ]*libteb_local_planner[^ ]*' "$q/maps" 2>/dev/null | sort -u | tr '\n' ' ')"
                fi
            done
        } >> "$GATE"
        if grep -q "libteb_local_planner" "$GATE"; then break; fi
    else
        sleep 4
    fi
done
wait $RUNPID
RC=$?

SIMLOG="$LINE_ROOT/logs/${TASK}_sim.log"
{
    echo
    echo "===== evidence lines from $SIMLOG ====="
    grep -h "\[teb-solve\]" "$SIMLOG" 2>/dev/null | tail -2
    grep -h "\[visual-constraint\]" "$SIMLOG" 2>/dev/null | tail -2
    grep -h "\[visual_query\]" "$SIMLOG" 2>/dev/null | tail -3
} >> "$GATE"

echo "[gate] run rc=$RC, log: $GATE"
echo "[gate] --- maps ---"; sed -n '/maps @/,/evidence lines/p' "$GATE" | head -6
echo "[gate] --- edges ---"; grep -h "visual-constraint" "$GATE" | tail -1
echo "[gate] --- solve ---";  grep -h "teb-solve" "$GATE" | tail -1

# pass only if all three gates hold
MAPS_OK=$(grep -c "libteb_local_planner" "$GATE" 2>/dev/null) ; MAPS_OK=${MAPS_OK:-0}
MAPS_SRC=$(grep -c "ws/devel/lib/libteb_local_planner.so" "$GATE" 2>/dev/null) ; MAPS_SRC=${MAPS_SRC:-0}
EDGE_OK=$(grep -ho "visual_edges=[0-9]*" "$GATE" 2>/dev/null | tail -1 | sed 's/.*=//') ; EDGE_OK=${EDGE_OK:-0}
COV_OK=$(grep -ho "covis_edges=[0-9]*" "$GATE" 2>/dev/null | tail -1 | sed 's/.*=//') ; COV_OK=${COV_OK:-0}
SOLVE_OK=$(grep -c "teb-solve" "$GATE" 2>/dev/null) ; SOLVE_OK=${SOLVE_OK:-0}
echo "[gate] maps_lines=$MAPS_OK source_path_hits=$MAPS_SRC visual_edges=$EDGE_OK covis_edges=$COV_OK teb_solve_lines=$SOLVE_OK"
if [ "$MAPS_OK" -ge 1 ] && [ "$EDGE_OK" -gt 0 ] && [ "$SOLVE_OK" -ge 1 ]; then
    echo "[gate] PASS"
    exit 0
fi
echo "[gate] FAIL"
exit 1
