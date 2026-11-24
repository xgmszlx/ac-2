#!/usr/bin/env bash
# P1 diagnostic: one timed episode + periodic read-only ROS snapshots.
#
#   p1_diag.sh <task_id> <budget_sim_s> [strategy]
#
# Writes:
#   logs/<task_id>_sim.log      stage-1 Gazebo log (run_task.sh)
#   logs/<task_id>_agent.log    stage-2 agent log (contains the [fdbg] filter lines)
#   logs/<task_id>_probe.log    rostopic snapshots (topics / hz / one message each)
#   results/runs/<task_id>/     the recorder CSVs (goals/cmd/slam/odom/state/coverage)
#
# Everything here is read-only w.r.t. the running stack: it only inspects topics so
# that the root cause of "filtered_points is empty" can be decided from evidence
# instead of from guessed thresholds.
set +u

LINE_ROOT=/home/wcqw/explorb_line_20261121
TASK_ID=${1:-p1diag}
BUDGET=${2:-120}
STRATEGY=${3:-native}

source "$LINE_ROOT/env.sh"
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11331}
export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-http://localhost:11346}
source "$LINE_ROOT/ws/devel/setup.bash"

PROBE="$LINE_ROOT/logs/${TASK_ID}_probe.log"
: > "$PROBE"

snap() {
    local tag="$1"
    {
        echo "===== [$tag] wall=$(date +%H:%M:%S) ====="
        echo "--- topics (costmap/frontier/map/goal/cmd) ---"
        timeout 8 rostopic list 2>/dev/null | grep -E 'costmap|filtered_points|detected_points|rectified_map|/goal|cmd_vel|/map$' | sort
        for t in /robot_1/move_base_node/global_costmap/costmap \
                 /move_base_node/global_costmap/costmap \
                 /frontier_detectors/filter/filtered_points \
                 /frontier_detectors/detected_points \
                 /gridmapper/rectified_map; do
            echo "--- info $t ---"
            timeout 6 rostopic info "$t" 2>&1 | head -8
        done
        echo "--- hz filtered_points (5s) ---"
        timeout 6 rostopic hz -w 5 /frontier_detectors/filter/filtered_points 2>&1 | head -4
        echo "--- hz detected_points (5s) ---"
        timeout 6 rostopic hz -w 10 /frontier_detectors/detected_points 2>&1 | head -4
        echo "--- one filtered_points msg ---"
        timeout 6 rostopic echo -n1 /frontier_detectors/filter/filtered_points 2>&1 | head -12
        echo "--- services (planner) ---"
        timeout 8 rosservice list 2>/dev/null | grep -E 'make_plan|global_plan|move_base' | sort
        echo "--- rosnode list ---"
        timeout 8 rosnode list 2>/dev/null | grep -E 'move_base|teb|costmap' | sort
        echo "--- one rectified_map info ---"
        timeout 6 rostopic echo -n1 /gridmapper/rectified_map/info 2>&1 | head -14
        echo "--- orb slam state ---"
        timeout 4 rostopic echo -n1 /orb_slam2_rgbd/info/state 2>&1 | head -4
    } >> "$PROBE" 2>&1
}

echo "[p1_diag] task=$TASK_ID budget=${BUDGET}sim-s strategy=$STRATEGY debug=${EXPLORB_FILTER_DEBUG:-0}"

"$LINE_ROOT/run_task.sh" "$STRATEGY" static "$TASK_ID" "$BUDGET" 0 &
RT_PID=$!

# first snapshot once the agent half is expected to be up (~90 s in on this machine)
sleep 75
snap "t+75"
sleep 30
snap "t+105"
sleep 30
snap "t+135"
sleep 30
snap "t+165"

wait $RT_PID
RC=$?
echo "[p1_diag] run_task.sh rc=$RC"
echo "[p1_diag] probe -> $PROBE"
echo "[p1_diag] fdbg lines in agent log: $(grep -c '\[fdbg\]' "$LINE_ROOT/logs/${TASK_ID}_agent.log" 2>/dev/null)"
echo "[p1_diag] run dir: $(ls "$LINE_ROOT/results/runs/$TASK_ID" 2>/dev/null | tr '\n' ' ')"
for f in goals cmd slam odom state coverage clock; do
    printf '[p1_diag] %s.csv lines=%s\n' "$f" "$(wc -l < "$LINE_ROOT/results/runs/$TASK_ID/$f.csv" 2>/dev/null || echo 0)"
done
exit $RC
