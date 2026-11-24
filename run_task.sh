#!/usr/bin/env bash
# Run one complete timed exploration task of the ExplORB-SLAM localization line.
#
#   run_task.sh <strategy> <scenario> <task_id> <budget_sim_s> [seed] [extra roslaunch args...]
#
# Everything (build, logs, results, ROS master, Gazebo master) stays inside this
# line's own directory so that it cannot collide with other work on the machine.
set +u

LINE_ROOT=/home/wcqw/explorb_line_20261121
STRATEGY=${1:-native}
SCENARIO=${2:-static}
TASK_ID=${3:-smoke}
BUDGET=${4:-900}
SEED=${5:-0}
shift 5 2>/dev/null || shift $#
EXTRA_ARGS="$*"

source "$LINE_ROOT/env.sh"

# dedicated ROS / Gazebo masters (never the default 11311/11345)
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11331}
export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-http://localhost:11346}
export ROS_PORT_SIM=11331

source "$LINE_ROOT/ws/devel/setup.bash"

RUN_DIR="$LINE_ROOT/results/runs/$TASK_ID"
mkdir -p "$RUN_DIR"
LOG="$LINE_ROOT/logs/$TASK_ID.log"

# wall-clock backstop: generously larger than the sim budget (rendering can be
# slower than real time); the budget_guard stops the run on the sim clock.
WALL_CAP=$(( BUDGET * 4 + 420 ))

echo "[run_task] strategy=$STRATEGY scenario=$SCENARIO task=$TASK_ID budget=${BUDGET}s seed=$SEED"
echo "[run_task] run_dir=$RUN_DIR"
echo "[run_task] ROS_MASTER_URI=$ROS_MASTER_URI GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"

timeout --signal=INT --kill-after=30 "$WALL_CAP" \
  roslaunch explorb_scenarios experiment.launch \
    task_id:="$TASK_ID" \
    strategy:="$STRATEGY" \
    scenario:="$SCENARIO" \
    seed:="$SEED" \
    budget_sim_s:="$BUDGET" \
    run_dir:="$RUN_DIR" \
    $EXTRA_ARGS \
  > "$LOG" 2>&1

RC=$?
echo "[run_task] roslaunch exited rc=$RC (124/130 = budget/INT as expected)"
exit 0
