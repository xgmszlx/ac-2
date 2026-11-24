#!/usr/bin/env bash
# Run one complete timed exploration task of the ExplORB-SLAM localization line.
#
#   run_task.sh <strategy> <scenario> <task_id> <budget_sim_s> [seed] [extra roslaunch args...]
#
# Everything (build, logs, results, ROS master, Gazebo master) stays inside this
# line's own directory so that it cannot collide with other work on the machine.
#
# The run is sequenced in two stages on purpose: on this machine gzserver dies with a
# libtinyxml segfault if the frontier detectors / decision maker / recorders are
# started in the same roslaunch as Gazebo (they hit the master while the robot is
# being spawned). Stage 1 brings the simulation up and waits until the robot really
# publishes odometry; stage 2 starts the agent half.
set +u

LINE_ROOT=/home/wcqw/explorb_line_20261121
STRATEGY=${1:-native}
SCENARIO=${2:-static}
TASK_ID=${3:-task}
BUDGET=${4:-900}
SEED=${5:-0}
shift 5 2>/dev/null || shift $#
EXTRA_ARGS="$*"

source "$LINE_ROOT/env.sh"

# dedicated ROS / Gazebo masters (never the default 11311/11345)
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11331}
export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-http://localhost:11346}

source "$LINE_ROOT/ws/devel/setup.bash"

# ---------------------------------------------------------------------------
# catkin's devel/setup.bash *replaces* ROS_PACKAGE_PATH with "<ws>/src:/opt/ros/noetic/share",
# which silently drops the no-sudo local prefix set by env.sh. The packages that live
# only in localroot (teb_local_planner, costmap_converter, octomap_ros, mbf_*) then stop
# being discoverable: on 2026-11-25 that made move_base die with
#
#   [FATAL] Failed to create the teb_local_planner/TebLocalPlannerROS planner ...
#   Declared types are  base_local_planner/TrajectoryPlannerROS dwa_local_planner/DWAPlannerROS
#
# and every downstream symptom of the first three rounds (no global costmap publisher,
# Filter.py blocked in wait_for_service(make_plan), no goal, no cmd_vel) followed from it.
# Re-prepending the prefix here keeps both the workspace and the local prefix visible.
# ---------------------------------------------------------------------------
export ROS_PACKAGE_PATH="$LINE_ROOT/localroot/opt/ros/noetic/share:$ROS_PACKAGE_PATH"
if ! rospack find teb_local_planner >/dev/null 2>&1; then
    echo "[run_task] ERROR: teb_local_planner is not discoverable - move_base would start without"
    echo "[run_task]        its local planner (see the ROS_PACKAGE_PATH note above)."
fi
echo "[run_task] teb_local_planner -> $(rospack find teb_local_planner 2>/dev/null || echo MISSING)"

RUN_DIR="$LINE_ROOT/results/runs/$TASK_ID"
mkdir -p "$RUN_DIR"
SIMLOG="$LINE_ROOT/logs/${TASK_ID}_sim.log"
AGENTLOG="$LINE_ROOT/logs/${TASK_ID}_agent.log"

# wall-clock backstop: the budget guard stops the run on the simulation clock; this
# only guarantees that a batch cannot hang if something wedges.
WALL_CAP=$(( BUDGET * 4 + 600 ))

echo "[run_task] strategy=$STRATEGY scenario=$SCENARIO task=$TASK_ID budget=${BUDGET}sim-s seed=$SEED"
echo "[run_task] run_dir=$RUN_DIR"

"$LINE_ROOT/cleanup.sh" > /dev/null 2>&1

# ---------------------------------------------------------------- stage 1: sim
timeout --signal=INT --kill-after=20 "$WALL_CAP" \
  roslaunch explorb_scenarios sim_house.launch spawn:=false $EXTRA_ARGS > "$SIMLOG" 2>&1 &
SIM_PID=$!

echo "[run_task] stage 1a: waiting for the Gazebo spawn service"
SPAWN_UP=0
for i in $(seq 1 60); do
    if timeout 6 rosservice list 2>/dev/null | grep -q '/gazebo/spawn_urdf_model'; then
        SPAWN_UP=1
        break
    fi
    sleep 3
done
if [ "$SPAWN_UP" != "1" ]; then
    echo "[run_task] ERROR: gazebo never advertised the spawn service; see $SIMLOG"
    "$LINE_ROOT/cleanup.sh" > /dev/null 2>&1
    exit 1
fi
# let the world finish streaming in before a body is inserted (spawning during world
# load is what makes gzserver die with a libtinyxml segfault on this machine)
sleep 12

echo "[run_task] stage 1b: spawning the robot (up to 3 attempts)"
SPAWNED=0
for attempt in 1 2 3; do
    if timeout 90 rosrun gazebo_ros spawn_model -urdf -param robot_description \
            -model robot_1 -x 0.0 -y 0.0 -z 0.02 > "$LINE_ROOT/logs/${TASK_ID}_spawn.log" 2>&1; then
        if grep -q "Successfully spawned" "$LINE_ROOT/logs/${TASK_ID}_spawn.log"; then
            SPAWNED=1
            echo "[run_task] robot spawned on attempt $attempt"
            break
        fi
    fi
    echo "[run_task] spawn attempt $attempt failed; retrying"
    sleep 5
done
if [ "$SPAWNED" != "1" ]; then
    echo "[run_task] ERROR: could not spawn the robot; see ${TASK_ID}_spawn.log"
    "$LINE_ROOT/cleanup.sh" > /dev/null 2>&1
    exit 1
fi

echo "[run_task] stage 1c: waiting for robot odometry"
WAITED=0
READY=0
while [ "$WAITED" -lt 240 ]; do
    N=$(timeout 8 rostopic echo -n1 /odom/header/seq 2>/dev/null | tr -d ' \n')
    if [ -n "$N" ]; then
        sleep 3
        N2=$(timeout 8 rostopic echo -n1 /odom/header/seq 2>/dev/null | tr -d ' \n')
        if [ -n "$N2" ] && [ "$N" != "$N2" ]; then
            echo "[run_task] stage 1 ready after ${WAITED}s (odom seq $N -> $N2)"
            READY=1
            break
        fi
    fi
    sleep 4
    WAITED=$((WAITED + 4))
done
if [ "$READY" != "1" ]; then
    echo "[run_task] ERROR: simulation never produced odometry; see $SIMLOG"
    "$LINE_ROOT/cleanup.sh" > /dev/null 2>&1
    exit 1
fi
# make sure the rendered image actually has structure before the agent starts; the
# SLAM frontend cannot initialise on the flat background of a half-streamed world
echo "[run_task] stage 1d: waiting for a visually ready camera frame"
if ! timeout 120 python3 "$LINE_ROOT/tools/wait_visual.py" 50 200 90; then
    echo "[run_task] WARN: camera frame never became visually rich"
fi

# -------------------------------------------------------------- stage 2: agent
echo "[run_task] stage 2: starting agent half"
timeout --signal=INT --kill-after=20 "$(( WALL_CAP - WAITED ))" \
  roslaunch explorb_scenarios stack.launch \
    task_id:="$TASK_ID" strategy:="$STRATEGY" scenario:="$SCENARIO" \
    seed:="$SEED" budget_sim_s:="$BUDGET" run_dir:="$RUN_DIR" \
    > "$AGENTLOG" 2>&1
RC=$?
echo "[run_task] agent half exited rc=$RC"

kill -INT $SIM_PID 2>/dev/null
sleep 4
"$LINE_ROOT/cleanup.sh" > /dev/null 2>&1

echo "[run_task] done: $(ls "$RUN_DIR" 2>/dev/null | tr '\n' ' ')"
if [ -f "$RUN_DIR/budget_reached.txt" ]; then
    echo "[run_task] budget marker: $(cat "$RUN_DIR/budget_reached.txt")"
fi
exit 0
