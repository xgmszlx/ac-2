#!/usr/bin/env bash
# Kill leftover simulation processes of this line -- and *only* this line's.
#
# Why the extra pass exists (2026-11-25): the pattern list below only covered the nodes
# that roslaunch itself starts. Nodes that roslaunch loses track of (a respawned
# Controller.py after a master restart, a rosout orphaned by a killed launch) kept
# running, reparented to systemd --user, and burned CPU for hours -- which pollutes the
# "[teb-solve]" real-time self-check and steals cycles from the other research line.
#
# Safety: every node started by this line carries `--log:=<LINE_ROOT>/logs/ros/...` on
# its command line (or lives in our own python paths), so matching on
# "$LINE_ROOT/logs/ros/" catches all of them and nothing belonging to another line.
set +u
LINE_ROOT=/home/wcqw/explorb_line_20261121
PIDFILE=$LINE_ROOT/.run_task.pid
FORCE=0
for a in "$@"; do [ "$a" = "--force" ] && FORCE=1; done

# Guard added after a real accident: a manual cleanup.sh run *during* an episode killed
# that episode's nodes (2026-11-25 02:30). run_task.sh writes its pid here; while it is
# alive, cleanup refuses to act unless --force is given.
if [ -f "$PIDFILE" ]; then
    OWNER=$(cat "$PIDFILE" 2>/dev/null)
    if [ -n "$OWNER" ] && kill -0 "$OWNER" 2>/dev/null; then
        if [ "$FORCE" != "1" ]; then
            echo "cleanup: REFUSED - run_task.sh (pid $OWNER) is running an episode."
            echo "cleanup:          if you really mean to kill it, use: cleanup.sh --force"
            exit 2
        fi
        echo "cleanup: --force given, killing the episode owned by pid $OWNER"
    fi
fi
SELF=$$
ANC=()
p=$SELF
while [ -n "$p" ] && [ "$p" != "0" ]; do
    ANC+=("$p")
    p=$(awk '{print $4}' /proc/$p/stat 2>/dev/null)
done

PATTERNS=(
    "gzserver -e ode"
    "gzserver --verbose"
    "/opt/ros/noetic/bin/rosmaster"
    "bin/roslaunch"
    "orb_slam2_ros_rgbd"
    "frontier_detector_global_rrt"
    "frontier_detector_local_rrt"
    "frontier_detector_gridmapper"
    "frontier_detector_octomapper"
    "decision_maker/Controller.py"
    "decision_maker/GraphDrawer.py"
    "frontier_detector/src/python/Filter.py"
    "frontier_detector/src/python/OpenCV_detector.py"
    "frontier_detector/src/python/Initializer.py"
    "move_base/move_base"
    "robot_state_publisher"
    "run_recorder.py"
    "budget_guard.py"
    "scenario_driver.py"
    "dynamic_mask.py"
    "slam_export.py"
    "goto_goal.py"
    # catch-all: every node of THIS line's ROS master carries this --log:= path
    "$LINE_ROOT/logs/ros/"
)

killed=0
for pat in "${PATTERNS[@]}"; do
    for pid in $(pgrep -f "$pat" 2>/dev/null); do
        skip=0
        for a in "${ANC[@]}"; do
            if [ "$pid" = "$a" ]; then skip=1; break; fi
        done
        [ "$skip" = "1" ] && continue
        if kill -9 "$pid" 2>/dev/null; then killed=$((killed + 1)); fi
    done
done
sleep 1
# Count only *real* leftovers: exclude this script's own ancestry, otherwise
# the pattern string inside the invoking shell's command line is counted as
# a "remaining" process (a cosmetic false alarm seen twice today).
count_real() {
    local pat="$1" n=0 pid skip a
    for pid in $(pgrep -f "$pat" 2>/dev/null); do
        skip=0
        for a in "${ANC[@]}"; do
            [ "$pid" = "$a" ] && skip=1 && break
        done
        [ "$skip" = "1" ] && continue
        n=$((n + 1))
    done
    echo "$n"
}
remaining=$(count_real "$LINE_ROOT/logs/ros/")
gz=$(count_real "gzserver -e ode")
echo "cleanup: killed=$killed remaining=$remaining gzserver=$gz"
# a non-zero exit here is what run_task.sh turns into "the machine is dirty, stop"
[ "${remaining:-0}" = "0" ]
