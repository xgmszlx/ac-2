#!/usr/bin/env bash
# Kill leftover simulation processes of this line.
#
# Safety: the patterns live in this file (so the invoking shell's own command line
# never matches them) and every ancestor of this script is excluded, so a caller
# can never kill its own shell.
set +u
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
echo "cleanup: killed=$killed remaining=$(pgrep -cf 'gzserver -e ode')"
