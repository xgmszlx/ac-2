#!/usr/bin/env bash
# Route pre-flight: ask the global planner about every waypoint BEFORE using it.
#
# Motivation (2026-11-25): a fixed route containing (5.27,0.87) aborted with
# `status=4 "Failed to find a valid plan"` 100 ms after dispatch in several generations,
# and the round was wasted. The costmap bounds were NOT the reason (25 m x 25 m at origin
# -12.5 covers it); either that cell was lethal *at that moment* or the corridor to it did
# not exist yet. Guessing from the outside is the wrong tool: ask NavFn.
#
#   tools/preflight_route.sh <route.csv> [out_csv] [--keep-only-reachable out_route.csv]
#
# For every waypoint it calls /robot_1/move_base_node/NavfnROS/make_plan from the robot's
# current pose and records (t_route, x, y, plan_points, status). Only plan_points > 0
# waypoints are usable; with --keep-only-reachable a filtered route is written that the run
# can then use. Run it after the robot is spawned and the global costmap has published at
# least one full grid (i.e. inside an episode, before the agent half starts).
#
# Must run inside the shared heavy lock and with the line's environment:
#   heavy --line A -- cat tools/preflight_route.sh route.csv
set +u
LINE_ROOT=/home/wcqw/explorb_line_20261121
ROUTE=${1:?usage: preflight_route.sh <route.csv> [out.csv] [--keep-only-reachable out_route.csv]}
OUT=${2:-$LINE_ROOT/results/route_preflight.csv}
KEEP=""
if [ "$3" = "--keep-only-reachable" ]; then KEEP=$4; fi

source "$LINE_ROOT/env.sh" >/dev/null 2>&1
source "$LINE_ROOT/ws/devel/setup.bash" >/dev/null 2>&1
export ROS_PACKAGE_PATH="$LINE_ROOT/ws/src:$LINE_ROOT/localroot/opt/ros/noetic/share:/opt/ros/noetic/share"
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
SVC=/robot_1/move_base_node/NavfnROS/make_plan

if ! timeout 10 rosservice list 2>/dev/null | grep -q "$SVC"; then
    echo "[preflight] ERROR: $SVC is not advertised; is move_base up?"
    exit 2
fi

# the robot's current pose (map frame) is the plan start
START=$(timeout 10 python3 - <<'PY'
import rospy, tf
rospy.init_node('preflight_pose', anonymous=True)
l = tf.TransformListener()
l.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(5))
t, q = l.lookupTransform('map', 'base_footprint', rospy.Time(0))
print('%.3f %.3f %.3f %.3f' % (t[0], t[1], q[2], q[3]))
PY
) || { echo "[preflight] ERROR: cannot read the robot pose"; exit 2; }
read -r SX SY SQZ SQW <<< "$START"
echo "[preflight] start=($SX, $SY) service=$SVC"

echo "t_route,x,y,plan_points,status" > "$OUT"
: > "${KEEP:-/dev/null}"

[ -n "$KEEP" ] && echo "# filtered by tools/preflight_route.sh on $(date -Is)" > "$KEEP"
n_ok=0; n_bad=0
while IFS=, read -r t x y yaw; do
    case "$t" in \#*|"") continue ;; esac
    x=$(echo "$x" | tr -d ' '); y=$(echo "$y" | tr -d ' ')
    resp=$(timeout 25 rosservice call "$SVC" "start:
  header: {frame_id: map}
  pose: {position: {x: $SX, y: $SY, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $SQZ, w: $SQW}}
goal:
  header: {frame_id: map}
  pose: {position: {x: $x, y: $y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
tolerance: 0.1" 2>&1)
    pts=$(printf '%s\n' "$resp" | grep -c '^ *- header:' 2>/dev/null || true)
    pts=${pts:-0}
    if [ "$pts" -gt 0 ]; then
        status=reachable; n_ok=$((n_ok + 1))
        [ -n "$KEEP" ] && echo "$t, $x, $y, ${yaw:-0.0}" >> "$KEEP"
    else
        status="no_plan"; n_bad=$((n_bad + 1))
    fi
    echo "$t,$x,$y,$pts,$status" >> "$OUT"
    printf '[preflight] (%s,%s) plan_points=%s %s\n' "$x" "$y" "$pts" "$status"
done < "$ROUTE"

echo "[preflight] reachable=$n_ok unreachable=$n_bad -> $OUT"
[ -n "$KEEP" ] && echo "[preflight] filtered route -> $KEEP"
[ "$n_bad" -eq 0 ]
