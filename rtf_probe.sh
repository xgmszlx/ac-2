#!/usr/bin/env bash
# Measure the Gazebo real-time factor of a world+robot pair.
#   rtf_probe.sh <tag> <world_file> <model_path>
set +u
LINE_ROOT=/home/wcqw/explorb_line_20261121
TAG=${1:-probe}; WORLD=${2:-}; MODELS=${3:-}
shift 3 2>/dev/null || shift $#

source "$LINE_ROOT/env.sh"
source "$LINE_ROOT/ws/devel/setup.bash"
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11491}
export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-http://localhost:11496}
OUT="$LINE_ROOT/logs/rtf_$TAG.log"

roslaunch "$LINE_ROOT/ws/src/explorb_scenarios/launch/sim_house.launch" \
    world_file:="$WORLD" model_path:="$MODELS" > "$OUT" 2>&1 &
LPID=$!

# wait until /clock exists and moves
for i in $(seq 1 60); do
    V=$(timeout 5 rostopic echo -n1 /clock 2>/dev/null | tr -d ' \n')
    case "$V" in *secs:*[1-9]*) break;; esac
    sleep 3
done

timeout 60 python3 "$LINE_ROOT/ws/src/explorb_scenarios/scripts/rtf_probe.py" _window:=20.0 _subscribe_camera:=${SUBSCRIBE_CAM:-true} 2>/dev/null | grep RTF_PROBE

kill -INT $LPID 2>/dev/null
sleep 3
"$LINE_ROOT/cleanup.sh" > /dev/null 2>&1
