#!/usr/bin/env bash
# run_probe.sh <tag> <seconds_before_probe> [roslaunch args...]
set +u
L=/home/wcqw/explorb_line_20261121
TAG=$1; WAIT=$2; shift 2
source $L/env.sh
source $L/ws/devel/setup.bash
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11601}
export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-http://localhost:11606}
roslaunch $L/ws/src/explorb_scenarios/launch/sim_house.launch "$@" > $L/logs/probe_$TAG.log 2>&1 &
LP=$!
sleep $WAIT
echo "PROBE_RESULT $TAG $(timeout 30 python3 $L/tools/sensor_probe.py 8 2>&1 | tail -1)"
echo "SEGV $TAG $(grep -c Segmentation $L/logs/probe_$TAG.log)"
kill -INT $LP 2>/dev/null; sleep 3
$L/cleanup.sh > /dev/null
