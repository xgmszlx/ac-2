#!/usr/bin/env bash
# Bring up the simulation, then the agent half, and report whether the exploration
# loop closes: frontiers -> decision -> goal -> base command.
set +u
L=/home/wcqw/explorb_line_20261121
source $L/env.sh; source $L/ws/devel/setup.bash
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11331}
export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-http://localhost:11346}
TASK=${1:-probe}
RUN=$L/results/runs/$TASK
mkdir -p $RUN
$L/cleanup.sh > /dev/null 2>&1

roslaunch explorb_scenarios sim_house.launch spawn:=false > $L/logs/${TASK}_sim.log 2>&1 &
SIM=$!
for i in $(seq 1 50); do timeout 5 rosservice list 2>/dev/null | grep -q '/gazebo/spawn_urdf_model' && break; sleep 3; done
sleep 14
rosrun gazebo_ros spawn_model -urdf -param robot_description -model robot_1 -x 0 -y 0 -z 0.02 > $L/logs/${TASK}_spawn.log 2>&1
sleep 8
timeout 60 python3 $L/tools/wait_visual.py 50 200 40 > /dev/null 2>&1
roslaunch explorb_scenarios stack.launch task_id:=$TASK budget_sim_s:=120 run_dir:=$RUN > $L/logs/${TASK}_agent.log 2>&1 &
AG=$!
sleep 75
echo "--- obs-instr ---"
grep -a obs-instr $L/logs/${TASK}_sim.log | tail -1
echo "--- frontier/decision/action counts ---"
for t in /frontier_detectors/detected_points \
         /frontier_detectors/opencv_detector/detected_point \
         /frontier_detectors/filter/filtered_points \
         /gridmapper/rectified_map /robot_1/move_base/goal; do
  N=$(timeout 10 python3 - "$t" <<'PY'
import sys, rospy, time
rospy.init_node('cnt', anonymous=True)
got = [0]
rospy.Subscriber(sys.argv[1], rospy.AnyMsg, lambda m: got.__setitem__(0, got[0] + 1))
time.sleep(4)
print(got[0])
PY
)
  echo "  $t -> $N msgs/4s"
done
echo "--- decision_maker log ---"
grep -aE "Controller received map|No frontiers|Information record|assigned to|Plan length" $L/logs/${TASK}_agent.log | tail -6
kill -INT $AG 2>/dev/null; kill -INT $SIM 2>/dev/null
sleep 4
$L/cleanup.sh > /dev/null
