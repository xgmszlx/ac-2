#!/usr/bin/env python3
"""Per-run recorder for the ExplORB-SLAM localization-observability line.

Writes compact CSV streams (no images) into <run_dir>:

  gt.csv          sim-time ground-truth robot pose from Gazebo (world frame)
  slam.csv        ORB-SLAM estimate (map frame) + tracking state
  odom.csv        wheel odometry pose
  cmd.csv         commanded base velocity actually delivered to the base
  goals.csv       move_base action goals/results as observed on the wire
  state.csv       tracking-state transitions (lost / relocalized events)
  coverage.csv    mapped-free / known-free area + path length (sampled)
  clock.csv       sim-time / wall-time ratio (real-time factor)

Everything is stamped with ROS (sim) time because use_sim_time is true.
"""
from __future__ import print_function

import csv
import os
import sys
import math
import time

import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock

try:
    from orb_slam2_ros.msg import ORBState
except Exception:                                            # pragma: no cover
    ORBState = None

try:
    import dynamic_reconfigure.client  # noqa: F401  (import guard only)
except Exception:
    pass


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Writer(object):
    def __init__(self, path, header):
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(header)
        self.f.flush()

    def row(self, *vals):
        self.w.writerow(vals)
        self.f.flush()

    def close(self):
        try:
            self.f.close()
        except Exception:
            pass


class Recorder(object):
    def __init__(self, run_dir, robot_model, robot_ns, camera_type):
        self.run_dir = run_dir
        self.camera_type = camera_type
        self.robot_model = robot_model
        self.robot_ns = robot_ns
        self.t0 = rospy.get_time()
        self.t_wall0 = time.time()
        self._t_pu = None
        self._sim_pu = 0.0

        self.gt = Writer(os.path.join(run_dir, 'gt.csv'),
                         ['t', 'x', 'y', 'z', 'yaw', 'vx', 'wz'])
        self.slam = Writer(os.path.join(run_dir, 'slam.csv'),
                           ['t', 'x', 'y', 'z', 'yaw', 'qx', 'qy', 'qz', 'qw', 'state'])
        self.odom = Writer(os.path.join(run_dir, 'odom.csv'),
                           ['t', 'x', 'y', 'z', 'yaw', 'vx', 'wz'])
        self.cmd = Writer(os.path.join(run_dir, 'cmd.csv'), ['t', 'vx', 'wz'])
        self.goals = Writer(os.path.join(run_dir, 'goals.csv'),
                            ['t', 'kind', 'x', 'y', 'yaw', 'status', 'info'])
        self.state = Writer(os.path.join(run_dir, 'state.csv'),
                            ['t', 'state', 'state_name', 'wall'])
        self.cov = Writer(os.path.join(run_dir, 'coverage.csv'),
                          ['t', 'known_cells', 'free_cells', 'free_area_m2',
                           'path_len_m', 'n_path_pts'])
        self.clock = Writer(os.path.join(run_dir, 'clock.csv'),
                            ['t_sim', 'wall_elapsed', 'rtf'])

        self.last_gt = None
        self.slam_state = -1
        self.path_len = 0.0
        self.n_path_pts = 0
        self.last_cov_t = -1e9

        self._last = {}
        self._sim_now = 0.0
        rospy.Subscriber('/clock', Clock, self.on_simclock, queue_size=50)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.on_models, queue_size=20)
        rospy.Subscriber('/%s/camera_pose' % self.slam_ns(), PoseStamped, self.on_slam, queue_size=20)
        rospy.Subscriber('/odom', Odometry, self.on_odom, queue_size=20)
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd, queue_size=50)
        rospy.Subscriber('/%s/move_base/goal' % robot_ns, MoveBaseActionGoal, self.on_goal, queue_size=20)
        rospy.Subscriber('/%s/move_base/result' % robot_ns, MoveBaseActionResult, self.on_result, queue_size=20)
        if ORBState is not None:
            rospy.Subscriber('/%s/info/state' % self.slam_ns(), ORBState, self.on_state, queue_size=20)
        rospy.Subscriber('/gridmapper/rectified_map', OccupancyGrid, self.on_map, queue_size=5)
        rospy.Subscriber('/%s/move_base_node/global_plan' % robot_ns, Float64MultiArray,
                         self.on_plan_len, queue_size=5)
        rospy.Timer(rospy.Duration(1.0), self.on_clock)

    def slam_ns(self):
        return 'orb_slam2_%s' % self.camera_type

    # ---- callbacks -------------------------------------------------------
    def on_simclock(self, msg):
        self._sim_now = msg.clock.to_sec()

    def _due(self, key, period):
        last = self._last.get(key, -1e9)
        now = rospy.get_time()
        if now - last < period:
            return False
        self._last[key] = now
        return True

    def on_models(self, msg):
        if not self._due('gt', 0.05):
            return
        try:
            i = msg.name.index(self.robot_model)
        except ValueError:
            return
        p = msg.pose[i]
        tw = msg.twist[i]
        self.last_gt = (p.position.x, p.position.y)
        self.gt.row(rospy.get_time(), p.position.x, p.position.y, p.position.z,
                    yaw_of(p.orientation), tw.linear.x, tw.angular.z)

    def on_slam(self, msg):
        p = msg.pose
        self.slam.row(rospy.get_time(), p.position.x, p.position.y, p.position.z, yaw_of(p.orientation),
                      p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w, self.slam_state)

    def on_odom(self, msg):
        if not self._due('odom', 0.2):
            return
        p = msg.pose.pose
        self.odom.row(rospy.get_time(), p.position.x, p.position.y, p.position.z, yaw_of(p.orientation),
                      msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_cmd(self, msg):
        if not self._due('cmd', 0.05):
            return
        self.cmd.row(rospy.get_time(), msg.linear.x, msg.angular.z)

    def on_goal(self, msg):
        p = msg.goal.target_pose.pose
        self.goals.row(rospy.get_time(), 'goal', p.position.x, p.position.y, yaw_of(p.orientation),
                       '', msg.goal_id.id)

    def on_result(self, msg):
        self.goals.row(rospy.get_time(), 'result', '', '', '', msg.status.status, msg.status.text)

    def on_state(self, msg):
        prev = self.slam_state
        self.slam_state = msg.state
        if prev != msg.state:
            self.state.row(rospy.get_time(), msg.state, str(msg.state), rospy.get_time() - self.t0)

    def on_plan_len(self, msg):
        # move_base publishes its current global plan; count points as a sanity signal only
        try:
            self.n_path_pts = len(msg.data) // 2
        except Exception:
            pass

    def on_map(self, msg):
        now = rospy.get_time()
        if now - self.last_cov_t < 1.0:
            return
        self.last_cov_t = now
        known = 0
        free = 0
        for v in msg.data:
            if v == -1:
                continue
            known += 1
            if v < 50:
                free += 1
        area = free * msg.info.resolution * msg.info.resolution

        # path length from ground truth
        self.cov.row(now, known, free, area, self.path_len, self.n_path_pts)

    def on_clock(self, _ev):
        wall = time.time()
        sim = self._sim_now
        if self._t_pu is None:
            self._t_pu, self._sim_pu = wall, sim
            return
        d_wall = wall - self._t_pu
        if d_wall < 5.0:
            return
        rtf = (sim - self._sim_pu) / d_wall
        self._t_pu, self._sim_pu = wall, sim
        self.clock.row(sim, wall - self.t_wall0, rtf)

    def close(self):
        for w in (self.gt, self.slam, self.odom, self.cmd, self.goals, self.state, self.cov, self.clock):
            w.close()


def main():
    rospy.init_node('run_recorder', anonymous=False)
    run_dir = rospy.get_param('~run_dir')
    robot_model = rospy.get_param('~robot_model', 'robot_1')
    robot_ns = rospy.get_param('~robot_ns', 'robot_1')
    camera_type = rospy.get_param('~camera_type', 'rgbd')
    if not os.path.isdir(run_dir):
        os.makedirs(run_dir)
    rec = Recorder(run_dir, robot_model, robot_ns, camera_type)
    rospy.on_shutdown(rec.close)
    rospy.loginfo('run_recorder writing to %s' % run_dir)
    rospy.spin()


if __name__ == '__main__':
    main()
