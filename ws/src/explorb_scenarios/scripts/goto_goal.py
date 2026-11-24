#!/usr/bin/env python3
"""Fixed start/end goal dispatcher (the task book's phase-1 escape hatch).

The book asks for this explicitly: if the native closed loop (frontier -> filter ->
goal) is blocked, a deterministic "drive from A to B" dispatcher must exist so that
phases 2-4 are not blocked on it. It is also what phase 4.2 means by a *fixed
start-end* comparison: every arm gets byte-identical goals at identical simulated
times, so a difference can only come from the local planner cost, never from a
different goal sequence.

Usage
-----
    rosrun explorb_scenarios goto_goal.py _goals_csv:=/path/goals_fixed.csv \
                                             _run_dir:=.../pilot_x
    rosrun explorb_scenarios goto_goal.py _demo:=true          # A->B->A in a straight line

CSV format (one goal per line):  t_sim, x, y, yaw_deg
A goal is dispatched when the *simulated* clock reaches t_sim.

This node never reads simulator ground truth; the goal list is a script of the
experiment, exactly like a fixed benchmark route.
"""
from __future__ import print_function

import csv
import math
import os

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion


def yaw_to_quat(yaw_deg):
    yaw = math.radians(yaw_deg)
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class FixedGoalDispatcher(object):
    def __init__(self):
        self.run_dir = rospy.get_param('~run_dir', '')
        goals_csv = rospy.get_param('~goals_csv', '')
        demo = bool(rospy.get_param('~demo', False))
        frame = rospy.get_param('~frame_id', 'map')
        ns = rospy.get_param('~move_base_ns', '/robot_1/move_base')

        if demo and not goals_csv:
            # a straight out-and-back leg in the free corridor in front of the spawn
            self.goals = [(20.0, 1.2, 0.0, 0.0), (45.0, 0.0, 0.0, 180.0)]
        elif goals_csv:
            self.goals = []
            with open(goals_csv) as f:
                for row in csv.reader(f):
                    if not row or row[0].strip().startswith('#'):
                        continue
                    if row[0].strip().lower() in ('t', 't_sim'):
                        continue
                    self.goals.append((float(row[0]), float(row[1]), float(row[2]),
                                       float(row[3]) if len(row) > 3 else 0.0))
        else:
            rospy.logerr('goto_goal: need ~goals_csv or ~demo:=true')
            self.goals = []

        self.frame = frame
        self.log_path = os.path.join(self.run_dir, 'export', 'fixed_goals.log') if self.run_dir else ''
        if self.log_path:
            os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
            self.logf = open(self.log_path, 'w')
            self.logf.write('# t_sim, x, y, yaw_deg, dispatched_at\n')
            self.logf.flush()
        else:
            self.logf = None

        self.client = actionlib.SimpleActionClient(ns, MoveBaseAction)
        self.sent = []
        rospy.loginfo('goto_goal: %d fixed goals, waiting for %s' % (len(self.goals), ns))
        if not self.client.wait_for_server(rospy.Duration(60.0)):
            rospy.logerr('goto_goal: move_base action server never appeared; doing nothing')
            self.client = None
        else:
            rospy.loginfo('goto_goal: action server up')

    def run(self):
        # The schedule is *relative to this node's start*, not absolute simulated time.
        # The simulation is already ~1 min old when the agent half starts (stage 1 boots
        # Gazebo, spawns the robot and waits for a visually rich frame), so absolute
        # times would fire several goals at once and cancel each other. Relative timing
        # makes every arm see the identical goal sequence.
        t0 = rospy.get_time()
        relative = bool(rospy.get_param('~relative', True))
        rospy.loginfo('goto_goal: t0=%.2f s, %s schedule' % (t0, 'relative' if relative else 'absolute'))

        for (t_sim, x, y, yaw) in self.goals:
            # wait for the simulated clock to reach the scheduled time
            while not rospy.is_shutdown():
                now = rospy.get_time()
                target = (t0 + t_sim) if relative else t_sim
                if now >= target:
                    break
                rospy.sleep(0.2)
            if rospy.is_shutdown():
                return
            self.send(x, y, yaw, t_sim)
        rospy.loginfo('goto_goal: all fixed goals dispatched')
        while not rospy.is_shutdown():
            rospy.sleep(1.0)

    def send(self, x, y, yaw, t_sim):
        if self.client is None:
            return
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = yaw_to_quat(yaw)
        self.client.send_goal(goal)
        rospy.loginfo('goto_goal: t_sim=%.1f -> (%.2f, %.2f, %.1f deg)'
                      % (t_sim, x, y, yaw))
        if self.logf:
            self.logf.write('%f, %f, %f, %f, %f\n' % (t_sim, x, y, yaw, rospy.get_time()))
            self.logf.flush()
        self.sent.append((t_sim, x, y, yaw))


def main():
    rospy.init_node('goto_goal')
    d = FixedGoalDispatcher()
    if d.goals:
        d.run()
    else:
        rospy.spin()


if __name__ == '__main__':
    main()
