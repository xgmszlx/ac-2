#!/usr/bin/env python3
"""Stop a timed exploration run on the *simulation* clock, not the wall clock.

use_sim_time is true for every node in this line, so a run budget expressed in
simulated seconds is the reproducible quantity: a run that is slowed down by
rendering/CPU does not silently get more or less exploration than another arm.

When the budget is reached the guard calls `rosnode kill -a`, which tears the
launch down (roslaunch exits once its nodes are gone). The runner additionally
imposes a wall-clock backstop so a wedged run cannot hang a batch.
"""
from __future__ import print_function

import os
import subprocess
import sys

import rospy
from rosgraph_msgs.msg import Clock

try:
    from orb_slam2_ros.msg import ORBState
except Exception:                                        # pragma: no cover
    ORBState = None


class BudgetGuard(object):
    def __init__(self):
        self.budget = rospy.get_param('~budget_sim_s', 900.0)
        self.run_dir = rospy.get_param('~run_dir', '')
        self.arm_time = rospy.get_param('~arm_after_sim_s', 0.0)
        self.t0 = None
        self.t_first_clock = None
        self.fired = False
        rospy.Subscriber('/clock', Clock, self.on_clock, queue_size=50)
        rospy.loginfo('budget_guard: budget %.1f sim-s, arming after %.1f sim-s',
                      self.budget, self.arm_time)

    def on_clock(self, msg):
        now = msg.clock.to_sec()
        if self.t_first_clock is None:
            self.t_first_clock = now
        if self.t0 is None and (now - self.t_first_clock) >= self.arm_time:
            self.t0 = now
            rospy.loginfo('budget_guard: armed at sim t=%.2f', now)
        if self.fired or self.t0 is None:
            return
        elapsed = now - self.t0
        if elapsed >= self.budget:
            self.fired = True
            self.write_marker(elapsed)
            rospy.logwarn('budget_guard: budget reached (%.1f sim-s) -> shutting down', elapsed)
            try:
                subprocess.call(['rosnode', 'kill', '-a'])
            except Exception as exc:                     # pragma: no cover
                rospy.logerr('budget_guard: rosnode kill failed: %s', exc)
            rospy.signal_shutdown('budget reached')

    def write_marker(self, elapsed):
        if not self.run_dir:
            return
        path = os.path.join(self.run_dir, 'budget_reached.txt')
        with open(path, 'w') as fh:
            fh.write('sim_elapsed=%.3f\n' % elapsed)


def main():
    rospy.init_node('budget_guard', anonymous=False)
    BudgetGuard()
    rospy.spin()


if __name__ == '__main__':
    main()
