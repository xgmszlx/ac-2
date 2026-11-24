#!/usr/bin/env python3
"""Measure the simulation real-time factor of whatever is running.

Subscribes to /clock, reports the achieved RTF (simulated seconds per wall
second) over an observation window after the clock has started moving.

Usage:  rosrun explorb_scenarios rtf_probe.py _window:=20.0
"""
from __future__ import print_function

import time

import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import OccupancyGrid


class RTFProbe(object):
    def __init__(self):
        self.window = float(rospy.get_param('~window', 20.0))
        self.sim = None
        self.t_wall = None
        self.samples = []
        rospy.Subscriber('/clock', Clock, self.cb, queue_size=100)
        # A sensor in Gazebo is only rendered while someone subscribes, so the
        # probe must hold the same subscriptions the real stack holds.
        self.cam_events = 0
        self.scan_events = 0
        if rospy.get_param('~subscribe_camera', True):
            rospy.Subscriber('/robot_1/camera/rgb/image_raw', Image, self.on_img, queue_size=2)
            rospy.Subscriber('/robot_1/camera/depth_registered/image_raw', Image,
                             self.on_img, queue_size=2)
            rospy.Subscriber('/robot_1/base_scan', LaserScan, self.on_scan, queue_size=2)

    def on_img(self, _m):
        self.cam_events += 1

    def on_scan(self, _m):
        self.scan_events += 1

    def cb(self, msg):
        sim = msg.clock.to_sec()
        if sim <= 0.0:
            return
        now = time.time()
        if self.sim is None:
            self.sim, self.t_wall = sim, now
            return
        if sim <= self.sim:
            return
        self.samples.append((sim, now))
        self.sim, self.t_wall = sim, now

    def run(self):
        while not rospy.is_shutdown() and not self.samples:
            time.sleep(0.2)
        t0 = self.samples[0][1]
        while not rospy.is_shutdown() and time.time() - t0 < self.window:
            time.sleep(0.2)
        if len(self.samples) < 2:
            print('RTF_PROBE no clock')
            return
        t_start = t0
        seg = [(s, w) for (s, w) in self.samples if w >= t_start]
        s0, w0 = seg[0]
        s1, w1 = seg[-1]
        rtf = (s1 - s0) / max(w1 - w0, 1e-6)
        hz = len(seg) / max(w1 - w0, 1e-6)
        print('RTF_PROBE sim=%.2fs wall=%.2fs rtf=%.3f clock_hz=%.1f '
              'cam_frames=%d scan=%d'
              % (s1 - s0, w1 - w0, rtf, hz, self.cam_events, self.scan_events))


if __name__ == '__main__':
    rospy.init_node('rtf_probe', anonymous=True)
    RTFProbe().run()
