#!/usr/bin/env python3
"""Deterministic dynamic-actor driver for the localization-observability line.

Design rules that make the comparison honest:

  * The actors follow *fixed spatial routes* with *fixed phase offsets drawn from
    the task seed*. They do not look at the robot, at the planner, or at the SLAM
    state. Two arms running the same (scenario, seed) therefore face exactly the
    same world dynamics as a function of simulation time; only the robot's own
    decisions differ.

  * Everything is a function of the simulation clock (`/clock`), never of wall
    time, so a slower machine does not produce a different scenario.

  * `visual_*` actors are visual-only: SDF without <collision> geometry, so the
    Hokuyo/Gazebo laser used by the move_base costmaps cannot see them and they
    cannot push the robot. They occlude *camera* structure only -> this isolates
    the localization (observability) channel.

  * `blocker_*` actors carry collision geometry and are laser-visible: they are
    physical obstacles, so a nominal costmap-based planner must detour around them
    -> this isolates the obstacle-avoidance channel.

Outputs written to <run_dir>:
  actors.csv   t, name, x, y, z, yaw   (one row per actor per control tick)
  actors_meta.json  the resolved schedule parameters (seed, speeds, phases)
"""
from __future__ import print_function

import csv
import json
import math
import os
import random

import rospy

try:
    from gazebo_msgs.srv import SpawnModel, SetModelState, GetWorldProperties
    from gazebo_msgs.msg import ModelState
    from geometry_msgs.msg import Pose, Twist
    from rosgraph_msgs.msg import Clock
except ImportError as exc:                                # pragma: no cover
    print('scenario_driver: gazebo_msgs unavailable: %s' % exc)
    raise


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def visual_panel_sdf(name, w, h, t, color):
    """Opaque, feature-poor, laser-invisible occluder (no <collision>)."""
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>{w} {t} {h}</size></box></geometry>
        <material><ambient>{r} {g} {b} 1</ambient>
                  <diffuse>{r} {g} {b} 1</diffuse>
                  <specular>0.05 0.05 0.05 1</specular></material>
      </visual>
    </link>
  </model>
</sdf>
""".format(name=name, w=w, h=h, t=t, r=color[0], g=color[1], b=color[2])


def blocker_sdf(name, w, h, t, color):
    """Laser-visible, physically blocking obstacle."""
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>{w} {t} {h}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{w} {t} {h}</size></box></geometry>
        <material><ambient>{r} {g} {b} 1</ambient>
                  <diffuse>{r} {g} {b} 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
""".format(name=name, w=w, h=h, t=t, r=color[0], g=color[1], b=color[2])


class Route(object):
    """Piecewise-linear closed route; position is a function of sim time."""

    def __init__(self, waypoints, speed, phase_s, dwell_s=0.0):
        self.waypoints = waypoints
        self.speed = speed
        self.phase = phase_s
        self.dwell = dwell_s
        self.lengths = []
        total = 0.0
        for i in range(len(waypoints)):
            a = waypoints[i]
            b = waypoints[(i + 1) % len(waypoints)]
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            self.lengths.append(seg)
            total += seg
        self.total = total
        self.period = total / speed + dwell_s * len(waypoints)

    def at(self, t):
        tt = (t + self.phase) % self.period
        # dwell model: each waypoint consumes dwell_s, motion consumes length/speed
        acc = 0.0
        for i, seg in enumerate(self.lengths):
            seg_time = seg / self.speed
            if tt < acc + self.dwell:
                a = self.waypoints[i]
                return (a[0], a[1], math.atan2(0.0, 0.0))
            acc += self.dwell
            if tt < acc + seg_time:
                a = self.waypoints[i]
                b = self.waypoints[(i + 1) % len(self.waypoints)]
                frac = (tt - acc) / max(seg_time, 1e-9)
                return (a[0] + frac * (b[0] - a[0]),
                        a[1] + frac * (b[1] - a[1]),
                        math.atan2(b[1] - a[1], b[0] - a[0]))
            acc += seg_time
        a = self.waypoints[-1]
        return (a[0], a[1], 0.0)


class ScenarioDriver(object):
    def __init__(self):
        self.scenario = rospy.get_param('~scenario', 'static')
        self.seed = int(rospy.get_param('~seed', 0))
        self.run_dir = rospy.get_param('~run_dir', '')
        self.rate_hz = float(rospy.get_param('~rate', 10.0))
        self.spawn_z_panel = float(rospy.get_param('~panel_center_z', 0.95))

        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=120.0)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=120.0)
        self.spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.actors = []
        self.t0 = None
        self.csv = None
        self.writer = None
        self.meta = {'scenario': self.scenario, 'seed': self.seed, 'actors': []}

        rospy.sleep(2.0)
        self.build()
        self.open_log()
        rospy.Subscriber('/clock', Clock, self.on_clock, queue_size=50)
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.tick)
        rospy.loginfo('scenario_driver: %s with %d actors (seed=%d)',
                      self.scenario, len(self.actors), self.seed)

    # ------------------------------------------------------------------ setup
    def build(self):
        rng = random.Random(self.seed)

        if self.scenario in ('visual', 'both'):
            # Panels sweep across the two main circulation corridors of the AWS
            # house at floor level. Routes are fixed in world coordinates and
            # independent of the robot.
            specs = [
                ('visual_panel_a', [(1.6, -2.2), (1.6, 2.4)], 0.42, 1.9, 1.1, (0.93, 0.93, 0.90)),
                ('visual_panel_b', [(-2.0, 1.2), (2.6, 1.2)], 0.36, 2.0, 1.1, (0.90, 0.91, 0.94)),
            ]
            for name, wps, speed, w, h, col in specs:
                phase = rng.uniform(0.0, 6.0)
                route = Route(wps, speed, phase)
                self.add_actor(name, visual_panel_sdf(name, w, h, 0.10, col),
                               route, z=self.spawn_z_panel, kind='visual')

        if self.scenario in ('blocker', 'both'):
            specs = [
                ('blocker_cart_a', [(0.9, -1.9), (0.9, 1.9)], 0.30, 0.55, 0.9, 0.45, (0.35, 0.40, 0.55)),
                ('blocker_cart_b', [(-1.6, 0.6), (2.2, 0.6)], 0.26, 0.55, 0.9, 0.45, (0.45, 0.35, 0.30)),
            ]
            for name, wps, speed, w, h, t, col in specs:
                phase = rng.uniform(0.0, 6.0)
                route = Route(wps, speed, phase)
                self.add_actor(name, blocker_sdf(name, w, h, t, col),
                               route, z=h * 0.5, kind='blocker')

    def add_actor(self, name, sdf, route, z, kind):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = 0.0, 0.0, z
        pose.orientation.w = 1.0
        try:
            self.spawn(name, sdf, '', pose, 'world')
        except Exception as exc:
            rospy.logwarn('scenario_driver: spawn %s failed: %s', name, exc)
        self.actors.append({'name': name, 'route': route, 'z': z, 'kind': kind})
        self.meta['actors'].append({'name': name, 'kind': kind,
                                    'speed': route.speed, 'phase_s': route.phase,
                                    'waypoints': route.waypoints, 'z': z,
                                    'period_s': route.period})

    # ------------------------------------------------------------------- loop
    def open_log(self):
        if not self.run_dir:
            return
        if not os.path.isdir(self.run_dir):
            os.makedirs(self.run_dir)
        self.csv = open(os.path.join(self.run_dir, 'actors.csv'), 'w', newline='')
        self.writer = csv.writer(self.csv)
        self.writer.writerow(['t', 'name', 'kind', 'x', 'y', 'z', 'yaw'])
        with open(os.path.join(self.run_dir, 'actors_meta.json'), 'w') as fh:
            json.dump(self.meta, fh, indent=2, sort_keys=True)

    def on_clock(self, msg):
        if self.t0 is None:
            self.t0 = msg.clock.to_sec()

    def tick(self, _ev):
        if self.t0 is None:
            return
        t = rospy.get_time() - self.t0
        for act in self.actors:
            x, y, yaw = act['route'].at(t)
            ms = ModelState()
            ms.model_name = act['name']
            ms.reference_frame = 'world'
            ms.pose.position.x = x
            ms.pose.position.y = y
            ms.pose.position.z = act['z']
            qx, qy, qz, qw = yaw_quat(yaw)
            ms.pose.orientation.x = qx
            ms.pose.orientation.y = qy
            ms.pose.orientation.z = qz
            ms.pose.orientation.w = qw
            ms.twist = Twist()
            try:
                self.set_state(ms)
            except Exception:
                pass
            if self.writer is not None:
                self.writer.writerow([t, act['name'], act['kind'],
                                      round(x, 4), round(y, 4), act['z'], round(yaw, 4)])
        if self.csv is not None:
            self.csv.flush()


def main():
    rospy.init_node('scenario_driver', anonymous=False)
    ScenarioDriver()
    rospy.spin()


if __name__ == '__main__':
    main()
