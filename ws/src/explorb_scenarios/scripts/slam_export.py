#!/usr/bin/env python3
"""Read-only export of everything the visual query and the offline metrics need.

Task book phase 1.3 asks for the read-only exports explicitly:

    static map points (id, 3D, observation statistics)
    SLAM pose + tracking state
    camera intrinsics + extrinsics
    static occlusion map
    dynamic object tracking output

All of it is already published by the *unchanged* ORB-SLAM frontend, so this node
subscribes and dumps; it never calls into the tracker and never modifies a single
keyframe, match, local-map or BA decision. That is deliberate: the constraint "no
changes to keyframe insertion / matching acceptance / local mapping / BA" is
easiest to honour by not touching the C++ at all.

Writes, under <run_dir>/export/:
    map_points.csv       t, id, x, y, z, n_obs          (0.2 Hz snapshot of the map)
    keyframes.csv        t, id, x, y, z, qx, qy, qz, qw (0.2 Hz)
    edges.csv            t, id_a, id_b, trace_H         (0.2 Hz)
    poses.csv            t, state, x, y, z, yaw, qx..qw (10 Hz)
    camera.json          intrinsics + base<->camera extrinsic (rewritten on change)
    static_occupancy.npz width, height, resolution, origin, data (every 5 s)
    manifest.json        what was exported, when, by which run -- and the explicit
                         statement that no simulator ground truth enters any of it

The dynamic-object export is produced by dynamic_mask.py (it owns that measurement);
this file only records the mask statistics it can see.
"""
from __future__ import print_function

import csv
import json
import os

import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64MultiArray
import tf

try:
    from orb_slam2_ros.msg import ORBState
except Exception:                                    # pragma: no cover
    ORBState = None


class SlamExport(object):
    def __init__(self):
        self.run_dir = rospy.get_param('~run_dir', '')
        self.rate_hz = float(rospy.get_param('~rate', 5.0))
        self.slam_ns = rospy.get_param('~slam_ns', '/orb_slam2_rgbd')
        self.camera_type = rospy.get_param('~camera_type', 'rgbd')
        self.out = os.path.join(self.run_dir, 'export') if self.run_dir else ''
        if self.out:
            os.makedirs(self.out, exist_ok=True)

        self.w = {}
        self.vertices = []       # (id, x, y, z, qx, qy, qz, qw)
        self.points = []         # (id, x, y, z, n_obs)
        self.edges = []          # (id_a, id_b, trace)
        self.pose = None
        self.state = None
        self.caminfo = None
        self.grid = None
        self.last_export = 0.0
        self.n_cycles = 0
        self.tf_listener = tf.TransformListener()

        if self.out:
            self._open('poses', ['t', 'state', 'x', 'y', 'z', 'yaw', 'qx', 'qy', 'qz', 'qw'])
            self._open('map_points', ['t', 'id', 'x', 'y', 'z', 'n_obs'])
            self._open('keyframes', ['t', 'id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            self._open('edges', ['t', 'id_a', 'id_b', 'trace_H'])

        rospy.Subscriber(self.slam_ns + '/vertex_list', Float64MultiArray, self.on_vertices, queue_size=1)
        rospy.Subscriber(self.slam_ns + '/edge_list', Float64MultiArray, self.on_edges, queue_size=1)
        rospy.Subscriber(self.slam_ns + '/point_list', Float64MultiArray, self.on_points, queue_size=1)
        rospy.Subscriber(self.slam_ns + '/camera_pose', PoseStamped, self.on_pose, queue_size=20)
        if ORBState is not None:
            rospy.Subscriber(self.slam_ns + '/info/state', ORBState, self.on_state, queue_size=10)
        rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.on_caminfo, queue_size=1)
        rospy.Subscriber('/gridmapper/rectified_map', OccupancyGrid, self.on_grid, queue_size=2)

        rospy.Timer(rospy.Duration(1.0 / max(0.5, self.rate_hz)), self.on_timer)
        rospy.loginfo('slam_export: writing to %s' % (self.out or '(disabled)'))
        if self.out:
            with open(os.path.join(self.out, 'manifest.json'), 'w') as f:
                json.dump({
                    'node': 'slam_export',
                    'read_only': True,
                    'statement': ('All exported quantities are read from published ORB-SLAM '
                                  'topics. No keyframe insertion, matching policy, local mapping '
                                  'or BA behaviour is modified, and no simulator ground truth is '
                                  'used as an online input anywhere in this line.'),
                    'ground_truth_online': False,
                    'camera_type': self.camera_type,
                    'started_at': rospy.get_time(),
                }, f, indent=2)

    # ------------------------------------------------------------------ storage
    def _open(self, name, cols):
        f = open(os.path.join(self.out, name + '.csv'), 'w', newline='')
        w = csv.writer(f)
        w.writerow(cols)
        f.flush()
        self.w[name] = (f, w)

    def _row(self, name, row):
        if name in self.w:
            _, w = self.w[name]
            w.writerow(row)
            self.w[name][0].flush()

    # --------------------------------------------------------------- callbacks
    def on_vertices(self, msg):
        d = msg.data
        n = len(d) // 8
        out = []
        for i in range(n):
            b = i * 8
            out.append((int(d[b]), d[b + 5], d[b + 6], d[b + 7],
                        d[b + 1], d[b + 2], d[b + 3], d[b + 4]))
        self.vertices = out

    def on_edges(self, msg):
        d = msg.data
        n = len(d) // 38
        out = []
        for i in range(n):
            b = i * 38
            tr = sum(float(d[b + 2 + k * 7]) for k in range(6))   # diagonal of the 6x6 H
            out.append((int(d[b]), int(d[b + 1]), tr))
        self.edges = out

    def on_points(self, msg):
        d = msg.data
        out = []
        i = 0
        n = len(d)
        while i < n:
            mp_id = int(d[i])
            if i + 3 >= n:
                break
            x, y, z = float(d[i + 1]), float(d[i + 2]), float(d[i + 3])
            j = i + 4
            n_obs = 0
            # the observation list is variable length; a negative or non-integer tail
            # would mean a layout change, in which case we keep what we have
            while j < n and float(d[j]) >= 0.0 and float(d[j]) == int(d[j]):
                n_obs += 1
                j += 1
            out.append((mp_id, x, y, z, n_obs))
            i = j
        self.points = out

    def on_pose(self, msg):
        q = msg.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.header.stamp.to_sec(), msg.pose.position.x, msg.pose.position.y,
                     msg.pose.position.z, float(yaw), q.x, q.y, q.z, q.w)

    def on_state(self, msg):
        self.state = getattr(msg, 'state', None)

    def on_caminfo(self, msg):
        if self.caminfo is None:
            self.caminfo = msg

    def on_grid(self, msg):
        self.grid = msg

    # ------------------------------------------------------------------- output
    def write_camera(self):
        if self.caminfo is None or not self.out:
            return
        k = self.caminfo.K
        doc = dict(
            width=self.caminfo.width, height=self.caminfo.height,
            fx=k[0], fy=k[4], cx=k[2], cy=k[5],
            distortion_model=self.caminfo.distortion_model,
            D=list(self.caminfo.D),
            frame_id=self.caminfo.header.frame_id,
        )
        try:
            (t, q) = self.tf_listener.lookupTransform('base_footprint', 'camera_link_optical',
                                                      rospy.Time(0))
            doc['T_base_camera'] = dict(translation=list(t), quaternion=list(q),
                                        convention='camera_link_optical expressed in base_footprint')
            x, y, z, w = q
            doc['R_base_camera'] = [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        except Exception as e:
            doc['T_base_camera'] = 'unavailable: %s' % e
        with open(os.path.join(self.out, 'camera.json'), 'w') as f:
            json.dump(doc, f, indent=2)

    def on_timer(self, _evt):
        t = rospy.get_time()
        self.n_cycles += 1
        if self.pose is not None:
            self._row('poses', [self.pose[0], self.state if self.state is not None else -1] +
                      list(self.pose[1:]))
        now = rospy.get_time()
        if now - self.last_export < 5.0:
            return
        self.last_export = now
        self.write_camera()
        if not self.out:
            return
        for (kid, x, y, z, qx, qy, qz, qw) in self.vertices:
            self._row('keyframes', [t, kid, x, y, z, qx, qy, qz, qw])
        for (mp_id, x, y, z, n_obs) in self.points:
            self._row('map_points', [t, mp_id, x, y, z, n_obs])
        for (a, b, tr) in self.edges:
            self._row('edges', [t, a, b, tr])
        if self.grid is not None:
            g = self.grid
            np.savez_compressed(
                os.path.join(self.out, 'static_occupancy.npz'),
                width=g.info.width, height=g.info.height,
                resolution=g.info.resolution,
                origin=np.array([g.info.origin.position.x, g.info.origin.position.y,
                                 g.info.origin.position.z]),
                frame_id=g.header.frame_id,
                data=np.asarray(g.data, dtype=np.int8))
        rospy.loginfo('slam_export: kf=%d mp=%d edges=%d grid=%s pose=%s'
                      % (len(self.vertices), len(self.points), len(self.edges),
                         'yes' if self.grid is not None else 'no',
                         'yes' if self.pose is not None else 'no'))

    def close(self):
        for (f, _w) in self.w.values():
            try:
                f.close()
            except Exception:
                pass


def main():
    rospy.init_node('slam_export')
    node = SlamExport()
    rospy.on_shutdown(node.close)
    rospy.spin()


if __name__ == '__main__':
    main()
