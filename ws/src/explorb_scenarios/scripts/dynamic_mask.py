#!/usr/bin/env python3
"""Perception-only dynamic-pixel mask source (the single shared input of phase 1.2).

What it is, and what it is NOT
------------------------------
This node publishes `/orb_slam2_rgbd/dynamic_mask` (mono8, 255 = "do not extract
features here"). It is produced *only* from sensor data and the SLAM pose:

  1. the current depth image is back-projected to 3D with the camera intrinsics and
     the current SLAM pose (the camera/base extrinsic comes from tf);
  2. every point is re-projected into the previous depth frame *with the ego-motion
     compensated* (the previous frame's pose is used, so a static world produces
     zero residual);
  3. a pixel whose expected depth differs from the measured depth by more than a
     scale-dependent tolerance is flagged: the surface it belongs to moved
     relative to the map, i.e. it is dynamic (or a depth discontinuity edge, which
     the erosion+dilation below removes);
  4. the mask is grown by a small dilation because a bleeding depth edge is wider
     than the flagged core.

No Gazebo ground truth, no model_states, no scenario description is read anywhere in
this file: the simulator truth is reserved for the offline metrics (task book
constraint 4). `scenario_driver` moves the actors, but this node never asks it
anything.

It also writes `dynamic_objects.csv` (connected components read back as 3D blobs with
size / centre / speed from consecutive frames), which the C++ visual query consumes as
its `DynamicObject` list, and which is a legitimate online input because it is
measured, not given.

Both arms of the A/B run receive exactly this one mask via
`orb_slam2_ros/use_dynamic_mask`; the switch lives in
`explorb_scenarios/launch/move_base_slam.launch`.
"""
from __future__ import print_function

import csv
import math
import os
import threading

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import CameraInfo, Image
import tf

DEF = dict(
    rate=10.0,               # Hz of mask publication
    max_pose_age=0.5,        # s, refuse to build a mask from a stale pose
    key_history=8,           # depth frames kept for the ego-motion residual
    residual_m=0.08,         # m, base tolerance of the depth residual
    residual_rel=0.02,       # relative tolerance (depth error grows with range)
    min_range=0.3,           # m
    max_range=4.5,           # m
    morph_kernel=5,          # px
    dilate_iter=2,
    min_blob_px=120,         # mask blobs smaller than this are dropped as noise
    log_csv=True,
)


class DynamicMaskNode(object):
    def __init__(self):
        self.p = dict(DEF)
        for k in list(self.p.keys()):
            self.p[k] = rospy.get_param('~' + k, self.p[k])

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.intr = None
        self.T_base_cam = None
        self.hist = []          # list of (stamp, depth float32, x, y, yaw)
        self.pose = None        # (stamp, x, y, yaw)
        self.depth = None
        self.depth_stamp = None

        self.tf_listener = tf.TransformListener()

        self.run_dir = rospy.get_param('~run_dir', '')
        self.csv = None
        self.csv_w = None
        if self.run_dir and self.p['log_csv']:
            try:
                os.makedirs(os.path.join(self.run_dir, 'export'), exist_ok=True)
                path = os.path.join(self.run_dir, 'export', 'dynamic_objects.csv')
                self.csv = open(path, 'w', newline='')
                self.csv_w = csv.writer(self.csv)
                self.csv_w.writerow(['t', 'n_blobs', 'n_mask_px', 'n_blobs_written'])
                self.csv.flush()
            except Exception as e:
                rospy.logwarn('dynamic_mask: cannot write csv (%s)' % e)

        self.pub = rospy.Publisher('/orb_slam2_rgbd/dynamic_mask', Image, queue_size=2)
        # measured dynamic objects (centre / size / velocity, map frame) for the C++
        # visual query. Layout: [t_ref, n, (cx,cy,cz,sx,sy,sz,vx,vy,vz) * n]
        self.pub_objs = rospy.Publisher('/dynamic_objects', Float64MultiArray, queue_size=2)
        self.prev_blobs = {}      # label -> (t, world centre) for the velocity estimate
        rospy.Subscriber('/camera/depth_registered/image_raw', Image, self.on_depth, queue_size=2,
                         buff_size=2 ** 24)
        rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.on_info, queue_size=1)
        rospy.Subscriber('/orb_slam2_rgbd/camera_pose', PoseStamped, self.on_pose, queue_size=20)

        self.n_masks = 0
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(0.5, self.p['rate'])), self.on_timer)
        rospy.loginfo('dynamic_mask: started (residual %.3f m + %.1f%%, range %.1f-%.1f m)'
                      % (self.p['residual_m'], 100.0 * self.p['residual_rel'],
                         self.p['min_range'], self.p['max_range']))

    # ------------------------------------------------------------------ callbacks
    def on_info(self, msg):
        if self.intr is None:
            self.intr = dict(fx=msg.K[0], fy=msg.K[4], cx=msg.K[2], cy=msg.K[5],
                             width=msg.width, height=msg.height)
            rospy.loginfo('dynamic_mask: intrinsics fx=%.1f fy=%.1f cx=%.1f cy=%.1f %dx%d'
                          % (self.intr['fx'], self.intr['fy'], self.intr['cx'],
                             self.intr['cy'], self.intr['width'], self.intr['height']))

    def on_pose(self, msg):
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.pose = (msg.header.stamp.to_sec(), msg.pose.position.x,
                         msg.pose.position.y, yaw)

    def on_depth(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except CvBridgeError as e:
            rospy.logwarn_throttle(2.0, 'dynamic_mask: %s' % e)
            return
        with self.lock:
            self.depth = depth
            self.depth_stamp = msg.header.stamp.to_sec()

    def _base_to_cam(self):
        if self.T_base_cam is not None:
            return self.T_base_cam
        try:
            (t, q) = self.tf_listener.lookupTransform('base_footprint', 'camera_link_optical',
                                                      rospy.Time(0))
            self.T_base_cam = (np.array(t), np.array(q))
        except Exception:
            return None
        return self.T_base_cam

    # --------------------------------------------------------------------- mask
    def build_mask(self, depth, pose_prev, pose_cur):
        """Residual of the ego-motion-compensated depth, in pixels (uint8).

        Returns (mask, n_blobs, blobs) where blobs is a list of
        (world_centre(3), size(3)) for the accepted components.
        """
        fx, fy = self.intr['fx'], self.intr['fy']
        cx, cy = self.intr['cx'], self.intr['cy']
        H, W = depth.shape[:2]

        (t_bc, q_bc) = self._base_to_cam()
        # rotation matrix of the camera in the base frame (sophus-free quaternion->R)
        x, y, z, w = q_bc
        R_bc = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

        def pose_to_Rt(p_):
            yaw = p_[3]
            c, s = math.cos(yaw), math.sin(yaw)
            Rw = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
            return Rw, np.array([p_[1], p_[2], 0.0])

        Rw_c, tw_c = pose_to_Rt(pose_cur)
        Rw_p, tw_p = pose_to_Rt(pose_prev)
        R_cam_w_cur = Rw_c.dot(R_bc)
        R_cam_w_prev = Rw_p.dot(R_bc)
        t_cam_w_cur = Rw_c.dot(t_bc) + tw_c
        t_cam_w_prev = Rw_p.dot(t_bc) + tw_p

        # back-project the current depth into the current camera frame, then to world
        v, u = np.mgrid[0:H, 0:W]
        zc = depth
        valid = np.isfinite(zc) & (zc > self.p['min_range']) & (zc < self.p['max_range'])
        if not np.any(valid):
            return None, 0, []

        xc = (u - cx) * zc / fx
        yc = (v - cy) * zc / fy
        P_cam = np.stack([xc, yc, zc], axis=-1)
        P_world = P_cam.dot(R_cam_w_cur.T) + t_cam_w_cur[None, None, :]

        # express in the previous camera frame and project
        P_prev = (P_world - t_cam_w_prev[None, None, :]).dot(R_cam_w_prev)
        zp = P_prev[..., 2]
        ok = valid & (zp > 1e-3)
        up = np.full((H, W), -1, dtype=np.int32)
        vp = np.full((H, W), -1, dtype=np.int32)
        up[ok] = np.round(fx * P_prev[..., 0][ok] / zp[ok] + cx).astype(np.int32)
        vp[ok] = np.round(fy * P_prev[..., 1][ok] / zp[ok] + cy).astype(np.int32)
        inb = ok & (up >= 0) & (up < W) & (vp >= 0) & (vp < H)

        tol = self.p['residual_m'] + self.p['residual_rel'] * zp
        resid = np.zeros((H, W), dtype=np.float32)
        resid[inb] = np.abs(zp[inb] - depth[vp[inb], up[inb]])
        flag = inb & (resid > tol)

        mask = (flag.astype(np.uint8)) * 255
        k = np.ones((int(self.p['morph_kernel']), int(self.p['morph_kernel'])), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.dilate(mask, k, iterations=int(self.p['dilate_iter']))

        # drop specks: keep only components with enough pixels
        n, labels, stats, _ = cv2.connectedComponentsWithStats((mask > 0).astype(np.uint8), 8)
        keep = np.zeros_like(mask)
        blobs = 0
        out = []
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] < self.p['min_blob_px']:
                continue
            keep[labels == i] = 255
            blobs += 1
            sel = (labels == i) & valid
            if not np.any(sel):
                continue
            # 3D centre / extent of the blob, in the map frame
            pts = P_world[sel]
            centre = pts.mean(axis=0)
            lo, hi = pts.min(axis=0), pts.max(axis=0)
            size = np.maximum(hi - lo, np.array([0.15, 0.15, 0.30]))
            out.append((centre, size))
        return keep, blobs, out

    def on_timer(self, _evt):
        with self.lock:
            depth = None if self.depth is None else self.depth.copy()
            pose = self.pose
            d_stamp = self.depth_stamp
        if depth is None or pose is None or self.intr is None:
            return
        if self._base_to_cam() is None:
            return
        if abs(rospy.get_time() - pose[0]) > self.p['max_pose_age']:
            return

        # ego-motion reference: the most recent stored frame that is far enough in time
        ref = None
        for (stamp, _d, px, py, pyaw) in reversed(self.hist):
            if abs(pose[0] - stamp) > 0.05:
                ref = (stamp, px, py, pyaw)
                break

        with self.lock:
            self.hist.append((d_stamp if d_stamp else rospy.get_time(), depth,
                              pose[1], pose[2], pose[3]))
            if len(self.hist) > int(self.p['key_history']):
                self.hist.pop(0)

        blobs_geo = []
        if ref is None:
            # first mask: nothing to compare against yet, publish an empty mask so the
            # frontend sees a consistent "mask present but empty" state
            mask = np.zeros(depth.shape[:2], np.uint8)
            blobs = 0
        else:
            mask, blobs, blobs_geo = self.build_mask(depth, ref, (pose[0], pose[1], pose[2], pose[3]))
            if mask is None:
                return

        try:
            msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        except CvBridgeError as e:
            rospy.logwarn_throttle(2.0, 'dynamic_mask: %s' % e)
            return
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'camera_link_optical'
        self.pub.publish(msg)

        # ---- measured dynamic objects with a constant-velocity estimate ----------
        t_now = rospy.get_time()
        objs = Float64MultiArray()
        rows = []
        new_prev = {}
        for bi, (centre, size) in enumerate(blobs_geo):
            vel = [0.0, 0.0, 0.0]
            best = None
            for lab, (tp, cp) in self.prev_blobs.items():
                d = float(np.linalg.norm(np.asarray(cp) - centre))
                if best is None or d < best[1]:
                    best = (lab, d)
            if best is not None and best[1] < 1.2 and (t_now - self.prev_blobs[best[0]][0]) > 1e-3:
                tp, cp = self.prev_blobs[best[0]]
                dt = max(1e-3, t_now - tp)
                vel = list((centre - np.asarray(cp)) / dt)
            new_prev[bi] = (t_now, centre.copy())
            rows.extend([float(centre[0]), float(centre[1]), float(centre[2]),
                         float(size[0]), float(size[1]), float(size[2]),
                         float(vel[0]), float(vel[1]), float(vel[2])])
        self.prev_blobs = new_prev
        objs.data = [t_now, float(len(blobs_geo))] + rows
        self.pub_objs.publish(objs)

        self.n_masks += 1
        if self.csv_w is not None:
            self.csv_w.writerow([rospy.get_time(), blobs, int(np.count_nonzero(mask)), 0])
            self.csv.flush()
        if self.n_masks % 50 == 0:
            rospy.loginfo('dynamic_mask: %d masks published (last: %d px, %d blobs)'
                          % (self.n_masks, int(np.count_nonzero(mask)), blobs))


def main():
    rospy.init_node('dynamic_mask')
    node = DynamicMaskNode()
    rospy.on_shutdown(lambda: node.csv and node.csv.close())
    rospy.loginfo('dynamic_mask: ready')
    rospy.spin()


if __name__ == '__main__':
    main()
