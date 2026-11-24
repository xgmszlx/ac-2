#!/usr/bin/env python3
"""Geometric localization-observability model for the ExplORB-SLAM line.

What this module adds to the author baseline
--------------------------------------------
The baseline (``Map.hallucinateHessianFromPose``) already builds a per-pose
information matrix from map points that fall inside the camera frustum, but it

  * mixes pixels^2/m^2 and pixels^2/rad^2 blocks without a common scale, so the
    "amount of information" of a hypothetical pose is not comparable across
    poses or against any requirement;
  * uses raw map-point *counts* through ``covisibleKeyFrames`` thresholds
    (150 / 400 points), i.e. feature quantity rather than geometry;
  * has no notion of *when* the pose is reached, so a pose that is fine now and
    blind in 4 s looks identical to one that is fine throughout.

This module supplies, for an executable plan:

  * a per-pose 6x6 information matrix built from the projection Jacobians of the
    map points that are actually visible with usable observation quality;
  * a *dimensionless* scalar, ``constraint_strength`` = 1 / max_i (C_ii / s_i^2),
    i.e. the weakest normalised marginal standard deviation of the pose. s_i are
    the required translational (m) and rotational (rad) accuracies, which is what
    unifies the two blocks instead of adding metres to radians;
  * a bounded information deficit relative to the *reference* strength that the
    same environment currently provides (a running median over measured
    keyframes), so no absolute calibration of the frontend is invented;
  * an explicit exploration credit for unknown space (the "new feature prior"),
    so that approaching unmapped structure is not punished for lacking old map
    points.

Everything here uses only the map / graph / grid data that the unchanged
ORB-SLAM frontend publishes plus the executable plan produced by move_base. No
future image, no future trajectory and no simulator ground truth is used.
"""
from __future__ import print_function

import numpy as np

# ---------------------------------------------------------------------------
# defaults; overridable from the Controller via rosparams
# ---------------------------------------------------------------------------
DEF = dict(
    max_visual_range=3.0,     # m, same order as the baseline's frustum limit
    min_visual_range=0.15,    # m, do not trust points closer than this
    sigma_t_req=0.05,         # m   required translational accuracy  (unified scale)
    sigma_r_req=0.02,         # rad required rotational accuracy      (unified scale)
    range_decay=1.6,          # m   observation-quality length scale
    deficit_cap=1.5,          # nat, bound on a single pose's deficit
    ref_percentile=50.0,      # reference strength = this percentile over KFs
    explore_gamma=1.0,        # exponent of the exploration-credit discount
    min_points_for_rank=12,   # below this the pose is treated as rank deficient
    singular_floor=1e-9,
)


def _skew(v):
    """3x3 skew-symmetric matrices for a stack of vectors (N,3) -> (N,3,3)."""
    S = np.zeros((v.shape[0], 3, 3))
    S[:, 0, 1] = -v[:, 2]
    S[:, 0, 2] = v[:, 1]
    S[:, 1, 0] = v[:, 2]
    S[:, 1, 2] = -v[:, 0]
    S[:, 2, 0] = -v[:, 1]
    S[:, 2, 1] = v[:, 0]
    return S


class CameraModel(object):
    """Pinhole intrinsics + the camera/base extrinsic held by ``Map``."""

    def __init__(self, fx, fy, cx, cy, width, height, R_cb, t_cb):
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.W = float(width)
        self.H = float(height)
        # Same numbers as Map.T_camera_base: from the tf lookup
        # (base_footprint <- camera_link_optical) the pose maps camera -> base.
        self.R_cb = np.asarray(R_cb, dtype=float).reshape(3, 3)
        self.t_cb = np.asarray(t_cb, dtype=float).reshape(3)

    def to_camera(self, X_cb, R_base, t_base):
        """Map points (in the baseline's cb frame) -> candidate camera frame.

        p_cam = R_cb^T ( R_base^T (X_cb - t_base) - t_cb )
        which is exactly ``(T_camera_base^-1 * T_map_base^-1) * p3D`` in Map.py.
        """
        pb = (X_cb - t_base[None, :]).dot(R_base)            # R_base^T (x - t)
        pc = (pb - t_cb[None, :]).dot(self.R_cb)             # R_cb^T  (pb - t_cb)
        return pc


class ObservabilityEstimator(object):
    """Per-pose and per-path geometric observability from the current map."""

    def __init__(self, params=None):
        self.p = dict(DEF)
        if params:
            self.p.update({k: v for k, v in params.items() if k in self.p})
        # unified metric: equal scale for the translation and rotation blocks
        self.s = np.array([self.p['sigma_t_req']] * 3 + [self.p['sigma_r_req']] * 3)

        self._X = None          # (N,3) map points in the cb frame
        self._ids = None
        self._stamp = None
        self.ref_strength = None

    # -- map ---------------------------------------------------------------
    def set_map(self, ids, X_cb):
        if ids is None or len(ids) == 0:
            self._X = np.zeros((0, 3))
            self._ids = np.zeros((0,), dtype=np.int64)
            return
        self._X = np.asarray(X_cb, dtype=float).reshape(-1, 3)
        self._ids = np.asarray(ids, dtype=np.int64)

    def n_points(self):
        return 0 if self._X is None else self._X.shape[0]

    # -- single pose -------------------------------------------------------
    def pose_eval(self, cam, R_base, t_base, min_range=None, weight_mode='range'):
        """Information of one hypothetical base pose.

        Returns a dict with
            n_vis      number of map points inside the image and range
            H          (6x6) information matrix (baseline-compatible Jacobians)
            Hn         (6x6) normalised information (unified t/r scale)
            strength   1 / max_i (C_ii / s_i^2)    (dimensionless)
            marginal_std  sqrt(C_ii) per axis
        """
        out = dict(n_vis=0, H=np.zeros((6, 6)), Hn=np.zeros((6, 6)),
                   strength=0.0, marginal_std=np.full(6, np.inf))
        if self._X is None or self._X.shape[0] == 0:
            return out

        d_max = self.p['max_visual_range'] if min_range is None else min_range
        d_min = self.p['min_visual_range']

        # cheap range gate around the candidate pose (map points are in cb frame,
        # the pose translation is in the same frame family the baseline uses)
        pc = cam.to_camera(self._X, R_base, t_base)
        z = pc[:, 2]
        rng = np.linalg.norm(pc, axis=1)
        ok = (z > d_min) & (z < d_max) & (rng <= d_max)
        if not np.any(ok):
            return out

        pc = pc[ok]
        idx = np.nonzero(ok)[0]
        z = pc[:, 2]
        u = cam.fx * pc[:, 0] / z + cam.cx
        v = cam.fy * pc[:, 1] / z + cam.cy
        inside = (u >= 0.0) & (u < cam.W) & (v >= 0.0) & (v < cam.H)
        if not np.any(inside):
            return out

        pc = pc[inside]
        idx = idx[inside]
        u, v, z = u[inside], v[inside], z[inside]
        rng = rng[ok][inside]
        n = pc.shape[0]

        # ---- observation-quality weight (NOT a feature count) -------------
        if weight_mode == 'range':
            # ORB scale-space: the expected number of matchable bits of a patch
            # decays with range because the same 3D structure occupies fewer
            # pixels; a bounded exponential is the simplest defensible model.
            w = np.exp(-(rng - d_min) / self.p['range_decay'])
            # points projected onto the very border of the image contribute
            # almost no usable 2-D constraint
            border = np.minimum(u, cam.W - 1.0 - u) / cam.W
            border = np.minimum(border, np.minimum(v, cam.H - 1.0 - v) / cam.H)
            w *= np.clip(4.0 * border, 0.15, 1.0)
        else:
            w = np.ones(n)

        # ---- projection Jacobians (same form as Map.hallucinateHessian...) --
        Jp = np.zeros((n, 2, 3))
        Jp[:, 0, 0] = cam.fx / z
        Jp[:, 0, 2] = -cam.fx * pc[:, 0] / (z * z)
        Jp[:, 1, 1] = cam.fy / z
        Jp[:, 1, 2] = -cam.fy * pc[:, 1] / (z * z)

        xw = self._X[idx]                       # cb-frame coordinates, as baseline
        S = _skew(xw)
        I3 = np.broadcast_to(np.eye(3), (n, 3, 3))
        Jse3 = np.concatenate([I3, -S], axis=2)  # (n,3,6): [I | -[x]_x]
        J = -np.einsum('nij,njk->nik', Jp, Jse3)

        Jw = J * np.sqrt(w)[:, None, None]
        H = np.einsum('nki,nkj->ij', Jw, Jw)

        out['n_vis'] = int(n)
        out['H'] = H
        out.update(self._analyse(H, n))
        return out

    # -- analysis of one information matrix --------------------------------
    def _analyse(self, H, n_pts):
        s = self.s
        Hn = H * np.outer(1.0 / s, 1.0 / s)      # dimensionless information
        # regularised inverse: an unobservable direction must read as "very
        # uncertain", not as a numerical explosion
        evals, evecs = np.linalg.eigh(Hn)
        floor = self.p['singular_floor']
        evals_c = np.maximum(evals, floor)
        Cn = (evecs / evals_c).dot(evecs.T)
        Cn = 0.5 * (Cn + Cn.T)
        marginal_std = np.sqrt(np.maximum(np.diag(Cn), 0.0))   # in units of s_i
        # rank deficiency: with few points the weakest direction is not meaningful
        if n_pts < self.p['min_points_for_rank']:
            marginal_std = marginal_std + (self.p['min_points_for_rank'] - n_pts) * 10.0
        worst = float(np.max(marginal_std))
        strength = 0.0 if not np.isfinite(worst) or worst <= 0 else 1.0 / worst
        return dict(Hn=Hn, strength=strength, marginal_std=marginal_std)

    # -- reference strength -------------------------------------------------
    def update_reference(self, strengths):
        vals = [x for x in strengths if x is not None and np.isfinite(x) and x > 0]
        if len(vals) >= 5:
            self.ref_strength = float(np.percentile(vals, self.p['ref_percentile']))
        return self.ref_strength

    def deficit(self, strength, rho_explore, ref=None):
        """Bounded information deficit of one pose, in nats.

        d = clip( log(ref / strength_eff), 0, cap ) * (1 - rho_explore)^gamma

        strength_eff already contains the occlusion discount applied by the
        caller (background constraints hidden by dynamic structure).  The
        exploration credit rho_explore discounts the deficit where the camera
        looks at unmapped structure, because new features (not old map points)
        are what carry the constraint there.
        """
        ref = self.ref_strength if ref is None else ref
        if ref is None or ref <= 0:
            return 0.0
        if strength is None or strength <= 0:
            d = self.p['deficit_cap']
        else:
            d = float(np.log(ref / strength))
        d = min(max(d, 0.0), self.p['deficit_cap'])
        rho = min(max(float(rho_explore), 0.0), 1.0)
        return float(d * (1.0 - rho) ** self.p['explore_gamma'])


class PathSampler(object):
    """Turns the executable global plan into poses with arrival times."""

    def __init__(self, spacing=0.45, v_nominal=0.22, v_min=0.08,
                 settle_time=0.0):
        self.spacing = float(spacing)
        self.v_nominal = float(v_nominal)
        self.v_min = float(v_min)
        self.settle = float(settle_time)

    def sample(self, plan_poses, start_xy=None):
        """plan_poses: list of geometry_msgs/PoseStamped from move_base make_plan.

        Returns (xy[N,2], yaw[N], tau[N]) where tau is the *arrival* time in
        seconds from now, accumulated with a simple trapezoidal speed profile
        clipped by the nominal and minimum speeds. The yaw at each sample is the
        plan's own heading, which is what the base will face on arrival (the
        local planner aligns the base with the path), so the observability is
        evaluated at the viewpoint that will actually exist at that time.
        """
        if not plan_poses or len(plan_poses) < 2:
            return (np.zeros((0, 2)), np.zeros((0,)), np.zeros((0,)))

        xy = np.array([[p.pose.position.x, p.pose.position.y] for p in plan_poses], dtype=float)
        yaw = np.array([_yaw_from_quat(p.pose.orientation) for p in plan_poses], dtype=float)

        # drop duplicate consecutive points
        keep = [0]
        for i in range(1, len(xy)):
            if np.hypot(*(xy[i] - xy[keep[-1]])) > 1e-3:
                keep.append(i)
        xy, yaw = xy[keep], yaw[keep]
        if len(xy) < 2:
            return (np.zeros((0, 2)), np.zeros((0,)), np.zeros((0,)))

        seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        total = s[-1]

        n = max(2, int(np.ceil(total / self.spacing)) + 1)
        s_new = np.linspace(0.0, total, n)
        x_new = np.interp(s_new, s, xy[:, 0])
        y_new = np.interp(s_new, s, xy[:, 1])
        yaw_new = np.interp(s_new, s, np.unwrap(yaw))

        # arrival times: constant nominal speed with a slow-down near the target
        ds = np.diff(s_new)
        mid = 0.5 * (s_new[:-1] + s_new[1:])
        remaining = total - mid
        v = np.clip(self.v_nominal * np.minimum(1.0, 0.35 + remaining / 1.2),
                    self.v_min, self.v_nominal)
        tau = np.concatenate([[0.0], np.cumsum(ds / v)])
        tau = tau + self.settle
        return np.stack([x_new, y_new], axis=1), yaw_new, tau


def _yaw_from_quat(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
