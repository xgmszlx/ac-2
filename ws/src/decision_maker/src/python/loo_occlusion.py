#!/usr/bin/env python3
"""Short-horizon dynamic-occlusion model for the ExplORB-SLAM line.

Measurement, not simulation truth
---------------------------------
ORB-SLAM's published map already contains, for every map point, the set of
keyframes that actually observe it. Two independent quantities are therefore
available for any keyframe pose ``k`` *at the moment it is published*:

    n_real(k)  points the frontend really matched from that pose
               (= |{p : k in observations(p)}|, straight from point_list)
    n_exp(k)   points that the current map says are geometrically visible from
               that same pose (projection + range + image bounds + depth sign)

Their ratio is a *measured* visibility ratio: structure that the map contains,
that the geometry says should be in view, but that the tracker did not match.
Causes are exactly the ones this line is about -- dynamic structure passing in
front of the background, blur, low texture. It is viewpoint-conditioned by
construction, so evaluating it at the *future* pose of the plan (instead of at
the current pose) is the ego-motion compensation.

The field is then a kernel smoother over recent keyframes in (x, y, yaw, time)
with a Beta posterior for the occlusion probability, and the planner consumes the
upper confidence bound, which is how the uncertainty enters the cost. Only
already-published map/graph data and past timestamps are used: no future image,
no future trajectory, no simulator state.
"""
from __future__ import print_function

import math

import numpy as np

DEF = dict(
    spatial_sigma=1.15,      # m, how far an occlusion observation transfers
    yaw_sigma_deg=65.0,      # deg, viewpoint similarity of the camera direction
    time_tau=9.0,            # s, short-horizon decay (dynamics are not stationary)
    time_hard_cut=45.0,      # s, older records are dropped entirely
    prior_a=1.0,             # Beta prior: "structure usually visible"
    prior_b=0.25,
    ucb_z=1.0,               # conservative quantile used by the planner
    min_points_expected=8,   # below this the ratio is too noisy to use
    max_records=400,
)


class OcclusionField(object):
    def __init__(self, params=None):
        self.p = dict(DEF)
        if params:
            self.p.update({k: v for k, v in params.items() if k in self.p})
        self.p['yaw_sigma'] = math.radians(self.p['yaw_sigma_deg'])
        self._t = []
        self._xy = []
        self._yaw = []
        self._occ = []
        self._w = []
        self._seen = set()

    # ------------------------------------------------------------------ feed
    def ingest(self, records):
        """records: iterable of dicts with keys id, t, x, y, yaw, n_exp, n_real."""
        added = 0
        for r in records:
            rid = r.get('id')
            if rid is not None:
                if rid in self._seen:
                    continue
                self._seen.add(rid)
            n_exp = float(r.get('n_exp', 0.0))
            if n_exp < self.p['min_points_expected']:
                continue
            n_real = float(r.get('n_real', 0.0))
            ratio = min(1.0, n_real / n_exp)
            # measurement confidence grows with the number of expected points
            w = n_exp / (n_exp + 25.0)
            self._t.append(float(r['t']))
            self._xy.append((float(r['x']), float(r['y'])))
            self._yaw.append(float(r['yaw']))
            self._occ.append(1.0 - ratio)
            self._w.append(w)
            added += 1
        self._prune()
        return added

    def _prune(self):
        n = len(self._t)
        if n <= self.p['max_records']:
            return
        order = np.argsort(self._t)[-self.p['max_records']:]
        self._t = [self._t[i] for i in order]
        self._xy = [self._xy[i] for i in order]
        self._yaw = [self._yaw[i] for i in order]
        self._occ = [self._occ[i] for i in order]
        self._w = [self._w[i] for i in order]

    def __len__(self):
        return len(self._t)

    # -------------------------------------------------------------- predict
    def predict(self, xy, yaw, t_now, tau):
        """Occlusion probability of the background constraints at a future pose.

        xy, yaw : the pose the plan reaches      (map frame)
        t_now   : current time
        tau     : seconds from now at which the pose is reached

        Returns (p_ucb, p_mean, sigma) with p in [0, 1].
        """
        if not self._t:
            return 0.0, 0.0, 0.0

        t = np.asarray(self._t)
        age = (t_now - t)
        dt_future = np.clip(age + float(tau), 0.0, None)     # the pose is reached later
        keep = (dt_future <= self.p['time_hard_cut'])
        if not np.any(keep):
            return 0.0, 0.0, 0.0

        xy_a = np.asarray(self._xy)
        yaw_a = np.asarray(self._yaw)
        occ_a = np.asarray(self._occ)
        w_a = np.asarray(self._w)

        d2 = np.sum((xy_a - np.asarray(xy, dtype=float)[None, :]) ** 2, axis=1)
        k_s = np.exp(-0.5 * d2 / (self.p['spatial_sigma'] ** 2))

        dyaw = np.abs(_wrap(yaw_a - float(yaw)))
        k_y = np.exp(-0.5 * (dyaw / self.p['yaw_sigma']) ** 2)

        k_t = np.exp(-dt_future / self.p['time_tau'])

        w = w_a * k_s * k_y * k_t
        w = np.where(keep, w, 0.0)
        sw = float(np.sum(w))
        if sw < 1e-6:
            return 0.0, 0.0, 0.0

        # Beta posterior over the occlusion probability of this viewpoint.
        a = self.p['prior_a'] + float(np.sum(w * occ_a))
        b = self.p['prior_b'] + float(np.sum(w * (1.0 - occ_a)))
        mean = a / (a + b)
        var = (a * b) / ((a + b) ** 2 * (a + b + 1.0))
        sigma = math.sqrt(max(var, 0.0))

        # The effective sample size caps how confident a single neighbour can make
        # us: sw counts kernel-weighted evidence, not observations.
        conf = sw / (sw + 1.5)
        sigma_eff = sigma + (1.0 - conf) * 0.5
        p_ucb = min(1.0, mean + self.p['ucb_z'] * sigma_eff)
        return float(p_ucb), float(mean), float(sigma_eff)

    def debug_state(self):
        return dict(n=len(self._t),
                    mean_occ=float(np.mean(self._occ)) if self._occ else 0.0)


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ExplorationCredit(object):
    """New-feature prior for unmapped structure (the anti-"protect old points" term).

    A pose that looks at a lot of unknown occupancy will generate new map points
    when it is reached; those new points carry the localization constraint there.
    The credit is the fraction of unknown cells inside the future ground-plane
    view wedge, so a frontier-facing pose is not punished for having few old map
    points in view. It is derived from the same rectified grid the frontier
    detector consumes.
    """

    def __init__(self, max_range=3.0, half_fov_deg=32.0, n_rays=25, n_rings=12):
        self.max_range = float(max_range)
        self.half_fov = math.radians(half_fov_deg)
        self.n_rays = int(n_rays)
        self.n_rings = int(n_rings)

    def credit(self, grid_data, info, xy, yaw):
        """grid_data: 1-D sequence (ros OccupancyGrid.data), info: its info field.

        Returns fraction of unknown cells sampled inside the view wedge.
        """
        if grid_data is None or info is None or len(grid_data) < 1:
            return 0.0
        res = info.resolution
        w = info.width
        h = info.height
        ox = info.origin.position.x
        oy = info.origin.position.y

        ys = np.linspace(-self.half_fov, self.half_fov, self.n_rays)
        rs = np.linspace(0.35, self.max_range, self.n_rings)
        # pose yaw follows the baseline convention (yawBtw2Points): an angle in
        # the map frame measured from +x towards +y, so the forward direction of
        # the camera is (cos yaw, sin yaw).
        ang = yaw + ys[None, :]
        px = xy[0] + np.cos(ang) * rs[:, None]
        py = xy[1] + np.sin(ang) * rs[:, None]
        ix = np.floor((px - ox) / res).astype(int)
        iy = np.floor((py - oy) / res).astype(int)
        valid = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        if not np.any(valid):
            return 0.0
        idx = iy[valid] * w + ix[valid]
        idx = idx[idx < len(grid_data)]
        if idx.size == 0:
            return 0.0
        vals = np.asarray([grid_data[int(i)] for i in idx])
        unknown = np.mean(vals == -1)
        return float(unknown)
