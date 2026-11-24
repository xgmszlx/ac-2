#!/usr/bin/env python3
"""Measured *observation reliability* field for the ExplORB-SLAM line.

Semantics corrected on 2026-11-25 (task book, phase 2.1)
-------------------------------------------------------
The quantity that the frontend actually measures is

    w_rel(k) = n_real(k) / n_exp(k)

  n_real(k)  points the frontend really matched from keyframe pose k
             (= |{p : k in observations(p)}|, straight from point_list)
  n_exp(k)   points the current map says are geometrically visible from the same
             pose (projection + range + image bounds + depth sign)

This ratio is an **empirical observation reliability**: how much of the structure
that geometry says should be in view the tracker actually used. Its shortfall has
many causes -- dynamic structure in front of the background, motion blur, low
texture, frontend gating -- and the frontend alone cannot attribute the share of
each. It was therefore *wrong* (previous version of this file) to call
``1 - w_rel`` a dynamic occlusion probability and feed it to the planner as
``p_dyn``: that asserts a cause the measurement does not identify, and it also
mixes an empirical reliability (which the map can never exceed) with a physical
occlusion probability (which the future can change).

What each model is allowed to say
---------------------------------
    w_rel(k)     *measured* reliability of a viewpoint, dimensionless in [0, 1],
                 viewpoint-conditioned by construction (it is computed at a pose).
                 Used as a discount on the constraint strength the planner expects:
                     strength_eff = strength(z) * w_rel
    p_dyn(t)     *predicted* dynamic-occlusion probability at a future time, from
                 tracked dynamic objects with ego-motion compensation and constant
                 velocity extrapolation. It is NOT derived from w_rel; see the
                 C++ query module (visual_query) in phase 2.3.

The planner consumes the lower confidence bound of w_rel, because this module's
uncertainty must push the cost up, not down.

Everything here uses only already-published map/graph data and past timestamps: no
future image, no future trajectory, no simulator state.
"""
from __future__ import print_function

import math

import numpy as np

DEF = dict(
    spatial_sigma=1.15,      # m, how far a reliability observation transfers
    yaw_sigma_deg=65.0,      # deg, viewpoint similarity of the camera direction
    time_tau=9.0,            # s, short-horizon decay (reliability is not stationary)
    time_hard_cut=45.0,      # s, older records are dropped entirely
    prior_a=2.0,             # Beta prior: "structure is usually observed reliably"
    prior_b=0.5,
    ucb_z=1.0,               # width of the confidence bound used by the planner
    min_points_expected=8,   # below this the ratio is too noisy to use
    max_records=400,
)


class ReliabilityField(object):
    """Kernel-smoothed empirical observation reliability over (x, y, yaw, t).

    Renamed from ``OcclusionField``: the field no longer claims to be an occlusion
    probability (see the module docstring). ``OcclusionField`` is kept as an alias
    so that older ablation scripts keep importing something meaningful.
    """

    def __init__(self, params=None):
        self.p = dict(DEF)
        if params:
            self.p.update({k: v for k, v in params.items() if k in self.p})
        self.p['yaw_sigma'] = math.radians(self.p['yaw_sigma_deg'])
        self._t = []
        self._xy = []
        self._yaw = []
        self._rel = []          # measured reliability w_rel in [0, 1]
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
            ratio = min(1.0, max(0.0, n_real / n_exp))     # w_rel, NOT an occlusion term
            # measurement confidence grows with the number of expected points
            w = n_exp / (n_exp + 25.0)
            self._t.append(float(r['t']))
            self._xy.append((float(r['x']), float(r['y'])))
            self._yaw.append(float(r['yaw']))
            self._rel.append(ratio)
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
        self._rel = [self._rel[i] for i in order]
        self._w = [self._w[i] for i in order]

    def __len__(self):
        return len(self._t)

    # -------------------------------------------------------------- predict
    def predict_reliability(self, xy, yaw, t_now, tau):
        """Empirical observation reliability of the plan's future viewpoint.

        xy, yaw : the pose the plan reaches      (map frame)
        t_now   : current time
        tau     : seconds from now at which the pose is reached

        Returns (w_lo, w_mean, sigma_eff): ``w_lo`` is the lower confidence bound
        the planner must use (uncertainty increases the cost), ``w_mean`` the
        posterior mean.
        """
        none = (0.0, 0.0, 0.0)
        if not self._t:
            return none

        t = np.asarray(self._t)
        age = (t_now - t)
        dt_future = np.clip(age + float(tau), 0.0, None)     # the pose is reached later
        keep = (dt_future <= self.p['time_hard_cut'])
        if not np.any(keep):
            return none

        xy_a = np.asarray(self._xy)
        yaw_a = np.asarray(self._yaw)
        rel_a = np.asarray(self._rel)
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
            return none

        # Beta posterior over the reliability of this viewpoint.
        a = self.p['prior_a'] + float(np.sum(w * rel_a))
        b = self.p['prior_b'] + float(np.sum(w * (1.0 - rel_a)))
        mean = a / (a + b)
        var = (a * b) / ((a + b) ** 2 * (a + b + 1.0))
        sigma = math.sqrt(max(var, 0.0))

        # The effective sample size caps how confident a single neighbour can make
        # us: sw counts kernel-weighted evidence, not observations.
        conf = sw / (sw + 1.5)
        sigma_eff = sigma + (1.0 - conf) * 0.5
        w_lo = max(0.0, mean - self.p['ucb_z'] * sigma_eff)
        return float(w_lo), float(mean), float(sigma_eff)

    def debug_state(self):
        return dict(n=len(self._t),
                    mean_rel=float(np.mean(self._rel)) if self._rel else 0.0,
                    mean_unreliable=float(np.mean(1.0 - np.asarray(self._rel))) if self._rel else 0.0)

    # -- alias kept for older ablation scripts / dashboards -----------------
    def predict(self, xy, yaw, t_now, tau):
        """Deprecated alias of :meth:`predict_reliability` (same return values)."""
        return self.predict_reliability(xy, yaw, t_now, tau)


# ``1 - w_rel`` is *not* a dynamic occlusion probability; the old name is kept only
# so that imports in existing scripts do not break.
OcclusionField = ReliabilityField


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
