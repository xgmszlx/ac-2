#!/usr/bin/env python3
"""Localization-observability cost for the ExplORB-SLAM candidate set.

The candidate set, the executable-plan sampling and the motion constraints are
the author's. This module only *scores* the very same candidates with one extra
term, so that a comparison against native ExplORB isolates that term.

Objective per candidate frontier ``c``::

    J(c) = I_expl(c)                     author D-opt information gain (unchanged)
         - lambda_t * T(c)               travel time of the executable plan
         - kappa * W(c)                  bounded information-deficit cost

    W(c) = sum_k  g(deficit_k) * dtau_k        (bounded per pose and in total)

    deficit_k = clip( log( ref / (strength_k * (1 - p_occ_k)) ), 0, cap )
                * (1 - rho_k)^gamma

    strength_k : weakest normalised marginal std of the pose's 6x6 information
                 matrix (translation and rotation on one scale)
    p_occ_k    : upper-confidence occlusion probability predicted for the pose's
                 arrival time from *past* frontend observations
    rho_k      : exploration credit = unknown-cell fraction in the pose's view
                 wedge (new-feature prior, so unmapped space is not punished)
    dtau_k     : exposure time between consecutive samples (duration matters:
                 a blind moment and a blind corridor are not the same risk)

All quantities are computed from the unchanged ORB-SLAM publications, the grid
map and the move_base global plan.
"""
from __future__ import print_function

import math

import numpy as np
import rospy

from loo_observability import (CameraModel, ObservabilityEstimator, PathSampler, _yaw_from_quat)
from loo_occlusion import OcclusionField, ExplorationCredit


DEF_SCORE = dict(
    lambda_travel=0.02,     # per second of travel time
    kappa_deficit=0.35,     # per (nat * second) of information deficit
    dtau_min=0.15,          # s, floor on a sample's exposure time
    use_duration=True,      # loo_noduration sets this False (ablation)
    use_temporal=True,      # loo_notemporal sets this False (ablation)
    use_credit=True,
    max_samples=40,
)


class LOOScorer(object):
    def __init__(self, map_obj, params=None):
        self.map = map_obj
        self.p = dict(DEF_SCORE)
        if params:
            self.p.update({k: v for k, v in params.items() if k in self.p})

        m = map_obj
        self.cam = CameraModel(m.fx, m.fy, m.cx, m.cy, m.imCols, m.imRows,
                               m.rot_camera_base.as_dcm(), m.t_camera_base)
        self.obs = ObservabilityEstimator(params)
        self.occ = OcclusionField(params)
        self.credit = ExplorationCredit(
            max_range=self.obs.p['max_visual_range'],
            half_fov_deg=params.get('half_fov_deg', 32.0) if params else 32.0)
        self.sampler = PathSampler(
            spacing=params.get('sample_spacing', 0.45) if params else 0.45,
            v_nominal=params.get('v_nominal', 0.22) if params else 0.22)

        self._kf_seen = set()
        self._last_ref = None
        self._strength_buf = []
        self.last_strengths = []

    # ------------------------------------------------------------------ state
    def refresh(self, t_now, update_records=True):
        """Rebuild the map snapshot and ingest occlusion records for new keyframes.

        Cost control: the geometric evaluation of a keyframe pose is only done
        *once*, when that keyframe is first published. Re-doing it every cycle
        would grow with the map and would also change the keyframe's recorded
        visibility ratio retrospectively (the map keeps growing), which would
        leak future structure into a past measurement.
        """
        m = self.map
        if not m.p3D:
            return
        ids = list(m.p3D.keys())
        X = np.asarray([m.p3D[i] for i in ids], dtype=float)
        self.obs.set_map(ids, X)

        if not update_records:
            return

        n_real = {}
        for pid, kfs in m.mpObs.items():
            for k in kfs:
                n_real[k] = n_real.get(k, 0) + 1

        records = []
        for kf_id, Tcw in m.Tcw.items():
            kid = int(kf_id)
            if kid == 0 or kid in self._kf_seen:
                continue
            t = np.asarray(Tcw.translation(), dtype=float)
            R = np.asarray(Tcw.rotationMatrix(), dtype=float)
            yaw = math.atan2(R[1, 0], R[0, 0])
            ev = self.obs.pose_eval(self.cam, R, t)
            self._kf_seen.add(kid)
            self._strength_buf.append(ev['strength'])
            records.append(dict(id=kid, t=t_now, x=float(t[0]), y=float(t[1]),
                                yaw=float(yaw), n_exp=float(ev['n_vis']),
                                n_real=float(n_real.get(kid, 0))))

        if len(self._strength_buf) > 400:
            self._strength_buf = self._strength_buf[-400:]

        if records:
            added = self.occ.ingest(records)
            if added:
                rospy.loginfo('LOO: +%d keyframe occlusion records (field n=%d, occ_mean=%.2f)',
                              added, len(self.occ), self.occ.debug_state()['mean_occ'])
        self._last_ref = self.obs.update_reference(self._strength_buf)

    def ref(self):
        return self._last_ref

    # ------------------------------------------------------------------ score
    def score(self, plan, xy_robot, t_now, grid_data, grid_info):
        """Score one candidate frontier from its executable plan."""
        out = dict(W=0.0, T=0.0, n_samples=0, samples=[],
                   mean_deficit=0.0, mean_pocc=0.0, mean_credit=0.0)
        if not plan:
            return out

        xy, yaw, tau = self.sampler.sample(plan)
        n = len(tau)
        if n == 0:
            return out
        if n > self.p['max_samples']:
            sel = np.linspace(0, n - 1, self.p['max_samples']).astype(int)
            xy, yaw, tau = xy[sel], yaw[sel], tau[sel]
            n = len(tau)

        dtau = np.diff(np.concatenate([tau, [tau[-1] + self.p['dtau_min']]]))
        dtau = np.maximum(dtau, self.p['dtau_min'])
        if not self.p['use_duration']:
            dtau = np.ones_like(dtau)

        ref = self.ref()
        W = 0.0
        samples = []
        for i in range(n):
            R = yaw_to_R(yaw[i])
            t_base = np.array([xy[i, 0], xy[i, 1], 0.0])
            ev = self.obs.pose_eval(self.cam, R, t_base)
            strength = ev['strength']

            if self.p['use_temporal']:
                p_occ, p_mean, _sig = self.occ.predict(xy[i], yaw[i], t_now, tau[i])
            else:
                p_occ, p_mean = 0.0, 0.0

            strength_eff = strength * max(0.0, 1.0 - p_occ)

            rho = 0.0
            if self.p['use_credit'] and grid_data is not None and len(grid_data) > 0:
                rho = self.credit.credit(grid_data, grid_info, xy[i], yaw[i])

            d = self.obs.deficit(strength_eff, rho, ref=ref)
            W += d * dtau[i]
            samples.append((float(xy[i, 0]), float(xy[i, 1]), float(yaw[i]), float(tau[i]),
                            float(strength), float(p_occ), float(d), float(dtau[i]), int(ev['n_vis'])))

        out.update(W=float(W), T=float(tau[-1]), n_samples=n, samples=samples,
                   mean_deficit=float(np.mean([s[6] for s in samples])),
                   mean_pocc=float(np.mean([s[5] for s in samples])),
                   mean_credit=float(np.mean([s[7] for s in samples])))
        return out

    # ------------------------------------------------------------------ final
    def combine(self, info_gain, scores):
        """Apply the bounded extra terms to the author's information gain."""
        lam = self.p['lambda_travel']
        kap = self.p['kappa_deficit']
        return [float(ig) - lam * s['T'] - kap * s['W']
                for ig, s in zip(info_gain, scores)]


# ---------------------------------------------------------------------------
def yaw_to_R(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def R_rob(R):
    """Placeholder: the estimator expects the rotation of the base pose.

    Map stores Tcw (camera poses) already converted to the ROS base frame in
    Map.setNodes; the same matrix is what the estimator needs here.
    """
    return np.asarray(R, dtype=float)


class DecisionAudit(object):
    """Per-decision audit trail (the 'actual action log' of the planner)."""

    COLS = ['t', 'cycle', 'strategy', 'ip', 'chosen', 'x', 'y',
            'info_gain', 'travel_s', 'W_deficit', 'score',
            'mean_deficit', 'mean_pocc', 'mean_credit', 'n_samples',
            'ref_strength', 'occ_records', 'planner_ms']

    def __init__(self, run_dir):
        import csv
        import os
        self.rows = []
        self.path = os.path.join(run_dir, 'decisions.csv') if run_dir else ''
        if self.path:
            self._f = open(self.path, 'w', newline='')
            self._w = csv.writer(self._f)
            self._w.writerow(self.COLS)
            self._f.flush()

    def row(self, **kw):
        if not self.path:
            return
        self._w.writerow([kw.get(c, '') for c in self.COLS])
        self._f.flush()

    def close(self):
        if self.path:
            try:
                self._f.close()
            except Exception:
                pass
