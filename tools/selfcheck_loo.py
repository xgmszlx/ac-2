#!/usr/bin/env python3
"""Offline self-checks that need no Gazebo, no ROS master and no simulator.

    python3 tools/selfcheck_loo.py

Covers
  1. ``informationGain_NUMBA`` dtype dispatch (the int8/array('b') trap that killed
     Filter.py) -- the same call the fusion filter makes.
  2. ``loo_observability`` normalisation direction: with dxi = S z the normalised
     information must be H_z = S^T H S, so scaling every required accuracy by a
     factor f must scale the reported strength by exactly f (and not by 1/f, which
     is what the pre-2026-11-25 S^-1 H S^-1 convention produced).
  3. rank deficiency -> strength 0, and monotonicity in the number of visible points.
  4. ``loo_occlusion`` semantics: n_real/n_exp is consumed as an empirical
     *reliability* w_rel (1 = fully reliable), and the planner-side bound is a lower
     confidence bound, never an "occlusion probability".

Exit code 0 = every check passed. Prints one line per check so the log is the
evidence.
"""
from __future__ import print_function

import array
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'ws/src/decision_maker/src/python'))

# Both packages ship a module called ``Functions``; the decision_maker one wins on the
# path, so the frontier_detector one (which owns informationGain_NUMBA) is loaded
# explicitly from its file.
import importlib.util

_spec = importlib.util.spec_from_file_location(
    'frontier_functions',
    os.path.join(ROOT, 'ws/src/frontier_detector/src/python/Functions.py'))
frontier_functions = importlib.util.module_from_spec(_spec)
sys.modules['frontier_functions'] = frontier_functions   # numba resolves globals by module name
_spec.loader.exec_module(frontier_functions)

FAILS = []


def check(name, ok, detail=''):
    print('[%s] %-58s %s' % ('PASS' if ok else 'FAIL', name, detail))
    if not ok:
        FAILS.append(name)


# ---------------------------------------------------------------- 1. numba dtype
def check_numba_dtype():
    try:
        informationGain = frontier_functions.informationGain
        informationGain_NUMBA = frontier_functions.informationGain_NUMBA
    except Exception as e:                                   # pragma: no cover
        check('numba import', False, repr(e))
        return
    W = H = 60
    data = [-1] * (W * H)
    for y in range(20, 40):
        for x in range(20, 40):
            data[y * W + x] = 0
    ref = informationGain.__wrapped__ if hasattr(informationGain, '__wrapped__') else None
    args = dict(res=0.1, width=W, ox=-3.0, oy=-3.0)

    # the exact thing Filter.py used to do: np.array(map_data_.data)
    bad = np.array(array.array('b', data))
    raised = False
    try:
        informationGain_NUMBA(args['res'], args['width'], args['ox'], args['oy'], bad, 1.0, 0.0, 1.0)
    except TypeError as e:
        raised = 'No matching definition' in str(e) or 'int8' in str(e)
    check('int8 dispatch is rejected (documents the old bug)', raised,
          'np.array(array("b",...)).dtype=%s' % bad.dtype)

    # the fixed path: int64 copy, made once per map message
    good = np.asarray(array.array('b', data), dtype=np.int64)
    v = informationGain_NUMBA(args['res'], args['width'], args['ox'], args['oy'], good, 1.0, 0.0, 1.0)
    # pure-python reference implementation
    class _I(object):
        pass
    info = _I()
    info.resolution, info.width, info.height = args['res'], args['width'], W
    info.origin = _I()
    info.origin.position = _I()
    info.origin.position.x, info.origin.position.y = args['ox'], args['oy']
    m = _I()
    m.info, m.data = info, data
    v_ref = informationGain(m, [1.0, 0.0], 1.0)
    check('int64 dispatch matches the python reference', abs(v - v_ref) < 1e-9,
          'numba=%.6f python=%.6f' % (v, v_ref))


# ------------------------------------------------- 2/3. observability normalisation
def _synthetic_scene(n_points=120, seed=3):
    rng = np.random.RandomState(seed)
    X = np.stack([rng.uniform(-1.2, 1.2, n_points),
                  rng.uniform(-0.8, 0.8, n_points),
                  rng.uniform(1.0, 3.0, n_points)], axis=1)
    ids = np.arange(n_points, dtype=np.int64)
    return ids, X


def check_observability_direction():
    from loo_observability import CameraModel, ObservabilityEstimator
    ids, X = _synthetic_scene()
    I = np.eye(3)
    cam = CameraModel(535.0, 535.0, 320.0, 240.0, 640, 480, I, np.zeros(3))
    ev = ObservabilityEstimator()
    ev.set_map(ids, X)

    base = ev.pose_eval(cam, I, np.zeros(3))
    check('pose_eval finds points', base['n_vis'] > 30, 'n_vis=%d' % base['n_vis'])

    f = 2.0
    ev2 = ObservabilityEstimator(dict(sigma_t_req=0.05 * f, sigma_r_req=0.02 * f))
    ev2.set_map(ids, X)
    scaled = ev2.pose_eval(cam, I, np.zeros(3))

    ratio = scaled['strength'] / base['strength'] if base['strength'] > 0 else float('nan')
    # H_z = S^T H S: doubling every s multiplies H_z by 4, so strength = sqrt(lam_min)
    # doubles. The previous convention (S^-1 H S^-1) would have halved it.
    check('strength scales with +1 power of s (H_z = S^T H S)',
          abs(ratio - f) < 1e-6, 'base=%.6f scaled=%.6f ratio=%.6f (want %.1f)'
          % (base['strength'], scaled['strength'], ratio, f))

    # marginal_std of z_i must be dimensionless and *shrink* when s grows
    check('marginal_std of z shrinks when s grows',
          np.all(np.isfinite(scaled['marginal_std'])) and
          np.all(scaled['marginal_std'] <= base['marginal_std'] + 1e-12),
          'max base=%.4f scaled=%.4f' % (np.max(base['marginal_std']), np.max(scaled['marginal_std'])))

    # the smooth worst-direction metric must sit just above the hard max (conservative)
    ev3 = ObservabilityEstimator()
    ev3.set_map(ids, X)
    lt = ev3.pose_eval(cam, I, np.zeros(3))
    hard = lt['worst_marginal_std']
    soft = 1.0 / lt['strength']
    check('smooth worst-case >= hard worst-case, within 1.5x',
          soft >= hard - 1e-12 and soft <= 1.5 * hard,
          'soft=%.6f hard=%.6f' % (soft, hard))


def check_observability_monotonic():
    from loo_observability import CameraModel, ObservabilityEstimator
    I = np.eye(3)
    cam = CameraModel(535.0, 535.0, 320.0, 240.0, 640, 480, I, np.zeros(3))
    strengths = []
    for n in (5, 20, 60, 120):
        ids, X = _synthetic_scene(n_points=n, seed=7)
        ev = ObservabilityEstimator()
        ev.set_map(ids, X)
        strengths.append(ev.pose_eval(cam, I, np.zeros(3))['strength'])
    check('strength is 0 below the rank threshold, then increases with points',
          strengths[0] == 0.0 and all(b >= a - 1e-9 for a, b in zip(strengths[1:], strengths[2:])),
          'strengths=%s' % ['%.4f' % s for s in strengths])


# --------------------------------------------------------- 4. reliability semantics
def check_reliability_semantics():
    from loo_occlusion import ReliabilityField, OcclusionField, DEF
    check('OcclusionField is an alias of ReliabilityField', OcclusionField is ReliabilityField)

    f = ReliabilityField()
    # a well-observed viewpoint: 90 of 100 expected points were really matched
    for i in range(12):
        f.ingest([dict(id=i, t=0.0, x=1.0, y=1.0, yaw=0.0, n_exp=100.0, n_real=90.0)])
    w_lo, w_mean, sig = f.predict_reliability(np.array([1.0, 1.0]), 0.0, 10.0, 0.0)
    check('reliability of a well-observed viewpoint is near 0.9', 0.6 < w_mean < 1.0,
          'w_mean=%.3f w_lo=%.3f sigma=%.3f' % (w_mean, w_lo, sig))
    check('the planner gets a *lower* confidence bound', w_lo <= w_mean + 1e-12,
          'w_lo=%.3f w_mean=%.3f' % (w_lo, w_mean))

    g = ReliabilityField()
    for i in range(12):
        g.ingest([dict(id=i, t=0.0, x=1.0, y=1.0, yaw=0.0, n_exp=100.0, n_real=10.0)])
    _, w_low, _ = g.predict_reliability(np.array([1.0, 1.0]), 0.0, 10.0, 0.0)
    check('a poorly-observed viewpoint reports *low* reliability (not an occlusion prob)',
          w_low < 0.45, 'w_mean=%.3f' % w_low)

    # the prior must keep an unevidenced viewpoint usable
    empty = ReliabilityField()
    check('empty field reports no evidence', len(empty) == 0 and empty.predict_reliability(
        np.array([0.0, 0.0]), 0.0, 0.0, 0.0) == (0.0, 0.0, 0.0))
    check('prior is biased towards usable structure', DEF['prior_a'] > DEF['prior_b'],
          'prior_a=%.2f prior_b=%.2f' % (DEF['prior_a'], DEF['prior_b']))


def main():
    print('=== selfcheck_loo (no Gazebo, no ROS master) ===')
    check_numba_dtype()
    check_observability_direction()
    check_observability_monotonic()
    check_reliability_semantics()
    print('=== %d failure(s) ===' % len(FAILS))
    if FAILS:
        for f in FAILS:
            print('  FAILED: %s' % f)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
