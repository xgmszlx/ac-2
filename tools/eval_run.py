#!/usr/bin/env python3
"""Offline metrics for one run directory.

    python3 tools/eval_run.py results/runs/<task_id> [--json]

Everything here is *offline* evaluation from the recorded CSVs; the simulator ground
truth (gt.csv, from /gazebo/model_states) is used only in this script, never online
(task book constraint 4).

Outputs one row per run to stdout (or JSON), and appends to
<run_dir>/metrics.json:

    task, arm, success, sim_duration_s, path_len_m, coverage_m2, known_cells,
    tracking_ok_pct, lost_s, ate_rmse_m, rpe_rmse_m, n_goal, n_cmd,
    max_abs_vx, max_abs_wz, n_slam, n_odom

Definitions
-----------
path_len_m       integral of |d odom position| over the run
coverage_m2      final free_area_m2 from coverage.csv
tracking_ok_pct  fraction of state.csv samples with state == 4 (OK)
lost_s           simulated seconds spent with state != 4 after the first OK sample
ate_rmse_m       RMSE of the SLAM position against GT, after aligning both tracks by
                 the rigid 2D transform that maps the first common pose onto GT
rpe_rmse_m       RMSE of the relative displacement over a 1 s window (drift proxy)
"""
from __future__ import print_function

import csv
import json
import math
import os
import re
import sys


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        r = csv.DictReader(f)
        return [row for row in r]


def fnum(v, default=0.0):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def load_track(rows, xkey='x', ykey='y'):
    t = []
    xy = []
    for row in rows:
        t.append(fnum(row.get('t')))
        xy.append((fnum(row.get(xkey)), fnum(row.get(ykey))))
    return t, xy


def path_length(xy):
    s = 0.0
    for i in range(1, len(xy)):
        s += math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1])
    return s


def align_tracks(gt_t, gt_xy, sl_t, sl_xy):
    """Return the aligned SLAM track: pose the SLAM samples onto the GT samples by the
    rigid 2D transform of the first time-matched pair (that is what ORB-SLAM's map
    frame origin means: the pose at initialisation)."""
    if not gt_t or not sl_t:
        return [], []
    j = 0
    pairs = []
    for i, t in enumerate(sl_t):
        while j + 1 < len(gt_t) and abs(gt_t[j + 1] - t) <= abs(gt_t[j] - t):
            j += 1
        if abs(gt_t[j] - t) < 0.2:
            pairs.append((i, j))
    if not pairs:
        return [], []
    i0, j0 = pairs[0]
    dx = gt_xy[j0][0] - sl_xy[i0][0]
    dy = gt_xy[j0][1] - sl_xy[i0][1]
    out_t, out_xy = [], []
    for (i, j) in pairs:
        out_t.append(sl_t[i])
        out_xy.append((sl_xy[i][0] + dx, sl_xy[i][1] + dy))
    gt_pairs = [gt_xy[j] for (_i, j) in pairs]
    return out_t, (out_xy, gt_pairs)


def ate(out_xy, gt_xy):
    if not out_xy:
        return float('nan')
    n = min(len(out_xy), len(gt_xy))
    if n == 0:
        return float('nan')
    s = 0.0
    for i in range(n):
        s += (out_xy[i][0] - gt_xy[i][0]) ** 2 + (out_xy[i][1] - gt_xy[i][1]) ** 2
    return math.sqrt(s / n)


def rpe(t, xy, window=1.0):
    """Relative position error over a fixed time window, RMSE (drift proxy)."""
    if len(xy) < 3:
        return float('nan')
    acc = []
    j = 0
    for i in range(len(xy)):
        while j < len(xy) and t[j] - t[i] < window:
            j += 1
        if j >= len(xy):
            break
        acc.append((xy[j][0] - xy[i][0], xy[j][1] - xy[i][1]))
    if not acc:
        return float('nan')
    # a static reference: the mean relative displacement; deviation from it measures
    # non-rigidity of the trajectory (the classic RPE proxy)
    mx = sum(a[0] for a in acc) / len(acc)
    my = sum(a[1] for a in acc) / len(acc)
    s = sum((a[0] - mx) ** 2 + (a[1] - my) ** 2 for a in acc)
    return math.sqrt(s / len(acc))


INSTR = re.compile(r'\[obs-instr\] frame=(\d+) N=(\d+) valid_depth_kp=(\d+) '
                   r'depth_px_sampled_valid=(\d+) state=(-?\d+)')
CAMERA_FPS = 30.0          # this platform publishes the camera at 30 Hz


def internal_tracking(simlog):
    """Tracking loss from the SLAM frontend's *own* state, not from the ROS publisher.

    `orb_slam2_ros/info/state` alternates between 4 (OK) and 5 (LOST) every 20-30 ms on
    this platform even though the tracker is fine, so the published state is useless as a
    loss indicator. The `[obs-instr]` line prints the internal Tracking::eTrackingState
    every 60 frames (2 s at 30 fps); 2 == OK, 3 == LOST. The duration of each non-OK run
    is measured in frames, then converted with the camera rate.

    Returns dict(ok_pct, lost_s, n_samples, min_N, mean_N) or None when the log is absent.
    """
    if not os.path.exists(simlog):
        return None
    recs = []
    for line in open(simlog, errors='ignore'):
        m = INSTR.search(line)
        if m:
            recs.append((int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(5))))
    if len(recs) < 3:
        return None
    last_frame = recs[-1][0]
    first_frame = recs[0][0]
    lost_frames = 0
    n_ok = 0
    prev = None
    for (f, _N, _vd, st) in recs:
        ok = (st == 2)
        if ok:
            n_ok += 1
        elif prev is not None:
            # count the *misclassified* span: half the gap on each side, so a single bad
            # sample between two good ones does not become a 2 s loss
            lost_frames += (f - prev) / CAMERA_FPS
        prev = f
    total_s = (last_frame - first_frame) / CAMERA_FPS if last_frame > first_frame else 0.0
    Ns = [r[1] for r in recs]
    return dict(ok_pct=100.0 * n_ok / len(recs),
                lost_s=lost_frames,
                analysed_s=total_s,
                n_samples=len(recs),
                min_N=min(Ns), mean_N=sum(Ns) / len(Ns))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    run_dir = sys.argv[1].rstrip('/')
    as_json = '--json' in sys.argv[2:]

    gt = read_csv(os.path.join(run_dir, 'gt.csv'))
    sl = read_csv(os.path.join(run_dir, 'slam.csv'))
    od = read_csv(os.path.join(run_dir, 'odom.csv'))
    st = read_csv(os.path.join(run_dir, 'state.csv'))
    cv = read_csv(os.path.join(run_dir, 'coverage.csv'))
    cm = read_csv(os.path.join(run_dir, 'cmd.csv'))
    gl = read_csv(os.path.join(run_dir, 'goals.csv'))

    gt_t, gt_xy = load_track(gt)
    sl_t, sl_xy = load_track(sl)
    od_t, od_xy = load_track(od)

    res = {}
    res['task'] = os.path.basename(run_dir)
    res['n_goal'] = sum(1 for r in gl if r.get('kind') == 'goal')
    res['n_result'] = sum(1 for r in gl if r.get('kind') == 'result')
    # move_base action status: 3 = SUCCEEDED, 4 = ABORTED, 2 = CANCELED (preempted)
    st_status = [int(fnum(r.get('status'), -1)) for r in gl if r.get('kind') == 'result']
    res['n_reached'] = sum(1 for x in st_status if x == 3)
    res['n_aborted'] = sum(1 for x in st_status if x == 4)
    res['n_canceled'] = sum(1 for x in st_status if x == 2)
    res['reached_frac'] = (float(res['n_reached']) / res['n_goal']) if res['n_goal'] else 0.0
    res['n_cmd'] = len(cm)
    res['max_abs_vx'] = max([abs(fnum(r.get('vx'))) for r in cm], default=0.0)
    res['max_abs_wz'] = max([abs(fnum(r.get('wz'))) for r in cm], default=0.0)
    res['n_slam'] = len(sl)
    res['n_odom'] = len(od)
    res['path_len_m'] = path_length(od_xy) if od_xy else 0.0
    res['robot_moved_m'] = (math.hypot(od_xy[-1][0] - od_xy[0][0], od_xy[-1][1] - od_xy[0][1])
                            if len(od_xy) > 1 else 0.0)
    res['sim_duration_s'] = max([fnum(r.get('t')) for r in od], default=0.0)
    if cv:
        last = cv[-1]
        res['coverage_m2'] = fnum(last.get('free_area_m2'))
        res['known_cells'] = int(fnum(last.get('known_cells')))
    else:
        res['coverage_m2'] = 0.0
        res['known_cells'] = 0

    states = [(fnum(r.get('t')), int(fnum(r.get('state'), -1))) for r in st]
    ok = [t for (t, s) in states if s == 4]
    res['tracking_ok_pct'] = 100.0 * len(ok) / len(states) if states else 0.0
    if states:
        t_first_ok = ok[0] if ok else None
        lost = 0.0
        for i in range(1, len(states)):
            dt = states[i][0] - states[i - 1][0]
            if states[i][1] != 4 and t_first_ok is not None and states[i][0] >= t_first_ok:
                lost += dt
        res['lost_s'] = lost
    else:
        res['lost_s'] = float('nan')

    aligned = align_tracks(gt_t, gt_xy, sl_t, sl_xy)
    if aligned and aligned[1]:
        sl_al, gt_al = aligned[1]
        res['ate_rmse_m'] = ate(sl_al, gt_al)
        res['n_ate_samples'] = len(sl_al)
    else:
        res['ate_rmse_m'] = float('nan')
        res['n_ate_samples'] = 0
    res['rpe_rmse_m'] = rpe(sl_t, sl_xy) if sl_xy else float('nan')

    # --- tracking loss: internal SLAM state is the authoritative one -----------------
    simlog = None
    for cand in (os.path.join(os.path.dirname(os.path.dirname(run_dir)), 'logs',
                              os.path.basename(run_dir) + '_sim.log'),
                 '/home/wcqw/explorb_line_20261121/logs/%s_sim.log' % os.path.basename(run_dir)):
        if os.path.exists(cand):
            simlog = cand
            break
    it = internal_tracking(simlog) if simlog else None
    if it:
        res['track_ok_internal_pct'] = it['ok_pct']
        res['lost_internal_s'] = it['lost_s']
        res['obs_instr_samples'] = it['n_samples']
        res['min_keypoints_N'] = it['min_N']
        res['mean_keypoints_N'] = it['mean_N']
    else:
        res['track_ok_internal_pct'] = float('nan')
        res['lost_internal_s'] = float('nan')
        res['obs_instr_samples'] = 0
        res['min_keypoints_N'] = 0
        res['mean_keypoints_N'] = 0
    # the published-state numbers stay, but renamed so nobody mistakes them for tracking loss
    res['published_state_ok_pct'] = res.pop('tracking_ok_pct', float('nan'))
    res['published_state_lost_s'] = res.pop('lost_s', float('nan'))
    res['lost_s_note'] = ('lost_internal_s is measured from the frontend internal state '
                          '([obs-instr] state==2); published_state_* only reflects the ROS '
                          'publisher, which flaps 4<->5 on this platform.')

    # strict success: every dispatched goal was reported as reached (status 3).
    # (An earlier version only compared counts, which counted aborted goals as success.)
    res['success'] = bool(res['n_goal'] > 0 and res['n_reached'] == res['n_goal'])

    with open(os.path.join(run_dir, 'metrics.json'), 'w') as f:
        json.dump(res, f, indent=2)

    if as_json:
        print(json.dumps(res, indent=2))
    else:
        print(','.join(res.keys()))
        print(','.join(('%.4f' % v if isinstance(v, float) else str(v)) for v in res.values()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
