#!/usr/bin/env python3
"""Join the two existing log streams into one mechanism table (task book phase 4.3).

    python3 tools/join_mechanism.py results/runs/<task> [--tol 0.2]

The question this answers: "does the *predicted* occlusion / co-visibility actually track
the *measured* tracking quality?"  Two logs already contain the two sides, and nothing has
to be re-run or re-instrumented:

  TEB side   [visual-constraint] lambda_g=.. lambda_c=.. dt_vis=.. vertices=..
                                visual_edges=.. covis_edges=.. graph_edges=..
             (sim log, one line per throttled planning cycle)
  SLAM side  [track-instr] frame=.. matched=<map points still associated after tracking>
             [obs-instr]   frame=.. N=.. valid_depth_kp=.. state=.. masked_out=.. mask_active=..
             (sim log, every 60 frames)
  mask side  export/dynamic_objects.csv   t, n_blobs, n_mask_px   (10 Hz, the w_dyn input)

Output: results/runs/<task>/mechanism.csv with
  t, w_dyn_proxy (masked pixels), n_blobs, covis_edges, visual_edges, vertices,
  matched_inliers, N_keypoints, valid_depth_kp, state, masked_out
joined by nearest simulated time (default tolerance 0.2 s), plus Pearson/Spearman
correlations and a binned table, printed and stored as mechanism_summary.json.

Why a *proxy* for w_dyn: the adapter's internal mean is not on the wire yet; the mask
coverage is exactly what feeds it (w_dyn = product of sigmoid gates over the tracked
objects), so it is the honest predictor-side quantity available offline.
"""
from __future__ import print_function

import csv
import json
import math
import os
import re
import sys

SIMLOG = '/home/wcqw/explorb_line_20261121/logs/%s_sim.log'
CAMERA_FPS = 30.0

RE_CONSTRAINT = re.compile(r'\[visual-constraint\].*lambda_g=([\d.]+) lambda_c=([\d.]+) '
                           r'dt_vis=([\d.]+) vertices=(\d+) visual_edges=(\d+) '
                           r'covis_edges=(\d+) graph_edges=(\d+)')
RE_TRACK = re.compile(r'\[track-instr\] frame=(\d+) matched=(\d+) N=(\d+) state=(-?\d+)')
RE_OBS = re.compile(r'\[obs-instr\] frame=(\d+) N=(\d+) valid_depth_kp=(\d+) '
                    r'depth_px_sampled_valid=(\d+) state=(-?\d+)')
RE_MASKED = re.compile(r'masked_out=(\d+) mask_active=(\d)')


def sim_time_of_line(line):
    """The log prefix carries [INFO] [walltime, simtime]: use the simulated clock."""
    m = re.search(r'\[[\d.]+,\s*([\d.]+)\]', line)
    return float(m.group(1)) if m else None


def parse_simlog(path):
    cons, track, obs = [], [], []
    if not os.path.exists(path):
        return cons, track, obs
    for line in open(path, errors='ignore'):
        t = sim_time_of_line(line)
        m = RE_CONSTRAINT.search(line)
        if m:
            cons.append(dict(t=t, lambda_g=float(m.group(1)), lambda_c=float(m.group(2)),
                             dt_vis=float(m.group(3)), vertices=int(m.group(4)),
                             visual_edges=int(m.group(5)), covis_edges=int(m.group(6)),
                             graph_edges=int(m.group(7))))
            continue
        m = RE_TRACK.search(line)
        if m:
            track.append(dict(frame=int(m.group(1)), matched=int(m.group(2)),
                              N=int(m.group(3)), state=int(m.group(4))))
            continue
        m = RE_OBS.search(line)
        if m:
            mo = RE_MASKED.search(line)
            obs.append(dict(frame=int(m.group(1)), N=int(m.group(2)),
                            valid_depth=int(m.group(3)), state=int(m.group(5)),
                            masked_out=int(mo.group(1)) if mo else 0))
    return cons, track, obs


def read_mask(path):
    out = []
    if not os.path.exists(path):
        return out
    for row in csv.DictReader(open(path)):
        try:
            out.append(dict(t=float(row['t']), n_blobs=int(float(row['n_blobs'])),
                            n_mask_px=int(float(row['n_mask_px']))))
        except (KeyError, ValueError):
            continue
    return out


def nearest(seq, t, tol, key='t'):
    best, bd = None, tol + 1e-9
    for r in seq:
        d = abs(r[key] - t)
        if d < bd:
            best, bd = r, d
    return best


def pearson(xs, ys):
    n = len(xs)
    if n < 3:
        return float('nan')
    mx, my = sum(xs) / n, sum(ys) / n
    sx = math.sqrt(sum((x - mx) ** 2 for x in xs) / n)
    sy = math.sqrt(sum((y - my) ** 2 for y in ys) / n)
    if sx < 1e-12 or sy < 1e-12:
        return float('nan')
    return sum((xs[i] - mx) * (ys[i] - my) for i in range(n)) / (n * sx * sy)


def spearman(xs, ys):
    def ranks(v):
        order = sorted(range(len(v)), key=lambda i: v[i])
        r = [0.0] * len(v)
        i = 0
        while i < len(order):
            j = i
            while j + 1 < len(order) and v[order[j + 1]] == v[order[i]]:
                j += 1
            avg = (i + j) / 2.0 + 1.0
            for k in range(i, j + 1):
                r[order[k]] = avg
            i = j + 1
        return r
    return pearson(ranks(xs), ranks(ys))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    run = sys.argv[1].rstrip('/')
    tol = 0.2
    if '--tol' in sys.argv:
        tol = float(sys.argv[sys.argv.index('--tol') + 1])
    task = os.path.basename(run)
    simlog = SIMLOG % task
    cons, track, obs = parse_simlog(simlog)
    mask = read_mask(os.path.join(run, 'export', 'dynamic_objects.csv'))
    print('run=%s  constraint=%d  track=%d  obs=%d  mask=%d  (tol=%.2fs)'
          % (task, len(cons), len(track), len(obs), len(mask), tol))
    if not cons and not track:
        print('nothing to join: this run has neither [visual-constraint] nor [track-instr]')
        return 1

    # The tracking records are the anchor (they carry the measured side); their simulated
    # time comes from the frame counter, because the line itself is plain stdout.
    rows = []
    for r in track:
        t = r['frame'] / CAMERA_FPS
        c = nearest(cons, t, tol) if cons else None
        m = nearest(mask, t, 1.0) if mask else None
        o = None
        if obs:
            # obs records are keyed by frame number, not by time
            of = nearest(obs, r['frame'], 60.0, key='frame')
            o = of
        rows.append(dict(t=round(t, 3),
                         w_dyn_proxy=(m['n_mask_px'] if m else ''),
                         n_blobs=(m['n_blobs'] if m else ''),
                         covis_edges=(c['covis_edges'] if c else ''),
                         visual_edges=(c['visual_edges'] if c else ''),
                         vertices=(c['vertices'] if c else ''),
                         matched_inliers=r['matched'], N_keypoints=r['N'],
                         valid_depth_kp=(o['valid_depth'] if o else ''),
                         state=r['state'], masked_out=(o['masked_out'] if o else '')))

    out_csv = os.path.join(run, 'mechanism.csv')
    cols = ['t', 'w_dyn_proxy', 'n_blobs', 'covis_edges', 'visual_edges', 'vertices',
            'matched_inliers', 'N_keypoints', 'valid_depth_kp', 'state', 'masked_out']
    with open(out_csv, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)

    # correlations between the predictor side and the measured side
    def pairs(a, b):
        xs, ys = [], []
        for r in rows:
            va, vb = r.get(a), r.get(b)
            if isinstance(va, (int, float)) and isinstance(vb, (int, float)):
                xs.append(float(va))
                ys.append(float(vb))
        return xs, ys

    summary = dict(task=task, n_rows=len(rows), tol_s=tol,
                   source='[visual-constraint] + [track-instr] + dynamic_objects.csv')
    for (a, b, label) in [('w_dyn_proxy', 'matched_inliers', 'masked_px vs matched_inliers'),
                          ('w_dyn_proxy', 'valid_depth_kp', 'masked_px vs valid_depth_kp'),
                          ('w_dyn_proxy', 'N_keypoints', 'masked_px vs N_keypoints'),
                          ('covis_edges', 'matched_inliers', 'covis_edges vs matched_inliers')]:
        xs, ys = pairs(a, b)
        summary[label] = dict(n=len(xs), pearson=pearson(xs, ys), spearman=spearman(xs, ys))
        print('%-32s n=%-4d pearson=%+.3f spearman=%+.3f'
              % (label, len(xs), summary[label]['pearson'], summary[label]['spearman']))

    # binned view on the predictor side
    xs, ys = pairs('w_dyn_proxy', 'matched_inliers')
    if len(xs) >= 8:
        pr = sorted(zip(xs, ys))
        q = max(1, len(pr) // 4)
        print('bins (masked px quartile -> mean matched inliers):')
        bins = []
        for b in range(4):
            chunk = pr[b * q:(b + 1) * q] if b < 3 else pr[3 * q:]
            if not chunk:
                continue
            mean_y = sum(c[1] for c in chunk) / len(chunk)
            bins.append(dict(q=b + 1, lo=chunk[0][0], hi=chunk[-1][0], mean_inliers=mean_y))
            print('  q%d  masked_px %6.0f..%6.0f   mean matched=%.1f'
                  % (b + 1, chunk[0][0], chunk[-1][0], mean_y))
        summary['bins'] = bins

    with open(os.path.join(run, 'mechanism_summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)
    print('wrote %s' % out_csv)
    print('wrote %s' % os.path.join(run, 'mechanism_summary.json'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
