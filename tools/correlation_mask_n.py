#!/usr/bin/env python3
"""Correlation between the *predicted* dynamic occlusion and the frontend's *actual*
observation count.

The task book asks for a "predicted occlusion / co-visibility vs actual tracking
inliers" correlation log. What is available without touching ORB-SLAM internals:

  predicted  the dynamic-mask coverage per frame, from export/dynamic_objects.csv
             (`n_mask_px`, and `n_blobs`) -- the same quantity the C++ query's
             w_dyn term is built from, measured from depth + ego-motion only
  actual     the frontend's usable observations per frame, from the `[obs-instr]`
             line in the sim log: `N` (ORB keypoints) and `valid_depth_kp`
             (keypoints with a usable depth = the ones that can become map points)

If the prediction means anything, more masked (occluded) pixels must go with fewer
usable observations. The script reports Pearson and Spearman correlation plus a
binned table, and writes export/occlusion_vs_inliers.csv.

Usage: python3 tools/correlation_mask_n.py results/runs/<task>
"""
from __future__ import print_function

import csv
import math
import os
import re
import sys

INSTR = re.compile(r'\[obs-instr\] frame=(\d+) N=(\d+) valid_depth_kp=(\d+) '
                   r'depth_px_sampled_valid=(\d+) state=(-?\d+)')


def read_frontend(simlog):
    out = []
    if not os.path.exists(simlog):
        return out
    for line in open(simlog, errors='ignore'):
        m = INSTR.search(line)
        if m:
            out.append(dict(frame=int(m.group(1)), N=int(m.group(2)),
                            valid_depth=int(m.group(3)), state=int(m.group(5))))
    return out


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


def pearson(xs, ys):
    n = len(xs)
    if n < 3:
        return float('nan')
    mx = sum(xs) / n
    my = sum(ys) / n
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
    task = os.path.basename(run)
    simlog = os.path.join(os.path.dirname(os.path.dirname(run)), 'logs', task + '_sim.log')
    if not os.path.exists(simlog):
        simlog = '/home/wcqw/explorb_line_20261121/logs/%s_sim.log' % task

    fe = read_frontend(simlog)
    mk = read_mask(os.path.join(run, 'export', 'dynamic_objects.csv'))
    print('run=%s frontend_frames=%d mask_samples=%d' % (task, len(fe), len(mk)))
    if len(fe) < 10 or len(mk) < 10:
        print('not enough samples for a correlation (needs a run with the mask node active)')
        return 1

    # Pair by *time*, not by index: the frontend logs every 60 frames and the mask node
    # publishes at 10 Hz, so index pairing would compare unrelated instants. The frame
    # counter runs at the camera rate (30 fps on this platform), and the mask CSV carries
    # the simulated time, so frame/30 is the frontend sample's simulated time.
    fps = 30.0
    mk_sorted = sorted(mk, key=lambda r: r['t'])
    def nearest_mask(t):
        best = min(mk_sorted, key=lambda r: abs(r['t'] - t))
        return best if abs(best['t'] - t) < 0.5 else None

    xs, ny, nd, times = [], [], [], []
    for rec in fe:
        if rec['state'] != 2:      # only while the frontend is actually tracking
            continue
        t = rec['frame'] / fps
        m = nearest_mask(t)
        if m is None:
            continue
        xs.append(float(m['n_mask_px']))
        ny.append(float(rec['N']))
        nd.append(float(rec['valid_depth']))
        times.append(t)
    n = len(xs)

    r_n = pearson(xs, ny)
    rho_n = spearman(xs, ny)
    r_d = pearson(xs, nd)
    rho_d = spearman(xs, nd)
    print('masked px vs N            : pearson=%+.3f spearman=%+.3f' % (r_n, rho_n))
    print('masked px vs valid_depth  : pearson=%+.3f spearman=%+.3f' % (r_d, rho_d))

    # binned view (quartiles of the predicted occlusion)
    pairs = sorted(zip(xs, nd))
    q = len(pairs) // 4
    if q >= 2:
        print('bin (masked px quartile) -> mean usable depth keypoints')
        for b in range(4):
            chunk = pairs[b * q:(b + 1) * q] if b < 3 else pairs[3 * q:]
            if not chunk:
                continue
            print('  q%d  masked_px %6.0f..%6.0f   mean valid_depth_kp=%.1f'
                  % (b + 1, chunk[0][0], chunk[-1][0],
                     sum(c[1] for c in chunk) / len(chunk)))

    out = os.path.join(run, 'export', 'occlusion_vs_inliers.csv')
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['i', 'frame', 'n_mask_px', 'N', 'valid_depth_kp'])
        for i in range(n):
            w.writerow([i, fe[i]['frame'], int(xs[i]), int(ny[i]), int(nd[i])])
    print('wrote %s' % out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
