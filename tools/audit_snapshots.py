#!/usr/bin/env python3
"""Audit a set of read-only run snapshots with the *internal* tracking-state metric.

    python3 tools/audit_snapshots.py /home/wcqw/.dsh_heavy/Asnap 'ab_*_visual_r1__*'

Why this exists: an overwritten run directory can still be audited if somebody kept a
read-only copy. The orchestrator keeps such copies under ~/.dsh_heavy/Asnap/<task>__<wall>/
(goals.csv + the *_sim.log of that episode), so the *internal* tracking state of a lost run
can be recomputed after the fact.

The metric is the same as tools/eval_run.py: `[obs-instr] … state=` is ORB-SLAM's
`Tracking::eTrackingState` (2 = OK, 3 = LOST); the duration of each non-OK run is measured
in frames and converted at the camera rate. The published /info/state is NOT used (it flaps
4<->5 for publisher reasons on this platform).
"""
from __future__ import print_function

import glob
import os
import re
import sys

INSTR = re.compile(r'\[obs-instr\] frame=(\d+) N=(\d+) valid_depth_kp=(\d+) '
                   r'depth_px_sampled_valid=(\d+) state=(-?\d+)')
EDGES = re.compile(r'\[visual-constraint\].*visual_edges=(\d+) covis_edges=(\d+)')
SOLVE = re.compile(r'\[teb-solve\].*mean=([\d.]+) ms')
AIM = re.compile(r'alpha')          # unused placeholder kept out of the hot loop
CAMERA_FPS = 30.0


def audit(dirpath):
    sims = glob.glob(os.path.join(dirpath, '*_sim.log'))
    if not sims:
        return None
    sim = sims[0]
    recs, states, teb, edges = [], {}, [], None
    for line in open(sim, errors='ignore'):
        m = INSTR.search(line)
        if m:
            f, st = int(m.group(1)), int(m.group(5))
            recs.append((f, st))
            states[st] = states.get(st, 0) + 1
        m = SOLVE.search(line)
        if m:
            teb.append(float(m.group(1)))
        m = EDGES.search(line)
        if m:
            edges = (int(m.group(1)), int(m.group(2)))
    if len(recs) < 3:
        return None
    lost, prev, nok = 0.0, None, 0
    for (f, st) in recs:
        if st == 2:
            nok += 1
        elif prev is not None:
            lost += (f - prev) / CAMERA_FPS
        prev = f
    goals = results = 0
    gpath = os.path.join(dirpath, 'goals.csv')
    if os.path.exists(gpath):
        txt = open(gpath, errors='ignore').read()
        goals = txt.count(',goal,')
        results = txt.count(',result,')
    return dict(name=os.path.basename(dirpath), n=len(recs), ok_pct=100.0 * nok / len(recs),
                lost_internal_s=lost, state_hist=states, goals=goals, results=results,
                teb_mean=(sum(teb) / len(teb)) if teb else None, edges=edges)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    root = sys.argv[1]
    pattern = sys.argv[2] if len(sys.argv) > 2 else '*'
    rows = []
    for d in sorted(glob.glob(os.path.join(root, pattern))):
        r = audit(d)
        if r:
            rows.append(r)
    if not rows:
        print('no snapshot with an [obs-instr] log matched %s/%s' % (root, pattern))
        return 1
    print('%-36s %-5s %-8s %-14s %-8s %-8s %s'
          % ('snapshot', 'n', 'ok %', 'lost_internal_s', 'goals', 'results', 'state_hist'))
    for r in rows:
        print('%-36s %-5d %-8.2f %-14.2f %-8d %-8d %s'
              % (r['name'], r['n'], r['ok_pct'], r['lost_internal_s'], r['goals'],
                 r['results'], r['state_hist']))
    bad = [r for r in rows if r['lost_internal_s'] > 1.0]
    print('\n%d/%d snapshots show a real internal tracking loss (>1 s)' % (len(bad), len(rows)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
