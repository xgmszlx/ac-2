#!/usr/bin/env python3
"""Add the per-run metrics the reviewer asked for, straight from the logs.

    python3 tools/run_extras.py [results/runs/*]

For every run directory this adds to metrics.json:

    arrival_time_s        dispatch -> move_base result, for the goals that reached (s)
    teb_p50_ms/teb_p95_ms/teb_max_ms   from every [teb-solve] sample (the `last=` field
                                       is one control cycle, so the distribution is real,
                                       not an average of averages)
    visual_edges_mean, covis_edges_mean, vertices_mean   from [visual-constraint]

Nothing here touches the simulation: it only parses the logs that already exist.
"""
from __future__ import print_function

import csv
import glob
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIMLOG = os.path.join(ROOT, 'logs', '%s_sim.log')

RE_SOLVE = re.compile(r'\[teb-solve\][^\n]*?last=([\d.]+)\s*ms')
RE_CONS = re.compile(r'\[visual-constraint\].*vertices=(\d+) visual_edges=(\d+) covis_edges=(\d+)')


def pct(vals, q):
    if not vals:
        return None
    s = sorted(vals)
    k = max(0, min(len(s) - 1, int(round((q / 100.0) * (len(s) - 1)))))
    return s[k]


def extras(rundir):
    task = os.path.basename(rundir)
    out = {}
    sim = SIMLOG % task
    solves, verts, ve, ce = [], [], [], []
    if os.path.exists(sim):
        for line in open(sim, errors='ignore'):
            m = RE_SOLVE.search(line)
            if m:
                try:
                    solves.append(float(m.group(1)))
                except ValueError:
                    pass
                continue
            m = RE_CONS.search(line)
            if m:
                verts.append(int(m.group(1)))
                ve.append(int(m.group(2)))
                ce.append(int(m.group(3)))
    if solves:
        out['teb_n_samples'] = len(solves)
        out['teb_p50_ms'] = round(pct(solves, 50), 1)
        out['teb_p95_ms'] = round(pct(solves, 95), 1)
        out['teb_max_ms'] = round(max(solves), 1)
    if ve:
        out['visual_edges_mean'] = round(sum(ve) / len(ve), 1)
        out['covis_edges_mean'] = round(sum(ce) / len(ce), 1)
        out['vertices_mean'] = round(sum(verts) / len(verts), 1)
        out['edge_samples'] = len(ve)

    gpath = os.path.join(rundir, 'goals.csv')
    if os.path.exists(gpath):
        rows = list(csv.DictReader(open(gpath)))
        gs = [r for r in rows if r.get('kind') == 'goal']
        rs = [r for r in rows if r.get('kind') == 'result']
        arrivals = []
        for i, g in enumerate(gs):
            if i >= len(rs):
                break
            try:
                if int(float(rs[i].get('status', -1))) == 3:
                    arrivals.append(float(rs[i]['t']) - float(g['t']))
            except (TypeError, ValueError):
                continue
        if arrivals:
            out['arrival_time_s'] = round(sum(arrivals) / len(arrivals), 2)
            out['arrival_time_max_s'] = round(max(arrivals), 2)
            out['n_arrivals_timed'] = len(arrivals)
    return out


def main():
    dirs = sys.argv[1:] or sorted(glob.glob(os.path.join(ROOT, 'results', 'runs', '*')))
    n = 0
    for d in dirs:
        if not os.path.isdir(d):
            continue
        mpath = os.path.join(d, 'metrics.json')
        m = {}
        if os.path.exists(mpath):
            try:
                m = json.load(open(mpath))
            except ValueError:
                m = {}
        e = extras(d)
        if not e and not m:
            continue
        m.update(e)
        with open(mpath, 'w') as f:
            json.dump(m, f, indent=2)
        n += 1
    print('updated %d metrics.json with arrival/teb-distribution/edge fields' % n)
    return 0


if __name__ == '__main__':
    sys.exit(main())
