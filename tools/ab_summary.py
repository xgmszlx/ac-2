#!/usr/bin/env python3
"""Build the final A/B table from the run directories.

    python3 tools/ab_summary.py [--md] [--dirs results/runs/ab_*]

For every run directory it (re)computes `metrics.json` with tools/eval_run.py and then
prints one row per run, plus a per-arm aggregate. `--md` emits a markdown table ready to
paste into REPORT_EXPLORB_TEB.md.

Why a separate summary instead of trusting ab_table.csv: the table written during the run
is produced by an earlier (and lenient) success criterion, and one arm was interrupted
mid-run. This script always recomputes from the九 CSVs, so the report cannot drift from
the evidence.
"""
from __future__ import print_function

import glob
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARMS = ['B0', 'B1', 'Ours', 'Ours_no_cov']
COLS = [
    ('arm', 'arm'), ('n_goal', 'goals'), ('n_reached', 'reached'),
    ('n_aborted', 'aborted'), ('n_cmd', 'cmd_vel'),
    ('path_len_m', 'path m'), ('robot_moved_m', 'net m'),
    ('coverage_m2', 'coverage m2'), ('ate_rmse_m', 'ATE m'),
    ('lost_s', 'lost s'), ('tracking_ok_pct', 'track ok %'),
    ('teb_ms', 'TEB ms'), ('success', 'all reached'),
]


def teb_mean(task):
    """Mean TEB solve time from the throttled [teb-solve] lines of that run."""
    log = os.path.join(ROOT, 'logs', '%s_sim.log' % task)
    val = ''
    if not os.path.exists(log):
        return val
    for line in open(log, errors='ignore'):
        if '[teb-solve]' in line and 'mean=' in line:
            try:
                val = line.split('mean=')[1].split('ms')[0].strip()
            except IndexError:
                pass
    return val


def arm_of(task):
    for a in sorted(ARMS, key=len, reverse=True):
        if task.startswith('ab_%s_' % a):
            return a
    return '?'


def main():
    want_md = '--md' in sys.argv
    pattern = os.path.join(ROOT, 'results', 'runs', 'ab_*')
    dirs = sorted(glob.glob(pattern))
    if not dirs:
        print('no ab_* run directories found')
        return 1

    rows = []
    for d in dirs:
        if not os.path.isdir(d):
            continue
        subprocess.call([sys.executable, os.path.join(ROOT, 'tools', 'eval_run.py'), d],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        mpath = os.path.join(d, 'metrics.json')
        m = json.load(open(mpath)) if os.path.exists(mpath) else {}
        task = os.path.basename(d)
        row = dict(m)
        row.update(task=task, arm=arm_of(task), teb_ms=teb_mean(task))
        rows.append(row)

    if want_md:
        print('| arm | goals | reached | aborted | cmd_vel | path m | net m | coverage m2 | '
              'ATE m | lost s | track ok pct | TEB ms | all reached |')
        print('|---|---|---|---|---|---|---|---|---|---|---|---|---|')
        for r in rows:
            print('| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |' % (
                r['arm'], r.get('n_goal', ''), r.get('n_reached', ''), r.get('n_aborted', ''),
                r.get('n_cmd', ''), _f(r.get('path_len_m')), _f(r.get('robot_moved_m')),
                _f(r.get('coverage_m2')), _f(r.get('ate_rmse_m', 0), 3),
                _f(r.get('lost_s')), _f(r.get('tracking_ok_pct'), 1),
                r.get('teb_ms') or '-', 'yes' if r.get('success') else 'no'))
    else:
        for r in rows:
            print('%-28s arm=%-12s %s' % (r['task'], r['arm'],
                  ' '.join('%s=%s' % (k, r.get(k)) for k, _l in COLS[1:] if k != 'arm')))
    return 0


def _f(v, nd=2):
    try:
        return ('%.' + str(nd) + 'f') % float(v)
    except (TypeError, ValueError):
        return '-'


if __name__ == '__main__':
    sys.exit(main())
