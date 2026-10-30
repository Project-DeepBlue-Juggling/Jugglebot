#!/usr/bin/env python3
"""Per-rung landing-bias read for the pre-release-hold ladder (map JB-102, runbook
``tests/hardware/session_oval_rung1.md`` § 11).

Reads a learner ``memory.csv`` (``temp/learn/<plant_id>/memory.csv``; columns
x0..x3,u0,u1,u2_apex_m,y0,y1,y2_apex_m,t_abs_s,ball_id,caught; u = commanded
landing, y = measured landing, metres) and prints, per rung, n, caught/n and the
mean and sd of the plant residual (y - u) in mm on x, y and apex.  With
``--baseline <plant-id>`` it prints the rung-minus-baseline difference with
SE = sqrt(sd1^2/n1 + sd2^2/n2) and the pre-registered verdict:

  delay effect confirmed : the x difference is short (negative) by >= 10 mm AND
                           |dy| < 10 mm AND |d apex| < 10 mm AND beyond 2 SE
  no effect              : |dx| < 10 mm and |dy|, |d apex| < 10 mm
  inconclusive           : n < 20 on either rung, or |dx| within 2 SE

The residual y - u does not move when the learner adapts (it is the plant error
against the commanded landing), which is why it is the primary read.  Rows are
whatever is in the file (every row is a throw with a measured landing); use
``--last N`` / ``--since-t T`` to window.  Axes are the robot frame: x is the
throw axis for the oval (sites at +-75 mm on x).

Pure stdlib, deterministic.  Run with ``python -I``.
"""
import argparse
import csv
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
COLS = (('x', 'y0', 'u0'), ('y', 'y1', 'u1'), ('apex', 'y2_apex_m', 'u2_apex_m'))


def load(path, last=None, since_t=None):
    with open(path, newline='') as fh:
        rows = list(csv.DictReader(fh))
    if since_t is not None:
        rows = [r for r in rows if float(r['t_abs_s']) >= since_t]
    if last is not None:
        rows = rows[-last:]
    return rows


def stats(vals):
    n = len(vals)
    if n == 0:
        return 0, float('nan'), float('nan')
    m = sum(vals) / n
    sd = math.sqrt(sum((v - m) ** 2 for v in vals) / (n - 1)) if n > 1 else 0.0
    return n, m, sd


def residuals(rows):
    return {name: [1000.0 * (float(r[y]) - float(r[u])) for r in rows]
            for name, y, u in COLS}


def summarise(label, rows):
    caught = sum(1 for r in rows if r['caught'].strip().lower() in ('true', '1'))
    res = residuals(rows)
    print('%s: n=%d caught=%d/%d' % (label, len(rows), caught, len(rows)))
    out = {}
    for name, _, _ in COLS:
        n, m, sd = stats(res[name])
        out[name] = (n, m, sd)
        print('  (y-u) %-4s mean %+7.1f mm  sd %5.1f mm' % (name, m, sd))
    return out


def verdict(a, b):
    """a = rung, b = baseline (both from summarise)."""
    n1, n2 = a['x'][0], b['x'][0]
    d = {k: a[k][1] - b[k][1] for k in a}
    se = {k: math.sqrt(a[k][2] ** 2 / max(n1, 1) + b[k][2] ** 2 / max(n2, 1)) for k in a}
    print('delta (rung - baseline), SE = sqrt(sd1^2/n1 + sd2^2/n2):')
    for k in ('x', 'y', 'apex'):
        print('  d%-4s %+7.1f mm  SE %4.1f mm' % (k, d[k], se[k]))
    if n1 < 20 or n2 < 20 or abs(d['x']) <= 2 * se['x']:
        if n1 >= 20 and n2 >= 20 and abs(d['x']) < 10 and abs(d['y']) < 10 and abs(d['apex']) < 10:
            return 'VERDICT: no effect (|dx| < 10 mm, within 2 SE)'
        return 'VERDICT: inconclusive (n < 20 or |dx| within 2 SE)'
    if d['x'] <= -10 and abs(d['y']) < 10 and abs(d['apex']) < 10:
        return 'VERDICT: delay effect confirmed (dx short by >= 10 mm, dy/dapex < 10 mm)'
    if abs(d['x']) < 10 and abs(d['y']) < 10 and abs(d['apex']) < 10:
        return 'VERDICT: no effect'
    return 'VERDICT: inconclusive (shift outside the delay signature: check dy / dapex / sign)'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--plant-id', help='reads <repo>/temp/learn/<id>/memory.csv')
    ap.add_argument('--file', help='explicit memory.csv path (overrides --plant-id)')
    ap.add_argument('--last', type=int, help='only the last N rows')
    ap.add_argument('--since-t', type=float, help='only rows with t_abs_s >= T')
    ap.add_argument('--baseline', help='baseline plant-id (same windowing)')
    a = ap.parse_args(argv)
    if not (a.file or a.plant_id):
        ap.error('give --plant-id or --file')
    path = a.file or os.path.join(REPO, 'temp', 'learn', a.plant_id, 'memory.csv')
    if not os.path.isfile(path):
        sys.exit('no memory file at %s' % path)
    rows = load(path, a.last, a.since_t)
    rung = summarise(a.plant_id or path, rows)
    if a.baseline:
        bpath = os.path.join(REPO, 'temp', 'learn', a.baseline, 'memory.csv')
        if not os.path.isfile(bpath):
            sys.exit('no memory file at %s' % bpath)
        base = summarise(a.baseline, load(bpath, a.last, a.since_t))
        print(verdict(rung, base))
    return 0


if __name__ == '__main__':
    sys.exit(main())
