#!/usr/bin/env python3
"""TEMPORARY (BUG-57 diagnostic capture): view a full-rate Hand capture CSV.

Reads a CSV written by teensy_bridge_node (launch arg ``hand_capture:=true`` ->
~/Desktop/Jugglebot/temp/hand_capture/hand_capture_<ts>.csv) and prints a
merged, time-ordered table of telem / diag / echo rows.

  python tools/probes/hand_capture_view.py                     # newest file, window around peak |vel|
  python tools/probes/hand_capture_view.py FILE --t0 12.3 --t1 12.8   # seconds since first row
  python tools/probes/hand_capture_view.py FILE --window 0.4 --png    # +-0.4 s about the peak, save PNG

Time axis is Jetson receive time (t_jetson_mono_s, relative to the first row);
t_src_us is the Teensy/bridge stamp where the frame carries one (telem, echo).
Pure stdlib + optional matplotlib (--png -> temp/probes/).
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_DIR = os.path.join(REPO, 'temp', 'hand_capture')


def load(path):
    with open(path, newline='') as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        return [], 0.0
    t0 = float(rows[0]['t_jetson_mono_s'])
    for r in rows:
        r['t'] = float(r['t_jetson_mono_s']) - t0
    rows.sort(key=lambda r: r['t'])
    return rows, t0


def _f(r, k):
    v = r.get(k, '')
    return float(v) if v not in ('', None) else None


def peak_time(rows):
    """Jetson-relative time of the largest |vel_rps| among telem rows (None if none)."""
    best, tb = -1.0, None
    for r in rows:
        if r['kind'] == 'telem':
            v = _f(r, 'vel_rps')
            if v is not None and abs(v) > best:
                best, tb = abs(v), r['t']
    return tb, best


def select(rows, t0=None, t1=None, window=0.25):
    if t0 is None or t1 is None:
        tp, _ = peak_time(rows)
        if tp is None:
            return rows
        t0, t1 = tp - window, tp + window
    return [r for r in rows if t0 <= r['t'] <= t1]


def fmt_table(rows):
    cols = ['t_s', 'kind', 't_src_us', 'pos_rev', 'vel_rps', 'iq_set', 'iq_meas',
            'cmd_pos', 'cmd_vel_ff', 'cmd_tor', 'err']
    keys = [None, 'kind', 't_src_us', 'pos_rev', 'vel_rps', 'iq_setpoint', 'iq_measured',
            'cmd_pos_rev', 'cmd_vel_ff_rps', 'cmd_tor_ff', 'active_errors']
    out = ['  '.join(f'{c:>11}' for c in cols)]
    for r in rows:
        cells = [f"{r['t']:11.4f}"]
        for k in keys[1:]:
            v = r.get(k, '')
            if v in ('', None):
                cells.append(f"{'':>11}")
            elif k in ('kind', 't_src_us', 'active_errors'):
                cells.append(f'{v:>11}')
            else:
                cells.append(f'{float(v):11.4f}')
        out.append('  '.join(cells))
    return '\n'.join(out)


def rates(rows):
    span = (rows[-1]['t'] - rows[0]['t']) if len(rows) > 1 else 0.0
    out = {}
    for r in rows:
        out[r['kind']] = out.get(r['kind'], 0) + 1
    return {k: (n, n / span if span > 0 else float('nan')) for k, n in out.items()}


def save_png(rows, sel, path_csv):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
    def series(kind, key):
        pts = [(r['t'], _f(r, key)) for r in sel if r['kind'] == kind and _f(r, key) is not None]
        return [p[0] for p in pts], [p[1] for p in pts]
    ax[0].plot(*series('telem', 'vel_rps'), '.-', label='vel_rps (meas)')
    ax[0].plot(*series('echo', 'cmd_vel_ff_rps'), '.-', label='cmd vel_ff'); ax[0].legend(); ax[0].set_ylabel('rev/s')
    ax[1].plot(*series('telem', 'pos_rev'), '.-', label='pos_rev (meas)')
    ax[1].plot(*series('echo', 'cmd_pos_rev'), '.-', label='cmd pos'); ax[1].legend(); ax[1].set_ylabel('rev')
    ax[2].plot(*series('diag', 'iq_measured'), '.-', label='iq_measured')
    ax[2].plot(*series('diag', 'iq_setpoint'), '.-', label='iq_setpoint'); ax[2].legend(); ax[2].set_ylabel('A')
    ax[2].set_xlabel('Jetson receive time since first row (s)')
    fig.suptitle(os.path.basename(path_csv)); fig.tight_layout()
    outdir = os.path.join(REPO, 'temp', 'probes'); os.makedirs(outdir, exist_ok=True)
    out = os.path.join(outdir, os.path.basename(path_csv).replace('.csv', '_view.png'))
    fig.savefig(out, dpi=110)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('csv', nargs='?', help='capture CSV (default: newest in temp/hand_capture/)')
    ap.add_argument('--t0', type=float); ap.add_argument('--t1', type=float)
    ap.add_argument('--window', type=float, default=0.25, help='half-width about the peak (s)')
    ap.add_argument('--png', action='store_true')
    a = ap.parse_args(argv)
    path = a.csv
    if not path:
        files = sorted(glob.glob(os.path.join(DEFAULT_DIR, 'hand_capture_*.csv')))
        if not files:
            print(f'no captures in {DEFAULT_DIR}', file=sys.stderr); return 1
        path = files[-1]
    rows, _ = load(path)
    if not rows:
        print(f'{path}: empty'); return 1
    print(f'{path}: {len(rows)} rows, span {rows[-1]["t"]:.2f} s, '
          f'dropped_total {rows[-1].get("dropped_total", "?")}')
    for k, (n, hz) in rates(rows).items():
        print(f'  {k}: {n} rows, {hz:.1f} Hz mean')
    tp, vp = peak_time(rows)
    if tp is not None:
        print(f'  peak |vel| {vp:.2f} rev/s at t={tp:.4f} s')
    sel = select(rows, a.t0, a.t1, a.window)
    print(fmt_table(sel))
    if a.png:
        print('PNG:', save_png(rows, sel, path))
    return 0


if __name__ == '__main__':
    sys.exit(main())
