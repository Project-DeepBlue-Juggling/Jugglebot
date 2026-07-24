"""Rung-2a single-throw open-loop accuracy sweep (landing error + separation).

Drives ``sim.juggle_throw.run_single_throw`` (the tilt-aimed open-loop throw:
carry up the tilted cup axis -> detach along it -> fly -> measure landing) across
a grid of target placements and cadences (flight times), under the §3 tracking
noise, and reports the landing error, the clean-separation rate, and the noisy
"observed landing" the Rung-2b catch would reach for. This is the harness that
grounds the throw realisation in ``sim/juggle_throw.py`` and the test thresholds
in ``tests/sim/test_juggle_throw.py``.

THE GATE FRAMING (from the Rung-1 knife-edge, logbook
2026-06-30-rung1-clean-single-catch.md): the throw's landing scatter IS the reach
the catch faces in the Rung-2b loop, and the catch is reliable to ~60-80 mm
reach. So the question this probe answers is "is the throw accurate enough (error
well inside ~60-80 mm) for the self-catch loop to sustain?".

Motivating logbook: ``logbook/2026-06-30-rung2a-single-ball-tilt-throw.md``
(Phase 2 / Rung 2a of ``plans/active/bb-online-juggle-tilt-rearchitecture.md``).

Headless MuJoCo plant, no hardware. Writes a CSV summary to
``temp/probes/juggle_throw_accuracy.csv`` (gitignored under ``temp/``).

Usage (from repo root, project venv active):
    python tools/probes/juggle_throw_accuracy.py                 # default sweep
    python tools/probes/juggle_throw_accuracy.py --radius 0 50 100 --seeds 5
    python tools/probes/juggle_throw_accuracy.py --cadence 0.5 0.6 0.7
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_REPO, os.path.join(_REPO, 'sim'),
           os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_REPO, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from sim.juggle_throw import run_single_throw, SingleThrowConfig  # noqa: E402
from sim.juggle_noise import NoiseConfig  # noqa: E402


def _targets(radii_mm):
    """Home + a ring (N/E/S/W + diagonals) at each radius."""
    out = [(0.0, 0.0)]
    for r in radii_mm:
        if r <= 0:
            continue
        for ang in (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0):
            out.append((round(r / 1000.0 * np.cos(np.radians(ang)), 4),
                        round(r / 1000.0 * np.sin(np.radians(ang)), 4)))
    return out


def sweep(radii_mm, cadences, n_seeds, track_mm):
    rows = []
    for flight in cadences:
        for txy in _targets(radii_mm):
            for s in range(n_seeds):
                noise = NoiseConfig(bb_throw_noise_frac=0.0, tracking_noise_mm=track_mm)
                r = run_single_throw(SingleThrowConfig(
                    target_xy_m=txy, flight_s=flight, seed=s, noise=noise))
                rows.append(dict(
                    flight_s=flight,
                    tgt_x_mm=round(txy[0] * 1000, 1), tgt_y_mm=round(txy[1] * 1000, 1),
                    seed=s, separated=int(r.separated),
                    error_mm=round(r.error_mm, 1), observed_error_mm=round(r.observed_error_mm, 1),
                    reach_mm=round(r.reach_mm, 1), tilt_deg=round(r.tilt_deg, 2),
                    speed_mps=round(r.takeoff_speed_mps, 3),
                    lateral_mps=round(r.lateral_takeoff_mps, 3),
                    cup_speed_mps=round(r.cup_speed_at_release_mps, 3)))
    return rows


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--radius', type=float, nargs='+', default=[0, 50, 100],
                   metavar='MM', help="Target radii to sweep (mm); 0 = column toss")
    p.add_argument('--cadence', type=float, nargs='+', default=[0.6],
                   metavar='S', help="Flight times (cadence) to sweep (s)")
    p.add_argument('--seeds', type=int, default=3, help="Seeds per target (tracking noise)")
    p.add_argument('--track-mm', type=float, default=0.5, help="§3 tracking noise σ (mm)")
    args = p.parse_args(argv)

    rows = sweep(args.radius, args.cadence, args.seeds, args.track_mm)

    out_dir = os.path.join(_REPO, 'temp', 'probes')
    os.makedirs(out_dir, exist_ok=True)
    out_csv = os.path.join(out_dir, 'juggle_throw_accuracy.csv')
    with open(out_csv, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)

    # per (cadence, target) summary
    print(f"{'flight':>6} {'target(mm)':>14} | sep | true_ERR  obs_ERR   reach  tilt")
    seen = []
    for r in rows:
        key = (r['flight_s'], r['tgt_x_mm'], r['tgt_y_mm'])
        if key in seen:
            continue
        seen.append(key)
        grp = [x for x in rows if (x['flight_s'], x['tgt_x_mm'], x['tgt_y_mm']) == key]
        sep = sum(x['separated'] for x in grp)
        es = np.array([x['error_mm'] for x in grp if x['separated']])
        os_ = np.array([x['observed_error_mm'] for x in grp if x['separated']])
        rc = np.array([x['reach_mm'] for x in grp if x['separated']])
        e = f"{es.mean():6.1f}" if es.size else "     -"
        o = f"{os_.mean():6.1f}" if os_.size else "     -"
        rcm = f"{rc.mean():5.0f}" if rc.size else "    -"
        print(f"{key[0]:6.2f} ({key[1]:+6.0f},{key[2]:+6.0f}) | {sep}/{len(grp)} |"
              f" {e}   {o}   {rcm}  {grp[0]['tilt_deg']:.1f}")

    sep_rows = [r for r in rows if r['separated']]
    es = np.array([r['error_mm'] for r in sep_rows]) if sep_rows else np.array([])
    print(f"\nSEPARATED {len(sep_rows)}/{len(rows)} "
          f"({100*len(sep_rows)/len(rows):.0f}%)")
    if es.size:
        within = int(np.sum(es <= 80.0))
        print(f"landing error (separated): mean {es.mean():.1f} mm  "
              f"median {np.median(es):.1f} mm  max {es.max():.1f} mm")
        print(f"within the catch's ~60-80 mm reliable reach: {within}/{len(sep_rows)} "
              f"({100*within/len(sep_rows):.0f}% of separated throws)")
    print(f"CSV -> {out_csv}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
