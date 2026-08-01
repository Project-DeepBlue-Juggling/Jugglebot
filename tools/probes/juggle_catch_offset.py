"""Rung-1 single-catch characterisation sweep (in-cup seat offset + clean rate).

Drives ``sim.juggle_catch.run_single_catch`` (the BB-reload single catch:
translate-to-reach + tilt-to-receive + lever-arm-compensated realisation, under
the §3 noise) across a grid of nominal landing placements and seeds, and reports
the clean-catch rate and the in-cup seat-offset distribution. This is the
harness that grounds the catch realisation constants in
``sim/juggle_catch.py::SingleCatchConfig`` and the test thresholds in
``tests/sim/test_juggle_catch.py``.

Motivating logbook: ``logbook/2026-06-30-rung1-clean-single-catch.md``
(Phase 1 / Rung 1 of ``plans/active/bb-online-juggle-tilt-rearchitecture.md``).

Headless MuJoCo plant, no hardware. Writes a CSV summary to
``temp/probes/juggle_catch_offset.csv`` (gitignored under ``temp/``).

Usage (from repo root, project venv active):
    python tools/probes/juggle_catch_offset.py                 # default sweep
    python tools/probes/juggle_catch_offset.py --seeds 20      # default placement, 20 seeds
    python tools/probes/juggle_catch_offset.py --radius 0 40 80 --seeds 5
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

_repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
from sim._paths import bootstrap_paths  # noqa: E402
_REPO = bootstrap_paths()

from sim.juggle_catch import run_single_catch, SingleCatchConfig  # noqa: E402
from sim.juggle_noise import NoiseConfig  # noqa: E402


def _placements(radii_mm):
    """Nominal landings: home + a ring of points at each radius (N/E/S/W)."""
    out = [(0.0, 0.0)]
    for r in radii_mm:
        if r <= 0:
            continue
        for ang in (0.0, 90.0, 180.0, 270.0):
            out.append((r / 1000.0 * np.cos(np.radians(ang)),
                        r / 1000.0 * np.sin(np.radians(ang))))
    return out


def sweep(radii_mm, n_seeds, bb_frac, track_mm):
    noise = NoiseConfig(bb_throw_noise_frac=bb_frac, tracking_noise_mm=track_mm)
    rows = []
    for lxy in _placements(radii_mm):
        for s in range(n_seeds):
            r = run_single_catch(SingleCatchConfig(landing_xy_m=lxy, seed=s, noise=noise))
            rows.append(dict(
                land_x_mm=round(lxy[0] * 1000, 1), land_y_mm=round(lxy[1] * 1000, 1),
                seed=s, clean=int(r.clean), caught=int(r.caught),
                held=int(r.held_at_end), in_cup_offset_mm=round(r.in_cup_offset_mm, 2),
                lateral_mm=round(r.lateral_offset_mm, 2),
                vertical_mm=round(r.vertical_offset_mm, 2),
                reach_mm=round(r.reach_mm, 1), tilt_deg=round(r.catch_tilt_deg, 2)))
    return rows


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--radius', type=float, nargs='+', default=[0, 40, 60, 80, 100],
                   metavar='MM', help="Reach radii to sweep (mm); 0 = home only")
    p.add_argument('--seeds', type=int, default=5, help="Seeds per placement")
    p.add_argument('--bb-frac', type=float, default=0.02, help="§3 BB throw noise fraction")
    p.add_argument('--track-mm', type=float, default=0.5, help="§3 tracking noise σ (mm)")
    args = p.parse_args(argv)

    rows = sweep(args.radius, args.seeds, args.bb_frac, args.track_mm)
    clean = [r for r in rows if r['clean']]
    offs = np.array([r['in_cup_offset_mm'] for r in clean]) if clean else np.array([])

    out_dir = os.path.join(_REPO, 'temp', 'probes')
    os.makedirs(out_dir, exist_ok=True)
    out_csv = os.path.join(out_dir, 'juggle_catch_offset.csv')
    with open(out_csv, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)

    # per-placement summary
    print(f"{'land (mm)':>14} | clean | mean_off  max_off")
    seen = []
    for r in rows:
        key = (r['land_x_mm'], r['land_y_mm'])
        if key in seen:
            continue
        seen.append(key)
        grp = [x for x in rows if (x['land_x_mm'], x['land_y_mm']) == key]
        co = [x['in_cup_offset_mm'] for x in grp if x['clean']]
        mo = f"{np.mean(co):6.2f}" if co else "     -"
        mx = f"{np.max(co):6.2f}" if co else "     -"
        print(f"({key[0]:+6.0f},{key[1]:+6.0f}) | {sum(x['clean'] for x in grp)}/{len(grp)}  "
              f" | {mo}   {mx}")
    print(f"\nTOTAL clean {len(clean)}/{len(rows)} "
          f"({100*len(clean)/len(rows):.0f}%)")
    if offs.size:
        print(f"in-cup offset (clean): mean {offs.mean():.2f} mm  "
              f"median {np.median(offs):.2f} mm  max {offs.max():.2f} mm")
    print(f"CSV -> {out_csv}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
