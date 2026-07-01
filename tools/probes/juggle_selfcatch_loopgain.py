"""Rung-2b single-ball self-catch loop-gain sweep (the MAKE-OR-BREAK evidence).

Drives ``sim/juggle_selfcatch.py::run_self_catch`` (compose the Rung-2a tilt-aimed
throw with the Rung-1 catch into a single-ball toss->catch->toss loop, re-planned
each cycle under the §3 tracking noise) and reports the per-cycle loop-gain trend
+ sustained cycle count — what the make-or-break gate reads. The gate needs >= 10
sustained cycles with a flat/decaying trend (loop gain < 1).

Two families of runs, both a BREAK:

1. **COLUMN** (``oscillate=False``, the original 2026-07-01 gate): throw+catch
   co-located, tilt ~0. Sweeps the three recover/geometry variants. Reports the
   per-cycle **reach trend**. Finding: does NOT sustain — every seed/variant
   diverges within 0-3 cycles (reach amplifies ~8 -> ~50 -> ~110 -> ~210 mm past
   the catch's ~60-80 mm reliable reach -> drop). But a column commands ~0 tilt,
   so this is a DEGENERATE test — the tilt mechanism never engages.

2. **OSCILLATION** (``oscillate=True``, the operator's OPTION-1 re-plan): shuttle a
   single ball between two lateral points A and B, so every throw is LATERAL and
   the commanded tilt is NON-ZERO (tilt ENGAGES). The honest tilt test on a
   NON-degenerate geometry. Sweeps A<->B axes/separations. Reports the per-cycle
   **in-cup-offset trend** AND the **landing-error trend**. Finding: STILL a BREAK
   — diverges within 1-4 cycles (max sustained 4 of >= 10, at x-20) at every
   separation (20-70 mm) / axis WITH tilt engaged. The
   in-cup SEAT offset stays small (~0.5-2 mm — tilt keeps the ball centred) but the
   LANDING amplifies (e.g. x-40 seed 1: 3.7 -> 89 -> 728 mm). Loop gain > 1.

Head-to-head reference (all three diverge, loop gain > 1):
* the **pre-tilt divergence** (logbook 2026-06-27-online-juggle-throw-fix-...):
  a caught-then-thrown ball walked off > 150 mm/cycle — the BAND-LIMITED level
  throw couldn't aim, so the ~15 mm catch offset amplified. Tilt was the fix.
* the **column BREAK** (logbook 2026-07-01-rung2b-selfcatch-column-divergence):
  reach amplified 8 -> 210 mm; tilt ~0 (degenerate, tilt inactive).
* the **oscillation BREAK** (logbook 2026-07-01-rung2b-oscillation-tilt-engaged-
  diverges): landing amplified 3.7 -> 728 mm WITH tilt engaged (~1.4 deg). Tilt
  fixes the band-limit but NOT the contact-detach's chaotic sensitivity to the
  throw-ORIGIN pose (dLanding/dOrigin ~4, up to ~11), which is the binding
  amplification off-origin.

Grounds ``tests/sim/test_juggle_selfcatch.py``. Headless MuJoCo plant, no hardware.
Writes ``temp/probes/juggle_selfcatch_loopgain.csv``. Run from the repo root:

    python tools/probes/juggle_selfcatch_loopgain.py [--seeds N] [--cycles N]
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

_repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
for _p in (_repo_root, os.path.join(_repo_root, 'sim'),
           os.path.join(_repo_root, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_repo_root, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from sim.juggle_selfcatch import run_self_catch, SelfCatchConfig, CATCH_REACH_MM

# The three COLUMN variants behind the original gate: the FAITHFUL composition
# (Rung-2a plan_cup_cycle recover, stationary column with reposition-to-origin),
# the DRIFT variant (throw in-place from the caught xy -> no reposition), and the
# RETRACT variant (an ad-hoc axial slider retract — a knife-edge).
COLUMN_VARIANTS = {
    "faithful (plan, stationary)": dict(recover="plan", stationary=True),
    "drift (plan, in-place)":       dict(recover="plan", stationary=False),
    "retract (stationary)":         dict(recover="retract", stationary=True),
}

# The OSCILLATION geometries (the OPTION-1 re-plan): A<->B two-point shuttles where
# tilt ENGAGES. Axes x / y / diagonal, separations 20-70 mm — the full range behind
# the BREAK verdict, swept in ONE committed run so the "every separation (20-70 mm)
# and axis (x/y/diagonal)" gate claim is reproducible from this probe alone. The x
# axis is swept across the whole range (20/30/40/50/60/70 mm); y and diagonal pin the
# axis breadth at representative separations. The 40 mm x-default composes cyc 0
# cleanly (lands ~3.7 mm off B); none of the eight sustains (max 4 of >=10, at x-20).
# The cyc-0 landing error is NON-monotonic in separation (43.9 mm at x-50, 22.8 at
# x-60, 11.7 at x-70) — the sign-changing off-origin asymmetry, not "worse with size".
OSC_GEOMS = {
    "osc x-20 (A=-10,B=+10)":  dict(osc_point_a_m=(-0.010, 0.0),   osc_point_b_m=(0.010, 0.0)),
    "osc x-30 (A=-15,B=+15)":  dict(osc_point_a_m=(-0.015, 0.0),   osc_point_b_m=(0.015, 0.0)),
    "osc x-40 (A=-20,B=+20)":  dict(osc_point_a_m=(-0.020, 0.0),   osc_point_b_m=(0.020, 0.0)),
    "osc x-50 (A=-25,B=+25)":  dict(osc_point_a_m=(-0.025, 0.0),   osc_point_b_m=(0.025, 0.0)),
    "osc x-60 (A=-30,B=+30)":  dict(osc_point_a_m=(-0.030, 0.0),   osc_point_b_m=(0.030, 0.0)),
    "osc x-70 (A=-35,B=+35)":  dict(osc_point_a_m=(-0.035, 0.0),   osc_point_b_m=(0.035, 0.0)),
    "osc y-40 (A=0,-20)":      dict(osc_point_a_m=(0.0, -0.020),   osc_point_b_m=(0.0, 0.020)),
    "osc diag-50":             dict(osc_point_a_m=(-0.018, -0.018), osc_point_b_m=(0.018, 0.018)),
}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seeds', type=int, default=6, help="number of seeds (default 6)")
    ap.add_argument('--cycles', type=int, default=12, help="cycles/run (default 12)")
    args = ap.parse_args(argv)

    out_dir = os.path.join(_repo_root, 'temp', 'probes')
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, 'juggle_selfcatch_loopgain.csv')

    rows = []
    print(f"Rung-2b self-catch loop-gain sweep — {args.seeds} seeds x {args.cycles} cycles\n"
          f"gate: sustained >= 10 with a flat/decaying trend (loop gain < 1)\n")

    print("=== COLUMN (tilt ~0, DEGENERATE test — reach trend) ===")
    for name, kw in COLUMN_VARIANTS.items():
        print(f"--- {name} ---")
        sustained_counts = []
        for seed in range(args.seeds):
            r = run_self_catch(SelfCatchConfig(seed=seed, n_cycles=args.cycles, **kw))
            sustained_counts.append(r.sustained)
            print(f"  seed {seed}: sustained {r.sustained:2d}/{args.cycles}  "
                  f"reach_trend_mm={r.reach_trend_mm}")
            for c in r.cycles:
                rows.append(dict(
                    mode="column", geom=name, seed=seed, cyc=c.cyc,
                    separated=c.separated, caught=c.caught, held=c.held_at_end,
                    tilt_deg=round(c.tilt_deg, 2),
                    in_off_start_mm=round(c.in_off_start_mm, 2),
                    in_off_end_mm=round(c.in_off_end_mm, 2),
                    reach_mm=round(c.reach_mm, 2),
                    reach_from_throw_mm=round(c.reach_from_throw_mm, 2),
                    landing_err_mm=round(c.landing_err_mm, 2),
                    seat_offset_mm=round(c.seat_offset_mm, 2)))
        mx = max(sustained_counts)
        verdict = "SUSTAINS (>=10)" if mx >= 10 else "DIVERGES (< 10) -> BREAK"
        print(f"  => sustained {sustained_counts}, max {mx}  [{verdict}]\n")

    print("=== OSCILLATION (tilt ENGAGED, honest test — in-cup-offset + landing-error trends) ===")
    for name, kw in OSC_GEOMS.items():
        print(f"--- {name} ---")
        sustained_counts = []
        for seed in range(args.seeds):
            r = run_self_catch(SelfCatchConfig(seed=seed, n_cycles=args.cycles,
                                               oscillate=True, **kw))
            sustained_counts.append(r.sustained)
            tilt = r.cycles[0].tilt_deg if r.cycles else 0.0
            print(f"  seed {seed}: sustained {r.sustained:2d}/{args.cycles}  tilt={tilt:.2f}deg  "
                  f"in_off_end_mm={r.in_off_trend_mm}  land_err_mm={r.landing_err_trend_mm}")
            for c in r.cycles:
                rows.append(dict(
                    mode="oscillation", geom=name, seed=seed, cyc=c.cyc,
                    separated=c.separated, caught=c.caught, held=c.held_at_end,
                    tilt_deg=round(c.tilt_deg, 2),
                    in_off_start_mm=round(c.in_off_start_mm, 2),
                    in_off_end_mm=round(c.in_off_end_mm, 2),
                    reach_mm=round(c.reach_mm, 2),
                    reach_from_throw_mm=round(c.reach_from_throw_mm, 2),
                    landing_err_mm=round(c.landing_err_mm, 2),
                    seat_offset_mm=round(c.seat_offset_mm, 2)))
        mx = max(sustained_counts)
        verdict = "SUSTAINS (>=10)" if mx >= 10 else "DIVERGES (< 10) -> BREAK"
        print(f"  => sustained {sustained_counts}, max {mx}  [{verdict}]\n")

    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"catch reliable-reach threshold: {CATCH_REACH_MM:.0f} mm (Rung 1)")
    print(f"wrote {csv_path}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
