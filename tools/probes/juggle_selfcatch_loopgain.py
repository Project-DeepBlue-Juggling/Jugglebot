"""Rung-2b single-ball self-catch loop-gain sweep (the MAKE-OR-BREAK evidence).

Drives ``sim/juggle_selfcatch.py::run_self_catch`` (compose the Rung-2a tilt-aimed
throw with the Rung-1 catch into a single-ball toss->catch->toss loop, re-planned
each cycle under the §3 tracking noise) across seeds and the three recover/geometry
variants, reporting the **per-cycle reach trend** and the **sustained cycle count**
— the loop-gain the make-or-break gate reads. The gate needs >= 10 sustained
cycles with a flat/decaying reach (loop gain < 1).

**Finding (2026-07-01): the pure column self-catch does NOT sustain (loop gain > 1)**
— every seed and every variant diverges within 0-3 cycles: the catch reach
amplifies each cycle (~8 mm -> ~50 -> ~110 -> ~210) past the catch's ~60-80 mm
reliable reach -> drop. See ``logbook/2026-07-01-rung2b-selfcatch-column-divergence.md``.

Head-to-head reference: the pre-tilt divergence (logbook
``2026-06-27-online-juggle-throw-fix-catch-axis-split-band-limit-cascade``) also
had loop gain > 1 (a caught-then-thrown ball walked off > 150 mm/cycle). Tilt does
NOT kill the divergence for a column — because for a column the tilt is ~0, so the
tilt mechanism (decoupling lateral aim from the band-limited platform) is inactive.

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

# The three variants behind the verdict: the FAITHFUL composition (Rung-2a
# plan_cup_cycle recover, stationary column with reposition-to-origin), the DRIFT
# variant (throw in-place from the caught xy -> no reposition, isolates the
# reposition's amplification contribution), and the RETRACT variant (an ad-hoc
# axial slider retract — explored to force separation of a centred caught ball;
# a knife-edge that also amplifies).
VARIANTS = {
    "faithful (plan, stationary)": dict(recover="plan", stationary=True),
    "drift (plan, in-place)":       dict(recover="plan", stationary=False),
    "retract (stationary)":         dict(recover="retract", stationary=True),
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
          f"gate: sustained >= 10 with a flat/decaying reach (loop gain < 1)\n")
    for name, kw in VARIANTS.items():
        print(f"--- {name} ---")
        sustained_counts = []
        for seed in range(args.seeds):
            r = run_self_catch(SelfCatchConfig(seed=seed, n_cycles=args.cycles, **kw))
            sustained_counts.append(r.sustained)
            trend = r.reach_trend_mm
            print(f"  seed {seed}: sustained {r.sustained:2d}/{args.cycles}  "
                  f"reach_trend_mm={trend}")
            for c in r.cycles:
                rows.append(dict(
                    variant=name, seed=seed, cyc=c.cyc,
                    separated=c.separated, caught=c.caught, held=c.held_at_end,
                    in_off_start_mm=round(c.in_off_start_mm, 2),
                    in_off_end_mm=round(c.in_off_end_mm, 2),
                    reach_mm=round(c.reach_mm, 2),
                    reach_from_throw_mm=round(c.reach_from_throw_mm, 2),
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
