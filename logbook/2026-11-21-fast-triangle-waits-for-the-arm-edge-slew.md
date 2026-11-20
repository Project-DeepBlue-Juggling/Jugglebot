---
title: "hand_stream_bench: the fast triangle no longer starts before the firmware's arm-edge recovery slew hands the Hand back, and its belt default is speed-derived (BUG-105 tooling)"
type: bugfix
date: 2026-11-21
status: complete
phase: "Bench Hand Sitting tooling"
related_plan: ""
related_issues: []
files_changed:
  - tests/hardware/hand_stream_bench.py
  - tests/teensy_link/test_hand_stream_bench_tri_cut.py
  - tools/probes/hand_foc_scope.py
  - logbook/2026-11-21-fast-triangle-waits-for-the-arm-edge-slew.md
  - logbook/INDEX.md
subsystem: [tooling, hand]
tags: [bench, hand, triangle, belt]
---

## What and why

At the 2026-11-20 bench Sitting every `--stage triangle --tri-speed 10 --tri-cut up` attempt (16 CSVs, `temp/logs/hand_stream_triangle_20261120_23*.csv`) aborted on the 0.5 rev belt about 50 ms after the arm, so the cut never happened. Neither the knot lead nor the ODrive lag explains it. The cause is the firmware's recovery slew (`leg_interp.cpp`, `RECOVER_SLEW_*`). Every output-enable edge, the arm included, makes the Hand's command walk from the captured encoder toward the lane target at 5 rev/s^2, at most 1 rev/s, and hands the lane back only within 0.002 rev. The stage started its triangle clock at the arm, moving at 10 rev/s from frame 0, and its pre-arm "hold" was `frame(0.0)`, a knot moving at the triangle speed. The target therefore ran away from the slew, the slew never handed back, and the belt read the plan leaving a crawling Hand. The evidence is in the CSVs: `echo_rev` follows the slew's 2.5 t^2 parabola (+0.006, +0.012 and +0.025 rev at about 50, 70 and 100 ms) while `enc_rev` is flat. The belt was right to fire. b99dd3a5 did not change the start or the timing. It added the 10 and 20 rev/s commands to a stage that had only ever flown at 0.5 rev/s, where the 1 rev/s slew catches up.

Fix: the triangle streams a true hold (v = 0) at its start from before the arm. The triangle clock starts only when an echo received after the arm is within 0.002 rev of the start (`tri_handback_step`; at least 0.3 s). If a fresh echo is still off the start at 1.5 s, the stage aborts. If no echo arrives at all, the clock starts at 1.5 s on the slew law alone, and the stage says so. The cut time is counted on the triangle clock.

The triangle belt default was a flat 0.5 rev, derived for motion at or below 0.5 rev/s. It is now `0.5 + speed/pos_gain`: 0.514 rev at 0.5 rev/s, 0.786 at 10 and 1.071 at 20. It refuses above the 2.0 rev lead clamp. This is an owner decision. Against a pessimistic simulated Hand (no feedforward credit, steady lag v/35) the honest residual is 0.28 rev at 10 rev/s and 0.53-0.56 rev at 20. `hand_foc_scope`'s Ctrl-C line now says what a clean exit is.

## Verification

- 2026-11-21, `python -m pytest -q -p no:cacheprovider tests/teensy_link/test_hand_stream_bench_tri_cut.py tests/teensy_link/test_hand_stream_bench_gap.py tests/motion/test_hand_stream_bench_order.py tests/motion/test_hand_stream_bench_stroke_cut.py tests/motion/test_hand_foc_scope.py` (under the test lock): **201 passed in 22.79 s**. I found the readers with `grep -rln hand_stream_bench --include='*.py'` and `grep -rln hand_foc_scope tests tools`.
- The new `test_a_fast_triangle_reaches_its_cut_against_a_slewing_lagging_hand` runs the real `main()` against a simulated lane, slew and ODrive (no feedforward credit, 8 ms stale telemetry). It covers up 10 rev/s, up 20, down 20 and down 10. It failed before the fix with `ABORT: driver deviation belt |1.371 - (0.382 + 0.3·8ms)| = 0.986 rev`, which is the bench signature.
- Not run against hardware. The slew hand-back depends on the echo actually arriving; the 233859 CSV had none, and that case falls back to the 1.5 s start.
