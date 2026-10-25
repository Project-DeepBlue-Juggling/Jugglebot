---
title: "I-PLAN-11 extended to the rest_slew seed: the reload pre-tilt REST refusals (219k, 262k) were the tilt-rate seam"
type: bugfix
date: 2026-10-23
status: in-progress
phase: "two-ball-skill-stack - post-R6"
related_plan: two-ball-oval.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tests/motion/test_splice_tilt_rate_seam.py
subsystem:
  - skills
tags:
  - kinematics
  - testing
---

## What changed

Two `reload` pre-tilt REST splices were refused `LIMIT_JERK` at leg 0, t=0.025 s on the re-gated clock: 2026-10-21 columns (219k) and 2026-10-23 `oval_1ball_fed` (262k, launch log L745-751). Both are I-PLAN-11 in the `rest_slew` branch, which I-PLAN-11 had exempted ("opens from a static attitude"). That premise is false for the pre-tilt REST: it is spliced into the opening/bridge REST at k_s 62-65, 5-8 knots before the bridge ends, and when the bridge is a cross-site move its banking schedule is unwinding (head ry rate -0.016 to -0.158 rad/s at knots 63-67), while `_smooth_slew` opened at the head's tilt value with ZERO rate (`tilt_schedule` dropped `start_tilt_rate` in this branch). The pose-x velocity gap (0.9-5.9 mm/s and growing) puts an acceleration jump at k_s-1. The 2026-10-21 "untraced t=0.025 reload-REST" case is the same event: before I-PLAN-10's k_s-2 range it flew unmeasured (170k at 120 mm would have passed).

The variable is the opening REST's cross-site travel d times k_s (d=0 -> 3k; d=146 mm at k65 -> 249k offline). Ruled out: hold tilt (8.5/9.231/10.0 deg identical within 1k), arrival azimuth, the yaw snapshot (BB aim only), absolute x / leg-0 lever (static head 3k), levelling offsets, k_wire worker/node skew (not in the gate's input).

Fix: `_smooth_slew(start_rate)` -> `_smooth_slew_carried`: quintic + `r0*T*(u - 6u^3 + 8u^4 - 3u^5)`; rate, accel and disc caps are checked on the COMBINED profile (2001 samples) and refuse (`TILT_PIN`), never clip; zero or None rate is bit-identical. INVARIANTS row I-PLAN-11 now covers every seed branch with the second enforcement point. Tradeoff accepted: the two-constant legacy branch and the held branch stay exempt (held is pinned by `HOLD_TILT_SEAM_TOL`; the pre-tilt now lands at rest). Option B (a named seam tilt-rate check at `splice_at`) NOT done; I-PLAN-11 is still enforced at the producers only.

## Verification

Offline sweep (production path, legs 350/5000/200k, offline k66 = live k_s 65), peaks at k63-67:

| d (mm) | before | after |
|---|---|---|
| 0 | 3k | 3k |
| 60 | 26-107k | 26k |
| 120 | 52-214k (205k/214k REF at k66/67) | 52k |
| 146 | 64-260k (249k/260k REF) | 64k |

- 2026-10-23, `D=0,60,120,146 TL=8.5,9.231,10.0 python /home/jetson/.claude/jobs/db285742/tmp/probe_sweep.py` (OPENBLAS/OMP 1 thread): 12 refusals before, 0 after.
- 2026-10-23, `pytest tests/motion/test_splice_tilt_rate_seam.py -q`: 23 passed in 1.95 s (16 existing + 7 new: both live refusals reproduced then accepted; with the rate pin disabled the control still refuses 205k / 249k at leg 0, t=0.025; zero-rate bit-identical; cap checks include the rate term).
- 2026-10-23, `pytest tests/motion/test_splice_tilt_rate_seam.py tests/motion/test_cup_realize.py tests/motion/test_unified_cycle.py tests/motion/test_unified_cycle_splice.py tests/motion/test_skills_schedule.py tests/motion/test_skills_executor.py tests/ros/test_install_segment.py tests/ros/test_skill_node_reload_only.py -q -p no:cacheprovider`: 703 passed, 2 skipped in 18.10 s.
- Box: `bash temp/sweeps/sweep_P1.sh` 2026-10-23 (four families x 2 runs, BLAS one thread, gated files frozen for the run) -> single/hop/columns/oval all IDENTICAL across the two runs; merged box content-identical to the committed one (20 boxes, same limits); `gate_hash` 21e873c3bd22 -> d6e60d5556b9, `swept_at` 2026-10-23 (`temp/logs/sweep_restslew_20261023.log`).
- Full gate: 2026-10-23, `./run_tests.sh --full` (BLAS single-thread, after the box re-sweep) -> 7054 passed, 9 skipped in 312.33 s + serial tail 7 passed, exit 0 (`temp/logs/gate_parity_restslew_20261023.log`).

NOT FLOWN: the next `reload` goals with a cross-site opening REST are the check. `cup_realize.py` is gated, hence the box re-sweep.
