---
title: "carry_zvd Hand smoothness term (BUG-90): a flight-window Hand ACCELERATION penalty, read only with the ZVD rows on, removes the ~5 Hz Hand stop-and-go at no cost to the mode cancellation; a jerk penalty was tried first and only turned the wiggle into a slow ±0.5-1.8 m/s swing"
type: bugfix
date: 2026-11-15
status: complete
phase: "carry-ring-zvd P3 prerequisite (Night Shift 2026-11-15)"
subsystem: motion
related_plan: carry-ring-zvd.md
related_items: [BUG-90]
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - tests/motion/test_cup_zvd.py
  - tests/motion/test_skills_nominate.py
  - logbook/INDEX.md
---

## What

With `carry_zvd` on, `cup_cycle._pose_augment` now adds a Hand smoothness cost next to the
I-PLAN-16 ring rows: `w/n · Σ a_hand,k²` at the flight knots between the Hand's post-release
brake and its capture move (`a_hand` = cup-z acceleration less the centroid-z one when the dz
block is assembled). Three new `PoseConfig` fields: `zvd_hand_smooth_weight` (**100**),
`zvd_hand_smooth_skip_release_s` and `zvd_hand_smooth_skip_catch_s` (**0.15 s** each, the margins
off the window's two ends). With the flag off nothing is computed, so the flag-off program is
untouched. The strict xfail `test_the_zvd_rows_do_not_make_the_hand_stop_and_go_in_flight` is now
a plain test and passes; a parametrised twin covers Dwell 0.35 and 0.50, and a third test pins
that the weight is not read with the flag off.

Why: the ring rows were met by a ~5 Hz Hand wiggle through the flight (the operator saw it at
the Sitting of 2026-11-13, step 21). Under the prescribed tilt, cup z moves the Platform
sideways through the lever term `arm · a_x''`, so a Hand wiggle was the cheapest way to cancel
the mode. The owner chose option 1 of TKT-69: a Hand smoothness term with the rows on.

## Discussion

**A jerk penalty was the first hypothesis and it failed.** I tried a penalty on the Hand's
acceleration change (cup-z jerk) first, as TKT-69 worded it. Over the whole flight
(spans 0..k_td−1) it also penalised the post-release brake (1400-1700 m/s³ for ~75 ms, from
+4.4 m/s). The solver then spread the brake into a slow swing: at Dwell 0.40 the Hand
velocity went from −376..751 to −1805..1146 mm/s (w = 30), with 1 turning point at every Dwell
against 0 flag-off. With the brake and capture margins added (0.15 s each), it still swung
−573..490 mm/s (w = 100, 1 turning point). The reason: a jerk penalty does not stop a slow
ramp, and a slow Hand offset during the tilt sweep (`Δz · a_x''`) is the same lever the wiggle
used. **Penalising the Hand's acceleration** closes the lever at every frequency. The rows then
get their cancellation from the lateral channel instead.

**Sweep (acceleration term, margins 0.15 / 0.15 s; turning points flag-on at Dwell 0.35 / 0.40 /
0.50, flag-off is 0 / 0 / 0):** w 0: 5 / 4 / 2 · w 3: – / 1 / – · w 10: 1 / 1 / 0 · w 30: 1 / 0 / 0 ·
**w 100: 0 / 0 / 0**. 100 is the smallest tried that clears all three Dwells. At w 100 the Hand
velocity in the test window (catch_k−30..catch_k−7) is −271..9 / −263..5 / −255..3 mm/s,
against −1121..2016 / −376..751 / −91..247 with no term and −82..193 / −88..194 / −112..230
flag-off. The window's negative end is the capture move starting about one knot earlier and
running monotone. Flag-off reverses sharply there instead (+194 → −200 → −686 mm/s).

**The tradeoff accepted: smoothness against mode cancellation. Measured, it costs nothing.** The
rows are equalities, so the nominal residual is 0 either way. Robustness (x axis e′ at release,
mm/s, mode frequency ×0.85 / 0.90 / 1.10 / 1.15), no term → w 100:

| Dwell | no term | w 100 | flag off ×0.90 / ×1.10 |
|---|---|---|---|
| 0.35 | −40.4 / −7.2 / +13.5 / +28.3 | −39.4 / −7.0 / +13.2 / +27.8 | −108.5 / −67.7 |
| 0.40 | −14.1 / −0.8 / +7.8 / +16.0 | −13.7 / −0.8 / +7.7 / +15.8 | −69.2 / −46.3 |
| 0.50 | −0.8 / +1.4 / +3.3 / +6.4 | −0.7 / +1.3 / +3.2 / +6.4 | −34.3 / −24.6 |

The realised (wire Hermite) residual is unchanged: |e′| 0.035 / 0.020 / 0.005 mm/s. Leg peaks
with no term → w 100 (v mm/s, a mm/s², j mm/s³) are also unchanged to < 1 %: 0.35: 316 / 5096 /
123041 → 318 / 5131 / 123723; 0.40: 275 / 3260 / 77138 → 275 / 3269 / 77435; 0.50: 224 / 1345 /
38569 → 224 / 1348 / 38561. All are inside 600 / 8000 / 200000. The peak cup-z jerk is the brake
and stays at ~1750-1990 m/s³.

**What is not addressed:** the "two-stage dive after the Catch" from the same Sitting step. It
sits outside the flight window the owner scoped, and its turning count after the Catch is the
same with and without the term (2 vs 3 flag-off on the crude post-catch count).

## Control-cycle walk-through

The term changes only the planned Segment, never the stream. Per 25 ms knot the executor emits
`pose` / `pose_vel` + `hand_rev` / `hand_vel_rps` at 40 Hz to the can-bridge, which Hermite-interpolates
at 500 Hz. The QP's variables are still piecewise-constant cup jerks. Positions, velocities and
accelerations stay continuous at every knot by construction, and the release triple, catch row,
seed and ring rows are unchanged equalities. So a cycle sees the same kind of command: no new
step, no feedforward change, and no seam change (the release state test still agrees to 1e-6).
Inside the flight one cycle now carries a Hand velocity that moves ≤ ~30 mm/s per knot, not the
±400 mm/s, 5 Hz oscillation. The ring is cancelled by the Platform's lateral motion instead, and
the leg peaks above show that motion stays well inside the limits. A failed solve still takes
the `RING` path. Its ring-less re-solve keeps this PSD cost, which cannot change feasibility,
only the residual it reports.

## Verification

- 2026-11-15, `night_shift_nonet.sh python -m pytest tests/motion/test_cup_zvd.py -q -p no:cacheprovider`: **39 passed in 221 s** (the flag-off golden byte-identity test included; the release-triple test now pins the objective change to exactly `2w/n · Aa[ks]ᵀAa[ks]` in the cup-z block and byte-identity elsewhere).
- Re-run by the orchestrator after the unit was cut off (API credit), 2026-11-15, `tools/night_shift_nonet.sh python -m pytest tests/motion/test_cup_zvd.py -q -p no:cacheprovider` on this tree: **39 passed in 183.66 s**.
- Finished by the Night Shift 2026-11-16 (all via `tools/night_shift_nonet.sh nice -n 19 env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python -m pytest -q -p no:cacheprovider`):
  - Every test file naming `carry_zvd`, `cup_cycle`, `PoseConfig` or `_pose_augment` outside test_cup_zvd (18 files under `tests/motion/` and `tests/ros/`), 2026-11-16: **1 failed, 1331 passed, 2 skipped in 303.74 s**. The one failure, `test_skills_nominate.py::test_oval_carry_zvd_at_gate_limits_refuses_035_and_names_040`, pinned the old flag-on objective's margins: the Dwell 0.35 refusal (same verdicts, LIMIT_ACC Leg 1) moved from -0.147113829 to -0.156255330 and the Dwell 0.40 viable margin from 0.116713586 to 0.115946813 (still viable, still the first viable Dwell). Test expectations updated, nothing else; the smoother Hand loads the Legs slightly more (leg-peak table above, < 1 %).
  - `tests/motion/test_skills_nominate.py` after that update, 2026-11-16: **47 passed in 54.12 s**.
  - Wall-clock budget tests `tests/motion/test_unified_cycle_budget.py tests/motion/test_check_segment_budget.py`, 2026-11-16, run alone: **4 passed in 10.27 s**.
  - The new tests fail without the change (source reverted to the parent commit, `-k "stop_and_go or smooth"`, 2026-11-16): 4 failed (the three Hand stop-and-go tests and the flag-off weight test; the weight test fails on the missing `PoseConfig` field).
  - Flag-off claim re-read from the diff: the new cost is built inside `if bool(pcfg.carry_zvd):` after the ring rows, so with the flag off nothing is computed and the pose program is untouched (the golden byte-identity test in test_cup_zvd also pins it).
- **Not run:** the full suite and an audit (outside the finishing unit's scope). The Discussion's numbers came from the earlier unit's one-off scripts and were not re-measured.
- Probes (uncommitted): `/tmp/probe_bug90.py` (weight sweep: turning points, Hand range, ±10/15 % residuals, realised), `/tmp/probe_bug90_acc.py` (acceleration-vs-jerk A/B), `/tmp/probe_bug90_legs.py` (leg peaks), `/tmp/probe_bug90_time.py` (interleaved solve time).
