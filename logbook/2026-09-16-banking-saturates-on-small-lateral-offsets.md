---
title: "Cup banking saturates to its 12° clamp on ANY nonzero lateral offset during the pre-catch dive — the 0.9 m 'wobble' and LIMIT_JERK root cause (carried)"
type: investigation
date: 2026-09-16
status: open
phase: "two-ball-skill-stack — R3 (carried to the planner)"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py (interim guard: lateral_authority_m)
  - ros_ws/src/jugglebot/jugglebot/skill_node.py (param learner_lateral_authority_mm, default 0)
---

## What was seen

At the 2026-09-16 apex ladder (`armB-090` attempts 2–3, `logbook/2026-09-16-outcome-landing-frozen-at-the-crossing.md`)
the learner commanded lateral landing offsets of 3.9 mm and 31 mm. The platform answered with 2–3° of tilt
and 8–15 mm of translation across the catch window (mocap `Platform` body vs `/trajectory/commanded_position`,
tracking to ~1.5 mm — the motion was COMMANDED, no guard/lead-clamp/CAN event), the cup moved out from
under the ball, and attempt 2's catch re-solve was refused `LIMIT_JERK` (161 323 vs 150 000 mm/s³). Offline
through the production `plan_segment`: dx=0 → leg jerk exactly 0; dx=3.9 mm → 137–140 k; dx=10 mm → 103–110 k;
dx=31 mm → 86 k. Non-monotone, worst at the smallest offsets, above the sweep's 135 k margin at 4 mm although
the admissible box says dx∈[0, 40] mm (the sweep never samples 0–10 mm).

## Root cause (read-only probe, 1 mm sweep of dx and dy at 0.9 m, both flights)

- Segment duration, knot count, catch knot and release knot are byte-identical for every dx (1.4500 s / 59 knots
  / k34 / k46 at T=0.8569): **the time does not shrink**.
- `cup_realize.py:611-616` sets the per-knot cup attitude to `tilt_to_receive(g − a_cup)`. In the 125 ms before
  the catch the cup dives at 1.17–2.84 g, so the apparent gravity in the cup points UP (`f_z > 0`) — no attitude
  can seat the ball, the prescription has no solution. `tilt_geometry.py:85-99` then clamps the angle to 12° and
  takes the azimuth from the NORMALISED lateral residual — scale-free. Measured: `raw_max = 12.000°` for every
  dx from 0.5 mm to 40 mm, 0.000° at dx=0 (the `lat_norm < 1e-9` guard). A 0.4 milli-g lateral residual and a
  0.06 g one demand the same full-scale tilt.
- The tilt smoother's widen loop (`cup_realize.py:460-497`) exits on tilt ACCELERATION only, never the third
  difference `LIMIT_JERK` refuses on; which branch it exits on is decided by 0.001 rad/s² (dx=7 mm: 5.221 ≤ cap
  5.2222 → 137 823; dx=8 mm: widen → 49 762; the `⌈⌉` in `L` at `:474` skips the widen again at dx 17→18 →
  100 743). Within a branch the jerk is flat in dx (0.1 %/mm). **Small movements DO invoke small residuals; the
  clamp turns them into a full-scale manoeuvre, and a discrete smoother branch sets the jerk.**
- Corollary: the smoother's cap is `0.5·leg_acc/478.72`, so RAISING the session leg-acc limit makes the plan
  jerkier (3000 → 59 k, 4000 → 99 k, 5000 → 140 k). The live 161 k is reproduced by a 0.30° seed tilt (a
  levelling-residual-sized splice seed): the peak is not a stable function of the command, so no box grid fixes it.
- Scale of the absurdity: a 3.93 mm landing shift needs 0.068° of release aim; the planner commanded 2.24° and
  13.5 mm of platform excursion — that excursion IS the operator's "wobble".

## Fix plan (owner to schedule)

1. **Contract (root):** banking is defined only while the cup retains seating force (`f_z ≥ −ε`); outside it carry
   the last valid attitude instead of saturating. One branch at `cup_realize.py:611-616` plus an
   amplitude-invariance test (tilt demand → 0 as the lateral residual → 0). Needs a re-measure and a full box
   re-sweep — not same-day.
2. Add the third difference to the widen loop's exit test.
3. Standing policy: sub-5 mm lateral corrections belong to the throw velocity vector (the hand), not the platform.

**Interim guard landed today:** `SkillExecutor.lateral_authority_m` / `skill_node` param
`learner_lateral_authority_mm` (default 0): the learner corrects FLIGHT only; the lateral command is the schedule's
`y_d`. At dx=0 leg jerk, acceleration and velocity are exactly zero. Adding 1, 2, 4, 6, 8 mm to
`SINGLE_SITE_OFFSETS_MM` and re-sweeping would collapse the P1/P1 0.85–0.95 box's lateral range to the identity
command only — the honest box, once the contract fix lands.

## Verification

Probe scripts and the 492-line report are session artefacts (`jerk_offset_report.md`, not committed; the numbers
above are copied from it). (2026-09-16, `pytest tests/motion/test_skills_executor.py tests/ros/test_skill_node.py -q`):
the guard's two tests pass with the files' existing suites — counts in the commit's full-gate triple.

(2026-09-16, `./run_tests.sh --full`, log `temp/logs/ladder_close_full3_20260916.log`, the final tree of this phase — ladder close-out, new launch defaults, per-release correlation, lateral-authority guard, audit follow-ups): **PASS — parallel 6233 passed, 9 skipped, 2 xfailed in 297.12 s; serial 6 passed in 18.98 s.**
