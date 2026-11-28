---
id: BUG-146
kind: bug
title: Aimed Throw from a level rest is refused HAND_LIMIT_C2 past about 6.3 deg of release tilt
status: complete
created: '2026-11-26'
updated: '2026-11-27T17:11:48Z'
owners: []
priority: medium
effort: s
hardware: robot
owner: none
radius: subsystem
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-28-hand-drop-rates-clamped-spline.md
parent: human-partner-juggling
filed_by: claude
---

An aimed THROW from a level rest is refused `HAND_LIMIT_C2` once the release tilt passes about 6.3 to 6.9 deg. Found 2026-11-26 by the PWAY-11 P4 design sweep (`executor.check_segment`, the context of `tools/probes/human_catch_reach.py`).

## Findings

- The refusal reads "peak hand knot-acceleration discontinuity 5.02 rev/s^2 > 4.50". It appears in all 8 directions, at Apex 1.5, 1.8 and 2.1 m, and for windows of 0.4, 0.6 and 0.8 s.
- It caps aimed reach at 250, 400 and 550 mm for those Apex heights. Leg jerk margin at the edge is still 0.04 to 0.41, so the Legs do not bind.
- The tolerance's own comment in `controller/feasibility.py` says a plan that needs a looser value has a real C2 defect.
- Root cause not investigated. Raw rows: temp/probes/pway11-p4/ (sweepA.jsonl, reach_table.md).

## Wanted

- Find where the Hand acceleration jump comes from as tilt grows (the tilt-coupled Hand compensation at a knot is the first suspect, unchecked), and fix the plan, not the tolerance.

## Findings (2026-11-27)

Night Shift diagnosis unit, offline (probe `/tmp/probe_bug146.py`, not committed). Reproduced: Apex 1.8 m, 450 mm (6.82 deg), window 0.6 s is refused at 5.384 > 4.5; 400 mm (6.07 deg) passes at 4.266. The Apex in this Item is absolute: the edge falls at about 280, 415 and 548 mm for 1.5, 1.8 and 2.1 m.

- Where: not in the launch (peak 0.18 to 0.63) but in the SETTLE tail, at tail Knot 4 (release + 0.100 s), where the tilt slews back to Level at about -0.58 rad/s.
- What: the Hand's tilt-drop term arm·(1 - cos θ). Its knot positions are exact, but its knot rates come from a second-order central difference of the axis (`cup_realize.py:1775`, used at `:1781` and `:1789`; stencil `:1636-1650`).
- Why it fails the gate: Pass 5 of `validate_cycle` (`feasibility.py:2120-2150`) rebuilds each span as a Hermite cubic from (position, rate). A rate error δv becomes an acceleration jump of roughly 320·δv. The cup stroke channel contributes 0.000.
- Scaling: the jump is about 0.116·θ² rev/s² (θ in deg): 1.04 at 3 deg, 2.90 at 5, 4.59 at 6.3, 5.66 at 7. The edge is 6.24 deg, the same at Apex 1.5 and 1.8 m, in the 0 and 90 deg directions, and for windows 0.4 and 0.6 s.
- Consequence: every tilted plan already carries this step in the Hand's knot acceleration, which the firmware's torque feedforward sees. The comment at `feasibility.py:203-221` (the plan is C2 to 8.5e-9) no longer holds for tilted plans.

Unchecked: splice seams (I-PLAN-11's 0.0041 rev/s Hand rate match would need knot 0 clamped to the seed's rate); the same finite-difference class in the Platform channels (`shift_dot`, `tilt_dot`, `dz_dot`), which feeds Leg jerk; whether the six borderline hop re-aim refusals in I-PLAN-11 (Hand C2 3.91 to 5.0) are this; pre-tilt and decay RESTs, held-tilt Catches, slider saturation.

## Options (2026-11-27)

1. Clamped cubic-spline knot rates for the drop term in `decompose` (one tridiagonal solve, end rates pinned to today's). The jump goes to 0.000 at every tilt; Hand positions unchanged; Hand knot rates move by at most 0.0094 rev/s at 6 deg. About 20 to 30 lines plus a test. Changes every tilted plan's rates, so it is held for a Sitting.
2. A fourth-order five-point stencil: one line, cuts the jump about 3.6x, but still fails near the 12 deg tilt cap. Not recommended.
3. Exact rates from the tilt schedule's own derivatives: leaves a residual of 0.16 to 0.24 and needs `tilt_schedule` to return derivatives. More invasive.
4. Leave it: aimed reach stays capped at about 6.2 deg of release tilt.

Recommendation: 1, with knot 0 clamped to the seed's Hand rate so splice seams are unchanged, then a Columns Attempt at a Sitting.

## Sitting

- setup: level, armed, bb-fed
- why: every tilted plan's Hand knot rates changed (the drop term's rates are now a clamped spline, up to 0.02 rev/s at 9 deg), so the Hand torque feedforward sees new rates on every Banking and aimed Throw; only a flown Columns Attempt shows the Hand is still clean.
- do:
  - Fly one Columns Attempt, Ball Butler fed, at the current Sitting limits and the default Dwell, for at least 20 Throws.
- pass: at least 20 consecutive Columns Catches, no HAND_LIMIT_C2 refusal in the launch log, no Hand Fault or Guard trip, and no audible or visible Hand knock at release or in the SETTLE tail.
- record: Catches and Throws, every refusal code in the launch log, any Hand Fault, and the bag name.
- minutes: 15
- branch: night-shift/2026-11-28-BUG-146

## Progress

- 2026-11-28: Night Shift 2026-11-28: held on night-shift/2026-11-28-BUG-146: clamped spline drop rates built per TKT-98; 1 audit finding open (no direct solver unit test); unblocked by a Sitting
