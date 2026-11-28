---
id: BUG-159
kind: bug
title: Banking tilt schedule under-reads Leg jerk, so a REST from a moving state can be refused at the live limits
status: complete
created: '2026-11-28'
updated: '2026-11-28T14:21:54Z'
owners: []
priority: medium
effort: m
hardware: none
owner: decision
radius: subsystem
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-28-banking-schedule-scores-the-realised-curve.md
filed_by: claude
---

Found 2026-11-28 when the sim gate's Leg limits were brought up to the robot's (600 mm/s, 8000 mm/s^2, 200000 mm/s^3).

## Findings

- A REST that starts from a moving state (0.33 s from about 130 mm/s) can plan over the Leg jerk limit: 211,669 mm/s^3 against 200,000 on a one-ball self-Throw's closing REST. The plan gate refuses it and the Attempt ends LIMIT_JERK, so it fails safe.
- About 98 % of the peak is the Banking tilt (roll), at the third Knot.
- Cause: the Banking schedule guards its own jerk on Knot third differences, while `validate_cycle` reads the interpolated curve the robot follows, which is 1.4 to 1.5 times higher here.
- The tilt acceleration cap is derived from the Leg acceleration limit (5.05 rad/s^2 at 5000, 8.09 at 8000). At the old limits that cap hid the mismatch (peak 96 %); at the live limits it no longer binds.
- In the sim at the live limits 4 of 10 self-Throw seeds refuse. Rest-seeded plans peak at 67 % or less and are not affected. The BUG-152 fix has no effect on this (identical peak with it on and off).
- On the robot, which has planned at these limits since 2026-11-03: two REST refusals in the logs (2026-11-07 at 200,003; 2026-11-13 oval_1ball at 365,412, Leg 3, t = 0.075 s). 345 of 1388 settle seeds were moving, 73 above 100 mm/s.

## Options

- (1) Make the Banking schedule score the same interpolated curve the gate reads: one rule, matching the gate. Changes every banked schedule, so it needs regression sweeps and the solve budget checked.
- (2) Pin the tilt acceleration cap to its old value: restores 96 %, leaves the knife edge, removes tilt authority flown since 2026-11-03.
- (3) Lower the tilt jerk budget fraction: one line, calibrated on one case.

## Unchecked

Whether any of the robot's 101 CATCH LIMIT_JERK refusals share this cause; the jerk margin of accepted RESTs (not logged); Hop and oval in the sim. Probes: scratchpad out/restjerk of the session that filed this.

## Progress

- 2026-11-28: fix built on branch offline/bug159, not merged. The Banking schedule scores its Leg jerk on the curve the gate reads; all ten self-Throw seeds plan at the live limits (REST peaks 87-106 % to about 50 %); no Pattern loses a Catch in the sim; tilt stage 1.4 to 2.0 times slower, budgets green. Open with the owner: two pinned measurements and the held Hop ordering test.
- 2026-11-29: Fixed (I-PLAN-18): the Banking schedule scores its Leg jerk on the curve the plan gate reads. Sim at the live limits: all ten self-Throw seeds plan (REST peaks 87-106 % to about 50 %), no Pattern loses a Catch. Nomination margins: Hop, oval, polygon walk up; Columns down 3 %; turned Columns at 45 deg 0.036 to 0.026. Merged 2026-11-29 after the full gate; unflown.
