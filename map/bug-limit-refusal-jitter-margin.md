---
id: JB-24
kind: bug
title: 10 % jitter-margin class of LIMIT_* refusals
status: inbox
created: '2026-10-24'
updated: '2026-11-03T21:54:18Z'
owners: []
priority: medium
effort: m
hardware: robot
owner: decision
radius: subsystem
diagnosis: hypothesis
area: skill-stack
blocked_by: []
links:
- plans/archived/skill-stack-dress-rehearsals-and-residue.md
- logbook/2026-11-04-sitting-debrief.md
parent: two-ball-oval
filed_by: claude
---

Landing-time jitter of +-20-30 ms against the 10 % margin, with session acc/jerk sitting on the YAML ceilings, makes `LIMIT_*` refusals the likely dominant miss class. Source: the sitting-4 analysis in `plans/archived/two-ball-skill-stack.md` and the wave-3 tally. Trigger: `LIMIT_*` returns as the dominant miss; the owner's options (a)-(d) from sitting 4 are still undecided. More Dwell costs transit.

## Options (2026-11-01)

1. Widen the planning margin from 10 to about 15 percent on time-dependent rows only (the catch-time rows), leaving the Sitting limits where they are; measured cost: a few percent of the admissible box per cell, re-swept offline before any Sitting.
2. Absorb the jitter in the re-aim instead of the margin: let the re-send after a tracker fit move the catch time within the splice budget without a fresh LIMIT check against the full margin (the refusal class moves from install to re-send, where a refusal is harmless).
3. Raise the Sitting limits a notch (acceleration or jerk) with the box re-swept: buys margin everywhere at the price of the hardware headroom the 2026-10-31 Hand findings just spent.
4. Measure first: tally `LIMIT_*` refusals per goal from the last three Sittings' launch logs (hold 0 changed the plan shape) and decide on that number; no code until the class is shown dominant at hold 0.

Recommendation: 4 now (one log read, the Item's own trigger), then 1 if the tally confirms the class.

## Findings (2026-11-01, JB-127 option 4)

The owner chose "measure first". `tools/probes/limit_refusal_tally.py` over the launch logs of the 2026-10-28, 10-30 and 10-31 Sittings (7 launches, 58 goals) says the `LIMIT_*` class is NOT the dominant miss class. All but one goal flew at the old hold 0.100.

| Measure | Count |
|---|---|
| Goals ending on a `LIMIT`-class code | 4 of 58 (3 are `HAND_LIMIT_ACC` 4049-4112 vs 3900 on the JB-120 cell, 1 `LIMIT_VEL`) |
| Leg `LIMIT_*` refusals (jerk 28, acc 6, vel 3) | 37: 31 re-sends (old plan stays), 5 aim fallbacks, 1 fatal |
| Non-clean ends that dominate | `MISSED_CATCH` 18, `LANE_REFUSED` 10 (JB-98), `ABORTED_NO_RELEASE` 3 |
| Goals at hold 0 | 1 (oval, 30/30, zero refusals) |

Decision per the owner's rule (no code until the class is shown dominant at hold 0): the margin stays at 10 percent (option 1 not taken). Re-run the probe after a Sitting with several hold-0 goals; the trigger stays "`LIMIT_*` returns as the dominant miss". Entry: [logbook/2026-11-01-jb24-limit-refusal-tally.md](logbook/2026-11-01-jb24-limit-refusal-tally.md).

A question ticket carries this to the owner; the answer under its `## Answer` re-opens this Item.

## Progress

- 2026-11-01: JB-127 option 4 tally: LIMIT_* not dominant (4 of 58 goals, 3 the JB-120 Hand cell); margin stays 10 percent; re-run tools/probes/limit_refusal_tally.py after more hold-0 goals
- 2026-11-01: 2026-11-01 Sitting (first at hold 0, 23 goals): HAND_STROKE 7, HAND_LIMIT_ACC 6, MISSED_CATCH 5, GUARD_LATCHED 2, leg LIMIT 2. Both leg LIMIT ends (5142.8 +2.9 pct, 361 +3.1 pct) were first installs seeded from a late re-send (+11 / +23 ms): this Item's class, but not dominant so the trigger is not met. Owner keeps the 10 pct margin.
- 2026-11-02: Sitting 2026-11-02: 5 LIMIT_VEL ends (350.5-360.7 mm/s), all the Ball 2 CATCH on leg 1 at plan +0.175 s; zero-tolerance gate; the y_d fallback absorbed 5 of 10 refusals. Not reached by the bump brake-seed fix (leg vel moves +-0.6 %); 2 of 5 follow a RESEND only.
- 2026-11-03: 2026-11-03 Sitting: the 3 leg trips (L3236, L3706, L8981) were first-install CATCHes exceeding by +0.5/+1.9/+1.6 % with ~315 ms of deadline unused; the y_d fallback made 2 of 3 worse. CATCH installs run at median 93.8 % p90 98.6 % of leg vel; a 365 mm/s limit would have cleared 13 of 14 refusals. Owner decision, not made.
- 2026-11-03: P4 working point (two-ball-oval P4, logbook 2026-11-03-p4-working-point-600-8000-20a) raises the leg velocity limit 350 -> 600 mm/s, well past the 365 mm/s that would have cleared 13 of the 14 refusals, so these refusals are expected to be absorbed; confirmation is the P4 ramp Sitting's columns canary (count CATCH leg-vel refusals). The jitter-margin option itself stays undecided.
- 2026-11-04: 2026-11-03 Sitting at 600/8000 (P4): 0 LIMIT_VEL and 0 LIMIT_ACC refusals across the Sitting, so the CATCH leg-velocity class did not recur. One LIMIT_JERK refusal on a CATCH re-aim RESEND (peak leg jerk 510456 vs 200000, leg 0, canary throw 20); the first plan stood and the ball was caught.
