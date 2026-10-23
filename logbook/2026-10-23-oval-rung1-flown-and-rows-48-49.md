---
title: "Oval rung 1 flown (28/28), I-PLAN-11 row 48 passed, catch-time A/B row 49 criterion not met"
type: feature
date: 2026-10-23
status: resolved
phase: "two-ball-skill-stack - post-R6 (sittings 2026-10-22 19:50 and 2026-10-23 11:06)"
related_plan: two-ball-oval.md
related:
  - 2026-10-22-hop-reaims-refused-mid-transit-tilt-rate-seam-and-the-new-cup
  - 2026-10-22-catch-observation-and-catch-time-offset
files_changed:
  - tests/hardware/session_skill_stack_rehearsals.md
  - tests/hardware/session_oval_rung1.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - plans/active/two-ball-oval.md
subsystem:
  - motion
  - config
tags:
  - testing
  - docs
---

# Oval rung 1 flown, rows 48 and 49 recorded

Hardware results only; no code in this entry. Evidence is the two launch logs (cited with line numbers) plus the bags.

- Row 48 (I-PLAN-11, hop): PASSED. 3 hop goals, 12/12 caught, zero LIMIT_JERK at t=0.075/0.050/0.025; hop re-aims 11 accepted / 7 refused (5 SPLICE_TOO_LATE, 1 HAND_LIMIT_C2, 1 infeasible); stop-REST half not exercised. One new refusal: 377161 mm/s3 leg 3 at t=0.425 on a late CATCH re-aim that moved the landing 31.5 mm (L3368), cost one throw. I-PLAN-11 Fate cell: FLOWN.
- Row 49 (catch-time A/B): criterion NOT met; settle medians 118 / 209 / 213 / 259 ms (before / -20 / 0 / +20), no arm shift above 60 ms, bimodal in every arm. Feed lines printed `--` for all 21 feeds (tracker status leaves IN_FLIGHT before the report; fix in the next commit, NOT FLOWN). Decision: no second catch learner; seat/rim geometry measurement retired (owner: ASA hand geometrically identical to PLA), friction the remaining candidate. Detail in `2026-10-22-catch-observation-and-catch-time-offset`.
- Oval rung 1 (row 12a, GUI, dwell 0.65, oval plant_id): 28/28 caught (hop 10/10, oval_1ball 4/4 and 10/10, oval_1ball_fed 4/4); seat median 0.09 s, settle 39 ms (launch log, n=11); seat-to-arrival 62 ms (bag, n=28), tighter than columns. 11 of 16 oval throw lines show a refused re-aim (all SPLICE_TOO_LATE) (solve 145-215 ms). One oval_1ball_fed goal ended 0/0 on a reload pre-tilt REST splice refusal, LIMIT_JERK 262k leg 0 at t=0.025 (L745-751), BB cancel clean. Same class as the 2026-10-21 219k refusal (hold 9.46 deg, L1392 of that log): 2 refusals of ~15 pre-tilt REST splices over three sittings; diagnosed the same day: I-PLAN-11 rest_slew seed, see 2026-10-23-iplan11-extended-to-the-rest-slew-seed. Owner: "Oval 1 ball patterns are working really nicely. I think we're ready to advance on this."

## Verification

| Date | Command | Result |
|---|---|---|
| 2026-10-22 | launch log `/home/jetson/.ros/log/2026-10-22-19-50-38-975040-jetson-50159/launch.log` (rows 48, 49; L3368, L3402-4405) | as above |
| 2026-10-23 | launch log `/home/jetson/.ros/log/2026-10-23-11-06-46-742322-jetson-202560/launch.log` (oval; L745-751) | as above |
