---
id: BUG-78
kind: bug
title: 'Two-ball Columns runs end on a missed Catch of Ball 2 with no collision: 13 of 23 ends, mean run 22 Throws'
status: in_progress
created: '2026-11-06'
updated: '2026-11-24T13:49:53Z'
owners: []
priority: high
effort: m
hardware: robot
owner: none
radius: subsystem
diagnosis: hypothesis
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-06-sitting-debrief-2.md
- bug-mid-air-ball-collisions-end-half-of-two-ball-columns
- logbook/2026-11-06-sitting-debrief-3.md
- logbook/2026-11-13-sitting-debrief.md
- logbook/2026-11-18-sitting-debrief.md
- logbook/2026-11-20-sitting-debrief.md
filed_by: claude
night_skip: sitting
night_skip_on: '2026-11-22'
night_skip_note: Next evidence is the 15 mm outward Catch authority trial in its Sitting section.
---

Two-ball Columns runs of 2026-11-06 mostly end on a smooth missed Catch of Ball 2, not on a collision (BUG-74).

## Findings

- 35 endurance goals (Columns, Apex 0.95 to 1.10, Sep. 125 to 133, 101 Throws asked): median 15 Throws, best 73, about 4.5 ends per 100 Throws.
- End classes: 13 smooth missed Catches (2.0 per 100 Throws), 8 collisions (BUG-74), 2 ambiguous, 5 drive faults, 2 lane refusals after a GC pause (BUG-58), 5 refused before the first Throw.
- 21 of the 23 MISSED_CATCH first ENDs are the Catch of Ball 2 (the negative-x Site), although both balls' Throws land equally well (lateral RMS 29.6 against 31.9 mm).
- In 9 of them the last fit was within 40 mm of the aim and the cup read EMPTY: bounce-out, a false EMPTY (the BUG-67 question mid-run) or a late fit, not resolved (needs `/hand_telemetry`).
- At the P2 Sitting three of five ends were the same class, each with the Catch aim clamped at the lateral authority (10 mm outward three times, 35 mm inward twice).

## Open questions

- Not diagnosed.
- Owner decision if the diagnosis points there: the outward authority (10 mm) is a limit, not changed.

## Evidence

`temp/probes/sitting_debrief_2026-11-06b/unit_endurance.md` sections 1 and 2, `unit_stack.md` section 3.

## Sitting

- setup: level, armed, bb-fed
- why: the owner allows a trial of a 15 mm outward Catch authority (10 mm now); late lateral re-plans are hard on the Leg base joints, so the trial watches the Legs as much as the Catches.
- do:
  - Before arming: `ros2 param set /skill_node catch_outward_authority_mm 15.0`.
  - Before arming: `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'columns,apex_m=0.95,separation_mm=125,dwell_s=0.27'}"` and read the margins.
  - Fly 8 Columns Attempts (Sep. 125, Apex 0.95, Dwell 0.27, Throws 21).
  - Afterwards `ros2 param set /skill_node catch_outward_authority_mm 10.0`.
- pass: no Attempt ends on a Leg guard latch or a LIMIT refusal of a Catch re-aim, and no Leg base joint moves; the debrief counts the Catches whose aim went more than 10 mm outward and how many were caught.
- record: the nomination margins, caught of thrown and the end code per Attempt, anything heard or seen at the Leg base joints.
- minutes: 20

## Progress

- 2026-11-06: 2026-11-06 20:20 Sitting: goals 2 and 4 are smooth Ball 2 misses with the ball deflected at Cup height: sideways speed jumps to 1.8 and 0.3 m/s within 30 ms, cup raw 0 throughout, so a false EMPTY and a late fit are out for these two. Ball 45 and 37 mm off the Platform origin at z 1000 (caught median 19, p90 36), Platform at 406 and 784 mm/s. Rim, Cone or edge: mocap cannot say.
- 2026-11-06: Across the endurance set and 20:20: every smooth no-collision end (15 of 15) is a Ball 2 Catch, and all 4 Ball 1 ends are collisions. Open: whether offset or Platform arrival error drives the strike; /trajectory/commanded_position was not read. Evidence: temp/probes/sitting_debrief_2026-11-06c/unit_ends.md.
- 2026-11-08: 2026-11-07: of 20 non-collision Ball 2 ends, 13 had a last fit beyond the Catch authority and 4 inside reach with the Cup empty. Owner: Cup rim 85 mm dia; in turned 90 deg Columns every strike hit the Platform top (top of the Hand's stroke); in other Patterns some misses hit the Hand, some other Platform parts. No seated-ball baseline in the bag.
- 2026-11-13: Sitting of 2026-11-13: Columns canary on the trained plant ended on Ball 2 in 4 of 6 Attempts, fitted landings mixed sign (+44, +25, -88 mm, one no Fit), 3 of 4 within 45 mm of aim with the Cup empty. columns_highlow: 9 of 15 ends Ball 2; 12 had a lost Throw with no Fit. Same signature on the oval and on two single-Throw ends at 20 mm.
- 2026-11-18: 2026-11-18 debrief: NO FIT on a MISSED_CATCH end is the carried empty Throw, not a Tracker miss (ball_held false 33 of 33). Of 28 lost flights with a Fit, 9 were 100 mm or more off, 7 of them outward at the Ball 2 Site, beyond the 10 mm outward Catch authority (owner decision, not changed). 7 lost Balls were found at rest beside the Cup. See BUG-114.
- 2026-11-18: Cup-empty diagnosis (plain Columns on main, 11 first-lost flights): 6 bounced out after entering 3-15 mm from the Cup axis; 2 rim strikes after a late inward re-aim; 2 far Throws after a late-seating Catch; 10 of 11 are Ball 2. Owner: Ball 2 differs only in order and Site, so small misalignments compound. Next: a 300 Hz replay of the bounce-outs.
- 2026-11-20: Sitting of 2026-11-20: 4 fitted lost robot Throws in Columns, 2 outward of the Site by 50 and 71 mm, 2 inside the Cup radius (26 and 37 mm) with the Cup reading empty (bounce-out class). Columns completed 5 of 18 against 9 of 26 on 2026-11-18 (no demonstrable regression).
- 2026-11-20: Owner, 2026-11-20: trial a 15 mm outward Catch authority at a Sitting (the parameter, not the default), no further: late lateral re-plans are aggressive on the Legs, the base joints most. The Sitting section holds the trial. Earlier offline replay passed at 10 mm with 2 % margin and failed at 20 mm.
- 2026-11-25: 300 Hz replay of 32 final missed Catches against 338 caught entries: no feature (lateral offset, relative vertical speed, Cup descent, late re-aim) separates them; the marker's 30 mm offset from the ball centre hides 3 to 15 mm entries. The 15 mm outward trial's gain cannot be predicted from Mocap; its own pass criterion stays the measurement.
