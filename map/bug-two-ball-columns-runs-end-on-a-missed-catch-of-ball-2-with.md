---
id: BUG-78
kind: bug
title: 'Two-ball Columns runs end on a missed Catch of Ball 2 with no collision: 13 of 23 ends, mean run 22 Throws'
status: in_progress
created: '2026-11-06'
updated: '2026-11-28T14:24:16Z'
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
night_skip_on: '2026-11-26'
night_skip_note: 300 Hz replay 2026-11-25 found no separating feature; the 15 mm outward Catch authority trial in its Sitting section decides.
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
- why: 25 mm of outward Catch authority lifted Leg 1 off its Base joint; the owner set the default to 15 mm, and this trial flies at it with the lowered Leg acceleration limit, after BUG-160's refit so the Feed does not confound it.
- do:
  - Before powering: inspect the Leg 0-2 Base joints by hand (seating, play, any witness marks) and note what you find.
  - Fly after BUG-160's refit trial, with the Feed trim at zero.
  - Before arming, once the code change (default 15 mm, a refusal above it, Leg acceleration limit 4500 mm/s^2) is on main and built: do not set `catch_outward_authority_mm`; leave it at the default.
  - Before arming: `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'columns,apex_m=0.95,separation_mm=125,dwell_s=0.27'}"` and read the margins, including the line with the Catch re-aim at its limit.
  - Fly 8 Columns Attempts (Sep. 125, Apex 0.95, Dwell 0.27, Throws 21) at the default authority.
- pass: no Leg base joint moves and no Attempt ends on a Leg guard latch; the debrief reads the realized Leg acceleration per Catch (bar: none above 4500 mm/s^2) and the Platform tilt (none above 3.5 deg).
- record: the nomination margins, caught of thrown and the end code per Attempt, anything heard or seen at the Leg base joints.
- minutes: 15

## Sitting result (2026-11-28 12:39)

- verdict: pending
- notes (split into bullets):
  - Attempts:
  - 1 - Completed without issue
  - 2 - Completed without issue
  - 3 - Ended with MAX_DEVIATION for the Hand
  - 4 - Ball2's feed hit the Platform frame, becoming uncatchable.
  - 5 - Ball2 seemed to my eye to hit the hand more or less spot-on, but it bounced out of the hand, making it unthrowable
  - 6 - Completed without issue
  - 7 - Ended with MAX_DEVIATION for the Hand. This Attempt had an extremely clean Ball2 feed from BB. If all Ball2 feeds were this smooth, that would be incredible.
  - 8 - Completed without issue

## Trial at 15 mm (2026-11-28, bag 2026-11-28_12-31-28)

Reading: leaning positive, not proven. Owner decision: try a larger value next (25 mm, Sitting section above). The verdict stays pending.

- The parameter was in force: all 38 outward-clamp lines read "authority 15 mm (inward 35 mm)".
- Attempts 1, 2, 6, 8 completed 20 of 20; 3 and 7 ended on a Hand MAX_DEVIATION after a lane refusal (the known bridge-stall race); 4 on a Ball 2 Feed that never seated; 5 on a Ball 2 Feed that bounced out.
- Throw-Landing Catches: 0 of 105 missed, Ball 2 0 of 50. The 2026-11-27 19:13 bag (10 mm) lost about 5 of 10 Attempts to a missed Ball 2 Landing Catch.
- 28 Catches clamped at 15.0 mm outward; requested outward aim 16 to 36 mm, median 20; median shortfall past the limit 4.9 mm (11.3 mm at 10 mm). 27 of the 28 were caught. The clamp is still binding, which is why 25 mm is next.
- Landing scatter and Platform lateral motion around Catches are unchanged (Ball 2 sd 19 / 19 mm against 20 / 20; Mocap speed median 1016 against 1000 mm/s).
- Confounds: the trial also ran the merged Learner (BUG-158) and the sample is 8 Attempts.
- Offline check of 25 mm: the live node does not bound the parameter, and the nomination for Columns Sep. 125, Apex 0.95, Dwell 0.27 with the Catch re-aim at its limit is accepted at 10, 15, 20 and 25 mm with worst margins 0.137, 0.111, 0.095 and 0.113 (2026-11-28, `nominate.nominate` with `AimAuthority(30, out, 35)` at the launch limits, all margins above 0).

## Sitting result (2026-11-28)

- verdict: fail
- notes (split into bullets):
  - I ran 10 attempts because so many ended prematurely. Attempts:
  - 1 - Ball2 feed collided with the Platform
  - 2 - Mid-air collision right at the start
  - 3 - Collision
  - 4 - Mid-air collision right at the start
  - 5 - Ball2 feed wasn't caught properly
  - 6 - Ball2 feed wasn't caught properly. Ball2 jammed under hand (briefly)
  - 7 - Messy Ball2 catch (post-start). BB Ball2 feed was clean
  - 8 - Mid-air collision right at the end. Looked relatively clean, and the Ball2 feed was great.
  - 9 - Completed. Was a little messy and Platform seemed a little wobbly at times.
  - 10 - Leg 1 separated from the Base towards the end.
  - (sheet 2026-11-28-robot, step 9)
  - derived: launch log 2026-11-28-19-38-37 and bag: in the last Attempt (start line 7024) a Catch re-aim of 30.8 mm (21.5 mm outward, 30.8 mm across) was installed 0.567 s before contact and passed every limit (realized Leg acceleration 5015 of 8000 mm/s^2); the Platform then rose about 8 mm above its median and tilted 4.8 deg within 125 ms, and the owner saw Leg 1 separate from the Base. One Catch re-aim was refused LIMIT_JERK.
  - derived (cont.): 4 of 10 Attempts completed.

## Evening Sitting 2026-11-28: fail at 25 mm, Leg 1 separated from the Base

- 25 mm is not to be flown again. In the last Attempt (skill 17) a RESEND re-aim of 30.8 mm (21.5 mm outward, 30.8 mm across, the across demand clamped from 51.5) was installed 0.567 s before contact.
- Every check passed: realized Leg peaks 348 mm/s, 5015 mm/s^2 and 137k mm/s^3, 58 to 69 % of the limits 600 / 8000 / 200000. The Platform then rose about 8 mm above its median (184.5 mm) and tilted 4.8 deg within 125 ms; Leg 1 reseated on the deceleration (inferred from Mocap and currents).
- The 21.5 mm outward part was possible only at 25 mm authority (about 6.5 mm at 10).
- Near-miss at 10 mm: step 7 Attempt 3, last Catch, 5754 mm/s^2, Platform z 179.3 mm and tilt 2.4 deg (owner: Leg 1 extremely close to being pulled from the Base). These two are the maxima over 353 Catches; every morning Catch stayed at or below 4394 mm/s^2.
- Medians are unchanged from the morning, and BUG-161's fix is not implicated (re-aims taken per Catch 1.25 against 1.27). Ruled out: the Feed trim, Mocap-driven demand growth, lateness.
- Owner decisions 2026-11-29: default outward authority 15 mm with a refusal above it; Leg acceleration limit 8000 to 4500 mm/s^2. The gap is filed as BUG-163.

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
- 2026-11-28: Sitting step E5 of the learner-efficacy plan: the 15 mm outward authority trial flies at the next Sitting, after BUG-158's check; E1 (2026-11-27) shows landing scatter 14-17 mm per axis is a Hand floor, so the Catch must absorb it.
- 2026-11-28: Trial at 15 mm flown 2026-11-28: 0 of 105 Landing Catches missed (leaning positive, 8 Attempts, Learner confound); clamp still binding; next trial 25 mm after BUG-160's Feed trim (detail below)
- 2026-11-29: Failed at the Sitting of 2026-11-28 (evening): Leg 1 separated from the Base at 25 mm authority on a 30.8 mm RESEND re-aim that passed every Leg limit. 25 mm is not flown again; next trial 15 mm default, 4500 mm/s^2 Leg limit, BUG-163 (detail below)
