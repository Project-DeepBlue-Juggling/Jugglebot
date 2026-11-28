---
title: "First flight of the merged Learner passes; the 15 mm Catch authority trial leans positive (8 Attempts, not proven); Ball 2's Feed lands 25 mm short of its announced landing and a clamped Catch re-aim is dropped by the re-send tolerance"
type: investigation
date: 2026-11-28
status: in-progress
phase: "Sitting 2026-11-28 (two flights)"
related_plan: learner-efficacy.md
related_issues: []
files_changed:
  - map/bug-first-flight-of-the-per-release-kind-plant-model-bug-138-and.md
  - map/bug-two-ball-columns-runs-end-on-a-missed-catch-of-ball-2-with.md
  - map/bug-ball-butler-s-feed-of-ball-2-lands-25-29-mm-short-of-its.md
  - map/bug-a-catch-re-aim-clamped-to-the-outward-authority-is-discarded.md
  - map/feature-install-the-host-hold-within-one-tick-of-a-firmware.md
  - map/bug-teensy-bridge-node-forwards-only-the-newest-waiting-knot.md
  - plans/active/learner-efficacy.md
  - logbook/2026-11-28-first-flight-of-the-learner-merge-and-the-15-mm-catch-authority-trial.md
subsystem: [skill-stack, learner, ball-butler, hand]
tags:
  - sitting-debrief
  - columns
  - catch-authority
---

# First flight of the merged Learner and the 15 mm Catch authority trial

## Summary

Two Sittings on 2026-11-28, both Columns (Sep. 125 mm, Apex 0.95 m, Dwell 0.27 s, 21 Throws). Bags: A = 2026-11-28_12-21-52 (BUG-158, 10 Attempts, Catch outward authority 10 mm); B = 2026-11-28_12-31-28 (BUG-78 trial, 8 Attempts, `catch_outward_authority_mm 15.0`); P = 2026-11-27_19-13-16 (10 Attempts at 10 mm, before the Learner merge).

## Findings

1. **BUG-158 passes** (owner verdict). All 10 `learner:` lines read "unusable 0", "rejected 44"; first line K 1.0820, 5811 rows (4684 free_cup, 1083 carried). No ADMISSIBILITY refusal. Replaying the Memory through the merged Learner reproduces the logged commands to about 1 mm; largest offset 17.4 mm lateral and 6.9 mm Apex (caps 30 and 20); merged and pre-merge code differ by under 0.5 mm on the same Memory. 52 rows written, all accepted. Landing error unchanged within the sample (standard error about 4 mm; sd 19 to 24 mm per axis in A and P).
2. **The 15 mm outward Catch authority trial leans positive.** B: 0 of 105 Throw-Landing Catches missed (Ball 2: 0 of 50), against about 5 of 10 Attempts lost to a missed Ball 2 Landing Catch in P. 28 Catches clamped at 15.0 mm; requested outward aim 16 to 36 mm, median 20; median shortfall past the limit 4.9 mm (11.3 mm at 10 mm); 27 of 28 caught. Scatter and Platform lateral motion are unchanged. Not proven: 8 Attempts, and B also ran the merged Learner.
3. **Ball Butler's Feed of Ball 2 lands 25 to 29 mm short.** Free-fall fit of the Mocap track to the Catch plane, 28 Ball 2 Feeds: dx mean / sd (mm) A -25.1 / 6.8, B -16.6 / 5.7, P -27.8 / 6.5; dy +4.9, +7.7, +6.5. Ball Butler lands 29 +/- 7 mm short of its own announced landing; the ball arrives with 6 to 7 % less horizontal speed than announced (inferred). Arrival time is within about 2 ms of the Schedule. In A and P every Feed with dx <= -26 mm was lost (11 of 11). Ball 2 Feed lost: A 6 of 10, P 7 of 10, B 2 of 8 (Fisher p = 0.096 for B against A and P). `columns_feed_bb_bias_mm` was [0, 0] in all three. Filed as BUG-160.
4. **A Catch re-aim clamped to 10.0 mm outward is dropped** by `resend_pos_tol_mm` (default 10.0, the same number as the default authority): logged RESEND-SKIPPED or BUMP-DECLINED WITHIN-TOLERANCE, so the Cup never moves. Seen in A Attempts 5, 6, 9 and P Attempts 3, 7, 10, all lost Feeds; at 15 mm the re-aim was taken. Filed as BUG-161; a fix is in progress this Session.
5. **The Hand MAX_DEVIATION ends (A7, B3, B7) are the known bridge-stall race.** In all three the bridge loop gap was 34 to 58 ms while the emitter was on time, the `:5557` drain held two knots and forwarded the newer (lag 46 to 52 ms), the frame reached firmware 4.7 / 7.7 / 6.2 ms after the playing frame's 70 ms cover ran out, firmware Hand C2 STOP "stamp gap" (#84, #94, #98), next frames refused, host plan moved on 80 to 300 ms before the host hold landed, guard tripped on the refused command against the held encoder. Hand within 0.001 to 0.04 rev of the firmware's own command, current under 4 A, bus 45.2 V, motor 32 to 35 C. A CATCH solve overlapped each stall (inferred from log timing). Since 2026-11-18, 15 of 17 Hand MAX_DEVIATION latches had a firmware STOP within 0.3 s before; about 0.10 per Attempt pooled (15 in about 149); 2026-11-28 had 3 in 18 (1.8 expected, P(>=3) about 0.27). Tracking margin on normal Throws: peak |cmd - meas| median 0.63 to 0.78 rev, max 1.58 to 1.66 rev, about 65 % of the 2.5 rev guard, unchanged from P. Progress notes on FEAT-40 and BUG-89.

6. **One Hand obstruction at the current limit (B Attempt 4).** While carrying Ball 1 down after its Catch the Hand was held up at about 2.6 rev for roughly 90 ms: measured velocity collapsed from -38 to -12 rev/s against a commanded -34, the current setpoint reached -69.9 A and the measured current -67.2 A (limit 70 A), then the Hand broke free and threw normally. Not commanded and not an end stop; the likely cause (inferred) is the lost Ball 2 from that Attempt's Feed under the Hand, which the tracker logged dwelling there 0.2 s earlier. Peak current on the other 373 Carry descents of the three Sittings was 39.1 A at most (median 11 A). Motor temperature 34.5 C before and after. It was shorter than the Hand stall guard's window (BUG-123), so no trip.

## Discussion

**Reframing of "Ball 2's Feed hits the Platform frame".** The owner saw Ball 2's Feed strike the Platform frame in most Attempts of A (1, 3, 4, 5, 6, 8, 9) and again in B (Attempt 4). That reads as a collision or geometry problem (a frame clearance at the Site, or the Cup in the wrong place). The fits say otherwise: the ball arrives where Ball Butler was aiming minus 25 to 29 mm, on schedule, with the Platform body where the Schedule put it. A Feed that lands 25 mm outward of Ball 2's Site is, by the owner's description and the Mocap track, near the Platform's edge (a frame strike is consistent with that, not separately shown), and it is beyond the 10 mm outward Catch authority so the Catch cannot meet it. The cause is an aim bias in Ball Butler, which a parameter already exists to trim; the owner confirmed the miss matches what he sees and chose the software trim.

**The Hand MAX_DEVIATION ends are not the overheated Hand motor.** Hand current was under 4 A, the motor 32 to 35 C, and the Hand sat within 0.04 rev of the firmware's own command at each latch; the deviation was the firmware command against a plan that had been held, not a motor that failed to follow. The base rate since 2026-11-18, with 15 of 17 latches preceded by a firmware STOP, puts these three in the known class.

**Ruled out for the Feed losses:** timing (arrival within about 2 ms of the Schedule), scatter (Ball 1's Feed has the same bias but 30 mm of authority and is caught; scatter is unchanged between bags), Platform tilt, and the Learner (P ran without the merge and shows the same miss).

**Tradeoff accepted for the next trials.** The 25 mm authority trial flies after the Feed bias trim so that the two changes are not confounded; the trim should remove most of the Feed misses, so any further gain at 25 mm is attributable to the authority. The cost is one more Sitting before the authority question closes.

## Owner decisions (2026-11-28)

1. BUG-158: pass.
2. BUG-78: try a larger value next, 25 mm; the 15 mm trial's verdict stays pending.
3. Trim Ball Butler's Feed bias in software: `columns_feed_bb_bias_mm` [-25.0, 5.0], 6 Attempts, pass at median |dx| of 10 mm or less and at most 1 of 6 Attempts lost to Ball 2's Feed.
4. Fix the clamped re-aim being dropped by the re-send tolerance now (BUG-161).

## Unknown

- Why the Feed bias reads about -25 mm now when it read +4 mm on 2026-10-11. No Feed-aim code changed between 2026-11-27 and 2026-11-28, and Ball Butler ran FW 10 in all three bags.
- Why Attempt 7 of B had a clean Feed (landed only 6.8 mm short, same logged throw parameters).
- Whether the 25 mm authority also needs the 10 mm re-send tolerance changed (BUG-161's fix decides).

## What flies next

1. BUG-160: 6 Columns Attempts with `columns_feed_bb_bias_mm` at [-25.0, 5.0], default 10 mm authority. The sign: the parameter is `landing - request` in the schedule frame and Ball Butler is asked to hit `aim_site - bias`, so a Feed landing at aim + (-25, +5) needs the bias [-25.0, +5.0].
2. BUG-78: 8 Columns Attempts at `catch_outward_authority_mm 25.0` with that trim in force. The nomination accepts 25 mm: for Columns Sep. 125, Apex 0.95, Dwell 0.27, the Catch re-aim figure's worst margin is 0.137, 0.111, 0.095 and 0.113 at 10, 15, 20 and 25 mm (2026-11-28, `nominate.nominate(cfg, aim=AimAuthority(30, out, 35))` at the launch limits); the node does not bound the parameter.

## Verification

Documentation and Item write-backs only; no code changed in this commit. Analysis numbers come from the three bags named above and their launch logs. Doc readers run 2026-11-28 with `python -m mapkit gen` then `pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_docs_links.py tests/sim/test_plans_index.py tests/sim/test_mapkit*.py -q`: result in the commit message.
