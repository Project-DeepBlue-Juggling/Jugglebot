---
title: "Robot Sitting 2026-11-28 evening debrief: the Feed trim failed because BB's static aim correction is the shortfall, and 25 mm of Catch authority lifted Leg 1 off its Base"
type: investigation
date: 2026-11-28
status: complete
phase: "Sitting debrief"
related_plan: learner-efficacy.md
related_issues: []
files_changed:
  - logbook/2026-11-28-sitting-debrief-evening.md
  - logbook/INDEX.md
  - map/bug-ball-butler-s-feed-of-ball-2-lands-25-29-mm-short-of-its.md
  - map/bug-two-ball-columns-runs-end-on-a-missed-catch-of-ball-2-with.md
  - map/bug-a-catch-re-aim-that-passes-every-leg-limit-can-lift-a-leg.md
  - plans/active/learner-efficacy.md
  - docs/records.md
  - docs/project-map.md
  - docs/runbook.md
subsystem: [skills, ball-butler, legs]
tags: [sitting, debrief, columns]
---

## Summary

One launch, sheet 2026-11-28-robot (launch log 2026-11-28-19-38-37, bag 2026-11-28_19-38-37): 16 Columns Attempts (Sep. 125, Apex 0.95, Dwell 0.27, Throws 21). Step 7 flew six at the default 10 mm Catch outward authority with `columns_feed_bb_bias_mm` at [-25.0, 5.0]; step 9 flew ten at 25 mm with the trim still on. Four Attempts completed, 12 ended MISSED_CATCH (10 of Ball 2 at its Feed or first Catch, one of Ball 1), no Hand MAX_DEVIATION end. Before the Sitting the owner adjusted one Mocap camera and re-calibrated QTM. The morning's Sittings were debriefed in `2026-11-28-sitting-debrief`; the owner confirmed both verdicts below on 2026-11-29 00:05.

| Item | Step | Verdict | What happened |
|---|---|---|---|
| BUG-160 | 7 | fail | Ball 2 Feed median \|dx\| 15.3 mm against the 10 mm bar; 3 of 6 Attempts lost to the Feed (bar: at most 1) |
| BUG-78 | 9 | fail | At 25 mm, Leg 1 separated from the Base in the last Attempt on a 30.8 mm re-aim that passed every Leg limit; 4 of 10 completed |

BUG-163 is filed from the Leg event. BUG-161's Item is complete and was not touched.

## Big picture

What went well: the fix for BUG-161 flew and was not implicated, the trim did exactly what it was asked, and the loss of Leg 1 was caught on a robot that came back whole, with the near-miss at 10 mm recorded beside it.

What did not: the Ball 2 Feed still landed 15 mm off the Site on median and cost three of six Attempts. At 25 mm a Catch re-aim lifted Leg 1 off its Base joint; only 4 of 10 Attempts completed, and five mid-air collisions went unexplained.

What it means: the Feed shortfall is not the steady 25 to 29 mm bias we trimmed. It moved from about 8 mm to about 35 mm within one launch with nothing changed in the log, and the lead is Ball Butler's own static aim correction table, which in the morning accounted for the shortfall to within a few millimetres. A fixed trim could not follow that, so the hypothesis is withdrawn. Separately, the Leg gate bounds along-Leg motion, and the failure happened at the Base joint with every Leg value at 58 to 69 % of its limit, so 25 mm is retired and a lower Leg acceleration limit replaces it as the working guard.

Next steps. BUG-160: refit Ball Butler's aim table against the freshly calibrated Mocap, then fly 6 Columns Attempts with the trim at zero. BUG-78: inspect the Leg 0-2 Base joints by hand, then fly 8 Attempts at the 15 mm default and the 4500 mm/s^2 Leg limit once that change is on main. BUG-163 is flown by that same trial. BUG-84 and BUG-74 gain notes only.

## Findings

**The Feed.** The trim moved BB's aim as intended: BB was asked to land Ball 2 at the Site minus the bias, the Catch prior stayed at the Site and Ball 1's Feed was untouched. Ball 2 Feed dx against the Site in step 7 (schedule frame): +5.6, +14.1, +16.6, +13.3, +20.5, +24.2 mm. Step 9's ten: +13.4, +29.7, +10.6, +34.1, +8.5, +10.8, -4.0, -11.0, -15.3, -9.9. BB's shortfall against its own announced landing: morning A -29.2 mm (sd 4.8), B -24.6 (sd 9.3); evening Attempts 1-12 -8.2 (sd 8.7), Attempts 13-16 -35.0 (sd 4.6). BB calibrated once per launch and its base pose is logged once, so the jump is unexplained.

**BB's aim correction.** `ros_ws/src/jugglebot/resources/throw_affine_correction.json` is [[0.9306, -0.0232, 37.58], [-0.0171, 0.9853, 15.62]], 116 pairs in BB-local xy mm, applied by `ball_butler_node._apply_aim_correction` just before the throw solve; the announcement is built from the uncorrected target. At the Columns Feed target it amounts to about (-30, -3) mm in robot x, y. Last refit 2026-10-09; before that 2026-06-10. In the morning the ball landed at announced plus correction to within +0.8 mm (A) and +5.4 mm (B) in x. Evening Attempts 1-12: ball minus (announced plus correction) = +21.8 mm; 13-16: -5.0 mm. Double application is ruled out (it would read about -60 mm).

**The Legs.** In the last Attempt (start at launch log line 7024, skill 17) a RESEND re-aim of 30.8 mm (dx -21.5 outward, dy -30.8 across, the across demand clamped from -51.5) was installed 0.567 s before contact. Realized Leg peaks were 348 mm/s, 5015 mm/s^2 and 137k mm/s^3 (58 to 69 % of the limits 600 / 8000 / 200000). The Platform rose about 8 mm above its median (184.5 mm) and tilted 4.8 deg within 125 ms; Leg 1 reseated on the deceleration (inferred from Mocap and currents). The 21.5 mm outward part was only possible at 25 mm authority (about 6.5 mm at 10). Near-miss at the default 10 mm: step 7 Attempt 3, last Catch (skill 20), 5754 mm/s^2, Platform z 179.3 mm, tilt 2.4 deg. These are the maxima of realized Leg acceleration over 353 Catches; every morning Catch stayed at or below 4394 mm/s^2, +1 mm of z and 2.9 deg. Medians match the morning (lateral velocity 1316 / 1247 / 1151 / 1198 mm/s, lateral acceleration 20.9k / 20.2k / 17.5k / 17.8k mm/s^2 for A / B / step 7 / step 9; Base displacement per Catch 0.5 to 0.6 mm in every set). The gap: `validate_cycle` and the bump gate check only along-Leg velocity, acceleration and jerk on the plan; nothing checks Leg axial tension, lateral load at the Base joint or Platform lateral acceleration.

**The trapped Ball.** `tools/trapped_ball_finder.py` (report `temp/logs/trapped_2026-11-28_19-38-37.txt`): 19:49:56.776 to .873, Hand at 1.64 rev against a command of 0.07, peak 69.8 A, trapped marker coincides at (-33, 40, -2) mm, Ball 107 (the owner's "Ball2 jammed under hand (briefly)", step 9 Attempt 6). The second such event in one day (12:35:40, 67.2 A). BUG-84.

**Collisions.** The owner saw mid-air collisions in step 7 Attempt 5 and step 9 Attempts 2, 3, 4 and 8. In the Feed's own flight window the closest marker pair was 102 mm or more in all 16 Attempts, so the trim did not put the Feed's path near Ball 1. The two 0/2 Attempts show Ball 2 landing -95 and -71 mm in x after the Feed Catch. Cause undetermined; BUG-74.

**Mocap re-calibration.** The QTM world frame rotated (robot-frame yaw -11.8 deg in the morning, -41.2 deg in the evening, origin about 130 mm away). The Robot frame absorbs the rigid shift, but in it BB's base origin moved (+10.5, +14.4) mm and its heading +0.8 deg, about 10 mm of lateral aim at 750 mm range (inferred as a contributor, not demonstrated). Base-marker fit rms 0.8 to 1.1 mm in the morning, 1.35 mm in the evening.

## Discussion

**The withdrawn hypothesis.** The morning's reading was "BB lands 25 to 29 mm short, steadily, so trim it". Two data points broke it. The evening shortfall was -8 mm for twelve Attempts and -35 mm for the last four with nothing changed in the log, so no constant is right. And the morning's landing equalled announced plus the static correction, so the shortfall was a property of the table, not of the throw. A fixed trim fits one of these states and is wrong in the other by up to 35 mm; it would also have been the wrong class of fix, correcting an output of a table whose inputs (the Mocap frame, BB's base pose) just moved. Hence the owner's decision: refit the table against the new Mocap and return the trim to [0, 0]. The jump between Attempts 12 and 13 is not explained and the refit does not claim to explain it.

**Why the BUG-161 fix was cleared.** The Leg event came with a re-aim of 30.8 mm, so the new rule (judge a clamped re-aim on the fit demand) was an obvious suspect. Re-aims taken per Catch were 0.96 (A), 1.27 (B) and 1.25 (step 7); re-aims taken at 10 mm or less were 12 and 10; both flagged re-aims exceed the 10 mm tolerance, so the old rule would have taken them too. The fix changes nothing about them.

**Ruled out for the Leg event.** The Feed bias (the Catch prior does not move), Mocap-driven demand growth (outward demand median 21.3 mm in A, 19.4 in step 7) and lateness (re-aim leads as in the morning). What remains is the size of the late lateral re-aim and the load it puts on the Base joint, which no gate sees.

**Why 4500 and 15.** The owner chose the Leg acceleration limit 4500 mm/s^2 and the default outward authority 15 mm from the morning's benign envelope at 15 mm: RESEND up to 34.9 mm at 0.544 s or more before contact, BUMP up to 28.8 mm at 0.265 s or more, realized Leg acceleration at most 4394 mm/s^2, tilt at most 2.9 deg. 4500 sits just above that maximum and below both bad events (5015 and 5754). The cost: any Catch that today needs between 4500 and 8000 mm/s^2 is now refused or re-planned, so some Catches the Learner could have saved will be given up, and the nomination margins must be re-read at the lower limit. 15 mm costs the outward reach 25 mm would have given; the clamp was still binding at 15 (median demand 20 mm), so some Ball 2 Feeds stay out of reach until BUG-160 fixes the Feed itself.

**Options not taken (owner, 2026-11-29), recorded in BUG-163 and not built:** cap the combined lateral re-aim vector at 30 mm (today 30 across plus 25 outward allows 39 mm); a Mocap lift guard that aborts on tilt above 3.5 deg or z more than 3 mm above the median; longer term a Leg tension check (J^-T W including inertia) in the gate. The guard and the cap act on the symptom or on a proxy; the tension check is the one that closes the gap, and is the largest piece of work. The owner also asked for the Leg 0-2 Base joints to be inspected before the next Sitting.

## Paperwork

- Records and Memory merged (`python tools/runtime_records.py merge --resolve-records keep-runtime`): 101 rows, 6,061 total, no refusal.
- Sheet 2026-11-28-robot: flushed, derived, confirmed 2026-11-29 00:05, closed. No held branches, so nothing went through `tools/sitting_merge.sh`.
- Trapped-Ball scan: one stall with a trapped marker, 19:49:56 (Findings).
- Follow-ups: BUG-163 filed; Progress notes on BUG-160, BUG-78, BUG-84 and BUG-74; E5 of `plans/active/learner-efficacy.md` updated.

## Estimates

The sheet's Items estimated 40 minutes of flying and the 16 Attempts took 14 minutes (19:40 to 19:54), so an Item's `minutes:` runs about three times high for Columns Attempts.

## Verification

- 2026-11-29, `pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_docs_links.py tests/sim/test_plans_index.py tests/sim/test_map_items.py tests/sim/test_mapkit*.py tests/sim/test_sitting_sections.py tests/sim/test_sitting_debrief_protocol.py tests/sim/test_runtime_records.py -q`: 469 passed in 46.82 s. Documentation, Items and records only in this commit; the default gate was not run for it. The limit and authority changes land in their own commit with the full gate.
