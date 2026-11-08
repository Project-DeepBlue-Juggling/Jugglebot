---
title: "Sitting 2026-11-06 (13:19) debrief: first powered run of the merged learner-generalisation stack - the stack caused no Attempt end; Sep. 170 separated Legs 1 and 2 from their base joints; four early ends were ball collisions"
type: investigation
date: 2026-11-06
status: complete
phase: "learner-generalisation first powered run"
related_plan: learner-generalisation.md
related_issues: []
files_changed:
  - logbook/2026-11-06-sitting-debrief.md
  - logbook/INDEX.md
  - tests/motion/test_migrate_memory_schema_v2.py
  - plans/active/learner-generalisation.md
  - map/feature-first-sitting-on-the-merged-learner-generalisation-stack.md
  - map/feature-count-scheduled-stream-holes-and-downlink-datagram-loss-in.md
  - map/bug-legs-1-and-2-left-their-magnetic-base-joints-on-columns-at.md
  - map/bug-nomination-margin-is-higher-than-the-live-plan-s-it-plans-at.md
  - map/bug-mid-air-ball-collisions-end-half-of-two-ball-columns.md
  - docs/runbook.md
  - docs/project-map.md
subsystem: [motion, ros, tracking]
tags: [safety, dynamics, testing]
---

# Sitting 2026-11-06 (13:19) debrief

Launch log `~/.ros/log/2026-11-06-13-19-19-766420-jetson-1466025/launch.log`, bag
`~/Desktop/rosbags/2026-11-06_13-19-19`. One Runbook row flown on purpose, FEAT-43 (the first powered run
of P3a, P3b, P1 and P4 of plan `learner-generalisation.md`, merged that morning); FEAT-41 and BUG-58 are
passive rows the same launch can answer. The three analysis reports and their scripts are kept under
`temp/probes/sitting_debrief_2026-11-06/` (`unit_learner.md`, `unit_missed.md`, `unit_detach.md`).

## Results

| Row | Verdict | Evidence |
|---|---|---|
| FEAT-43 | pass (owner, typed 13:33; confirmed at the debrief) | Clause 1: `skills/check` and `skills/nominate` answered before power (launch.log 369, 443). Clause 2 as written is NOT met: no Sep. 100 Attempt read 20/20 (they ended on collisions); no Attempt ended on an `ADMISSIBILITY` refusal. Clause 3: first Throws at Sep. 170 within 5.2 mm of the commanded Apex, two Throws only. Closed. |
| FEAT-41 | pass (owner confirmed) | `can-bridge v40`; three `downlink summary` lines (launch.log 2495, 4086, 5136), 12613 setpoints, lost 0 and queue_dropped +0 on all; seq_gaps +3 in the arm period that ended in the E-stop. Closed. |
| BUG-58 | not exercised | No LANE_REFUSED end. Nine `firmware scheduler STOP` WARNs, two of them 0.2 s after an Attempt's Start (Attempts 2 and 6), neither ended an Attempt. |

Attempts (all two-ball Columns, Apex 1.00, Dwell 0.27, Throws 21; goal and end lines of launch.log):

| # | Sep. | Goal / end line | End | Class |
|---|---|---|---|---|
| 1 | 100 | 957 / 1557 | MISSED_CATCH, 19/19 | collision 0.14 s before the final landing (closest 65 mm) |
| 2 | 100 | 1639 / 1950 | MISSED_CATCH, 5/5 | cannot tell (closest 84 mm, no velocity jump) |
| 3 | 100 | 1967 / 2228 | MISSED_CATCH, 5/5 | collision (71 mm) |
| 4 | 170 | 2292 / 2498 | 2/2, E-stop button at 1793931869.57 | Legs 1 and 2 left their base joints at 1793931867.53 |
| 5 | 125 | 3119 / 3406 | MISSED_CATCH, 6/6 | collision (76 mm) |
| 6 | 125 | 3424 / 4018 | COMPLETED 20/20 | closest 92 mm |
| 7 | 135 | 4186 / 4489 | MISSED_CATCH, 9/9 | collision (73 mm) |
| 8 | 135 | 4509 / 5013 | COMPLETED 20/20 | closest 107 mm |

## Symptoms

- Attempt 4 (Sep. 170): the owner saw "the aggressive lateral movements" pull Legs 1 and 2 out of their
  magnetic base joints and pressed the E-stop button. No damage. The Legs were refitted and the Platform
  re-levelled before Attempt 5.
- Five Attempts ended MISSED_CATCH early; the owner saw mid-air collisions.

## Diagnosis

**The new stack.** 208 `ADMISSIBILITY` lines: 100 `ADMISSIBILITY-EARLY ... the early resolution is sent`,
0 `re-checked at dispatch`, 100 `passed m=`, 8 `passed the envelope`; no refusal, no
`NO_ADMISSIBLE_COMMAND`, no `CHECK_NOT_HONOURED`, no refused goal. The early check ran 174 to 324 ms before
dispatch (median 291) and the candidate moved 0.0 mm on every one. Every goal was nominated OK (about 2 s
uncached, 0.0 s at accept from the cache). `learner:` lines read rows 3365 to 3424; K 1.0773 to 1.0806 over
the Sitting. No install was late in the 2 s before any end.

**The Learner.** Apex error against the commanded 1.00 m, mean / RMS / worst in mm: Sep. 100 +0.4 / 13.6 /
+26.9 (29 Throws); Sep. 125 -2.2 / 25.9 / -50.3 (26); Sep. 135 +0.4 / 7.2 / -15.4 (29); Sep. 170 +1.7 / 3.9 /
+5.2 (2). Lateral landing error 29 to 32 mm RMS at every separation. First Throws at the new Sites: Sep. 135
-4.6, +7.3, -15.4, -0.2 mm; Sep. 170 -1.8, +5.2 mm. The command stayed within 9.4 mm of the model command in
Apex (authority 20) and about 20 mm laterally (authority 30). The first Throws after the Legs were refitted
(Attempt 6) were 41 and 43 mm low; not explained.

**The Leg separation (Attempt 4).** Measured from the bag (encoder forward kinematics against the Platform
mocap body, `/leg_cmd_executed`, `/link_status` clamp counters) and the install lines:

| Quantity | Clean today, worst | Sep. 170 |
|---|---|---|
| Planned Leg speed / acceleration (install line) | 361 mm/s / 4672 mm/s^2 | 486 / 6296 (81.0 % / 78.7 % of 600 / 8000) |
| Planned Leg speed above the bridge's 249 mm/s feedforward cap | 112 mm/s | 237 mm/s |
| Lead-clamp ticks, Legs 1 / 2 | 29 / 36 in 21 transits | 55 / 65 in 2 transits |
| Platform lateral speed per transit | 1228 mm/s | 1345 (first transit), 1205 (second) |
| Platform lateral acceleration per transit | 33.3 m/s^2 (one transit) | 34.2, 15.0 |
| Estimated Leg tension (not checked against a load cell) | 47 N | 59 N (Leg 1, first transit), 32 N |
| Transit length | 89 to 156 mm | 188, 187 mm |

The joints let go at the END of the second transit (the Catch of Ball 2 arriving near x = -90 mm): the
mocap pose left the encoder-implied pose by 14 mm at 1793931867.53, 61 mm 50 ms later, 377 mm by 1868.08, with
the encoders nearly still. The E-stop button followed 2.0 s later. The gate and the nomination bound Leg
velocity, acceleration and jerk; nothing bounds Leg tension, Platform lateral acceleration or joint pull-off,
and nothing knows the can-bridge caps the Leg velocity feedforward at 3.5 rev/s (249 mm/s). The nomination
printed 0.194 for this goal; the live checks on the same Segments read 0.075 to 0.258, because the nomination
plans at the nominal Sites and the live Catch aims carried the 10 mm outward re-aim at each end.

**The missed Catches.** From `/balls` in the bag: in Attempts 1, 3, 5 and 7 the two balls came within 65 to
76 mm centre to centre (diameter 74 mm) near the Apex and both tracks jump 350 to 540 mm/s laterally at that
instant; the completed Attempts had no approach under 92 mm and no jump above 30 mm/s. A Tracker identity swap
at close range was not excluded with raw markers, but the victims' later tracks diverge by 400 to 900 mm.
Closest approach was about 70 mm at every separation. Attempt 1 is not the BUG-67 signature (the ball was
deflected 0.14 s before landing; the completed Attempts read their final Catch as seated). BUG-68 was not
exercised: Attempt 4's first end precedes the fault by 0.93 s. The ENDED summary line names a later skill than
the first failing Catch in Attempts 2, 3, 5 and 7.

## Discussion

**A premise of the row was wrong.** The row sent the owner to Sep. 170 because offline nomination gave it a
margin of 0.21 while Sep. 200 passed at 0.08, and the P4 entry had shown a thin margin failing in the sim. The
reasoning treated the nomination margin as the measure of how hard a goal is on the machine. It is not: it is
headroom of the planned Leg kinematics against the Sitting limits, and at Sep. 170 the plan was inside those
limits while the machine was not inside anything it had flown before (Leg speed 35 % above the day's worst,
twice the excess over the bridge's feedforward cap). A row that takes the robot to a configuration it has never
flown should say how far outside the flown envelope it is in Leg speed and transit length, not only what the
nomination printed. The earlier flight at Sep. 250 was a one-ball hop at 350 / 5000 with a whole flight for
each transit; it says nothing about a 0.3 s two-ball transit.

**Why the joints let go is not settled.** Two readings fit: the first transit's tension began to unseat the
joints and the second finished it; or the arrival of a long, fast transit rolled a joint out while its axial
preload was near zero. The mocap-against-encoder check resolves 10 to 20 mm and saw nothing after the first
transit. Peak lateral acceleration does not separate Sep. 170 from the clean Attempts (one Sep. 125 transit
matched it); planned Leg speed, the feedforward shortfall and transit length do. A bench pull-off test of one
joint would decide between the readings.

**Owner decision: no bound now.** The consequence was minor, and a limit set from one event risks being too
conservative; the owner will be mindful of the Patterns requested. The numbers above are on BUG-72 for the day
a bound is wanted. Making the nomination plan at the re-aim extremes would move the acceptance edge, so it is
filed as an owner decision (BUG-73) and not changed here.

**The pass.** FEAT-43's Sep. 100 clause was written as 20/20 and was not met. The owner judged the row passed:
the row exists to show the merged stack runs on the powered robot, the stack caused none of the ends, and
Sep. 100 leaves 26 mm between two 74 mm balls. The new-Site evidence is thin (Sep. 135 is 5 mm from Sites the
memory holds; Sep. 170 gave two Throws, one of them a model-only command), so T-H2 is recorded as supported,
not as closed beyond doubt.

## Fix

No code change. Written back: FEAT-43 and FEAT-41 closed; BUG-72 (Leg separation, owner decision recorded),
BUG-73 (nomination margin) and BUG-74 (ball collisions) filed; plan `learner-generalisation.md`: P3a complete,
the first powered run recorded, P3b in progress on BUG-73, a note on P2. Earlier the same day the 89
uncommitted rows of `temp/learn/jugglebot/memory.csv` were committed at the owner's ask (`edd98bff`), which
unblocks the auto-pull.

## Verification

The numbers above come from the launch log and the bag named at the top; each unit report states its
grep or script. Gate on the debrief's tree: 2026-11-06 14:07, `./run_tests.sh`: 8754 passed, 1 failed, 49
skipped + 7 serial, RESULT FAIL. The failure was `tests/motion/test_migrate_memory_schema_v2.py::test_committed_corpus`,
which pins the committed `memory.csv` at 3276 rows; the memory-rows commit `edd98bff` made it 3365 and its
message named only two of the three tests that read the file (the third builds the path with `os.path.join`,
so `grep "temp/learn/jugglebot"` missed it). Re-pinned to 3365 in its own commit (the v1 file no longer grows;
carried 977 and the 175 rows at 0.125 are unchanged). 2026-11-06 14:16, `./run_tests.sh` after the re-pin:
8755 passed, 49 skipped + 7 serial, RESULT PASS.

## Outcome

The merged stack is flown: checks, nomination and the Learner behaved as built and ended no Attempt. What it
exposed is outside the Learner: acceptance that sees Leg kinematics only, and ball collisions as the main
reason a Columns Attempt ends early.

## Open Questions

- What the base joints hold, axially and off-axis (bench pull-off test), and whether the bridge's 3.5 rev/s
  feedforward cap should rise (a firmware change) or the gate should know it. Owner decision; none taken.
- Whether the nomination should plan at the re-aim extremes (BUG-73).
- What sets the 60 to 78 mm lateral drift of the colliding flights (BUG-74); P2's release telemetry is the
  measurement.
- The Destination says a Pattern can be changed mid-Pattern; no operator channel calls
  `request_beat_change` yet, and a per-beat rotation of the Pattern is refused until P2 chooses its frame.
