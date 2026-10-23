---
title: Two-ball columns sessions 2026-10-14 - why attempts end short - catch misses are stale aims and wide throws (no late wrong re-aim), SPLICE_TOO_LATE is the solve-time tail against a fixed 200 ms handoff budget, the four short feeds hit the cup wall after a 25-34 mm landing error, and the feed catch never sees the real ball because the tracker confirms it on a static Ball Butler marker; two fixes landed
type: investigation
date: 2026-10-14
status: tuned
phase: "two-ball-skill-stack — post-R6 reliability (columns sessions 2026-10-14)"
related_plan: skill-stack-dress-rehearsals-and-residue.md
related:
  - 2026-10-13-skill-stack-r5-sitting-8-gate-met.md
  - 2026-10-12-skill-stack-r5-bb-feed-no-drift-release-jitter.md
  - 2026-10-12-skill-stack-r5-outward-reaim-and-catch-evidence.md
  - 2026-10-11-skill-stack-r5-hop-late-limit-jerk.md
  - 2026-09-16-banking-saturates-on-small-lateral-offsets.md
  - 2026-09-16-tracker-correlation-follows-the-flight-in-progress.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/tracking/matcher.py
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py
  - tests/motion/test_skills_schedule.py
  - tests/motion/test_skills_executor.py
  - tests/ros/test_install_segment.py
  - tests/ros/test_ball_tracker_announced_motion.py
  - tests/ros/test_ball_tracker_gate.py
  - tests/ros/test_ball_tracker_flight_fit.py
  - tests/ros/test_skills_plan_bench.py
  - tests/sim/test_skills_gate.py
  - tests/ros/test_gui_replay_chart.py
  - tests/ros/test_gui_replay_digest.py
  - tests/ros/test_gui_replay_ball_history.py
  - logbook/2026-10-14-two-ball-columns-sessions-why-attempts-end.md
  - logbook/INDEX.md
subsystem:
  - skill-stack
  - tracking
  - ball-butler
  - motion
tags:
  - hardware-sitting
  - timing
  - tracking
  - catching
---

# Two-ball columns sessions of 2026-10-14: why attempts end short

## Summary

- Two sessions (S1 20:54, S2 21:27, local), 53 goals, 23 attempts ended short of their goal
  (refusals and the by-construction last-throw miss excluded). By first cause: **10 missed
  catches of a robot-thrown ball, 7 `SPLICE_TOO_LATE`, 4 missed feed catches (all S1), 2
  `LIMIT_*` ends.** S2 columns streaks with the feed counted: 10, 6, 8, 4, 3, 4, 3, 5, 7, 2, 3, 5, 7
  (median 4-5; sitting 8: median 5, one 32).
- **Not one miss came from a late re-aim toward a wrong estimate** (0 of 13 `MISSED_CATCH`
  lines; 0 of 11 bag-decomposed misses). The "sideways slide late in the catch" the owner saw
  on columns is the catch transit itself: the plan reaches the catch xy only at the ball's
  crossing, still decelerating (command ~140 mm/s), the platform trails the command by ~30 ms
  (26 mm mid-transit), and the cup crosses at 250-330 mm/s and overshoots 7 mm (caught) to
  10-14 mm (the five misses it contributed to). It is intrinsic to apex 0.95 / sep 125 / dwell
  0.27 at today's leg limits: the lateral transit has ~170 ms. Hops are at rest 45-120 ms
  before the ball; the hop "slides" are not explained by the data.
- **`SPLICE_TOO_LATE` is the right tail of the catch solve time against a fixed 200 ms handoff
  budget**: 2.7-5.1 % of first-CATCH dispatches per session (sitting 8: 3.4 %), a 40-80 %
  chance of one self-end in a 30-catch attempt. Not a regression. FIX 1: two more knots of
  handoff lead (325 ms, 250 ms of budget) plus the node waiting to the pinned cliff and a
  refusal timing line.
- **The four short feeds** landed 25-34 mm from the cup centre before any contact (19-31 mm
  short toward Ball Butler, 8-14 mm in -y), struck the cup wall ~130 mm above the plane from
  outside and bounced out; nothing on the structure was hit. Ball Butler's stroke, pitch and
  timing were identical on every feed; the request and the cup were where they should be. The
  feed bias is per LAUNCH (median x -17 mm S1, +6 S2, -7 sitting 8 with one command) and drifted
  -y inside S1.
- **The feed catch has never used the tracker**: `tracking/matcher.py` confirms Ball Butler's
  announced ball on the nearest marker within 200 mm of the announced start, with no motion
  test; six static `Ball Butler - N` body markers sit 78-142 mm from that start and one of them
  wins on every feed, the real ball having already risen out of the gate. The "flight fit" of a
  stationary point predicts a landing 0.43 s later at Ball Butler's position; the executor's
  identity gate correctly refuses it (the logged `-0.35 s from schedule`). FIX 2: confirmation
  requires consistency with the announced flight over consecutive frames.
- Live-only tracker defect found, not fixed: 9 of 124 throws (7 %) never got a converged fit
  live, yet the production tracker replayed offline on the same bag converges on every one at
  release +0.11 s. Their unfitted catch aims were 3x worse (median 29 vs 9 mm).

## What happened

Sources: S1 bag `~/Desktop/rosbags/2026-10-14_20-54-20`, log
`~/.ros/log/2026-10-14-20-54-20-258010-jetson-4122860/launch.log`; S2 bag `2026-10-14_21-27-22`,
log `2026-10-14-21-27-21-980308-jetson-4138247`; sitting 8 as the baseline. Firmware can-bridge
v30 / protocol 10; limits leg acc 5000 / jerk 200k / vel 350; dwell 0.27 s; apex 0.95; sep 125 mm;
catch plane 830; tracker-aimed catches; lateral authority 20 mm, outward 10 mm; BB command
identical on every columns feed (`in 2.5 s, flight 0.88 s, 3.29 m/s, aim correction (-30/-31,
-4/-5) mm`). Counting rule as the sitting-8 entry.

| S | pattern (n) | node | end | streak |
|---|---|---|---|---|
| S1 | self_toss 10 | 10/10 | done | 10 |
| S1 | hop 10, hop `reload` 10, hop `reload` 10 | 1/2, 2/3, 5/6 | SPLICE_TOO_LATE x3 (stop REST refused LIMIT_ACC -> hold) | 1, 2, 5 |
| S1 | columns_1ball_fed 10 | 4/5 | done (final throw) | 5 |
| S1 | columns 10 | 4/4 | MISSED_CATCH Ball 1 skill 6, Ball 2 skill 7 | 5 |
| S1 | columns 10 x4 | 2/2 each | MISSED_CATCH skill 1 = the FEED catch | 0, 0, 0, 0 |
| S1 | columns_1ball 10, 10, `reload` 5 | 2/2, 4/4, 5/5 | SPLICE_TOO_LATE; MISSED_CATCH skill 11 (throw landed -16,+73); done | 2, 4, 5 |
| S2 | hop 10; hop `reload` 10 | 4/4; 10/10 | MISSED_CATCH skill 6 (rim bounce); done | 4; 10 |
| S2 | columns 10 | 9/10 | done (final throw) | 10 |
| S2 | columns 50 x11 | 7/7, 9/9, 5/5, 2/2, 4/5, 2/4, 5/6, 6/6, 1/3, 4/4, 4/6, 7/8 | MISSED_CATCH x7 (Ball 2 at P1 x5, Ball 1 at P2 x2), LIMIT_ACC, LIMIT_JERK, SPLICE_TOO_LATE x3 | 6, 8, 4, 3, 4, 3, 5, 7, 2, 3, 5, 7 |

Refusals: 2 `REJECTED_BALL_IN_HAND`, 2 `REJECTED_NO_BALL` (one at the throw after a missed
hop-`reload` feed, 66 mm off), 2 `MAGAZINE_UNREACHABLE`. Release offset median +9/+10 ms (the
sitting-8 +20 ms step did not recur); arrival median -1 ms. No lane latch, no hand jam, no
emitter window over 42.7 ms. `QTM frames lost` only during the pre-goal BB calibration sweeps.

## Method

Nine read-only analysis units (Sonnet; one Opus for the catch decomposition) from one shared
brief, each under 80 tool calls with a handoff file; two independent verification units
re-derived the two load-bearing physics claims from raw data without seeing the conclusions.
Per-row artefacts: `temp/probes/u3_catches.csv` (137 robot-thrown flights: ground-truth landing,
tracker estimate at the freeze, aim lines, cup trace), `temp/probes/u4_feeds.csv` (45 BB
throws), `temp/probes/u6_throws.csv` (124 throws, fit status), the refusal and ledger CSVs. The
scripts live in the session scratchpad and are not kept; the recipes are the probes named below.

## Discussion

**1. Catch misses (10 attempts, 13 END lines) - the class table.** Every one had a re-aim of
more than 10 mm refused, skipped or clamped on that catch; none had an accepted re-aim (log census).
Bag decomposition of the 11 determinable misses: 5 a correct aim not delivered (3 no live
converged fit so the catch stayed on the learned prior while the ball landed 35-95 mm away; 1 the
fit arrived inside the contact window; 1 a 62 mm correction clamped to the authority); 3 wide
throws (ball 76-203 mm from the site, beyond any authority; two of them thrown from a ball
caught 73-82 mm off-centre); 2 rim bounces with the cup still moving; 1 the transit overshoot
alone (cup 7.8 mm from the true landing at -50 ms, 30 mm at the crossing, tracker correct to
4.7 mm). Plus 3 hop "misses" after a SPLICE_TOO_LATE end where no catch was planned at all.

*Withdrawn:* (a) "the platform corrected toward a wrong prediction" - 0 of 13; the pattern
exists only early and harmlessly (3 of 6 accepted self-throw re-aims moved 480-770 ms before
landing toward an estimate 9-19 mm worse; all caught). (b) "LIMIT_JERK/LIMIT_ACC are costing
catches" - 81 % of requested re-aims (46 of 57) were refused, but 37 of the 40 non-timing
refusals were followed by that catch succeeding on its old aim; the three that preceded a miss
are not provably causal. (c) The 2026-09-16 banking saturation - refused peaks were 1.15-2.8x
the limit on the transit after the catch, not the ~140k small-offset signature; tilt at contact
0.1-1.2 deg on columns.

*What the catch move is doing (verified on `/leg_cmd_executed` + forward kinematics, 19
catches):* the command is within 5 mm of its final xy at a median -35 ms (range -155..+30) and
still at ~140 mm/s at the crossing (750 mm/s at -150 ms); `cup_cycle.py` pins only POSITION at
the touch-down knot (lateral velocity a soft cost, target 0.7x the ball's lateral arrival,
weight 200; no lateral acceleration bound; the contact window's one hard row is vertical). The
measured platform is the command 25-35 ms earlier (rest residual 2.2 mm rms, so lag, not frame),
hence 250-330 mm/s at contact and 7 mm (caught) vs 10-14 mm (misses) of overshoot at +50 ms. At
this operating point the P2->P1 transit has about half-beat minus dwell = 170 ms, which the legs
cannot complete at rest within 5000 mm/s^2: arriving earlier is a limits / geometry (apex,
separation, dwell) / feedforward question, not a planner-constraint edit. Hops: at rest 45-120 ms
early; mechanism does not apply.

*Tracker at the freeze (0.25 s before the crossing):* error p50 8-14 mm, p90 15-45 mm; 45 % of
throws over 10 mm, 15-20 % over 20 mm; the LAST estimate is no better (11.0 vs 9.2 mm median),
so this is fit accuracy or ground-truth noise, not convergence lag. A 20 mm re-aim is the size
of this noise. Two-ball P1/P2 scatter of the throws themselves: sd 14-19 mm per axis, with a
+y tail to +82 mm on Ball 2 at P1.

**2. `SPLICE_TOO_LATE` (7 ends; 6 non-fatal RESEND refusals).** Accepted handoff-CATCH installs
(dispatch stamp -> install) p50 137-139 / p90 188-198 / max 223-231 ms in S1, S2 and sitting 8;
the seven ends had their reply 215-272 ms after the stamp: the same distribution's tail. Solve
cost rises with the aim offset (schedule-aimed p50 85-90, learned prior 116, tracker 126-146 ms).
Refuted: a RESEND queued ahead (the worker was idle >= 290 ms before every end dispatch), an
emitter stall (no window > 34.5 ms within 0.6 s), slower hop solves (hop p50 97 ms). The two
"solver did not answer" ends were the cliff too: at 2 extra knots the pinned cliff (~200 ms) sits
before the node's fixed 250 ms wait, so the wait never decided anything. `_reject_segment` dropped
the refused solve's duration, which is why sitting 8 read its 0.23-0.26 s request-to-refusal
as the solve time. Five of the six RESEND refusals were by construction: a free-standing re-aim
has 150-175 ms against tracker-aim solves of p50 126-146 (self-throw 169).

*Tradeoff accepted:* +2 knots of handoff lead moves every columns catch dispatch 50 ms earlier
(minimum gap between consecutive dispatches 0.350 -> 0.300 s; the predecessor's install, capped
at the cliff, completes >= 69 ms before). The seam does not move (pinned at the release knot), the
seed is the same planned state, `LEAD_S`/`CATCH_FREEZE_S` and the re-aim window are untouched,
`schedule.py` is not box-gated. A longer `SOLVE_BUDGET_KNOTS` (the free-standing lead) was NOT
taken: it would move the freeze and shrink the re-aim window one for one.

**3. The feeds (4 ends, all S1).** Ball ground truth from the raw markers on all 12 S1/S2
columns feeds: misses at 25.5, 33.9, 26.7, 25.0 mm from the cup centre, the two caught S1 feeds
before them at 16.0 and 19.7; every feed under ~22 mm was caught, some at 26-40 mm still were.
The track leaves free fall at z 956-968 mm, 37-48 mm outboard of the cup axis - the cup wall -
and never gets below 860; caught balls reach 817-838. BB hand stroke 7.15 rev, peak speed,
onset, pitch and throw time identical on every feed (encoder); C3 hand-timing slope +0.32
ms/min. Request (-62.5, 0), bias [0, 0], cup within 1.3-3.6 mm of the announced landing, frame
offset 0.5-4.3 mm. The step into the misses is mostly -y (vy deficit -8..-15 mm/s, ~0.5 deg of
launch direction) with the same stroke: ball seating/spin in BB's cup is the one untested
candidate. *Withdrawn:* "fell short and hit the Platform structure" (it was the cup wall);
BB release speed loss; a moved request; a timing (cup-contact) miss.

**4. Why the feed catch cannot help itself (the tracker confirmation).** The executor compares
`/balls time_at_land` with the announcement's `landing_time` (both wall time, plane 830, the
announcement sane: the ball crosses 830 at +7..+35 ms of it). `matcher._match_announced_balls`
confirms on the nearest non-excluded marker within 200 mm of the announced start; only
`Platform,Base` labels are excluded. Marker `Ball Butler - 7` at about (-884, -350, 1720) is
78-142 mm from the start and is the first CONFIRMED sample on all five feeds checked, 0.02-0.04 s
after throw_time; the real ball is already at z 1910 at throw_time - 57 ms (the announced start
is the pre-stroke cup, z 1739). The fit of a point at rest at 1720 predicts landing in
sqrt(2*890/9810) = 0.426 s at (-884, -350): `/balls` reads -0.34..-0.35 s from the announced time,
reproducing the logged -0.349..-0.359 at the CATCH-AIM tick. One S2 track later migrated to the
real ball and passed the gate (then clamped to 10 mm outward). *Not changed:* the executor gate
and `EXTERNAL_LANDING_TIME_BAND_S` - the gate did its job; the 2026-10-02 wrong-ball incident
stays closed. *Rejected:* excluding `Ball Butler` labels (QTM labelled the flying ball
`Ball Butler - N` on 2026-09-15; it would re-blind the tracker).

**5. Live-only no-fit.** 9 of 124 throws never had `landing_from_fit` on 150-800 live `/balls`
updates; one track id per throw; mocap healthy (283-353 Hz, max gap 38 ms); the production
`BallTracker` replayed offline on the same frames fits all of them from release +0.11 s. 8 of 9
are two-ball columns throws, 7 of them Ball 1 at P2. Cause under investigation (unit U8); the
catch aim is allowed to use the unfitted Kalman estimate meanwhile.

## Fix

**Fix 1 - `SPLICE_TOO_LATE` budget** (`schedule.py`, `executor.py`, `trajectory_node.py`).
`HANDOFF_LEAD_EXTRA_KNOTS` 2 -> 4 (handoff lead 13 knots = 325 ms; 250 ms of solve budget plus
quantisation; docstring carries the measurement). `executor.splice_knot_for` /
`splice_cliff_s` factor the splice-knot arithmetic `install_segment` already ran, so the node can
compute, BEFORE the solve, the cliff of a pinned splice from the record's release knots and wait
until the later of `t_wall + LEAD_S + dt` and that cliff minus `REPLY_TRANSIT_ALLOW_S` (never
past the executor's own refusal; REST and fresh origins unchanged; `InstallSegment.srv`
unchanged). Every post-solve refusal now logs one DEBUG line: which check refused (`node-wait`,
`transit`, `worker:<code>`), solve ms, plan ms, `t_inst`, the deadline, `k_s`, `k_wire` at the
worker and at the node, and the cliff.

One control cycle at 40 Hz, after the change: the catch is dispatched 325 ms (was 275) before
the pinned release knot; the executor builds the terminal from the learned prior / schedule
(the tracker re-aim still arrives later through RESEND, frozen at `LEAD_S + dt` before
touch-down); the worker solves from the plan's state AT the release knot (identical seed); the
reply splices at that knot while the emitter keeps sampling the old record. The predecessor
carrying the release knot installs in at most 231 ms of its own dispatch, 0.300 s before this
one. A RESEND is not snapped: its budget, freeze and deadline are unchanged. A hold still
supersedes an install (the post-solve check and the epoch check at install are unchanged). A
solve that still misses ends exactly as today - `SPLICE_TOO_LATE`, nothing installed, the END
cascade - no new failure mode on the wire; limits, feedforward and the emitter are untouched.

**Fix 2 - announced-ball confirmation** (`tracking/matcher.py`; `ball_tracker_node.py` passes its
logger as `debug_log`). An announced track is CONFIRMED only on a marker consistent with the announced
flight over consecutive frames: the spatial gate is taken around the announcement's PREDICTED position
(radius `announced_gate_mm` 200 + |v_pred| x 0.10 s, about 528 mm at a BB release, because the real
BB ball sits 230-380 mm from the announced path for its whole visible flight - the announced start is
the pre-stroke cup); the candidate's frame-to-frame continuation must move at 0.4-1.8x the predicted
speed and roughly along it (cos >= 0.5) for 2 consecutive frames, with history from 0.10 s before
throw_time and confirmation only at or after it; a CONFIRMED announced ball whose RAW marker moves
under 0.5 m/s for more than 60 ms while the announcement predicts over 2 m/s is demoted to ANNOUNCED
and that marker position excluded (raw marker, not KF speed: the KF velocity is seeded from the
announcement and decays far slower than 60 ms; only until 0.1 s before the announced landing, so a
ball at rest in the cup is never demoted). One DEBUG line per confirmation / demotion; the decisions
are kept in `tracker.identity_events`. Values are constructor defaults, not YAML. Jugglebot's own
throws confirm on the first in-flight frame as before (unit test + bag). The executor gate,
`EXTERNAL_LANDING_TIME_BAND_S` and `excluded_label_prefixes` are unchanged. `ball_possession.py`'s
CONFIRMED is its own hand-sensor vocabulary and does not read the tracker.

*Replay, S1 bag, six feeds (fresh tracker per feed, session scratch `f2/replay_win.py`, copied to
`temp/probes/columns_2026-10-14/f2_replay/`):* confirmation on the real ball +0.000..+0.002 s after
throw_time on 6/6 (before: the static `Ball Butler - 7` on 6/6); tracker landing time minus announced
at landing - 0.61 s: -0.002, -0.002, -0.005, -0.007, -0.007, -0.004 s (before: -0.35 s); landing xy
26-49 mm from the announced site (before: ~700 mm); no demotion fired. Not done: an S2 replay and a
whole-session replay with other balls in flight.

**Consequence to rehearse before trusting it on hardware:** the feed catch will now pass the identity
gate and RE-AIM from the tracker for the first time, inside the existing clamp (10 mm outward toward
BB, 35 mm inward, 20 mm across) and the feasibility gate - the same path every Jugglebot catch already
takes. First fed attempts of the next sitting: watch `CATCH-AIM skill 1 source=tracker` and the
matcher's confirmation DEBUG line in launch.log; a feed re-aim refused `LIMIT_*` leaves the catch on
the schedule point exactly as today.

## Verification

| item | (date, command, result) |
|---|---|
| F1 scoped | 2026-10-14 23:16, `pytest tests/motion/test_skills_schedule.py tests/ros/test_install_segment.py tests/ros/test_skills_plan_bench.py tests/motion/test_skills_executor.py -q -p no:cacheprovider`, 516 passed in 266.93 s |
| F2 scoped | 2026-10-15 00:14, `pytest ros_ws/src/jugglebot/jugglebot/tracking/tests tests/ros/test_ball_tracker_announced_motion.py tests/ros/test_ball_tracker_flight_fit.py tests/ros/test_ball_tracker_frame_stamp.py tests/ros/test_ball_tracker_gate.py tests/ros/test_ball_tracker_identity_wire.py tests/ros/test_perception_console_lines.py tests/ros/test_possession_replay.py -q -p no:cacheprovider`, 109 passed in 2.13 s (every test file importing the tracker) |
| Full gate | 2026-10-15 00:43-00:48, ./run_tests.sh --full (temp/logs/gate_columns_20261015c.log): parallel 1 failed, 6627 passed, 11 skipped in 291.07 s; serial 7 passed in 27.30 s. The one failure, tests/ros/test_skill_node_resend_param.py::test_the_cap_is_clamped_to_zero_to_five[0-0] (AttributeError: the node's _executor still None when asserted), is in an unmodified file, passed in the two preceding full runs of the night (gate_columns_20261015.log, _b.log, whose only failures were the GUI node-check tests fixed in the test-infra commit) and passes 5/5 twice in isolation at 00:50: recorded as a load flake (three full gates and four peer sessions on the box) |
| Dispatch-monotone probe | 2026-10-14, session scratchpad `probe_handoff_lead.py` (compile_columns / compile_one_ball, apex 0.95 / sep 125 / dwell 0.27 / n=10): monotone for 2..8 extra knots, min gap 0.350 / 0.300 / 0.250 s at 2 / 4 / 6; now pinned by `test_skills_schedule.py` |
| Catch-arrival claim | 2026-10-14, unit V1 own FK on `/leg_cmd_executed` + `/robot_state` (S2, 19 catches): command within 5 mm at median -35 ms, 138 mm/s at the crossing; measured lag 25-35 ms; rest residual 2.2 mm rms |
| Feed landing claim | single-source: unit U4's own gravity-fixed fit of the raw descending markers (`temp/probes/u4_feeds.csv`, 45 BB throws); unit U7 independently put the 830 crossing at +7..+35 ms of the announced time on 4 feeds; the planned second-method re-fit (V2) ran out of memory on the loaded box and was not completed |


**Gate note (test infrastructure, separate commit).** The first full run on the shared tree failed only
`tests/ros/test_gui_replay_{chart,digest,ball_history}.py::test_node_check_module(s)`: `node --check` on
the unmodified GUI replay ES-module files, because node walks up from `ros_ws/gui/js/replay/` to a stray
`/home/jetson/package.json` (`{}`, 2024) and parses them as CommonJS. They had been SKIPPED at the
sitting-8 gate (node not on that shell's PATH), so 2026-10-15 was their first run on this box. A module
marker in `ros_ws/gui/` is NOT the fix: `test_gui_geometry.TestLegacyFilesRemoved` pins that directory
free of `package.json` (the second full run failed exactly there). Fix: the three tests check a COPY of
each file next to a `{"type": "module"}` marker in a temp dir (`--check` only parses), so the verdict
is the file's own wherever the box's home directory points.

## Open (owner decisions, with the numbers that frame them)

- **Catch arrival at the operating point.** The ~170 ms lateral transit at apex 0.95 / sep 125 /
  dwell 0.27 cannot end at rest within leg acc 5000 mm/s^2; the cup crosses at 250-330 mm/s with
  a 30 ms tracking lag and 7-14 mm of overshoot. Levers, all owner-level: raise the leg limits
  (the 15 A clamp; ON HOLD), change the geometry (apex / separation / dwell; the oval arc's first
  rung is 150 mm / 0.50 s), or close the 30 ms lag and the overshoot with the acceleration
  feedforward chapter. A hard lateral-velocity row at touch-down would trade directly against
  `LIMIT_JERK` and is not recommended without one of the above.
- **Hold the catch aim on the prior when no fit exists by the freeze** - the unfitted estimate
  is 3x worse (median 29 vs 9 mm). S-sized; a behaviour change to the catch; decide after U8.
- **Lateral authority vs scatter.** 20 mm across / 10 mm outward against a two-ball landing
  scatter of sd 14-19 mm per axis (tails to +82 mm) and a freeze-instant tracker noise of ~10 mm.
  Raising it needs a box re-sweep and buys little while 81 % of 10-40 mm re-aims are refused at
  the limits.
- **Per-launch feed bias.** Median x -17 / +6 / -7 mm across three launches with one BB command:
  a fixed `columns_feed_bb_bias_mm` cannot serve; the per-launch BB calibration is the suspect
  (V2). With Fix 2 the feed catch can take up to 10 mm outward / 20 mm across from the tracker;
  the 25-34 mm misses would have been cut to 15-24 mm. A higher outward authority for the FEED
  catch only, or an online feed-bias estimate from the first feeds, are the remaining levers.
- **Wide throws from off-centre catches** (n = 2: +73 / +82 mm seats then 200 mm throws): an
  observation to watch, no rule proposed.
- **Live-only no-fit** (U8 in progress): the next diagnostic is per-track fit sample / rejected
  counts (a `BallState.msg` field = an interfaces rebuild, or a DEBUG line in the node).
