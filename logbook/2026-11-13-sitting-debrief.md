---
title: "Robot Sitting 2026-11-13 debrief: the LANE_REFUSED ends were a :5557 drain discarding one knot (newest-only is right, the cover margin is 7-14 ms), carry_zvd drives a 5 Hz Hand stop-go, turned Columns at 90 deg is a margin cliff, BUG-80 and BUG-82 resolved, and four readings were withdrawn"
type: investigation
date: 2026-11-13
status: complete
phase: "Sitting debrief"
related_plan: ""
related_issues: []
files_changed:
  - logbook/2026-11-13-sitting-debrief.md
  - logbook/2026-11-09-landing-offset-parameter.md
  - logbook/INDEX.md
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/nominate.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - tests/ros/test_teensy_bridge_node_bridge_diag.py
  - tests/ros/test_skill_node_landing_offset.py
  - tests/motion/test_skills_landing_offset.py
  - tests/motion/test_cup_zvd.py
  - tests/sim/test_mapkit_sheet.py
  - mapkit/sheet.py
  - .claude/commands/sitting-sheet.md
  - CLAUDE.md
  - docs/project-map.md
  - docs/runbook.md
subsystem: [can-bridge, skill-stack, learner, tooling]
tags: [sitting, hardware, diagnostics, debrief]
---

# Robot Sitting 2026-11-13 debrief

Sheet `2026-11-13-robot`, four launches, 45 steps (step 39 skipped as already done). Launches 1 and 2 flew `main`-equivalent code; launches 3 and 4 flew the candidate branch with IDEA-22 and BUG-80. Times are local (UTC+11).

## Summary

| Item | Typed | From the data | Outcome |
|---|---|---|---|
| IDEA-10 | pass | pass | complete. 20/20 at 12:03:51, Apex difference 100.9 mm, Hand iq 35.9 A over that Attempt |
| BUG-80 | pass | pass on a proxy slope; landing follows the offset | complete, merged by the owner without the gate (see Discussion) |
| FEAT-46 | pending | pass | complete. 108 STOP records, 0 overwritten; they located the drain cause (BUG-89) |
| BUG-82 | pass | pass on before/after (not on the Item's in-run control) | complete. 0 of 41,952 Leg windows over 25 ms after, against 245 of 27,046 over 50 ms before |
| IDEA-16 | pass | pass (bench report run offline at this debrief); lighting unrecorded | complete |
| FEAT-39 | fail | fail | in_progress. 30 consecutive Catches only at Dwell 0.40; 0.30 ends on LIMIT_JERK |
| IDEA-11 | fail | fail | in_progress. 45 deg 20/20 once; 90 deg 0 of 5 past 2 Throws |
| IDEA-22 | fail | fail | in_progress. "3 deg tilt-back makes 90 deg flyable" refuted at today's limits |
| FEAT-32 | pass | fail (typed pass meant "row flown") | in_progress. ZVD works on the bias, drives a Hand stop-go; B arm confounded |
| BUG-63 | pending | unresolved | stays open. 180 mm first-Throw miss on neither plant |
| BUG-58 | pending | pass by the letter, premise refuted | stays in_progress. "Host stream clean" is false (BUG-89) |

## Inventory

- **Launch 1 (11:42 to 12:18, steps 1 to 17).** FEAT-39 ladder on the oval (Dwell 0.50 to 0.30), the Columns canary once, columns_highlow (one 20/20 in 9 Attempts on the trained plant, 8 Attempts on a fresh plant), the Feed rows. Ends: MISSED_CATCH dominant, 2 LANE_REFUSED (11:46:53 at 7/8, 11:49:33 at 5/6), 3 NO_ADMISSIBLE_COMMAND at Dwell 0.30 (each a LIMIT_JERK refusal of the re-aim), 2 REJECTED_NO_BALL, one ABORTED_NO_RELEASE (12:01:36), one COMPLETED 20/20 (12:03:51).
- **Launch 2 (12:50 to 13:03, steps 18 to 23, `gc_freeze:=true`).** FEAT-32 A/B on the oval at Dwell 0.40. A arm (flag off): MISSED_CATCH x4 then 32/32 at 12:53:51. B arm (flag on, 12:55 to 12:57): MISSED_CATCH 1/2 x5 and one REJECTED_NO_BALL; no carried Throw reached a landing (only pre-landing Fits). Columns canary 13:00 to 13:02: COMPLETED 20/20 twice, MISSED_CATCH x4. LANE_REFUSED 0.
- **Launch 3 (13:06 to 13:37, steps 25 to 38).** IDEA-16 human-Throw window 13:10:48 to 13:12:40. columns_turned 45 deg, 11 Attempts 13:14 to 13:19 (two COMPLETED, one ABORTED_FAULT at 13:15:23). columns_turned 90 deg, 5 Attempts 13:20 to 13:21 (LIMIT_JERK x3, NO_ADMISSIBLE_COMMAND x2). BUG-80 single-Throw block 13:24 to 13:36, 44 goals in two blocks, 2 MISSED_CATCH. LANE_REFUSED 0.
- **Launch 4 (13:38 to 14:00, steps 40 to 44).** BUG-82 hold, armed 13:41:06 to 13:59:59 (18.9 min), no goals.
- STOP records: 108 over the four launches. LANE_REFUSED ends: 2, both in launch 1.

## Findings

### LANE_REFUSED and the bridge drain (BUG-58, FEAT-46, BUG-89)

- Both launch 1 refusals were a STOP while the plan was moving. The FEAT-46 records say stamp gap with a +50.0 ms successor on both lanes (records #2/#3 and #8/#9), and the firmware refused the resume (`refused +6`, `+5`). Host starvation was refuted again: GC_PROBE n=0 on both stream nodes through both ends, `lost +0`, `seq_gaps +0`, `sent == received`, frame age under 25 ms.
- Mechanism, by elimination with counters: the emitter published every knot (`/trajectory/status` seq minus t/25 ms is flat, zero steps in 33.2 min); teensy_bridge_node's `_SetpointCommandSource.recv_latest` drained :5557 and forwarded only the newest of two waiting knots; the older one left no counter. `setpoints_sent` minus t/25 ms steps by -1 at isolated instants (44 clean steps in launch 1) beside forward gaps of 44 to 67 ms.
- The firmware view: each frame carries three knots, so frame P already covers the dropped span with zero shape error, and the ISR promotes only the newest due frame. Both ends were the surviving frame arriving 3.6 ms and 0.2 ms after frame P's 70 ms cover-plus-grace, inside the C2 stop, refused by the resume rule as designed. Normal arrival is 6 to 13 ms after the stamp and grace allows 20, so the margin is 7 to 14 ms.
- Event 1 was a bridge stall at a CATCH install under CPU contention (Hand capture RX lag 40 to 59 ms). Event 2 came from upstream and is unattributed. Launch 2 shows a host STOP at 13:01:35 (forward gap 76.1 ms, loop 50.9 ms, a 50 ms stamp gap) with no GC pause and no refusal; cause not found.
- GC: three gen-2 pauses of 61 to 65 ms in teensy_bridge_node in launch 1 (11:44:48, 12:00:57, 12:17:35, about 16.5 min apart). Every armed-time pause (four, 59.8 to 64.7 ms, across launches 1, 3 and 4; the 11:44:48 one was before arming) coincided with a host STOP (4 of 4) and none ended an Attempt. With `gc_freeze` (launch 2) there was none in 12.5 min, which is shorter than the recurrence, so it does not separate the freeze from chance.
- BUG-58 pass line met by launch 1, its hypothesis (host starvation or GC) refuted for both ends; the "host stream clean" verdict text was wrong for these ends and is replaced. FEAT-46 worked as built: 108 records, `overwritten 0`, the new-record count equals the sum of the STOP WARN increments in launches 2 to 4 (16, 52, 2); launch 1 had 38 records against 36 WARN increments (two records without a WARN, not chased).

### Hand stalls (facts only)

- Three stalls in the four Hand captures: 12:02:17.66 and 12:08:30.63 in launch 1, 13:15:22.69 in launch 3. Each: Hand diving from about 9.4 rev at -30 to -46 rev/s, stopped within about 25 ms, error pinned at exactly 2.00 rev (the firmware lead clamp), iq at about 70 A (-70.13, -67.8 and -70.93 A measured), stopped at 2.43, 2.10 and 2.61 rev against commanded bottoms of 0.42, 0.04 and 0.61 rev, so about 65 mm short each time.
- Launch 1 stalls freed themselves in 150 to 200 ms when the command rose. The launch 3 stall was the first one the firmware HAND JAM detector caught (13:15:22.83): the deviation reached -2.503 rev, the Teensy guard latched MAX_DEVIATION at 13:15:22.916 (Hand first; all six Legs within 0.01 rev), the Attempt ended ABORTED_FAULT at 2/4, and the Hand recovered by itself (HAND_JAM_RECOVERED 13:15:30, FAULT to IDLE 13:15:32, Active again 13:15:38). 16 s from fault to the next goal, no re-flash, no bridge reboot.
- Peak Hand iq per launch: 70.13, 44.3, 70.93 and 3.57 A. The Leg and Hand current caps were not changed.

### Trapped Balls under the Hand (BUG-84): all three stalls confirmed

- **Verdict:** all three Hand stalls were a Ball trapped under the Hand. The raw, unlabelled markers on `/mocap_data` (Platform pose from `/rigid_body_poses`; Platform frame = pose x, y, and z plus the 578.2 mm stow height, in which the labelled Platform marker is constant to 0.3 mm) show a marker that belongs to no rigid body sitting 65 to 85 mm below the Hand at each stall instant, static for the dwell. The owner saw the same marker trapped under the Hand in the GUI replay.
- **12:02:17.66 (launch 1).** Marker static at (22, 3, -38) mm for 0.13 s, present 12:02:17.58 to .86 within (23..26, 5..11, -36..-16). The Hand stopped at 2.43 rev with its own marker at z +32. It is Ball 295, the Ball the Hand was diving to Catch: it fell from the Cup region (z +35 at -0.3 s) down the tube (z -3 at -0.12 s, -35 at +0.01 s) into the Hand's path. The release came 0.16 s later.
- **12:08:30.63 (launch 1).** Marker at (52, 54, 8) from 12:08:30.13, then (29, 35, -8) and (23, 26, -20), 0.36 s each, present 12:08:30.60 to 31.00; Hand stopped at 2.04 rev. It traces to Ball 419 (the missed Ball) and then Ball 418. A marker moved across the Platform top and into the tube.
- **13:15:22.69 (launch 3).** Marker rested at the rim pocket (-47, 56, 4) from 13:15:21.81 to 22.52, rolled across the Platform top ((-23, 50, 4) to (17, 46, 3)) and settled at (15, 23, -20) from 13:15:22.679 to 23.483, so the dwell starts 15 ms before the stall line and lasts 0.8 s; Hand stopped at 2.61 to 2.71 rev. It is Ball 134 on the rim (the finder lists Ball 132 / 134). This is the stall the Teensy guard latched; the owner's reading at the latch, a ball stuck under the Hand that cleared by itself, is the data's reading.
- **Why the Tracker-only reading missed it.** `/balls` lists only Balls the Tracker confirmed, and the Tracker had dropped each of these Balls after it jostled on the Platform. The Ball ancestry therefore reaches back 2 to 4 s and 54 to 138 mm in the worst cases. All three markers sit at about the Cup-bottom x, y (15..26, 5..35) but 65 to 85 mm below the Cup seat, so it is not a Ball seated in the Cup.
- **A dwell alone does not predict a stall.** 21 dwells of at least 0.3 s in the four bags had no stall (11, 5, 5 and 0 per bag). Most are Balls resting in the rim pocket beside the tube at about (-40..-76, 45..70, 0..17), up to 13 s. Three of the 21 were classed UNDER_HAND without a stall (12:55:40.2, 0.41 s; 12:57:11.96, 1.77 s; 13:16:21.3, 0.37 s). A marker under the Hand at the stall instant is the signature: 3 of 3. Hypothesis only, for the FEAT-32 unit: the 12:56:50 and 12:57:13 soft stops (iq at most 24 A, the operator's "stepped" Hand) coincide with rim-pocket Balls within the arm opening; not examined.
- **Tool: `tools/trapped_ball_finder.py <bag> [--hand-csv <csv>] [--window T0 T1] [--out-dir D]`** (5 to 12 min per 30 min bag, writes `temp/reports/trapped_ball_<bag>.{txt,json}`). It converts unlabelled markers to the Platform frame, drops the Hand's own marker (z = -47 + 32.567 x rev, on the axis) and Balls seated in the Cup (z = -25 + 32.567 x rev, x, y = 8, 23), finds dwells (within 15 mm for 0.30 s; 0.08 s when judging a stall, which event 1 needed), classes them UNDER_HAND or ON_PLATFORM, traces ancestry to the closest `/balls` track, and flags a stall (|iq| at least 55 A for 20 ms, or command minus position over 1 rev for 50 ms while stopped) when an UNDER_HAND marker is present within 0.05 s. Two markers are calibrated: the Hand marker offset and the Cup seat. Thresholds were fitted on this one Sitting and the three stalls are also its calibration set, so they are not independent; re-check both markers after any Hand or marker re-mount. Limits: it needs the Hand marker tracked, and occlusion under the Cup can hide a trapped Ball, so "cannot see" stays a possible answer and is never "refuted". The only other flag, 13:15:27.5 (iq 2.2 A, no marker), is a post-fault error-rule hit, not a stall at the clamp.
- Not checked: Platform tilt at the stalls (the finder works in the Platform frame), whether the Hand physically touched the Ball (a 65 to 85 mm stop height is consistent with a Ball plus the Cup rim but unmeasured), the 12:56:16 soft stop, the occluded fraction under the Cup.

### carry_zvd stop-go (FEAT-32, BUG-90)

- The operator's "stepped" Hand at step 21 was real and is in the command. With the rows on, the pose QP gives the Hand a stop-go velocity of about 5 Hz in flight and a two-stage dive after the Catch. Hand capture: 4 turning points in the flight window (median over 15 windows) against 0 with the flag off (44 windows); sample command velocity per 50 ms, flag off -113 -103 -84 -54 -5 +153, flag on -166 -256 -194 +27 +185 +96. Echo spacing, position/velocity consistency and the firmware Hermite were identical in both arms, so it is not a hold or repeated knots.
- Offline, `test_cup_zvd._solve` reproduces it. Turning points off to on at Dwell 0.35, 0.40, 0.50: 0 to 5, 0 to 4, 0 to 2; Hand velocity range at 0.40 -88..194 to -376..751 mm/s. Hypothesis: under a tilted Platform, cup z moves the Platform laterally through the lever term, so a Hand wiggle is a cheap way to cancel the mode.
- Nothing approached a limit (Leg acceleration about 20 % higher, 42 % of limit; Leg jerk lower; no guard, LIMIT, RING, MAX_DEVIATION, SPINOUT or LANE_REFUSED line). The plan watched only the HAND peak gates.
- FEAT-32's own hypothesis (the rows remove the carried long bias): supported in direction. Platform mocap velocity excess at release shifted by -81 mm/s; the carried Throw's fitted landing minus command went from about +70 mm (A arm, 32/32 Attempt) to about -3 mm. Unverified against the pass line (no carried Throw reached a landing in the B arm, only pre-landing Fits; `platform_mode_fit` output not found).
- The five B-arm MISSED_CATCH ends came from the protocol, not the stop-go: the B arm shared the A arm's plant_id, whose Memory had learned to aim carried Throws about 68 mm short. With the bias gone the first carried Throw of every B Attempt was fitted about 71 mm inside the Site against 35 mm of inward lateral authority (AIM-LATERAL-CLAMPED).

### Turned Columns at 90 deg (IDEA-11, IDEA-22, BUG-91)

- Flown setting: Apex 0.90, tilt-back 3 deg. Nominate said OK at +0.027 (CATCH Ball 1 at P2@90) and printed -0.080 at the re-aim limit with a warning. The five Attempts all ended at skill 2, the first Catch after the Feed: margins +0.031, +0.032, +0.009 (then install refusals at 217k, 210k and 205k mm/s^3 against 200k, leg 2, +0.050 s into the re-gated range), -0.301 and -0.288 (NO_ADMISSIBLE_COMMAND). Best 2 Throws.
- One 25 ms knot of Ball 2 arrival time moved the CATCH Ball 1 margin from +0.031 to -0.301 at the same command, so the rehearsal margin sits inside the splice-state spread. The 0.95 setting was refused offline at -0.26 (step 27), as the sheet expected.
- The learner is not the root cause: Attempts 1 to 3 failed at the identity command. A warmed plant did push the command to (-35.6, +0.7) mm and Apex 0.827, which nominate on an empty plant never evaluates (the BUG-87 class). Mechanism for the install-time refusals (a junction transient at the splice) is a hypothesis.
- 45 deg: 11 Attempts, COMPLETED 20/20 at 13:18:59 and 19/19 at 13:17:28; 8 of the 9 MISSED_CATCH ends were Throws with no Tracker Fit. Scatter against the step 9 control (one Attempt, n 10 per ball): inside the 15 mm mean bar on all four ball and axis cells (one at 15.0 mm), outside the 10 mm sd bar on one (Ball 1 x, +13 mm).

### Oval long carry and fresh-plant K (FEAT-39, IDEA-18, IDEA-23)

- Carried minus command (mm): +34, +57, +111 at Dwell 0.50, 0.40, 0.35; from rest +28, +32, +27. The carry adds +7, +25 and +84 mm over from rest. Launch 2 confirms on a second plant: from rest +27 mm, carried +71 +/- 18 mm (31 rows) at 0.40; one pair at the identical command landed +23 versus +141 mm.
- Dwell ladder: 0.50 reached 32/32 after a LANE_REFUSED at 7/8; 0.40 reached 31/32 fed and 32/32 unfed; 0.35 reached 25/26; 0.30 ended 1/1, 1/1, 0/1 with LIMIT_JERK margins -0.127, -0.122, -0.120 (peak Leg jerk 365,412 mm/s^3 against the 200,000 limit). So 30 consecutive Catches was met only at Dwell 0.40.
- The b_x and s split is degenerate (every row at one release Site, rank 1, posterior correlation +0.97 to +0.99); only the sum at the query Site (+55 mm at row 110, +90 mm at row 152) is observable. The printed pair (+188 mm, +1.30 per m) is cosmetic. Latent hazard at another Site, not exercised.
- Fresh-plant K: 9 fresh starts measured a first-Throw Apex ratio of 1.077 +/- 0.015 (+51 to +92 mm), which the learner then fits (K 1.063 after one row, 1.078 after two). Filed as IDEA-23.

### Missed Catches without a Fit (BUG-78, BUG-33, BUG-63)

- columns_highlow: 15 non-complete ends, 9 on Ball 2 and 6 on Ball 1 (BUG-78's Ball 2 ratio did not repeat); 12 had a lost Throw with no Fit, so the lost Throw is exactly the untracked one; 3 were MISSED at 13 to 43 mm landing. No firmware STOP or late solve precedes any of them; every planned Catch was OK (minimum margin 0.083, caught). The same signature appears in all 9 oval MISSED_CATCH ends, so it is not two-ball specific.
- Columns canary on the trained plant (K 1.081, 4,714 rows): 4 of 6 Attempts ended on Ball 2 with fitted landings of mixed sign (+44, +25, -88 mm, one no Fit), 3 of 4 within 45 mm of aim with the Cup empty. The single-Throw block adds two empty-Cup ends at 20 mm (13:28:12, 13:33:10).
- ABORTED_NO_RELEASE at 12:01:36 (BUG-33) follows a MISSED Throw 8 at 13 mm: the lost-ball-at-the-Catch reading again. BUG-63 first Throw after the Feed: trained plant max 25 mm, fresh plant max 33 mm along the axis (n 6 each); no 180 mm case.

### BUG-80 landing offset response

- 44 goals in two blocks (x then y; one empty-Cup Feed Attempt excluded). Landing followed the offset: x +20 to +18.2 mm (n 12), -20 to -16.4 (n 9); y +20 to +19.4 (n 10), -20 to -25.6 (n 11); sd 8.5 to 10.8 mm. The shift is 34.6 mm (x) and 45.0 mm (y) for 40 mm commanded, about 3.4 sigma each, so the operator's "little visible difference" is a 10 mm sd against a 17 to 23 mm shift.
- Proxy slope of ball lateral velocity on mocap Platform tilt (mean over +0.2 to +0.5 s): x 4.64 +/- 0.09, y 4.66 +/- 0.11 mm/s per mrad, n 22 each, against the pass band 4.4 +/- 0.5 (5.7 is the small-Tilt plant gain). The commanded-Tilt slope was not computed.

### BUG-82 before/after

- Hypothesis tested: an even-ms encoder period phase-locks to the 2 ms setpoint tick and drops frames in runs; an odd period cannot. Metric: `age_max_us` per 1 s window in `/cache_diag`. Before (Legs at 10 ms, bags 2026-11-06 and 11-07): 245 of 27,046 Leg windows over 50 ms (299 over 25 ms), 0.91 %. After the all-axes 9 ms change (2026-11-08 13:01): 0 of 41,952 over 25 ms in six launches; every Leg peaks at 17.1 to 17.2 ms (one lost frame at 9 ms, a hypothesis).
- Same-launch control 2026-11-07 16:15: Leg 4 at 9 ms had 0 of 2,396 windows over 25 ms (peak 17.1 ms) while the other five Legs had 20 to 30. The Hand has 2 isolated windows over 25 ms in 6,992 (27.1 and 28.1 ms, under the 30 ms gate). The bridge was reflashed after the rate change (FW 42 then FW 43), so the 2026-11-07 control is the confound-free half.
- Launch 4: 1,128 hold windows, 17.1 ms on all seven axes. Frame rate 110.4 to 110.9 per s on all seven, so the Item's "Leg 4 and Hand at 111, others at 100" never held after the change. Not measured: frame-loss spacing, the Hand heartbeat dropouts, which side drops the frame.

### IDEA-16 bench report and markers (BUG-92)

- `tools/probes/human_throw_bench.py` is an offline replay of `/mocap_data`; I ran it on a 130 s slice of the launch 3 bag. 114 of 114 truth Throws were served by a tracker ball; landing within 50 mm and 30 ms at 0.30 s before the plane: 69 of 81 (41 of 71 on 2026-11-02). Landing sd at the Cup side 73 x 101 mm (2026-11-02: 129 x 90). Phantoms 30, 13.8 per minute (upper bound), 5.1 per minute near a truth release, against 27.3 and 12.5 on 2026-11-02. The lighting change was not recorded, and the two bench Sittings differ in more than lighting.
- The operator's observation is right and is the dominant phantom source: 12 of 124 live `/balls` human_throw tracks and 26 of the 30 bench phantoms start on static Ball Butler frame markers (`Ball Butler - 1..7` plus four unlabelled static markers). A static-cell exclusion (proposal, owner decision) removes all 26 and all 12 and 0 of 114 true Throws.

## Discussion

Four readings were withdrawn at this debrief, written here before the fixes so the fix choices can be judged against them.

1. **"Forward both knots would have saved the Attempts" (diag-knotskip) is WITHDRAWN.** The first read of the two LANE_REFUSED ends concluded that a bridge that forwarded P+25 as well as P+50 would most likely have survived. The owner's position was the opposite (newest-only is right), and the owner's physical intuition about the stream overturned the design before it was built. Reading `leg_interp.cpp` confirmed it: a frame carries u0..u2, so frame P already holds the P+25 to P+50 cubic with zero shape error, and the ISR promotes only the newest due frame. At emit lead 0 a frame's stamp is its emission instant, so P+25 would reach the firmware at the same moment as P+50 and could not arrive earlier than newest-only does. What killed both Attempts was lateness against a 7 to 14 ms margin (3.6 and 0.2 ms past the 70 ms deadline), not the missing knot. Forwarding the stale knot is mildly harmful (noise, queue fill, a possible 2 ms tick on the pre-splice plan). The one real caveat is emit lead 1 or more, where a pending frame may not yet be due; the general rule is "newest due plus every not-yet-due, oldest first", which equals newest-only at lead 0.
2. **"The learner chases its own command" (diag-oval, launch 1, Dwell 0.35) did not reproduce at Dwell 0.40.** On a fresh plant in launch 2 the command went -29 to -78 mm and the landing minus command stayed flat (+74.5 then +68.6, sd 18); the landing swung +35 to -43 mm and back, a regression slope of +1.49 (31 carried rows, r 0.68). That is the over-correcting loop of the flyback (BUG-80's name), not the half-realised positive feedback of launch 1. Withdrawn for Dwell 0.40, open at 0.35 (different plants, n confounded with time). It is not filed as a new Item: the over-correction is recorded on BUG-80.
3. **BUG-82 "unverified, void by its own rule" (verdicts2) is superseded.** The Item's in-run control (another Leg still at 10 ms) cannot exist now that all seven axes run 9 ms, so the unit read the pass as void. The before/after comparison across the pre-change bags and the 2026-11-07 same-launch Leg 4 control resolves it more strongly than the Item's design would have. The pass line is reworded to a before/after criterion.
4. **"Mocap refutes a Ball under the Cup at 12:08:30" (diag-misses2) is WITHDRAWN.** The unit read `/balls` (the Tracker lists only Balls it confirmed) and withdrew the Ball-under-Cup hypothesis for two of the three stalls. The owner saw a marker trapped under the Hand in the replay at 12:02:17 and 12:08:30 and a ball stuck under the Hand at 13:15:23, and the raw unlabelled markers on `/mocap_data` confirm it for all three (section "Trapped Balls under the Hand"). The Tracker had dropped each Ball after it jostled on the Platform, which is why the Tracker-only read missed it. Physical observation outranked the Tracker list, and a stall is no longer judged from `/balls` alone.

**Why newest-only stands, and why emit lead and grace were not changed.** The levers that add margin are timing constants: emit lead 0 to 1 (+25 ms of cover, margin about 32 to 39 ms, with the changed bridge rule and a re-derived `WIRE_READ_KNOTS`) and firmware grace 20 to 30 ms (alone it would have rescued both ends). Both are safety-bearing, the owner decided to fly a Sitting with the dropped-knot counter first, and to add a `cpu_partition` off/on A/B to that Sitting row. None is chosen; BUG-89 lists them with the SCHED_FIFO option (the 2026-10-28 solver-worker FIFO trial caused STOPs) and the pump step-gate budget (a guard change). A walked control cycle at lead 1 for event 1's stall promotes the P+75 frame at P+76 inside P+25's cover with 20 ms to spare.

**Why BUG-80 was merged ungated.** The Night Shift merge gate held it. The owner merged it on their say-so after the Sitting. The audit had two findings: one narrative (the logbook said 45 passed and "Held, not on main") and one behavioural, an offset applied on an empty-Cup Feed Attempt. The owner merged the branch into the debrief integration branch with both findings open (the merge was not gated). The follow-up commit in the same push fixed the one behavioural finding (a non-zero offset is refused on the Feed path) and found the other to be narrative (the Memory row stores the command sent, offset included, so it is self-consistent). The flown evidence (landing follows the offset; proxy slope inside the band) was taken on the unfixed code, on a path the fix does not touch (a from-rest Throw).

**Process findings.** A typed verdict that disagrees with the pass line stays the owner's, but the paperwork records what the data supports: FEAT-32's pass meant "row flown" (the owner's answer) while its literal pass line fails; BUG-82's pass is right but not for the reason the Item states.

## Fixes made this debrief

Each is a separate gated commit on the debrief branch. The gate (`./run_tests.sh`) result is in Verification.

1. **BUG-80 follow-up** (`skill_node.py`, `nominate.py`, `schedule.py` and two test files). A non-zero landing offset on an empty-Cup Feed Attempt is refused at goal accept, before anything is built or dispatched (`refused: landing offset (...) applies only to a from-rest self_toss ...`). The accept WARN now names the live plant_id and says rows go to it. No limit, margin, timing or trajectory change. (2026-11-13, `OMP_NUM_THREADS=1 python -m pytest -q tests/ros/test_skill_node_landing_offset.py tests/motion/test_skills_landing_offset.py`): 50 passed (4 new). (2026-11-13, 15 files importing skill_node / the accept path, `-n 4 --dist loadfile`): 867 passed in 276 s. (2026-11-13, `tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py`): 16 passed.
2. **Bridge drop counter** (`teensy_bridge_node.py`, `tests/ros/test_teensy_bridge_node_bridge_diag.py`). Observability only; forwarding and timing are unchanged. `recv_latest` keeps the discarded dicts; `_note_setpoint_drops` logs a throttled `setpoint drain dropped N older frame(s)` WARN with seq, stamp and lag; `/link_status` gains `setpoints_coalesced`; both STOP WARNs and the FEAT-46 record line end with the drop clause; the BUG-58 "host stream clean" verdict is replaced by `host forward cadence clean, no knot dropped at the drain, cover expired downstream or at a stamp` with the checked and not-checked parts named, or `host forward cadence below the cover BUT this node dropped a knot` when a drop is within 1 s. (2026-11-13, `python -m pytest tests/ros/test_teensy_bridge_node_bridge_diag.py -q`): 26 passed in 7.53 s (1 updated, 3 new). (2026-11-13, `python -m pytest tests/ros/test_teensy_bridge_node*.py tests/ros/test_install_segment.py -q -p no:cacheprovider`): 678 passed, 1 failed in 461.38 s; the failure, `test_a_fresh_throw_installs_through_the_real_worker_process` ("solver did not answer by the wire deadline"), is a wall-clock test that never imports teensy_bridge_node and failed alone (1 failed in 4.57 s) at load average 17.6 from parallel debrief units; not caused by this change.
3. **Sheet check, sitting-sheet instructions, CLAUDE.md zsh line.** The sheet's `source ros_ws/install/setup.bash` fails in zsh (no BASH_SOURCE, colcon resolves siblings against the cwd) and `tee ../temp/logs/...` lost the launch 3 and 4 logs to a relative path. `mapkit/sheet.py` gains `_check_shell` (called only for the author's check, `live=False`) failing `source|. ...setup.bash` and `tee <relative path>`; `.claude/commands/sitting-sheet.md` gains a bullet (zsh uses `setup.zsh`; tee to an absolute path); CLAUDE.md gets the matching sourcing line. The flown 2026-11-10 and 11-13 sheets are records and were not edited; `sheet check` on them now fails by design, and `test_committed_sheets_valid` calls only `validate_shape`. (2026-11-13, `python -m pytest tests/sim/test_mapkit*.py -q`): 248 passed in 30 s. (2026-11-13, `tests/sim/test_night_shift_gate.py tests/sim/test_no_legacy_item_ids.py`): 157 passed in 196.9 s.
4. **ZVD strict xfail** (`tests/motion/test_cup_zvd.py`). `test_the_zvd_rows_do_not_make_the_hand_stop_and_go_in_flight` (Dwell 0.40, strict xfail: turning points on <= off) and the passing control `test_the_flag_off_hand_velocity_is_monotone_through_the_flight` (all three Dwells). Planner unchanged. (2026-11-13, `python -m pytest tests/motion/test_cup_zvd.py -q`): 35 passed, 1 xfailed in 507.63 s.
5. **Trapped-Ball finder** (`tools/trapped_ball_finder.py`, `tests/sim/test_trapped_ball_finder.py`). Offline tool, no robot path. Tests are synthetic (no bag, no ports). (2026-11-13, `python -m pytest tests/sim/test_trapped_ball_finder.py -q`): 14 passed in 0.21 s. It is wired into `.claude/commands/sitting-debrief.md` section 1B (item 4) and `.claude/commands/diagnose.md` Step 4.

## Owner decisions open

- **Margin lever for BUG-89**: emit lead 0 to 1 with the due-aware bridge rule and a re-derived `WIRE_READ_KNOTS`; or `SCHED_GRACE_S` 20 to 30 ms (flash); or neither until the stalls are attributed. First fly the counter Sitting with the `cpu_partition` off/on A/B. Also: the pump step-gate budget `max(arrival elapsed, stamp delta)` (a coalesced 50 ms step is judged against 25 to 47 ms today; up to about 2x apparent velocity) and a `sent_epoch_us` field in the frame.
- **Hand stall policy**: the firmware jam guard latched MAX_DEVIATION on a stall that launch 1 shows clearing in 150 to 200 ms, costing the Attempt and 7.2 s of recovery (16 s to the next goal). Keep it, or allow one retry for a stall under 250 ms. No change to the 70 A cap or the 2.0 rev clamp from this evidence.
- **Nominate policy (BUG-91)**: refuse or flag in capitals a negative re-aim-limit margin. Turned 90 deg needs a margin of the order of +0.10 to +0.15 at the worst splice state; none of the offline candidates reaches it (tilt-back 1.9 deg at Apex 0.95 passed at 0.011, Apex 0.90 at 3 deg is +0.027). Options: Leg jerk limit 200,000 to about 230,000 mm/s^3 for this Pattern (safety-bearing, not recommended without Leg and driver data), tilt-back toward 0 with an offline re-measure (first choice, no hardware), or park 90 deg.
- **Identity prior K (IDEA-23)**: 1.000 now against 1.077 +/- 0.015 measured; a prior of about 1.07 takes the first Throw from +68 mm to about +10 mm. A plant-model prior change needs a dress rehearsal on a fresh plant.
- **Oval at Dwell 0.40**: the carried offset +71 mm needs a command near -70 mm and the loop over-corrects (slope +1.49). Proposed, for the owner to set: do not fly Dwell 0.30 again until the carried landing at 0.35 is within 30 mm of aim (the FEAT-39 note), which in practice waits on the flyback over-correction or the IDEA-18 split; ZVD flag-on Dwell 0.35 must not fly until BUG-90 is fixed. A command cap near 60 mm total, or fitting the offset per context (IDEA-18), are the candidates.
- **Static-cell exclusion for human_throw (BUG-92)**: cell 10 mm, occupied over 90 % of the last 30 to 120 s, first sample within about 40 mm.

## Process notes

- Standing debrief step: run `tools/trapped_ball_finder.py` on every bag and hand every Hand stall and every under-Hand dwell to the diagnosis units; never judge a Hand stall from the Tracker's ball tracks alone.
- Launch 3 and 4 tee logs were lost to the relative `../temp/logs` path; both were analysed from `~/.ros/log`. Fix 3 closes the class.
- Step 39 (the reflash and re-home check) was skipped as already done, so no launch 2 to 4 shows a `homing: 7 axes homed` line and the step 40 stop_if was not literally testable. All ODrive encoder rates changed days earlier (2026-11-08); nothing changed during the Sitting.
- The Columns canary was flown once at step 9 (the sheet asked for five), so the IDEA-11 control sd rests on n 10 per ball. Step 22 asked two canary Attempts each COMPLETED; the first two ended MISSED_CATCH. Step 36 flew 44 goals against a sheet count of about 40; step 33 flew 5 Attempts against 4. Neither affects a verdict.
- Not checked: the platform_mode_fit output (step 23), the BUG-80 slope from the commanded Tilt, whether the canary refusals are new against launch 1, the Platform pose at the Hand stalls, and BUG-82 frame-loss spacing.

## Follow-ups filed

- BUG-89: teensy_bridge_node forwards only the newest waiting knot (high; includes the Sitting row with the `cpu_partition` off/on A/B and the unexplained 13:01:35 STOP).
- BUG-90: carry_zvd rows make the Hand stop and go at about 5 Hz in flight.
- BUG-91: nominate accepts a setting whose own reply shows a negative margin with the Catch re-aim at its limit.
- BUG-92: static Ball Butler frame markers are tracked as human_throw.
- IDEA-23: seed a fresh plant K from the trained plant.
- The slope +1.49 landing-on-command finding went onto BUG-80, not a new Item. It is now also a progress note on FEAT-39.
- FEAT-50: log a WARN when an unlabelled marker dwells under the Hand (live version of the trapped-Ball finder; log only).
- BUG-84 (existing) carries the three confirmed stalls.

## Verification

- Full suite on the combined code (2026-11-13, `./run_tests.sh --full` in a detached worktree at 09b81d6a, the integration branch before the paperwork): parallel phase **9543 passed, 1 failed, 50 skipped, 1 xfailed in 645.39 s**; serial phase **10 passed in 38.34 s**. The one failure was `tests/sim/test_glossary_avoid_words.py::test_no_avoid_words_in_live_docs` on two uses of an avoid-word in `logbook/2026-11-09-landing-offset-parameter.md`, written by the BUG-80 follow-up commit; the wording was corrected in this commit. The xfail is the strict BUG-90 test. The load-sensitive `test_a_fresh_throw_installs_through_the_real_worker_process`, which failed in the bridge unit run at load average 17.6, passed here.
- Everything after that run is documentation, Item files, `.claude/commands` text, `docs/records.md` and `memory_v2.csv` (2026-11-13, `python -m pytest tests/sim/test_mapkit*.py tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_glossary_avoid_words.py tests/sim/test_docs_links.py tests/sim/test_logbook_front_matter.py -q -p no:cacheprovider`: **478 passed, 43 skipped in 38.05 s**; and the audit pass run of the same set plus `test_plans_index.py`, `test_sitting_debrief_protocol.py`, `test_night_shift_gate.py` and `test_no_legacy_item_ids.py`: **687 passed, 43 skipped in 200.82 s**). The full suite was not re-run on the final tree.
