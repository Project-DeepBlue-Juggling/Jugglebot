---
title: "Robot Sitting 2026-11-20 debrief: the Hand lost its home at a MISSING_ESTIMATE, stayed flagged homed and pushed 70 A into its top stop for 4.4 s (88 C); polygon walk and BUG-110 passed, can-bridge FW 46 merged"
type: investigation
date: 2026-11-20
status: complete
phase: "Sitting debrief"
related_plan: "pattern-progression.md"
related_issues: []
files_changed:
  - logbook/2026-11-20-sitting-debrief.md
  - logbook/INDEX.md
  - docs/records.md
  - temp/learn/jugglebot/memory_v2.csv
  - map/sheets/2026-11-20-robot.results.yaml
  - plans/active/pattern-progression.md
  - docs/project-map.md
  - docs/runbook.md
subsystem: [hand, can-bridge, skill-stack, tracking, tooling]
tags: [sitting, hardware, diagnostics, debrief]
---

# Robot Sitting 2026-11-20 debrief

Sheet `2026-11-20-robot`, 26 steps, two launches. Launch 1 flew `main` (can-bridge FW 45, Platform Teensy FW 11, both for the first time); launch 2 flew the candidate `night-shift/sitting-2026-11-20` (main plus BUG-93, can-bridge FW 46). Logs `temp/logs/launch_2026-11-20_1.log` and `_2.log`; bags `~/Desktop/rosbags/2026-11-20_12-49-25` and `2026-11-20_13-48-32`.

## Summary

- The Hand motor was overheated and now slides roughly. At 13:32:16 the Hand drive raised MISSING_ESTIMATE at rest and its position came back exactly 5.00 turns lower. The robot kept `is_homed` true, so `clear_errors` returned to Idle with no Homing, and the next Feed Catch (13:34:22) drove the Hand into its top stop: 68-70 A for 4.4 s, thermistor peak 88.3 C. MAX_DEVIATION latched 80 ms in, but a latch only freezes the output and the drive kept pushing; the E-stop button ended it.
- Passed and closed: BUG-110 (no negative forwarded lag in 72 drop lines), IDEA-9 (polygon walk 25/25, plan step P7), BUG-93 (FW 46 flown, merged by hand).
- Failed: BUG-89 (the dropped-a-knot verdict is missing on 4 of 6 stamp-gap STOPs), FEAT-49 (the capture tool crashed at the real fault and saved nothing).
- Unresolved or data: BUG-111 (a SPINOUT, not captured), BUG-58 (host GC pauses precede 2 to 3 of 5 lane stops), IDEA-16 (single-lob replay report).
- Platform Teensy FW 11: tilt read (+0.0107, +0.0084) rad at Level, gravity offset (+0.0003, +0.0049) rad against (-0.0024, +0.0053) on 2026-11-18; the Hand fan worked on command and in auto.

## Big picture

What went well. The polygon walk caught 25 of 25 on its first full Attempt from an empty Memory, so a Pattern whose Site changes every Throw works and the Learner copes with first visits. The emitter fix for early knots held: none of 72 dropped-frame lines shows the old signature. Three new firmwares flew without a fault of their own (can-bridge 45 and 46, Platform Teensy 11), and firmware 46 is merged. The single-lob recording gave the human-partner work its first usable data: of 23 lobs inside the Cup's reach, 16 had an admitted plan, about 30 mm from the real crossing.

What did not. The Hand motor was overheated and damaged. A drive fault lost the Hand's position reference, the robot still believed it was homed, and no guard stops a Hand that is stalled on the streamed lane. Columns completed only 5 of 18, about the same as two days earlier. The stream still stops: four oval Attempts and one polygon walk ended on a late setpoint stream. The fault capture crashed at the one real fault.

What it means. The Hand's protection had two holes that are now Items with fixes in hand (BUG-122, BUG-123); until both are flown, re-home after any Hand fault or E-stop button press. Stream stops now have a named host cause in most cases: garbage-collection pauses of 67 to 94 ms in the two stream nodes, so the dropped-knot theory (BUG-89) is a symptom, not the cause. Columns reliability is not a firmware or Hand problem; it is lost Balls (bounce-outs, outward Throws, Feeds landing outward), and the ABORTED_NO_RELEASE label hides them.

Next steps. Hand safety: fly BUG-122's branch and the firmware 47 stall guard (40 A, 1 rev/s, 150 ms, then Idle) first at the next Sitting. Pattern progression (PWAY-8): P7 is met; rotating Columns (P8) is next. Human partner (IDEA-16): work the BUSY refusals, then an armed single-lob Catch. Stream: remove the gen-2 pauses from the stream nodes (BUG-58). Columns: trial the 15 mm outward Catch authority (BUG-78) and decide the end-code relabel (BUG-33). Capture: keep the fixed tool armed (FEAT-49, BUG-111).

## Inventory

| Launch | Pattern | Attempts | Ends |
| --- | --- | --- | --- |
| 1 | Columns (Sep. 125, Apex 0.95, Dwell 0.27) | 11 | 2 done, 8 MISSED_CATCH, 1 ABORTED_NO_RELEASE |
| 1 | oval_1ball_fed (plant `bug110-oval-2026-11-20`) | 12 | 5 done, 4 LANE_REFUSED, 1 MISSED_CATCH, 1 REJECTED_NO_BALL, 1 ABORTED_FAULT (SPINOUT) |
| 1 | polygon_walk (plant `polygon-2026-11-20`) | 3 | 1 done 25/25, 1 ABORTED_FAULT at the start (the stall), 1 LANE_REFUSED at 29/29 of 50 |
| 2 | Columns on FW 46 | 8 | 3 done, 1 MISSED_CATCH, 3 ABORTED_NO_RELEASE, 1 LANE_REFUSED |

## Findings

### The Hand stall (BUG-122, BUG-123, BUG-125)

- 13:22:42 SPINOUT_DETECTED during the post-release brake of a normal oval Throw (69/70). The disarmed Hand coasted into the top stop at about 49 rev/s. The position stayed continuous through the clear (9.510 rev before, 9.508 after).
- 13:32:16.53 MISSING_ESTIMATE with the Hand at rest at 5.40 rev: NaN for about 50 ms, then 0.40 rev. `is_homed` stayed 1 in every `/robot_state` sample. FAULT, BOOT, Idle at 13:32:28 with no Homing (`state_machine.py:347-349`).
- 13:34:22.458 to 13:34:26.89: the Hand stopped at 5.85-5.89 rev on the shifted encoder (about 10.86 rev true if the 5.00-turn offset holds, which is the top stop; an inference that fits what the operator heard) at 61.7 then 68-70.4 A. MAX_DEVIATION latched at 13:34:22.539 (+2.825 rev); the drive held its lead-clamped `input_pos` 7.85 rev. The E-stop button cut power at 13:34:26.9. Thermistor: 33.5 C at the start, 49.6 C at the cut, 88.3 C at 13:34:30.
- After that power cycle of all seven drives `is_homed` was still 1 and BOOT went to Idle at 13:37:48; the operator homed by hand.
- Why nothing stopped it: the FW 36 stall guard covers only the ACTIVATE / HAND_MOVE_TO ladder; the host `hand_jam` detector excludes the top stop by design; the drive's 60/70 C thermistor limits lag by at least 3 s.
- The launch log's `hand fan: ... motor=` lines read 34-35 C all Sitting because they print only on a fan state change.

### First Homing a revolution high (BUG-124)

The 12:49:37 Homing tripped about 1.0 rev above the bottom stop (3.0 rev of travel, peak 9.1 A). Columns Attempts 1-6 pressed the top stop at 9.78-9.80 rev against a 9.94-10.30 rev command at each Feed Catch (24-39 A, under 0.2 s). The 13:07:19 re-home fixed it. Landing medians (18-41 mm before, 25-41 mm after) and completions (1 of 6, 1 of 4) did not shift. Cause of the early trip: unknown.

### Firmware lane stops and the multi-rev Hand MAX_DEVIATION latches (BUG-58, BUG-89, BUG-110)

Five LANE_REFUSED ends in launch 1 and one in launch 2. The Hand latches at -3.968, +6.717, +6.968 and -4.630 rev each follow a firmware STOP by 0.01-0.3 s: the STOP holds the Hand and the plan walks away from it (Hand current at most 36 A, no stall). The ends at 13:17:21 and 13:28:57 follow gen-2 GC pauses of 75.6 ms (bridge) and 93.8 ms (trajectory_node); the polygon_walk end at 13:41:01 (STOP 13:40:58) follows a 67.0 ms bridge pause. All 72 `setpoint drain dropped` lines have a positive forwarded lag (+2.3 to +27.4 ms). `setpoints_coalesced` resets at each arming.

### Columns (BUG-33, BUG-74, BUG-78, BUG-84, BUG-114)

5 of 18 completed, against 9 of 26 on 2026-11-18 (Fisher p about 0.7): no demonstrable regression on FW 45/46, Platform FW 11 or last night's merges. In launch 1 the Feed Ball was the first loss in 3 of 10 Attempts (two Feeds 24 mm outward of typical) and a robot Throw in 4. All four ABORTED_NO_RELEASE ends show a normal Hand stroke with an empty Cup; one logged no lost flight at all. Launch 2's Hand tracking, stroke peaks and release timing sit inside launch 1's range, so the damaged Hand is not shown to cause its three ABORTED_NO_RELEASE ends. One new trapped-Ball stall: 13:52:36.69, 72.3 A for 0.09 s at 2.08 rev.

### Polygon walk (IDEA-9, P7)

25/25 from an empty Memory, then 29/29 of a 50-Throw Attempt (the log's landed values: median 16 mm, worst 43 and 38 mm; the Memory rows give a worst of 52.4 and 43.1 mm). Landed square: sides 101.9, 92.1, 95.6, 97.9 mm then 107.9, 107.5, 98.9, 96.0 mm for 100 commanded; corner angles 78-100 deg; per-axis scatter 12-26 mm with 6-7 landings per Site, so the shape error is of the order of the noise. Plots: `temp/probes/polygon_2026-11-20/`.

### Single lobs (IDEA-16)

55 lobs detected, 49 crossing the Catch plane, 23 inside the Cup's reach, 16 of those with an admitted plan (5 BUSY, 2 NO_EVENT); planned Cup centre to real crossing median 30.5 mm, worst 52.0 mm. Most Balls were caught by hand near the Cup, which the replay does not model.

## Discussion

- **Withdrawn: "the SPINOUT reset the Hand's home."** That was the reading at the robot and this debrief's first framing. The bag shows the position continuous through the SPINOUT and its clear, and 57 + 3 + 72 Throws caught afterwards. The reference was lost ten minutes later at the MISSING_ESTIMATE, by an integer five turns. Whether the SPINOUT's top-stop impact caused that fault is unknown (BUG-125).
- **Reframed: the multi-rev Hand MAX_DEVIATION latches are not a mis-referenced Hand.** Four of the five are the guard working after a stream STOP; only 13:34:22 is the stall.
- **The class.** The position reference became invalid while the robot still reported homed, and no guard on the streamed lane or on a latched hold watches current against motion. BUG-56 (2026-11-02) was the same class; its fix covered only the cold-start ladder. Two parts: BUG-122 (invalidate the homed flag; a bug, no threshold) and BUG-123 (a stall guard and what a latch does with the Hand; thresholds and a guard action, so owner decisions).
- **BUG-58 and IDEA-16 derived as data although their pass text is met literally.** BUG-58's criterion is about evidence being on record, and the starvation is not fixed (the owner's 2026-11-18 ruling); IDEA-16's is a data row of an open Idea.
- **BUG-93 merged outside the script.** `tools/sitting_merge.sh prepare` refuses the branch with exit 4 (frozen firmware paths, gate condition (f)); it was merged by hand at the owner's confirmation, with the full suite on the merged tip.

## Fixes made this debrief

- `tools/probes/hand_foc_scope.py` (on main, 658fd7c0, `logbook/2026-11-20-hand-foc-scope-keeps-the-forced-window.md`): a force-stopped scope's window is saved and flagged, and one failed capture no longer ends a long arm.
- BUG-122, held on `night-shift/2026-11-20-BUG-122` (not merged): the bridge clears the homed flag when a Leg or the Hand reports MISSING_ESTIMATE or INITIALIZING. Held because it also edits the fault-exit stow condition in `state_machine.py` and step 6 of `ARMING_CONTRACT.md`, and the recovery path is unflown. Scoped run 2026-11-20, `pytest -q -n 4 --dist loadfile tests/ros/test_teensy_bridge_node_*.py tests/ros/test_state_machine.py tests/ros/test_state_machine_deactivate_confirm.py tests/ros/test_orchestrator_node.py tests/ros/test_orchestrator_conduit_contract.py tests/ros/test_orchestrator_conduit_integration.py tests/ros/test_hand_capture.py`: 960 passed. No full gate or audit yet.
- BUG-93 merged to main (0bd90eb7).

## Owner decisions open

1. BUG-123: DECIDED 2026-11-20 (owner): 40 A / 1 rev/s / 150 ms, then the Hand goes Idle; being built as can-bridge FW 47 on a held branch.
2. BUG-124: DECIDED 2026-11-20 (owner): no fix for now; the record is kept in case a false Homing happens again.
3. BUG-33: relabel ABORTED_NO_RELEASE with an empty Cup as the lost flight (changes an end code INVARIANTS lists).
4. BUG-78: DECIDED 2026-11-20 (owner): trial 15 mm at a Sitting, no further (late lateral re-plans are hard on the Leg base joints); the trial is the Item's `## Sitting` section.
5. Carried: the BUG-89 emit lead or due-aware bridge rule, the BUG-111 options, the human_catch holding timeout.

## Follow-ups filed

BUG-122 (homed flag survives reference loss), BUG-123 (latch leaves the Hand pushing; no streamed-lane stall guard), BUG-124 (Homing a revolution high), BUG-125 (MISSING_ESTIMATE at rest), BUG-126 (`human_catch_replay.py` time window).

## Verification

- Records and Memory commit, 2026-11-20: `pytest -q tests/sim/test_runtime_records.py tests/motion/test_records.py tests/motion/test_memory.py tests/motion/test_learner_replay.py tests/ros/test_gui_records_api.py tests/ros/test_runtime_writes_untracked.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_docs_links.py`: 220 passed.
- BUG-93 merged tip 0bd90eb7, 2026-11-20: `flock /tmp/jugglebot-tests.lock ./run_tests.sh --full -q`: exit 0 (parallel 775 s rc=0, serial 43 s rc=0; the quiet log prints no pass count).
- Capture-tool fix on the rebased tip, 2026-11-20: `python -m pytest -q tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_docs_links.py tests/motion/test_hand_foc_scope.py`: 135 passed.
- Debrief tree, 2026-11-20: `flock /tmp/jugglebot-tests.lock ./run_tests.sh`: 1 failed, 10635 passed, 54 skipped, 1 xfailed (parallel), 7 passed (serial). The failure was `tests/sim/test_map_items.py::test_item_bodies_are_readable` on this debrief's own over-long Item notes; after restructuring them, `python -m pytest -q -n 4 --dist loadfile tests/sim/test_mapkit*.py tests/sim/test_sitting_sections.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_docs_links.py tests/sim/test_plans_index.py tests/sim/test_map_items.py`: 360 passed. The gate was not re-run in full after that map-only edit.
