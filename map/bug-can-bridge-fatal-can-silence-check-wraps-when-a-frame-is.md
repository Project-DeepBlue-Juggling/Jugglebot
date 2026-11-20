---
id: BUG-93
kind: bug
title: 'can-bridge fatal CAN-silence check wraps when a frame is stamped after its clock read: a live Leg reads silent and the Platform Stows'
status: complete
created: '2026-11-14'
updated: '2026-11-20T04:31:31Z'
owners:
- night-shift
priority: high
effort: xs
hardware: robot
owner: none
radius: subsystem
diagnosis: confirmed
area: firmware
blocked_by: []
links:
- logbook/2026-11-20-sitting-debrief.md
- logbook/2026-11-15-canbridge-silence-predicate-wrap.md
filed_by: claude
---

The can-bridge fault machine can read a live Leg as silent for one evaluation, set the fatal CAN error and Stow the Platform in the middle of an Attempt. Found by the 2026-11-13 maintainer review; the expression was read and confirmed, how often it fires is not known.

## Findings

- `fault_machine.cpp:186` (`any_present_leg_silent`) and `:219` (`all_present_legs_alive`) compute `now - atomic_read_u64(&axes[i].last_rx_us) > CAN_AXIS_SILENCE_TIMEOUT_US` with no ordering guard.
- `last_rx_us` is stamped by `task_can_rx` (priority 5), which can preempt the fault machine (priority 3) after `now` was captured. The stamp is then newer than `now`, the unsigned subtraction wraps to about 2^64 and the 2 s threshold is exceeded.
- The same file already carries the cure at `:573` (the MOTOR_FB_STALE loop): `now > ts && (now - ts > ...)`, with a comment that describes this exact hazard. It was never applied to the fatal predicate.
- `first_silent_leg` (`:198`) and `fault_hb_stale_mask` (`:722`) have the same subtraction; those only affect reporting.

## Signature to look for

A CAN_BUS_DOWN trip whose latched record names Leg 0xFF with age 0: the rescan at the trip instant finds nothing silent.

## Wanted

- Apply the `now > ts` guard to all four sites, ideally through one helper so the class is closed at one enforcement point.
- A native test in `tests/firmware/native/` that stamps `last_rx_us` after `now` and asserts no trip.
- A firmware version bump and a flash; that is why this was not fixed in the review itself.

## Sitting

- setup: bridge-cycled, level, armed
- why: The held branch is can-bridge FW 46 (main is FW 45, the Platform IMU relay of IDEA-20; renumbered 2026-11-20): the CAN-silence predicates no longer wrap when a Leg is stamped after the fault machine reads its clock; it must be flashed and flown before it merges.
- do:
  - Flash the can-bridge with FW 46 from the branch and power-cycle the bridge.
  - Launch on the branch and check the start-up log shows `BRIDGE_FW_CHECK` passing at version 46.
  - Fly your usual Columns Attempts for the rest of the Sitting.
- pass: BRIDGE_FW_CHECK passes at 46 and no CAN_BUS_DOWN trip occurs; any CAN_BUS_DOWN that does occur names a real Leg with a real age, never Leg 0xFF with age 0.
- record: the FW version the bridge reports, every CAN_BUS_DOWN or Stow with its latched Leg and age, the Attempts flown.
- minutes: 15
- branch: night-shift/2026-11-15-BUG-93

## Sitting result (2026-11-20 15:11)

- verdict: pass
- notes (split into bullets):
  - This is what I get: " (venv) jetson:/home/jetson/Desktop/Jugglebot/ros_ws/src/jugglebot/Teensy_code_canbridge/ (night-shift/sitting-2026-11-20) $ grep -nE "CAN_BUS_DOWN|BRIDGE_FW_CHECK|SPINOUT|LANE_REFUSED" ~/Desktop/Jugglebot/temp/logs/launch_2026-11-20_2.log 28:13:48:36.575 teensy BRIDGE_FW_CHECK: OK — can-bridge v46 [install_skew=0] 1343:13:55:42.630 skill END LANE_REFUSED:
  - hold installed — the firmware is holding a REFUSED lane (legs or hand) 1346:13:55:42.692 skill lane recovery (END LANE_REFUSED): recover_lane: nothing to do — not latched (flags bit 15 clear) 1356:13:55:44.592 skill ERROR columns ENDED (LANE_REFUSED):
  - the firmware refused a streamed lane (legs or hand) and is holding it · 3/7 caught · apex 0.74–0.96 m · worst landing 51 mm · landing median 51 mm over 4 (3 without a Fit) · release -3 ms..-3 ms · entered by a Feed " (sheet 2026-11-20-robot, step 24) [derived: temp/logs/launch_2026-11-20_2.log:28 BRIDGE_FW_CHECK OK can-bridge v46; no CAN_BUS_DOWN, CAN-silence or Leg-age line in the launch;
  - 8 Columns Attempts flown, downlink lost 0, crc_errors 0. Absence over about 9 minutes of streaming; the launch's one fault is a Hand MAX_DEVIATION after a stream STOP (line 1338), unrelated to the changed predicates.]

## Progress

- 2026-11-15: Night Shift 2026-11-15: held on night-shift/2026-11-15-BUG-93: firmware is gate-frozen and unflashed (FW 44); suite failed only the 4 Sunday runner tests (BUG-107); 3 same-class findings open; unblocked by a flash and a Sitting
- 2026-11-18: 2026-11-18: main is now can-bridge FW 44 (per-group scheduler grace). The held branch of this Item also calls its firmware FW 44 and must renumber to FW 45 (version constant, history line, EXPECTED_BRIDGE_FW_VERSION, xref tests) before it lands.
- 2026-11-18: Held branch merged up to main and renumbered to FW 45 (version constant, history line, EXPECTED_BRIDGE_FW_VERSION, cross-reference test, native test names); no predicate change. Unblocked by a flash and a Sitting. A bridge on FW 45 fails BRIDGE_FW_CHECK on main until this branch lands, so the Sitting ends with the merge or a flash back to FW 44.
- 2026-11-19: 2026-11-19 owner decision: can-bridge FW 45 goes to the Platform IMU relay (IDEA-20); this Item's held branch must renumber to FW 46 (version constant, history line, EXPECTED_BRIDGE_FW_VERSION, cross-reference test, native test names) before it lands or is flashed.
- 2026-11-20: Night Shift 2026-11-20: held branch renumbered to can-bridge FW 46 (FW 45 is the Platform IMU relay); Sitting section says 46; unblocked by a flash and a Sitting
- 2026-11-20: Passed at the Sitting of 2026-11-20: BRIDGE_FW_CHECK OK at v46, no CAN_BUS_DOWN over 8 Columns Attempts; merged by hand (frozen firmware paths) with the full suite green on the merged tip. The bridge was left flashed with FW 46.
