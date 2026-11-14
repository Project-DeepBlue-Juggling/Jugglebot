---
id: BUG-93
kind: bug
title: 'can-bridge fatal CAN-silence check wraps when a frame is stamped after its clock read: a live Leg reads silent and the Platform Stows'
status: in_progress
created: '2026-11-14'
updated: '2026-11-14T20:05:42Z'
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
links: []
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
- why: The held branch is can-bridge FW 44: the CAN-silence predicates no longer wrap when a Leg is stamped after the fault machine reads its clock; it must be flashed and flown before it merges.
- do:
  - Flash the can-bridge with FW 44 from the branch and power-cycle the bridge.
  - Launch and check the start-up log shows `BRIDGE_FW_CHECK` passing at version 44.
  - Fly your usual Columns Attempts for the rest of the Sitting.
- pass: BRIDGE_FW_CHECK passes at 44 and no CAN_BUS_DOWN trip occurs; any CAN_BUS_DOWN that does occur names a real Leg with a real age, never Leg 0xFF with age 0.
- record: the FW version the bridge reports, every CAN_BUS_DOWN or Stow with its latched Leg and age, the Attempts flown.
- minutes: 15
- branch: night-shift/2026-11-15-BUG-93

## Progress

- 2026-11-15: Night Shift 2026-11-15: held on night-shift/2026-11-15-BUG-93: firmware is gate-frozen and unflashed (FW 44); suite failed only the 4 Sunday runner tests (BUG-107); 3 same-class findings open; unblocked by a flash and a Sitting
