---
id: BUG-123
kind: bug
title: 'A guard latch leaves the Hand pushing at full current: no stall guard on the streamed lane or a latched hold'
status: in_progress
created: '2026-11-20'
updated: '2026-11-20T07:07:50Z'
owners: []
priority: high
effort: m
hardware: robot
owner: hands
radius: subsystem
diagnosis: confirmed
area: can-bridge
blocked_by: []
links:
- logbook/2026-11-20-sitting-debrief.md
- bug-odrive-gui-save-resets-the-hand-zero-the-bridge-stays-homed
- bug-hand-stalls-at-the-70-a-clamp-mid-dive-probably-on-a-ball
filed_by: claude
---

Sitting of 2026-11-20 (bag `2026-11-20_12-49-25`, 13:34:22). The Hand stalled against its top stop on a lost home (see the linked Item). MAX_DEVIATION latched 80 ms into the stall, but a latch only freezes the output: the drive kept its last lead-clamped `input_pos` (measured 5.86 + MAX_LEAD_HAND_REV 2.0 = 7.85 rev) and pushed 68-70.4 A for 4.4 s (about 21,800 A^2 s) until the E-stop button cut power.

Thermistor 33.5 C at the start, 49.6 C at the cut, peak 88.3 C 3.4 s later; the Hand has slid roughly since.

## Why nothing stopped it

What exists and why each was silent: the FW 36 stall guard (25 A, 0.5 rev/s, 300 ms) covers only the ACTIVATE / HAND_MOVE_TO ladder; the host `hand_jam` detector excludes the top stop by design (descending only); the drive's thermistor limits (60 C derate, 70 C error) lag by at least 3 s.

## Owner decisions

Owner decisions, with the measured numbers:
1. A stall guard on the streamed lane and on a latched hold. Seen this Sitting: the real stall 68-70 A at under 0.3 rev/s for 4.4 s; mis-homed Feed Catch touches up to 38.7 A in bursts under 0.1 s (0.1-0.2 s by a second read); a trapped-Ball pinch 72.3 A for 0.09 s; Throw and brake peaks 28-48 A while moving; no other run of 40 A or more below 1 rev/s longer than 0.15 s. Options:
   - (a) the ladder's 25 A / 0.5 rev/s / 300 ms (would have fired at 13:34:22.76, about 6 % of the event's I2t); (b) 40 A / 1 rev/s / 150 ms; (c) at 0.9 x 70 A for 100 ms. The action is a separate choice: Idle, relief to a low current, or move the setpoint to the measured position.
2. Whether a guard latch should leave the Hand energised at its lead-clamped setpoint at all.
3. Thermal: an I2t budget (this event: about 2.5 mC per A^2 s), lower drive thermistor limits (cannot act within 4 s at 70 A), or rely on (1).
4. Observability: the launch log prints the Hand motor temperature only when the fan changes state (`teensy_bridge_node.py:7874-7878`), so it showed 34-35 C all Sitting; the 88 C is only in the bag.

The owner intends to keep running this motor to establish the cut-offs.

## Sitting

- setup: bridge-cycled
- why: the guard must be seen to put a pushing Hand into Idle outside the ladder, with its latch, log line and recovery, without holding the damaged motor at 70 A.
- do:
  - Check out the branch, rebuild both packages, flash the can-bridge with FW 47 and power-cycle the bridge; check `BRIDGE_FW_CHECK` passes at 47.
  - Launch and home; do NOT arm. Lower the Hand drive's current limit to 45 A for this test.
  - Move the Hand to mid-stroke with a Hand move, wait for it to finish, then lower it onto its bottom stop.
  - Command a Hand position about 0.5 rev below the bottom stop and watch the Hand current and the log.
  - Clear errors, re-home, and put the Hand current limit back to 70 A.
- pass: the Hand goes Idle within 200 ms of its current first reaching 40 A; the launch log shows the HAND_STALL line and `fault_state` HAND_STALL; ACTIVATE is refused until errors are cleared; no second current rise.
- record: the time from the first 40 A sample to Idle, the peak current, the thermistor before and after, the log line, the bag path.
- minutes: 15
- branch: night-shift/2026-11-20-BUG-123

## Progress

- 2026-11-20: Owner decision, 2026-11-20: 40 A / 1 rev/s / 150 ms, then the Hand enters Idle. Being built as can-bridge FW 47 on a held branch.
- 2026-11-20: Built and held on night-shift/2026-11-20-BUG-123 (d75d42a7): can-bridge FW 47, PROTOCOL unchanged. Outside the ladder, 40 A or more at 1 rev/s or less for 150 ms of real samples sends the Hand Idle and latches a new fault HAND_STALL (INVARIANTS I-FW-17); the Legs are held as for a Hand drive fault; only the operator's clear releases it.
- 2026-11-20: FW 47 branch verification: native tests 41 of 41 cases, compile OK; not flashed, no full gate.
- 2026-11-20: Open on the branch: the Sitting proof pushes this motor at about 45 A for under 0.5 s (the bench Hand is the zero-risk alternative); the exact commands for the current limit and the raw position move are fixed when the sheet is written. Closest margin in the 2026-11-20 data: a latched hold at 36.2 A. A Hand-only CAN silence is not covered by any guard.
