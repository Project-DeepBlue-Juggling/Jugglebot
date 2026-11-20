---
id: BUG-123
kind: bug
title: 'A guard latch leaves the Hand pushing at full current: no stall guard on the streamed lane or a latched hold'
status: in_progress
created: '2026-11-20'
updated: '2026-11-20T05:22:21Z'
owners: []
priority: high
effort: m
hardware: none
owner: decision
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

## Progress

- 2026-11-20: Owner decision, 2026-11-20: 40 A / 1 rev/s / 150 ms, then the Hand enters Idle. Being built as can-bridge FW 47 on a held branch.
