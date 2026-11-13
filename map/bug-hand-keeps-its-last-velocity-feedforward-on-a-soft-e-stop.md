---
id: BUG-105
kind: bug
title: Hand keeps its last velocity feedforward on a Soft E-stop, disarm or link loss and stays in Closed-loop
status: inbox
created: '2026-11-14'
updated: '2026-11-13T23:10:31Z'
owners: []
priority: high
diagnosis: hypothesis
area: hand
blocked_by: []
links:
- bug-motor-fb-stale-suppression-leaves-the-odrive-holding-its
- bug-can-bridge-fatal-can-silence-check-wraps-when-a-frame-is
filed_by: claude
---

On every software stop except MOTOR_FB_STALE, the last frame the Hand ODrive holds keeps its velocity feedforward, and the Hand stays in Closed-loop. Found by the 2026-11-13 maintainer review and checked on 2026-11-14 against the owner's objection that a power cut clears the terms. Related to BUG-19, whose cure (the FW 35 hold flush) was scoped to MOTOR_FB_STALE.

## Findings

- `leg_interp.cpp:1903-1918`: the vel_ff is zeroed only when `hold_flush` is set, and `s_hold_flush_ok` is set for MOTOR_FB_STALE alone (`:151-166`).
- `leg_interp.cpp:1920-1936`: the torque drain then re-sends the last frame with `hand_emit_vel = s_hand_tx_vel` and zero torque, so the stale vel_ff is actively refreshed.
- On a Soft E-stop, a disarm or a link loss nothing commands the Hand to Idle (`fault_machine.cpp:602-660`; `docs/can_bridge/safety.md:42-44` says to let the ODrive hold). The Hand's own CAN Watchdog is off.
- BUG-19's bench result on a Leg: the axis settles at `input_pos + vel_ff / pos_gain` (0.05 rev at vel_ff 2.0, gain 40). It was never measured on the Hand.

## Size on the Hand (arithmetic, not measured)

| Stroke speed at the stop | Offset at gain 35 |
| --- | --- |
| 150 rev/s | about 4.3 rev |
| 300 rev/s (the vel_ff cap) | about 8.6 rev |

The Hand deviation Guard is 2.5 rev, but it only latches the Soft E-stop, which is the state already in force. The stroke is about 10.5 rev.

## The owner's objection

A power cut does clear the terms: they are volatile, and the axis comes back Idle. That covers the E-stop button if it cuts ODrive power; no wiring document in the repo says what it cuts. It does not cover a Soft E-stop, a disarm or a link loss, where the axis never leaves Closed-loop. Whether entering Closed-loop clears vel_ff on ODrive Pro 0.6.11 is undocumented, and the open 0.5.6 source does not clear it.

## Wanted

1. A bench-Hand measurement: stop the stream mid-stroke by a disarm and record where the Hand settles. Get the bench top stop first; the bench Hand has the shorter stroke.
2. If it matches the arithmetic, widen the hold flush to every suppress edge for the Hand, or send one frame with zero vel_ff on those edges. The drain already transmits on a Soft E-stop, so the rule "transmit nothing a stop forbids" is not what blocks it.
3. Record what the E-stop button cuts.
