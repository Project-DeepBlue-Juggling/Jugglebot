---
id: BUG-105
kind: bug
title: Hand keeps its last velocity feedforward on a Soft E-stop, disarm or link loss and stays in Closed-loop
status: inbox
created: '2026-11-14'
updated: '2026-11-14T02:40:04Z'
owners: []
priority: high
effort: s
hardware: bench-hand
owner: hands
radius: subsystem
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

## Already known in the tree (2026-11-14)

The property is not new: `tests/hardware/hand_stream_bench.py:150` and the 2026-11-03 logbook entry on the Hand stall-stop options both state that the bridge does not flush vel_ff on a disarm and that a disarm while moving leaves the Hand at `input_pos + vel_ff/35`. The bench driver is built to disarm only from a hold for that reason. What is missing is a measurement on the Hand and a decision on a cure.

## Sitting

- setup: bridge-cycled, armed
- why: the offset law `vel_ff / pos_gain` was measured on a Leg and only derived for the Hand; two low-speed disarms confirm it before any firmware change is designed around it.
- do:
  - Launch down. Bench Hand on the bridge, bridge rebooted after re-cabling, Hand homed and parked at or below 0.9 rev, Cup empty, E-stop in hand. Know the bench top stop in rev first: this row needs it at or above 6.0 rev.
  - From the repo root in the venv: `python tests/hardware/hand_stream_bench.py --stage triangle --tri-span 4 --tri-speed 10 --duration 60 --close-loop`. Press Ctrl-C once in the middle of an upward ramp, well away from either turning point. Expected offset 10 / 35 = 0.29 rev above the last command.
  - Read where the Hand settled and the last commanded position (the driver's last printed telemetry, or `pos_estimate` over the isolated USB). Recover with `python tests/hardware/hand_stream_bench.py --clear-errors`, re-home if needed.
  - Repeat at `--tri-speed 20` (expected 0.57 rev), again cutting on an upward ramp, and once more cutting on a downward ramp (expected 0.57 rev below).
  - Do not go faster in this row: the offset grows with speed and the purpose is the law, not the limit.
- pass: each settled offset is within 0.05 rev of `tri-speed / 35` in the direction of travel, and the Hand holds there without drifting.
- record: for each cut the ramp direction, the last commanded position, the settled position, the offset, whether any Guard or latch fired, and the bench top stop in rev. Also what the E-stop button cuts (ODrive bus power or not), since the repo does not say.
- minutes: 20
