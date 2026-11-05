---
id: JB-160
kind: bug
title: MAX_DEVIATION latched on 40-90 ms stale Hand telemetry without MOTOR_FB_STALE firing first
status: complete
created: '2026-11-03'
updated: '2026-11-05T12:29:20Z'
owners: []
priority: high
effort: m
hardware: robot
owner: hands
radius: normative
diagnosis: hypothesis
area: hand
blocked_by: []
links:
- map/bug-hand-odrive-feedback-starvation-701-ms-heartbeat-dropout.md
- map/bug-teensy-guard-max-deviation-latched-on-the-hand-8-4-rev-mid.md
- map/bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms.md
- logbook/2026-11-04-sitting-debrief.md
- logbook/2026-11-05-sitting-debrief.md
filed_by: sitting-debrief 2026-11-03
---

At 2026-11-03 12:35:23 (launch.log L2528) the Teensy guard latched MAX_DEVIATION with the Hand "first to cross" at +3.012 rev, but the Hand was at a normal 9.34 rev against a 9.395 rev command. The deviation was a fresh command compared against a frozen encoder reading.

## Findings

- Bridge Hand telemetry repeated stale samples for 40 ms (frozen at 0.114 rev), 90 ms (3.288 rev) and 30 ms (8.949 rev).
- The setpoint echo had a 60 ms gap, after which the command jumped 1.59 -> 8.06 rev; the drive's own loop hit +59.94 A on that step.
- The Hand itself was inside its stroke and topped at 9.34 rev.
- `MOTOR_FB_STALE` (the Hand guard's 30 ms freshness gate, FW 26) did not fire before MAX_DEVIATION, although the frozen windows were 40 and 90 ms.
- One of 3 Hand faults in the 10-Attempt Apex 0.95 block (L1019 and L2277 were SPINOUTs, see JB-147); the owner noticed nothing at the time.

## Not yet known

Which layer froze: the drive's CAN feedback, the bridge's Hand telemetry cache, or the order in which the guard evaluates MAX_DEVIATION and the freshness gate. The 100 Hz capture cannot tell.

## Next

Find why the guard compared a fresh command with an encoder frozen 40-90 ms without `MOTOR_FB_STALE` first. A correctness fix, not a threshold change. Same starvation class as JB-137 (Hand feedback starvation), JB-98 (MAX_DEVIATION latched on the Hand) and JB-149 (setpoint stream starvation).

## Findings (Night Shift 2026-11-04, firmware read)

- `MOTOR_FB_STALE` cannot fire inside 40-90 ms. Its threshold is `MOTOR_FB_STALENESS_US` = 150 ms (`canbridge_config.h`), for the Hand as for a Leg (`fault_machine.cpp`, the FW 27 Hand block). Its silence on these windows is by design, not an evaluation-order bug.
- The 30 ms figure is a different gate: `HAND_DEV_FRESH_US` (FW 27, `leg_interp.cpp` near the dev_over counter). A Hand exceed tick counts toward the MAX_DEVIATION latch only when the encoder anchor (`pos_timestamp_us`, stamped at each encoder CAN RX) is at most 30 ms old. An older anchor's tick counts as a stale skip instead.
- So the latching tick had an anchor at most 30 ms old by the firmware's own RX clock. The firmware did not compare a fresh command with a 40-90 ms frozen anchor. Either the drive kept sending fresh frames carrying a frozen position, or the freeze is in the recorded telemetry and not in the firmware's anchor.
- An honest residual fits the record: the setpoint echo resumed after its 60 ms gap with a 1.59 -> 8.06 rev command step, and the drive's loop hit +59.94 A chasing it. A Hand lagging such a step by more than 2.5 rev (`MAX_DEVIATION_HAND_REV`) trips the guard correctly. That would put this trip in the JB-149 stream-gap class.

## Next (revised 2026-11-04)

Read the `hand_capture` CSV of 12:35:23. In the frozen windows, do the `telem` rows' `t_src_us` (Teensy stamps) keep advancing at the native rate? If yes, the firmware's own `pos_rev` froze while frames arrived, so look at the drive's feedback. If they gap too, the freeze is on the uplink or Jetson side and the trip is the honest residual above. Then read the guard's `dev_stale` (stale-skip) count across the block. No threshold change either way.

## Progress

- 2026-11-04: 2026-11-03 Sitting (22:29, P4 ramp): the two Hand MAX_DEVIATION latches (goals 3, 15) were preceded by a firmware scheduler STOP, not by stale telemetry: the Hand capture shows the command frozen and the encoder stopped; the deviation grew from the host plan moving on.
- 2026-11-05: Closed 2026-11-05: refuted twice (11-03 Night Shift read, 11-05 Sitting). Both 11-05 MAX_DEVIATION trips were the refused plan base vs a Hand at rest on its held setpoint, telemetry source gaps <= 10 ms (max 10.07), fb within 0.02 rev of the capture. The class is JB-98 / JB-166 (C1 stop, latched hold, refused frames).
