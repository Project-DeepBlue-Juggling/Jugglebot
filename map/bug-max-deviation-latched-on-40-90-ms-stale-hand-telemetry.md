---
id: JB-160
kind: bug
title: MAX_DEVIATION latched on 40-90 ms stale Hand telemetry without MOTOR_FB_STALE firing first
status: inbox
created: '2026-11-03'
updated: '2026-11-03T02:50:51Z'
owners: []
priority: high
effort: m
hardware: robot
diagnosis: hypothesis
area: hand
blocked_by: []
links:
- map/bug-hand-odrive-feedback-starvation-701-ms-heartbeat-dropout.md
- map/bug-teensy-guard-max-deviation-latched-on-the-hand-8-4-rev-mid.md
- map/bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms.md
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
