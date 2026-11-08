---
id: FEAT-46
kind: feature
title: 'Firmware STOP record: report why a scheduler cover ended (frame stamps, arrivals, queue, time-base steps)'
status: inbox
created: '2026-11-08'
updated: '2026-11-08T00:28:19Z'
owners: []
priority: high
effort: m
owner: decision
radius: normative
area: firmware
blocked_by: []
links:
- map/bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms.md
- logbook/2026-11-07-sitting-debrief.md
filed_by: claude
---

The next step for BUG-58. Filed 2026-11-08 at the owner's request after the debrief of the 2026-11-07 Sitting (logbook/2026-11-07-sitting-debrief.md).

## Why

At the 2026-11-07 Sitting, 20 of 23 firmware scheduler STOPs had a clean host stream:
- forward gaps of 31 to 64 ms, under the 70 ms cover plus grace;
- frame age at arrival at most 35.7 ms;
- downlink lost 0 of 83,340 setpoints;
- `expired`, `superseded`, `stale`, `future_drops` and `q_overflow` all 0.

Both `LANE_REFUSED` ends were in this class. Today's STOP line cannot say why the cover ran out.

## What the cover depends on

The cover runs from each frame's `t_start` (its stamped play time mapped onto `micros64()` through the wall offset), not from its arrival (`leg_interp.cpp`, the COVER and QUEUE comments). A STOP therefore needs no newer frame promoted within 70 ms of the playing frame's `t_start` (64 ms for a Hand C1 stop). On-time arrival and a STOP are compatible only if a stamp or the mapping is off.

## The probe

Add, for every STOP, a record that reaches the host's STOP WARN:
1. the STOP entry time and the playing frame's `t_start`;
2. the `t_start` and arrival time of the last four frames queued for that group, and the queue depth at the STOP;
3. the next frame's `t_start` and arrival time, when it comes;
4. any time-base step or slew larger than 1 ms in the 200 ms before the STOP (`time_base.cpp` steps on a large error).

## What it would separate

- **A stamp gap:** the host stamped successive frames more than 25 ms apart, for example at a Splice or a re-plan.
- **A mapping step:** the wall offset moved, so on-time frames map to a play time already past.
- **A promotion fault:** a due frame sat in the queue and was never promoted.

None of these is a conclusion. The data so far cannot tell them apart.

## Cost

Firmware and a HeartbeatT2J or new-message wire change, so a `PROTOCOL_VERSION` bump and a flash. Behaviour unchanged: observability only. A host-side half (log each frame's stamp spacing at send) is cheap and could land first.
