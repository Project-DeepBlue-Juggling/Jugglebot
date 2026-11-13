---
id: FEAT-46
kind: feature
title: 'Firmware STOP record: report why a scheduler cover ended (frame stamps, arrivals, queue, time-base steps)'
status: complete
created: '2026-11-08'
updated: '2026-11-13T05:58:07Z'
owners: []
priority: high
effort: m
hardware: robot
owner: decision
radius: normative
area: firmware
blocked_by: []
links:
- map/bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms.md
- logbook/2026-11-07-sitting-debrief.md
- logbook/2026-11-13-sitting-debrief.md
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

## Sitting

- setup: armed
- why: FW 42 uplinks a STOP record per lane group, and the bridge classifies each one, so the next STOP on a clean host stream says whether a stamp gap, a mapping step or an unpromoted frame ended the cover.
- do: nothing extra: a ride-along on any armed Attempt, after the FW 42 flash and the colcon build (the launch log's BridgeIdentity line reads fw_version 42).
- pass: every firmware scheduler STOP WARN in the launch log is followed by its `firmware STOP record #<n> <group>` line carrying a classification.
- record: each STOP record line's verdict and numbers (successor stamp gap, map shift, queue head, time-base change) and the `overwritten` count.
- minutes: 1

## Sitting result (2026-11-13 13:59)

- verdict: pending
- notes: (sheet 2026-11-13-robot, step 43)

## Progress

- 2026-11-08: FW 42 / PROTOCOL 13 STOP record built and tested (pio build OK, native + host tests), NOT flashed: HeartbeatT2J 190->380 B carries one record per lane group; the bridge logs one classified line per record (stamp gap / mapping step / queued, not promoted / none of these). Needs the flash, a colcon build and a Sitting.
- 2026-11-08: 2026-11-08 14:41: FW 42 flashed (launch down), receipt 6,225 frames, 0 decode errors at PROTOCOL 13; colcon built; full gate PASS. Live check: the first STOP at the next Sitting must print its classified STOP record line.
- 2026-11-13: Passed at the Sitting of 2026-11-13: 108 STOP records over four launches (counter 1..108, overwritten 0), every STOP WARN has its record. The records pointed at the host (stamp gap, successor +50 ms) and led to the located cause, the :5557 drain in teensy_bridge_node discarding one older knot (BUG-89).
