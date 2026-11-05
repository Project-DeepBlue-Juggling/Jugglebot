---
id: JB-174
kind: feature
title: Count scheduled-stream holes and downlink datagram loss in the can-bridge heartbeat
status: in_progress
created: '2026-11-05'
updated: '2026-11-05T13:06:21Z'
owners:
- jugglebot-f5
priority: high
effort: m
hardware: robot
owner: none
radius: normative
area: firmware
blocked_by: []
links:
- map/bug-firmware-scheduler-stop-fires-on-a-clean-host-stream-and.md
- map/bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms.md
filed_by: claude
---

On 2026-11-05 three Hand-only STOPs ended Attempts with every host-side counter clean, so a Hand frame lost in the downlink is invisible today (JB-166, JB-149).

## Evidence
- Drop counters 0, frame ages at most 28 ms, every numbered frame sent (seq vs setpoints_sent constant), no host stamp hole.

## Wanted
- Uplink `s_sched_frames` and the udp_link downlink `seq_gaps` in the can-bridge heartbeat.
- A count of frames received without HAS_HAND / HAS_SCHED.
- A promotion stamp-hole counter per group.
- A per-frame host ring (seq, flags, stamp).
- PROTOCOL version bump; flash with the launch down.

## Sitting

- setup: armed
- why: confirm that the FW 40 stream-socket queue fix takes downlink setpoint loss from about 1.5 % to zero (three Sittings measured 1.3 to 1.5 %).
- do: Launch and arm as usual, then leave the robot holding the Active pose for 5 minutes before the first goal. Fly the rest of the Sitting as normal. The bridge logs one `downlink summary` line each time streaming ends.
- pass: every `downlink summary` line of the Sitting reads lost 0, queue_dropped +0 and seq_gaps +0, over at least 10,000 setpoints in total.
- record: the launch log path and its `downlink summary` lines.
- minutes: 5

## Progress

- 2026-11-06: Claimed 2026-11-06 (owner go-ahead): FW 40 / PROTOCOL 12 lands the stream-socket queue fix (1 -> 8 packets) and the heartbeat counters (setpoints received, queue drops, seq gaps, checksum failures, scheduled frames, flagless setpoints) plus a host lost-setpoint figure. Evidence: sent-minus-census shortfall 1.3-1.5 % in three bags. The stamp-hole counter and host ring stay open here.
