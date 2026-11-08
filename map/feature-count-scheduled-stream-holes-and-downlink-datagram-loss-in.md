---
id: FEAT-41
kind: feature
title: Count scheduled-stream holes and downlink datagram loss in the can-bridge heartbeat
status: complete
created: '2026-11-05'
updated: '2026-11-06T03:04:56Z'
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
- logbook/2026-11-06-jb174-stream-socket-queue.md
- logbook/2026-11-06-sitting-debrief.md
filed_by: claude
---

On 2026-11-05 three Hand-only STOPs ended Attempts with every host-side counter clean, so a Hand frame lost in the downlink is invisible today (BUG-66, BUG-58).

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
- do: Confirm the launch log reads `can-bridge v40`. Launch and arm as usual, then leave the robot holding the Active pose for 5 minutes before the first goal. Fly the rest of the Sitting as normal. The bridge logs one `downlink summary` line each time streaming ends.
- pass: every `downlink summary` line of the Sitting reads lost 0 and queue_dropped +0, over at least 10,000 setpoints in total (seq_gaps is recorded, not judged).
- record: the launch log path and its `downlink summary` lines.
- minutes: 5

## Progress

- 2026-11-06: Claimed 2026-11-06 (owner go-ahead): FW 40 / PROTOCOL 12 lands the stream-socket queue fix (1 -> 8 packets) and the heartbeat counters (setpoints received, queue drops, seq gaps, checksum failures, scheduled frames, flagless setpoints) plus a host lost-setpoint figure. Evidence: sent-minus-census shortfall 1.3-1.5 % in three bags. The stamp-hole counter and host ring stay open here.
- 2026-11-06: FW 40 / PROTOCOL 12 built and tested 2026-11-06: stream socket queue 1 -> 8, heartbeat ledger (dn_rx_setpoints, dn_queue_dropped, dn_seq_gaps, dn_crc_errors, sched_frames, sp_seq_drops, sp_no_hand, sp_no_sched), /link_status setpoints_lost, STOP WARN downlink segment, one downlink summary line per streaming episode. Open here: promotion stamp-hole counter, host per-frame ring.
- 2026-11-06: Passed at the Sitting of 2026-11-06 (owner confirmed): can-bridge v40, three downlink summary lines, 12613 setpoints, lost 0 and queue_dropped +0 on all; seq_gaps +3 in the arm period that ended in the E-stop (recorded, not judged).
