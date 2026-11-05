---
title: "JB-174 / JB-149: the can-bridge stream socket held ONE packet, so about 1.5 % of setpoints never reached the scheduler; FW 40 / PROTOCOL 12 deepens the queue to 8 and uplinks a downlink ledger"
type: investigation
date: 2026-11-06
status: in-progress
phase: "JB-174 downlink loss accounting"
related_entries:
  - 2026-11-05-sitting-debrief.md
  - 2026-11-03-jb149-firmware-sched-counters.md
  - 2026-11-04-jb149-probe-sitting-and-jb139-close.md
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_link.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_link.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - teensy_link/gap_monitor.py
  - teensy_link/rpc_args.py
  - tests/firmware/native/hal_shims/QNEthernet.h
  - tests/firmware/native/test_udp_link.cpp
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/ros/test_teensy_bridge_node_bridge_diag.py
  - tests/ros/test_teensy_bridge_node_read.py
  - tests/teensy_link/test_gap_monitor.py
  - tests/teensy_link/test_hand_move_to.py
  - tests/teensy_link/test_protocol_codec.py
  - tests/teensy_link/test_v5_wire_regression.py
subsystem: can-bridge
tags: [udp, downlink, setpoint-stream, LANE_REFUSED, JB-149, JB-166, JB-174]
---

# JB-174 / JB-149: the stream socket's one-packet queue was deleting setpoints

## Summary

The firmware STOPs "on a clean host stream" (JB-149's second class, JB-166) each need exactly one setpoint frame to be missing at the firmware, and frames are missing: in every bag that carries the FW 38 frame-age census the firmware counted 1.3 to 1.5 % fewer stamped frames than the host sent. The stream socket was still on QNEthernet's default receive queue of ONE packet, which a second datagram in the same 1 kHz `task_net` tick overwrites. The host sends two kinds of datagram to that socket (40 Hz SETPOINT, 10 Hz HEARTBEAT_J2T), so a heartbeat landing just after a setpoint deletes it. FW 40 gives the stream socket the queue of 8 the RPC socket got in FW 22 and uplinks a downlink ledger so the next loss is a number. Built, tested and flashed 2026-11-06; the confirming Sitting row (JB-174) is still owed.

## Symptoms

Sitting 2026-11-05 22:46 (launch log `~/.ros/log/2026-11-05-22-46-00-334370-jetson-688679/launch.log`, bag `~/Desktop/rosbags/2026-11-05_22-46-00`): four LANE_REFUSED ends and one SPINOUT. Four of the five firmware STOPs had a host stream the bridge reported clean (forward gap 38 to 44 ms, loop gap under 20 ms, every `fw_sched` drop counter +0, frame ages 11.6 to 27.6 ms). The fifth (L1602) was host-side: a 60.0 ms gen-2 GC pause in `teensy_bridge_node`, forward gap 99.5 ms.

## Diagnosis

**1. The firmware's rule needs a missing frame.** A frame covers 50 ms from its stamp and frames are stamped 25 ms apart (`leg_interp.cpp` COVER). With every frame present, a both-lane STOP needs a frame older than 45 ms at arrival (cover 50 + grace 20 - 25) and the FW 37 Hand C1 stop needs one older than 31 ms (50 + 6 - 25). No frame near any of the four clean STOPs was that old. With ONE frame missing, the next frame has to arrive younger than the grace: 20 ms for both lanes, 6 ms for a fast-rising Hand.

| Launch line | STOP | Oldest frame nearby | Needs |
|---|---|---|---|
| L1186 (ends 9/9) | legs + Hand, +2 | 23.6 ms | one missing frame, next older than 20 ms |
| L1399 (ends 3/3) | Hand C1 | 15.6 ms | one missing frame, next older than 6 ms |
| L2361 (ends 25/26) | Hand C1 | 11.6 ms | same |
| L2589 (SPINOUT) | Hand C1 | 27.6 ms | same |

The per-window frame age (max per 100 ms heartbeat window, 2,932 streaming windows) has median 10.6 ms, is above 6 ms in 99.2 % of windows and above 20 ms in 2.1 %. So nearly every frame lost during a Throw upstroke trips the C1 stop, and about one loss in fifty trips both lanes. That also explains why every clean both-lane STOP since 2026-11-03 showed a forward gap of 37 to 47 ms: the next frame has to be late by about 12 ms on top of its usual 8 to 10 ms age.

**2. Frames are missing, at a steady rate.** `/link_status` deduplicated by `uptime_ms` (one row per firmware heartbeat), streaming stretches only, host `setpoints_sent` against the summed frame-age buckets (`rx_age_note` runs for every scheduled frame that passes the future-drop check, and `sched_demoted`, `sched_future_drops`, `setpoints_rejected` are all 0):

| Bag | Streaming | Sent | Counted by firmware | Short |
|---|---|---|---|---|
| 2026-11-03_22-29-00 | 620 s | 24,794 | 24,427 | 367 (1.48 %) |
| 2026-11-04_18-45-46 | 93 s | 3,701 | 3,654 | 47 (1.27 %) |
| 2026-11-05_22-46-00 | 293 s | 11,724 | 11,551 | 173 (1.48 %) |

The shortfall is the same holding at rest (1.11 % over 148 s) as juggling. The uplink on the same cable shows `seq_gaps` 0 and `crc_errors` 0 over more than 87,000 frames, and the Jetson's `eth0` shows `tx_dropped` 0 and `tx_errors` 0, so the wire and the host interface are unlikely.

**3. The 2026-11-03 night ends fit.** Re-read with the same script: its one refusing both-lane STOP had a 24.8 ms frame, and its other two refusing STOPs were Hand C1 stops (`sched_hand_c1_stops` +1) that the 2026-11-04 note had filed as "firmware-side, unknown".

**4. Where a frame can go missing.** `udp_link.cpp` documents it: QNEthernet's `EthernetUDP` queues ONE packet and overwrites the oldest when full (`QNEthernetUDP.cpp` `recvFunc`: `droppedReceiveCount_++`, then `inBuf_.put()`), FW 22 deepened the RPC socket's queue for exactly this, and "the stream socket keeps the library default". The queue is emptied once per 1 ms `task_net` tick.

## Discussion

**Withdrawn.** JB-149's body treated the clean-host STOPs as an unknown firmware-side cause (stamps, the ISR, or link latency). The frame-age census rules out latency, and the firmware's own cover rule plus the measured shortfall replace "unknown" with "one frame missing". The host-starvation hypothesis stays true for the gaps it was written for (2026-11-02, and L1602 here) and is not the cause of the other class.

**What is established and what is not.** Established: the necessary condition (a missing frame per clean STOP) and the supply (1.3 to 1.5 % of frames uncounted, three Sittings). NOT established: that the one-packet queue is where they go. A uniform-phase estimate of setpoint-then-heartbeat inside one 1 ms tick gives about 0.5 % (10 Hz x 40 Hz x 0.5 ms), a third of the measurement. Sends may cluster (the heartbeat thread tends to get the GIL right after the forwarding thread's `sendto`), and part of the shortfall may be back-to-back setpoints after a host stall, where the overwritten one is harmless. The ledger settles it.

**Pre-registered criterion.** JB-174's Sitting row: hold armed at rest for five minutes; pass is `lost 0` and `queue_dropped +0` on every `downlink summary` line. If `lost` stays near 1.5 % with `queue_dropped +0`, the queue was not the mechanism: `dn_seq_gaps` then says whether the datagrams reached the firmware socket at all, and the fallback is tolerance instead of prevention (one knot of emit lead, an owner decision with an offline replay first, because it moves splice timing).

**Why the queue and not the grace.** FW 39 widened the Hand C1 grace from 6 to 14 ms. That lets most single losses ride through (frame age exceeds 14 ms in 13 to 20 % of windows) at the price of Apex 1.10 as the ceiling. Removing the loss removes the trigger for both the C1 stop and the both-lane stop and costs nothing in the stroke budget.

**Why the heartbeat and a PROTOCOL bump.** An additive message would have avoided the bump, but the counters belong with the `sched_*` family they close, the STOP WARN already differences the heartbeat ring, and a flash was needed for the queue anyway.

**Not done here (stays on JB-174).** The per-group promotion stamp-hole counter (ISR code, safety-bearing) and the host per-frame ring.

## Fix

- **Firmware (FW 39 -> 40, wire change).** `STREAM_RX_QUEUE_CAPACITY = 8`, set before `begin`, `static_assert`ed against the drain budget like the RPC one. New cumulative counters: `rx_setpoints` in `UdpStats` (every valid SETPOINT datagram, before the interp guards), `udp_link_stream_queue_dropped()` (the library's `droppedReceiveCount`), and in `leg_interp.cpp` `s_sp_seq_drops`, `s_sp_no_hand`, `s_sp_no_sched`. HeartbeatT2J grows 158 -> 190 B with `dn_rx_setpoints`, `dn_queue_dropped`, `dn_seq_gaps`, `dn_crc_errors`, `sched_frames`, `sp_seq_drops`, `sp_no_hand`, `sp_no_sched`; PROTOCOL_VERSION 11 -> 12 through the generator.
- **Jetson.** `DownlinkLossTracker` (pure, `teensy_link/gap_monitor.py`): lost = recent minimum of (sent - received) minus its all-time minimum, so unequal baselines and frames in flight are not loss. The bridge node samples its sent count on the RX thread when a heartbeat lands, publishes the eight counters and `setpoints_lost` on `/link_status`, appends a `downlink last N heartbeats` segment to every STOP WARN, and logs one `downlink summary` line each time streaming ends (INFO when clean, WARN otherwise). `EXPECTED_BRIDGE_FW_VERSION = 40`.

**One control cycle with the change.** A setpoint and a heartbeat arrive in one tick. Before: the heartbeat overwrote the setpoint, the scheduler never saw that knot, and the previous frame's cover had to bridge 50 ms. Now both sit in the queue and the drain dispatches them in arrival order in the same tick: the setpoint enters the schedule queue with its true age, the heartbeat updates the flags. Two setpoints in one tick (after a host stall) used to deliver only the newer; now the older is dispatched first and the newer second, which is what adjacent ticks already did (the schedule queue orders by stamp, the ISR promotes the newest due frame). Nothing is delayed, no tolerance or grace changes, and the drain budget of 8 per tick still bounds a flood.

## Verification

- 2026-11-06, `python -m pytest tests/firmware/test_native_firmware.py -q -p no:cacheprovider`: **17 passed in 238.60 s**. Includes the new `test_udp_link.cpp` case: with the library default of one packet a setpoint followed by a heartbeat in one tick gives `rx_setpoints` +0, `seq_gaps` +1, dropped +1; with the FW 40 queue both are dispatched and all three stay flat.
- 2026-11-06, `python -m pytest -q -p no:cacheprovider -n 4 tests/teensy_link tests/firmware/test_bridge_fw_version_xref.py tests/firmware/test_udp_protocol_xlang.py tests/ros/test_teensy_bridge_node_bridge_diag.py tests/ros/test_teensy_bridge_node_read.py`: **629 passed in 14.58 s**.
- 2026-11-06, `pio run -e teensy41` in the commit's tree: **SUCCESS in 83.64 s**.
- 2026-11-06 00:42, `./run_tests.sh -q` in the commit's tree, launch down, no other gate running: **RESULT: PASS** (parallel 441 s rc=0, serial 26 s rc=0, total 467 s).
- 2026-11-06 00:52, `pio run -e teensy41 -t upload` (launch down, board was FW 39): **SUCCESS**, 287,744 bytes. Receipt, an 8 s listen with the commit's `teensy_link`: BridgeIdentity `fw_version` 40 in 8 of 8 frames, 80 of 80 HeartbeatT2J decoded at 190 B, all eight ledger counters 0 (no stream yet), host `seq_gaps` 0. The robot was unpowered, so the heartbeat read fault state ODRIVE_FATAL, as it did under FW 39 in the unpowered launch at 00:37; nothing here exercised a lane.
- NOT yet verified: the loss figure under a live stream. That is the JB-174 Sitting row.

## Open

- JB-174 Sitting row (five minutes armed at rest): the confirming measurement. Until it passes, the queue as the loss mechanism is a hypothesis.
- JB-149: the gc_freeze A/B for `teensy_bridge_node` (the 60 ms pause) is independent of this and still owed.
- The refused resume after a STOP (profiled rejoin) remains an owner decision on JB-149.
- If the row passes, the FW 39 Apex 1.10 ceiling can be revisited (owner): the 14 ms grace was bought to survive losses that should no longer occur.
