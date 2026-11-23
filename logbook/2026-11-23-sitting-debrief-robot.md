---
title: "Robot Sitting 2026-11-23 debrief: the Attempt capture is checksum-verified, closing IDEA-20 and Rung P5; Armed uploads take 57.7 s and 49.3 s; the Platform sends every frame and the bridge's CAN receive loses about 0.6 %"
type: investigation
date: 2026-11-23
status: complete
phase: "Sitting debrief"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - logbook/2026-11-23-sitting-debrief-robot.md
  - logbook/INDEX.md
  - map/sheets/2026-11-23-robot.results.yaml
  - map/idea-platform-accelerometer.md
  - map/idea-minimise-the-platform-imu-upload-time-after-an-attempt.md
  - map/bug-teensy-bridge-node-freezes-for-about-340-ms-once-in-some.md
  - plans/active/platform-imu.md
  - docs/records.md
subsystem: [platform-imu, firmware, bridge]
---

## Summary

Sheet `2026-11-23-robot`, 12 steps, one launch (14:31 to 14:39), can-bridge firmware 49, Platform Teensy firmware 14.
Launch log `~/.ros/log/2026-11-23-14-31-05-999737-jetson-2636737/launch.log`, bag `~/Desktop/rosbags/2026-11-23_14-31-06`.
One Attempt: `oval_1ball_fed`, 8/8 caught. No Guard latch, no Hand stall, no Fault.

| Item | Step | Derived | Owner | Measured |
|---|---|---|---|---|
| IDEA-20 | 9 | pass | confirmed | Attempt upload: 21248 samples, 40.1 s, complete yes, crc ok (chunks), file `20261123T033855Z_4AFA` |
| IDEA-25 | 6 | fail | confirmed | 57.7 s at gap 80 (limit 57), 49.3 s at gap 63 (limit 48); every other clause met |

Nothing merged (neither Item had a held branch). IDEA-20 is closed and Rung P5 of `platform-imu.md` is complete.
The records commit for this Sitting (`docs(records): Sitting 2026-11-23 robot ledger`) carries the trailer
`2026-11-23-sitting-debrief`, which is the bench Hand debrief's slug; it belongs to this entry.

## Big picture

What went well. The last open requirement of the Platform IMU work is met: a real Attempt's capture came back
complete and verified chunk by chunk, so IDEA-20 and Rung P5 are done. Firmware 14, the pipelined refill reads and
the stall watchdog all ran Armed and in an Attempt for the first time with no failure. Whole-ring uploads while Armed
now take 57.7 s at the default pace and 49.3 s at the faster pace, against 63 s and about 67 s on 2026-11-22.

What did not. Both uploads missed their estimated limits, by 0.7 s and 1.3 s. The bridge node froze once more, for
313 ms, one second after start-up.

What it means. The transmit counters settle where the frames go: in all three uploads the Platform placed and sent
every frame (66518 of 66518 in the first), the bus error counters stayed at zero, and the bridge counted about 400
fewer. The frames are acknowledged on the wire and never reach the bridge's software, so the bridge's CAN receive
drops them, in step with its 500 Hz setpoint tick as found earlier. The Platform and the wire are cleared. Refill
reads now cost about 20 ms each instead of 60 ms, so the upload time is almost all first-pass pacing: 52 s at the
default pace, 41 s at the faster one. The faster pace is now a real gain. The freeze has the same stack in all three
unprovoked dumps: one thread inside the ROS publish of the robot state, every other thread idle.

Next steps. IDEA-20: closed; `platform-imu.md` is ready for `/archive-plan`, and the capture goes to the
carry-ring-zvd analysis. IDEA-25: owner decisions on making the faster pace the default (saves 8 s now) and on a
can-bridge change to stop the receive loss, which is what would allow a pace above the present Armed cap. BUG-129:
the dumps point at the robot-state publish; the next step is to move or guard that publish, which is a design choice
to agree first.

## Results

Upload lines (launch log lines 784, 1282, 2070) and capture statistics (`temp/imu_capture/<file>.json`):

| Capture | Pace | Wall | Refill | Missing after pass 1 | Duplicates | Platform placed / sent | Bridge received | Bridge minus Jetson |
|---|---|---|---|---|---|---|---|---|
| `4AF8` whole ring | gap 80 | 57.7 s | 5.4 s | 413 | 999 | 66518 / 66518 | 66116 | 0 |
| `4AF9` whole ring | gap 63 | 49.3 s | 8.0 s | 436 | 52 | 65604 / 65599 | 65158 | 0 |
| `4AFA` Attempt span | gap 80 | 40.1 s | 4.6 s | 377 | 936 | 44919 / 44919 | 44499 | 0 |

- Transmit and receive error counter sums 0, lost mailboxes 0, `tx_queued_anyway` 0 in all three. `4AF9`'s 5
  unsent frames were still pending when the counters were read.
- Frames waited more than 2 ms in a transmit mailbox 1333 to 1994 times per upload (longest 5.6 ms): the bus is busy,
  but nothing was dropped on the transmit side.
- Checksums: `chunks+payload` on both whole rings, `chunks` on the Attempt span; 0 chunk re-reads.
- BUG-129: stall file `temp/logs/stall_teensy_bridge_node_2026-11-23T14-31-10_2636775_1.txt`, 313.2 ms, at
  14:31:10, before Homing and unarmed. Stack: `_publish_robot_state` inside `rclpy/publisher.py` `publish`; all
  other threads waiting. Same as the 14:07:33 dump. No freeze during the Armed period.
- Trapped-Ball scan: nothing found (Verification).

## Verification

- 2026-11-23, `python -m mapkit sheet derive / confirm / close 2026-11-23-robot`: wrote 2 Sitting results (IDEA-25,
  IDEA-20); owner confirmed the derived table unchanged.
- 2026-11-23, `python tools/trapped_ball_finder.py ~/Desktop/rosbags/2026-11-23_14-31-06` (25014 Mocap frames):
  no unlabelled-marker dwell, no Hand stall.
- 2026-11-23, `pytest tests/sim/test_mapkit*.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_plans_index.py tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py -q`:
  325 passed. Documentation only (map, plan, logbook, records), so the named tests are the gate; no code changed
  and nothing merged.
