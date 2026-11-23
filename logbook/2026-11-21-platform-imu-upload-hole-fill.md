---
title: "Platform IMU upload: hole-fill READs cover one cluster of holes and the settle drops to 5 ms (IDEA-25); the launch log gains an upload-finished line"
type: optimization
date: 2026-11-21
status: complete
phase: "IDEA-25"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - teensy_link/platform_imu.py
  - teensy_link/platform_imu_capture.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/docs/platform_imu_protocol.md
  - tests/teensy_link/fake_platform_imu.py
  - tests/teensy_link/test_platform_imu.py
  - tests/ros/test_teensy_bridge_node_platform_imu.py
subsystem: [platform-imu, teensy-link]
tags: [imu, upload, can]
---

# Platform IMU upload: hole-fill and settle (IDEA-25)

## What and why

At the Sitting of 2026-11-20 uploads while Armed took 79 to 84 s against 56 s at rest. The debrief
(`2026-11-20-sitting-debrief-robot-2`) found that about 0.4 % of data frames go missing when Armed and that the
Jetson's hole-fill re-read every held word between the first and last hole within 256 words (14651 duplicates of
82580 frames in capture AEDE). The owner approved the Jetson-only change on 2026-11-21. No firmware or wire change.

- `_Region.next_block`: a READ starts and ends on a missing word and bridges a run of present words only when it is
  at most `refill_merge_words` (16, about one READ turnaround in frames). 256 restores the old rule.
- `settle_s` 30 ms to 5 ms, and the worker now wakes when a settle falls due (it polled every 20 ms, which would
  have hidden the cut). A late frame still fills its word whichever block is current.
- Every hole is still requested once per pass; the stall guard (4 empty READs), the pass limit (8), the 300 s cap,
  the block deadline, the pacing and the Armed gap clamp are unchanged.
- The node logs one line when an upload ends: INFO "Platform IMU upload finished (...)" with samples, seconds,
  frames, duplicates, READs, block timeouts, hole passes, CRC result and file stem; WARNING for failed, aborted,
  incomplete or CRC mismatch. It is raised from the one place the result is finalised.
- The capture file is now named from the capture's `uploaded_wall_time_s`, so the logged stem is the file's stem.

## Model (real UploadMachine, fake Platform with seeded loss, whole ring, gap 80)

At p = 0.004: duplicates 13772 to 98 and 12671 to 168 (two seeds), READs 391 to 519, wall 69.8 s to 55.6 s. The fake
has no READ round trip, so real times run about 8 to 10 s higher: expect about 62 to 66 s Armed (79 s before) and
56 s or less at rest. At p = 0.0005: duplicates 242-330 to 0.

## Still to verify on the robot

Whole-ring upload at rest: fewer than 100 duplicates, 56 s or less, CRC ok. Armed in the Active pose: fewer than
1000 duplicates, about 500 READs, 62 to 66 s, CRC ok, CAN1 load unchanged at about 74 to 77 %. Read the numbers
from the new log line. Where the 0.4 % is lost is still unknown (needs counters in both firmwares).

## Verification

See the commit message for the (date, command, result) triple.

## Closed (2026-11-23)

Closed with the plan's archival. The code ran Armed and in an Attempt at the Sitting of 2026-11-23 (`2026-11-23-sitting-debrief-robot`): uploads complete and checksum-verified. Upload time is carried by IDEA-25.
