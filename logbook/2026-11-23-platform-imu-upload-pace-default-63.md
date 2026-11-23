---
title: "Platform IMU upload pace: the node default is now gap 63 (about 1590 frames/s), from 80"
type: feature
date: 2026-11-23
status: complete
phase: "IDEA-25"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/docs/platform_imu_protocol.md
  - tests/ros/test_teensy_bridge_node_platform_imu.py
  - map/idea-minimise-the-platform-imu-upload-time-after-an-attempt.md
subsystem: [platform-imu, bridge]
---

## Summary

Owner decision after the Sitting of 2026-11-23 (`2026-11-23-sitting-debrief-robot`): the `teensy_bridge_node`
parameter `imu_read_gap_10us` defaults to 63 instead of 80. Armed, a whole ring took 49.3 s at 63 against 57.7 s
at 80, both complete and checksum-verified. 63 is the can-bridge's Armed clamp, so the bus rate while Armed is
no higher than the bridge already allowed. `UploadConfig.gap_10us` in `teensy_link/platform_imu.py` stays 80 (the
node always passes its own value). The fallback for a refused initial value is 63 too.

Not flown on the new default: uploads at rest (unarmed) now also ask for 63, where the earlier at-rest numbers
were taken at 80. IDEA-25's Sitting section checks the first Attempt upload.

The bridge's CAN receive loss is held as an open inquiry on IDEA-25 (owner, 2026-11-23); no firmware change.

## Verification

- 2026-11-23, `flock /tmp/jugglebot-tests.lock ./run_tests.sh -q` on this change over main at 67a33eec: RESULT: PASS (parallel rc=0 in 738 s, serial rc=0 in 33 s, total 771 s; `-q` printed no count).
- 2026-11-23, `pytest tests/ros/test_teensy_bridge_node_platform_imu.py -q`: 11 passed.
