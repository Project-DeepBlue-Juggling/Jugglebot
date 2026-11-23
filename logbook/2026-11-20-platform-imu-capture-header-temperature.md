---
title: "Platform IMU: the capture header's temperature -128 was 24.03 C, not the unknown sentinel; the capture .json now carries it decoded, and the UI filter is decided at ODR/2 (IDEA-20)"
type: bugfix
date: 2026-11-20
status: complete
phase: "platform-imu P4"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - teensy_link/platform_imu_capture.py
  - tests/teensy_link/test_platform_imu.py
  - tests/teensy_link/test_platform_imu_native.py
  - ros_ws/docs/platform_imu_protocol.md
  - ros_ws/src/jugglebot/Teensy_code_platform/icm42688.h
  - ros_ws/src/jugglebot/Teensy_code_platform/icm42688.cpp
  - plans/active/platform-imu.md
subsystem:
  - can
tags:
  - testing
  - docs
---

# Platform IMU: capture header temperature

**What was seen.** Capture `20261120T095142Z_C33B` (FW 12) had `temp_raw_start = temp_raw_freeze = -128` beside an
IMU_STATUS `temp_c` of 24, read as the STATUS "unknown" sentinel.

**Root cause: no firmware defect, a units misreading.** The header fields are raw TEMP_DATA counts
(`platform_imu_protocol.md` section 3 table, degC = raw / 132.48 + 25), not the STATUS i8. Both come from the same
FIFO packet byte (`icm42688.cpp` packet parse, degC = byte / 2.07 + 25, `icm42688.h` `Sample::temp`): STATUS rounds it
(`imu_service.h` `temp_c_from_fifo`), the header stores it x 64 (`temp_data_from_fifo`, 132.48 / 2.07 = 64). FIFO byte
-2 gives STATUS 24 and header -128 = 24.03 C: the same reading, and the natives already pinned both conversions. The
gap was on the Jetson: the `.json` held only the raw counts, and nothing said they are not the STATUS scale.

**Fix (Jetson only, no firmware change, no version bump).** `platform_imu_capture.header_to_dict` adds
`temp_c_start` / `temp_c_freeze` (raw / 132.48 + 25; None when N = 0, where the firmware writes 0 for "no reading"),
keeping every raw key. The spec states the formula, the 64-count step and the N = 0 rule.

**Ride-along (comments and plan only).** UI filter decided at ODR/2, code 0x00 (owner, 2026-11-20): the `TODO(IDEA-20
P2)` in `icm42688.h` and the plan's decisions row now say so. Note: the header's GYRO_ACCEL_CONFIG0 byte is the
`UI_FILT_CODE` constant (`.ino` `header_info`), not a chip read-back.

**Verification.** 2026-11-20: the new `test_header_temperatures_decode_to_degrees_c_in_the_capture_json` and the
extended `test_writer_round_trip` were red before the fix (2 failed) and pass after. 2026-11-20,
`flock /tmp/jugglebot-tests.lock pytest tests/firmware tests/teensy_link/test_platform_imu.py
tests/teensy_link/test_platform_imu_native.py tests/ros/test_teensy_bridge_node_platform_imu.py -q`: **435 passed,
4 skipped in 370.53 s**. 2026-11-20, `pytest tests/sim/test_plans_index.py tests/sim/test_docs_links.py
tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_no_legacy_item_ids.py -q`:
**61 passed**. 2026-11-20, `pio run -e teensy40` and `-e teensy40_imudiag_const`: both SUCCESS (comment-only
firmware edits).

**PENDING (hardware):** no flash is needed. At the next capture upload, confirm the `.json` header carries
`temp_c_start` / `temp_c_freeze` within 0.5 C of the STATUS `temp_c`.

## Closed (2026-11-23)

Closed with the plan's archival. The code ran Armed and in an Attempt at the Sitting of 2026-11-23 (`2026-11-23-sitting-debrief-robot`): uploads complete and checksum-verified. Upload time is carried by IDEA-25.
