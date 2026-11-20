---
title: "Platform IMU: platform_imu/status publishes the board health counters (commit 7be5a941), launch check on FW 12 and the first at-rest capture and upload (IDEA-20)"
type: feature
date: 2026-11-20
status: complete
phase: "platform-imu P4"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - teensy_link/platform_imu.py
subsystem:
  - can
  - ros
tags:
  - testing
  - docs
---

# Platform IMU: status counters and the first capture at rest

## What and why

Platform FW 12 (I2C retry and bus recovery, `2026-11-20-platform-imu-i2c-error-bursts`) is on main, the owner fitted 2.2 kOhm external pull-ups on SDA and SCL (the corruption is gone), and the diagnostic firmware envs are off by default. Commit 7be5a941 then made `platform_imu/status` publish what a Sitting needs to watch without a bag: `i2c_errors` and `fifo_overflows` (raw, saturated flags, Jetson-side totals since launch, age of the last increase), `temp_c`, `sync_peak` (raw, ms, valid, saturated) and the five flags as booleans. The level goes to WARN when `i2c_errors` or `fifo_overflows` rose in the last 60 s. No consumer reads the level. One audit of that commit: clean.

## Sitting 2026-11-20 (owner, FW 11)

The tilt request worked (read (+0.0107, +0.0084) rad at Level) and the Hand fan worked on command and in auto. The IMU upload toggle was off.

## Launch check, 20:47

Robot at rest, 48 V off, not Armed, `jugglebot` rebuilt with colcon first. PLATFORM_FW_CHECK OK v12, BRIDGE_FW_CHECK OK v46. `platform_imu/status` at level OK with every new key present: state RUNNING, `i2c_errors` 0 and `fifo_overflows` 0 about 30 min after the board's last boot, `temp_c` 24, flags SENSOR_OK, TIME_SYNCED and RING_WRAPPED set, DISCONTINUITY clear, `sync_peak` saturated at 2.54 ms.

## First capture and upload

The service is `SetString` carrying JSON; an empty object uploads the whole frozen ring:

`ros2 service call /platform_imu/upload jugglebot_interfaces/srv/SetString "{data: \"{}\"}"`

Result: `temp/imu_capture/20261120T095142Z_C33B.npz` and `.json`.

- 32000 samples, complete, payload CRC OK, 0 discontinuities, 0 unfilled holes, resume OK.
- 65138 frames in 263 blocks, 158 duplicate frames, 0 block timeouts, 1 hole pass, 0 CRC re-reads.
- Upload wall time 56.25 s at the default gap 80 x 10 us (not Armed).
- Time fit: chip rate 993.88 Hz, one segment, 323 anchors, residual 291 us rms local and 408 us rms wall.
- Header: fw_version 12, odr 1000, accel 2048 LSB/g, gyro 32.8 LSB/dps, i2c 1 MHz, `i2c_errors` 0, `fifo_overflows` 0, `sync_innovation_us` 10725.
- After the upload the board was back in RUNNING with no failure.

Data at rest: mean accel (2048.2, 14.6, 43.7) counts, |a| 2048.75 counts = 1.0004 g, so IMU x reads +1 g, consistent with the mount (IMU x = Platform +z). Noise std about 3 counts per accel axis (about 1.4 mg) and 1.5 counts per gyro axis (about 0.047 dps). Largest sample-to-sample step 17 counts accel and 9 counts gyro, so no corrupted samples. Gyro means (12.1, 8.9, -1.6) counts.

## GYRO_ACCEL_CONFIG0

The header reads back 0x00. That is what the firmware writes: `UI_FILT_CODE = 0x00` in `icm42688.h` (BW = ODR/2 for both accel and gyro), and `tools/probes/imu_bench/README.md` lists "value 0 = BW ODR/2" as MATCH against DS-000347 14.40. So the read-back confirms the chip holds what was written and the register is not being altered. It does not match the plan's agreed ODR/4 setting (plan section 3 table; the `TODO(IDEA-20 P2)` in `icm42688.h`): the ODR/4 code (0x11 from memory) was never verified because the datasheet text could not be read, so the filter was left at ODR/2. The register value is recorded; changing to ODR/4 remains an open owner decision, not done.

## Open defect

`temp_raw_start` and `temp_raw_freeze` in the capture header were both -128 (unknown) although the STATUS frame reports 24 C. Not diagnosed.

## Verification

- 2026-11-20, `flock /tmp/jugglebot-tests.lock ./run_tests.sh --full` on the branch tip: parallel 10664 passed, 60 skipped, 1 xfailed in 714.94 s; serial 10 passed in 40.00 s; RESULT PASS.
- 2026-11-20, launch check and capture above, on the robot at rest.

Still needing the owner with the robot powered: see Rung P4 in the plan and the IDEA-20 Sitting section.
