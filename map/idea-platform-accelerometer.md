---
id: IDEA-20
kind: idea
title: Platform Accelerometer?
status: in_progress
created: '2026-11-08'
updated: '2026-11-20T12:38:38Z'
owners:
- claude
priority: medium
effort: m
hardware: robot
owner: hands
radius: subsystem
area: platform-teensy
blocked_by: []
links:
- plans/active/platform-imu.md
- logbook/2026-11-20-sitting-debrief-robot-2.md
filed_by: harrison
night_skip: owner-session
night_skip_on: '2026-11-18'
night_skip_note: A day Session owns this; the Night Shift does not build Platform Teensy firmware.
---

Let's discuss the possible advantages that we might receive as a result of adding an accelerometer to the 2020 extrusion on which Jugglebot's hand's linear rail is mounted. Might this help with our 'input shaping' work, by allowing us to capture high resolution data on Platform acceleration? I imagine we have the Teensy pins available for this? What specific sensor might be best to use?

## Feasibility (2026-11-09)

Read from `Teensy_code_platform.ino`, with no sensor datasheet opened tonight, so the part figures below are from memory and need checking.

- **Pins.** The Platform Teensy already runs the SCL3300 inclinometer on SPI (pins 10 to 13). A second SPI sensor needs only one more chip-select pin. The Hand fan's pin is checked against those same pins at compile time.
- **What it would add.** A direct Platform acceleration signal at about 1 kHz or more. Mocap gives position, so acceleration comes from differentiating it twice, which is where high-frequency content is lost. It would show ringing after a Throw or a Banking move, which is what input shaping needs.
- **Existing sensor.** The SCL3300 is an accelerometer too, but its filter is tens of Hz (from memory), too slow for ringing. One bench capture of it would confirm that cheaply.
- **Sensor candidates.** ICM-42688-P: cheap, SPI, very fast, 6-axis. ADXL355: lower noise, 3-axis, slower, dearer.
- **Effort m, hardware robot.** A board on the 2020 extrusion, a cable, firmware, and a capture path.
- **Risk.** Streaming 1 kHz over CAN adds load on a bus that carries the Attempt, so a capture buffered on the Teensy and uploaded after the Attempt is the safer first version. The cable also rides a moving Platform.

## Options (2026-11-09)

1. Mount an ICM-42688-P, buffer a few seconds on the Platform Teensy and upload after the Attempt. No added CAN load in flight.
2. The same sensor, streamed live at a reduced rate over CAN, so a signal is available to future control. Needs a bus-load check first.
3. An ADXL355 instead, if noise floor matters more than bandwidth.
4. Read the SCL3300 at its fastest setting first and decide after that capture. No new hardware.

Recommendation: 4 as a quick look, then 1. A buffered capture answers the input-shaping question without touching the bus during an Attempt.

## Decision (2026-11-11)

From TKT-54. You ordered an ICM-42688-P breakout, and it is on its way. You want option 2 together with option 1: a low-rate live stream during the Attempt, plus the full-rate data buffered on the Platform Teensy and uploaded after the Attempt.

- What comes next is for a day Session, once the board is here. It covers the Platform Teensy firmware (SPI read, a ring buffer, the upload after the Attempt) and a protocol addition for the live rate.
- The live stream needs a CAN bus-load check before it is sized.
- The board needs a mount on the 2020 extrusion beside the Hand's rail.
- The Night Shift does not build Platform Teensy firmware.

## Bench proof of concept (2026-11-18)

Working at `tools/probes/imu_bench/` (commit afdf0e91, logbook `2026-11-18-imu-bench-proof-of-concept`).

- **Wiring.** Teensy 4.0 and ICM-42688-P over I2C, chosen over the SPI this Item first assumed; Stemma QT to the Platform Teensy pins 18 and 19.
- **Driver.** Chip-FIFO driver, 0 FIFO overflows and 0 I2C errors at 1 and 2 kHz; sensor rate 999.97 Hz at 1 kHz.
- **Rest check.** |a| 1.004 g at rest on one face; the other five faces and a 90 degree rotation are still open.
- **Next.** A day Session started 2026-11-18: the question round, calibration options and CAN transport design come before any Platform Teensy firmware.

## Sitting

- setup: level
- why: Rung P5 of plans/active/platform-imu.md: measure how the IMU sits on the Platform so its readings can be expressed in the Platform frame, and close the residue of Rung P4 (passed 2026-11-20).
- do:
  - Launch, home and Level with Mocap tracking the Platform. With the Platform still at Level, upload the whole ring with `ros2 service call /platform_imu/upload jugglebot_interfaces/srv/SetString "{data: \"{}\"}"` and wait for the GUI IMU panel to leave "uploading"; note the wall-clock time.
  - Tilt the Platform about its X axis, hold still for 10 s, return to Level; then the same about Y. Upload the whole ring again within 30 s of the last move (the ring holds 32 s) and note the time. The Tilt command and its angle are not fixed yet (2026-11-20): the Session preparing the sheet names them and rehearses them before this row goes on a sheet.
  - Rate ramp left from Rung P4: upload the whole ring Armed at 400, 800 and 1600 frames/s once the live pace hook for `imu_read_gap_10us` exists (not built, 2026-11-20); note each upload's wall-clock window.
- pass:
  - The Level capture and the Tilt captures are complete and CRC-checked, and the mount rotation fitted from them reproduces gravity in the Platform frame to within 1 degree at Level and at both Tilts (computed at the debrief).
  - A calibration file is written beside the other calibration artefacts.
- record:
  - The wall-clock time of each upload and each Tilt hold; the Tilt angles commanded.
  - For each ramp step the upload time; the counters are read from the bag at the debrief.
- minutes: 30

## Sitting result (2026-11-20 23:45)

- verdict: pass
- notes: setpoints_coalesced is now 10 (sheet 2026-11-20-robot-2, step 15) [derived: owner flipped inconclusive to pass. Replug returned to RUNNING at 22:52:41 on the same boot; i2c_errors and fifo_overflows 0 in all 1593 Armed status messages; four captures complete, ring CRC OK. Not met literally: forced gap unflown; Armed uploads 79-84 s with CAN1 at 74-77 %]

## Progress

- 2026-11-09: Night Shift 2026-11-09: feasibility and options written; awaiting the owner's answer
- 2026-11-11: Night Shift 2026-11-11: TKT-54 answered (ICM-42688-P, live low-rate plus buffered upload); decision recorded, re-rated owner hands for a day Session
- 2026-11-18: Bench proof of concept working (ICM-42688-P on a Teensy 4.0, 0 FIFO overflows and 0 I2C errors at 1 and 2 kHz); day Session started, no Platform Teensy firmware before the question round, calibration options and CAN transport design (detail below)
- 2026-11-18: Design round done: decisions agreed with the owner and written up in plans/active/platform-imu.md (phases P1 to P5, buffered capture only, no live stream).
- 2026-11-19: Owner decisions recorded in the plan: uploads run while Armed between Attempts (the Jetson gates; the bridge clamps the READ gap), target under 60 s for the full window, P4 gains a rate ramp.
- 2026-11-19: FW 11 written and audited on branch imu-p2, awaiting the gate and the flash.
- 2026-11-20: Rung P4 at rest done on FW 12 (identities, tilt, fan, status counters, first capture and upload: 32000 samples, CRC OK, |a| 1.0004 g, 56.25 s); open: capture header temperature reads -128, GYRO_ACCEL_CONFIG0 reads 0x00 (ODR/2, ODR/4 code unverified); the Sitting section now lists only the Armed checks (logbook 2026-11-20-platform-imu-status-counters-and-first-capture)
- 2026-11-20: Correction to the line above: the header temperature -128 was a valid 24.03 C (TEMP_DATA counts); the json now carries temp_c_start and temp_c_freeze. The header GYRO_ACCEL_CONFIG0 is the firmware constant, not a chip read-back. Owner decided the UI filter stays at ODR/2 (logbook 2026-11-20-platform-imu-capture-header-temperature).
- 2026-11-20: Rung P4 passed at the Sitting of 2026-11-20 (owner): replug recovered without a power cycle, i2c_errors and fifo_overflows 0 Armed through three oval Attempts, four complete captures. Residue: forced gap unflown, Armed uploads 79-84 s (IDEA-25), rate ramp. Next: Rung P5 mount calibration.
