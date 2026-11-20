---
id: IDEA-20
kind: idea
title: Platform Accelerometer?
status: in_progress
created: '2026-11-08'
updated: '2026-11-18T11:19:42Z'
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

- setup: armed
- why: finish the hardware proof of the Platform IMU path (plans/active/platform-imu.md P4): the at-rest checks are done, what remains needs the robot powered and Armed, and the upload rate must be chosen before any Attempt relies on it.
- do:
  - Upload again with a forced gap and check the re-request heals it.
  - With the IMU unplugged, power-cycle the Platform Teensy: it must boot, answer CAN and accept a firmware update. Then replug the IMU with the board still powered and check the status frame shows the IMU return to RUNNING without another power cycle.
  - Arm the robot and hold the Active pose. Read the baseline (bridge loop and emit-gap counters, RX queue depth, bus error counts, ODrive errors, Leg position error). Then upload at 400, 800, 1250 and 1600 frames/s, reading the same counters at each step.
  - Arm and run an Attempt-like move; compare the bus-load counters in `/profile` with an earlier bag. Check the automatic upload after that Attempt and after a Fault.
  - Soak Armed while watching `i2c_errors` and `fifo_overflows` in `platform_imu/status`.
- pass:
  - With the IMU unplugged the board boots, answers CAN and takes an update, and after the replug the status frame returns to RUNNING without a power cycle.
  - The forced gap is re-requested and healed and the upload passes CRC.
  - At the chosen pace every counter equals its baseline; the bus-load counters are unchanged during the Attempt-like move; the automatic uploads start and finish.
  - `i2c_errors` stays 0 through the Armed soak.
  - Not checked here: the tilt request's invalid reply with the inclinometer absent (the SCL3300 is not unplugged); P2's native tests cover it.
- record:
  - The forced-gap outcome; for each ramp step the counters and the upload time, and the operating rate chosen (the highest step with no change from baseline).
  - The `/profile` bus-load before and after; the soak length and the final `i2c_errors` and `fifo_overflows`.
- minutes: 60

## Progress

- 2026-11-09: Night Shift 2026-11-09: feasibility and options written; awaiting the owner's answer
- 2026-11-11: Night Shift 2026-11-11: TKT-54 answered (ICM-42688-P, live low-rate plus buffered upload); decision recorded, re-rated owner hands for a day Session
- 2026-11-18: Bench proof of concept working (ICM-42688-P on a Teensy 4.0, 0 FIFO overflows and 0 I2C errors at 1 and 2 kHz); day Session started, no Platform Teensy firmware before the question round, calibration options and CAN transport design (detail below)
- 2026-11-18: Design round done: decisions agreed with the owner and written up in plans/active/platform-imu.md (phases P1 to P5, buffered capture only, no live stream).
- 2026-11-19: Owner decisions recorded in the plan: uploads run while Armed between Attempts (the Jetson gates; the bridge clamps the READ gap), target under 60 s for the full window, P4 gains a rate ramp.
- 2026-11-19: FW 11 written and audited on branch imu-p2, awaiting the gate and the flash.
- 2026-11-20: Rung P4 at rest done on FW 12 (identities, tilt, fan, status counters, first capture and upload: 32000 samples, CRC OK, |a| 1.0004 g, 56.25 s); open: capture header temperature reads -128, GYRO_ACCEL_CONFIG0 reads 0x00 (ODR/2, ODR/4 code unverified); the Sitting section now lists only the Armed checks (logbook 2026-11-20-platform-imu-status-counters-and-first-capture)
- 2026-11-20: Correction to the line above: the header temperature -128 was a valid 24.03 C (TEMP_DATA counts); the json now carries temp_c_start and temp_c_freeze. The header GYRO_ACCEL_CONFIG0 is the firmware constant, not a chip read-back. Owner decided the UI filter stays at ODR/2 (logbook 2026-11-20-platform-imu-capture-header-temperature).
