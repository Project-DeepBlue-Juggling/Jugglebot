---
title: "ICM-42688-P bench proof of concept: Teensy 4.0 firmware with a FIFO driver, and a Web Serial viewer (IDEA-20)"
type: feature
date: 2026-11-18
status: complete
phase: "IDEA-20 bench"
subsystem: tools
files_changed:
  - tools/probes/imu_bench/README.md
  - tools/probes/imu_bench/firmware/platformio.ini
  - tools/probes/imu_bench/firmware/src/icm42688.h
  - tools/probes/imu_bench/firmware/src/icm42688.cpp
  - tools/probes/imu_bench/firmware/src/main.cpp
  - tools/probes/imu_bench/viewer/index.html
  - logbook/INDEX.md
---

## What and why

IDEA-20 wants a Platform accelerometer for input shaping. Before any Platform Teensy firmware changes, this is a
standalone bench: a Teensy 4.0 reads an ICM-42688-P through its FIFO and streams text over USB serial; a single-file
browser page (Web Serial) plots accel and gyro, shows a 3D orientation and records CSV (`?demo=1` runs it with no
hardware). The aim is to validate the sensor and a transport-agnostic driver (it takes a `TwoWire&`) so the later
Platform Teensy Session lifts known-good code. Layout, wiring, protocol and checklist are in
`tools/probes/imu_bench/README.md`.

Facts worth keeping:
- **Pins.** The Platform Teensy firmware leaves pins 18/19 (I2C0) free: CAN1 uses 22/23, the SCL3300 uses SPI on
  10-13, and pin 2 is the Hand fan.
- **I2C, not SPI.** IDEA-20 assumed SPI. The bench uses I2C over Stemma QT for wiring ease (about 10 cm of cable on the
  Platform). SPI stays an option if 400 kHz / 1 MHz I2C proves too slow at 2 kHz.
- **Chip FIFO.** The Platform Teensy loop can stall about 100 ms in the SCL3300 retry path, so the driver drains the
  chip FIFO rather than polling data registers (a 2 kB FIFO holds about 60 ms at 2 kHz; longer stalls overflow, counted
  and flushed).
- **Two fixes from checking the code against datasheet DS-000347 rev 1.6.** FIFO_CONFIG1 needs
  FIFO_RESUME_PARTIAL_RD (0x4F, was 0x0F) for a FIFO read in several transactions; the FIFO timestamp tick is 32/30 us
  without an external clock (was assumed 1 us, a 6.7 percent rate error). The full table is in the README.

## Verification

- 2026-11-18, `cd tools/probes/imu_bench/firmware && pio run` (from the committed copy, fresh `.pio/`): **SUCCESS in
  6.70 s**, flash 39792 B code, RAM1 11360 B variables.
- 2026-11-18, owner on bench Teensy 4.0, viewer over Web Serial: the sensor answered and the orientation and plots
  tracked hand motion correctly by eye.
- 2026-11-18, direct serial capture from the bench Teensy (pyserial on COM4; commands `o`, `i`, `s`; 20 s per setting,
  then 15 s steady per rate): address 0x68, WHO_AM_I 0x47; **0 FIFO overflows, 0 I2C errors, 0 bad packets** at
  1 kHz / 400 kHz, 2 kHz / 400 kHz and 2 kHz / 1 MHz; sensor-timestamp rate **999.97 Hz** at 1 kHz and 1999.94 Hz at
  2 kHz, which confirms the 32/30 us tick on this unit; in the 15 s steady captures (29,888 samples at 2 kHz, 14,939
  at 1 kHz) the only sequence gap is the 1.5 s the host deliberately did not read after the rate command (counted by
  the firmware as `serial_drops`, by design). At rest, Z up, +-16 g / +-2000 dps: |a| = 1.004 to 1.005 g, gyro offset
  up to 0.5 dps per axis, accel noise 1.3 to 2.2 mg rms per axis, gyro 0.05 to 0.07 dps rms, 23.6 degC.

- Follow-up, same day: the bench firmware tree is a `platformio.ini`, so `test_every_firmware_tree_is_frozen` failed on
  `main` (seen in a peer session's `./run_tests.sh --full`, 1 failed). `tools/probes/imu_bench/firmware/*` is now on the
  Night Shift gate's frozen list (`tools/night_shift_gate.py` `FORBIDDEN_PATHS`, pinned in `test_night_shift_gate.py`);
  the viewer and README stay open. The commit message carries the gate run.

**Not verified:** |a| on the other five faces and a measured 90 degree rotation (one face only was held still);
behaviour on a moving Platform, with the real cable, or inside the Platform Teensy firmware (nothing there is
touched). Datasheet checks were read from a text mirror, not the PDF.
The breakout is undocumented (no schematic), so its pull-ups and CS strap are checked by multimeter per the README.
