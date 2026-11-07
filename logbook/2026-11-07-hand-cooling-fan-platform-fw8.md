---
title: "Hand cooling fan on the Platform Teensy (FW 8): pin 2, on above 35 C and off below 34 C from the Hand ODrive's temperature broadcast; built and tested, not flashed"
type: feature
date: 2026-11-07
status: complete
phase: "hand cooling fan"
subsystem: firmware
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_platform/hand_fan.h
  - ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino
  - config/hardware_config.yaml
  - teensy_link/rpc_args.py
  - tests/firmware/native/test_platform_hand_fan.cpp
  - tests/firmware/native/build.py
  - tests/firmware/test_native_firmware.py
  - tests/firmware/test_platform_hand_fan_xref.py
---

# Hand cooling fan on the Platform Teensy (FW 8)

## What changed

The Platform Teensy now switches the Hand fan through an N-channel low-side MOSFET with a gate pull-down, on **pin 2** (HIGH = on). It reads the Hand ODrive's cyclic Get_Temperature frame (`0xD5`, motor thermistor at byte 4; `temperature_msg_rate_ms: 100` in `odrive_pro_hand_config.json`), so it needs no ROS2 and no Jetson. On above 35 C, off below 34 C; off after 5 s with no usable reading. Remote frames, short frames and non-finite readings count as no reading. Values live in `teensy_operational`. CAN is receive-only here: the TX set and I-FW-15 are unchanged. The pin is driven low before the inclinometer wait in `setup()`, which never returns if the SCL3300 is absent.

**Pin choice.** Pins 2-5 have no UART, I2C, SPI or analog role. Pins 10-13 are the SCL3300's SPI, 22/23 are this board's CAN. Pins 6 and 9 are on the chip's boot-configuration pads (from memory of the reference manual), a poor place for a pull resistor.

**Not flashed (owner's call, flash later).** `PLATFORM_FW_VERSION_EXPECTED` is 8, so launches log `PLATFORM_FW_CHECK: FAIL` until the board is flashed; the check warns and refuses nothing. The MOSFET's 3.3 V gate drive is unchecked: no part number yet.

## Discussion

**The threshold evidence.** The first read of the two latest bags (Hand motor peaks 26.8 and 28.3 C) suggested 35 C might never trigger. That was withdrawn: the owner has run the fan continuously since the afternoon of 2026-11-05, and those two bags were simply light. Across 36 bags from 2026-10-25 to 2026-11-07, ordinary running peaks at 41.0 C, so 35 C does trigger. Peaks passed 35 C in 6 of 14 fan-off bags and 4 of 8 fan-on bags, and 45 C in none. Margin is 19 C to the ODrive's 60 C limit. A 35/34 C fan would have run in 10 of the 36 bags for 12 to 815 s each, simulated on recorded temperatures. The only excursion above 45 C is the 2026-11-02 16:52 stall (23 to 95.7 C in about 8 s), which no fan would help.

**The fan shows no measurable effect**, medians per bag, fan off (14) against fan on (8), Mann-Whitney: heating 2.69 against 2.54 C per 1000 A^2 s (p 0.87), cooling time constant 4.9 against 5.4 min (p 0.97), peak above Leg temperature 16.2 against 15.1 C (p 0.76). Columns at about 1.0 m Apex peak at 37.2 and 40.8 C with it off, and 40.8, 40.9 and 41.0 C with it on. This is weak: the fan-on bags come from two days, room and duty differ, and an effect under roughly 10-20 % would not show. A model-free cool-down check found only 4 usable idle stretches, which settles nothing. It needs a controlled test before anyone concludes the fan works or doesn't. `iq_measured` stays frozen at its last value outside closed loop, so an iq^2 integral must be masked to state 8 (unmasked it read up to 10x high). Probe outputs: `temp/probes/hand_thermal_fan/` (not committed).

**The 1 C gap.** Measured thermistor noise is far below 1 C (largest step 0.15 C on 2026-11-06), so noise does not need a wider gap. A long Attempt hovering near 35 C still gets real restarts: 2026-11-03 12:30 simulates 8 starts in 35 minutes at 35/34 C, 5 at 35/33 C. Restarts are harmless to a fan, and the owner kept 35/34 C.

**Off on lost signal.** The owner chose off after 5 s over fail-on. No frames usually means the Hand ODrive is not driving the motor, and its own thermal limits do not depend on this fan or bus.

## Verification

Gate (`./run_tests.sh -q`, run 2026-11-07, on this tree before this section was filled in): **RESULT: PASS**, parallel phase rc=0 in 589 s, serial phase rc=0 in 32 s; 9069 passed and 62 skipped by counting the progress characters, because the doubled quiet flag suppressed pytest's own count line.

Fan tests inside it: `tests/firmware/native/test_platform_hand_fan.cpp` (8 cases, 27 assertions) and `tests/firmware/test_platform_hand_fan_xref.py` (7).

After filling this in: `pytest -q tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py` (run 2026-11-07), the two tests that read this file.

Image: `pio run -e teensy40` (2026-11-07) SUCCESS, with `handFan` and `updateHandFan()` in the ELF and pin 2 in its disassembly. Not run on hardware.
