---
id: IDEA-20
kind: idea
title: Platform Accelerometer?
status: backlog
created: '2026-11-08'
updated: '2026-11-08T17:49:41Z'
owners: []
priority: medium
effort: m
hardware: robot
owner: decision
radius: subsystem
area: platform-teensy
blocked_by: []
links: []
filed_by: harrison
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

## Progress

- 2026-11-09: Night Shift 2026-11-09: feasibility and options written; awaiting the owner's answer
