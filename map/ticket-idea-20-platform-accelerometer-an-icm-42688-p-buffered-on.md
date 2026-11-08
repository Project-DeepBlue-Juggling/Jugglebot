---
id: TKT-54
kind: ticket
title: 'IDEA-20 Platform accelerometer: an ICM-42688-P buffered on the Platform Teensy, streamed live, an ADXL355, or read the SCL3300 first?'
status: open
created: '2026-11-09'
updated: '2026-11-08T17:49:43Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: platform-teensy
blocked_by: []
links: []
parent: idea-platform-accelerometer
type: question
---

You asked what a Platform accelerometer would give, whether the pins exist and which sensor. Pins are not the constraint: the Platform Teensy has SPI up already for the SCL3300, so one more chip-select is enough. The Item's `## Feasibility` has the detail; part figures are from memory and need a datasheet check.

## Options

1. ICM-42688-P on the 2020 extrusion, captured to a buffer on the Platform Teensy and uploaded after the Attempt. No added CAN load in flight.
2. The same sensor streamed live at a reduced rate over CAN. Needs a bus-load check first.
3. An ADXL355 instead, if noise floor matters more than bandwidth.
4. First read the SCL3300 at its fastest setting and decide after that capture. No new hardware.

Recommendation: 4 as a quick look, then 1.

## What each answer unblocks

- 1 or 3: a parts order from you, then firmware and a capture tool on a held branch.
- 2: a bus-load measurement Sitting before any firmware.
- 4: a Sitting row, which the next night can write.
