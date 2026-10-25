---
title: "Wave Camera GPIO findings: Pin 4 is a free-running 30.000 Hz tick, the first tick after the GPI2 edge starts the clip, Pin 5 inert and dropped; ESP32 firmware and docs moved into Jugglebot"
type: investigation
date: 2026-10-25
status: in-progress
phase: "wave-camera-can3-integration - Phase 1 (adapter firmware) bench"
related_plan: wave-camera-can3-integration.md
subsystem: can
tags:
  - docs
  - testing
files_changed:
  - ros_ws/src/jugglebot/WaveCamera_ESP_code/
  - ros_ws/docs/wave_camera_gpio_explorer.md
  - ros_ws/docs/wave_camera_can_adapter.md
  - ros_ws/docs/wave_camera_findings.md
  - plans/active/PROMPT-wave-camera-jetson-bringup.md
  - .gitignore
---

# Wave Camera GPIO findings (short form)

## Summary

Bench results from the ESP32 GPIO explorer on the Freefly Wave (fw 1.4.1),
2026-10-19 to 2026-10-25. The full running log, with every table and the bench
log behind each claim, is [ros_ws/docs/wave_camera_findings.md](../ros_ws/docs/wave_camera_findings.md);
this entry is the headline only. The firmware, its tests and the bench logs
moved into Jugglebot the same day as `ros_ws/src/jugglebot/WaveCamera_ESP_code/`
(from a standalone repo with no history).

## Symptoms

Which Wave pin does what, and can a GPI2 edge be tied to a robot wall-clock
instant good to 1 ms?

## Diagnosis

- Pin 3 is a 115200 8-N-1 UART out: `Created new clip.\r\n` (19 bytes) at every
  record start, nothing at stop; about 33 boot-console lines while the camera boots.
- Pin 4 is a free-running 30.000 Hz tick (period 33334.3 us, stdev about 1 us),
  independent of recording and of an HDMI sink.
- The first Pin 4 tick after the GPI2 falling edge starts the clip; the clip
  message follows that tick by about 15-43 ms depending on camera condition.
- Pin 5 (GPI1) showed no response to anything and was dropped (repin
  2026-10-23: GPI2 on GPIO26, Pin 3 on GPIO32, Pin 4 on GPIO27).

## Fix

Level-mode GPI2, start stamped on the tick, not on the message (the
[CAN adapter design](../ros_ws/docs/wave_camera_can_adapter.md)). Repin and
CAN state machine verified 2026-10-25 with simulated intent; no bridge yet.

## Outcome

Verification (2026-10-25, Windows host, from `ros_ws/src/jugglebot/WaveCamera_ESP_code/`):
`pio run -e esp32dev` SUCCESS, 0 warnings; `pio test -e native` 80 test cases:
80 succeeded. `./run_tests.sh`: gate not run on this host; run on the Jetson.

## Withdrawn claims

- "latency + tick phase is constant": the 2026-10-19 CAN-phase series did not
  fit it (54-76 ms); clip-message latency shifts with camera condition.

## Open Questions

Which recorded frame is the start tick (needs the clapboard flash); the
on-robot bring-up rows of the design's s10.
