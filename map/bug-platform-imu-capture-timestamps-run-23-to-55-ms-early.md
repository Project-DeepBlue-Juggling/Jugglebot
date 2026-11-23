---
id: BUG-132
kind: bug
title: Platform IMU capture timestamps run 23 to 55 ms early against Mocap and the encoders
status: in_progress
created: '2026-11-23'
updated: '2026-11-23T11:18:11Z'
owners: []
priority: medium
effort: s
hardware: robot
owner: hands
radius: cross-cutting
diagnosis: confirmed
area: platform-teensy
blocked_by: []
links:
- logbook/2026-11-23-platform-ring-identification-from-the-imu.md
- logbook/2026-11-23-platform-imu-clock-early-rx-ring-leak.md
filed_by: claude
night_skip: sitting
night_skip_on: '2026-11-24'
night_skip_note: FW 15 flashed and checked at rest; the Armed check is the Item's Sitting section (one Attempt, then the round-trip probe).
---

Found offline on 2026-11-23 (logbook 2026-11-23-platform-ring-identification-from-the-imu), diagnosed the same day (logbook 2026-11-23-platform-imu-clock-early-rx-ring-leak).

## Cause

- The Platform firmware built against the stock FlexCAN_T4. Its receive ring loses count of frames (an `_available` race), so stranded frames turn the ring into a delay line.
- The 0x7DD time sync was stamped at dispatch, so the Platform wall clock ran early by the queue delay, and every command to the Platform arrived late by the same delay. The FREEZE stamp cancels the error, which hid it.
- On 2026-11-22 the delay was 37 to 49 ms and the IMU read 50 to 56 ms early. On the short-uptime days (2026-11-20, 2026-11-23) the queue share is under 4 ms and the remaining 14 to 22 ms is late reference stamps (BUG-134).
- Measured on 2026-11-23 with FW 14: a Platform-answered request took 45 ms more than a bridge-only one (about 95 stranded frames).

## Fix

- Platform FW 15 builds against the can-bridge's patched library, stamps 0x7DD at reception in the interrupt, and estimates the offset from the least-delayed beacon. Header fields `rx_ring_leak_max`, `rx_ring_depth_max` and `sync_rx_delay_ms` expose the receive path. Built and tested, not flashed.
- Related Items: BUG-133 (Ball Butler), BUG-136 (Catching cone), BUG-134 (reference stamps), BUG-135 (receive queue overflow).

Against Mocap as stamped today expect the IMU about 13 ms early in any comparison: that is the Mocap stamp (BUG-134, closed; IDEA-28), not a failure of this Item.

## Sitting

- setup: level, armed, bb-fed
- why: Prove on the flashed board, after Armed traffic, that the Platform receive ring no longer delays frames and that the IMU capture clock is right.
- do:
  - Fly one Attempt: `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: oval_1ball_fed, apex_m: 1.00, separation_mm: 150.0, dwell_s: 0.50, num_cycles: 8}" --feedback`. Wait for the "Platform IMU upload finished" line in the launch log.
  - At the end of the Sitting, with the launch stopped and the Platform Teensy still powered, from the repo root with the venv active: `python tools/probes/platform_rtt_probe.py`.
- pass: The upload line reads "complete yes, crc ok". The probe exits 0 (floor excess at most 5 ms; FW 14 read 37 ms). At the debrief the capture json shows `header.fw_version` 15, `flags.clock_suspect` false and `header.rx_ring_leak_max` 0, and `python tools/probes/imu_clock_check.py --capture <stem>` prints PASS.
- record: The probe's printed line and the capture stem. The header values and the clock check are read at the debrief.
- minutes: 10

## Progress

- 2026-11-23: Cause found: Platform RX ring leak delays the 0x7DD beacon; FW 15 fix built, not flashed (detail in logbook)
- 2026-11-23: FW 15 flashed 2026-11-23 18:37 and checked at rest (leak 0, probe floor 1.5 ms); the Armed check is the Sitting section, rewritten so the Sitting index parses it
