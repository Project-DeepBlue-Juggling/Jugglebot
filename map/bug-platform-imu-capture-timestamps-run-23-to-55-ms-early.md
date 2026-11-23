---
id: BUG-132
kind: bug
title: Platform IMU capture timestamps run 23 to 55 ms early against Mocap and the encoders
status: in_progress
created: '2026-11-23'
updated: '2026-11-23T07:12:36Z'
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

## Sitting

- setup: armed
- why: Prove on the flashed board that the Platform receive ring no longer delays frames and that the IMU capture clock is within spec.
- do:
  - Launch down, Platform up for at least 30 min including some Armed time: run `python tools/probes/platform_rtt_probe.py`. It must exit 0.
  - Fly one `oval_1ball_fed` Attempt at Dwell 0.50 with the automatic IMU capture and the bag recording.
  - Offline after the Sitting: run `python tools/probes/imu_clock_check.py --capture <stem>`.
- pass: The probe exits 0 (median excess of a Platform-answered request over a bridge-only one at most 5 ms; FW 14 read 44 ms on 2026-11-23). In the capture json `header.fw_version` is 15, `flags.clock_suspect` is false and `header.rx_ring_leak_max` is 0. The clock check prints PASS (IMU within 5 ms of the tick-restamped encoders, lagging the executed command by 0 to 10 ms).
- record: The probe medians, the capture stem, the four header values, the clock check output and the Platform uptime at the Attempt.
- minutes: 45

Against Mocap as stamped today expect the IMU about 14 to 18 ms early: that is the Mocap stamp, tracked on BUG-134, not a failure of this Item.

## Progress

- 2026-11-23: Cause found: Platform RX ring leak delays the 0x7DD beacon; FW 15 fix built, not flashed (detail in logbook)
