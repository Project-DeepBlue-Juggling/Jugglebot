---
id: BUG-132
kind: bug
title: Platform IMU capture timestamps run 23 to 55 ms early against Mocap and the encoders
status: complete
created: '2026-11-23'
updated: '2026-11-27T06:40:53Z'
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
- logbook/2026-11-27-imu-clock-check-lag2-band-15-ms.md
- logbook/2026-11-27-sitting-debrief.md
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

## Sitting result (2026-11-27 15:08)

- verdict: fail
- notes (split into bullets):
  - I got: (venv) jetson:/home/jetson/Desktop/Jugglebot/ (main) $ python tools/probes/platform_rtt_probe.py [14:30:16] platform_rtt_probe: floor excess 1.72 ms (limit 5.0), median excess 5.72 ms; wrote /home/jetson/Desktop/Jugglebot/temp/probes/platform_rtt_20261127T033126Z.json (sheet 2026-11-27-robot, step 40)
  - [derived: Upload line complete yes, crc ok; probe floor excess 1.72 ms (limit 5.0); capture 33D0 header fw_version 15, clock_suspect false, rx_ring_leak_max 0. But imu_clock_check.py on 33D0 prints FAIL: lag2 12.0 ms outside 0 to 10 ms (33D1 passes at 9.0 ms, 33D2 fails at 10.6 ms; FW 14 read -4 to -43 ms).]

## Progress

- 2026-11-23: Cause found: Platform RX ring leak delays the 0x7DD beacon; FW 15 fix built, not flashed (detail in logbook)
- 2026-11-23: FW 15 flashed 2026-11-23 18:37 and checked at rest (leak 0, probe floor 1.5 ms); the Armed check is the Sitting section, rewritten so the Sitting index parses it
- 2026-11-27: Failed at the Sitting of 2026-11-27 on the clock check only: lag 12.0, 9.0 and 10.6 ms on three FW 15 captures against the 0 to 10 ms band (FW 14 read -4 to -43 ms). Probe floor 1.72 ms, header fw 15, leak 0, clock_suspect false all pass. Owner decision: widen the band to about 0 to 15 ms, or chase the last 10 ms.
- 2026-11-27: Closed 2026-11-27 on the owner's decision to widen the clock check's command-lag band to 15 ms: that lag also holds how far the Legs trail the command. The clock measure (IMU against encoders) read -3.2, -3.8 and -2.0 ms on FW 15, as before the defect; all three captures now print PASS. Probe floor 1.72 ms, leak 0.
