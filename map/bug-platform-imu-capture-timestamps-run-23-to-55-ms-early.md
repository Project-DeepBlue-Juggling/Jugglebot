---
id: BUG-132
kind: bug
title: Platform IMU capture timestamps run 23 to 55 ms early against Mocap and the encoders
status: inbox
created: '2026-11-23'
updated: '2026-11-23T04:37:10Z'
owners: []
priority: medium
blocked_by: []
links:
- logbook/2026-11-23-platform-ring-identification-from-the-imu.md
filed_by: claude
---

Found offline on 2026-11-23 (logbook 2026-11-23-platform-ring-identification-from-the-imu).

- The IMU wall-clock stamps are early by 53 to 57 ms in the two captures of 2026-11-22 and by 22 to 24 ms in the capture of 2026-11-23.
- Measured by correlating the gyro with the Mocap and encoder orientation rate (r 0.95 to 0.997). Mocap and encoders agree with each other within 3 ms.
- Constant within a capture (under 0.5 ms/s), different by day: a per-boot or per-sync offset in the wall-time model is likely. Not diagnosed.
- Effect: the GUI IMU charts and Throw or Catch markers are misaligned by that amount; any analysis must estimate and remove the shift per capture.
- First step: compare the capture header's sync fields (freeze_wall_us, sync_innovation_us) across the three captures against the measured shifts.
