---
id: BUG-151
kind: bug
title: Legs trail the command by 13 to 15 ms on 2026-11-27, against 6 to 11 ms on 11-20 and 11-23
status: inbox
created: '2026-11-27'
updated: '2026-11-27T06:41:07Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: subsystem
diagnosis: unknown
area: legs
blocked_by: []
links:
- bug-platform-imu-capture-timestamps-run-23-to-55-ms-early
- feature-can-bridge-fw-53-a-leg-leaves-closed-loop-only-at-stow-i-fw
filed_by: claude
---

Found while closing BUG-132. tools/probes/imu_clock_check.py reports the IMU's lag against the encoders (lag1) and against the command (lag2); their difference is how far the Legs trail the command.

- 2026-11-27, can-bridge FW 53 (captures 33D0, 33D1, 33D2, ovals): 15.2, 12.8 and 12.6 ms.
- 2026-11-20 and 11-23, can-bridge FW 49 (captures AEDF, AEE0, AEE1, 4AFA): 7.4, 6.4, 7.2 and 11.0 ms.

Seven captures cannot separate the causes: the firmware changes between FW 49 and FW 53, the motion flown (4AFA is also an oval and reads 11.0), bridge uptime, or the measure itself (two cross-correlations subtracted).

## Wanted

- Measure the command-to-encoder lag directly from the bags (/leg_cmd_executed against /robot_state restamped to the bridge tick, the pairing imu_clock_check.py already does), per launch, for the Sittings of 2026-11-13 to 11-27, by Pattern and by firmware.
- If it moved with the firmware, find which of FW 50 to 53 added it; if it is the motion or the measure, close this.
