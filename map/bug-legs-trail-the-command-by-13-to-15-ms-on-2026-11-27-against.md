---
id: BUG-151
kind: bug
title: Legs trail the command by 13 to 15 ms on 2026-11-27, against 6 to 11 ms on 11-20 and 11-23
status: complete
created: '2026-11-27'
updated: '2026-11-27T07:06:39Z'
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
- logbook/2026-11-27-imu-clock-check-lag2-band-15-ms.md
filed_by: claude
---

Found while closing BUG-132. tools/probes/imu_clock_check.py reports the IMU's lag against the encoders (lag1) and against the command (lag2); their difference is how far the Legs trail the command.

- 2026-11-27, can-bridge FW 53 (captures 33D0, 33D1, 33D2, ovals): 15.2, 12.8 and 12.6 ms.
- 2026-11-20 and 11-23, can-bridge FW 49 (captures AEDF, AEE0, AEE1, 4AFA): 7.4, 6.4, 7.2 and 11.0 ms.

Seven captures cannot separate the causes: the firmware changes between FW 49 and FW 53, the motion flown (4AFA is also an oval and reads 11.0), bridge uptime, or the measure itself (two cross-correlations subtracted).

## Wanted

- Measure the command-to-encoder lag directly from the bags (/leg_cmd_executed against /robot_state restamped to the bridge tick, the pairing imu_clock_check.py already does), per launch, for the Sittings of 2026-11-13 to 11-27, by Pattern and by firmware.
- If it moved with the firmware, find which of FW 50 to 53 added it; if it is the motion or the measure, close this.

## Progress

- 2026-11-27: Measured the Leg lag directly from the bags, commanded position against encoder position, no IMU. With the same method on both sides, FW 53 on 11-27 reads 1.4 ms less than the earlier Sittings (10 oval Attempts against 22). The Legs are not later than before.
- 2026-11-27: Bags before 23 Nov 21:52 have no tick stamp on the encoder messages, so the tools pair each one with the nearest command message. About a quarter get a stamp one or two ticks early, which makes the Legs look about 5.6 ms quicker than they are. The 11-27 bags use the true stamp; that is the whole apparent rise.
- 2026-11-27: Closed as a measurement artefact, 2026-11-27. The true Leg lag is about 13 ms (Leg 1 about 10, Leg 0 about 15), the same in every launch on 11-27; tracking error unchanged at about 1 mm rms and 2 to 5 mm peak per Leg. The two probe docstrings are corrected. The clock check's 15 ms band stands: a healthy reading is 9 to 12 ms.
