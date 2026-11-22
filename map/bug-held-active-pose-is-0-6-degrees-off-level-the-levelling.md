---
id: BUG-130
kind: bug
title: 'Held Active pose is 0.6 degrees off Level: the levelling correction reaches Jog targets but not the hold'
status: in_progress
created: '2026-11-22'
updated: '2026-11-22T13:24:39Z'
owners: []
priority: medium
effort: s
hardware: none
owner: decision
radius: subsystem
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- map/idea-platform-accelerometer.md
filed_by: claude
---

Found at the Sitting of 2026-11-22 by the Platform IMU mount fit (IDEA-20). After Level, the Active pose the Platform holds (ACTIVE:STANDBY and ACTIVE:TRAJECTORY at rest) is the uncorrected orientation: Mocap reads (-0.05, +0.61) deg, the IMU 0.598 deg from the levelled direction, and the Leg commands agree. Targets from the GUI Jog panel do carry the levelling correction, so Jog Home is Level and the held Active pose is not. Effective offset including the 0.38 deg trim: (-0.044, +0.574) deg.

The operator's working assumption (and the 2026-11-22 sheet's) was that the Active pose equals Level. Whether this is intended (the correction is applied only to planned motion) or a defect in the path that produces the held pose is not traced. If Patterns start from the uncorrected hold, the first move of every Attempt includes a 0.6 deg re-level.

Evidence: temp/debrief/2026-11-22/mountfit.md (captures 20261122T100747Z_572D and 20261122T100946Z_572E, bag 2026-11-22_21-03-42); logbook 2026-11-22-sitting-debrief.

## Progress

- 2026-11-22: 2026-11-23: traced, designed behaviour, no code defect. ACTIVATE goes to the uncorrected neutral and the hold is encoder-seeded (levelling_frame.md C-LEVEL-1); every planned move is corrected, and an Attempt's pre-level REST re-levels. levelling_frame.md corrected. Open for the owner: a GUI not-Level indicator, or an automatic go_home after ACTIVATE.
