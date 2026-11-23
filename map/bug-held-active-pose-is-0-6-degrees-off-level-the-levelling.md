---
id: BUG-130
kind: bug
title: 'Held Active pose is 0.6 degrees off Level: the levelling correction reaches Jog targets but not the hold'
status: in_progress
created: '2026-11-22'
updated: '2026-11-23T15:48:41Z'
owners: []
priority: medium
effort: m
hardware: robot
owner: none
radius: subsystem
diagnosis: confirmed
area: platform-teensy
blocked_by: []
links:
- map/idea-platform-accelerometer.md
filed_by: claude
night_skip: input-decision
night_skip_on: '2026-11-23'
night_skip_note: Builds the GUI not-Level indicator on option 2 and merges it if the gate says MERGE; option 1 closes BUG-130; 3 gets a held branch and a Sitting row.
night_skip_input: 'Pick option 1-4 in TKT-92: leave it, a GUI not-Level indicator, an automatic go_home after ACTIVATE, or ACTIVATE landing on Level.'
---

Found at the Sitting of 2026-11-22 by the Platform IMU mount fit (IDEA-20). After Level, the Active pose the Platform holds (ACTIVE:STANDBY and ACTIVE:TRAJECTORY at rest) is the uncorrected orientation: Mocap reads (-0.05, +0.61) deg, the IMU 0.598 deg from the levelled direction, and the Leg commands agree. Targets from the GUI Jog panel do carry the levelling correction, so Jog Home is Level and the held Active pose is not. Effective offset including the 0.38 deg trim: (-0.044, +0.574) deg.

The operator's working assumption (and the 2026-11-22 sheet's) was that the Active pose equals Level. Whether this is intended (the correction is applied only to planned motion) or a defect in the path that produces the held pose is not traced. If Patterns start from the uncorrected hold, the first move of every Attempt includes a 0.6 deg re-level.

Evidence: temp/debrief/2026-11-22/mountfit.md (captures 20261122T100747Z_572D and 20261122T100946Z_572E, bag 2026-11-22_21-03-42); logbook 2026-11-22-sitting-debrief.

## Options (2026-11-23)

The trace (commit d02b620d) found no code defect: ACTIVATE goes to the uncorrected neutral, the hold is seeded from the encoders (C-LEVEL-1), and every planned move is corrected. So the question is what the operator should see or get after ACTIVATE.

1. **Leave it as designed.** `levelling_frame.md` now says so; anything that needs Level follows `go_home`, a Jog target or an Attempt's pre-level REST. No code.
2. **A GUI not-Level indicator.** The GUI marks the Platform as not Level from ACTIVATE until the first planned move. Jetson and GUI only, no motion change; the Night Shift can build and test it offline.
3. **An automatic `go_home` after ACTIVATE.** trajectory_node issues the corrected `go_home` once ACTIVATE completes, so the held Active pose is Level. It adds an unrequested 0.6 degree profiled move at activation and changes flown behaviour, so it needs a Sitting.
4. **ACTIVATE lands on the corrected neutral.** The Jetson computes levelled leg targets and sends them with ACTIVATE. A firmware RPC change and a flash; the largest of the four.

Recommendation: 2 now (cheap, no motion); 3 as well if Level should be the default state after ACTIVATE. Not 4.

## Progress

- 2026-11-22: 2026-11-23: traced, designed behaviour, no code defect. ACTIVATE goes to the uncorrected neutral and the hold is encoder-seeded (levelling_frame.md C-LEVEL-1); every planned move is corrected, and an Attempt's pre-level REST re-levels. levelling_frame.md corrected. Open for the owner: a GUI not-Level indicator, or an automatic go_home after ACTIVATE.
