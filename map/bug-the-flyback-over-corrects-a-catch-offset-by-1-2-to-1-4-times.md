---
id: JB-193
kind: bug
title: 'The flyback over-corrects a Catch offset by 1.2 to 1.4 times: each landing error returns at -0.3 to -0.45 on the next Throw'
status: inbox
created: '2026-11-06'
updated: '2026-11-06T06:13:45Z'
owners: []
priority: medium
hardware: none
blocked_by: []
links:
- logbook/2026-11-06-sitting-debrief-2.md
parent: learner-generalisation
filed_by: claude
---

In the one-ball goals of the P2 Sitting (2026-11-06) each landing error comes back on the next Throw at -0.3 to -0.45 of itself. It damps, so it is not an instability, but the cause is a flyback gain that is too strong, a planner matter and not a plant-model term.

## Findings
- The Cup releases from where it caught: commanded release position against the previous landing has slope 0.87 (r +0.93).
- The planner Banks to fly the offset back. Tilt against the previous landing: r -0.91, -0.21 mrad per mm. Ball lateral velocity against commanded tilt: 6.7 mm/s per mrad, against about 4.2 expected from the Hand speed.
- The flyback returns 1.19 mm per mm of offset against 0.87 mm of release offset, so the net is -0.32, which matches the measured lag-1 slope (-0.30; corpus one-ball x -0.45, y -0.24). The gain is about 1.2 to 1.4 times too strong.
- In Columns it acts at lag 2 (same Site): -0.12 and -0.18, including about -0.1 of demeaning bias.
- The lateral scatter is set by the Platform tilt at release (ball lateral velocity against tilt r 0.78 x, 0.62 y). Release-time error (+9 ms mean, sd 7.5 ms) has no effect on Apex.

## Wanted
- Find where the flyback gain comes from in the Banking and flyback planner, and whether it should be 0.87 instead of 1.19. A change touches the feedforward and timing path: analyse it as a control change and replay it offline first.

## Links
Evidence: `temp/probes/sitting_debrief_2026-11-06b/unit_transit.md` section 3.3, `unit_A_C.md`. Related: `logbook/2026-11-06-sitting-debrief-2.md`.
