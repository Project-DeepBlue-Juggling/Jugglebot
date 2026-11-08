---
id: BUG-74
kind: bug
title: 'Mid-air ball collisions end half of two-ball Columns Attempts at Apex 1.00: closest approach about 70 mm at Sep. 100, 125 and 135'
status: in_progress
created: '2026-11-06'
updated: '2026-11-08T00:13:25Z'
owners: []
priority: medium
effort: l
hardware: robot
owner: decision
radius: cross-cutting
diagnosis: hypothesis
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-06-sitting-debrief.md
- logbook/2026-11-06-sitting-debrief-2.md
- logbook/2026-11-06-sitting-debrief-3.md
filed_by: claude
---

At the Sitting of 2026-11-06 four of eight two-ball Columns Attempts (Apex 1.00, Dwell 0.27) ended on a mid-air collision between the two balls; a fifth early end is unclassified. Collisions are now the main Attempt killer for Columns.

## Evidence
- Bag `2026-11-06_13-19-19`, `/balls`: in Attempts 1 and 3 (Sep. 100), 5 (Sep. 125) and 7 (Sep. 135) the two tracked balls came within 65, 71, 76 and 73 mm centre to centre (ball diameter 74 mm) near the Apex, and both tracks show a lateral velocity jump of 350 to 540 mm/s at that instant.
- The two completed Attempts (20/20, Sep. 125 and 135) had a closest approach of 92 and 107 mm and no jump above 30 mm/s.
- Closest approach was about 70 mm at every separation, so the wider gap did not show up in these counts (3, 2 and 2 Attempts: very weak evidence either way). The colliding flights left a straight line by 60 to 78 mm laterally.
- Lateral landing error per separation was 29 to 32 mm RMS. None of the ends involved the admissibility check, the nomination or the Learner at its limit.

## Wanted
- Know what sets the lateral drift of a flight (release velocity error, Platform ring, tilt at release) well enough to say which separation gives a usable collision rate at Apex 1.00, or to reduce the drift. P2 of plan `learner-generalisation.md` (the characterisation Sitting) records the release telemetry this needs.

## Progress

- 2026-11-06: 2026-11-06 afternoon: collisions confirmed again (closest approach 70 to 77 mm, both balls kink) in 8 of 23 MISSED_CATCH ends over 35 endurance goals (Sep. 125 to 133, Apex 0.95 to 1.05) and 2 of 5 at the P2 Sitting (Sep.
- 2026-11-06: 120, pair turned 90 degrees). About 1.2 collision ends per 100 Throws; Sep. 133 was no better than 125 (small n). 5 of 7 collision-like events at P2 did not end the Attempt.
- 2026-11-06: A cause candidate, measured: two-ball Throws land pulled toward the partner Site by 0.18 of the Site coordinate (11 mm at Sep. 120), the same along x and along a pair turned 90 degrees, so the two columns lean together; one-ball Throws show none. Platform tilt at release sets the lateral scatter (r 0.78 x, 0.62 y in one-ball goals).
- 2026-11-06: Evidence: `temp/probes/sitting_debrief_2026-11-06b/unit_B_D.md`, `unit_A_C.md`.
- 2026-11-06: 2026-11-06 20:20 Sitting (Apex 1.00, Sep. 120, Hand at 9 ms): 2 of 4 ends are collisions, goals 1 and 3, both Ball 1: closest approach 71 and 72 mm, both balls kink 257 to 318 mm/s at z about 1390, 1.25 to 1.33 s before the end. Aim asked +160 to +191 mm, +10 applied; no authority would have saved them. Same signature at every Sep. flown so far.
- 2026-11-08: 2026-11-07: 7 collisions in about 300 Throws (2.3 per 100), closest approach 31-74 mm with both balls kinking 200-1550 mm/s, including 2 in 13 Throws at high-low Sep. 100 and 2 at turned 90 deg.
