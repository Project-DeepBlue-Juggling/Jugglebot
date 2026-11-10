---
id: IDEA-22
kind: idea
title: 'Turned Columns at 90 deg: tilt the Platform back (+x) at the Catch so the ball stops hitting the Platform top'
status: inbox
created: '2026-11-10'
updated: '2026-11-10T12:41:43Z'
owners: []
priority: medium
effort: m
hardware: robot
owner: decision
radius: subsystem
area: skill-stack
blocked_by: []
links:
- idea-jp-10-turned-columns-the-columns-pair-at-fixed-angles-about
- bug-ball-2-s-first-throw-after-the-feed-catch-lands-about-180-mm
parent: pattern-progression-columns-to-centre-and-orbit
filed_by: claude (sitting debrief 2026-11-10)
---

Owner, Sitting of 2026-11-10 (sheet step 8, IDEA-11): "45 deg passed perfectly, but 90 deg once again didn't succeed. I think we need to make the 90 deg version tilt the Platform slightly back (+ve x) when catching, to stop the ball hitting the top of the Platform so often."

Evidence the same night (launch log 2026-11-10-20-11-22, bag ~/Desktop/rosbags/2026-11-10_20-11-22): 4 Attempts at 90 deg, 8/8, 3/3, 3/3, 3/3, all MISSED_CATCH on an empty Cup; three ended at Ball 2's first Catch after the Feed with the Tracker fit 47, 26 and 14 mm outward along -y and 9-23 mm in -x. The 2026-11-03 and 2026-11-07 notes on IDEA-11 report the same Platform-top hits at 90 deg only.

Questions before software: is the Catch tilt a per-Site constant (a Catch plane tilted about the pair axis) or does it follow the arrival direction; how many degrees clear the Platform top at a 125 mm separation; does the pose planner's admissible box at 90 deg (Apex 0.875-0.975) survive the tilt at the Sitting limits (a nominate probe answers it offline).
