---
id: BUG-125
kind: bug
title: Hand drive MISSING_ESTIMATE at rest lost exactly five turns, ten minutes after a SPINOUT top-stop impact
status: inbox
created: '2026-11-20'
updated: '2026-11-20T15:50:15Z'
owners: []
priority: medium
effort: s
hardware: robot
owner: hands
radius: local
diagnosis: unknown
area: hand
blocked_by: []
links:
- logbook/2026-11-20-sitting-debrief.md
- bug-hand-overshoot-at-apex-1-35-saturated-the-stepped-brake-at
filed_by: claude
---

Sitting of 2026-11-20, launch 1 (`temp/logs/launch_2026-11-20_1.log:6915`, bag `2026-11-20_12-49-25`). At 13:32:16.53, with the Hand at rest at 5.40 rev between Attempts, the Hand drive raised MISSING_ESTIMATE; the position read NaN for about 50 ms and returned 5.00 turns lower. Ten minutes earlier (13:22:42) a SPINOUT had disarmed the drive during a post-release brake and the Hand coasted into the top stop at about 49 rev/s.

Whether that impact damaged the encoder or magnet is unknown. A 5.00-turn jump fits the drive re-seeding its multi-turn estimate from the single-turn onboard encoder (inference).

Next: watch for a second MISSING_ESTIMATE on this motor; if one occurs, inspect the encoder magnet and the drive's encoder error counters.
