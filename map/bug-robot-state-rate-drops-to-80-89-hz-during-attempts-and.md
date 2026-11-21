---
id: BUG-112
kind: bug
title: /robot_state rate drops to 80-89 Hz during Attempts and drifts down over a launch
status: inbox
created: '2026-11-18'
updated: '2026-11-18T15:51:11Z'
owners: []
priority: medium
effort: m
hardware: robot
owner: hands
radius: subsystem
diagnosis: unknown
area: can-bridge
blocked_by: []
links:
- logbook/2026-11-18-sitting-debrief.md
filed_by: sitting-debrief 2026-11-18
night_skip: sitting
night_skip_on: '2026-11-22'
night_skip_note: Diagnosis unknown; the next step is per-thread CPU of teensy_bridge_node in a launch, which no bag records.
---

Sitting of 2026-11-18 (operator note on step 41; bags 2026-11-18_*). Related: BUG-98.

/robot_state (nominal about 98-100 Hz) dips during Attempts on every launch, main included: in-Attempt median 80-89 Hz against 92-98 outside; lowest 5 s bin 80.8 Hz (launch 1), 71.2 (launch 2), 70.2 (launch 4). Launch 3 also decayed from 98 to about 80 Hz over the launch, including about 3.5 min idle after 11:53:30, and launch 4 trends at -1.5 Hz/min. The operator reading of 45 Hz was the ros2 topic hz terminal, not the publisher: no bag bin is below 70 Hz.

Not separated: box load, bridge uptime (about 120 h), and the leftover ros2 topic hz subscriber. Stamps and log time agree, so the publisher itself slows.

Next step (no hardware needed first): per-thread CPU of teensy_bridge_node during Columns from an existing bag or a bench launch; then one launch with no extra subscriber and a fresh bridge boot.
