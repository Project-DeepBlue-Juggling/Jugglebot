---
id: JB-167
kind: feature
title: Install the host hold within one tick of a firmware scheduler STOP so a guard latch becomes a recoverable LANE_REFUSED
status: inbox
created: '2026-11-04'
updated: '2026-11-03T21:55:21Z'
owners: []
priority: medium
effort: m
hardware: robot
owner: decision
radius: subsystem
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-04-sitting-debrief.md
- bug-teensy-guard-max-deviation-latched-on-the-hand-8-4-rev-mid
filed_by: claude
---

Filed by the 2026-11-04 debrief (2026-11-03 Sitting).

- After a firmware STOP the host learns of it from the 10 Hz /link_status poll (~100 ms), then installs the hold 130-214 ms after the skill END; goal 15's Hand guard tripped MAX_DEVIATION 68 ms after the END (STOP ~022.74, END 022.839, trip 022.907, hold 023.045).
- Candidate: read sched_stops/sched_refused on the faster telemetry path and install the hold in the same tick (skill_node _on_tick LANE_REFUSED branch / force_hold path).
- Converts goal-3/15-style guard latches (FAULT, operator CLEAR_ERRORS) into goal-9-style LANE_REFUSED with auto-recovery; does not remove the Attempt end.
- Owner decision: touches the hold path next to the Teensy guard (the guard stays the authority).
- Probe STOP->hold latency first.