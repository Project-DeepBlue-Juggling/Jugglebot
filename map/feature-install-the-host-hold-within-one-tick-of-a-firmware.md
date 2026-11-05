---
id: JB-167
kind: feature
title: Install the host hold within one tick of a firmware scheduler STOP so a guard latch becomes a recoverable LANE_REFUSED
status: inbox
created: '2026-11-04'
updated: '2026-11-05T12:29:17Z'
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

## Options (2026-11-05)

Where the host learns of a STOP today: `skill_node` reads `sched_refused` only from the bridge's 10 Hz `link_status` diagnostic (`_on_link_status`, `_hand_lane_refused`), so a STOP can sit up to ~100 ms plus a 25 ms tick before `_on_tick`'s LANE_REFUSED branch installs the hold.

1. Probe first: measure STOP to hold latency per STOP from the launch logs and bags already flown (the Item's own Next). No code on the robot path. xs.
2. Publish `link_status` on change as well as at 10 Hz, when `sched_stops` or `sched_refused` moves, so `skill_node` acts within one tick. Same message type, so no interface rebuild. s, needs a Sitting.
3. Raise `link_status` to 40 Hz. One parameter, but 4x the diagnostic traffic on a loaded box. xs, needs a Sitting.
4. Leave it: the Teensy guard stays the authority and a latch costs one operator CLEAR_ERRORS.

Recommendation: 1, then 2 if the probe shows the latch would have been a LANE_REFUSED inside one tick.

## Progress

- 2026-11-05: Probe done 2026-11-05 (option 1): STOP -> hold 130-340 / 300 / 320 / 275 ms; WARN lag from 10 Hz link_status 0-130 ms, then 110-210 ms END -> hold installed. Guard latched before the hold in 2 of 4 (L1399 by 170 ms, L2361 by 51 ms). Publishing on change helps; the END -> install leg is the larger term.
