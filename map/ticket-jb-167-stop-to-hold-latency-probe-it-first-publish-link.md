---
id: JB-171
kind: ticket
title: 'JB-167 STOP to hold latency: probe it first, publish link_status on change, raise it to 40 Hz, or leave the guard as the authority?'
status: open
created: '2026-11-05'
updated: '2026-11-04T17:51:17Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: subsystem
area: skill-stack
blocked_by: []
links: []
parent: feature-install-the-host-hold-within-one-tick-of-a-firmware
type: question
---

JB-167 asks to install the host hold within one tick of a firmware scheduler STOP, so a Hand guard latch (FAULT, CLEAR_ERRORS) becomes a recoverable LANE_REFUSED. Today `skill_node` learns of a STOP only from the 10 Hz `link_status` diagnostic, and on 2026-11-03 goal 15 the guard tripped 68 ms after the END while the hold landed 206 ms after it.

## Options
1. Probe first: measure STOP to hold latency per STOP from the logs and bags already flown. No robot-path code. xs.
2. Publish `link_status` on change too, when `sched_stops` or `sched_refused` moves, so the hold lands within one 25 ms tick. Same message type. s, needs a Sitting.
3. Raise `link_status` to 40 Hz. One parameter, 4x the diagnostic traffic. xs, needs a Sitting.
4. Leave it: the Teensy guard stays the authority.

Recommendation: 1, then 2.

## What each answer unblocks
- 1: a night unit that writes the latency table into JB-167.
- 2 or 3: a unit with a Sitting row; the hold path sits next to the guard, so it flies before it merges.
- 4: JB-167 parks.
