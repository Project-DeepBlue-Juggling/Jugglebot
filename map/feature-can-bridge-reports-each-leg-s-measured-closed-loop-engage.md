---
id: FEAT-70
kind: feature
title: can-bridge reports each Leg's measured Closed-loop engage time
status: inbox
created: '2026-11-27'
updated: '2026-11-27T04:12:42Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: subsystem
area: can-bridge
blocked_by: []
links:
- feature-can-bridge-fw-53-a-leg-leaves-closed-loop-only-at-stow-i-fw
filed_by: claude
---

FEAT-69's pass line asks that every Leg confirms Closed-loop in under 190 ms. At the Sitting of 2026-11-27 that clause could not be judged: the bag holds no stamp for when the bridge sends the Closed-loop command and none for a Leg's first Closed-loop heartbeat. The host-side figure (orchestrator command to all six Legs reading Closed-loop on /robot_state, 10 ms samples) was 266 to 389 ms and includes host latency.

The firmware already waits for each Leg's confirmation (400 ms bound), so it knows the number.

## Wanted

- The bridge reports, per ladder, each Leg's time from its Closed-loop command to its first Closed-loop heartbeat (a field on the ladder's reply or on the status stream), and the host logs one line per ladder and publishes it where the bag records it.
- Then FEAT-69's engage clause is read from the launch log of any Sitting.
