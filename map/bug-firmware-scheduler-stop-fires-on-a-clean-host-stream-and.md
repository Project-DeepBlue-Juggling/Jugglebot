---
id: JB-166
kind: bug
title: Firmware scheduler STOP fires on a clean host stream and ends the Attempt
status: inbox
created: '2026-11-04'
updated: '2026-11-05T12:29:17Z'
owners: []
priority: high
effort: m
hardware: robot
owner: none
radius: subsystem
diagnosis: unknown
area: can-bridge
blocked_by: []
links:
- logbook/2026-11-04-sitting-debrief.md
- bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms
filed_by: claude
---

Filed by the 2026-11-04 debrief of the 2026-11-03 22:29 Sitting (JB-162).

- Three of 24 oval goals ended on a can-bridge scheduler STOP (cover expired) while the host stream was clean: goal 3 GUARD_LATCHED (Hand MAX_DEVIATION -3.057 rev), goal 9 LANE_REFUSED, goal 15 LANE_REFUSED then Hand MAX_DEVIATION +7.660 rev.
- Forward gap 37-47 ms, loop gap <= 20 ms, emit max 34 ms (all under the 50 ms cover); bridge frame age max 24.8 ms; sched_expired/superseded/stale did not move.
- Same conclusion as the 12:30 Sitting (JB-149's host-starvation reading refuted twice).
- Next diagnostic: log fw_sched frame-age buckets and sched_expired per heartbeat at 10 Hz around STOPs, and the knot stamp lead (stamp - arrival) at the bridge for the 2 s before each STOP; a bench-hand run counting STOPs per hour with the bridge idle vs the CATCH solve running.

## Progress

- 2026-11-05: the 3 Hand-only STOPs are FW 37 C1 stops (sched_stops +1 = hand_c1_stops +1, legs never stopped), Hand frozen mid-upstroke at 52-100 rev/s, resume refused (8 rev/s tolerance). Drop counters 0, frame ages <= 27.6 ms, every numbered frame sent, no host stamp hole: one hand frame lost in the downlink is the only candidate. Stops 316-334 ms after first CATCH install.
