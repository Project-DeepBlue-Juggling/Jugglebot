---
id: BUG-35
kind: bug
title: mocap_node loses frames to queue full in bursts before the first dispatch (544 / 373 / 490 frames in the three 2026-10-28 Sittings)
status: inbox
created: '2026-10-28'
updated: '2026-11-01T17:51:41Z'
owners: []
priority: low
effort: m
hardware: robot
owner: hands
radius: subsystem
area: tracking
blocked_by: []
links:
- 2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented
parent: two-ball-oval
filed_by: claude (BUG-21 21:02 Sitting read-out)
---

Each 2026-10-28 Sitting lost mocap frames to the node queue (150 deep) in one or two bursts in the first ~90 s after launch and none during goals.

## Losses per Sitting

| Sitting | Frames lost |
|---|---|
| 09:49 | 490 |
| 14:29 | 373 |
| 21:02 | 544 at t+44..85 s, in two WARNs of 170 and 272 frames |

## What is known

- BUG-21 showed the starvation was a fellow victim of box load, now reduced, yet the pre-dispatch bursts persist, so something else stalls the consumer early (QTM reconnect at start, robot-frame fit, calibration solve thread, GUI connecting?).
- Flame graph pyspy_mocap_node.svg in temp/probes/profile_20261028-210241/ shows the 100 Hz publish timer at 54 % of samples.

## Priority

Low priority while no frames are lost in goals.
