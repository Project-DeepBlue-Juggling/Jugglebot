---
id: BUG-84
kind: bug
title: Hand stalls at the 70 A clamp mid-dive, probably on a Ball under the Cup, and the next launch comes up slow
status: inbox
created: '2026-11-07'
updated: '2026-11-13T06:09:31Z'
owners: []
priority: medium
effort: m
hardware: robot
owner: hands
radius: local
diagnosis: hypothesis
area: hand
blocked_by: []
links:
- map/bug-hand-overshoot-at-apex-1-35-saturated-the-stepped-brake-at.md
filed_by: jugglebot-b1
---

Four times in the captures, the Hand stopped dead mid-dive and pressed down at the 70 A clamp for about 100 ms, with no fault. The owner believes a Ball was stuck under the Hand at the 2026-11-07 event, though not with certainty. For the three stalls of 2026-11-13 the cause is confirmed (see Progress): a Ball trapped under the Hand, seen in raw mocap markers and in the replay. The four earlier events remain unmeasured.

## Evidence

| when | stopped at (rev) | iq setpoint peak (A) | nearby in the log |
|---|---|---|---|
| 2026-11-06 15:18:04 | 2.48 | -69.9 | a Ball missed 0.69 s earlier |
| 2026-11-06 15:25:19 | 2.52 | -69.9 | Throw 27 missed 0.03 s earlier, Attempt ended in a mode change 1.5 s later |
| 2026-11-06 15:35:57 | 2.38 | -69.9 | Attempt ended on mode ERROR 0.9 s later |
| 2026-11-07 16:17:19 | 1.97 | -69.93 (-72.27 measured) | a Ball landed about 89 mm from its aim point about 0.7 s earlier |

- **Shape:** the Hand dives at about -45 rev/s, slows from near 3 rev while iq is still small, then holds within 0.01 rev.
- **Effect on the next Throw:** on 2026-11-07 the launch reached 107 of its commanded 128 rev/s and the Ball topped out at 0.99 m against about 1.12 m. It was still caught.
- **Count:** one in the 397 launches of the 2026-11-07 Sitting. None in the other captures since 2026-11-03.

## Open questions

- Is it always a Ball? A Ball's diameter would fit a stop 2 to 2.5 rev above the floor, but nobody has confirmed one on 2026-11-06.
- During the 2026-11-06 stalls the command sat exactly 2.00 rev below the Hand. That matches the firmware's +/-2.0 rev Hand lead clamp named in the `hand_stream_bench.py` docstring, so it is a consequence of the stall and not a separate fault.
- No guard acted. As filed in BUG-56, the FW 36 stall guard covers the ACTIVATE and HAND_MOVE_TO ladder, and it needs 300 ms.

Sources: `~/.ros/log/2026-11-07-16-15-21-645789-jetson-2503132/launch.log`, `temp/hand_capture/hand_capture_20261107_161525.csv`, and the 2026-11-06 captures 151412 and 153512. Found while checking BUG-57.

## Progress

- 2026-11-13: Three new instances 2026-11-13 (12:02:17.66, 12:08:30.63, 13:15:22.69), all CONFIRMED as a Ball trapped under the Hand: raw unlabelled mocap markers 65-85 mm below the Hand, static at the stall instant; the owner saw it in the replay.
- 2026-11-13: The 13:15:22 one latched the Hand MAX_DEVIATION guard and self-recovered in 7.2 s. Found with tools/trapped_ball_finder.py; the Tracker-only read had missed it (it drops a Ball after it jostles on the Platform).
