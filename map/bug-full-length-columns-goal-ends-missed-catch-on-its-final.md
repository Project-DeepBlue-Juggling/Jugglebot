---
id: JB-169
kind: bug
title: Full-length Columns goal ends MISSED_CATCH on its final Catch although the ball was caught
status: inbox
created: '2026-11-04'
updated: '2026-11-05T18:57:21Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: local
diagnosis: hypothesis
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-04-jb149-probe-sitting-and-jb139-close.md
filed_by: claude
---

Sitting 2026-11-04 18:45, goal 3 (L2362): the owner saw the last Catch succeed. Columns holds one ball, so the Stop ball rests on the held ball. The report line just before (L2361) reads 'throw 50/50 STOP (landed on the held ball, by design; not counted)'.

The missed-catch detector still judged the final CATCH (Ball 1, skill 50 of 50) from 67 cup samples, all EMPTY over [-0.21, +0.50] s, and ended the goal MISSED_CATCH at 48/48. Same shape on 2026-10-28 09:49 (Ball 2, skill 35 of 35, 33/33). A clean full-length goal ends 'columns done (one ball held ...) 20/20 caught' (2026-11-03 22:29).

Effect: a full-length goal is booked as a miss in the operator line and records, and the final ball's OUTCOME rows are dropped.

The detector already skips a catch whose carried throw is the Stop's shadow landing (executor.py _register_catch, the shadow_landing check); the final CATCH has no such exemption. Why the cup read EMPTY with a ball in it is open (two stacked balls vs the sensor, or the window). Hypothesis only: read the cup-sensor trace in the bag before fixing.

## Findings (2026-11-05)

From the 2026-11-04 18:45 bag (`/hand_telemetry`, `/balls`) and launch.log, Night Shift, read-only.

- Why the final CATCH is judged: skill 50 is a plain final CATCH of Ball 1 with no carried Throw, so the `shadow_landing` exemption never applies. The Stop is Ball 2's Throw carried by skill 49, not a CATCH skill, so the other-ball landing guard never caps the window. It closes at landing + 0.50 s and the verdict fires 38 ms later.
- What the cup saw: EMPTY from 479.36 to 480.20 with valid samples, the Hand at rest from about 479.77. The first SEATED read is at 480.203, which is the Stop landing (seat +0.117 s), after the window closed.
- What mocap saw: Ball 1 came down to about cup height (z 702-706 mm) and was then lost, as an occluded ball in the cup is. The Stop ball rests at z 728 mm, about 26 mm above and 40 mm across from Ball 1.
- Skill 50's Catch was marginal: aim clamped to 35 mm, last fit 15-19 mm from the aim, second re-aim refused (LIMIT_JERK).
- Open: either the cup sensor does not read a lone off-centre ball at rest (it did read the stacked pair), or Ball 1 never seated and the Stop landed in an empty cup. The bag cannot tell them apart.

## Progress

- 2026-11-05: 2026-11-05 L2615 MISSED_CATCH is a different class: Hand SPINOUT disarm at 503.68 inside the Catch window, verdict 503.767 after the lane latched; not a false miss, and JB-172 option 4 (no Stop) would not cover it. Separate bug filed for fault-preempted Catch verdicts.
