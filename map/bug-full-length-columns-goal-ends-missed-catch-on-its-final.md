---
id: JB-169
kind: bug
title: Full-length Columns goal ends MISSED_CATCH on its final Catch although the ball was caught
status: inbox
created: '2026-11-04'
updated: '2026-11-04T08:01:01Z'
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
