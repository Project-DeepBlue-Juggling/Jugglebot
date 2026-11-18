---
id: BUG-114
kind: bug
title: MISSED_CATCH end line names the empty carried Throw, not the flight that was lost one cycle earlier
status: inbox
created: '2026-11-18'
updated: '2026-11-18T02:07:43Z'
owners: []
priority: medium
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-18-sitting-debrief.md
filed_by: sitting-debrief 2026-11-18
---

Found at the debrief of the Sitting of 2026-11-18 (34 MISSED_CATCH ends). Related: BUG-78, BUG-33.

The end line names the carried Throw (NO FIT), but in 24 of 34 ends that Throw never flew: its track is announced-only at Cup height and ball_held was false for the 0.4 s before its release in 33 of 33. The real loss is the same Ball one cycle earlier, which has no per-Throw log line. The MISSED-BALL evidence line (executor.py) is logged at DEBUG, so the tee log lacked it for all 34 ends.

Fix made at the debrief: the MISSED-BALL line is logged at INFO. Still open: make the end line name the lost flight (the Throw one cycle earlier) instead of the empty carried Throw.
