---
id: JB-97
kind: bug
title: 'ABORTED_NO_RELEASE: three goals ended because the throw never left the hand (new 2026-10-28 21:02)'
status: inbox
created: '2026-10-28'
updated: '2026-10-28T10:22:56Z'
owners: []
priority: high
area: hand
blocked_by: []
links:
- 2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented
parent: two-ball-oval
filed_by: claude (JB-76 21:02 Sitting read-out)
---

21:02 Sitting: goals 1 (columns, throw 61), 2 (columns, throw 25) and 3 (oval_1ball, throw 32) ended ABORTED_NO_RELEASE: throw N never left the hand (no release evidence by +0.5 s). Not seen in the 09:49 or 14:29 Sittings the same day. Not a solver symptom (JB-76 is closed).

## Next step

Needs the bag (`~/Desktop/rosbags/2026-10-28_21-02*`) read for hand telemetry at those three instants: hand stroke commanded and executed, ball sensor, cup state.

Launch log: `~/.ros/log/2026-10-28-21-02-48-831501-jetson-3765883/launch.log`.
