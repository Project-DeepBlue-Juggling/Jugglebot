---
id: BUG-33
kind: bug
title: 'ABORTED_NO_RELEASE: three goals ended because the throw never left the hand (new 2026-10-28 21:02)'
status: inbox
created: '2026-10-28'
updated: '2026-11-08T00:13:26Z'
owners: []
priority: high
effort: m
hardware: robot
owner: hands
radius: subsystem
diagnosis: unknown
area: hand
blocked_by: []
links:
- 2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented
- logbook/2026-11-06-sitting-debrief-2.md
parent: two-ball-oval
filed_by: claude (BUG-21 21:02 Sitting read-out)
---

21:02 Sitting: goals 1 (columns, throw 61), 2 (columns, throw 25) and 3 (oval_1ball, throw 32) ended ABORTED_NO_RELEASE: throw N never left the hand (no release evidence by +0.5 s). Not seen in the 09:49 or 14:29 Sittings the same day. Not a solver symptom (BUG-21 is closed).

## Next step

Needs the bag (`~/Desktop/rosbags/2026-10-28_21-02*`) read for hand telemetry at those three instants: hand stroke commanded and executed, ball sensor, cup state.

Launch log: `~/.ros/log/2026-10-28-21-02-48-831501-jetson-3765883/launch.log`.

## Progress

- 2026-11-06: Two ABORTED_NO_RELEASE ends on 2026-11-06 were not release failures. P2 goal 19 (L11050): the Hand stroke was normal and the cup read empty for the whole stroke; the ball was lost at the Catch before. Endurance 14:55 goal 9 (L9001-L9014): the release had been cancelled by an earlier END. The end code names the wrong thing; the three 2026-10-28 ends were not re-examined.
- 2026-11-08: 2026-11-07 L2489 matches the 2026-11-06 reading: Throw 19/21 was a missed Catch (ball_held 0 for 0.9 s), reported as ABORTED_NO_RELEASE.
