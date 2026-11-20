---
id: BUG-33
kind: bug
title: 'ABORTED_NO_RELEASE: three goals ended because the throw never left the hand (new 2026-10-28 21:02)'
status: complete
created: '2026-10-28'
updated: '2026-11-20T14:03:32Z'
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
- logbook/2026-11-13-sitting-debrief.md
- logbook/2026-11-18-sitting-debrief.md
- logbook/2026-11-20-sitting-debrief.md
- logbook/2026-11-20-empty-cup-no-release-ends-as-missed-catch.md
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
- 2026-11-13: Sitting of 2026-11-13: the 12:01:36 ABORTED_NO_RELEASE (columns_highlow, 6/8 caught) follows a MISSED Throw 8 near aim (13 mm): same lost-ball-at-the-Catch reading as the 2026-11-06 and 11-08 notes. Confirming needs ball_held in the 2026-11-13_11-42-08 bag around 12:01:35.
- 2026-11-18: 2026-11-18 debrief: another case of the same signature, launch 4 12:20:41 ABORTED_NO_RELEASE (throw 16 missed at 12:20:39.4, a Ball under the Hand 12:20:38.97-39.89); the end code names the wrong thing.
- 2026-11-20: Sitting of 2026-11-20: 4 of 4 ABORTED_NO_RELEASE ends (launch 1 Attempt 10; launch 2 Attempts 2, 3, 4) show a normal Hand stroke with ball_held 0 throughout: the Cup was empty because a Ball was lost one cycle earlier. Launch 2 Attempt 3 logged no MISSED-BALL line at all, so the end hides the loss entirely. Relabel location: executor.py _advance_release_evidence;
- 2026-11-20: it changes an end code INVARIANTS lists, so it is an owner decision.
- 2026-11-21: Landed 2026-11-21 (cf8ecd1c): a Throw from a Cup that read empty since the Ball's previous release ends as a missed Catch. Gate: parallel 10667 passed, serial 7 passed. Audit found no behaviour-affecting findings.
