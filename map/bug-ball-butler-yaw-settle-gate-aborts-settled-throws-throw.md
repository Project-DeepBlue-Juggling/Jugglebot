---
id: BUG-48
kind: bug
title: Ball Butler yaw-settle gate aborts settled throws (THROW_ABORTED_NOT_SETTLED false positives)
status: inbox
created: '2026-11-01'
updated: '2026-11-06T23:52:43Z'
owners: []
priority: low
effort: s
hardware: robot
owner: hands
radius: local
diagnosis: hypothesis
area: ball-butler
blocked_by: []
links:
- map/bug-not-settled-retry-dead-in-the-field.md
parent: ball-butler
filed_by: claude
---

Ball Butler's yaw-settle gate aborts throws it could have made: `THROW_ABORTED_NOT_SETTLED` ended 4 of 19 attempts on 2026-09-30, judged plausible false positives (`not_settled_report.md`). Since BUG-6 closed on 2026-11-01 (TKT-21 option 1) the host retries nothing: each false positive costs one attempt, `REJECTED_BB(THROW_ABORTED_NOT_SETTLED)`.

The fix lives in the BallButler repo (`ball_butler_main/HandTrajectoryStreamer.h`, the settle check before the first frame): tighten or re-tune the settle criterion so a settled Ball Butler is not refused. Do it when the Ball Butler firmware is next touched. Trigger: `NOT_SETTLED` ends return in a Sitting's tally.

No firmware change exists yet, so there is no `## Sitting` section and `hardware` is left unrated; whoever writes the change rates it `robot` and adds the section (BB-fed throws, pass: no `NOT_SETTLED` end on a settled Ball Butler).

## Review (2026-11-07)

Not a Night Shift Item: the fix is firmware in the BallButler repo, whose tree is dirty, and it must fly. Rated `hardware: robot`. It waits on its own trigger, `NOT_SETTLED` ends returning in a Sitting's tally.
