---
id: BUG-149
kind: bug
title: CATCH install CPU grows about 1 ms per 100 record Knots through an Attempt
status: complete
created: '2026-11-27'
updated: '2026-11-27T16:17:50Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: subsystem
diagnosis: hypothesis
area: skill-stack
blocked_by: []
links:
- bug-oval-catch-solve-cpu-81-ms-p50-leaves-no-margin-under-the
- bug-catch-re-aim-refused-infeasible-on-8-4-of-throws-at-the-p2
- logbook/2026-11-28-catch-install-cpu-growth.md
filed_by: claude
---

Found in the debrief of the Sitting of 2026-11-27 (launches 3 and 4, 32-Throw ovals), on main and on offline/bug32-warm alike: the CPU of a CATCH install rises about 1 ms per 100 Knots of record, about +12 to +15 ms by Throw 32, while the CPU of the matching check stays flat. So the growth is in an install-only step.

It tracks the re-aim refusals: SPLICE_TOO_LATE per 16 Throws by Throw index 1-8, 9-16, 17-24, 25-32 was 0, 4, 6, 5 on 2026-11-27; on main, 32-Throw ovals of 11-18 and 11-20 refused 13.8 % of Throws and 200-Throw ovals 25 %, rising with Throw index, against 3.8 % on 8-Throw ovals.

## Hypothesis (unchecked)

The splice (splice_at or extend) copies the whole record head at every install, so its cost grows with the Attempt.

## Wanted

- Profile one install late in a long oval offline and name the step that grows.
- If it is the copy, bound it (keep only the tail the splice needs). This is a cheaper lever on the 150 ms re-aim cliff than the warm start.

Evidence: the debrief's analysis is summarised in logbook/2026-11-27-sitting-debrief.md.
