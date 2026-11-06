---
id: JB-196
kind: ticket
title: 'JB-190 held on night-shift/2026-11-07-JB-190: apply the audit''s 3 fixes and merge, apply only the probe regex, merge as is, or drop?'
status: open
created: '2026-11-07'
updated: '2026-11-06T18:58:00Z'
owners: []
priority: low
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-a-goal-entered-by-a-feed-prints-pattern-reload-ended-and-no
type: question
---

JB-190 is built on night-shift/2026-11-07-JB-190: a goal entered by a Feed now ends on `columns_turned ENDED (...) ... · entered by a Feed`, so a search for the pattern finds it. The full gate passed on the tip, but the audit found one probe that reads the old shape, so the merge gate held it.

## The open findings

1. `tools/probes/feed_catch_bag_probe.py:125` matches `self_toss (reload) ENDED (REJECTED_NO_BALL)`. On new logs it would never match and would report no such outcome, silently. The fix is one regex that accepts both shapes.
2. `skill_node._end_label` cuts the suffix off on the flag's word. It is safe today; a check that the label really ends with it costs one line.
3. No node test drives the executor-retire end line with a Feed. One test.

## Options

1. Apply all three and merge. Nothing on the robot path changes beyond the line text.
2. Apply only the probe regex and merge.
3. Merge as is: the probe goes blind on new logs until someone fixes it.
4. Drop the branch.

Recommendation: 1.

## What each answer unblocks

- 1 or 2: the next night applies them, re-gates and merges.
- 3: the next night re-gates and merges.
- 4: JB-190 goes back to open.
