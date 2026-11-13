---
id: BUG-97
kind: bug
title: Juggle action Result path has no outer exception guard and can be reset by the next request
status: inbox
created: '2026-11-14'
updated: '2026-11-13T23:08:55Z'
owners: []
priority: low
diagnosis: hypothesis
area: skill-stack
blocked_by: []
links: []
filed_by: claude
---

Two weaknesses in how `skill_node` builds a Juggle action Result. Both were reported by the 2026-11-13 maintainer review as plausible and were not reproduced.

## Findings

- `_juggle_execute` (`skill_node.py:7040-7160`) has two local try/except blocks and no outer guard. An exception elsewhere (for example in the report summary helpers) leaves the action in EXECUTING and `_goal_armed` set, so every later request is refused until a relaunch.
- `_maybe_signal_goal_done` (`:4115`) sets the done event, and only then does `_juggle_execute` read `_goal_end_code`, `_goal_reports`, `_goal_missed_catches` and the Feed fields, without a lock, before writing the records ledger.
- `_start_pattern_body` (`:4366-4375`) resets those same fields. `_refuse_if_running` (`:4119`) checks the executor, the Feed wait, the magazine context and `_goal_starting`, but not `_goal_armed`, so a second request in that window of a few milliseconds is not refused. The first Result and its ledger row would then carry the wrong counts.

## Wanted

- A try/finally around the Result path that always ends the action.
- Snapshot the Result fields under the lock when the done event is set, or make `_refuse_if_running` also refuse while a Result is being built.
