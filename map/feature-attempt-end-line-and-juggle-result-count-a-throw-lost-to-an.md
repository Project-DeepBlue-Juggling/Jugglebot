---
id: FEAT-48
kind: feature
title: Attempt end line and Juggle result count a Throw lost to an empty Cup
status: complete
created: '2026-11-11'
updated: '2026-11-11T01:35:44Z'
owners: []
priority: low
effort: s
hardware: none
owner: none
radius: subsystem
area: gui
blocked_by: []
links:
- logbook/2026-11-11-attempt-tally-counts-lost-throws.md
filed_by: claude
---

The Attempt end line's "N/M caught" tally and the Juggle result count only Throws that have a ThrowReport. A Throw lost to an empty Cup is dropped before it gets one, so both omit it.

## Gap

- When a Catch ends MISSED_CATCH because the Cup read EMPTY, the executor drops the ball's pending outcome rows and no ThrowReport is made, so the tally does not show the lost Throw.
- The drops counter of FEAT-47 already counts these Throws through `SkillExecutor.missed_catch_drops`. The end line and the Juggle result disagree with the counter by that number.
- Wanted: the end line and the Juggle result count them in M (Throws) and say so, without changing the executor's row handling, motion or timing.

## Test coverage gaps (closed 2026-11-11)

- The join from the executor to the node for Feed misses is pinned by seeded values, not by a real executor run through the node.
- The same holds for Throws lost to an empty Cup.

Detail: logbook entries 2026-11-11-drops-count-lost-throws and 2026-11-11-drops-history-backfill.

## Progress

- 2026-11-11: Complete 2026-11-11: the tally counts a Throw lost to an empty Cup in M (9/10 caught), logbook 2026-11-11-attempt-tally-counts-lost-throws; the executor-to-node join is now pinned with a real executor
