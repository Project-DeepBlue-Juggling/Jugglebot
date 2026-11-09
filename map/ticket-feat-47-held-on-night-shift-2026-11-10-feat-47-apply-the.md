---
id: TKT-61
kind: ticket
title: 'FEAT-47 held on night-shift/2026-11-10-FEAT-47: apply the audit''s 3 fixes and merge, also count Feed misses, merge as is, or drop?'
status: open
created: '2026-11-10'
updated: '2026-11-09T18:16:28Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: gui
blocked_by: []
links: []
parent: feature-add-total-drops-to-date-counter
type: question
---

The live Drops to date counter is built on `night-shift/2026-11-10-FEAT-47`: a section in `docs/records.md` (live plus a history slot, since a date) and a line atop the GUI's Records panel. Ball Butler Feed misses are not counted yet, because the executor does not say which missed Catch was the Feed's; the history backfill is for a day Session. The audit found three things that hold it.

## The open findings

1. **Every `magazine` run books a false drop.** Its report is marked not caught as a placeholder, and nothing is thrown to be caught. Fix: a Feed pattern books only Feed misses (0 today), with a test on a real magazine end.
2. **The counter hides when no record rows show** (a fresh ledger, or every Pattern hidden in View). Fix: show the panel when the drops section exists.
3. **One new log line sits outside the never-raises guard.** Fix: move it inside.

## Options

1. Apply all three and merge.
2. Apply all three, also count Ball Butler Feed misses (a small executor change marking the Feed's Catch), and merge.
3. Merge as is.
4. Drop the branch.

Recommendation: 1. Feed misses can follow as their own slice once 1 is live.

## What each answer unblocks

- 1 or 2: the next night applies them, re-gates and merges.
- 3: the next night re-gates and merges.
- 4: FEAT-47 goes back to backlog.
