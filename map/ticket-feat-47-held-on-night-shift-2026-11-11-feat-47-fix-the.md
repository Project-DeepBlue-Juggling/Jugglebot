---
id: TKT-64
kind: ticket
title: 'FEAT-47 held on night-shift/2026-11-11-FEAT-47: fix the wording and merge with magazine at 0, also count magazine Feed misses, merge as is, or drop?'
status: complete
created: '2026-11-11'
updated: '2026-11-10T21:42:31Z'
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

Slice 2 counts a Ball Butler Feed that misses the Cup as a drop in the fed Patterns: the executor records a Feed Catch's MISSED_CATCH and the node books it. The full suite passed (`./run_tests.sh --full`, 2026-11-11: 9337 passed + 10 serial, `EXIT=0`), but the audit found a gap in `magazine`.

## The open findings

1. **A `magazine` run still books no Feed miss.** It runs without the skill executor, so the new count never reaches it. Counting it needs the magazine's own judgement of whether the Feed seated.
2. **Its test sets a state the magazine path cannot reach**, and the comment and logbook read as if magazine Feed misses are counted.

## Options

1. **Fix the wording, drop the magazine test and merge.** Fed Patterns count Feed misses now; `magazine` stays at 0 and is written down as the next slice.
2. **Also count magazine Feed misses** from the magazine's own Feed check, then merge. One more unit and a re-gate.
3. Merge as is.
4. Drop the branch.

Recommendation: 2, so FEAT-47 can close with every Feed counted, as you asked in TKT-61.

## What each answer unblocks

- 1 or 2: the next night applies it, re-gates and merges.
- 3: the next night re-gates and merges.
- 4: Feed misses go back to not counted.

## Answer

`magazine` Throws should never be considered in the drop counts; when I get around to tuning these properly, these throws will be from Jugglebot to Ball Butler's magazine, with nothing to 'catch' them. If a ball is being fed from Ball Butler to Jugglebot for Jugglebot to make the `magazine` throw, and the BB throw is missed by Jugglebot, that drop/miss should be counted. But not throws from Jugglebot to the magazine.

So I believe the right choice here is Option 1
