---
id: TKT-57
kind: ticket
title: 'BUG-60 held again on jugglebot-map:night-shift/2026-11-08-BUG-60: make the row read-only while saving and add the two tests, only the read-only row, merge as is, or drop?'
status: complete
created: '2026-11-10'
updated: '2026-11-09T21:49:58Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: map-app
blocked_by: []
links: []
parent: bug-ctrl-enter-to-save-the-result-of-a-next-sitting-entry
type: question
---

Your three fixes from TKT-49 are on the branch and the app suite passed on 2026-11-09. That night's audit found something new, and a behaviour-affecting finding holds a branch until you decide it.

## The open findings

1. **Text typed during a save is wiped.** The row stays editable while a save is in flight, and Ctrl+Enter leaves focus in it. Keep typing or change the verdict during a slow save, and that input is cleared when the save succeeds, with no toast.
2. The test named "saves once" does not isolate the key-repeat guard: the in-flight check blocks the same events.
3. No test covers Cmd+Enter, which the handler claims.

## Options

1. Apply all three (the row read-only while saving, two tests) and merge.
2. Apply only 1 and merge.
3. Merge as is: the wipe needs a slow save and a keystroke in that window.
4. Drop the branch.

Recommendation: 1, the row read-only while saving is one attribute.

## What each answer unblocks

- 1 or 2: the next night applies them, re-gates and merges.
- 3: the next night re-gates and merges.
- 4: BUG-60 goes back to open.

## Answer

Let's go with Option 2. I don't use Mac, so CMD won't be realised, and we can add more tests in the future if this proves buggy.
