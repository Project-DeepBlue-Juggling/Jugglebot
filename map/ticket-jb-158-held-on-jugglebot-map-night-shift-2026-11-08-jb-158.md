---
id: JB-206
kind: ticket
title: 'JB-158 held on jugglebot-map:night-shift/2026-11-08-JB-158: apply the audit''s 3 fixes and merge, apply only the lost-notes fix, merge as is, or drop?'
status: complete
created: '2026-11-08'
updated: '2026-11-08T00:16:27Z'
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

JB-158 is built on `jugglebot-map:night-shift/2026-11-08-JB-158`: Ctrl+Enter or Cmd+Enter in a Next Sitting row's notes saves the result. The app suite passed on the tip. The audit found an older bug that this makes easier to hit, so the merge gate held it.

## The open findings

1. **A failed save loses the result.** `onSave` catches every error and resolves, so the row clears the verdict and the typed notes anyway. With a keypress you are not looking at the row when the toast appears. The fix keeps the row's text until a save succeeds.
2. No IME guard on the keypress. One condition.
3. No test of a failed save followed by a retry. One test.

## Options

1. Apply all three and merge.
2. Apply only 1 and its test, and merge.
3. Merge as is: the older bug stays, and is filed as its own Item.
4. Drop the branch.

Recommendation: 1. Notes typed during a Sitting are the one thing here that cannot be recovered.

## What each answer unblocks

- 1 or 2: the next night applies them, re-gates and merges.
- 3: the next night re-gates, merges and files the bug.
- 4: JB-158 goes back to open.

## Answer

Agreed.
