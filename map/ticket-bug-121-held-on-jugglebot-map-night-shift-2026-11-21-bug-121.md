---
id: TKT-89
kind: ticket
title: 'BUG-121 held on jugglebot-map:night-shift/2026-11-21-BUG-121: apply the audit''s 2 fixes and merge, merge as is, or drop?'
status: complete
created: '2026-11-21'
updated: '2026-11-20T22:37:50Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: map-app
blocked_by: []
links: []
parent: bug-saving-comments-in-next-sitting-steps-could-be-improved
type: question
---

BUG-121 is built on `jugglebot-map:night-shift/2026-11-21-BUG-121`. Each Next Sitting Step now has one "Save feedback" button that saves the comment and every Item's notes in one request, and Ctrl+Enter in any box does the same. A saved Step collapses, and a failed save keeps the Step open and names the boxes. The app suite is green, and the audit left two small findings open.

1. **Collapse race.** If you collapse the Step yourself while the save is in flight, it re-opens when the save lands, because the save toggles the Step instead of closing it. The fix is a close call instead of a toggle.
2. **Failure message.** After a failed save the message also names notes you saved earlier and did not touch, which reads as if they were lost. A save that throws instead of failing cleanly shows no message at all. The fix names only the changed boxes and catches the throw.

## Options

1. **Apply both fixes and merge.** The next night makes the two changes, re-runs the app suite and gate, and merges on MERGE.
2. **Merge as is.** Both are edge cases, and no typed text is ever lost.
3. **Drop the branch.** BUG-121 goes back to open.

## What each answer unblocks

- 1 or 2: the next night re-gates and merges on MERGE. The layout of the new button below the comment panel can only be judged by eye.
- 3: BUG-121 reopens.

Recommendation: 1. Each fix is a few lines, and the first one stops a Step you closed from re-opening.

## Answer

Agreed. Go with Option 1
