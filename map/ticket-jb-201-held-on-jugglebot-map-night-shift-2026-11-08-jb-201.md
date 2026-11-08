---
id: JB-207
kind: ticket
title: 'JB-201 held on jugglebot-map:night-shift/2026-11-08-JB-201: apply the audit''s small fixes and merge, also guard mouseenter, merge as is, or drop?'
status: complete
created: '2026-11-08'
updated: '2026-11-08T00:12:32Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: map-app
blocked_by: []
links: []
parent: bug-arrow-keying-through-the-map-search-items-doesn-t-scroll
type: question
---

JB-201 is built on `jugglebot-map:night-shift/2026-11-08-JB-201`: arrow-keying through the search results now scrolls the active one into view. The audit found small edge cases in the new code, so the merge gate held it. Nothing is live.

## The open findings

1. **A stray scroll.** With a single result an arrow key changes nothing, so a flag stays set and the next results change scrolls once. Harmless at the top of a fresh list.
2. The hover half of the new test passes without the line it seems to cover. One test.
3. The test's `scrollIntoView` stub is never restored. One line.
4. **Unverified:** a list scrolling under a resting pointer may fire `mouseenter` in some browsers, and that would take the selection away from the arrow keys. Only a real browser can show it.

## Options

1. Apply 1 to 3 and merge; you try 4 in the browser after the deploy and file it if it happens.
2. Apply 1 to 4, ignoring `mouseenter` with no real pointer movement, and merge. More code for a case nobody has seen.
3. Merge as is.
4. Drop the branch.

Recommendation: 1.

## What each answer unblocks

- 1 or 2: the next night applies them, re-gates and merges.
- 3: the next night re-gates and merges.
- 4: JB-201 goes back to open.

## Answer

Agreed.
