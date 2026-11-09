---
id: TKT-58
kind: ticket
title: 'BUG-83 held again on jugglebot-map:night-shift/2026-11-08-BUG-83: merge as is, also scroll the group label into view, or drop?'
status: open
created: '2026-11-10'
updated: '2026-11-09T17:48:26Z'
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

Your three fixes from TKT-50 are on the branch, and the app suite passed on 2026-11-09. That night's audit found one new small thing, and any behaviour finding holds a branch until you decide it.

## The open finding

Arrow-keying to the first result of a group scrolls that row into view but leaves the group's label (ITEMS, PLANS, TERMS) just above the top edge. The active row is always visible, so this is cosmetic.

## Options

1. Merge as is.
2. Also scroll the group label into view on the first result of a group, and merge. One line and a test.
3. Drop the branch.

Recommendation: 1. It is the fix you asked for, and the label only hides on a list long enough to scroll.

## What each answer unblocks

- 1: the next night re-gates and merges.
- 2: the next night applies it, re-gates and merges.
- 3: BUG-83 goes back to open.
