---
id: TKT-74
kind: ticket
title: 'BUG-91 held on night-shift/2026-11-15-BUG-91: add the warning-text pin test and merge, merge as is, or drop?'
status: complete
created: '2026-11-15'
updated: '2026-11-14T23:37:09Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-nominate-accepts-a-setting-whose-own-reply-shows-a-negative
type: question
---

Your TKT-70 option 2 is built on night-shift/2026-11-15-BUG-91: an accepted nomination with a negative re-aim margin ends its line with WARNING: RE-AIM MARGIN NEGATIVE and the GUI shows it red. The gate held it because tonight's suite failed the 4 Sunday runner tests (BUG-107) and one wall-clock serial test that passed on another branch tonight, and because the audit left one finding open.

## Options

1. **Add the pin test and merge:** a test that the GUI's copy of the warning text equals `nominate.AIM_WARNING_TEXT`, so a Python wording change cannot silently drop the red state. The next weekday night builds it, re-gates and merges on MERGE.
2. **Merge as is:** the next weekday night re-gates the branch unchanged and merges on MERGE.
3. **Drop the branch.**

## What each answer unblocks

- 1 or 2: the next weekday night (the Sunday failures are absent on other days).
- 3: BUG-91 goes back to open.

Recommendation: 1. It is a few lines, and the red state is the whole point of the change.

## Answer

Agreed.
