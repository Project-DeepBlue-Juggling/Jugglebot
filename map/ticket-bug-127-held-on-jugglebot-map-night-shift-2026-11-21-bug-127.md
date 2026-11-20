---
id: TKT-90
kind: ticket
title: 'BUG-127 held on jugglebot-map:night-shift/2026-11-21-BUG-127: apply the audit''s 2 fixes and merge, merge as is, or drop?'
status: complete
created: '2026-11-21'
updated: '2026-11-20T22:32:22Z'
owners: []
priority: low
effort: xs
hardware: none
owner: decision
radius: local
area: map-app
blocked_by: []
links: []
parent: bug-next-sitting-progress-bar-should-have-more-colours
type: question
---

The first part of BUG-127 is built on `jugglebot-map:night-shift/2026-11-21-BUG-127`. The Next Sitting progress rail now draws one strip per Step, coloured by the Step's recorded state only: green done, red problem, amber skipped, grey untouched. Folding or opening a Step no longer changes its strip. Each strip is labelled for screen readers ("Step 3: More, Problem"). The app suite is green, and the audit left two small findings open.

1. **Counter.** The per-launch-group done counter changed meaning. It now counts done and skipped Steps, and a Problem Step with saved notes no longer counts. Nothing reads it today. The fix restores the old count.
2. **Tests.** Two branches of the colour rule are untested: a saved `fail` verdict on an older sheet gives red, and saved notes with no status give green. The fix adds two tests.

Not built: marking the Steps that depend on a problem Step. The sheet format has no field that names another Step, so you need to decide how a Step declares what it depends on first.

## Options

1. **Apply both fixes and merge.** The next night makes them, re-runs the app suite and gate, and merges on MERGE. The dependency part stays open until you choose a format.
2. **Merge as is.** Neither finding changes what you see.
3. **Drop the branch.**

## What each answer unblocks

- 1 or 2: the next night re-gates and merges. Only your eyes can judge whether the amber is clearly different from the red and whether thin strips stay readable.
- 3: BUG-127 stays open, unchanged.

Recommendation: 1, both fixes are a few lines each.

## Answer

'Problem' steps should still be counted as 'done'
Otherwise, go ahead and apply the Test fixes and merge.

On the dependency work: I was hoping this could be something that the Claude that builds the sitting-sheet could figure out. I'd like to start trialling this, and I can give feedback and refine this figuring-out as we go
