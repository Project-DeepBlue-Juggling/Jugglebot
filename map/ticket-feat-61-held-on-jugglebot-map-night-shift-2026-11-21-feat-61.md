---
id: TKT-91
kind: ticket
title: 'FEAT-61 held on jugglebot-map:night-shift/2026-11-21-FEAT-61: apply the audit''s 2 fixes and merge, merge as is, or drop?'
status: complete
created: '2026-11-21'
updated: '2026-11-20T22:34:30Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: map-app
blocked_by: []
links: []
parent: feature-improved-next-sitting-readability
type: question
---

FEAT-61 is built on `jugglebot-map:night-shift/2026-11-21-FEAT-61`. In the expanded Next Sitting Step, `do`, `see`, `stop if` and `record` each get a label column and a thin coloured left border, and `stop if` is amber. Multi-line fields list one entry per line, and several actions are numbered 3a, 3b. There is no new padding and no font change.

The audit left two small findings open, and the app suite failed on a test this change does not touch. That test passes on its own and is filed as its own Bug.

1. **Phone width.** The label column narrows the content by about 52 px, so on a phone long `do` commands can wrap sooner and a Step can get taller. The fix narrows the column on small screens.
2. **Sub-step letters.** After 26 actions the letters repeat. No real Step has that many.

## Options

1. **Apply both fixes and merge.** The next night makes them, re-runs the app suite and gate, and merges on MERGE.
2. **Merge as is.** You judge the look on the live app, and the next night only re-runs the suite.
3. **Drop the branch.** You want a different layout; say what in your answer.

## What each answer unblocks

- 1 or 2: the next night re-gates and merges. This change can only be judged by eye, so look at a long Step after the deploy.
- 3: FEAT-61 stays open for a layout you describe.

Recommendation: 1. The phone-width fix keeps your no-new-scrolling rule on the phone too.

## Answer

Don't worry about phone use. This will only ever be used on a computer with a proper monitor.
Let's go with Option 2. Merge as-is and I'll give feedback
