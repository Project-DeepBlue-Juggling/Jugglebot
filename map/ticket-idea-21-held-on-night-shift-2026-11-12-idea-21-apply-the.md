---
id: TKT-66
kind: ticket
title: 'IDEA-21 held on night-shift/2026-11-12-IDEA-21: apply the audit''s 2 fixes and merge, apply only the empty-Cup count, merge as is, or drop?'
status: complete
created: '2026-11-12'
updated: '2026-11-11T21:16:12Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: gui
blocked_by: []
links: []
parent: idea-create-and-display-an-accuracy-tidyness-score
type: question
---

The landing score is built on night-shift/2026-11-12-IDEA-21, your option 1 from TKT-63. Each Throw is scored in mm from the Fit's landing to its target, and the end line adds the median and the count without a Fit. The scores also go to a new Landing scores section of `docs/records.md`, Throws per Pattern and Feeds in one table. The suite passed; the audit left two findings that change what a number means, so the branch waits for you.

## Options

1. **Apply both fixes and merge.** (a) Score every Feed against the same target: the point Jugglebot's Cup aims at, so a plain Feed and a Columns Feed read alike. Today a plain Feed is scored against Ball Butler's raw announcement and a Columns Feed against the bias-corrected point. (b) Count Throws lost to an empty Cup under "without a Fit", so Scored plus Without a Fit equals the Throws.
2. **Apply (b) only and merge.** Feed scores stay against the announced landing, and the logbook says the Columns Feed is bias-corrected.
3. **Merge as is.** Both differences stay, written down in the logbook.
4. **Drop.**

## What each answer unblocks

- 1 or 2: the next night applies the fixes, re-runs the suite and merges.
- 3: the next night merges the branch as is.

Recommendation: 1, so every Feed number means the same thing and the counts add up.

## Answer

I'm not sure I understand (a) correctly. To be clear: the score for both Ball Butler and Jugglebot throws should be measured against the target that that throw was aimed at. What the other robot is doing (or what's done for the catch) shouldn't matter. We're measuring throw accuracy here. 

I agree with (b).
