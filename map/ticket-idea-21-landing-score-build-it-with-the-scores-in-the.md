---
id: TKT-63
kind: ticket
title: 'IDEA-21 landing score: build it with the scores in the records ledger, in a CSV beside the Memory, also add path vs plan as a release number, or not now?'
status: complete
created: '2026-11-11'
updated: '2026-11-10T21:46:43Z'
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

You asked for both scores on logs. IDEA-21 Findings (2026-11-11) has the result: mocap noise moves a landing by under 0.4 mm against errors of 14 to 32 mm. Path vs plan scores the release, not the landing, and ranks flights differently. Path vs the Fit measures only mocap noise.

## Options

1. **Build the landing score, the scores kept in the ledger.** Per Throw in mm. At the end of an Attempt the GUI line adds the median and a miss count. Each Feed shows its own score. Every score goes into a section of `docs/records.md`, committed at the Sitting close-out like the records: Throws per Pattern, Feeds in one list.
2. **The same, the scores kept in a CSV under `temp/learn/`** beside the learner's Memory. Easier to plot, and nothing renders it.
3. **Option 1 plus path vs plan** as a separate release-accuracy number, computed from the bag after the Sitting.
4. Not now.

Recommendation: 1. It is your design from TKT-60, it reads in mm, and it costs nothing live.

## What each answer unblocks

- 1, 2 or 3: the next night builds the landing score and its readout on a branch. The suite judges the build; no Sitting is needed.
- 4: IDEA-21 goes back to backlog.

## Answer

Agreed. Go for Option 1
