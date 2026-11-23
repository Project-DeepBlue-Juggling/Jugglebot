---
id: TKT-92
kind: ticket
title: 'After ACTIVATE the Platform holds 0.6 degrees off Level by design: leave it, add a GUI not-Level indicator, go_home automatically, or land ACTIVATE on Level?'
status: complete
created: '2026-11-23'
updated: '2026-11-22T22:51:50Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-held-active-pose-is-0-6-degrees-off-level-the-levelling
type: question
---

After ACTIVATE the Platform holds the uncorrected neutral, 0.6 degrees off Level, until its first planned move; the trace found this is by design (C-LEVEL-1), not a defect. What should change, if anything?

1. Leave it as designed: `levelling_frame.md` documents it, and anything that needs Level follows `go_home`, a Jog target or an Attempt's pre-level REST.
2. A GUI not-Level indicator from ACTIVATE until the first planned move. Jetson and GUI only, no motion; the Night Shift can build it.
3. An automatic `go_home` after ACTIVATE, so the held Active pose is Level. An unrequested 0.6 degree move at activation; needs a Sitting.
4. ACTIVATE lands on the corrected neutral: levelled leg targets sent with ACTIVATE. A firmware change and a flash.

Answer 1 closes BUG-130. Answer 2 lets the next night build the indicator and merge it if the gate says MERGE. Answer 3 gets a held branch and a Sitting row. Answer 4 goes to a day Session.

Recommendation: 2, and 3 as well if Level should be the default state after ACTIVATE.

## Answer

I lean Option 4. The Platform should raise into an already-level Active pose
