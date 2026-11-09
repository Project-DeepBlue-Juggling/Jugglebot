---
id: IDEA-21
kind: idea
title: Create and display an 'accuracy'/'tidyness' score
status: inbox
created: '2026-11-09'
updated: '2026-11-09T08:45:04Z'
owners: []
priority: medium
effort: m
hardware: none
blocked_by: []
links: []
filed_by: harrison
---

A metric should be devised that 'scores' juggling patterns based on how close the real pattern matches the target pattern. I'm thinking something like `sum of ( error^2 / sample rate )` for each throw, as well as an average for the whole pattern. 

If this would add excessive burden to the Jetson, this could be calculated after each session as long as we're able to buffer the relevant data for long enough. 

Please also make this Item a part of a new "GUI Improvements" Pathway, then delete this part of this Description.
