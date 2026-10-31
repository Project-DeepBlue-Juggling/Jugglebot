---
id: JB-93
kind: idea
title: The records should be sorted by number of balls, from least- to most-difficult, with sub-headings and small dividers between them.
status: complete
created: '2026-10-28'
updated: '2026-10-31T19:44:51Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: none
radius: local
area: gui
blocked_by: []
links:
- logbook/2026-11-01-jb93-records-grouped-by-ball-count.md
filed_by: harrison
---

## Feasibility (2026-11-01)

What it takes: the row model (`ros_ws/gui/js/records-model.js`) already carries `balls` per Pattern and the ledger is in ball order; a pure grouping (ascending `balls`, ties in ledger order) feeds the panel one sub-heading and divider per group, a group with every row hidden renders nothing, the record flash stays per row; the node harness pins order, titles and the empty-group rule.

Effort xs, hardware none. Risk: none beyond layout (a few lines of CSS). Built by the Night Shift as row 1 tonight.
