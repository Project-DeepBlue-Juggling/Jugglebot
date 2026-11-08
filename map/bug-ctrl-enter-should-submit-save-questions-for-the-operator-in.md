---
id: BUG-46
kind: bug
title: CTRL+Enter should submit/save "Questions for the Operator" in Night Shift Reports.
status: complete
created: '2026-11-01'
updated: '2026-11-01T00:45:50Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: map-app
blocked_by: []
links: []
filed_by: harrison
---

The cursor should also immediately snap to the next text box, ready for me to add my comments. Also, we should add a column to the Report "Questions for teh Operator" column, recording my responses

Filed from the Robot tab at 2026-11-01T11:19:41+11:00.

## Progress

- 2026-11-01: Shipped in the map app (fork 7b13f3a, deployed 2026-11-01): Ctrl/Cmd+Enter saves an answer, the cursor moves to the next question, the report's questions table gains a live Your answer column from each ticket.
