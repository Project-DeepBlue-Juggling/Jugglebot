---
id: IDEA-7
kind: idea
title: 'In ''View'' the selection text doesn''t need to repeat ''Records: '''
status: complete
created: '2026-10-28'
updated: '2026-10-31T19:35:17Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: none
radius: local
area: gui
blocked_by: []
links:
- logbook/2026-11-01-jb92-view-menu-records-prefix.md
filed_by: harrison
---

## Feasibility (2026-11-01)

What it takes: drop the `Records: ` prefix from the per-Pattern label in `menuEntries` (`ros_ws/gui/js/records-model.js`) and let the View menu's section heading carry the word; one assertion in `tests/ros/test_gui_records_panel.py` moves. Effort xs, hardware none. Risk: none beyond the label text (storage key and Pattern ids unchanged). Built by the Night Shift as row 1 tonight.
