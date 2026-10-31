---
title: "JB-92: the View menu's per-Pattern entries drop the redundant 'Records: ' prefix; the 'Records' section heading is the one place the word appears"
type: feature
date: 2026-11-01
status: complete
phase: "night-shift"
related_plan:
  - night-shift.md
files_changed:
  - ros_ws/gui/js/records-model.js
  - ros_ws/gui/js/records-panel.js
  - tests/ros/test_gui_records_panel.py
  - logbook/INDEX.md
subsystem:
  - gui
tags:
  - ux
  - night-shift
---

# JB-92: View menu entries no longer repeat "Records: "

**What.** `menuEntries` in `ros_ws/gui/js/records-model.js` labelled each View-menu checkbox `Records: <Pattern>`, directly under a `scene-menu-heading` that already reads `Records` (`records-panel.js`, the heading text was already literally "Records" and is unchanged). The label is now just `<Pattern>` (`Hop`, `Self-throw`, `Columns`). Storage key, Pattern ids and checked state are untouched. The `records-panel.js` header comment was updated to match.

**Why.** Owner quick capture 2026-10-28: the selection text does not need to repeat "Records: ". Display-only change; no control, timing or hardware path involved.

**Tests.** The assertion pinning the label in `test_view_menu_entries_and_storage` (`tests/ros/test_gui_records_panel.py`, three expected labels) now expects the bare Pattern names.

**Verification.**
- Fails without the change (2026-11-01, model reverted to the prefixed label, `pytest tests/ros/test_gui_records_panel.py -q -p no:randomly`): 2 failed, 6 passed (`test_view_menu_entries_and_storage` and `test_one_row_per_pattern_in_ledger_order`).
- With the change (2026-11-01, `pytest tests/ros/test_gui_records_panel.py tests/ros/test_gui_record_panel.py -q -p no:randomly`): 24 passed in 0.52 s.
- `./run_tests.sh` not run (unit brief).
