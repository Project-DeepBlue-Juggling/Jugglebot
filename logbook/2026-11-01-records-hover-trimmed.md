---
title: "GUI: a record's hover is trimmed to how its Attempt ended plus the run parameters"
type: refactor
date: 2026-11-01
status: complete
phase: gui
related_entries:
  - 2026-11-01-jb90-records-run-parameters.md
files_changed:
  - ros_ws/gui/js/records-model.js
  - tests/ros/test_gui_records_panel.py
  - tests/ros/test_gui_records_hover_place.py
  - logbook/INDEX.md
subsystem:
  - gui
tags:
  - records
  - ux
---

# Records hover trimmed

**What.** Owner ask (2026-11-01), right after FEAT-29 shipped: the records row's hover repeated the
Catches/Throws and the date (both already on the row), carried the time and the Note, and ended
with "beaten N× before" (a count of the earlier records in that Pattern's table, read as if the
current record had itself been beaten). The hover is now two lines: `Ended: <category>
(<END_CODE>)` (the shortest label the owner accepted for the termination reason), then the FEAT-29
parameters line. The ledger is unchanged: the Note and the history remain in `docs/records.md`.

## Verification

- 2026-11-01, `pytest tests/ros/test_gui_records_panel.py tests/ros/test_gui_records_hover_place.py tests/ros/test_gui_record_panel.py -q`: 32 passed in 0.81 s.
