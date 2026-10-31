---
title: "GUI: the records overlay groups Patterns by ball count, fewest first, with a sub-heading and divider per group (JB-93)"
type: feature
date: 2026-11-01
status: complete
phase: night-shift
files_changed:
  - ros_ws/gui/js/records-model.js
  - ros_ws/gui/js/records-panel.js
  - ros_ws/gui/css/records-panel.css
  - tests/ros/js/records_panel_harness.js
  - tests/ros/test_gui_records_panel.py
subsystem:
  - gui
tags:
  - ux
---

# GUI: the records overlay groups Patterns by ball count (JB-93)

## Summary

Owner idea (2026-10-28): records sorted by number of balls, least to most difficult,
with sub-headings and small dividers.

- `records-model.js`: new pure `recordSections(rows)` groups the (already hidden-filtered)
  rows by `balls` ascending, ties keep ledger order, yielding `{ balls, title, rows }` with
  title "1 ball" / "N balls". A ball count with no visible row yields no section, so an
  all-hidden group renders nothing.
- `records-panel.js`: `render()` emits a `.records-group` heading per section and then its
  rows; row building moved to `buildRow`, unchanged, so the `beaten` flash on
  `record_beaten` still applies per row.
- `records-panel.css`: `.records-group` (top border as the divider, none on the first).

Display only; no ledger, `docs/records.md` or control-path change.

## Verification

- 2026-11-01, `pytest tests/ros/test_gui_records_panel.py tests/ros/test_gui_record_panel.py -q -p no:randomly`: 27 passed in 0.55 s. New tests pin order (ledger order c2,a1,t3,b2,d1 becomes 1,2,3 with ties in ledger order), titles, and the empty-group rule.
- 2026-11-01, harness run against `git show HEAD:ros_ws/gui/js/records-model.js`: `SyntaxError ... does not provide an export named 'recordSections'`, so the new tests fail without the change.
- Not run: the live GUI render (no browser pass).
