---
title: "JB-91: the item description is multi-line in both filing forms (GUI File new change request, map app + New)"
type: feature
date: 2026-11-01
status: complete
phase: gui
files_changed:
  - ros_ws/gui/index.html
  - ros_ws/gui/css/app-tabs.css
  - tests/ros/test_gui_quick_capture.py
  - map/idea-larger-text-box-when-creating-items.md
  - docs/project-map.md
  - logbook/INDEX.md
subsystem:
  - gui
tags:
  - map
  - ux
---

# JB-91: multi-line description when filing an Item

**What.** Owner idea (2026-10-28): the single-line description field in both filing forms
should assume multi-line text. Both are now a 4-row textarea that the operator can stretch
vertically, labelled Description:

- GUI bar "File new change request" (`ros_ws/gui/index.html` `#qc-note`, was a text input
  labelled "Note (optional)"; styled with the form's inputs in `css/app-tabs.css`).
- Map app "+ New" modal (fork `jugglebot-map` commit 959966c, `NewItemModal.tsx`; the "One line"
  label is now "Description").

A textarea never submits its form on a bare Enter, so Enter adds a line. Ctrl+Enter / Cmd+Enter
still files from any field: both forms' form-level keydown handlers already did that (JB-78).
Nothing changes downstream: both forms already sent the text as the Item body
(`description`), which the map server writes verbatim, newlines included. The Title stays a
single line, because it becomes the Item's slug.

## Verification

- 2026-11-01, `pytest tests/ros/test_gui_quick_capture.py -q`: 19 passed in 0.59 s (new test: `#qc-note` is a 4-row textarea, the CSS resizes vertically, and Ctrl+Enter still files).
- 2026-11-01, map app `npx vitest run`: 331 passed, 1 skipped (43 files); lint and typecheck exit 0. `Drawer.test.tsx` alone: 21 passed. New test: Description is a TEXTAREA, a bare Enter adds a line and never files, and `first line\nsecond line` reaches the POST body. The label-based tests were renamed from 'One line'.
- 2026-11-01, `./run_tests.sh -q`: RESULT PASS (parallel 383 s rc=0, serial 23 s rc=0).
