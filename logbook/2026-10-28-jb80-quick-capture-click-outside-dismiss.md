---
title: "JB-80: a click outside the Robot tab's + File new change request form closes it without filing"
type: bugfix
date: 2026-10-28
status: resolved
phase: "inbox-wall"
files_changed:
  - ros_ws/gui/js/quick-capture.js
  - tests/ros/test_gui_quick_capture.py
  - map/bug-clicking-off-this-menu-file-new-change-request-should-close.md
  - docs/project-map.md
subsystem:
  - gui
tags:
  - inbox-wall
---

## Summary

The quick-capture form closed on Esc, Cancel and the "+" toggle but not on a click
elsewhere. `quick-capture.js` now has a capture-phase `pointerdown` listener on
`document` that calls `dismiss()` when the form is open and the target is outside both the
form and the "+" button (the toggle handles its own click, so no double-toggle). It
reuses `dismiss()`, so it is busy-guarded: an in-flight filing is never interrupted and a
late failure stays visible. It never calls `file()`. The form's key-event isolation is
untouched.

Draft retention (unchanged, preserved): the draft (kind/title/note) is written to
localStorage on every keystroke and restored on open, cleared only on a successful filing.
So a click-outside dismissal does not file, and reopening shows the same text again.

The test is a static-markup contract (the file's existing style; the dismiss logic is
DOM-bound, so no node harness): it pins the capture-phase listener, the three-way outside
predicate and the busy-guarded `dismiss()`. No headless-browser check was run.

## Verification

- 2026-10-28, `pytest tests/ros/test_gui_quick_capture.py -q`: 14 passed.
- 2026-10-28, `./run_tests.sh` (gate, before the logbook/map edits): PASS, parallel 401 s rc=0, serial 23 s rc=0.
