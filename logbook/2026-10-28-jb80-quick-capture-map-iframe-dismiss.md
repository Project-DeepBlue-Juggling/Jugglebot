---
title: "JB-80 reopened: the quick-capture form now also closes on a click inside the Map iframe"
type: bugfix
date: 2026-10-28
status: resolved
phase: "inbox-wall"
files_changed:
  - ros_ws/gui/js/quick-capture.js
  - tests/ros/test_gui_quick_capture.py
subsystem:
  - gui
tags:
  - inbox-wall
---

## Summary

The owner reported, after a service restart and a hard refresh of the page, that the form closed only on a
click inside the Robot | Map tab bar. `quick-capture.js` gains a `window` `blur` listener
that dismisses (via the busy-guarded `dismiss()`, one tick deferred) when focus has moved to
an `<iframe>`: the Map pane, whose clicks never reach the parent document.

## Discussion

Wrong theory withdrawn: the first fix (06a3a552) assumed the capture-phase `document`
`pointerdown` listener could be defeated by Robot-pane handlers, and the reopen brief
suspected it was. A headless Chromium probe against the live GUI at :8081 (served file
confirmed to contain the listener, `Cache-Control: no-cache`) shows it is not: real mouse
clicks on the 3D viewer, the chart pane, the state minimap and the empty tab bar all close
the form, and a 78-point grid over the whole page left it open outside the form at zero
points. The only click that left it open was inside the Map iframe. So the Robot-pane
failure is NOT reproduced; the owner's report fits clicking while the Map tab is showing.
If it recurs on the Robot tab, capture the browser and the pane (the probe is
`temp/screenshots/jb80/`, scripts in the session scratchpad).

## Verification

- 2026-10-28, headless Chromium on live :8081 (before): viewer/chart/minimap/tab bar
  `closedAfter=true`, Map iframe `closedAfter=false` (`temp/screenshots/jb80/before_live.txt`).
- 2026-10-28, same probe on this worktree's GUI on :8097 (after): all five `closedAfter=true`
  (`temp/screenshots/jb80/after_worktree.txt`).
- 2026-10-28, `pytest tests/ros/test_gui_quick_capture.py -q`: 15 passed.
- 2026-10-28, `./run_tests.sh`: 7443 passed, 1 failed (the avoid-word scan, on the word "reload" in this entry, fixed after); scoped re-run of glossary, plans-index and quick-capture tests: 31 passed.
