---
title: "Quick capture moves left of the bar as '+ File new change request' (JB-65); quick capture closed on its first live filing"
type: bugfix
date: 2026-10-26
status: resolved
phase: "map-improvements"
files_changed:
  - ros_ws/gui/index.html
  - ros_ws/gui/css/app-tabs.css
  - ros_ws/gui/js/quick-capture.js
  - tests/ros/test_gui_quick_capture.py
subsystem:
  - gui
tags:
  - project-map
---

## Summary

The owner's first filing from the live GUI was JB-65: "Move the button to the LHS, right of the 'Map' text, and make it say '+ File new change request'". It went from the "+" through the map app's `create`, which allocated its ID, and reached `main` as `022167af`. That filing is the check quick capture was waiting for, so `feature-quick-capture-files-a-map-item-from-the-robot-tab` closes here. It is also P4's last Done-check in `plans/active/map-item-ids.md`.

The fix:
- **Placement:** the button and its toast move out of the right-hand prefs group into their own `app-tabs-capture` group, just right of the Map tab.
- **Label:** the button reads "+ File new change request". Its tooltip still says what it files.
- **Popover:** `openForm()` places the popover under the button and keeps it on-screen. The CSS default moves from `right` to `left`.
- **Phone width:** the label never wraps. Below 600 px it collapses to "+", because at 390 px the full label wrapped to three lines and overflowed the 32 px bar.

## Verification

- 2026-10-26, `pytest tests/ros/test_gui_quick_capture.py tests/ros/test_gui_app_tabs.py -q`: 22 passed. The new test pins the button between the Map tab and the prefs group, its label, the no-wrap and phone rules, and the popover placement.
- 2026-10-26, headless Chromium 149 against the bar served from this tree (filing not exercised):
  - **1500 px:** the button starts 8 px right of the Map tab (124 -> 132) on one line. The popover opens at x 132 with focus in the title.
  - **390 px:** the button shows "+" (20 px wide). The popover stays on-screen (106-386).
- The full gate result is in the commit message. The live GUI needs a hard refresh once the live checkout has this commit.
