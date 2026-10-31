---
title: "GUI: the View menu's scene toggles persist across a page refresh (JB-89)"
type: bugfix
date: 2026-11-01
status: complete
phase: night-shift
related_plan:
  - night-shift.md
files_changed:
  - ros_ws/gui/js/view-toggles.js
  - ros_ws/gui/js/main.js
  - tests/ros/js/view_toggles_harness.js
  - tests/ros/test_gui_view_toggles.py
  - logbook/INDEX.md
  - map/bug-gui-persistent-view-setting-selections.md
  - docs/project-map.md
subsystem:
  - gui
tags:
  - ux
  - night-shift
---

# GUI: the View menu's scene toggles persist across a page refresh (JB-89)

## Summary

Owner bug (2026-10-28): hiding the `Jugglebot` group (the robot model) in the View menu did not survive a
GUI refresh. Every generic scene-group toggle (`sceneGroups` in `main.js`
`initSceneMenu`) reset to the scene's default on refresh. The records rows and camera
slots already persisted; these did not.

- New DOM-free `ros_ws/gui/js/view-toggles.js`: one localStorage key
  `jugglebot-view-toggles`, JSON `{groupName: bool}`. `loadViewToggles` returns `{}` for an
  absent key, garbage JSON, a non-object, or throwing storage, and drops non-boolean
  values; `saveViewToggles` swallows a blocked storage; `toggleState` returns the saved
  choice, else the scene's own default (so a group never toggled keeps its default; the stale-snapshot case across two tabs, where a save rewrites the whole map from the state read at build, is left open by the audit).
- `main.js`: when the menu is built, the saved state is applied to each group (`obj.visible`)
  before the checkbox is set, and every change writes the whole map. The records section
  and camera slots are untouched.

Display state only; no control-path change.

## Verification

- 2026-11-01, `pytest tests/ros/test_gui_records_panel.py tests/ros/test_gui_robot_assets.py tests/ros/test_gui_vendor_pin.py tests/ros/test_gui_view_toggles.py -q -p no:randomly`: 23 passed, 1 skipped in 2.89 s (the skip is in a pre-existing file; the 5 new tests ran, 5 passed alone).
- Fails without the change: `git show origin/main:ros_ws/gui/js/view-toggles.js` is `fatal: ... exists on disk, but not in 'origin/main'`, so the harness import fails and the four harness-backed tests fail; `git show origin/main:ros_ws/gui/js/main.js | grep -c view-toggles` is 0, so the source-text wiring test fails.
- Not run: the live GUI (no browser pass possible); the apply-on-build and save-on-change wiring in `main.js` is pinned by source-text checks only.
