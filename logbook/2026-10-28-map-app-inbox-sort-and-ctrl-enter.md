---
title: "Map app: Inbox wall sort options (JB-77) and Ctrl+Enter creates the item from the app modal and the Robot tab quick-capture form (JB-78)"
type: feature
date: 2026-10-28
status: resolved
phase: "map-improvements"
files_changed:
  - ros_ws/gui/js/quick-capture.js
  - ros_ws/gui/index.html
  - tests/ros/test_gui_quick_capture.py
  - map/feature-sort-options-for-inbox-wall-sort-by-type-age-severity.md
  - map/bug-ctrl-enter-to-create-item-either-from-new-or-from-file-new-c.md
  - docs/project-map.md
subsystem:
  - gui
  - project-map
tags:
  - map-improvements
  - inbox-wall
---

## Summary

Two owner-filed items from the Robot tab, worked by one Opus agent on 2026-10-28.

**JB-77, Inbox wall sort.** The map app's Inbox wall header gains a "Sort: Type · Age ·
Severity" radiogroup. The comparators are pure functions in `frontend/src/lib/wallSort.ts`
(type = bug, feature, idea then newest first; age = newest filed first, ties by id;
severity = high > medium > low > none then newest first). The viewer's choice is kept in
`localStorage` (`jugglebot-map-inbox-sort`, try/catch). Default is Severity, which differs
from the old fixed order only within one priority: ties now go newest-first instead of
kind then title. Fork commit `58865fa`.

**JB-78, Ctrl+Enter creates.** One helper, `isSubmitChord` in `ui/keyboard.ts`, serves
the drawer's save and the New-item modal's create; the chord fires from any control in
the form, a ref stops a double press filing twice, and the shortcuts dialog lists it.
Fork commit `ea405ff`. The "+ File..." entry is NOT that modal: it is the Robot GUI's own
quick-capture form (`ros_ws/gui/js/quick-capture.js`), so Ctrl/Cmd+Enter there now calls
its existing `file()` from the form's keydown handler, and the File button's tooltip names
the chord. That half lives in this repo (this entry's commit).

Fork `main` fast-forwarded to `ea405ff`, CI run 40611698983 green (python, frontend,
no-upstream-name), redeployed on the Jetson with `deploy-jetson.sh` (health ok, :8765 200).

## Verification

- 2026-10-28, fork `frontend/`: `npm run lint` exit 0; `npm run typecheck` exit 0;
  `npx vitest run`: **294 passed, 1 skipped**; `npm run build` exit 0.
- 2026-10-28, fork root: `MAPKIT_SRC=~/Desktop/Jugglebot/mapkit python -m pytest -q`: **69 passed**.
- 2026-10-28, agent worktree: `./run_tests.sh`: **2 failed, 7360 passed in 552 s**; both failures
  pass scoped (`pytest -q tests/ros/test_skill_node.py::test_a_slow_test_run_cannot_age_the_fixture_sample_past_the_accept_check tests/ros/test_teensy_bridge_node_setpoint.py`: 36 passed in 11.6 s).
  The first is the listed JB-64 load-flake; the second,
  `test_teensy_bridge_node_setpoint.py::test_hand_step_violation_not_sent` (`assert 2 == 1`), is a
  NEW load-flake on the hand-safety path and is filed as its own Bug.
- Full gate on the merged batch: see the inbox-wall merge commit's message.
