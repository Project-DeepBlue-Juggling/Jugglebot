---
title: GUI replay - hovering the speed pill opens speed presets (±0.5, ±1, ±2) above it
type: feature
date: 2026-10-13
status: resolved
phase: "gui-rosbag-replay - follow-up (owner ask)"
related_plan: gui-rosbag-replay.md
files_changed:
  - logbook/2026-10-13-gui-replay-speed-hover-menu.md
  - logbook/INDEX.md
  - ros_ws/gui/css/replay.css
  - ros_ws/gui/js/replay/ui/trackbar.js
  - tests/ros/js/replay_ui_trackbar_harness.js
  - tests/ros/test_gui_replay_ui_trackbar.py
subsystem:
  - gui
tags:
  - feature
---

# GUI replay - hovering the speed pill opens speed presets above it

## Change

Owner ask: stepping the FF/RW ladder to reach a playback speed was tedious. The trackbar's speed pill
(`#rp-speed`) now sits in a `.rp-speed-wrap`; hovering it opens a column of presets above the pill —
×2, ×1, ×0.5, -×0.5, -×1, -×2 (`SPEED_MENU` in `trackbar.js`, labels in the pill's own `fmtSpeed` format).
A click does what FF/RW do: `engine.setSpeed(x)` then `play()`. The current speed is highlighted (`.on`, rewritten
only when the speed readout changes, so no per-frame work). Show/hide is pure CSS (`:hover`); the menu's
`padding-bottom` bridges the pill-to-menu gap so the pointer does not drop hover on the way up. All six values are
already on the engine's `SPEED_LADDER`, so `snapSpeed` passes them through unchanged. FF/RW and the ladder are untouched.

## Verification

- `pytest tests/ros/test_gui_*.py -q -n 4` (cloud container venv, run 2026-10-13): **440 passed, 1 skipped**
  (392 + the 48 in the four rosbags-dependent modules, run separately after installing `rosbags`). New:
  `test_speed_pill_hover_menu` (labels/order, current-speed highlight, click → `setSpeed:x` + `play`).
- Headless Chromium against the real `replay.css` + `trackbar.js` with a fake engine (run 2026-10-13): hover
  shows the six presets above the pill with -×0.5 highlighted; clicking -×2 set speed -2 and the pill read -×2;
  moving the pointer away hid the menu.
- Gate (`PDJ_VENV_PY=$(which python3) ./run_tests.sh`, cloud container, run 2026-10-13): 5913 passed, 12 skipped,
  380 failed + 274 errors; the failed/errored set is **identical** with this change stashed (5912 passed), all
  missing modules in that container (`quaternion`, `can`, `sensor_msgs`, `casadi`, `qtm_rt`).
