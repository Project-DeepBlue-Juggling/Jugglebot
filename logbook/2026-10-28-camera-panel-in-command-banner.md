---
title: "Robot tab: the Wave camera panel joins the command banner (left end), buttons centred to the Trail control, panel hidden in replay (JB-74)"
type: feature
date: 2026-10-28
status: resolved
phase: "gui"
files_changed:
  - ros_ws/gui/index.html
  - ros_ws/gui/css/camera-panel.css
  - ros_ws/gui/css/viewer.css
  - ros_ws/gui/css/replay.css
  - ros_ws/gui/js/commands.js
  - ros_ws/gui/js/replay/ui/lobby.js
  - ros_ws/docs/wave_camera_can_adapter.md
  - tests/ros/test_gui_record_panel.py
  - map/feature-move-the-wave-camera-panel-to-be-in-the-same-banner-as-the-h.md
  - docs/project-map.md
subsystem:
  - gui
tags:
  - wave-camera
  - replay
---

## Summary

Owner ask JB-74. `#camera-panel` is no longer an absolute box above the strip beside the collapsed minimap. It is now the first child of `#command-overlay`, the bottom banner. The five command buttons now go into a new `#command-buttons` group (`commands.js` appends there, falling back to the overlay for the node harness). `margin: 0 auto` on the group centres it between the camera panel and the Trail control's reserved padding. The bar is now `space-between`, so a wrapped row holding only the camera panel stays at the left end; `.replay-lobby` centres itself with auto margins. `--trail-bar-w` went from 30 % to 25 %, so camera panel + buttons fit on one row down to a ~860 px viewer (measured at 888 px: an 18 px gap on each side of the group). On a narrow viewer, camera on row 1 with the status ellipsised (`min-width: 0`), buttons centred below, Trail clear. REC/STOP semantics, topics and the DOM-free core are unchanged.

**Hide rule:** `css/replay.css` hides `#camera-panel` and `#command-buttons` on `#command-overlay.replay-active` / `.replay-lobby-on`, the same rule that used to hide the bare `.cmd-btn` children. It also hides the panel on `body.replay`, the fence's own class from `replay/fence.js`. So the panel is gone during review, and while disconnected it hides with the buttons, since the lobby says the GUI is disconnected. Before the first failed connect, it still reads "ROS disconnected" beside the disabled buttons.

## Verification

- Screenshots (fake in-page rosbridge via Playwright `routeWebSocket`, IDLE + an IDLE camera heartbeat; replay = `setReplayFence(true)` + `.replay-active` + a dock stand-in): `temp/screenshots/jb74/{before,after}_w{1600,1100}_{live,replay,lobby}.png` (viewer 888 px and 388 px).
- 2026-10-28, `pytest tests/ros/ -q -k gui`: 487 passed, 1 skipped.
- 2026-10-28, `./run_tests.sh`: exit 0. Parallel phase 7362 passed, 13 skipped in 384.73 s; serial tail 5 passed.
