---
title: "GUI quick capture: a + in the Robot | Map bar files a bug, idea or feature in the map"
type: feature
date: 2026-10-25
status: resolved
phase: "map-improvements - quick capture"
related_plan: ""
files_changed:
  - ros_ws/gui/index.html
  - ros_ws/gui/js/quick-capture.js
  - ros_ws/gui/css/app-tabs.css
  - tests/ros/test_gui_quick_capture.py
subsystem:
  - gui
tags:
  - ux
  - project-map
---

## Summary

A `+` button (same styling as Light / A- / A+) in the Robot | Map bar opens a compact popover: kind (bug default | idea | feature), title (required), optional note. Enter in the title files; Esc or Cancel closes. File does `GET /api/v1/projects` then `POST /api/v1/projects/{[0].id}/tasks` on the map origin (host of the GUI, port 8765, as app-tabs.js derives the iframe URL) with `status: inbox`, `X-Dev-User: harrison`, and a body of the note plus `Filed from the Robot tab at <local ISO with offset>.`. Success closes the form and shows a fading "Filed: <title>"; failure keeps the form and text with "Map server unreachable - text kept" (or the server's detail). The draft lives in localStorage `jugglebot.quickCapture.draft` (saved per keystroke, restored on open, cleared on success).

Choices: every key event inside the form stops propagation so nothing reaches the Robot tab's key handlers; focus goes to the title on open and back to the button on close. The Map iframe is not told to refresh (the map app live-updates from its own event stream). The bag name is NOT included: the GUI exposes only the camera-recorder state (camera-panel.js), not the rosbag name, so only the time is emitted. Needs the map server's CORS for the GUI origin (fork 533daa2) deployed before it works live.

## Verification

2026-10-25, `pytest tests/ros/test_gui_quick_capture.py tests/ros/test_gui_app_tabs.py -q`: 17 passed. Static parsing only; no browser run. Gate 2026-10-26, `./run_tests.sh`: RESULT PASS (parallel 326 s rc=0, serial 22 s rc=0, total 348 s); its first run caught the unmarked wall-clock read in test_gui_clock_contract, fixed with a marker.
