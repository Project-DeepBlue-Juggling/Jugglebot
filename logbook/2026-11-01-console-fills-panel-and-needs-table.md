---
title: "GUI CONSOLE fills the topics panel's own box (CAN TRAFFIC stays put across ROS / UDP / CONSOLE); the map app's Needs a Sitting section is an invisible table"
type: feature
date: 2026-11-01
status: complete
phase: gui
related_entries:
  - 2026-11-01-jb36-gui-console-view.md
  - 2026-11-01-runbook-grouped-by-rig.md
files_changed:
  - ros_ws/gui/css/panels.css
  - logbook/INDEX.md
---

# CONSOLE fills the panel; the needs list aligns

**What the owner asked.** (1) The CONSOLE view as large as possible by default, with CAN TRAFFIC
where it sits in the ROS and UDP views. (2) The Next Sitting page's "Needs a Sitting section"
aligned as though in an invisible table.

**CONSOLE.** `#left-sidebar #panel-topics` is `flex: 1; min-height: 0` (theme.css): it takes the
sidebar's height left after CAN TRAFFIC, and the ROS and UDP views stretch inside it. The CONSOLE
view (2026-11-01, JB-36) overrode that with `.console-on { flex: 0 0 auto }` and a fixed 220 px,
user-resizable list, so the panel sized to its content and CAN TRAFFIC moved. The override is gone;
`#console-container` and `.console-list` are `flex: 1; min-height: 0`, so the list takes the
panel's remainder after the header, chips, search row and count. The resize handle is dropped: in
a fixed box it could only push CAN TRAFFIC.

**Needs table** (map app fork, c0e934f + the no-wrap follow-up). A borderless table, one row of
five cells per entry (rig chip, kind pill, ID, title, reason), so each kind of text shares a
column; chip and reason cells never wrap.

**Verification** (2026-11-01).
- Headless Chromium on a worktree server (:8099, bag 2026-10-31_13-13-19), `getBoundingClientRect`: CAN TRAFFIC top 318 px at 1500x900 and 498 px at 1920x1080 in all three modes; the topics panel 44 / 262 and 44 / 442 in all three; console list 160 and 340 px (the panel remainder); no page errors in replay or in live mode with no rosbridge.
- `pytest tests/ros/test_gui_console.py tests/ros/test_gui_traffic_reset.py tests/ros/test_gui_app_tabs.py tests/ros/test_gui_geometry.py` -> 154 passed; `./run_tests.sh` -> 7920 passed, 48 skipped in 413.61 s; serial 5 passed; exit 0.
- Map app: typecheck and lint clean, vitest 332 passed (new assertion: five cells per needs row); deployed, and a headless screenshot of the live page shows one aligned line per entry.
