---
title: "IDEA-4: a read-only CONSOLE view (/rosout) beside ROS | UDP in the GUI's topics panel, live and in replay: severity chips, search with highlights, click a line or prev/next to seek the replay"
type: feature
date: 2026-11-01
status: complete
phase: gui
related_plan:
  - gui-rosbag-replay.md
related_entries:
  - 2026-11-01-night-shift.md
files_changed:
  - ros_ws/gui/js/console-logic.js
  - ros_ws/gui/js/console-view.js
  - ros_ws/gui/js/udp-traffic.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/replay/allowlist.js
  - ros_ws/gui/js/replay/policy.js
  - ros_ws/gui/replay/schema.py
  - ros_ws/gui/index.html
  - ros_ws/gui/css/panels.css
  - tests/ros/test_gui_console.py
  - tests/ros/js/console_harness.js
  - tests/ros/test_gui_traffic_reset.py
  - map/idea-rosbag-search.md
  - logbook/INDEX.md
---

# IDEA-4: the CONSOLE view

**What the owner asked** (TKT-25): option 2, a search box in the GUI replay, placed as a third
button beside "ROS | UDP" called CONSOLE; also in the live GUI; read-only; filterable by severity.

**What it is.** The topics panel's segmented control is now ROS | UDP | CONSOLE (persisted under
the existing `jugglebot-topics-mode` key). CONSOLE shows `/rosout` (`rcl_interfaces/msg/Log`):
per entry a row of time, level badge and node name, then the message, wrapping. Five severity
chips (DEBUG to FATAL) toggle and persist, each with a count of the lines matching the search; the
search is a case-insensitive substring over node and message, highlighted with `<mark>`, not
persisted (a stale query hid lines after a page refresh). Auto-scroll follows the bottom until the
operator scrolls up; "Jump to latest" returns. The list is resizable. Every string goes through
`escapeHtml` / `highlightHtml`; nothing is published to ROS.

**Live.** `main.js` subscribes to `rosout` through the shared rosbridge connection from page load
into a 5,000-line ring (about 8 minutes at the measured rate) and renders only while CONSOLE is
shown, on a 250 ms timer. The `/rosout` rate in the last six bags was 5-11 msg/s (peak 11.4,
2026-10-28 21:02), so no throttling was added.

**Replay.** The view follows the replay mode and loads `/rosout` per slot through the source's
existing filtered lite lane (`source.load(i, {lite: true, topics: ['/rosout']})`), the one the
ball history uses: no second MCAP reader. `/rosout` joins the replay allowlist in three places
(`replay/schema.py`, `js/replay/allowlist.js`, `js/replay/policy.js` as an event-class topic).
Clicking a line seeks the trackbar to its time; prev / next jump to the nearest matching line
before / after the playhead; the line at the playhead is highlighted as it moves. The pure logic
(filters, search, ring, hit selection) is in `console-logic.js`, the DOM in `console-view.js`.

**Verification.**
- Scoped, 2026-11-01: `pytest tests/ros/test_gui_*.py tests/ros/test_replay_*.py tests/ros/test_rosbridge_websocket_lean.py -n 4 --dist loadfile` -> 677 passed, 1 skipped; after the layout fixes `pytest tests/ros/test_gui_console.py tests/ros/test_gui_traffic_reset.py` -> 9 passed.
- Headless Chromium (CDP, throwaway driver, worktree server on :8099), bag 2026-10-31_13-13-19 (125 `/rosout` lines): all lines rendered, chips DEBUG 41 / INFO 82 / WARN 1 / ERROR 1; INFO off 125 -> 43 lines; search "INSTALL" 11 matches with highlights; clicking a line moved the playhead to that line's time exactly; prev / next moved to matching lines; ROS -> UDP -> CONSOLE titles and persistence correct; live mode with no rosbridge shows the empty state; zero page errors. Dark and Light backgrounds checked.
- Gate, 2026-11-01: `./run_tests.sh` -> 7906 passed, 51 skipped in 401.14 s; serial 5 passed; exit 0.
- Not yet seen by the owner at real size; the live service needs a restart to pick it up.
