---
title: "FEAT-35: hovering a CONSOLE line in a replay draws a thin dashed marker on the trackbar at that line's time"
type: feature
date: 2026-11-02
status: complete
phase: gui
related_entries:
  - 2026-11-01-jb36-gui-console-view.md
files_changed:
  - ros_ws/gui/js/console-logic.js
  - ros_ws/gui/js/console-view.js
  - ros_ws/gui/js/replay/mode.js
  - ros_ws/gui/js/replay/ui/trackbar.js
  - ros_ws/gui/css/replay.css
  - tests/ros/test_gui_console.py
  - tests/ros/js/console_harness.js
  - tests/ros/test_gui_replay_ui_trackbar.py
  - tests/ros/js/replay_ui_trackbar_harness.js
  - logbook/INDEX.md
  - map/feature-hovering-on-a-console-message-in-a-recording-replay-should-i.md
  - docs/project-map.md
---

# FEAT-35: CONSOLE hover marks the trackbar

**What.** In a replay, hovering a CONSOLE row shows a thin dashed amber line (`--replay-hover-marker`,
next to `--replay-playhead`) on the trackbar at that line's time, so the operator sees where in the
recording the message sits. Moving onto another row moves it; leaving the rows clears it. Hidden when
the time is outside the current zoom range. Live mode never publishes (no trackbar).

**Seam.** A new `marker` event on the replay mode singleton: `mode.setMarker(t|null)` emits it,
the trackbar subscribes and draws it in its existing per-frame `frame()` (same cache-and-write-on-change
pattern as the playhead, reusing overview.js `frac`). `console-view.js` never touches the trackbar's
DOM. Pure `hoverTime(target)` in `console-logic.js` maps an event target to a finite row `data-t` or null.
Nothing is persisted or published to ROS; leaving replay clears the marker once.

**Audit follow-up (owner-approved, 2026-11-02).** The Night Shift held the branch on one audit
note: hiding the CONSOLE view by keyboard with the mouse resting on a row left the marker drawn,
because the browser sends no `mouseout` for a node that stops being shown. `setConsoleVisible(false)`
now clears it (`setMarker(null)`, a no-op outside replay). Pinned in `console_harness.js` /
`test_hover_publishes_only_in_replay`; with the fix reverted that test fails (2026-11-02,
`python -m pytest -q tests/ros/test_gui_console.py`: 1 failed, 8 passed; with it, 9 passed).

**Verification.** (2026-11-02,
`python -m pytest -q -p no:cacheprovider tests/ros/test_gui_console.py tests/ros/test_gui_traffic_reset.py tests/ros/test_gui_replay_ui_trackbar.py tests/ros/test_gui_replay_mode.py tests/ros/test_gui_trails_wiring.py tests/ros/test_gui_replay_chart.py tests/ros/test_gui_replay_rename_live.py`,
**77 passed in 9.63 s**). With the `ros_ws` changes stashed, the new tests fail (both node harnesses
abort on the missing `hoverTime` / `hover marker` support: 27 errors, 2 passed). The docs and map
paths this commit touches (this entry, `logbook/INDEX.md`, the Item closed, the regenerated
`docs/project-map.md`) are read by (2026-11-02,
`python -m pytest -q -p no:cacheprovider tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_mapkit_docs.py tests/sim/test_map_items.py`,
**36 passed in 1.77 s**). The full gate is the orchestrator's (Night Shift 2026-11-02).
