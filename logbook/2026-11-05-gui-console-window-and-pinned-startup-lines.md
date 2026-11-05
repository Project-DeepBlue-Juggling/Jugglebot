---
title: "GUI CONSOLE shows its window (lines since HH:MM:SS, N dropped) and pins the bridge's one-shot FW_CHECK lines against ring eviction"
type: feature
date: 2026-11-05
status: complete
phase: "JB-163"
files_changed:
  - ros_ws/gui/js/console-logic.js
  - ros_ws/gui/js/console-view.js
  - ros_ws/gui/css/panels.css
  - tests/ros/js/console_harness.js
  - tests/ros/test_gui_console.py
  - logbook/2026-11-05-gui-console-window-and-pinned-startup-lines.md
  - logbook/INDEX.md
subsystem: gui
---

# JB-163: CONSOLE search missing BRIDGE_FW_CHECK / PLATFORM_FW_CHECK

**What.** Owner's report: searching "v38" or "v7" did not find the BRIDGE and PLATFORM firmware check lines. `matchesQuery` matches their text; the lines were simply not in the CONSOLE. Two pieces, built in the owner's order (JB-164: "do (2), then (1)"):

2. A window line above the list: `lines since HH:MM:SS (page subscribed), 0 dropped` while nothing has been evicted (the time is when the page started listening, so a check logged before it reads as outside the window), and `lines since HH:MM:SS, N dropped (ring full)` once the 5000-line ring has evicted (time = the oldest ring line). `windowLabel` in `console-logic.js`; element `#console-window`.
1. Messages containing `_FW_CHECK` (the bridge's `BRIDGE_FW_CHECK:` and `PLATFORM_FW_CHECK:` lines, read from the bridge node source) go to a pinned set of at most `PIN_CAP` = 50 that the ring cap never evicts. `liveLines` merges pinned and ring in time order, so search, level chips and counts see them like any line. The identity lines were not pinned separately: only the `_FW_CHECK` texts were found as one-shot startup logs.

**Audit fixes (2026-11-06, JB-173 answered "Agreed", option 1).** (a) `st.dropped` stays the total shown as "N dropped", but ring and pinned evictions are now counted separately (`droppedRing`, `droppedPinned`), and "(ring full)" shows only when `droppedRing > 0`; before, a pinned-cap overflow alone printed "(ring full)". (b) Once the ring has evicted and a pinned line is strictly older than `ring[0]`, the label appends ", startup lines pinned", since "lines since HH:MM:SS" would otherwise overstate the window; a pinned line with the same timestamp as `ring[0]` is inside the window and adds nothing. (c) Tests added for the pinned-cap-only label (PIN_CAP+7 lines, no "(ring full)"), ring-only eviction, pinned older than the window, and the equal-timestamp tie (label and `liveLines` order, pinned first).

**Why / Discussion.** The owner's point: they searched during a Sitting that had only just begun, with far fewer than 5000 messages, so the 2026-11-03 miss was probably the late subscription (`/rosout` is volatile on Foxy, so a page that connects after the check never sees it), not eviction. Option 2 makes that case visible; option 1 only covers eviction and cannot recover a line that arrived before the page subscribed. The subscribe time is taken when the console view initialises (page load), not on a rosbridge reconnect, so a reconnect gap is not shown. GUI display only; nothing on a robot, control or telemetry path.

**Verification.** 2026-11-05, `pytest -q tests/ros/test_gui_console.py tests/ros/test_gui_traffic_reset.py`: **12 passed in 2.03 s**. The new test (`test_window_label_and_pinned_lines`: drop count at the cap, window label both forms, a pinned line surviving RING_CAP+10 later lines, the pinned bound, search and level filter finding it) was confirmed red with the old `console-logic.js` restored (the harness could not load the new functions: 8 errors) and green with the change. The window line was not seen on a live browser. Scoped run, 2026-11-05, `pytest -q tests/ros/test_gui_console.py tests/ros/test_gui_clock_contract.py tests/ros/test_gui_geometry.py tests/ros/test_gui_traffic_reset.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py`: **321 passed, 41 skipped in 6.82 s**. Audit-fix run, 2026-11-06, `pytest -q tests/ros/test_gui_console.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py`: **26 passed in 0.97 s**. Red check: with HEAD's `console-logic.js` restored the label test fails (first at the pinned-cap-only case, which printed "(ring full)"), green again with the change restored; the ring-only and tie cases pass on the old code too, as regression guards for the new branch (the tie pins the strict `<`).
