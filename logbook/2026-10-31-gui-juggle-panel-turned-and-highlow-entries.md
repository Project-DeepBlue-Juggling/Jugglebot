---
title: "GUI Juggle panel: Columns — turned and Columns — high-low entries (pattern-progression P3/P5 operator path); the parameter-backed goals write columns_angle_deg / apex_m_ball2 to skill_node before dispatch"
type: feature
date: 2026-10-31
status: complete
phase: "pattern-progression P3/P5 prep"
related_plan:
  - pattern-progression.md
related_entries:
  - 2026-10-31-pattern-progression-p2-turned-columns.md
  - 2026-10-31-pattern-progression-p4-high-low-columns.md
files_changed:
  - ros_ws/gui/js/juggle-panel.js
  - ros_ws/gui/test_juggle_panel.html
  - tests/ros/test_gui_juggle_defaults.py
  - plans/active/pattern-progression.md
  - map/idea-jp-9-high-low-columns-ball-1-at-apex-1-05-m-ball-2-at-0-95-m.md
  - map/idea-jp-10-turned-columns-the-columns-pair-at-fixed-angles-about.md
  - docs/runbook.md
subsystem:
  - gui
tags:
  - patterns
  - columns
  - gui
---

# GUI Juggle panel: entries for the two new Columns goals

## What changed

The owner asked (2026-10-31) for the two goals landed offline the night before, `columns_turned` (JP-10, map JB-108) and `columns_highlow` (JP-9, map JB-107), to be startable from the GUI instead of a terminal. Both goals read a **node parameter** that is not a goal field (`columns_angle_deg`, `apex_m_ball2`; the relay `orchestrator_node._parse_juggle_request` refuses any token it does not know), so the panel gained a second kind of field:

- Two pattern rows, `Columns — turned` and `Columns — high-low` (labels as in `records.PATTERNS`), each naming its `param`. Pre-fill from the 2026-10-31 sweep: turned at Apex 0.95 / Sep. 125 / Dwell 0.27 / Angle 45; high-low at Apex 1.00 / Sep. 100 / Dwell 0.27 / Apex 2 0.90.
- Two parameter-backed fields, Angle (deg) and Apex 2 (m), shown only for their pattern (the other patterns' field grid is untouched) and **required** there: a blank one is a Start error, never "node default", because the node's value would be whatever the previous goal left and would fly silently under a blank field.
- Start on such a pattern first writes the typed value through `/skill_node/set_parameters` and dispatches the request only when the node answers `successful` (the node refuses the write while a goal runs). A refused or unanswered write dispatches nothing and says so on the status line. The value never rides the request string.
- The `juggle_start` event's `angle_deg` / `apex_m_ball2` keys (skill_node ATTEMPT_START, this goal only) feed the running line and the stale-relay cross-check.
- Diagram: each ball flies its own apex (the low ball lands early and sits through its longer dwell; timing follows the higher ball); turned Sites are labelled `P1@45` as the node names them.
- Harness `test_juggle_panel.html`: a fake `/skill_node/set_parameters` (accept / refuse / never answer, `?param=`), the echo of the written value on the fake `juggle_start`.
- Found by the headless run, fixed in the same change: `buildField` built the input id with a single-underscore replace, so `apex_m_ball2` would have become `juggle-apex-m_ball2`; it now replaces every underscore (the four existing ids have one underscore each and are unchanged).

No Python changed: the relay passes the pattern name through and `skill_node` already serves both goals. The plan's P5 text and both Items' `## Sitting` sections now lead with the GUI path and keep the terminal commands as the fallback.

## Verification

- 2026-10-31, `node --input-type=module --check < ros_ws/gui/js/juggle-panel.js`: syntax ok.
- 2026-10-31, `pytest -q tests/ros/test_gui_juggle_defaults.py tests/ros/test_gui_replay_ui_hide.py`: **19 passed** (two new pins: the pre-filled Angle and Apex 2 sit in admissible cells of their own; the param write precedes dispatch and never reaches the request string; the pattern count is 9).
- 2026-10-31, headless Chromium against the harness page served by `python -m http.server` from `ros_ws/gui` (`/snap/bin/chromium --headless=new --virtual-time-budget=10000 --dump-dom`, DOM dumps and screenshots in `temp/gui_harness/`): `columns_highlow … scene=dispatch&param=ok` logs set_parameters `apex_m_ball2 = 0.9`, then the request `columns_highlow,apex_m=1,separation_mm=100,dwell_s=0.27`, and the running line reads `apex 2 0.900 m`; `columns_turned … param=refuse` logs the refused write and NO request; `columns_turned … param=ok` dispatches and the running line reads `angle 45 deg`, Site labels `P1@45` / `P2@45`; plain `columns` is unchanged (`P1` / `P2`).
- 2026-10-31, `./run_tests.sh` in the scratchpad worktree, log `temp/logs/gui_entries_gate.log`: **parallel 7814 passed / 63 skipped in 372.00 s, serial 5 passed in 19.78 s, exit 0** (JavaScript is outside the suite: the headless run above is its runtime check).
