---
title: "Records ledger v2: each record keeps the run's parameters, shown on hover in the GUI records panel (FEAT-29)"
type: feature
date: 2026-11-01
status: complete
phase: gui
files_changed:
  - ros_ws/src/jugglebot/jugglebot/records.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - docs/records.md
  - ros_ws/gui/js/records-model.js
  - ros_ws/gui/js/records-panel.js
  - tests/motion/test_records.py
  - tests/ros/test_skill_node.py
  - tests/ros/js/records_hover_place_harness.js
  - tests/ros/test_gui_records_hover_place.py
  - map/feature-hovering-over-a-record-should-show-that-run-s-parameters-for.md
  - docs/project-map.md
  - logbook/INDEX.md
subsystem:
  - gui
  - skills
tags:
  - records
---

# FEAT-29: a record's run parameters on hover

**What.** The ledger (`docs/records.md`, IDEA-3) kept when, Throws, Catches and the end code, and
nothing about how the Attempt was configured. Owner's choice (2026-11-01): save the resolved goal
values plus each Pattern's extras.

- `records.py` format v2: a last `Parameters` column, `key=value` pairs joined by `; `, with an
  empty cell meaning not recorded. `parse` still reads v1, and `render` always writes v2, so
  `docs/records.md` is re-rendered here: marker, one prose sentence, and an empty cell per
  existing row, with every Note kept. `|`, `;` and `=` are sanitised to `/`. `to_json` records
  gain `params` (`{}` when not recorded).
- `skill_node._start_pattern` captures the params at accept (`_goal_record_params`), after each
  value resolves from the goal or the node param. `_offer_record` writes that tuple, so a node
  param changed mid-Attempt does not alter the row. Captured per Pattern: `apex_m`,
  `separation_mm` (all but the single-Site `self_toss`), `dwell_s`, `turned_angle_deg`
  (columns_turned), `apex_m_ball2` (columns_highlow) and `planner`. `num_cycles` is left out (owner).
  The capture never raises into the goal. It touches only bookkeeping after the schedule compiles,
  with no motion or timing path.
- GUI: a row's hover gains a second line, `Apex 0.95 m · Separation 125 mm · Dwell 0.27 s ·
  Planner pose`, or `Parameters not recorded` for rows set before v2 (every record today).

**Deploy order.** `skill_node` runs from the colcon install, and the GUI server loads
`records.py` once at start. Both must pick up v2 before the next Sitting: an old `records.py`
refuses the v2 file, so a record would be logged as "not updated" and `/api/records` would 500.
So: `colcon build --packages-select jugglebot` and restart `jugglebot-gui.service`.

## Verification

- 2026-11-01, `pytest tests/motion/test_records.py tests/ros/test_gui_records_api.py tests/ros/test_gui_record_panel.py tests/ros/test_gui_records_panel.py tests/ros/test_replay_stdlib.py -q`: 83 passed.
- 2026-11-01, `pytest tests/ros/test_skill_node.py tests/motion/test_records.py -q`: 433 passed in 63.7 s.
- 2026-11-01, `pytest tests/ros/test_gui_records_hover_place.py -q` (the hover line): part of the 16 passed in the BUG-38 entry.
