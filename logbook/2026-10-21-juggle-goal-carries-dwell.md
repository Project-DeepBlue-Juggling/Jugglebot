---
title: "The Juggle goal carries dwell_s: GUI defaults it per pattern from the box (revises decision 39)"
type: feature
date: 2026-10-21
status: resolved
phase: "two-ball-oval - P1 flown-in"
related_plan: two-ball-oval.md
files_changed:
  - ros_ws/src/jugglebot_interfaces/action/Juggle.action
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/gui/js/juggle-panel.js
  - tests/ros/conftest.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_orchestrator_node.py
  - tests/ros/test_gui_juggle_defaults.py
  - tests/hardware/session_oval_rung1.md
  - plans/active/two-ball-oval.md
subsystem:
  - skills
  - gui
tags:
  - oval
  - interfaces
---

# The Juggle goal carries dwell_s

**Why.** The 2026-10-21 22:53 sitting's oval was refused: `oval_1ball refused: no admissible box covers site pair(s) -- ('P1','P2'): dwell_s: the live dwell is 0.27 s but the box covering this request was swept at dwell_s=0.65 s`. Decision 39 (2026-10-17) kept dwell a node parameter, so the operator had to `ros2 param set` it per pattern. Class closed: a per-pattern value living in a node parameter someone must remember. Owner, 2026-10-21: the goal carries it and the GUI defaults it per pattern from the box.

**What.** `Juggle.action` gains `float64 dwell_s` (0 => the node param). `skill_node._start_pattern` resolves ONE dwell per goal (`goal.dwell_s or param`) and every downstream path (columns, hop, one-ball, `reload`, box selection) uses that local; the resolved value is on the `juggle_start` attempt event. The orchestrator relay parses a `dwell_s=` token into the goal (omitted = 0). The GUI's `PATTERN_POINTS` gain `dwell_s` (columns family / hop / self_toss 0.27, oval 0.65) and the read-only Dwell span became an editable field sent on the wire. The box refusal text (in gated `admissible.py`) is rewritten in `skill_node` to point at the goal field. `skills/check` still judges the NODE param (it has no goal).

**Enforcement.** `tests/ros/test_gui_juggle_defaults.py::test_point_dwell_equals_the_boxs_swept_dwell_per_pattern`: a re-sweep at a new dwell fails the GUI default until `PATTERN_POINTS` follows it.

**Deploy.** Interfaces changed: `colcon build` of BOTH packages, a FULL launch restart and a GUI hard refresh before the next sitting.

## Verification
See the test triples in the commit message / handoff (2026-10-21, scoped `pytest tests/ros/...`).
