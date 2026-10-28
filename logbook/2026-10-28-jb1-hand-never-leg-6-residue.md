---
title: "JB-1 residue: the last two operator-facing strings that named the Hand by its Leg-style axis index now say hand"
type: bugfix
date: 2026-10-28
status: resolved
phase: "inbox-wall"
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/gui/js/state-minimap.js
  - tests/ros/test_teensy_bridge_node_read.py
  - map/bug-can-bridge-guard-reports-hand-as-leg-6.md
  - docs/project-map.md
subsystem:
  - teensy_bridge
  - gui
tags:
  - glossary
  - inbox-wall
---

## Summary

Bug JB-1 (filed 2026-09-21): the can-bridge Guard's operator-facing messages named
the Hand "leg 6", which on 2026-09-13 sent two Hand `MAX_DEVIATION` latches to the
wrong actuator. Surveying it on 2026-10-28 found most of it already closed by the F3
work: `teensy_bridge_node._axis_label` names axis 6 `hand` and the MAX_DEVIATION hint
and `guard_fault_leg` path go through it; the firmware itself formats no "leg N"
string. Two consumers still bypassed it:

- `teensy_bridge_node.py`, the CAN_BUS_DOWN edge ERROR: printed `leg {can_fault_leg}`
  from the raw axis id. Now `leg unknown` for 0xFF, else `_axis_label(axis)`
  (legs keep `leg N`, the Hand says `hand`).
- `ros_ws/gui/js/state-minimap.js`, the Clear Guard Latch tooltip: appended
  `', leg ' + guard_fault_leg`, which is an axis id. `'6'` now renders `hand`.

No wire-format change, no PROTOCOL_VERSION bump, no flash: the text never originated
in firmware.

## Verification

- 2026-10-28, `python -m pytest tests/ros/test_teensy_bridge_node_read.py -q -k can_bus_down`
  (new `test_can_bus_down_edge_names_the_hand_not_leg_6` asserts `CAN_BUS_DOWN — hand silent`
  and no `leg 6`): **3 passed, 62 deselected in 2.66 s**.
- 2026-10-28, `node --check ros_ws/gui/js/state-minimap.js`: syntax ok.
- Full gate: see the inbox-wall merge commit's message (one `./run_tests.sh` covers the batch).
