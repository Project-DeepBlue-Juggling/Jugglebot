---
title: GUI — ODrive charts draw every measured trace solid and every commanded trace dashed
type: refactor
date: 2026-10-06
status: resolved
phase: two-ball-skill-stack — R5 (operator surface)
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/gui/js/telemetry-charts.js
subsystem:
  - gui
---

# GUI: measured solid, commanded dashed

## What changed (owner ask, 2026-10-06)

The line-style convention was inverted between the two measured/commanded pairs: Pos drew
measured solid and commanded dashed, while Current drew the setpoint solid and the measured
current dashed. Current now matches Pos — `iq_measured` is solid, `iq_setpoint` is dashed — and
the setpoint's label is `Current (cmd)` (was `Current (set)`), so both commanded traces read
"cmd". The two tooltips that named the dashed trace were updated to match. Signal keys,
colours, scales and data paths are unchanged, so saved signal selections still load.

Follow-up (same day, owner): the toolbar now lists `Current (meas)` before `Current (cmd)`,
so both pairs read meas-then-cmd.

## Verification

- 2026-10-06, `pytest tests/ros/test_gui_geometry.py tests/sim/test_logbook*.py -q`: **169 passed**.
  The change is JS-only; `test_gui_geometry.py` is the only test that reads `telemetry-charts.js`.
