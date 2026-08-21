---
title: Phase B audit fixes — rebind-only counters, honest docstrings, kept scroll
type: bugfix
date: 2026-08-22
status: resolved
phase: "operator-observability Phase B"
related_plan: operator-observability.md
files_changed:
  - teensy_link/client.py
  - ros_ws/gui/js/telemetry-charts.js
  - ros_ws/gui/js/udp-traffic.js
  - tests/ros/test_gui_geometry.py
  - logbook/INDEX.md
subsystem:
  - teensy_link
  - gui
tags:
  - observability
  - audit
---

# Phase B audit fixes

## Summary

The independent Phase-B audit of F1+F2 (`cddd670`, `f59eb9b`) returned no
BLOCKING and no WARNING findings — five NOTEs, all approved and applied:

1. `LinkStats` bump sites made **rebind-only** (`if key in dict`) at all three
   per-type counters, closing the last snapshot-race insert path (an
   unknown-msg_type frame from newer additive firmware — a normal operational
   state here — could still INSERT on the RX thread while `snapshot()` copied
   on the ROS timer). Unknown types now count only in the aggregates, which
   the class docstring already documented as the superset.
2. `_count_tx` docstring states the best-effort guarantee honestly (the
   read-modify-write can lose an increment across concurrent send threads —
   the same guarantee `tx_frames` always had), instead of pointing at a
   snapshot race that no longer exists.
3. `buildUPlotOpts` docstring no longer claims per-chart tick decimals (ticks
   stay on uPlot's auto formatter; per-chart decimals are callouts + Δ pills,
   exactly what plan § 4 U4 specifies).
4. `udp-traffic.js` repaint preserves `scrollTop` across the once-per-second
   `innerHTML` rebuild of the 21-row scrolling table (engine-dependent reset;
   selection loss stays for the § 8 (3b) browser eyeball to judge).
5. `test_bb_pitch_maps_onto_the_configured_range` now derives its endpoints
   from `ball_butler_pitch.deg_min/deg_max` in the YAML (was: hardcoded 90/12
   literals, algebraically implied by a sibling test) and adds a
   quarter-turn span check that a rad/deg or 180-per-rev slip would break.

## Verification

Scoped (2026-08-22, worktree, venv): `pytest tests/teensy_link/
tests/ros/test_teensy_bridge_node_udp_diag.py tests/ros/test_gui_geometry.py -q`
→ **339 passed in 12.94 s**; `node --check` clean on both touched JS files;
`node tools/probes/uplot_nan_gap_probe.js` → ALL 21 ASSERTIONS PASSED.
Phase-B closure gate: `./run_tests.sh --full` result recorded in the commit
message of this entry's commit.
