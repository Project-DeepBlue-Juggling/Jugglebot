---
title: Telemetry charts read in physical units, and the 3D hand stops being 2.2x too long
type: feature
date: 2026-08-22
status: resolved
phase: "operator-observability F1"
related_plan: operator-observability.md
files_changed:
  - config/generate_config.py
  - config/generated/geometry-config.js
  - ros_ws/gui/js/geometry-config.js
  - ros_ws/gui/js/telemetry-charts.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/css/charts.css
  - tests/ros/test_gui_geometry.py
  - logbook/INDEX.md
subsystem:
  - gui
  - config
tags:
  - observability
  - units
  - bugfix
---

# Chart physical units (F1)

## Summary

All nine telemetry charts plotted raw motor revs, so an operator reading a leg
trace had to carry a per-leg factor in their head and a BB-pitch trace said
nothing about where the barrel actually pointed.

- **U1** `generate_gui_js` now emits `HAND_MM_PER_REV`, `BB_HAND_MM_PER_REV`
  (both **derived** from the rev/m gains `compute_derived` already produces —
  `1000 / gain`, never a pasted number) plus `BB_PITCH_DEG_PER_REV = 360` /
  `BB_PITCH_DEG_OFFSET = 90`. Regeneration touched the two
  `geometry-config.js` copies and nothing else.
- **U2** `axisUnitsFor(chartIdx)` in `telemetry-charts.js` is the one unit
  table: legs use **per-leg** `1/MM_TO_REV[i]` (the six differ by ~1.3 %), hand
  and BB hand their own spool gains, BB pitch the affine
  `deg = 90 + 360·rev` — **absolute** barrel degrees, matching the BB panel's
  `pitch_deg` and the configured 12–90° range so the two readouts compare
  directly.
- **U3** conversion happens **once, at ingestion**, applied identically to
  `pos_measured`, `vel_measured` and `pos_commanded`. That is the whole point:
  converting per-consumer is how one of the two traces eventually misses a
  factor and the chart shows a constant tracking error that does not exist.
  `NaN` survives an affine map, so the `nanGaps` pen-up semantics are untouched
  (probe re-run, below).
- **U4/U5** axis label, callouts, Δ pills, y-range pad floors (2 mm / 1° /
  5 mm·s⁻¹ / 5°·s⁻¹, replacing a hardcoded 0.5 that meant **half a rev = ±35 mm**
  on a leg) and CSV headers (`leg_0.pos_measured_mm`, `bb_pitch.pos_measured_deg`)
  all follow the same table. No in-repo consumer parses the exported CSVs.
- **U6** the 3D hand bug: `main.js` divided the hand's rev by `MM_TO_REV[0]`,
  the **leg** factor — 70.5 mm/rev instead of the hand's 31.6, so the rendered
  hand travelled **2.2×** too far and topped out at under half its commanded
  extension. Now `pos_estimate * HAND_MM_PER_REV`.

Sanity, from the emitted constants: leg 0 = 70.505 mm/rev → 4.2 rev max = 296 mm
against a 280 mm stroke; hand = 31.628 mm/rev → 10.8 rev hard stop = 341.6 mm
against a 344.75 mm stroke; BB pitch rev −0.2167 → 11.99°, rev 0 → 90.00°.

## Verification

- `./run_tests.sh` (run 2026-08-22, worktree `~/Desktop/Jugglebot-obs`):
  parallel phase **5446 passed, 6 skipped, 3 warnings in 233.78 s**; serial
  phase 5887 deselected (every `serial`-marked test is also `nightly`, so the
  gate's serial phase is empty); total 246 s, `RESULT: PASS`.
- `node tools/probes/uplot_nan_gap_probe.js` (2026-08-22): **ALL ASSERTIONS
  PASSED** (21) — the two string-pinned literals survive the refactor.
- One-off `/tmp` module probe (stubbed DOM, shipped module imported verbatim):
  1.0 rev in → 70.505 mm on leg 0, 31.628 mm on the hand, 450.00 deg on BB
  pitch, commanded converted with the identical factor, BB commanded still NaN.
- Not yet eyeballed on hardware — plan § 8 operator check (3).
  Deploy is a browser reload; no `colcon build`, no flash.
