---
title: "GUI: violet CLOSED_LOOP indication on pills, chart titles and 3D parts; BB 'Coordinates' throw target; chart isolate with snapshot restore; Show-all button"
type: feature
date: 2026-09-06
status: resolved
phase: "gui — operator tooling"
files_changed:
  - ros_ws/gui/js/telemetry-charts.js
  - ros_ws/gui/js/stewart-model.js
  - ros_ws/gui/js/ball-butler-model.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/panels.js
  - ros_ws/gui/css/charts.css
  - ros_ws/gui/css/panels.css
subsystem:
  - gui
---

# GUI: CLOSED_LOOP indication, BB coordinate target, chart isolate, Show-all

## Summary

Four owner-requested GUI edits, `ros_ws/gui/` only, no Python touched:

1. **Armed indication.** Any axis whose `current_state` is `CLOSED_LOOP` (8) is now
   marked on all three surfaces at once: its chart pill (`.armed`), its in-cell chart
   title (`.armed`, violet outline over the existing opaque backdrop), and its 3D part —
   legs 0–5, the Stewart hand axis, the BB pitch group and the BB hand sphere breathe
   at ~0.7 Hz (`sin(now/230)`) in `--accent-purple` / `0xa78bfa`, colour lerped toward
   violet with a violet emissive. Fed from `main.js`'s `robot_state` handler after the
   fault fan-out; an axis absent from `motor_states` (the 7-axis BB-dark shrink) reads
   NOT armed. DOM writes are change-gated.
2. **BB "Coordinates" target.** The Throw Director gains a `Coordinates` option that
   reveals X/Y/Z (mm, world frame) and Release-delay (s) inputs and calls
   `bb/throw_at_target` with `use_target_point: true`. Non-finite X/Y/Z errors locally
   without calling the service; blank/negative delay → 0 (node default). The named-target
   branch sends the identical payload as before.
3. **Chart isolate.** Long-press (500 ms, primary pointer only), Shift-click and keys 1–9
   now share one mechanism: snapshot the visible set, show one chart, style the pill
   `.isolated`; any short click on any pill restores the snapshot; isolating another chart
   switches and keeps the original snapshot; localStorage persists the snapshot, not the
   lone chart. Replaces the stateless `toggleSoloChart` (0 references remain).
4. **Show all charts** button between "Hide charts" and the pills; clears isolation, shows
   all 9, dims when already all-visible, no-ops without a rebuild in that state, and leaves
   the grid-hidden state alone.

## Decisions

- **Violet, not red.** Red is already the fault colour on every one of these 3D parts
  (2 s pulse then steady glow on `active_errors`/`disarm_reason`); a red armed pulse would
  make an armed leg indistinguishable from a faulted one. Fault wins over armed on every
  path; fault-clear while still armed hands the material back to the breathing loop; disarm
  restores exactly what fault-clear restores (diffed pairwise in the audit).
- **Release delay, not landing time.** The service's only time field is `throw_delay_s`;
  `solve_throw_local(x, y, z)` picks time-of-flight itself, so a landing-time input would
  need a `.srv` + node change. Owner chose the GUI-only mapping.
- **A coordinate throw does not arm the catch.** `target_name` on the point path becomes
  the announcement's `target_id`, and `catch_coordinator_node` moves the platform only when
  that equals its `robot_name`. `'Coordinates'` therefore never moves the platform — the
  safe default, deliberately kept.

## Audit

`/audit --unstaged` (2026-09-06): no blocking findings; 3 WARNING + 2 NOTE, all applied —
primary-button/primary-pointer guard on the long-press, a tracked swallow-flag timer
(disarmed on every fresh press), real theme tokens for the coordinate inputs (`--bg-input`
and `--border` do not exist in `theme.css`, so the copied `.bb-throw-select` idiom rendered
`#222` in light theme — that pre-existing rule is left as-is, out of scope), a Show-all
no-op early return, and the layered title backdrop.

## Verification

- (2026-09-06, `./run_tests.sh` after the audit fixes, **PASS — parallel 6644 passed,
  4 skipped in 276.60 s; serial 1 passed in 14.87 s; rc=0 both phases, 298 s total**).
- (2026-09-06, `pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py -q`,
  **34 passed in 0.57 s**) — the entry and INDEX row landed after the gate started.
- (2026-09-06, `node --input-type=module --check` on the five edited `.js` files, **5/5 OK**).
- No pytest reads the seven changed files. The tests that read `ros_ws/gui/` are
  `tests/ros/test_gui_geometry.py` and `tests/firmware/test_config_drift.py`
  (`geometry-config.js`) and `tests/ros/test_gui_fk_golden.py` (`stewart-fk.js`), all
  untouched here and all inside the gate above.
- **Not yet loaded in a browser** — this was a headless session. The first operator page
  load is the visual check; nothing here actuates hardware except the pre-existing
  `bb/throw_at_target` service.
