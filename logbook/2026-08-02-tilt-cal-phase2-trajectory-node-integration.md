---
title: Tilt calibration Phase 2 — trajectory_node integration (loader, reload service, status fields, per-pose ingest keying)
type: feature
date: 2026-08-02
status: resolved
phase: "tilt-calibration-grid Phase 2"
related_plan: tilt-calibration-grid.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/tilt_map.py
  - ros_ws/src/jugglebot_interfaces/msg/TrajectoryStatus.msg
  - ros_ws/src/jugglebot/setup.py
  - ros_ws/docs/levelling_frame.md
  - ros_ws/docs/choreography.md
  - tests/ros/test_trajectory_tilt_map.py
  - tests/ros/test_levelling_frame.py
  - tests/ros/conftest.py
  - tests/motion/test_tilt_map.py
  - plans/archived/tilt-calibration-grid.md
  - tests/hardware/session_anomaly_fixes.md
subsystem:
  - ros
---

# Tilt calibration Phase 2 — trajectory_node integration

## Summary

C-LEVEL-2 is now live in the node that applies it. `trajectory_node` loads
`config/tilt_calibration.yaml` at construction (`$JUGGLEBOT_TILT_CAL` → repo
source tree → ament share — the **inverse** of `friction_ff_params.py`, because
the Phase-3 tool rewrites the source file at runtime and share-first would serve
the last build's stale calibration with a green status), re-reads it on the new
`trajectory/reload_tilt_map` (`std_srvs/Trigger`, so no interfaces rebuild for
the service itself), and publishes `tilt_map_loaded` / `tilt_map_version` on the
existing 5 Hz `trajectory/status`. All six C-LEVEL-1 ingest sites now build their
correction with `levelling.correction_for_pose(offset, tilt_map, intent)` keyed
on the **uncorrected intent pose**, so `timed_target(hold_after=False)` — one
call, two external poses — gives the target the corner residual and the neutral
return the home-node residual. Absent map ⇒ silent identity; invalid map ⇒ loud
ERROR, previous map kept; a lookup that raises ⇒ that one pose degrades to
offset-only and warns once per map, never a dead callback. Nothing gates on
either field. **Requires `colcon build --packages-select jugglebot_interfaces
jugglebot` — both packages.** A `jugglebot`-only build makes `_publish_status`
assign a field the stale generated message's `__slots__` lack; rclpy re-raises
timer exceptions out of `spin()`, so **`trajectory_node` exits ~200 ms after
launch** and `activate` then fails at the A2 arm with "no mpccmd frame"
(`tests/hardware/session_anomaly_fixes.md` row B — whose "already carries
`gravity_correction_loaded`" carve-out these two fields expire, updated in the
same commit).

## Discussion

Trigger (b) — two non-obvious tradeoffs were accepted, and trigger (c) — the
chosen approach beat a reasonable alternative for a reason the code alone does
not show.

Three decisions worth carrying forward. (1) The env override is **authoritative**
— a typo'd `$JUGGLEBOT_TILT_CAL` loads nothing rather than falling through to a
*different* calibration under a `tilt_map_loaded=True` status. (2) Reload with
the file **absent unloads the map and reports success**: reload's contract is
"agree with the file", and Phase 3's `--force-uninstall` depends on it to reach
`tilt_map_loaded == false` before a capture. Only an *invalid* file keeps the
previous map. (3) The structural guard grew a **third kind** (`store` / `build:`
/ `apply:`) rather than mislabelling `correction_for_pose` as a `store`; a
`build` hoisted to the `/gravity_offset` callback compiles, passes every
one-pose test, and applies the home node's residual to the whole workspace.

The keying regression the phase specified turned out to be unwritable, which is
a stronger result than the test would have been: C-LEVEL-1 is rotation-only, so
the corrected and uncorrected poses have identical x/y and key the same cell —
keying on the corrected pose is a fixed-point iteration that converges in one
step, not a wrong answer. The property is pinned instead
(`corrected[:3] == intent[:3]`), so if the correction is ever allowed to move a
position the keying rule stops being free and fails loudly.

**The pre-commit audit caught a blocker that would have made the whole
resolution-order inversion inert.** The first draft resolved the source tree with
a fixed five-level `__file__` walk, copied from `friction_ff_params.py`. `colcon`
*copies* the package, so from
`ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot/motion/` those
five levels land on `ros_ws/install/jugglebot` — and `setup.py` installs config
into `share/jugglebot/config/`, so `install/jugglebot/config/` is a directory
nothing ever creates. In production the source-tree candidate could never exist
and every load would have fallen through to the build-time share copy: the
acquisition tool writes the file, calls reload, and the node re-applies the
*previous build's* calibration while reporting `tilt_map_loaded=true` with a
plausible version. Precisely the trap the inversion exists to prevent, restored
by the mechanism meant to prevent it, and green under every test — because every
assertion was about the *order*, and the order was right. The resolver now
*searches* upward for the repo-root marker (`config/hardware_config.yaml` beside
`ros_ws/`), which is correct from both trees and returns `None` for a genuinely
detached deployment. Pinned by an assertion that candidate 0's directory
actually exists, plus a simulated install-tree layout.

One unasked addition: an autouse `no_tilt_calibration` fixture in
`tests/ros/conftest.py`. Once Phase 4 commits a real capture, every
`TrajectoryNode` in `tests/ros/` would otherwise start applying hardware
calibration and the whole C-LEVEL-1 E-row family would fail on a few
thousandths of a radian — with the suite's answers depending on whether *this
machine* had been calibrated.

## Verification

The session crossed midnight; the entry is dated by the work, the runs by the
clock.

- `pytest tests/ros/test_trajectory_tilt_map.py tests/motion/test_tilt_map.py
  tests/ros/test_levelling_frame.py -q` (run 2026-08-03): **146 passed in
  15.51 s**.
- `./run_tests.sh --full` (run 2026-08-03, final tree — the phase gate):
  **parallel 4541 passed + 3 xfailed in 446.47 s, serial 9 passed in 40.54 s,
  total 493 s — RESULT: PASS**.
- After this section was written, the doc-reading tests were re-run, since a
  logbook edit is inside the test surface: `pytest
  tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py
  tests/sim/test_plans_index.py -q` (run 2026-08-03): **55 passed in 0.48 s**.
