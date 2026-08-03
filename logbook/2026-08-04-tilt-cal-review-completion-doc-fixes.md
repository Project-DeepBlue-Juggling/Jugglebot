---
title: "Tilt-cal review completion: 5 doc-drift WARNINGs from the recovered cross-doc/conventions dimensions"
type: bugfix
date: 2026-08-04
status: resolved
phase: "Tilt calibration grid — review completion"
related_plan: tilt-calibration-grid.md
subsystem: [ros, docs]
tags: [docs, levelling, calibration, review]
files_changed:
  - ros_ws/src/jugglebot_interfaces/msg/TrajectoryStatus.msg
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/docs/levelling_frame.md
  - tests/ros/test_trajectory_tilt_map.py
  - tests/ros/conftest.py
  - tests/hardware/session_tilt_calibration.md
  - tests/hardware/tilt_cal_grid.py
  - tools/tilt_cal_analyse.py
  - plans/active/tilt-calibration-grid.md
---

# Tilt-cal review completion: doc-drift fixes

## Summary

The adversarial review's cross-doc and conventions dimensions, lost to the
2026-08-03 usage-limit boundary, re-ran against `00c7818`: 5 confirmed
WARNINGs (0 refuted by verifiers) and 3 NOTEs — every one documentation or
comment drift left behind by the `00c7818` dormancy-semantics change, no
runtime defects. Fixed: `TrajectoryStatus.msg`, the `_publish_status` comment,
a test assertion message and the mock conftest all still said the status pair
means "loaded AND applied" (the contract's normative semantics are LOADED, not
applied — a loaded map is dormant until `level` runs); the contract's own
"adding a new pose surface" recipe named `self._tilt_map`, which would bypass
the dormancy gate at a future seventh ingest surface with no test able to
catch it (the manifest keys on the call, not its arguments) — it now names
`self._active_tilt_map()` with the trap spelled out; `CURVATURE_*` pinning
attributed to C0 in the runbook vs C1 in the analyser/plan — C1 is correct
(the constants need a real field, not C0's four probe poses); the runbook's
partial-rebuild signature (AttributeError in the 5 Hz status timer, not
module-scope ImportError); one 18.86 s vs 18.95 s verification-triple
mismatch in the plan; `--force-uninstall` help text and an analyser docstring
flag name still describing pre-`00c7818` semantics.

## Verification

Docs/comments/help-strings only — no logic changed. (date, command, result):
run 2026-08-04, `./run_tests.sh`, **parallel 4226 passed in 192.28 s (rc=0),
serial phase empty (all serial-marked tests are nightly-tier), total 202 s —
RESULT: PASS**.
