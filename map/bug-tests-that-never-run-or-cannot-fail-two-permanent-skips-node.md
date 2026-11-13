---
id: BUG-103
kind: bug
title: 'Tests that never run or cannot fail: two permanent skips, Node lookup, stale-log fixture, loose ROS mocks'
status: inbox
created: '2026-11-14'
updated: '2026-11-13T23:09:15Z'
owners: []
priority: medium
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links: []
filed_by: claude
---

Tests that look like protection and are not, from the 2026-11-13 maintainer review. The tracker tests that were never collected are being moved into `tests/tracking/` by the review itself; this Item holds the rest.

## Findings

- `tests/motion/test_unified_cycle.py:1394` and `:1497` are unconditional skips whose reason says the port "does NOT reproduce the claim". They have never run. Fix the recipe or delete them.
- 38 files under `tests/ros/` find Node with a bare `shutil.which`; 30 also know the `~/.nvm` fallback. Node on this box lives only under `~/.nvm`. Whether the 04:00 systemd environment has it on `PATH` was not confirmed, and the suite output does not print skip reasons, so a silent mass skip of the GUI harness tests is possible. Last night's 100 skips are consistent with either reading.
- `tests/firmware/test_hermite_xref.py:43-46` needs `temp/logs/mpc_*.csv`, logs of a chain deleted on 2026-09-01. It runs today only because old files remain; cleaning `temp/` turns it into a silent skip.
- `ros_ws/src/jugglebot/jugglebot/tests/test_bb_calibration.py` is a hand-rolled script with no `test_` functions, so nothing collects it. Part of it was copied to `tests/ros/test_bb_calibration_arc_span.py`.
- The ROS mock layer (`tests/ros/conftest.py`) uses plain dataclasses for messages, so assigning a misspelt field passes, and nothing compares the mock message definitions with the real `jugglebot_interfaces` files.

## Wanted

- One shared `find_node()` helper and skip reasons (`-rs`) in the suite report.
- A committed fixture for the hermite cross-check in place of `temp/logs`.
- Port or delete the calibration script.
- `__slots__` or a strict `__setattr__` on the mock messages, and a test that the mock fields equal the `.msg`, `.srv` and `.action` definitions.
