---
id: BUG-103
kind: bug
title: 'Tests that never run or cannot fail: two permanent skips, Node lookup, stale-log fixture, loose ROS mocks'
status: complete
created: '2026-11-14'
updated: '2026-11-18T02:51:45Z'
owners: []
priority: medium
effort: m
hardware: none
owner: none
radius: subsystem
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

Two further findings (the uncollected calibration script, the loose ROS mocks) were split into their own Items on 2026-11-18.

## Wanted

- One shared `find_node()` helper and skip reasons (`-rs`) in the suite report.
- A committed fixture for the hermite cross-check in place of `temp/logs`.

## Progress

- 2026-11-18: 2026-11-16 review, Node lookup confirmed from 31 nights of junit: 59 tests in 8 tests/ros/test_gui_*.py files skipped on every 04:00 run (bare shutil.which; Node is only under ~/.nvm). Owner wants this part fixed first.
- 2026-11-18: Findings 1 to 3 fixed (shared Node lookup, -rs in the gate, two permanent skips deleted, committed hermite fixture); logbook 2026-11-18-tests-that-never-ran-node-lookup-skips-hermite-fixture.
