---
id: BUG-115
kind: bug
title: 'Uncollected Ball Butler calibration check script in the ROS package: port what is not covered, then delete'
status: complete
created: '2026-11-18'
updated: '2026-11-19T15:55:41Z'
owners: []
priority: low
effort: m
hardware: none
owner: decision
radius: subsystem
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links:
- bug-tests-that-never-run-or-cannot-fail-two-permanent-skips-node
- logbook/2026-11-20-bb-calibration-check-script-ported-and-deleted.md
filed_by: claude
---

A 760-line calibration check script sits in the ROS package and no test run collects it. Split from BUG-103 on 2026-11-18.

## Findings

- `ros_ws/src/jugglebot/jugglebot/tests/test_bb_calibration.py` is a hand-rolled script with no `test_` functions. It runs only by hand: `python -m jugglebot.tests.test_bb_calibration`.
- Part of it was copied to `tests/ros/test_bb_calibration_arc_span.py`.
- Twelve `tests/ros/test_bb_calibration_*.py` files now cover the calibration code; how much of the script they repeat is not measured.

## Wanted

- Compare each check in the script with the twelve collected files.
- Port what is not covered, then delete the script.
- Before deleting, confirm nobody runs it by hand at a Sitting.
