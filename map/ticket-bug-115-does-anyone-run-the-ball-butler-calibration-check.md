---
id: TKT-84
kind: ticket
title: 'BUG-115: does anyone run the Ball Butler calibration check script by hand at a Sitting, or may the night port its uncovered checks and delete it?'
status: open
created: '2026-11-19'
updated: '2026-11-18T15:54:16Z'
owners: []
priority: low
effort: xs
hardware: none
owner: decision
radius: local
area: tests-tooling
blocked_by: []
links: []
parent: bug-uncollected-ball-butler-calibration-check-script-in-the-ros
type: question
---

BUG-115: a 760-line Ball Butler calibration check script (`ros_ws/src/jugglebot/jugglebot/tests/test_bb_calibration.py`) has no `test_` functions, so no test run collects it. The plan is to compare its checks with the twelve collected `tests/ros/test_bb_calibration_*.py` files, port what is not covered, and delete it. The Item says to confirm first that nobody runs it by hand at a Sitting.

## Options

1. **Nobody runs it by hand:** the night compares, ports the uncovered checks into collected tests, deletes the script, and merges if the gate says MERGE.
2. **You do run it by hand:** the night ports the uncovered checks but keeps the script, and adds a line to its docstring saying the collected tests are the source of truth.
3. **Leave it:** BUG-115 is dropped.

## What each answer unblocks

1 or 2 lets a night build BUG-115 as a branch. 3 closes it.

Recommendation: 1, unless the script is part of a Sitting routine you use. No Sitting sheet or Runbook row names it.
