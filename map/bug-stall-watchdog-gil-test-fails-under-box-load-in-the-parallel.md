---
id: BUG-143
kind: bug
title: Stall watchdog GIL test fails under box load in the parallel phase
status: complete
created: '2026-11-25'
updated: '2026-11-25T15:55:45Z'
owners: []
priority: low
effort: xs
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links:
- logbook/2026-11-26-stall-watchdog-gil-test-names-before-stall.md
filed_by: claude
---

`tests/ros/test_stall_watchdog.py::test_gil_held_stall_is_dumped_mid_stall_and_named` failed once in the default gate on 2026-11-25 04:32 with the box at load average 15 to 18. The dump did not name the `gil_holder` thread.

## Findings

- The file passed 8 of 8 on three re-runs alone under the test lock.
- The test measures wall clock (a 100 ms watchdog timeout) and runs in the parallel phase, not the `serial` tail.

## Wanted

- Either mark it `serial`, or make its assertion tolerant of a late dump under load.
