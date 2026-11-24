---
id: BUG-143
kind: bug
title: Stall watchdog GIL test fails under box load in the parallel phase
status: inbox
created: '2026-11-25'
updated: '2026-11-24T17:39:57Z'
owners: []
priority: low
blocked_by: []
links: []
filed_by: claude
---

`tests/ros/test_stall_watchdog.py::test_gil_held_stall_is_dumped_mid_stall_and_named` failed once in the default gate on 2026-11-25 04:32 with the box at load average 15 to 18. The dump did not name the `gil_holder` thread.

## Findings

- The file passed 8 of 8 on three re-runs alone under the test lock.
- The test measures wall clock (a 100 ms watchdog timeout) and runs in the parallel phase, not the `serial` tail.

## Wanted

- Either mark it `serial`, or make its assertion tolerant of a late dump under load.
