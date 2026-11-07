---
id: JB-182
kind: bug
title: test_normal_interpolation_unchanged reads the wall clock and failed one full gate under load
status: complete
created: '2026-11-06'
updated: '2026-11-06T19:21:40Z'
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
- logbook/2026-11-07-jb182-motor-guard-test-frozen-clock.md
filed_by: claude
---

`tests/motion/test_motor_guard.py::test_normal_interpolation_unchanged` failed one full gate on 2026-11-06 at 05:21 and passed the next run on the same commit.

## Findings

- The miss was 0.19 mrev against a tolerance of 0.1 mrev (`atol=1e-4`).
- The test sets `_mpc_base_timestamp` from `time.perf_counter()` and `time.sleep(0.02)`, then asserts a linear interpolation, so the result depends on wall clock under load.
- A bag read was running on the box during that run.
- The test is not marked `serial`. The rule in `CLAUDE.md` says a test measuring wall clock should be.

## Wanted

- Inject the clock, or mark it `serial`. The test's safety claim does not need real time.
