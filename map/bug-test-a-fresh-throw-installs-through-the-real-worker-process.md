---
id: BUG-120
kind: bug
title: test_a_fresh_throw_installs_through_the_real_worker_process misses the solver wire deadline under box load and fails the gate
status: complete
created: '2026-11-19'
updated: '2026-11-20T15:49:19Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links:
- bug-test-ti1-in-test-install-segment-goes-stale-under-box-load
- logbook/2026-11-20-real-worker-install-test-load.md
filed_by: night-shift
---

Found by the Night Shift of 2026-11-19 while re-gating BUG-108. Related: BUG-108.

`tests/ros/test_install_segment.py::test_a_fresh_throw_installs_through_the_real_worker_process` fails under box load with `AssertionError: solver did not answer by the wire deadline` (`assert False is True` on the response's `.accepted`).

## Evidence

- 2026-11-19, load about 20 (a day Session's tests and the Night Shift suite running at once): it failed in both of the BUG-108 auditor's whole-file runs and in the orchestrator's (`night_shift_nonet.sh timeout 400 python -m pytest tests/ros/test_install_segment.py -q -p no:cacheprovider`: 1 failed, 86 passed in 159.69 s).
- The BUG-108 logbook entry (`logbook/2026-11-17-test-install-segment-stale-clock.md`) already recorded one unexplained failure of this test, and said to file a recurrence as its own Bug.
- TI1, the test BUG-108 fixed, passed in the same runs.

## Wanted

- Find which wall-clock deadline the test leaves to real time: the worker process's solve time against the wire deadline. Pin it with the test's own clock, or give the test a load-independent deadline, as BUG-108 did for the staleness window.
- A gate that fails on box load turns a clean branch into a HOLD.

## Progress

- 2026-11-20: Night Shift 2026-11-20: held on night-shift/2026-11-20-BUG-120: (b) 2 audit findings open (the skip can hide a slowdown, one restart retried away); suite green; unblocked by TKT answer
- 2026-11-21: Night Shift 2026-11-21: TKT-87 answered (apply both findings): the starvation skip now needs a 1-minute load average above the core count, and a second worker restart fails the test.
