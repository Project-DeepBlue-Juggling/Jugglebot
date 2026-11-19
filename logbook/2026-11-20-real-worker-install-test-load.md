---
title: "the real-worker install test widens its own reserved lead under box load (BUG-120)"
type: bugfix
date: 2026-11-20
status: complete
phase: night-shift
subsystem: tests
related_items: [BUG-120, BUG-108]
files_changed:
  - tests/ros/test_install_segment.py
  - logbook/2026-11-20-real-worker-install-test-load.md
---

# the real-worker install test widens its own reserved lead under box load (BUG-120)

## What and why

Deadline arithmetic (fresh THROW, `trajectory_node._svc_install_segment`): `lead_s = schedule.install_lead_s(THROW) = FRESH_THROW_LEAD_S = 0.325 s`; wire deadline `deadline_s = t_wall + lead_s + KNOT_DT (0.025) = t_wall + 0.35 s`; the executor's solve budget is `lead_s - WIRE_READ_KNOTS*dt = 0.250 s`, read on the wall clock after the reply. A late reply is `ORIGIN_TOO_LATE` ("the solve took X s, past the 0.250 s budget" or the client's "did not answer by the wire deadline"). The test's origin `at + 0.8` plays no part. The worker's warm solve is ~190 ms idle and crosses 250 ms under CPU oversubscription, and the old 4 back-to-back retries all landed in one starved stretch.

Reproduction (2026-11-20, 6 cores, 8 busy loops, `-k real_worker_process`): the old test failed 6/6 and 4/4 (all four attempts `ORIGIN_TOO_LATE`, solves 0.254-0.291 s). Twelve retries with pauses still failed 4/6, so retrying cannot fix it.

Fix (test only, no production constant touched): the test monkeypatches `schedule.install_lead_s` to 1.5 s for this one request (the node ships it as `req.lead_s` and derives the wire deadline from it), event at `at + 2.5`; the node's deadline arithmetic and the late-reply mapping stay asserted by the fake-solver tests, but no test now measures a real worker against the 250 ms budget. Up to 6 attempts, retrying the timing refusal and the designed worker restart (`SOLVER_UNAVAILABLE`, then `wait_ready`). If all attempts only hit `ORIGIN_TOO_LATE` the test SKIPS with the reason (worker starved past 1.5 s); any other refusal fails, and a pass still asserts accepted, `OK`, active plan and cycle installed through the real process. Residual: at 8-10 busy loops on 6 cores about 1 run in 10 still fails with `SOLVER_UNAVAILABLE: worker died; restarting` (the client's 2 s `SOLVER_HANG_S` kill); not masked, since a real worker crash looks the same.

## Verification

- 2026-11-20, ten runs each of `tools/night_shift_nonet.sh python -m pytest tests/ros/test_install_segment.py -k real_worker_process -q -p no:cacheprovider` with 8 busy-loop Python processes (`while True: pass`) running on the 6 cores: fixed test 7 passed, 2 skipped, 1 failed (hang-limit kill), so 3 of 10 runs are not a pass at that load; old test 0 of 10 passed.
- 2026-11-20, `timeout 900 tools/night_shift_nonet.sh python -m pytest tests/ros/test_install_segment.py -q -p no:cacheprovider` (unloaded): 87 passed in 23.24 s.
