---
title: "The worker-lateness test failed on a garbage collection in the test process, not on the worker"
type: bugfix
date: 2026-11-14
status: complete
phase: "night-shift close-out"
subsystem: tests
files_changed:
  - tests/motion/test_solver_worker_lateness.py
  - logbook/INDEX.md
---

# The worker-lateness test failed on a garbage collection in the test process

## What happened

`tests/motion/test_solver_worker_lateness.py::test_the_worker_keeps_the_40hz_grid_while_the_in_process_solve_starves_it` failed in four of ten full gate runs on 2026-11-14 with p99 between 111 and 127 ms against the 5 ms bound (max up to 403 ms), always in the gate's serial phase and never when the file ran alone. The nightly had passed it every night from 2026-11-10 to 2026-11-14.

## Discussion

Three explanations were held and dropped before the cause was found.

1. **Box load.** The first failures overlapped other sessions' and agents' test runs, so they were read as load. Withdrawn: the value repeated within a few ms across runs, a run alone under the shared test lock failed too, and the test alone under 6 and 10 CPU-hogging processes passed 6 of 6 (worst p99 3.9 ms).
2. **The Night Shift no-network wrapper**, which gained mounts and device masks the same day and which every gate run went through. Withdrawn: the serial phase passed 7 of 7 wrapped and 6 of 6 unwrapped on main.
3. **State left by an earlier serial test.** Withdrawn by the same 19 runs.

What separated the runs was the tree. On main at 66b70b6e (9892 tests deselected in the serial phase) the serial phase passed every time; on the close-out stack (9940 deselected) it failed 3 of 3, with p99 117.5, 110.8 and 114.8 ms. The serial phase collects the whole suite before deselecting, so the test process holds about 430 000 tracked objects, and where a collection falls depends on how many objects collection created. The deciding run changed nothing in the test: with `gc.disable()` set process-wide from a `sitecustomize.py` on `PYTHONPATH`, the same tree passed 3 of 3 (p99 0.36, 0.28, 0.62 ms). An instrumented copy of the test (a `gc.callbacks` timer) passed and saw no collection, which is the same effect: the extra allocations moved the collection out of the window. The test's docstring records that GC was ruled out in October with `gc.callbacks`; that was true for the heap of the time.

The stall is in the caller: the emitter thread shares the pytest process, and a collection there stops it whatever the worker does. That is not what the test measures (the worker's solve must not delay the emitter), so the fix belongs in the test setup and not in the 5 ms bound. `gc.collect()` then `gc.freeze()` before the two measurements, `gc.unfreeze()` after, which is what the node's opt-in `gc_freeze` cure does. The control assertion (an in-process 150 ms hold must show up as more than 50 ms of lateness) is untouched, so the test still proves it can see starvation.

Not done: the same exposure may exist in the other wall-clock serial tests (`test_unified_cycle_budget` failed once on the same day, under real load). They are left as they are until one fails on a quiet box.

## Verification

- 2026-11-14, close-out stack, unmodified test, three runs of `tools/night_shift_nonet.sh python -m pytest tests/ -q -m "serial and not nightly" -p no:cacheprovider -s`: 1 failed, 6 passed each time.
- 2026-11-14, same tree and command with `gc.disable()` from `sitecustomize.py`: 7 passed, three times.
- 2026-11-14, same tree and command with the fix: 7 passed, three times (p99 0.30, 0.63, 0.30 ms).
- The full gate on the stack is cited in the commit that lands it.
