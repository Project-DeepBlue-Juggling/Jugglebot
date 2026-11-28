---
title: "The skills plan bench caps the BLAS pool only when numpy is not loaded yet, so a test importing it no longer splits the solver worker's thread count from its parent's (BUG-162)"
type: bugfix
date: 2026-11-29
status: complete
phase: "Night Shift 2026-11-29"
subsystem: tests-tooling
files_changed:
  - tests/hardware/skills_plan_bench.py
  - tests/ros/test_skills_plan_bench.py
---

# The bench's BLAS cap applies only before numpy loads (BUG-162)

**What.** `tests/hardware/skills_plan_bench.py` set `OPENBLAS_NUM_THREADS` and `OMP_NUM_THREADS` to 1 with `setdefault` on import. When the bench runs as a script that happens before numpy loads, which is its purpose: the cap sizes the pool. But `tests/ros/test_skills_plan_bench.py` imports it after numpy is loaded. At that point the cap no longer reaches the test process and only leaks to processes spawned later. On a bare pytest run with the variables unset, the test process kept numpy's six threads, a solver worker spawned later inherited one, and the two round-trip tests in `tests/motion/test_solver_worker.py` differed by about 1e-13 in the Hand channel. The gate exports the variables, so it was always green. The bench now sets them only `if 'numpy' not in sys.modules`. The script behaviour is unchanged.

**Why not `tests/conftest.py`.** Pinning the count there would also work, but every conftest is frozen for the Night Shift, and the leak's source is the bench.

**Test.** `test_the_blas_cap_is_set_only_before_numpy_loads` runs two fresh interpreters with both variables unset. With numpy imported first, the bench leaves both unset. With the bench first, both are 1. With the guard planted back to `if True:`, the numpy-first case fails (`'1 1' == 'None None'`).

## Verification

- 2026-11-29, `env -u OPENBLAS_NUM_THREADS -u OMP_NUM_THREADS tools/night_shift_nonet.sh pytest -q -p no:cacheprovider tests/ros/test_skills_plan_bench.py tests/motion/test_solver_worker.py` (one process, so the bench import precedes the worker spawn, the case the Item describes): **96 passed in 438.76 s** (the two new parametrised cases included). No `./run_tests.sh` run is cited here: the Night Shift gate runs `./run_tests.sh --full` on the branch tip.
- 2026-11-29, `tools/night_shift_nonet.sh pytest -q tests/ros/test_skills_plan_bench.py -k blas_cap` with the guard planted to `if True:`: **1 failed, 1 passed** (the numpy-first case); restored: **2 passed**.
