---
id: BUG-162
kind: bug
title: Solver worker round-trip tests fail when the BLAS thread count is not pinned
status: inbox
created: '2026-11-28'
updated: '2026-11-28T05:18:28Z'
owners: []
priority: low
effort: xs
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links: []
filed_by: claude
---

Found 2026-11-28 during BUG-159.

- `tests/hardware/skills_plan_bench.py` sets OPENBLAS_NUM_THREADS to 1 on import (setdefault), and `tests/ros/test_skills_plan_bench.py` imports it.
- With the variable unset, the test process keeps numpy's 6 threads while a solver worker spawned later inherits 1. The Hand channel then differs by about 1e-13, so the two round-trip tests in `tests/motion/test_solver_worker.py` fail. Reproduces on main.
- The gate exports the variable, so the gate is green; a bare pytest run is not.
- Fix: pin the thread count in `tests/conftest.py`, or stop the bench script setting it on import.
