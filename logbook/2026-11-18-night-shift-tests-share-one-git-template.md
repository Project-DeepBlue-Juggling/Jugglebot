---
title: "Three Night Shift test files build their git setup once per module and copy it per test (FEAT-58)"
type: optimization
date: 2026-11-18
status: complete
phase: "maintainer review"
subsystem: tests
files_changed:
  - tests/sim/test_night_shift_gate.py
  - tests/sim/test_night_shift_candidate.py
  - tests/sim/test_sitting_merge.py
---

## What changed

`test_night_shift_gate.py`, `test_night_shift_candidate.py` and `test_sitting_merge.py` built a whole git
setup (init, commits, bare origin, clones, a `mapkit/` copy) for every test. Each now builds it once in a
module-scoped `template` fixture and copies it per test, re-pointing every remote at the test's own copy
(`git remote set-url`), as `test_night_shift_runner.py` already did. Only fixture and setup code changed.

## Why

The 2026-11-16 test-suite review measured these files at about 300 of the suite's 2,330 CPU-seconds, and
their load sat behind the subprocess-timeout flakes that held Night Shift branches that night.

## What was not changed

- `test_night_shift_deploy.py` was converted, measured and dropped: 77.29 s original against 77.10 s
  converted on a quiet box. Its time is the deploy runs (waits, polling, a browser), not the setup.
  FEAT-58's estimate of 220 to 250 CPU-seconds, read from the code, was wrong for that file.
- `_app_repo` in the gate file still builds from scratch (cheap, nine call sites in test bodies).

## Verification (all 2026-11-18, scratch worktree)

- Collected test IDs identical before and after for each file (`pytest <file> --collect-only -q`, sorted,
  `diff` empty): 149, 9 and 14.
- `git diff origin/main | grep assert` is empty; no hunk is inside a `test_*` function.
- Three deliberate breaks per file in the code under test (`tools/night_shift_gate.py`, the candidate
  tool, `tools/sitting_merge.sh`) failed the same set of tests in the original and the converted file.
- Timing, `pytest <file> -q -n0`, loaded box: gate file 646 s to 328 s; candidate file 75.7 s to 11.1 s
  (baseline inflated by load; setup was about a third); sitting-merge file about 25 to 30 % faster in
  adjacent runs (19.5 s to 13.7 s).
- `flock /tmp/jugglebot-tests.lock ./run_tests.sh`: 10,071 passed, 60 skipped, 1 xfailed in 1,020 s;
  serial phase 7 passed. One later edit (the copied bare origin's own remote now points at the test's work
  directory, an independent reviewer's note) was covered by
  `flock /tmp/jugglebot-tests.lock python -m pytest tests/sim/test_night_shift_gate.py -q -n 4 --dist load`:
  149 passed in 37.6 s.

## Found on the way

Dropping the `merge --abort` from the candidate tool fails no test in either the original or the
converted candidate file. The gap predates this change and is filed separately.
