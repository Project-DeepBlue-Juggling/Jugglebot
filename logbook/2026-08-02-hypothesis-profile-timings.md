---
title: The hypothesis profile docstring still promised a 5-minute suite from when it had 1200 tests
type: bugfix
date: 2026-08-02
status: resolved
phase: "Developer workflow — test-suite runtime"
files_changed:
  - tests/conftest_hypothesis.py
subsystem:
  - sim
tags:
  - testing
  - docs
---

# The hypothesis profile docstring promised a 5-minute suite

## Summary

`tests/conftest_hypothesis.py` documented `ci-fast` as targeting "< 5 minutes
wall-clock" and `ci-deep` as "~10 min". Both dated from a ~1200-test suite; it is
now ~4400, and both had been wrong for months. The docstring also named the wrong
invocation (`pytest tests/ -q --hypothesis-profile=ci-deep`) — that predates
`run_tests.sh`'s tiering and would run the whole suite serially without the
`--full` marker composition.

Replaced with measured wall-clock, each carrying its date, plus the reason the
ratio is not what the `max_examples` numbers suggest.

## Fix

| run | profile | wall-clock |
|---|---|---|
| `./run_tests.sh` (per-commit) | ci-fast | 206 s (2026-08-01) |
| `./run_tests.sh --full` | ci-fast | 485 s (2026-08-02) |
| `./run_tests.sh --full` | ci-deep | 25m 52s (2026-08-02) |

ci-deep is **~3.2×** the ci-fast `--full` run, not the ~20× the 50→1000
`max_examples` ratio implies: only the hypothesis properties scale, and they are a
minority of the suite's cost. That ratio is the useful number for anyone deciding
whether a deeper profile is affordable, and it was exactly what the old "5 min vs
10 min" pair got wrong — it implied a 2× cost for a 20× example count.

The ci-deep figure is not a bespoke measurement: it is the 2026-08-02 nightly
(`temp/reports/nightly/`, GREEN, 4415/4418), which now runs this profile every day
and re-measures it for free.

## Verification

- `pytest tests/ -q --collect-only` (run 2026-08-02): **4418 collected**, so the
  module still imports and its `settings.register_profile` side effects still run —
  the thing a docstring edit here could plausibly break.
- `pytest tests/sim/test_ballistics.py -q --hypothesis-profile=ci-deep
  --collect-only` (run 2026-08-02): **15 collected**, i.e. the profile name still
  resolves; an unregistered profile is a hard error at startup.
- Gate (`./run_tests.sh`, run 2026-08-02): **parallel 195 s (3983 passed) |
  serial 8 s | total 203 s** — `RESULT: PASS`. Run despite the change being a
  docstring, because this module is imported at collection time by every test run:
  the "docs-only" exemption does not apply to a file whose import side effects
  configure the suite.
