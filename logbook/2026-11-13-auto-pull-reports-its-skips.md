---
title: "The auto-pull reports its skips: a status line and a PULL line in the ticker check"
type: feature
date: 2026-11-13
status: complete
phase: "night-shift close-out C6"
subsystem: tooling
files_changed:
  - tools/systemd/jugglebot-pull.sh
  - tools/systemd/README.md
  - tools/nightly_ticker.sh
  - tests/sim/test_pull_status.py
  - logbook/INDEX.md
---

# The auto-pull reports its skips

## What changed

The 2-minute auto-pull exited silently whenever the live checkout had a tracked modification, so the checkout the robot launches from sat 43 commits behind main with nobody told. Its source now lives in `tools/systemd/jugglebot-pull.sh` (behaviour-identical, still installed to `~/System_Scripts/` by hand) and every run writes `temp/reports/pull/status`: `<iso> <WORD> behind=<n> since=<first non-OK iso> files=<up to 3>`. Existing early exits map to SKIPPED_DIRTY, SKIPPED_SUITE, DIVERGED, DETACHED, FETCH_FAILED; two exits fit none and got new words, NO_UPSTREAM (no origin/branch) and FF_FAILED (ff-only refused). Non-OK also goes to `logger -p user.warning`. `tools/nightly_ticker.sh check` prints `PULL <word> behind=<n> since=<t>` after its usual line when a non-OK status is over 10 minutes old; the first line and exit code are unchanged. Not installed: the owner approves `install -m 755 tools/systemd/jugglebot-pull.sh ~/System_Scripts/`.

## Verification

- 2026-11-13, `pytest tests/sim/test_pull_status.py tests/sim/test_live_session_guard.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py -q`: 30 passed in 10.29 s.
- 2026-11-13, `./run_tests.sh -q` from the worktree: parallel phase 1 failure, tests/ros/test_install_segment.py (a solver-worker wire-deadline timing test, load average ~8 from a parallel suite; unrelated to these files); that file rerun scoped: the failing test passed 3/3, and the file passed 87/87 when run alone earlier. Serial phase 7 passed.
