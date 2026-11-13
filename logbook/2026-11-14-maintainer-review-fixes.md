---
title: "Maintainer review: seven small fixes landed, the rest filed as Items"
type: bugfix
date: 2026-11-14
status: complete
phase: "maintainer review"
subsystem: tooling
files_changed:
  - tests/tracking/
  - teensy_link/client.py
  - tests/teensy_link/test_client.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/memory.py
  - tests/motion/test_memory.py
  - tests/firmware/test_sched_c2_twin.py
  - tools/night_shift_gate.py
  - tests/sim/test_night_shift_gate.py
  - plans/active/night-shift.md
  - tools/systemd/jugglebot-pull.sh
  - tools/systemd/jugglebot-pull.service
  - tools/systemd/jugglebot-pull.timer
  - tools/systemd/README.md
  - tests/sim/test_pull_status.py
  - mapkit/atomic.py
  - mapkit/items.py
  - mapkit/ids.py
  - tests/sim/test_mapkit_items.py
  - logbook/INDEX.md
---

## What

A read-only review of the last three months of work (2026-11-13, ten reviewers, main
findings re-read against the code) found a handful of defects with a clear, small cure.
Seven landed here; everything larger or less certain was filed as BUG-93 to BUG-106,
FEAT-51 to FEAT-57 and IDEA-24.

1. **Tracker tests collected.** `tracking/tests/` sat outside `testpaths`, so its 49
   tests (matcher, Kalman, flight Fit, lifecycle) never ran in the default gate or the
   04:00 suite. Moved to `tests/tracking/`; all 49 pass unchanged.
2. **TX sequence counters serialised.** `send_stream` and `send_rpc` incremented their
   counters with no lock while the heartbeat, setpoint and service threads all send. One
   lock per socket now covers increment, encode and send; the shared TX stats sit under
   an inner lock.
3. **Memory CSV.** `append` wrote the header only when the file was missing, so an
   existing empty `memory_v2.csv` got rows with no header and the next load refused the
   plant. The header is now written to a missing or empty file, and a torn last row gets
   a newline first so the new row is not glued onto it.
4. **Resume tolerances pinned.** The executor's `REBASE_TOL_*` and the schedule's
   `REST_RETURN_ONSET_BUDGET_REV` restate firmware constants; a test now reads
   `canbridge_config.h` and asserts equality (and the documented half for the last).
5. **Merge gate freezes every conftest under `tests/`.** Only `tests/conftest.py` and
   `tests/sim/conftest.py` were frozen; a Night Shift branch could have edited
   `tests/ros/conftest.py` or added a new one and dropped tests from the suite.
6. **Auto-pull fetch bounded.** `git fetch` now runs under a 60 s `timeout`, reported as
   `FETCH_FAILED`. The pull unit files were never tracked; reference copies are added,
   the service with `TimeoutStartSec=90s`.
7. **Items written atomically.** `mapkit/items.py` and `mapkit/ids.py` truncated and
   rewrote an Item in place; they now go through `mapkit/atomic.py` (temp file, fsync,
   `os.replace`, mode bits kept).

## Why these and not the others

Each of the seven is confined to one module, has a test that was red before the fix, and
changes nothing a flown Throw or Catch reads. The firmware findings need a version bump
and a flash; the Ball Butler announcement and the REST lead change flown behaviour; the
gate running the suite itself (owner decision, 2026-11-14, FEAT-51) changes the
unattended protocol and wants a dry night first.

Holding the TX lock across `sendto` cannot stall the 40 Hz setpoint thread: both sockets
are non-blocking, so the locked section is one encode and one system call.

Two things were checked and need no change: Ball Butler's firmware ignores a repeated
`RELOAD` command while one is in progress and only re-checks ball presence afterwards, <!-- avoid-ok -->
so the default RPC retry on `bb/reload` is safe; and the audit's notes on the atomic
write (a symlinked Item becomes a regular file, no directory fsync) are accepted.

## Not live yet

The auto-pull change takes effect only after the owner installs it:
`install -m 755 tools/systemd/jugglebot-pull.sh ~/System_Scripts/ && install -Dm644 tools/systemd/jugglebot-pull.service ~/.config/systemd/user/jugglebot-pull.service && systemctl --user daemon-reload`.

## Verification

- 2026-11-14, `./run_tests.sh -q` on the seven commits over origin/main 8e5347c2:
  RESULT PASS (parallel rc=0 in 868 s, serial rc=0 in 33 s; `-q` prints no count).
- 2026-11-14, `pytest tests/tracking --collect-only -q`: 49 tests collected.
- 2026-11-14, one audit pass over `origin/main..HEAD`: no blocking finding.
- Every new test was seen red before its fix by the implementing agent.
