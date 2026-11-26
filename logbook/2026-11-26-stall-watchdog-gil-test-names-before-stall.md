---
title: "Stall watchdog GIL test: the holder thread stalls only after a watchdog tick has named it, so the name in the dump no longer depends on which thread takes the GIL under box load (BUG-143)"
type: bugfix
date: 2026-11-26
status: complete
phase: "Night Shift 2026-11-26"
related_plan: ""
related_issues: [BUG-143]
files_changed:
  - tests/ros/test_stall_watchdog.py
subsystem: [tests, ros]
tags:
  - night-shift
---

# Stall watchdog GIL test: named before the stall

**What.** `tests/ros/test_stall_watchdog.py::test_gil_held_stall_is_dumped_mid_stall_and_named` failed once in the default gate on 2026-11-25, at load average 15 to 18. The dump held the `_hold_gil` frame but not the name `gil_holder`.

**Why.** `StallWatchdog.tick` records thread names on its ticks, every 30 ms in the test, and `_report` adds the threads alive at report time. The module documents that it names threads alive before a stall. The test started its holder thread straight into a 300 ms GIL hold, so the name was recorded only if a tick ran in one of two short windows:

- between the holder's start and its first GIL grab;
- when the holder let go of the GIL, before the thread exited.

At the hand-off a pending GIL request is served by whichever thread asked for it. In an xdist worker that can be execnet's IO thread rather than the watchdog. Under load the watchdog lost both windows, and the holder exited unnamed. That is a test race against the documented contract, not a watchdog defect.

**Fix (test only).** The holder waits on an Event. The test polls until the watchdog's name table holds `gil_holder` (asserted, 5 s bound), then releases it into the stall. The dump assertions are unchanged. Not `serial`: the test now holds under concurrent load, so it keeps its place in the parallel phase.

## Verification

- 2026-11-26, `tools/night_shift_nonet.sh timeout 300 python -m pytest tests/ros/test_stall_watchdog.py tests/ros/test_stall_watchdog.py tests/ros/test_stall_watchdog.py -q -p no:cacheprovider`: **24 passed in 5.87 s** (the file three times).
- The original failure was a load-dependent race and does not reproduce on demand (8 of 8 passes alone on 2026-11-25). The new `never named` assertion makes the precondition explicit instead.
