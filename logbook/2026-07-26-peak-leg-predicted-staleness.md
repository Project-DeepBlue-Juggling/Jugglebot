---
title: "peak_leg_* predicted peaks: a report-less install now publishes 0.0 instead of the superseded plan's numbers"
type: bugfix
date: 2026-07-26
status: in-progress
phase: "Self-toss anomaly fixes — catch-reach-degenerate-overshoot Phase 2 (separate deliverable)"
related_plan: "catch-reach-degenerate-overshoot.md"
files_changed:
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - tests/ros/test_trajectory_node.py
  - tests/hardware/mvp_bench_runbook.md
commits:
  - XXDIAGSHAXX
subsystem:
  - ros
tags:
  - testing
  - docs
---

# `peak_leg_*` predicted peaks: report-less installs publish 0.0

## Summary

`trajectory/diagnostics`' three `peak_leg_*` fields carry the **gate-predicted**
leg peaks of the last install that had a `FeasibilityReport` in hand. Six install
paths have no report and left the fields untouched, so after any of them the
diagnostics described a **superseded** plan under an already-advanced `move_seq`.
`_install` now clears the fields, so those paths publish `0.0` — "this install
carried no prediction" — and every report-carrying install writes *after* its
`_install`. Closes the predicted half of `tests/hardware/mvp_bench_runbook.md`
open item 7.

## Problem

Observed at the bench 2026-07-09 and recorded as `mvp_bench_runbook.md` open
item 7. `_svc_hold`, `_svc_go_home`, `_install_guard_descent`,
`_retry_pending_stop`, `_install_graceful_stop` and the follower's input-loss stop
all call `_install` (which bumps `_move_seq` and resets the *realized* peaks)
without touching `_last_peak_*`. A `hold` after a `go_to_pose` therefore published
the `go_to_pose`'s predicted peaks against the hold's `move_seq` — a diagnostic
that silently attributes one plan's numbers to another.

This is exactly the misreading that cost time during the 2026-07-25 investigation,
which read `14.2 / 142.4 / 3950` identically before and after a catch install and
inferred the field was cached. It is not cached — it is per-plan for
report-carrying installs and *stale* for the rest, and the two are
indistinguishable from a bag.

## Root Cause

Two independent gaps, both in `trajectory_node.py`:

1. `_install` reset the realized peaks but not the predicted ones.
2. `_svc_go_to_pose` wrote `_last_peak_*` **before** calling `_install`, so once
   `_install` started clearing them the write would have been erased.

## Discussion

The alternative was to make every report-less path write the peaks of whatever it
installed. Rejected: those paths genuinely have no `FeasibilityReport` — a hold, a
graceful stop and a guard descent are built without the ~350 ms fine-sampled
validate that produces one — so satisfying that would mean either running the
expensive gate on paths that deliberately skip it, or synthesising a number that
is not the gate's. `0.0` meaning "no prediction for this plan" is honest, is
cheap, and is distinguishable from a real prediction (a real accepted move always
has a non-zero peak velocity).

The ordering half is the subtler one and is why the fix is two changes rather than
one: adding the clear inside `_install` without moving `_svc_go_to_pose`'s write
would have *introduced* a new bug — a real `go_to_pose` reporting `0.0` — turning
a stale-diagnostic defect into a blank-diagnostic defect. All four report-carrying
write sites were enumerated and checked to follow their `_install`.

`_install` already ran under `_plan_lock`, and the three added assignments sit
inside the same lock, so no new lock scope and no work on the emitter path. This
is diagnostics-only: no commanded motion changes.

## Fix

- `_install` clears `_last_peak_vel_mmps` / `_last_peak_acc_mmps2` /
  `_last_peak_jerk_mmps3` alongside the existing realized-peak reset.
- `_svc_go_to_pose`'s three-field write moved to **after** its `_install` call,
  matching the ordering `_plan_and_install_timed` and `_plan_and_install_catch`
  already used.
- The `__init__` and `_publish_status` comments now describe the new contract.

## Verification

Two tests in `tests/ros/test_trajectory_node.py`:

- `test_go_to_pose_records_predicted_peaks_after_the_install_clears_them` —
  catches the write being left before `_install` (peaks erased, a real move
  reporting 0.0).
- `test_a_report_less_install_clears_the_predicted_peaks` — catches the clear
  being dropped, i.e. a hold publishing the superseded plan's peaks under a new
  `move_seq`. Fails pre-fix (`17.41 == 0.0`).

Full suite: see `logbook/2026-07-26-catch-reach-overshoot-fix.md` — both
deliverables were gated by the same run, `pytest tests/ -q`, run 2026-07-26 on the Jetson in the project venv: **3543 passed, 3 xfailed in 1376.13 s (22:56)**

Bench check: `tests/hardware/session_anomaly_fixes.md` § CHECK CCATCH-5. Requires
`colcon build --packages-select jugglebot` and a relaunch.

## Related

- `logbook/2026-07-26-catch-reach-overshoot-fix.md` — C-CATCH-1, the sibling
  deliverable from the same phase.
- `tests/hardware/mvp_bench_runbook.md` open item 7 — the **realized**-peak half
  (per-install rather than rolling-window on the SpaceMouse / reactive-catch path)
  is untouched and still open.
