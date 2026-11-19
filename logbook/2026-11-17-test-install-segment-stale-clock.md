---
title: "test_TI1 pins robot_state freshness with its own clock (BUG-108)"
type: bugfix
date: 2026-11-17
status: complete
phase: night-shift
subsystem: tests
related_items: [BUG-108]
files_changed:
  - tests/ros/test_install_segment.py
  - logbook/2026-11-17-test-install-segment-stale-clock.md
---

# test_TI1 pins robot_state freshness with its own clock

## What and why

`trajectory_node._robot_state_fresh()` compares `time.perf_counter()` against `_robot_state_mono` (stamped in `_perf_node` on the real clock) with a 0.5 s bound. `test_TI1_seven_channel_frames_reach_the_wire_and_the_flags_fall` builds the paired bridge node, a ZMQ source and more after `_perf_node`, then installs under `_frozen_perf`. Frozen time starts from the real clock, so any setup stall over 0.5 s on a loaded box made the install refuse `robot_state telemetry stale`.

Fix (test only): `_refresh(traj)` immediately before the frozen install, as the file's other planning tests already do (the live graph delivers robot_state at 100 Hz). The production guard and its threshold are untouched; `test_stale_telemetry_refuses_before_planning` stays green and still asserts the refusal.

## Findings

Reproduced deterministically: a temporary `time.sleep(0.8)` after the source setup failed the test with the stale message before the fix and passed with it. The sleep was then removed. `test_a_fresh_throw_installs_through_the_real_worker_process` already calls `_refresh` right before each attempt and the staleness check has a single call site at service entry, so its one stale-looking failure is not explained by this window and is not changed here; it stays unexplained (one failure in four loaded runs on 2026-11-16), and a recurrence should be filed as its own Bug.

## Verification

2026-11-17, `night_shift_nonet.sh nice -n 19 env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python -m pytest -q tests/ros/test_install_segment.py`: 87 passed in 7.94 s.

2026-11-17, the same command with a temporary `time.sleep(0.8)` after the source setup in `test_TI1`: failed `robot_state telemetry stale` without `_refresh(traj)`, passed with it.
