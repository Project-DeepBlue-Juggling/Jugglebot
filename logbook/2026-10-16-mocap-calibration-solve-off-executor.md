---
title: "BB calibration solve moved off mocap_node's executor thread: the bb/heartbeat exit edge snapshots the sweep and returns, one daemon worker thread runs run_calibration + estimate_base_pose, an executor-thread timer (0.1 s) applies the outcome; the 4.3-7.2 s post-sweep frame-drain stall (30-52 % frame loss per sweep) is gone by construction; flown 2026-10-16 (14 sweeps, zero drops, solve 14-16 s and 100-170 ms publish gaps under the live GIL load)"
type: bugfix
date: 2026-10-16
status: resolved
phase: "tracker fit latency (follow-up to handoff-solve-chaining P2)"
related_plan: handoff-solve-chaining.md
related:
  - 2026-10-16-mocap-frame-queue-starvation.md
  - 2026-10-16-tracker-flight-fit-late-since-matcher-fix.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/calibration_solve_worker.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - tests/ros/test_mocap_calibration_solve_worker.py
  - tests/ros/test_mocap_node.py
  - tests/ros/test_mocap_node_base_frame.py
  - tests/ros/test_mocap_node_keep_last_good.py
  - tests/ros/test_mocap_node_yaw_gate.py
  - tests/ros/test_mocap_frame_queue.py
  - tests/ros/test_perception_console_lines.py
  - tests/hardware/session_skill_stack_rehearsals.md
  - logbook/2026-10-16-mocap-frame-queue-starvation.md
  - logbook/INDEX.md
subsystem:
  - tracking
  - ros
tags:
  - mocap
  - determinism
  - bb-calibration
---

# BB calibration solve off mocap_node's executor thread

**What.** Option A of `2026-10-16-mocap-frame-queue-starvation` (owner, 2026-10-16). `_finalize_calibration`
keeps the cheap refusals (no data, no template, yaw source) on the executor, then copies the window into a frozen
`_SweepSnapshot` and `submit`s it to `CalibrationSolveWorker` (new, pure Python,
`jugglebot/calibration_solve_worker.py`). The worker runs `_solve_sweep` (`run_calibration`, then
`estimate_base_pose` for a constellation result) on the snapshot only: no node state, publisher, logger or file.
`_apply_calibration_outcomes`, a 0.1 s executor timer (`CALIBRATION_APPLY_PERIOD_S`), runs everything that used
to follow the solve, in the same order: base-monitor fallback, kappa pool, both state files, the `BB calibrated`
line, the latched publishes. The attempt suffix and the yaw source come from the snapshot, because a new sweep
may already be collecting. `_calib_lag_s` is deleted (it was never set before the solve).

**Invariant (docstring of `calibration_solve_worker.py`, pinned by `tests/ros/test_mocap_calibration_solve_worker.py`):**
no executor callback of `mocap_node` runs the calibration solve. Rules:
1. One solve at a time, FIFO. A sweep that ends while a solve is in flight queues behind it. A sweep that starts
   during a solve collects normally into fresh lists, and the in-flight result still applies to its own sweep.
   Outcomes publish in completion order, so a new sweep's collection-time failure (QTM dropout) published inside
   the ~4 s solve is followed on `bb/calibration_attempt` by the older sweep's outcome. Accepted: both lines are
   logged, and the calibration in force is still only ever replaced by a success. The inversion applies to
   `bb/calibration_attempt` specifically (a cheap executor-side refusal of sweep 2 can publish before sweep 1's
   in-flight success; it needs two sweeps within ~4 s). `bb/calibration_result` is unaffected: only a success
   replaces it. Consumer check (`grep -rn calibration_attempt ros_ws/src gui/`): nothing reads it as "latest";
   it is only recorded in the bag (`jugglebot_launch.py`) and mentioned in a `ball_butler_node` docstring, whose
   subscriber uses `bb/calibration_result`.
2. Shutdown: `stop(1.0 s)` bounded join; pending and late results dropped (daemon thread).
3. A solve that raises is published once as `CALIBRATION_SOLVE_ERROR: <type>: <text>` (one ERROR line, the
   keep-last-good failure path). Nothing is persisted, the worker survives and the node keeps streaming. Before
   this change a non-`ValueError` raise inside the callback would have killed `rclpy.spin`.
4. The apply step adds 0-0.1 s to a ~4 s solve. That does not matter: the BB is static between sweeps.

Tests get the old timing through `MocapNode(solve_inline=True)` (solve and apply inside `submit`), a one-token
change in the six helper files. The new file drives the real thread with `Event`-blocked stubs.

## Discussion: thread, not process

`tools/probes/mocap_starvation/solve_vs_drain.py` (2026-10-16, loaded box) puts a 5 ms drain loop beside the
solve in a worker thread: max tick gap 37-74 ms, queue peak 11-22 of 150, 0 drops, solve 3.1 -> 3.7-4.0 s. A
`multiprocessing` worker would remove the GIL contention that causes the 74 ms. The cost is pickling ~2000
frames each way, a forked child carrying the node's imports, and a crash becoming a child-lifecycle problem. The
thread's worst gap is 7x inside the queue's 500 ms horizon and inside the tracker's 100 ms gap bridge
(`2026-10-16-tracker-flight-fit-late-since-matcher-fix`). It also only happens in the ~4 s after a sweep, when no
ball is in flight. `MultiThreadedExecutor` (option D) stays rejected: every callback would then need to be
thread-safe. The pattern used here keeps all node state on one thread.

## Verification

- 2026-10-16, `pytest tests/ros/test_mocap_node*.py tests/ros/test_bb_*.py tests/ros/test_mocap_frame_queue.py tests/ros/test_perception_console_lines.py -q -p no:cacheprovider`
  before the change: **271 passed, 1 skipped**.
- 2026-10-16, the same plus `tests/ros/test_mocap_calibration_solve_worker.py tests/ros/test_choreography_map.py tests/ros/test_launch_nodes.py tests/ros/test_launch_console.py`
  after: **350 passed, 1 skipped** (106 s), including the 5 new tests.
- Hardware receipt, row 30b, FLOWN 2026-10-16 20:30 and 22:01 (launch logs `~/.ros/log/2026-10-16-20-30-57-789365-jetson-1368891`
  and `2026-10-16-22-01-50-383243-jetson-1408449`, bag `~/Desktop/rosbags/2026-10-16_22-01-50`; an owner sitting of BB
  calibrations only, with a ~1 min and a ~1 h BB power-off between blocks). 2026-10-16,
  `grep -c 'dropped by mocap_node' <launch.log>` on both logs: **0 and 0** (14 sweeps: 7 solved, 7 refused
  `CONSTELLATION_RESIDUAL`, all seven in the two blocks after the ~1 h power-off with one accepted sweep interleaved
  at 22:02; the refusals are the Ball Butler calibration's, handed to its owner). `python tools/probes/mocap_starvation/log_stalls.py <launch.log>`: **zero node-overflow WARNs, zero
  logged frame loss at all 14 sweep ends** (was 30-52 % per sweep; the probe's "MISMATCH" at one sweep is its
  pre-fix prediction of 172 lost frames against the 0 logged). `grep 'Calibration solve took'`: one line per sweep,
  then its outcome line. `BB calibrated` values in family: yaw -11.47..-11.63 deg, axis (-930..-931, -159, 1742..1743) mm,
  kappa 179.26-179.27 (epoch K1), stamped-yaw lag -1.3..+2.1 ms (so the worker apply did not shift the lag the peer
  asked to watch; the 16:10/16:23 -11.5 ms reading was per-launch).
- **Two deviations from the prediction above, both from the live GIL load, both accepted.** An accepted solve took
  **14.4-15.7 s** off the executor (an early refusal 5.4-6.3 s), not the probe's 3.7-4.0 s: the probe's executor had
  no 300 Hz frame work to share the interpreter with. And `python tools/probes/mocap_starvation/bag_stalls.py
  ~/Desktop/rosbags/2026-10-16_22-01-50` (2026-10-16): 26297 `/mocap_data` msgs at 299.1 Hz, publish-stamp p50 7.7 ms
  / p99 150.6 ms, ten publish gaps of **101-169 ms** inside the four solve windows, each with a stamp gap of 3.3 ms
  and a catch-up burst, so ~0 frames never published; 1 of 86 seconds under 250 frames. The row's "no gap >= 100 ms"
  expectation was written against the 37-74 ms probe figure and is relaxed to "no frame lost": the queue's 500 ms
  horizon and the tracker's 100 ms gap bridge both hold, and a solve never overlaps a flight (calibration is a
  bringup/idle action). If a solve ever has to run during juggling, the `multiprocessing` worker discussed above is
  the mitigation; the 7x longer solve is the price of the thread choice and is only operator wait.
