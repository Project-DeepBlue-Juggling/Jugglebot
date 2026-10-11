---
title: skill_node starved its own service responses — two ABORTED_BB_THROW_TIMEOUT ends while Ball Butler had answered and thrown
type: investigation
date: 2026-10-11
status: resolved
phase: "two-ball-skill-stack — R5 (Ball Butler calibration) / two-ball columns"
related_plan: two-ball-skill-stack.md
related:
  - 2026-10-11-skill-stack-r5-hop-late-limit-jerk.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/ros/test_skill_node.py
  - tests/ros/conftest.py
  - tools/probes/skill_node_response_starvation_probe.py
  - logbook/2026-10-11-skill-stack-r5-skill-node-response-starvation.md
  - logbook/INDEX.md
subsystem:
  - ros
tags:
  - concurrency
  - ball-butler
---

# skill_node starved its own service responses

## What happened

Sitting 7 re-fly (2026-10-11 evening; launch.log `~/.ros/log/2026-10-11-22-15-28-601295-jetson-3213327`, bag
`~/Desktop/rosbags/2026-10-11_22-15-28`). Two `columns (reload) ENDED (ABORTED_BB_THROW_TIMEOUT)` lines (log L1597,
L1779), but the owner saw Ball Butler throw both times. Both episodes have the same order (L1590-1600 is typical):

- ball_butler_node answered `bb/throw_at_target` at 1695.690, and trajectory_node planned the bridge REST install
  in 81.8 ms (1695.778).
- skill_node saw neither reply. At exactly +2.0 s it logged `END SERVICE_TIMEOUT ... trajectory/install_segment`
  (1697.689), then ABORTED_BB_THROW_TIMEOUT (1697.694). Its text said BB "answered nothing".
- Only then did the feed compile log "swapped the executor" and "feed accepted" (1697.698-.700).
- BB's throw went out regardless (`bb/throw: OK` 1698.26).

## Cause

All four threads of skill_node's `MultiThreadedExecutor(num_threads=4)` were parked in blocking waits at the same
time:

1. a tick in `_wait_future` on the BB throw;
2. the next tick (the timer's group is Reentrant) in `_wait_future` on the REST install, holding `_tick_lock`;
3. the BB announcement callback waiting on `_tick_lock` for the feed compile;
4. the Juggle action's wait loop.

Foxy rclpy hands a ready client response to whichever pool thread is free, so there was none left to deliver the
two replies those waits were polling for. Ten other attempts in the same log survived with 0.1-1.2 s near-miss
gaps, so this is a race, not a constant.

Two consequences:

- **Misattribution.** The BB path's `throw_resp is None` text claimed BB itself was silent. It overwrote the real
  first end (SERVICE_TIMEOUT).
- **Latent re-arm.** The feed compile won `_tick_lock` after the END. `threading.Lock` is unfair, and it got in
  before `_end_attempt` cleared `_reload_ctx`. It armed a fresh executor behind the decided outcome. Nothing moved,
  because `_end_attempt` flagged that executor `attempt_ended` before any tick ran it. But no guard prevented a
  tick in between from dispatching its skill 0.

## Discussion

- **Raising `num_threads` was rejected.** The number of callbacks that can block at once is not bounded: the
  Reentrant 25 ms timer takes another thread on every fire while earlier ones are blocked, and the go_to_pose,
  prelevel and stop paths each wait up to 4.3 s on the lock. More threads only move the cliff.
- **A reserved callback group cannot fix it either.** Foxy's `MultiThreadedExecutor` lets any pool thread take any
  ready entity (`rclpy/executors.py` ~L503-508, L654), so no group can reserve a free thread.
- **Converting every blocking `_wait_future` to done-callbacks** is the structurally cleanest fix, but it rewrites
  the sequential install, stop and reload logic. It was rejected for this unit.
- **Chosen: a dedicated client node.** Every client skill_node waits on lives on a small helper node
  (`skill_node_clients`), spun by its own `SingleThreadedExecutor` in a daemon thread. Response delivery therefore
  no longer competes with blocking callbacks, whatever their number, and `_wait_future` polls the future across
  threads unchanged. The five clients are install_segment, go_to_pose, hold, bb/reload and bb/throw_at_target.
- **Control/timing implications.** No schedule, segment or 40 Hz cadence change. Replies arrive in milliseconds
  instead of at the 2 s timeout, so the REST install and the BB claim happen in order.

## Fix

`skill_node.py`:

- **The helper client node.** Created in `__init__`; `main` spins it on its own thread and shuts it down in
  `finally`. `num_threads` stays 4.
- **`_ended_code_for_swap`.** An END is either a captured `_goal_end_code` or a live executor with `attempt_ended`,
  which flips only on a refusal. All three reload executor swaps now decline after an END and log one line: the
  columns feed, the one-ball reload and the `columns_1ball` reload.
- **`_end_attempt` is first-end-wins.** A later derived END is logged and ignored, not written over the first.
- **Neutral BB no-response text.** "no response ... within N s (the request may still have been executed -- check
  ball_butler_node's log)".

`tests/ros/conftest.py` mocks `SingleThreadedExecutor`. Probe: `tools/probes/skill_node_response_starvation_probe.py`.

## Verification

- **Real-rclpy probe**, 2026-10-11, `bash -c 'source /opt/ros/foxy/setup.bash && python3
  tools/probes/skill_node_response_starvation_probe.py'`: (a) same-node client with all pool threads blocked,
  `done=False after 1001 ms`; (b) helper-node client, `done=True after 2 ms`. REPRODUCED + FIX CONFIRMED.
- **Scoped**, 2026-10-11, `pytest tests/ros/ -q -p no:cacheprovider -n 4 -k skill_node`: 230 passed, before the two
  swap-guard tests were added.
- **New tests:** clients on the helper node; `main` spins it; the swap declines after a captured END and over a
  refused live executor; the BB text is neutral and the first end wins. With the live-executor branch of
  `_ended_code_for_swap` disabled, the refused-live-executor test fails (2026-10-11, `-k swap_declines`: 1 failed,
  1 passed).
- **Gate**, 2026-10-11, `./run_tests.sh` (log `temp/logs/gate_client_node_20261011.log`): 6667 passed, 9 skipped in 304 s; serial 3 passed; exit 0.
- **Not yet live on hardware.** Needs `colcon build --packages-select jugglebot` and a relaunch (done at commit);
  the next sitting is the hardware check.
- **Not fixed:** the same blocking-poll shape in `teensy_bridge_node.py:4795` (recover reseed, default-thread
  `MultiThreadedExecutor()`). It is not shown to starve in practice.
