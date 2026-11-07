---
title: "Rosbridge service pool names the server, not the executor, when the last ready poll loses to the deadline"
type: bugfix
date: 2026-11-08
status: complete
phase: night-shift
files_changed:
  - ros_ws/src/jugglebot/jugglebot/rosbridge_websocket_lean.py
  - tests/ros/test_rosbridge_websocket_lean.py
subsystem:
  - gui
tags:
  - rosbridge
  - service-pool
  - JB-209
---

# Rosbridge service pool: the timeout names the server once the executor has run

**What.** `ServiceClientPool.call` polls `service_is_ready()` on the executor thread. When the deadline passed while a later poll was still waiting, `_run_on_executor_and_wait` returned `ran=False` and `call` raised "the rosbridge executor thread never ran the send", although earlier polls had run. That made `test_pool_times_out_without_sending_when_the_server_never_appears` flake RED in the 2026-11-08 nightly. Now `call` remembers that a check ran at least once; a later `ran=False` raises the "server never became visible" text. An executor that never ran the first check still raises "never ran the send". Both messages are unchanged. The abandon-under-the-lock behaviour of `_run_on_executor_and_wait` is untouched, so a request whose caller gave up is still never sent.

**Why.** The wrong message points the diagnosis at the executor when the server was the absent party. This is the GUI service path, not the motion path.

**Test.** `test_pool_blames_the_server_not_the_executor_when_a_later_poll_is_dropped`: an executor stand-in runs the first scheduled task and drops the rest, 0.05 s timeout, asserts the "never became visible" text and that nothing was sent. `test_pool_never_sends_a_request_its_caller_abandoned` still pins "never ran the send".

## Verification

- Without the change (2026-11-08): production hunk reverted; `tools/night_shift_nonet.sh pytest -q tests/ros/test_rosbridge_websocket_lean.py` gave 1 failed (the new test), 36 passed.
- With it (2026-11-08, same command): 37 passed in 0.96 s.
