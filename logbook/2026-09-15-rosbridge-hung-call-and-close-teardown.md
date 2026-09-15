---
title: "rosbridge: bounded service calls + immediate close teardown"
type: bugfix
date: 2026-09-15
status: done
phase: standalone — idle CPU
files_changed:
  - ros_ws/src/jugglebot/jugglebot/rosbridge_websocket_lean.py
  - tests/ros/test_rosbridge_websocket_lean.py
---

## What / Why

rosbridge_server 1.3.1 runs one `IncomingQueue` thread per browser
connection, executing `protocol.incoming(msg)` FIFO; `call_service` runs
synchronously on that thread via stock's `client.call(inst)`
(`rclpy.client.Client.call`), which blocks with no timeout. At the
2026-09-15 sitting one service reply never came, the queue thread blocked
forever, and a later message on the same connection (an operator IDLE
click) never executed. A tab refresh then made it worse: the old
connection's `on_close()` only called `incoming_queue.finish()` —
`Protocol.finish()` (unsubscribe every capability) runs at the end of
`IncomingQueue.run()`, on the still-blocked thread, so it never ran; the
old subscriptions leaked and rosbridge logged `WebSocketClosedError` at
1 Hz for the rest of the launch.

Two fixes in `rosbridge_websocket_lean.py`:

1. **Bounded service calls.** `_call_with_timeout` replaces
   `client.call(inst)` with `call_async` + a `threading.Event` wait, bounded
   by `CALL_SERVICE_TIMEOUT_S = 50.0`. On timeout it calls
   `client.remove_pending_request(future)` and raises `TimeoutError`, which
   stock `CallService._failure` turns into a `service_response` with
   `result: False` — same path as any other service-call exception.
   50 s = the GUI's largest client-side service timeout
   (`T_SVC_CLEAR = 45000` ms, `ros_ws/gui/js/state-minimap.js`, the armed
   reroute's `clear_errors` step) + 5 s margin, so the GUI's own
   `Promise.race` always gives up first; `ros-bridge.js::callService` (used
   by `panels.js` for `bb/calibrate`/`bb/reset`/`bb/throw_at_target`) sets no
   client-side timeout at all, so those calls now rely entirely on this
   server-side bound instead of hanging forever.
2. **Immediate close teardown, scheduled onto the executor thread.**
   `patch_rosbridge_websocket` wraps the real
   `rosbridge_server.RosbridgeWebSocket` class: `open()` replaces the fresh
   `protocol.finish` with a `_guard_protocol_finish` wrapper (one-shot,
   lock-protected); `on_close()` calls stock `on_close` then
   `protocol.finish()`. Audited every stock capability's `finish()`
   (`rosbridge_library/capabilities/*.py`, 1.3.1): four inherit the base
   no-op, the other four (`Advertise`, `Publish`, `Defragment`, `Subscribe`)
   each clear their own container on first call and
   `Protocol.unregister_operation` self-guards, so the real `Protocol
   .finish()` is idempotent when called serially — but `Subscribe.finish()`
   / `Advertise.finish()` reach `node.destroy_subscription()` /
   `node.destroy_publisher()`, which mutate the same node-internal entity
   lists `SingleThreadedExecutor.spin_once()` iterates on the dedicated spin
   thread (item 2 of the 2026-09-14 entry) — the exact hazard `ClientReaper`
   already exists for in this file, just for subscriptions/publishers
   instead of service clients. A caught-in-review defect in this session's
   first pass called `protocol.finish()` inline from tornado's I/O thread,
   which would have raced that iteration. Fixed: a second reaper,
   `ProtocolReaper` (deque + guard condition, identical shape to
   `ClientReaper`), and `_guard_protocol_finish`'s one-shot wrapper now
   *schedules* the real `finish()` onto the executor thread via
   `reaper.schedule` instead of ever running it inline — thread-safe for the
   same reason `ClientReaper` is. Because both call sites (this patched
   `on_close`, and stock `IncomingQueue.run()`'s trailing
   `self.protocol.finish()` once its blocked call unblocks) go through the
   same wrapped `protocol.finish` attribute, the stock trailing call is
   covered for free — it schedules onto the reaper exactly like `on_close`
   does, with no change to `IncomingQueue` itself. Logs one INFO line naming
   how many subscriptions were scheduled for teardown.

## Verification

(2026-09-15, `source ~/Desktop/PDJ_venv/venv/bin/activate && pytest
tests/ros/test_rosbridge_websocket_lean.py tests/ros/test_choreography_map.py
-q`): **60 passed in 8.19 s** (`_call_with_timeout` result/timeout/exception
paths, the `CALL_SERVICE_TIMEOUT_S`-exceeds-GUI-timeout tripwire,
`build_call_service` under a never-completing client, `ProtocolReaper`
schedule/drain, `_guard_protocol_finish` scheduling exactly once +
concurrent-call safety (asserting the real `finish()` is NOT called until
the reaper drains), `_subscription_count`, and `patch_rosbridge_websocket`'s
schedule-not-inline/double-close/queue-thread-trailing-call/no-protocol
paths).

(2026-09-15, `pytest tests/sim/test_plans_index.py
tests/sim/test_logbook_front_matter.py -q`): **83 passed in 0.46 s**.

(2026-09-15, `./run_tests.sh --full`, log `temp/logs/r3_followups_full2_20260915.log`, the combined tree of the four same-day units): **PASS — parallel 6146 passed, 9 skipped, 2 xfailed in 286.47 s; serial 6 passed in 18.79 s.**
