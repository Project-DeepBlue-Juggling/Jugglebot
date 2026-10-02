---
title: "rosbridge: one pooled service client per service, created and sent on the executor thread"
type: bugfix
date: 2026-10-02
status: resolved
phase: standalone — GUI rosbridge
files_changed:
  - ros_ws/src/jugglebot/jugglebot/rosbridge_websocket_lean.py
  - tests/ros/test_rosbridge_websocket_lean.py
  - ros_ws/gui/js/ros-bridge.js
  - tools/gen_choreography_map.py
  - tests/ros/test_choreography_map.py
subsystem:
  - ros
  - gui
---

## Symptom

Two rosbridge ERRORs recur in the bags. A `/rosout` scan of every bag since
the lean rosbridge landed (2026-09-14 .. 10-02, 31 bags, ~14,000 s of GUI
time):

| Error | Count | Rate |
|---|---|---|
| `rosbridge_spin: unhandled exception … AssertionError` (`rclpy/callback_groups.py:103`) | 12 in 9 launches (11 bagged + today's unbagged 13:25 launch) | ~1 per 400 GUI service calls |
| `call_service TimeoutError: service call to '/rosapi/topics' timed out after 50.0 s with no response` | 5 in 4 sittings | ~1 per 900 calls |

Every AssertionError traceback is the same line,
`clients.extend(filter(self.can_execute, node.clients))`. rosapi stayed up
(10-02: no rosapi error in `launch.log`), and the poll 3 s after each lost
reply worked. The 3 `bb/aim` timeouts of 2026-09-28 21:41 are a different fault:
three in a row is the ball_butler_node that had stopped answering, as
diagnosed then.

## Diagnosis

The GUI polls `/rosapi/topics` every 3 s (`ros_ws/gui/js/main.js`), and
rosbridge built a fresh client for each call on the `ServiceCaller` worker
thread. Foxy rclpy 1.0.13 is unsafe for that in two places, neither under a
lock:

- `Node.create_client` appends to the node's client list (`node.py:1273`)
  one line before `callback_group.add_entity` (`:1274`). If the spin thread
  builds a wait set between the two, `MutuallyExclusiveCallbackGroup
  .can_execute` asserts. Nothing else in rosbridge can leave a client in that
  state, since the destroy side already ran on the executor thread
  (`ClientReaper`, 2026-09-14).
- `Client.call_async` sends the request (`client.py:126`) before putting the
  future in `_pending_requests` (`:131`). A reply the executor takes in that
  gap hits `except KeyError: # The request was cancelled` (`executors.py:365-367`)
  and is dropped. The caller then waits out the 50 s bound.

The assert is harmless as handled: `spin_forever` catches it before any wait
or take, and the next pass picks the client up. The dropped reply matters
more. Nothing confines it to rosapi: an actuating GUI call
(`bb/throw_at_target`, `clear_errors`, …) whose reply is dropped has already
run on the server, but the GUI reports a failure, so an operator retry acts
twice. The 2026-09-15 "one service reply never came", whose cause that entry
never found, fits the same mechanism: the discovery poll is nearly all of the
GUI's call volume.

## Discussion

**The timeout mechanism was reframed twice.** The first lead was the
`call_async` gap, read from the code. A standalone probe (a Foxy server plus a
rosbridge-shaped client node, 3000 calls per mode, domain 77) reproduced
neither fault. One reply also went missing in its *fixed* mode, which cannot
lose a reply to that gap, though its 1 s timeout couldn't tell lost from slow.
That pointed at the second candidate: each fresh client makes the server
wait for DDS to match a new reply reader. Fast-DDS 1.3.2 bounds that wait at
100 ms and carries a "client will not receive response" path, and the
probe's per-call p99 sat at that cliff (99.8 ms, max 108 ms). The fix was
chosen to close **both** candidates, so it didn't depend on picking one. The
discriminating evidence came later, from the real rosbridge + rosapi under
load (Verification): HEAD lost 12 replies, and an orphaned-reply counter on
HEAD's executor caught `/rosapi/topics seq 1` being taken before `call_async`
registered it. Candidate (i) is confirmed as a live path; (ii) was never
observed.

**Why a pool, not "create on the executor thread, still one per call".**
Moving `create_client` + `call_async` onto the executor thread alone closes
both rclpy gaps. But every call would still pay fresh-client DDS matching:
candidate (ii), plus the ~100 ms p99 tail. And it would keep the
create/destroy churn behind the 2026-09-14 leak. A long-lived client per
(service, type) pays matching once per launch. It is bounded by distinct
services (~20), not calls. A long-lived client sits in every wait set, which
makes the `call_async` gap *more* reachable from a worker thread, so sending
on the executor thread is what makes the pool safe, not an extra.

**`service_is_ready()` before the first send.** This closes candidate (ii)
for the first call to each service after launch or after a server restart.
That first call matters most for actuating services, which are called rarely.
The cost is a poll at `SERVICE_READY_POLL_S` (20 ms) inside the existing 50 s
bound. A server that never becomes visible now times out with a message that
says so ("never became visible") instead of sending into the void.

**Ruled out: patching rclpy's callback group** (make `can_execute` return
False for an unregistered entity instead of asserting). One line, and it
silences the assert for subscriptions too. But it hides the class instead of
closing it, and does nothing for the dropped reply.

**Accepted:** the subscription twin of the assert stays open. rosbridge's
`Subscribe` creates subscriptions on the IncomingQueue thread, the same
race on `node.subscriptions`. It was hit 0 of 12 times: ~44 creations per
connect, a few connects per sitting, against a client every 3 s. Closing it
means reaching into rosbridge_library's subscription manager.

## Fix

`rosbridge_websocket_lean.py`, module docstring item 6:

- `ServiceClientPool`: one client per (service, type), created and every
  `call_async` sent in `_send_if_ready`, which runs only on the executor
  thread. The worker thread only waits. On timeout, `remove_pending_request`
  is also scheduled onto the executor thread, the reader of
  `_pending_requests`. A send whose caller has given up is marked abandoned
  under the lock it runs under, so it never goes out late.
- `ExecutorThreadRunner` replaces `ClientReaper` (no per-call client is left to
  destroy) and the identical-shaped `ProtocolReaper`. Its `_drain` now runs
  every queued callable even if one raises, re-raising the first afterwards
  for `spin_forever` to log, so a raising teardown can't strand a queued send
  until the next trigger.
- `build_call_service(svc, pool, timeout_sec)`: stock 1.3.1's request
  building, verbatim; the call goes through the pool.

`ros-bridge.js::discoverTopics`: one poll in flight, with a 5 s local bound
(`T_DISCOVERY_MS`). A silent rosapi used to collect ~16 concurrent 50 s calls
per tab. The two choreography-map comments that said "a client per request"
now say "on demand".

## Verification

- **Scoped tests**, 2026-10-02:
  `pytest tests/ros/test_rosbridge_websocket_lean.py tests/ros/test_choreography_map.py tests/ros/test_gui_bb_aim_timeout.py -q`
  gave **68 passed in 16.00 s**. The contract test
  (`test_pool_creates_and_sends_only_on_the_executor_thread`) runs the pool
  against a real second thread and checks the thread of every
  `create_client` / `service_is_ready` / `call_async`.
- **Real rclpy**, 2026-10-02 (domain 77, scratch probe: Foxy `Trigger`
  server, 500 Hz load topic, 61 subscriptions, GIL-hogging thread):
  pool + `ExecutorThreadRunner`, 3000 calls: **0 timeouts, 0 asserts,
  `create_client` once, on the spin thread**; p99 50.0 ms, max 67 ms. The
  per-call-client baseline under the same harness was p99 99.8 ms, max 108 ms.
- **Real rosbridge + real rosapi**, 2026-10-02, interleaved A/B with
  `python3 tools/probes/rosbridge_cpu_probe.py leak --server lean --calls 1200`
  (domain 87; `--idle-seconds 5` for the A/B pairs). The HEAD arm ran the same probe from a scratch tree holding
  HEAD's module. A live launch was up on domain 0 throughout
  (load average ~9, sitting-like conditions):
  - HEAD: **12 lost replies** (50 s `TimeoutError` on `/rosapi/topics`) in
    1706 calls. Both runs hit the probe's time cap: 1021 and 685 calls.
  - pooled: **0 lost in 3600 calls** (3 runs). Mean latency 8.4–11.6 ms,
    one client created per run, no ERROR/WARN, idle CPU 0.00–0.20% after.
  - HEAD with an orphaned-reply counter subclassing the executor: an orphan
    on `/rosapi/topics seq 1` before any timeout. That is the `call_async`
    gap, observed directly. The run was cut short to spare the live sitting.
- **Gate**, 2026-10-02 (`./run_tests.sh`, run after the live sitting
  ended): **PASS**. Parallel phase 5783 passed, 9 skipped in 221.97 s;
  serial phase 3 passed in 9.21 s; 236 s total.
- **Field check (follow-up)**: re-run the bag `/rosout` scan after ~4 h of GUI
  time. At the old rates that window would hold ~12 asserts and ~5 timeouts.
  Zero rosbridge ERRORs closes it.

Deploy: `colcon build --packages-select jugglebot` (the launch runs the
installed copy) and relaunch.
