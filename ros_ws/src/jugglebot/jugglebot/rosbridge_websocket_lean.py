"""rosbridge_websocket_lean.py — a lean drop-in replacement for
rosbridge_server's stock `rosbridge_websocket` executable (1.3.1, Foxy).

The GUI's rosbridge used 48% of a core in the live launch. Two causes lived in
the stock script and its library, and a third in the GUI's own JS (fixed in
ros_ws/gui/js/ros-bridge.js, not here):

1. **Service-client leak.** `rosbridge_library/internal/services.py::call_service`
   runs `node_handle.create_client(...)` on every call and never destroys the
   client (rclpy keeps it in `node.__clients`, scanned on every executor wait).
   The GUI's topic discovery calls `/rosapi/topics` every 3 s, so this leaks one
   client every 3 s for the life of the process — it persists after the browser
   disconnects. Measured 2026-09-13: idle rosbridge CPU was 2.95% before and
   59.45% after 1,200 calls (an hour of GUI discovery); per-call latency went
   from 7.3 to 38.9 ms. Fixed first (2026-09-14) by destroying each client
   after its call, on the executor thread; since item 6 there is no per-call
   client left to leak — `ServiceClientPool` keeps one per service, so the
   client count is bounded by the services the GUI calls, not by its calls.
2. **1 ms timer spin.** Stock `main()` drives the executor from a tornado
   `PeriodicCallback(lambda: executor.spin_once(timeout_sec=0.01), 1)` — a
   1 kHz poll from inside the IOLoop. py-spy put 68% of rosbridge's CPU in
   rclpy's `_wait_for_ready_callbacks`. A dedicated thread blocking in
   `executor.spin_once()` (no timeout — see `spin_forever` below) measured
   34.7% -> 24.3% under GUI load, and 4.8% -> 0.03% with no client connected.
   Upstream made the same change.
3. **Unbounded service wait blocking the connection's queue thread.**
   rosbridge_server's `IncomingQueue` runs one thread per browser connection
   executing `protocol.incoming(msg)` FIFO; `call_service` runs synchronously
   on that thread via stock 1.3.1's `client.call(inst)`, which blocks with no
   timeout (`rclpy.client.Client.call`). One service reply that never came
   (2026-09-15 sitting) blocked that thread forever, so a later message on
   the same connection (an IDLE click) never executed. `ServiceClientPool
   .call` below replaces `client.call(inst)` with a bounded wait on the
   reply, so a hung service call surfaces as a `service_response` with
   `result: False` (via stock `CallService._failure`) instead of wedging the
   connection. (Item 6 is the likely reason that reply never came.)
4. **Leaked subscriptions on reconnect.** With the queue thread blocked as in
   (3), a tab refresh's `on_close()` only called `incoming_queue.finish()` —
   `Protocol.finish()` (which unsubscribes every capability) runs on the
   blocked thread and never ran, so the old connection's subscriptions leaked
   and rosbridge logged `WebSocketClosedError` at 1 Hz for the rest of the
   launch. The patched `on_close` below calls `protocol.finish()` directly
   (bounded now that (3) is fixed), guarded so a second call from the queue
   thread once it unblocks is harmless — and, since `Subscribe.finish()` /
   `Advertise.finish()` reach `node.destroy_subscription()` /
   `node.destroy_publisher()`, which mutate the same node-internal entity
   lists the executor thread iterates (the hazard item 6 closes for service
   clients), the guarded `finish()` never runs inline off the executor
   thread: it's scheduled there via an `ExecutorThreadRunner` — and because stock
   `IncomingQueue.run()`'s own trailing `self.protocol.finish()` call goes
   through this same guarded, reaper-backed `protocol.finish` attribute, it
   is covered for free, with no change to `IncomingQueue` itself.

5. **One unanswered service call still stalled the whole tab for 50 s.** Item
   3 bounded the wait but left it on the queue thread; every other message from
   that tab waited behind it (2026-09-28 21:41 sitting: a deaf
   `ball_butler_node` held the GUI for 50 s per `bb/aim`).
   `run_service_calls_off_the_queue_thread` runs each call on its own thread.

6. **Service clients were created, and requests sent, off the executor
   thread.** Foxy's `Node.create_client` appends the client to the node's
   client list one line before registering it with its callback group
   (`rclpy/node.py` 1273-1274), and `Client.call_async` sends the request
   before recording that a reply is expected (`rclpy/client.py` 126-131) —
   no lock in either. rosbridge did both on a worker thread while
   `spin_forever` iterated those same lists: a wait set built between the two
   lines raised `AssertionError` (`rclpy/callback_groups.py` 103; 12 times
   2026-09-16..10-02, every one on `node.clients`), and a reply the executor
   takes in the second gap is dropped as "cancelled" (`rclpy/executors.py`
   365-367). A fresh client per call also made every call wait for DDS to
   match a new reply reader (probe p99 ~100 ms, Fast-DDS's 100 ms bound), and
   five GUI `/rosapi/topics` polls got no reply at all (a 50 s
   `TimeoutError`) — which of those two mechanisms lost them is not pinned
   down. `ServiceClientPool` closes both: one long-lived client per service,
   created on the executor thread and used once its server is visible to it,
   and every `call_async` issued there too, so the thread that takes replies
   is the thread that registers them. The worker thread (item 5) only waits.

Both numbers, the bench methodology and the acceptance measurements for (1)
and (2) are recorded in `logbook/2026-09-14-rosbridge-cpu-leak-and-spin.md`;
(3) and (4) are recorded in
`logbook/2026-09-15-rosbridge-hung-call-and-close-teardown.md`; (6) in
`logbook/2026-10-02-rosbridge-service-clients-on-executor-thread.md`.

This module is imported by unit tests with no ROS2 or rosbridge_server on the
path (the test gate runs against mocked rclpy modules — see
`tests/ros/conftest.py`), so nothing here imports `rosbridge_library`,
`ament_index_python`, `tornado` or real `rclpy` at module scope: those imports
live inside `main()` and the functions it calls, and only run when this
module is actually launched as the rosbridge process.
"""

from __future__ import annotations

import collections
import importlib.util
import os
import sys
import threading
import time
import traceback

# Bound on how long a single `call_service` may block its thread (see this
# module's docstring, item 3; since item 5 that is the call's own worker
# thread, not the connection's IncomingQueue thread). Must be ABOVE
# the GUI's largest per-step client-side service timeout, so the GUI's own
# `Promise.race` always times out first and the server-side bound only fires
# as a backstop (e.g. a browser tab gone with the request still in flight).
# The largest GUI client timeout is `T_SVC_CLEAR = 45000` ms
# (ros_ws/gui/js/state-minimap.js, the armed-reroute clear_errors step, which
# blocks through a reseed+converge) — every other `T_SVC_*` there is smaller,
# and `ros_ws/gui/js/ros-bridge.js::callService` (used by panels.js for
# bb/calibrate, bb/reset, bb/throw_at_target) sets no client-side timeout at
# all, so those calls rely entirely on this server-side bound. 45 s + 5 s
# margin = 50 s.
CALL_SERVICE_TIMEOUT_S = 50.0

# How often a call re-checks a server its client cannot see yet (module
# docstring item 6). Only the first call to a service after launch, or after
# its server restarts, ever waits on this; the whole wait sits inside
# CALL_SERVICE_TIMEOUT_S.
SERVICE_READY_POLL_S = 0.02


class ExecutorThreadRunner:
    """Runs scheduled zero-arg callables on the executor thread.

    `rclpy.node.Node` keeps its clients, subscriptions and other entities in
    plain lists that `SingleThreadedExecutor.spin_once()` iterates on
    `spin_thread` while building each wait set, so an rclpy call that
    creates or destroys an entity — or sends a request whose reply the
    executor will take (module docstring item 6) — must run on that thread.
    A guard condition is the standard rclpy way to hand work there:
    triggering it wakes `spin_once()` and runs `_drain` on the executor
    thread. `main()` builds two: one behind `ServiceClientPool` (item 6), and
    one running each closed connection's `Protocol.finish()` (item 4), whose
    `Subscribe.finish()` → `node.destroy_subscription()` and
    `Advertise.finish()` → `node.destroy_publisher()` are the same hazard.
    """

    def __init__(self, node):
        self._node = node
        self._pending = collections.deque()
        self._guard = node.create_guard_condition(self._drain)

    def schedule(self, fn):
        """Queue the zero-arg callable `fn` to run on the executor thread.
        Safe to call from any thread."""
        self._pending.append(fn)
        self._guard.trigger()

    def _drain(self):
        """Guard-condition callback — runs on the executor thread. Pops and
        runs every pending callable, leaving the queue empty. One callable
        raising does not strand the ones queued behind it (they would wait
        for the next trigger — a service call's send among them); the first
        error is re-raised once the queue is empty, for `spin_forever` to
        log."""
        errors = []
        while self._pending:
            fn = self._pending.popleft()
            try:
                fn()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise errors[0]


def _run_on_executor_and_wait(run_on_executor, fn, deadline):
    """Run `fn` on the executor thread, via `run_on_executor` (an
    `ExecutorThreadRunner.schedule`), and block the calling thread until it
    has run.

    Returns `(True, <fn's return value>)`, re-raises what `fn` raised, or
    returns `(False, None)` if `deadline` (a `time.monotonic()` value) passes
    first. In that last case `fn` is marked abandoned under the same lock it
    runs under, so it never runs later: a request its caller has given up on
    is never sent.
    """
    ran = threading.Event()
    lock = threading.Lock()
    box = {'abandoned': False}

    def task():
        with lock:
            if box['abandoned']:
                return
            try:
                box['value'] = fn()
            except Exception as exc:  # handed to the caller, not the executor
                box['error'] = exc
            ran.set()

    run_on_executor(task)
    if not ran.wait(max(0.0, deadline - time.monotonic())):
        with lock:
            if not ran.is_set():
                box['abandoned'] = True
                return False, None
    if 'error' in box:
        raise box['error']
    return True, box['value']


class ServiceClientPool:
    """One long-lived service client per service, created — and every
    request sent — on the executor thread (module docstring item 6).

    `call()` runs on a `ServiceCaller` worker thread (item 5) and only waits.
    `_send_if_ready()` is the one place a client is created or a request is
    sent, and it runs only on the executor thread, handed there through
    `run_on_executor` (an `ExecutorThreadRunner.schedule` in production).
    Because that is also the thread that takes replies, it cannot take this
    request's reply before `call_async` has registered it: it is busy
    running `_send_if_ready` until then.

    `_clients` is read and written only on the executor thread. It keeps one
    client per (service name, service type) for the life of the process —
    bounded by the distinct services the GUI calls, never by the number of
    calls, which is what item 1's leak was. A client whose server restarts
    stays valid; DDS matches it to the new server, and `service_is_ready()`
    holds the next call until it has.
    """

    def __init__(self, node, run_on_executor, log=None):
        self._node = node
        self._run_on_executor = run_on_executor
        self._log = log
        self._clients = {}

    def call(self, service, service_type, service_class, request, timeout_sec):
        """Send `request` to `service` and block until its reply arrives.

        Raises `TimeoutError` if `timeout_sec` passes first — whether the
        executor thread never ran the send, the server never became visible
        to the client, or the reply never came — and re-raises the reply
        future's exception if it carries one. Stock
        `CallService._failure` turns either into a `service_response` with
        `result: False`.
        """
        deadline = time.monotonic() + timeout_sec

        def send():
            return self._send_if_ready(
                service, service_type, service_class, request)

        while True:
            ran, sent = _run_on_executor_and_wait(
                self._run_on_executor, send, deadline)
            if not ran:
                raise TimeoutError(
                    f'service call to {service!r} timed out after '
                    f'{timeout_sec} s: the rosbridge executor thread never '
                    f'ran the send')
            if sent is not None:
                break
            if time.monotonic() + SERVICE_READY_POLL_S >= deadline:
                raise TimeoutError(
                    f'service call to {service!r} timed out after '
                    f'{timeout_sec} s: the server never became visible to '
                    f"rosbridge's client")
            time.sleep(SERVICE_READY_POLL_S)

        client, future, replied = sent
        if not replied.wait(max(0.0, deadline - time.monotonic())):
            # Done early so a late reply can't fire callbacks on a future
            # nothing waits on any more — on the executor thread, which is
            # the thread that reads _pending_requests.
            self._run_on_executor(
                lambda: client.remove_pending_request(future))
            raise TimeoutError(
                f'service call to {service!r} timed out after {timeout_sec} s '
                f'with no response')
        exc = future.exception()
        if exc is not None:
            raise exc
        return future.result()

    def _send_if_ready(self, service, service_type, service_class, request):
        """Executor thread only. Returns `(client, future, replied)` once the
        request is sent, or None while the server is not yet visible to the
        client (the caller polls)."""
        key = (service, service_type)
        client = self._clients.get(key)
        if client is None:
            client = self._node.create_client(service_class, service)
            self._clients[key] = client
            if self._log is not None:
                self._log.info(
                    f'rosbridge: created the service client for {service} '
                    f'({service_type}); every later call reuses it '
                    f'({len(self._clients)} clients)')
        if not client.service_is_ready():
            return None
        replied = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _future: replied.set())
        return client, future, replied


def build_call_service(svc, pool, timeout_sec=CALL_SERVICE_TIMEOUT_S):
    """Return a drop-in replacement for `rosbridge_library.internal.services
    .call_service(node_handle, service, args=None)`.

    The request-building half below is the 1.3.1 function verbatim (every
    helper referenced through `svc`, the real `rosbridge_library.internal
    .services` module, so this stays byte-for-byte faithful to upstream's
    request-building and error handling). The call itself goes through
    `pool` (a `ServiceClientPool`) instead of stock's per-call
    `create_client` + `client.call(inst)`: no client per call (item 1), a
    bounded wait (item 3), and no rclpy entity touched off the executor
    thread (item 6). A timeout raises `TimeoutError`, which stock
    `CallService._failure` turns into a `service_response` with
    `result: False` for the browser, same as any other service-call
    exception. Exceptions raised before the call — an unknown service
    (`svc.InvalidServiceException`) or bad args
    (`svc.args_to_service_request_instance`) — never reach the pool.
    """

    def call_service(node_handle, service, args=None):
        # Given the service name, fetch the type and class of the service,
        # and a request instance

        # This should be equivalent to rospy.resolve_name.
        service = svc.expand_topic_name(
            service, node_handle.get_name(), node_handle.get_namespace())

        service_names_and_types = dict(node_handle.get_service_names_and_types())
        service_type = service_names_and_types.get(service)
        if service_type is None:
            raise svc.InvalidServiceException(service)
        # service_type is a tuple of types at this point; only one type is supported.
        if len(service_type) > 1:
            node_handle.get_logger().warning(
                f"More than one service type detected: {service_type}")
        service_type = service_type[0]

        service_class = svc.get_service_class(service_type)
        inst = svc.get_service_request_instance(service_type)

        # Populate the instance with the provided args
        svc.args_to_service_request_instance(service, inst, args)

        result = pool.call(service, service_type, service_class, inst,
                           timeout_sec)
        if result is not None:
            # Turn the response into JSON and pass to the callback
            return svc.extract_values(result)
        raise Exception(result)

    return call_service


def run_service_calls_off_the_queue_thread(call_service_module):
    """Item 5 of this module's docstring: each ``call_service`` on its own thread.

    Stock 1.3.1 ``CallService.call_service`` ends with
    ``ServiceCaller(...).run()`` — "Run service caller in the same thread" —
    i.e. ON the connection's one ``IncomingQueue`` thread, so while a call waits
    for its reply every later message from that browser tab (any button, any
    other service call, any publish) waits behind it. Item 3 bounded that wait
    at :data:`CALL_SERVICE_TIMEOUT_S`; it did not remove it. MEASURED 2026-09-28
    21:41 sitting: ``ball_butler_node`` answered nothing, and each GUI ``bb/aim``
    held the tab for the full 50 s (three rosbridge ``TimeoutError`` lines) —
    the owner's "the GUI freezes until I refresh".

    ``ServiceCaller`` is a ``threading.Thread`` subclass whose docstring offers
    exactly this choice ("Use start() to start in a separate thread or run() to
    run in this thread"), so the patch swaps the module-global name the
    capability looks up at call time for a shim whose ``run()`` calls the real
    caller's ``start()``. The worker still goes through the patched
    ``internal.services.call_service`` (item 3's bound, item 6's pooled
    clients). Replies reach the socket through ``RosbridgeWebSocket.send_message``,
    which hands off via ``IOLoop.add_callback`` — thread-safe. Replies may now
    complete out of order; every GUI caller correlates by id.
    """
    stock_caller = call_service_module.ServiceCaller

    class _OffQueueServiceCaller:
        def __init__(self, *args, **kwargs):
            self._inner = stock_caller(*args, **kwargs)
            self._inner.daemon = True

        def run(self):
            self._inner.start()

    call_service_module.ServiceCaller = _OffQueueServiceCaller
    return _OffQueueServiceCaller


def _guard_protocol_finish(protocol, schedule):
    """Wrap `protocol.finish` (a `rosbridge_library.protocol.Protocol`
    instance) so repeat calls SCHEDULE the real teardown, via `schedule` (an
    `ExecutorThreadRunner.schedule` bound method in production), onto the executor
    thread at most once, thread-safely — never run it inline on whichever
    thread called `finish()`.

    `Protocol.finish()` calls `capability.finish()` for every registered
    capability, and `Subscribe.finish()` / `Advertise.finish()` reach
    `node.destroy_subscription()` / `node.destroy_publisher()` — the same
    entity-list-mutation-off-the-executor-thread hazard module docstring
    item 6 closes for service clients (see `ExecutorThreadRunner`'s
    docstring above), so `finish()` must
    never run inline off the executor thread regardless of how many times
    it's called.

    Audited 2026-09-15 against rosbridge_server 1.3.1's stock capability
    set for a SEPARATE property — whether running the real `finish()` twice
    is safe at all: `AdvertiseService`, `CallService`, `ServiceResponse` and
    `UnadvertiseService` have no override and inherit `Capability.finish`
    (`pass` — a no-op); `Advertise`, `Publish`, `Defragment` and `Subscribe`
    each clear their own container on first call, so a second *serial* call
    finds an empty container and its loop is a no-op, and the shared
    `Protocol.unregister_operation` guards itself
    (`if opcode in self.operations: del ...`) — so the real `Protocol
    .finish()` is idempotent when called serially. The lock below still
    only lets ONE of this module's two call sites — the patched `on_close`
    below, and stock `IncomingQueue.run()`'s trailing `self.protocol
    .finish()`, once its thread unblocks — win the schedule, so the real
    teardown is enqueued to the executor thread exactly once no matter which
    call site (or how many overlapping calls) gets there first. Because
    BOTH call sites go through this same wrapped `protocol.finish`
    attribute, the stock `IncomingQueue` trailing call is covered for free —
    it schedules onto the reaper exactly like the patched `on_close` does,
    without `IncomingQueue.run()` itself needing to change. See module
    docstring item 4 and
    logbook/2026-09-15-rosbridge-hung-call-and-close-teardown.md.

    Returns the wrapped callable; the caller assigns it to `protocol.finish`
    (`build_call_service`-style factory, not a decorator, so it stays
    testable with a plain fake protocol object). The wrapped callable
    returns `True` for the one call that won the schedule, `False`
    otherwise — a schedule, not a completion: the real `finish()` runs
    later, on the executor thread, once the reaper drains.
    """
    original_finish = protocol.finish
    lock = threading.Lock()
    state = {'scheduled': False}

    def finish_once():
        with lock:
            if state['scheduled']:
                return False
            state['scheduled'] = True
        schedule(original_finish)
        return True

    return finish_once


def _subscription_count(protocol, subscribe_class):
    """Number of live subscriptions on `protocol`'s `Subscribe` capability,
    read BEFORE `finish()` clears it — for the close-time log line. Returns
    0 if no capability of `subscribe_class` is registered (shouldn't happen
    for a real `RosbridgeProtocol`, but this is also called from tests with
    minimal fake protocols)."""
    for capability in protocol.capabilities:
        if isinstance(capability, subscribe_class):
            return len(capability._subscriptions)
    return 0


def patch_rosbridge_websocket(rb_websocket_cls, subscribe_class, log, reaper):
    """Patch a rosbridge_server `RosbridgeWebSocket` class (a tornado
    `WebSocketHandler`) in place, fixing module docstring item 4 (leaked
    subscriptions when the connection's `IncomingQueue` thread is blocked —
    since item 5 no service call runs on that thread, and item 3 bounds the
    ones that did, but the patch stays as the teardown path):

    - Wraps `open()` so, right after stock `open()` creates
      `self.protocol`, `self.protocol.finish` is replaced with the
      `_guard_protocol_finish` wrapper bound to `reaper.schedule` — so BOTH
      this patch's `on_close` and stock `IncomingQueue.run()`'s trailing
      `self.protocol.finish()` call go through the same one-shot guard,
      regardless of which thread gets there first, and neither ever runs
      the real teardown inline: it's always scheduled onto the executor
      thread via `reaper` (an `ExecutorThreadRunner`).
    - Wraps `on_close()` so, after stock `on_close()` runs (client-count
      bookkeeping, logging, `incoming_queue.finish()` — unchanged), it also
      calls `self.protocol.finish()` — which, since `patched_open` already
      replaced it, SCHEDULES immediate teardown instead of waiting for the
      connection's (possibly still-blocked) queue thread to reach it. Logs
      one INFO line naming how many subscriptions were scheduled for
      teardown, but only on the call that actually won the schedule (a
      same-tick double `on_close` or a close that beat the queue thread to
      it and then races it logs once).

    Mutates `rb_websocket_cls` in place; returns nothing. Called once from
    `main()`, on the real `rosbridge_server.RosbridgeWebSocket` class,
    after `RosbridgeWebsocketNode()` construction and before
    `stock.start_hook()` starts the tornado IOLoop that would accept
    connections.
    """
    stock_open = rb_websocket_cls.open
    stock_on_close = rb_websocket_cls.on_close

    def patched_open(self):
        stock_open(self)
        protocol = getattr(self, 'protocol', None)
        if protocol is not None:
            protocol.finish = _guard_protocol_finish(protocol, reaper.schedule)

    def patched_on_close(self):
        stock_on_close(self)
        protocol = getattr(self, 'protocol', None)
        if protocol is None:
            return
        n_subs = _subscription_count(protocol, subscribe_class)
        if protocol.finish():
            log.info(
                f'rosbridge on_close: scheduled immediate teardown of '
                f'{n_subs} subscription(s) (client '
                f'{getattr(self, "client_id", "?")}) instead of waiting on '
                f'the connection\'s queue thread')

    rb_websocket_cls.open = patched_open
    rb_websocket_cls.on_close = patched_on_close


def spin_forever(executor, is_ok, log, shutdown_exceptions=None):
    """The dedicated executor-thread body that replaces stock's 1 ms
    `PeriodicCallback`.

    Loops calling `executor.spin_once()` (no timeout — it blocks until there
    is work, rather than the stock 1 kHz poll) while `is_ok()` is true.
    `shutdown_exceptions` end the loop cleanly; any other exception is logged
    with a traceback and the loop continues.

    Why swallow-and-continue rather than let the thread die: stock rosbridge's
    callbacks raised into tornado's `PeriodicCallback`, which logs the
    exception and keeps calling the callback on the next tick — a bare
    `executor.spin()` thread here would instead die silently on one bad
    message, leaving the websocket up with no ROS data flowing and no visible
    error. This reproduces the stock resilience with a single thread instead
    of a 1 kHz timer.

    `shutdown_exceptions` defaults to the real
    `(rclpy.executors.ShutdownException, rclpy.executors.ExternalShutdownException)`
    pair, imported lazily here so this module stays importable with no rclpy
    on the path; tests inject fake exception classes instead. In practice only
    `ExternalShutdownException` (raised once `rclpy.shutdown()` has run) ends the
    loop from inside `spin_once()`: Foxy's `SingleThreadedExecutor.spin_once`
    swallows `ShutdownException`, which needs `executor.shutdown()`, and this
    module never calls that. `is_ok()` turning false is the other exit.
    """
    if shutdown_exceptions is None:
        from rclpy.executors import ExternalShutdownException, ShutdownException
        shutdown_exceptions = (ShutdownException, ExternalShutdownException)

    while is_ok():
        try:
            executor.spin_once()
        except shutdown_exceptions:
            break
        except Exception:
            log.error(
                'rosbridge_spin: unhandled exception in the executor thread, '
                'continuing:\n' + traceback.format_exc())


def _load_stock_script():
    """Load rosbridge_server's stock `rosbridge_websocket.py` under a distinct
    module name (so its `if __name__ == "__main__":` guard never fires) and
    validate the pieces `main()` below depends on.

    Fails loudly, naming the path and "expected rosbridge_server 1.3.1", if
    the file or any of `RosbridgeWebsocketNode` / `start_hook` /
    `shutdown_hook` is missing — a stock-script contract break should read as
    a clear error here, not an obscure failure deep inside `main()`.
    """
    import ament_index_python.packages as ament_packages

    prefix = ament_packages.get_package_prefix('rosbridge_server')
    path = os.path.join(prefix, 'lib', 'rosbridge_server', 'rosbridge_websocket.py')
    if not os.path.isfile(path):
        raise ImportError(
            f"rosbridge_server's stock rosbridge_websocket.py was not found at "
            f"{path!r} (expected rosbridge_server 1.3.1).")

    spec = importlib.util.spec_from_file_location(
        '_jugglebot_rosbridge_stock', path)
    stock = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(stock)

    for name in ('RosbridgeWebsocketNode', 'start_hook', 'shutdown_hook'):
        if not hasattr(stock, name):
            raise ImportError(
                f"rosbridge_server's stock rosbridge_websocket.py at {path!r} "
                f"has no `{name}` (expected rosbridge_server 1.3.1).")
    return stock


def main(args=None):
    if args is None:
        args = sys.argv

    stock = _load_stock_script()

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    import rosbridge_library.internal.services as svc
    from rosbridge_library.capabilities.subscribe import Subscribe
    from rosbridge_server import RosbridgeWebSocket

    rclpy.init(args=args)
    # RosbridgeWebsocketNode.__init__ does all of stock's parameter handling
    # and binds/starts the tornado HTTP(S) server — identical to stock main().
    node = stock.RosbridgeWebsocketNode()

    # Patch call_service (a module global ServiceCaller.run looks up at call
    # time, so this takes effect for every future call) to call through one
    # pooled client per service, created and sent on the executor thread
    # (module docstring items 1 and 6), with a bounded wait (item 3).
    service_runner = ExecutorThreadRunner(node)
    pool = ServiceClientPool(node, service_runner.schedule, node.get_logger())
    svc.call_service = build_call_service(svc, pool, CALL_SERVICE_TIMEOUT_S)

    # The hung-close fix (module docstring item 4): tear a connection's
    # subscriptions down immediately on close rather than waiting for its
    # (possibly still-blocked) IncomingQueue thread to reach it. The real
    # destroy_subscription()/destroy_publisher() calls that teardown makes
    # must run on the executor thread (the entity-list hazard of item 6), so
    # a second runner schedules them there instead of running inline off
    # tornado's I/O thread. Patching the class here,
    # before start_hook() starts the IOLoop that accepts connections, means
    # every connection for the life of this process goes through the
    # patched open()/on_close().
    # Item 5: a service call no longer runs on (and blocks) the connection's
    # IncomingQueue thread, so one unanswered call cannot stall every other
    # message from that browser tab.
    import rosbridge_library.capabilities.call_service as call_service_cap
    run_service_calls_off_the_queue_thread(call_service_cap)

    protocol_runner = ExecutorThreadRunner(node)
    patch_rosbridge_websocket(
        RosbridgeWebSocket, Subscribe, node.get_logger(), protocol_runner)

    # The spin-loop fix: one thread blocking in spin_once(), not a 1 kHz
    # tornado PeriodicCallback.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop_spin = threading.Event()
    spin_thread = threading.Thread(
        target=spin_forever,
        args=(executor,
              lambda: rclpy.ok() and not stop_spin.is_set(),
              node.get_logger()),
        name='rosbridge_spin',
        daemon=True,
    )
    spin_thread.start()

    try:
        stock.start_hook()
        # Stock tore the node down on the same thread that had been spinning
        # it. Here the executor lives on spin_thread, so stop and join it
        # first: destroy_node() mutates the entity lists that thread iterates
        # to build each wait set. wake() unblocks a spin_once() waiting with
        # no timeout. (A SIGINT raises KeyboardInterrupt out of start_hook()
        # and skips this block, as in stock.)
        stop_spin.set()
        executor.wake()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('Exiting due to SIGINT')
    finally:
        stock.shutdown_hook()  # shutdown hook to stop the server


if __name__ == '__main__':
    main()
