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
   from 7.3 to 38.9 ms. `ClientReaper` + `build_call_service` below reproduce
   the stock 1.3.1 `call_service` verbatim but destroy the client afterwards,
   on the executor thread (upstream's `ros2` branch made the same change).
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
   the same connection (an IDLE click) never executed. `_call_with_timeout`
   below replaces `client.call(inst)` with a bounded `call_async` +
   `threading.Event` wait, so a hung service call surfaces as a
   `service_response` with `result: False` (via stock `CallService._failure`)
   instead of wedging the connection.
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
   lists the executor thread iterates (the (2) hazard `ClientReaper` exists
   for), the guarded `finish()` never runs inline off the executor thread:
   it's scheduled there via a second reaper, `ProtocolReaper`, exactly like
   `ClientReaper` schedules a leaked client's destroy — and because stock
   `IncomingQueue.run()`'s own trailing `self.protocol.finish()` call goes
   through this same guarded, reaper-backed `protocol.finish` attribute, it
   is covered for free, with no change to `IncomingQueue` itself.

Both numbers, the bench methodology and the acceptance measurements for (1)
and (2) are recorded in `logbook/2026-09-14-rosbridge-cpu-leak-and-spin.md`;
(3) and (4) are recorded in
`logbook/2026-09-15-rosbridge-hung-call-and-close-teardown.md`.

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
import traceback

# Bound on how long a single `call_service` may block the connection's
# IncomingQueue thread (see this module's docstring, item 3). Must be ABOVE
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


def _call_with_timeout(client, request, timeout_sec):
    """A bounded replacement for stock 1.3.1's `client.call(inst)`
    (`rclpy.client.Client.call`, which waits with no timeout — see this
    module's docstring item 3).

    Mirrors `Client.call`'s own implementation (`call_async` + a
    `threading.Event` set from the future's done-callback) but gives up
    after `timeout_sec`: on timeout, `client.remove_pending_request(future)`
    is called (the same cleanup `call_async`'s done-callback would have done,
    done early so a late response can't fire callbacks on a future nothing
    is waiting on any more) and `TimeoutError` is raised. On completion
    before the deadline, the future's exception is re-raised if it carries
    one, else its result is returned — matching `Client.call`'s contract.
    """
    event = threading.Event()

    def _unblock(_future):
        event.set()

    future = client.call_async(request)
    future.add_done_callback(_unblock)

    if not event.wait(timeout_sec):
        client.remove_pending_request(future)
        raise TimeoutError(
            f"service call to {client.srv_name!r} timed out after "
            f"{timeout_sec} s with no response")

    exc = future.exception()
    if exc is not None:
        raise exc
    return future.result()


class ClientReaper:
    """Destroys leaked rosbridge service clients on the executor thread.

    `rclpy.node.Node.destroy_client` mutates the node's internal client list,
    which the executor iterates while building each wait set — calling it from
    a `ServiceCaller` worker thread (a plain `threading.Thread`, see stock
    `rosbridge_library/internal/services.py::ServiceCaller`) would race that
    iteration. A guard condition is the standard rclpy way to hand work to the
    executor thread: triggering it wakes `spin_once()` and runs `_drain` there.
    Upstream schedules the destroy the same way.
    """

    def __init__(self, node):
        self._node = node
        self._pending = collections.deque()
        self._guard = node.create_guard_condition(self._drain)

    def schedule(self, client):
        """Queue `client` for destruction. Safe to call from any thread —
        this is what `build_call_service`'s wrapper calls from a
        `ServiceCaller` worker thread after each service call completes."""
        self._pending.append(client)
        self._guard.trigger()

    def _drain(self):
        """Guard-condition callback — runs on the executor thread. Pops and
        destroys every pending client, leaving the queue empty."""
        while self._pending:
            client = self._pending.popleft()
            self._node.destroy_client(client)


class ProtocolReaper:
    """Runs scheduled zero-arg callables on the executor thread. Mirrors
    `ClientReaper` above, generalised from "destroy this client" to "run
    this callable" — used to run a connection's `Protocol.finish()` (module
    docstring item 4) off whichever thread closed the connection.

    `Protocol.finish()` calls each capability's `finish()`; `Subscribe
    .finish()` → `Subscription.unregister()` → `node.destroy_subscription()`
    and `Advertise.finish()` → `node.destroy_publisher()` mutate the same
    node-internal entity lists `SingleThreadedExecutor.spin_once()` iterates
    on `spin_thread` while building each wait set — the exact hazard
    `ClientReaper` exists for (`node.destroy_client` above), just for
    subscriptions/publishers instead of clients. Calling `protocol.finish()`
    inline from tornado's I/O thread (where `on_close` runs) or from a
    connection's `IncomingQueue` thread would race that iteration; routing
    it through a guard condition, as here, hands the work to the executor
    thread instead.
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
        runs every pending callable, leaving the queue empty."""
        while self._pending:
            fn = self._pending.popleft()
            fn()


def build_call_service(svc, schedule_destroy, timeout_sec=CALL_SERVICE_TIMEOUT_S):
    """Return a drop-in replacement for `rosbridge_library.internal.services
    .call_service(node_handle, service, args=None)`.

    The body below is the 1.3.1 function verbatim (every helper referenced
    through `svc`, the real `rosbridge_library.internal.services` module, so
    this stays byte-for-byte faithful to upstream's request-building and error
    handling), with two changes: the client is destroyed in a `finally` after
    the call, via `schedule_destroy` (a `ClientReaper.schedule` bound method
    in production); and `client.call(inst)` is replaced with
    `_call_with_timeout(client, inst, timeout_sec)` so the call can never
    block the connection's IncomingQueue thread forever (module docstring
    item 3) — a timeout raises `TimeoutError`, which `finally` still schedules
    the client for destroy and which stock `CallService._failure` turns into
    a `service_response` with `result: False` for the browser, same as any
    other service-call exception. Exceptions raised before `create_client` —
    an unknown service (`svc.InvalidServiceException`) or bad args
    (`svc.args_to_service_request_instance`) — never reach the `finally`, so
    nothing is scheduled for a client that was never created.
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

        client = node_handle.create_client(service_class, service)
        try:
            result = _call_with_timeout(client, inst, timeout_sec)
            if result is not None:
                # Turn the response into JSON and pass to the callback
                json_response = svc.extract_values(result)
            else:
                raise Exception(result)

            return json_response
        finally:
            # Unlike stock 1.3.1: the client is never freed there, and rclpy
            # keeps it in node.__clients for the life of the process (see this
            # module's docstring). This is the fix.
            schedule_destroy(client)

    return call_service


def _guard_protocol_finish(protocol, schedule):
    """Wrap `protocol.finish` (a `rosbridge_library.protocol.Protocol`
    instance) so repeat calls SCHEDULE the real teardown, via `schedule` (a
    `ProtocolReaper.schedule` bound method in production), onto the executor
    thread at most once, thread-safely — never run it inline on whichever
    thread called `finish()`.

    `Protocol.finish()` calls `capability.finish()` for every registered
    capability, and `Subscribe.finish()` / `Advertise.finish()` reach
    `node.destroy_subscription()` / `node.destroy_publisher()` — the same
    entity-list-mutation-off-the-executor-thread hazard `ClientReaper`
    exists for (see `ProtocolReaper`'s docstring above), so `finish()` must
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
    see `_call_with_timeout` above for why that thread can no longer block
    forever, but it can still take up to `timeout_sec`):

    - Wraps `open()` so, right after stock `open()` creates
      `self.protocol`, `self.protocol.finish` is replaced with the
      `_guard_protocol_finish` wrapper bound to `reaper.schedule` — so BOTH
      this patch's `on_close` and stock `IncomingQueue.run()`'s trailing
      `self.protocol.finish()` call go through the same one-shot guard,
      regardless of which thread gets there first, and neither ever runs
      the real teardown inline: it's always scheduled onto the executor
      thread via `reaper` (a `ProtocolReaper`), the same way `ClientReaper`
      schedules a leaked client's destroy.
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

    # The leak fix: patch call_service (a module global ServiceCaller.run
    # looks up at call time, so this takes effect for every future call)
    # to destroy its client afterwards, on the executor thread. Also bounds
    # the call (module docstring item 3) so it can never block a
    # connection's IncomingQueue thread forever.
    reaper = ClientReaper(node)
    svc.call_service = build_call_service(
        svc, reaper.schedule, CALL_SERVICE_TIMEOUT_S)

    # The hung-close fix (module docstring item 4): tear a connection's
    # subscriptions down immediately on close rather than waiting for its
    # (possibly still-blocked) IncomingQueue thread to reach it. The real
    # destroy_subscription()/destroy_publisher() calls that teardown makes
    # must run on the executor thread (same entity-list hazard ClientReaper
    # exists for above), so a second reaper schedules them there instead of
    # running inline off tornado's I/O thread. Patching the class here,
    # before start_hook() starts the IOLoop that accepts connections, means
    # every connection for the life of this process goes through the
    # patched open()/on_close().
    protocol_reaper = ProtocolReaper(node)
    patch_rosbridge_websocket(
        RosbridgeWebSocket, Subscribe, node.get_logger(), protocol_reaper)

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
