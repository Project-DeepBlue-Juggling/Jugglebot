"""test_rosbridge_websocket_lean.py — unit tests for
jugglebot/rosbridge_websocket_lean.py (the CPU-leak + spin-thread fix for the
GUI's rosbridge, logbook/2026-09-14-rosbridge-cpu-leak-and-spin.md), plus
string-level tripwires pinning its launch/setup.py/GUI wiring.

Every ExecutorThreadRunner / ServiceClientPool / build_call_service /
spin_forever test below uses plain
fakes — no rclpy, no rosbridge_library, no real ROS2. That's deliberate: this
module is designed to import cleanly with mocked rclpy (see this repo's
tests/ros/conftest.py), and a test suite is the only thing that would catch a
regression on that contract. The launch/setup.py/GUI tests are honest
string-level tripwires in the style of test_launch_nodes.py and
test_gui_geometry.py — they cannot prove the process starts under ROS2
(no launch runtime here), only that the wiring text says what it should.
"""

from __future__ import annotations

import importlib
import queue
import re
import sys
import threading
import types
from pathlib import Path

import pytest

from jugglebot.rosbridge_websocket_lean import (
    run_service_calls_off_the_queue_thread,
    CALL_SERVICE_TIMEOUT_S,
    ExecutorThreadRunner,
    ServiceClientPool,
    _guard_protocol_finish,
    _subscription_count,
    build_call_service,
    patch_rosbridge_websocket,
    spin_forever,
)

ROOT = Path(__file__).resolve().parent.parent.parent
LAUNCH_PY = (ROOT / 'ros_ws' / 'src' / 'jugglebot' / 'launch'
             / 'jugglebot_launch.py')
SETUP_PY = ROOT / 'ros_ws' / 'src' / 'jugglebot' / 'setup.py'
GUI_JS_DIR = ROOT / 'ros_ws' / 'gui' / 'js'
ROS_BRIDGE_JS = GUI_JS_DIR / 'ros-bridge.js'
MAIN_JS = GUI_JS_DIR / 'main.js'


# ═══════════════════════════════════════════════════════════════════════
# ExecutorThreadRunner
# ═══════════════════════════════════════════════════════════════════════


class _FakeGuardCondition:
    """Stand-in for rclpy's guard condition: records trigger() calls and
    invokes the callback synchronously when the test asks it to."""

    def __init__(self, callback):
        self.callback = callback
        self.trigger_count = 0

    def trigger(self):
        self.trigger_count += 1


class _FakeNode:
    """Stand-in for the rclpy Node ExecutorThreadRunner is built on."""

    def __init__(self):
        self.guard = None

    def create_guard_condition(self, callback):
        self.guard = _FakeGuardCondition(callback)
        return self.guard


def test_runner_schedule_appends_and_triggers_the_guard():
    node = _FakeNode()
    runner = ExecutorThreadRunner(node)
    calls = []

    runner.schedule(lambda: calls.append('a'))

    assert len(runner._pending) == 1
    assert node.guard.trigger_count == 1
    assert calls == []  # nothing run until _drain runs


def test_runner_drain_runs_each_pending_callable_exactly_once():
    node = _FakeNode()
    runner = ExecutorThreadRunner(node)
    calls = []
    runner.schedule(lambda: calls.append('a'))
    runner.schedule(lambda: calls.append('b'))

    runner._drain()

    assert calls == ['a', 'b']
    assert len(runner._pending) == 0

    # A second drain with nothing pending runs nothing more.
    runner._drain()
    assert calls == ['a', 'b']


def test_runner_concurrent_schedule_from_several_threads_then_one_drain_runs_all():
    node = _FakeNode()
    runner = ExecutorThreadRunner(node)
    calls = []
    threads = [threading.Thread(target=runner.schedule,
                                args=(lambda i=i: calls.append(i),))
               for i in range(20)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert len(runner._pending) == 20

    runner._drain()

    assert sorted(calls) == list(range(20))
    assert len(runner._pending) == 0


def test_runner_drain_runs_the_rest_when_one_raises_then_reraises_the_first():
    """A raising teardown must not strand a service call's send queued behind
    it until the next trigger; the error still reaches spin_forever's log."""
    node = _FakeNode()
    runner = ExecutorThreadRunner(node)
    calls = []

    def boom():
        raise RuntimeError('boom')

    runner.schedule(lambda: calls.append('a'))
    runner.schedule(boom)
    runner.schedule(lambda: calls.append('c'))

    with pytest.raises(RuntimeError, match='boom'):
        runner._drain()

    assert calls == ['a', 'c']
    assert len(runner._pending) == 0


# ═══════════════════════════════════════════════════════════════════════
# ServiceClientPool — module docstring item 6
# ═══════════════════════════════════════════════════════════════════════


class _FakeFuture:
    """Stand-in for rclpy.task.Future: add_done_callback (firing immediately
    if already done, matching rclpy), exception() and result()."""

    def __init__(self):
        self._done_callbacks = []
        self._done = False
        self._result = None
        self._exception = None

    def add_done_callback(self, cb):
        self._done_callbacks.append(cb)
        if self._done:
            cb(self)

    def _complete(self, result=None, exception=None):
        self._done = True
        self._result = result
        self._exception = exception
        for cb in self._done_callbacks:
            cb(self)

    def exception(self):
        return self._exception

    def result(self):
        return self._result


class _ExecutorThread:
    """A real thread running scheduled callables FIFO — the stand-in for
    ExecutorThreadRunner + spin_forever, so the tests can check which thread
    each rclpy call ran on."""

    def __init__(self):
        self._queue = queue.Queue()
        self.thread = threading.Thread(target=self._run, daemon=True,
                                       name='fake-executor')
        self.thread.start()

    def schedule(self, fn):
        self._queue.put(fn)

    def _run(self):
        while True:
            fn = self._queue.get()
            if fn is None:
                return
            fn()

    def flush(self):
        """Block until everything scheduled so far has run."""
        done = threading.Event()
        self.schedule(done.set)
        assert done.wait(2.0)

    def stop(self):
        self._queue.put(None)
        self.thread.join(2.0)


class _PoolClient:
    """Stand-in for rclpy.client.Client. Records the thread of every call.
    Its reply, like rclpy's, is delivered by the executor thread."""

    def __init__(self, executor, srv_name, reply='raw-reply', error=None,
                 not_ready_checks=0, never_replies=False):
        self._executor = executor
        self.srv_name = srv_name
        self._reply = reply
        self._error = error
        self._not_ready_checks = not_ready_checks
        self._never_replies = never_replies
        self.ready_checks = 0
        self.sent = []
        self.removed_pending = []
        self.calls = []  # (method, thread)

    def service_is_ready(self):
        self.calls.append(('service_is_ready', threading.current_thread()))
        self.ready_checks += 1
        return self.ready_checks > self._not_ready_checks

    def call_async(self, request):
        self.calls.append(('call_async', threading.current_thread()))
        self.sent.append(request)
        future = _FakeFuture()
        if not self._never_replies:
            self._executor.schedule(
                lambda: future._complete(result=self._reply,
                                         exception=self._error))
        return future

    def remove_pending_request(self, future):
        self.calls.append(('remove_pending_request',
                           threading.current_thread()))
        self.removed_pending.append(future)


class _PoolNode:
    """Stand-in for the rosbridge node: create_client builds a _PoolClient
    with per-service behaviour from `behaviour`."""

    def __init__(self, executor, behaviour=None, create_error=None):
        self._executor = executor
        self._behaviour = behaviour or {}
        self._create_error = create_error
        self.created = []  # (service, thread)
        self.clients = []

    def create_client(self, service_class, service):
        self.created.append((service, threading.current_thread()))
        if self._create_error is not None:
            raise self._create_error
        client = _PoolClient(self._executor, service,
                             **self._behaviour.get(service, {}))
        self.clients.append(client)
        return client


class _InfoLog:
    def __init__(self):
        self.infos = []

    def info(self, msg):
        self.infos.append(msg)


@pytest.fixture
def executor():
    ex = _ExecutorThread()
    yield ex
    ex.stop()


def test_pool_creates_and_sends_only_on_the_executor_thread(executor):
    """Item 6's contract: create_client, service_is_ready and call_async all
    run on the executor thread, never the caller's. Fail-before: until
    2026-10-02 build_call_service ran create_client + call_async on the
    ServiceCaller worker thread, which raced the executor's wait-set build
    (AssertionError) and its reply take (a reply dropped as 'cancelled')."""
    node = _PoolNode(executor)
    pool = ServiceClientPool(node, executor.schedule)

    assert pool.call('/svc', 'pkg/srv/T', 'cls', 'req', 2.0) == 'raw-reply'

    assert [t for _, t in node.created] == [executor.thread]
    client = node.clients[0]
    assert {name for name, _ in client.calls} == {'service_is_ready',
                                                  'call_async'}
    assert all(t is executor.thread for _, t in client.calls)
    assert client.sent == ['req']


def test_pool_keeps_one_client_per_service_for_every_later_call(executor):
    node = _PoolNode(executor)
    log = _InfoLog()
    pool = ServiceClientPool(node, executor.schedule, log)

    for i in range(5):
        pool.call('/rosapi/topics', 'rosapi/srv/Topics', 'cls', f'r{i}', 2.0)
    pool.call('/bb/aim', 'pkg/srv/Aim', 'cls', 'aim', 2.0)
    pool.call('/rosapi/topics', 'rosapi/srv/Topics', 'cls', 'r5', 2.0)

    assert [s for s, _ in node.created] == ['/rosapi/topics', '/bb/aim']
    topics, aim = node.clients
    assert topics.sent == ['r0', 'r1', 'r2', 'r3', 'r4', 'r5']
    assert aim.sent == ['aim']
    assert len(log.infos) == 2  # one line per client created, not per call


def test_pool_reraises_the_replys_exception(executor):
    node = _PoolNode(executor, {'/svc': {'error': RuntimeError('boom')}})
    pool = ServiceClientPool(node, executor.schedule)

    with pytest.raises(RuntimeError, match='boom'):
        pool.call('/svc', 'pkg/srv/T', 'cls', 'req', 2.0)


def test_pool_holds_the_send_until_the_server_is_visible_then_sends_once(executor):
    """A fresh client is not used until service_is_ready(): its request then
    goes to a server that can see it, rather than racing DDS matching."""
    node = _PoolNode(executor, {'/svc': {'not_ready_checks': 3}})
    pool = ServiceClientPool(node, executor.schedule)

    assert pool.call('/svc', 'pkg/srv/T', 'cls', 'req', 2.0) == 'raw-reply'

    client = node.clients[0]
    assert client.ready_checks == 4
    assert client.sent == ['req']


def test_pool_times_out_without_sending_when_the_server_never_appears(executor):
    node = _PoolNode(executor, {'/svc': {'not_ready_checks': 10 ** 9}})
    pool = ServiceClientPool(node, executor.schedule)

    with pytest.raises(TimeoutError, match='never became visible'):
        pool.call('/svc', 'pkg/srv/T', 'cls', 'req', 0.2)

    assert node.clients[0].sent == []


def test_pool_times_out_with_no_response_and_unregisters_on_the_executor_thread(executor):
    """A reply that never comes raises the same 'timed out after ... with no
    response' text the bags were grepped for, and the pending request is
    removed on the executor thread — the thread that reads
    _pending_requests."""
    node = _PoolNode(executor, {'/svc': {'never_replies': True}})
    pool = ServiceClientPool(node, executor.schedule)

    with pytest.raises(TimeoutError, match='timed out after 0.2 s with no response'):
        pool.call('/svc', 'pkg/srv/T', 'cls', 'req', 0.2)

    executor.flush()
    client = node.clients[0]
    assert len(client.removed_pending) == 1
    assert [t for name, t in client.calls
            if name == 'remove_pending_request'] == [executor.thread]


def test_pool_never_sends_a_request_its_caller_abandoned():
    """If the executor thread does not get to the send before the deadline,
    the caller gives up and the send, run later, does nothing: no client is
    created and no request goes out for a call that already reported
    failure."""
    held = []
    node = _PoolNode(executor=None)
    pool = ServiceClientPool(node, held.append)

    with pytest.raises(TimeoutError, match='never ran the send'):
        pool.call('/svc', 'pkg/srv/T', 'cls', 'req', 0.05)

    for task in held:  # the executor thread finally gets there
        task()
    assert node.created == []


def test_pool_hands_a_send_error_to_the_caller_and_keeps_serving(executor):
    """An exception in the send is raised on the caller's thread, never on
    the executor thread, which goes on serving later calls."""
    node = _PoolNode(executor, create_error=RuntimeError('no client'))
    pool = ServiceClientPool(node, executor.schedule)

    with pytest.raises(RuntimeError, match='no client'):
        pool.call('/svc', 'pkg/srv/T', 'cls', 'req', 2.0)

    node._create_error = None
    assert pool.call('/svc', 'pkg/srv/T', 'cls', 'req', 2.0) == 'raw-reply'


def test_call_service_timeout_exceeds_the_guis_largest_client_timeout():
    """The GUI's largest per-step client-side service timeout is
    T_SVC_CLEAR = 45000 ms (ros_ws/gui/js/state-minimap.js) — the server
    bound must sit above every GUI client timeout, or the server could give
    up mid-call while the GUI is still legitimately waiting."""
    t_svc_clear_ms = 45000
    assert CALL_SERVICE_TIMEOUT_S * 1000 > t_svc_clear_ms


# ═══════════════════════════════════════════════════════════════════════
# build_call_service
# ═══════════════════════════════════════════════════════════════════════


class _InvalidServiceException(Exception):
    def __init__(self, servicename):
        Exception.__init__(self, f'Service {servicename} does not exist')


def _fake_svc():
    """A fake stand-in for rosbridge_library.internal.services, carrying
    exactly the six attributes build_call_service's body references."""
    return types.SimpleNamespace(
        expand_topic_name=lambda service, name, ns: '/' + service,
        get_service_class=lambda t: f'class-for-{t}',
        get_service_request_instance=lambda t: types.SimpleNamespace(type=t),
        args_to_service_request_instance=lambda service, inst, args: None,
        extract_values=lambda result: {'extracted': result},
        InvalidServiceException=_InvalidServiceException,
    )


class _FakeNodeHandle:
    def __init__(self, service_type=('some_msgs/srv/Trigger',)):
        self._service_type = service_type

    def get_name(self):
        return 'rosbridge_websocket'

    def get_namespace(self):
        return '/'

    def get_service_names_and_types(self):
        if self._service_type is None:
            return []
        return [('/my_service', self._service_type)]

    def get_logger(self):
        return types.SimpleNamespace(warning=lambda msg: None)


class _FakePool:
    def __init__(self, result=None, error=None):
        self._result = result
        self._error = error
        self.calls = []

    def call(self, service, service_type, service_class, request, timeout_sec):
        self.calls.append((service, service_type, service_class, request,
                           timeout_sec))
        if self._error is not None:
            raise self._error
        return self._result


def test_success_calls_the_pool_once_and_returns_extracted_values():
    pool = _FakePool(result='raw-result')
    call_service = build_call_service(_fake_svc(), pool, timeout_sec=7.0)

    response = call_service(_FakeNodeHandle(), 'my_service', args=None)

    assert response == {'extracted': 'raw-result'}
    [(service, service_type, service_class, request, timeout)] = pool.calls
    assert (service, service_type, service_class, timeout) == (
        '/my_service', 'some_msgs/srv/Trigger',
        'class-for-some_msgs/srv/Trigger', 7.0)
    assert request.type == 'some_msgs/srv/Trigger'


def test_a_none_result_raises():
    call_service = build_call_service(_fake_svc(), _FakePool(result=None))

    with pytest.raises(Exception):
        call_service(_FakeNodeHandle(), 'my_service', args=None)


def test_a_pool_timeout_propagates_for_stock_callservice_to_report():
    """Stock CallService._failure turns this into a service_response with
    result: False (see module docstring item 3)."""
    pool = _FakePool(error=TimeoutError('timed out after 0.05 s'))
    call_service = build_call_service(_fake_svc(), pool, timeout_sec=0.05)

    with pytest.raises(TimeoutError, match='timed out after'):
        call_service(_FakeNodeHandle(), 'my_service', args=None)


def test_unknown_service_raises_invalid_service_and_never_reaches_the_pool():
    svc = _fake_svc()
    pool = _FakePool(result='unused')
    call_service = build_call_service(svc, pool)

    with pytest.raises(svc.InvalidServiceException):
        call_service(_FakeNodeHandle(service_type=None), 'my_service', args=None)

    assert pool.calls == []


# ═══════════════════════════════════════════════════════════════════════
# _guard_protocol_finish / patch_rosbridge_websocket
# ═══════════════════════════════════════════════════════════════════════


class _FakeSubscribeCapability:
    """Stand-in for rosbridge_library.capabilities.subscribe.Subscribe —
    just the _subscriptions container and an idempotent finish()."""

    def __init__(self, subscriptions):
        self._subscriptions = dict(subscriptions)

    def finish(self):
        self._subscriptions.clear()


class _OtherCapability:
    def finish(self):
        pass


class _FakeProtocol:
    """Stand-in for rosbridge_library.protocol.Protocol — enough surface
    for _guard_protocol_finish / _subscription_count / patch_rosbridge_websocket."""

    def __init__(self, capabilities):
        self.capabilities = capabilities
        self.finish_calls = 0

    def finish(self):
        self.finish_calls += 1
        for capability in self.capabilities:
            capability.finish()


class _FakeCloseLog:
    def __init__(self):
        self.infos = []

    def info(self, msg):
        self.infos.append(msg)


class _FakeReaper:
    """Stand-in for ExecutorThreadRunner: records scheduled callables and only
    runs them when drain() is called explicitly — mirroring how the real
    reaper only runs them once the executor thread's guard condition
    fires, never inline on the scheduling thread."""

    def __init__(self):
        self.pending = []

    def schedule(self, fn):
        self.pending.append(fn)

    def drain(self):
        pending, self.pending = self.pending, []
        for fn in pending:
            fn()


def test_guard_protocol_finish_schedules_the_real_finish_exactly_once():
    """_guard_protocol_finish must never run the real finish() inline — see
    the coordinator's audit finding 2026-09-15: Subscribe.finish() /
    Advertise.finish() mutate node entity lists the executor thread
    iterates, so the real finish() may only run once scheduled onto it."""
    protocol = _FakeProtocol([])
    reaper = _FakeReaper()
    finish_once = _guard_protocol_finish(protocol, reaper.schedule)

    assert finish_once() is True
    assert protocol.finish_calls == 0  # scheduled, not run inline
    assert len(reaper.pending) == 1

    assert finish_once() is False  # second call: nothing more scheduled
    assert len(reaper.pending) == 1

    reaper.drain()
    assert protocol.finish_calls == 1

    reaper.drain()  # nothing pending — draining again changes nothing
    assert protocol.finish_calls == 1


def test_guard_protocol_finish_is_safe_under_concurrent_calls():
    protocol = _FakeProtocol([])
    reaper = _FakeReaper()
    finish_once = _guard_protocol_finish(protocol, reaper.schedule)
    results = []
    results_lock = threading.Lock()

    def call():
        ran = finish_once()
        with results_lock:
            results.append(ran)

    threads = [threading.Thread(target=call) for _ in range(20)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert len(reaper.pending) == 1  # exactly one call won the schedule
    assert results.count(True) == 1
    assert results.count(False) == 19

    reaper.drain()
    assert protocol.finish_calls == 1  # the real finish ran exactly once


def test_subscription_count_reads_the_subscribe_capabilitys_container():
    sub_cap = _FakeSubscribeCapability({'t1': object(), 't2': object()})
    protocol = _FakeProtocol([_OtherCapability(), sub_cap])

    assert _subscription_count(protocol, _FakeSubscribeCapability) == 2


def test_subscription_count_returns_zero_with_no_matching_capability():
    protocol = _FakeProtocol([_OtherCapability()])

    assert _subscription_count(protocol, _FakeSubscribeCapability) == 0


class _FakeRosbridgeWebSocket:
    """Stand-in for rosbridge_server.RosbridgeWebSocket (a tornado
    WebSocketHandler) — enough surface for patch_rosbridge_websocket:
    open()/on_close() that patch wraps, and a `protocol` attribute open()
    assigns (as stock open() does)."""

    def __init__(self, protocol_to_assign):
        self._protocol_to_assign = protocol_to_assign
        self.stock_open_calls = 0
        self.stock_on_close_calls = 0
        self.client_id = 'fake-client-1'

    def open(self):
        self.stock_open_calls += 1
        self.protocol = self._protocol_to_assign

    def on_close(self):
        self.stock_on_close_calls += 1


def test_patch_schedules_teardown_via_the_reaper_without_calling_finish_inline():
    """(a) on_close must SCHEDULE exactly one finish via the reaper and must
    NOT call protocol.finish inline — the fake protocol's finish is not
    invoked until the reaper drains (coordinator audit finding 2026-09-15:
    Subscribe.finish()/Advertise.finish() mutate node entity lists the
    executor thread iterates — the hazard module docstring item 6 closes for
    service clients)."""
    sub_cap = _FakeSubscribeCapability({'t1': object(), 't2': object()})
    protocol = _FakeProtocol([sub_cap, _OtherCapability()])
    log = _FakeCloseLog()
    reaper = _FakeReaper()

    # A fresh subclass per test: patch_rosbridge_websocket mutates the class
    # it's given, and each test must patch an UNPATCHED class — patching the
    # shared _FakeRosbridgeWebSocket base repeatedly would stack wrappers
    # from earlier tests (each closing over that earlier test's `log`).
    class Handler(_FakeRosbridgeWebSocket):
        pass

    patch_rosbridge_websocket(Handler, _FakeSubscribeCapability, log, reaper)
    handler = Handler(protocol)
    handler.open()
    assert handler.stock_open_calls == 1

    handler.on_close()

    assert handler.stock_on_close_calls == 1
    assert protocol.finish_calls == 0  # NOT run inline off the I/O thread
    assert sub_cap._subscriptions != {}  # not torn down yet
    assert len(reaper.pending) == 1  # exactly one finish scheduled
    assert len(log.infos) == 1
    assert '2 subscription' in log.infos[0]

    reaper.drain()  # stand-in for the executor thread draining the reaper

    assert protocol.finish_calls == 1
    assert sub_cap._subscriptions == {}


def test_patch_on_close_then_incoming_queues_trailing_finish_is_a_no_op():
    """Reproduces the real path this patch relies on: stock
    IncomingQueue.run()'s trailing self.protocol.finish() call (once its
    thread's blocked call eventually returns) goes through the SAME
    guarded, reaper-backed protocol.finish attribute on_close used — so it
    schedules nothing new, and draining still runs the real teardown
    exactly once."""
    sub_cap = _FakeSubscribeCapability({'t1': object()})
    protocol = _FakeProtocol([sub_cap])
    log = _FakeCloseLog()
    reaper = _FakeReaper()

    class Handler(_FakeRosbridgeWebSocket):
        pass

    patch_rosbridge_websocket(Handler, _FakeSubscribeCapability, log, reaper)
    handler = Handler(protocol)
    handler.open()

    handler.on_close()
    ran_again = protocol.finish()  # stand-in for IncomingQueue.run()'s call

    assert ran_again is False
    assert len(reaper.pending) == 1  # still just the one scheduled call
    assert len(log.infos) == 1  # only the close that won the schedule logs

    reaper.drain()
    assert protocol.finish_calls == 1  # real teardown ran exactly once


def test_patch_on_close_called_twice_schedules_once_and_drains_once():
    """(b) draining runs the real finish once even when scheduled twice."""
    sub_cap = _FakeSubscribeCapability({'t1': object()})
    protocol = _FakeProtocol([sub_cap])
    log = _FakeCloseLog()
    reaper = _FakeReaper()

    class Handler(_FakeRosbridgeWebSocket):
        pass

    patch_rosbridge_websocket(Handler, _FakeSubscribeCapability, log, reaper)
    handler = Handler(protocol)
    handler.open()

    handler.on_close()
    handler.on_close()

    assert handler.stock_on_close_calls == 2  # stock bookkeeping still runs
    assert len(reaper.pending) == 1  # only one schedule won
    assert len(log.infos) == 1

    reaper.drain()
    reaper.drain()  # a second drain (nothing pending) changes nothing

    assert protocol.finish_calls == 1  # real teardown ran exactly once


def test_patch_open_leaves_protocol_none_handling_to_on_close():
    """If stock open() failed to set self.protocol (the real stock open()
    catches exceptions during protocol construction and just logs), the
    patched open()/on_close() must not raise."""
    log = _FakeCloseLog()
    reaper = _FakeReaper()

    class _HandlerWithNoProtocol(_FakeRosbridgeWebSocket):
        def open(self):
            self.stock_open_calls += 1
            # deliberately does not set self.protocol

    patch_rosbridge_websocket(
        _HandlerWithNoProtocol, _FakeSubscribeCapability, log, reaper)
    handler = _HandlerWithNoProtocol(protocol_to_assign=None)

    handler.open()
    handler.on_close()  # must not raise

    assert log.infos == []
    assert reaper.pending == []


# ═══════════════════════════════════════════════════════════════════════
# spin_forever
# ═══════════════════════════════════════════════════════════════════════


class _FakeExecutor:
    """spin_once() raises the next queued effect (or does nothing)."""

    def __init__(self, effects):
        self._effects = list(effects)
        self.calls = 0

    def spin_once(self):
        self.calls += 1
        effect = self._effects.pop(0)
        if effect is not None:
            raise effect


class _FakeLog:
    def __init__(self):
        self.errors = []

    def error(self, msg):
        self.errors.append(msg)


class _FakeShutdown(Exception):
    pass


def test_spin_forever_logs_a_generic_exception_and_continues():
    executor = _FakeExecutor([RuntimeError('boom'), None])
    log = _FakeLog()
    state = {'n': 0}

    def is_ok():
        state['n'] += 1
        return state['n'] <= 2  # true for the two spin_once calls, then stop

    spin_forever(executor, is_ok, log, shutdown_exceptions=(_FakeShutdown,))

    assert executor.calls == 2  # both queued effects were consumed: it kept going
    assert len(log.errors) == 1
    assert 'RuntimeError' in log.errors[0] and 'boom' in log.errors[0]


def test_spin_forever_ends_on_shutdown_exception_without_logging_an_error():
    executor = _FakeExecutor([_FakeShutdown()])
    log = _FakeLog()

    # is_ok() would keep saying yes forever — only the shutdown exception may
    # end the loop here, proving the break is exception-driven, not a
    # coincidence of is_ok() flipping.
    spin_forever(executor, lambda: True, log,
                 shutdown_exceptions=(_FakeShutdown,))

    assert executor.calls == 1
    assert log.errors == []


# ═══════════════════════════════════════════════════════════════════════
# Import hygiene
# ═══════════════════════════════════════════════════════════════════════


_MODULE_NAME = 'jugglebot.rosbridge_websocket_lean'
_FORBIDDEN_PREFIXES = ('rosbridge_library', 'tornado', 'ament_index_python')


class _BlockingServiceCaller(threading.Thread):
    """Stands in for stock ``ServiceCaller``: a Thread whose ``run`` is the
    service call. This one never returns until released — an unanswered
    service (2026-09-28 21:41: ``ball_butler_node`` answered nothing)."""

    instances = []

    def __init__(self, service, args, success, error, node_handle):
        threading.Thread.__init__(self)
        self.service = service
        self.release = threading.Event()
        self.entered = threading.Event()
        self.ran_on = None
        _BlockingServiceCaller.instances.append(self)

    def run(self):
        self.ran_on = threading.current_thread()
        self.entered.set()
        self.release.wait(5.0)


def test_a_hung_service_call_does_not_block_the_calling_queue_thread():
    """Item 5: ``CallService.call_service`` calls ``ServiceCaller(...).run()``;
    after the patch that returns at once and the call runs on its own daemon
    thread, so the tab's IncomingQueue thread is free for the next message.
    Fail-before (stock ``.run()`` in the same thread): this ``run()`` would not
    return until the service answered."""
    _BlockingServiceCaller.instances = []
    cap = types.SimpleNamespace(ServiceCaller=_BlockingServiceCaller)
    run_service_calls_off_the_queue_thread(cap)
    assert cap.ServiceCaller is not _BlockingServiceCaller

    caller = cap.ServiceCaller('bb/aim', [], None, None, None)
    done = threading.Event()
    t = threading.Thread(target=lambda: (caller.run(), done.set()))
    t.start()
    assert done.wait(1.0), 'run() blocked the calling (queue) thread'
    inner = _BlockingServiceCaller.instances[0]
    assert inner.entered.wait(1.0)
    assert inner.ran_on is not t and inner.daemon is True
    inner.release.set()
    inner.join(1.0)


def test_two_calls_from_one_tab_run_concurrently():
    """A second call is not queued behind a first one that never answers."""
    _BlockingServiceCaller.instances = []
    cap = types.SimpleNamespace(ServiceCaller=_BlockingServiceCaller)
    run_service_calls_off_the_queue_thread(cap)
    cap.ServiceCaller('bb/aim', [], None, None, None).run()
    cap.ServiceCaller('rosapi/topics', [], None, None, None).run()
    a, b = _BlockingServiceCaller.instances
    assert a.entered.wait(1.0) and b.entered.wait(1.0)
    for c in (a, b):
        c.release.set()
        c.join(1.0)


def test_import_does_not_pull_in_rosbridge_tornado_or_ament():
    """Every real-ROS import in the module lives inside main() / the
    functions it calls, so importing the module itself must not add any of
    rosbridge_library, tornado or ament_index_python to sys.modules."""
    for name in list(sys.modules):
        if name == _MODULE_NAME or name.startswith(_MODULE_NAME + '.'):
            del sys.modules[name]  # force a genuine re-import, not a cache hit

    before = {n for n in sys.modules if n.startswith(_FORBIDDEN_PREFIXES)}
    importlib.import_module(_MODULE_NAME)
    after = {n for n in sys.modules if n.startswith(_FORBIDDEN_PREFIXES)}

    assert after == before, (
        f'importing {_MODULE_NAME} pulled in {sorted(after - before)} — the '
        f'module must import cleanly with no ROS or rosbridge on the path.')


# ═══════════════════════════════════════════════════════════════════════
# Launch pin: jugglebot_launch.py's rosbridge Node + setup.py's entry point
# ═══════════════════════════════════════════════════════════════════════


@pytest.fixture(scope='module')
def launch_src():
    return LAUNCH_PY.read_text()


@pytest.fixture(scope='module')
def setup_src():
    return SETUP_PY.read_text()


def _rosbridge_node_block(launch_src):
    """The source text of the ONE Node(...) block whose executable is
    rosbridge_websocket_lean (same cut-then-filter idiom as
    test_launch_nodes.py's _node_block, for the same reason: cutting by
    executable name first risks a non-greedy span swallowing half the file)."""
    blocks = re.findall(r'^    \w+ = Node\($\n(.*?)^    \)$',
                        launch_src, re.S | re.M)
    hits = [b for b in blocks if "executable='rosbridge_websocket_lean'" in b]
    assert len(hits) == 1, (
        f'Expected exactly one Node(...) block for rosbridge_websocket_lean, '
        f'found {len(hits)} (of {len(blocks)} blocks parsed).')
    return hits[0]


def test_rosbridge_node_launches_the_lean_package_and_executable(launch_src):
    block = _rosbridge_node_block(launch_src)
    assert "package='jugglebot'" in block
    assert "executable='rosbridge_websocket_lean'" in block


def test_rosbridge_node_keeps_its_name_and_ping_parameters(launch_src):
    block = _rosbridge_node_block(launch_src)
    assert "name='rosbridge_websocket'" in block
    assert "'websocket_ping_interval': 10," in block
    assert "'websocket_ping_timeout': 30," in block


def test_setup_py_declares_the_lean_console_entry_point(setup_src):
    assert ("'rosbridge_websocket_lean = "
            "jugglebot.rosbridge_websocket_lean:main'") in setup_src


# ═══════════════════════════════════════════════════════════════════════
# GUI pins: subscribeSpy's raw compression + the GUI_SUBSCRIBED_TOPICS
# ═══════════════════════════════════════════════════════════════════════


@pytest.fixture(scope='module')
def ros_bridge_js():
    return ROS_BRIDGE_JS.read_text()


@pytest.fixture(scope='module')
def main_js():
    return MAIN_JS.read_text()


def _extract_function_body(js_text, func_name):
    m = re.search(rf'function {func_name}\([^)]*\)\s*\{{(.*?)\n\}}',
                  js_text, re.S)
    assert m, f'Could not find function {func_name} in JS source'
    return m.group(1)


def test_subscribe_spy_requests_raw_cbor_compression(ros_bridge_js):
    body = _extract_function_body(ros_bridge_js, 'subscribeSpy')
    assert "compression: 'cbor-raw'" in body, (
        "subscribeSpy no longer requests raw CBOR — rosbridge will decode "
        "every spied message again, including the two 100 Hz topics.")


def test_discover_topics_keeps_one_bounded_poll_in_flight(ros_bridge_js):
    """main.js polls every 3 s; without the single-flight guard a silent
    rosapi collects ~16 concurrent 50 s calls per tab on rosbridge, and
    without the local bound a lost reply holds the slot for 50 s."""
    body = _extract_function_body(ros_bridge_js, 'discoverTopics')
    assert 'if (discoveryInFlight) return;' in body
    assert 'discoveryInFlight = true;' in body
    assert 'withTimeout(' in body and 'T_DISCOVERY_MS' in body
    assert '.finally(() => { discoveryInFlight = false; })' in body


def _gui_subscribed_topics(main_js_text):
    m = re.search(r'const GUI_SUBSCRIBED_TOPICS = new Set\(\[(.*?)\]\)',
                  main_js_text, re.S)
    assert m, 'Could not find GUI_SUBSCRIBED_TOPICS in main.js'
    topics = set(re.findall(r"'([^']+)'", m.group(1)))
    assert topics, 'GUI_SUBSCRIBED_TOPICS extraction went stale (empty set)'
    return topics


def _real_subscribe_topic_literals():
    """Every topic-name literal passed to ros.subscribe(...) across
    ros_ws/gui/js/*.js — the actual decoded-subscription set subscribeSpy
    must never be allowed to also target."""
    topics = set()
    for path in GUI_JS_DIR.glob('*.js'):
        topics |= set(re.findall(r"ros\.subscribe\('([^']+)'", path.read_text()))
    assert topics, 'ros.subscribe(...) extraction went stale (found nothing)'
    return topics


def test_gui_subscribed_topics_matches_the_real_subscribe_calls(main_js):
    """CONTRACT (also stated at both source sites): GUI_SUBSCRIBED_TOPICS
    must equal the set of topics actually passed to ros.subscribe(...).
    rosbridge fixes a subscription raw-vs-decoded at first creation and
    shares it across every subscriber of that topic — a topic present in one
    set but not the other means a spy (raw) and a real subscription (decoded)
    could race to create the same rosbridge-side subscription, corrupting
    whichever one loses."""
    declared = _gui_subscribed_topics(main_js)
    actual = _real_subscribe_topic_literals()
    assert declared == actual, (
        'GUI_SUBSCRIBED_TOPICS (main.js) has drifted from the real '
        'ros.subscribe(...) calls across ros_ws/gui/js/*.js.\n'
        f'declared but never actually subscribed: {sorted(declared - actual)}\n'
        f'subscribed but missing from the declared set: {sorted(actual - declared)}')
