"""test_rosbridge_websocket_lean.py — unit tests for
jugglebot/rosbridge_websocket_lean.py (the CPU-leak + spin-thread fix for the
GUI's rosbridge, logbook/2026-09-14-rosbridge-cpu-leak-and-spin.md), plus
string-level tripwires pinning its launch/setup.py/GUI wiring.

Every ClientReaper / build_call_service / spin_forever test below uses plain
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
import re
import sys
import threading
import types
from pathlib import Path

import pytest

from jugglebot.rosbridge_websocket_lean import (
    ClientReaper,
    build_call_service,
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
# ClientReaper
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
    """Stand-in for the rclpy Node ClientReaper is built on."""

    def __init__(self):
        self.destroyed = []
        self.guard = None

    def create_guard_condition(self, callback):
        self.guard = _FakeGuardCondition(callback)
        return self.guard

    def destroy_client(self, client):
        self.destroyed.append(client)


def test_schedule_appends_and_triggers_the_guard():
    node = _FakeNode()
    reaper = ClientReaper(node)

    reaper.schedule('client-a')

    assert list(reaper._pending) == ['client-a']
    assert node.guard.trigger_count == 1
    assert node.destroyed == []  # nothing destroyed until _drain runs


def test_drain_destroys_each_pending_client_exactly_once_and_empties_queue():
    node = _FakeNode()
    reaper = ClientReaper(node)
    reaper.schedule('client-a')
    reaper.schedule('client-b')

    reaper._drain()

    assert node.destroyed == ['client-a', 'client-b']
    assert len(reaper._pending) == 0

    # A second drain with nothing pending destroys nothing more.
    reaper._drain()
    assert node.destroyed == ['client-a', 'client-b']


def test_concurrent_schedule_from_several_threads_then_one_drain_destroys_all():
    node = _FakeNode()
    reaper = ClientReaper(node)
    clients = [f'client-{i}' for i in range(20)]

    threads = [threading.Thread(target=reaper.schedule, args=(c,))
               for c in clients]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert len(reaper._pending) == 20

    reaper._drain()

    assert sorted(node.destroyed) == sorted(clients)
    assert len(reaper._pending) == 0


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
        expand_topic_name=lambda service, name, ns: service,
        get_service_class=lambda t: f'class-for-{t}',
        get_service_request_instance=lambda t: types.SimpleNamespace(type=t),
        args_to_service_request_instance=lambda service, inst, args: None,
        extract_values=lambda result: {'extracted': result},
        InvalidServiceException=_InvalidServiceException,
    )


class _FakeNodeHandle:
    def __init__(self, service_type=('some_msgs/srv/Trigger',), client=None):
        self._service_type = service_type
        self._client = client
        self.create_client_calls = []

    def get_name(self):
        return 'rosbridge_websocket'

    def get_namespace(self):
        return '/'

    def get_service_names_and_types(self):
        if self._service_type is None:
            return []
        return [('my_service', self._service_type)]

    def get_logger(self):
        return types.SimpleNamespace(warning=lambda msg: None)

    def create_client(self, service_class, service):
        self.create_client_calls.append((service_class, service))
        return self._client


class _FakeClient:
    def __init__(self, result=None, error=None):
        self._result = result
        self._error = error
        self.call_count = 0

    def call(self, inst):
        self.call_count += 1
        if self._error is not None:
            raise self._error
        return self._result


def test_success_returns_extracted_values_and_schedules_destroy_once():
    svc = _fake_svc()
    client = _FakeClient(result='raw-result')
    node_handle = _FakeNodeHandle(client=client)
    scheduled = []
    call_service = build_call_service(svc, scheduled.append)

    response = call_service(node_handle, 'my_service', args=None)

    assert response == {'extracted': 'raw-result'}
    assert scheduled == [client]
    assert client.call_count == 1


def test_client_call_returning_none_raises_and_still_schedules_destroy_once():
    svc = _fake_svc()
    client = _FakeClient(result=None)
    node_handle = _FakeNodeHandle(client=client)
    scheduled = []
    call_service = build_call_service(svc, scheduled.append)

    with pytest.raises(Exception):
        call_service(node_handle, 'my_service', args=None)

    assert scheduled == [client]


def test_client_call_raising_propagates_and_still_schedules_destroy_once():
    svc = _fake_svc()
    boom = RuntimeError('boom')
    client = _FakeClient(error=boom)
    node_handle = _FakeNodeHandle(client=client)
    scheduled = []
    call_service = build_call_service(svc, scheduled.append)

    with pytest.raises(RuntimeError, match='boom'):
        call_service(node_handle, 'my_service', args=None)

    assert scheduled == [client]


def test_unknown_service_raises_invalid_service_and_schedules_nothing():
    svc = _fake_svc()
    node_handle = _FakeNodeHandle(service_type=None)  # no services registered
    scheduled = []
    call_service = build_call_service(svc, scheduled.append)

    with pytest.raises(svc.InvalidServiceException):
        call_service(node_handle, 'my_service', args=None)

    assert node_handle.create_client_calls == []
    assert scheduled == []


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
