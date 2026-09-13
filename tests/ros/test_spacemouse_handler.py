"""Tests for jugglebot.spacemouse_handler — the mode-scoped idle-CPU lifecycle.

Until 2026-09-14 this node held a 100 Hz timer and pumped
``while rclpy.ok(): spin_once`` unconditionally, whether or not SPACEMOUSE
was the active control mode — measured at ~6% of a core with no device
connected (2026-09-13 pidstat baseline, see
``logbook/2026-09-14-spacemouse-handler-idle-cpu.md``). The fix makes the
timer and the open device mode-scoped: both exist only while SPACEMOUSE is
the active ``control_mode_topic`` mode. This file pins that lifecycle:

* no timer / no HID / no publish outside the mode (a);
* entering creates exactly one 0.01 s timer and streams real axes (b);
* device absent in mode: every tick publishes home, reconnect attempts are
  rate-limited to one per ``_RECONNECT_INTERVAL_S`` (c);
* a mid-mode ``read()`` failure closes the device, publishes home, and
  recovers on a later successful reopen (d);
* leaving the mode destroys the timer and closes the device, idempotently (e);
* repeated identical mode messages create no extra timers and trigger no
  reopen (f);
* ``main()`` drives the node with ``rclpy.spin``, not a ``spin_once`` poll
  loop (g) — mirrors ``test_skill_node.py``'s identical contract for
  ``skill_node.main()``.

``pyspacemouse`` is monkeypatched wholesale (a small fake with open/read/close
call counts — see ``jugglebot.spacemouse_handler``'s own module docstring
for why probing the real HID device is out of scope here), and
``time.monotonic`` is monkeypatched to a controllable fake clock everywhere
the 2 s reconnect cadence matters. The mock ``Node`` (``tests/ros/conftest.py``)
has no ``destroy_timer`` — real ``rclpy.Node`` does, and this node is the
first in the tree to need it — so each test node gets one bound as a plain
per-instance function (never a ``Mock``; ``test_ros_mock_hygiene.py`` is the
reason: a process-lifetime ``Mock`` retains every call forever, and while a
per-test node isn't process-lifetime, there's no reason to use one anyway).
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

import rclpy
from std_msgs.msg import String

import jugglebot.hardware_config as hw
import jugglebot.spacemouse_handler as sm


# ── Harness ──────────────────────────────────────────────────────────────────

class _FakeClock:
    """A controllable stand-in for ``time.monotonic`` — no real sleeping."""

    def __init__(self, t: float = 1000.0):
        self._t = t

    def __call__(self) -> float:
        return self._t

    def advance(self, dt: float) -> None:
        self._t += dt


class _FakePySpaceMouse:
    """Stand-in for the ``pyspacemouse`` module: open/read/close + counts.

    ``open_result`` / ``read_result`` drive the scenario: a bool for
    ``open()``'s return, and for ``read()`` either ``None`` (no new report),
    an axis namespace, or an ``Exception`` instance to raise (mirrors a real
    HID read failure on unplug).
    """

    def __init__(self):
        self.open_calls = 0
        self.read_calls = 0
        self.close_calls = 0
        self.open_result = True
        self.read_result = None

    def open(self, *a, **kw):
        self.open_calls += 1
        return self.open_result

    def read(self):
        self.read_calls += 1
        if isinstance(self.read_result, Exception):
            raise self.read_result
        return self.read_result

    def close(self):
        self.close_calls += 1


def _mode(name: str) -> String:
    return String(data=name)


def _axes(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0) -> SimpleNamespace:
    return SimpleNamespace(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)


@pytest.fixture
def clock(monkeypatch):
    fake = _FakeClock()
    monkeypatch.setattr(sm.time, 'monotonic', fake)
    return fake


@pytest.fixture
def fake_pm(monkeypatch):
    fake = _FakePySpaceMouse()
    monkeypatch.setattr(sm, 'pyspacemouse', fake)
    return fake


@pytest.fixture
def node(monkeypatch, fake_pm, clock):
    n = sm.SpaceMouseHandler()

    # Spy on create_timer/destroy_timer without touching the shared MockNode
    # class — plain per-instance functions, not Mocks (see module docstring).
    calls = {'create_timer': [], 'destroy_timer': []}
    real_create_timer = n.create_timer

    def _create_timer(period, callback):
        timer = real_create_timer(period, callback)
        calls['create_timer'].append((period, callback, timer))
        return timer

    def _destroy_timer(timer):
        calls['destroy_timer'].append(timer)
        return True

    monkeypatch.setattr(n, 'create_timer', _create_timer)
    # raising=False: MockNode has no destroy_timer to shadow.
    monkeypatch.setattr(n, 'destroy_timer', _destroy_timer, raising=False)
    n.calls = calls
    return n


# ── (a) no timer / no HID / no publish outside SPACEMOUSE mode ──────────────

def test_idle_outside_spacemouse_mode(node, fake_pm):
    assert node.timer is None
    assert fake_pm.open_calls == 0
    assert fake_pm.read_calls == 0
    assert node.publisher_.published == []

    for name in ('STANDBY', 'GUI', 'STANDBY'):
        node.control_mode_callback(_mode(name))

    assert node.timer is None
    assert node.calls['create_timer'] == []
    assert fake_pm.open_calls == 0
    assert fake_pm.read_calls == 0
    assert node.publisher_.published == []


# ── (b) entering creates one timer; first tick opens + streams axes ────────

def test_enter_spacemouse_creates_timer_and_streams(node, fake_pm):
    node.control_mode_callback(_mode('SPACEMOUSE'))

    assert node.timer is not None
    assert len(node.calls['create_timer']) == 1
    period, callback, _timer = node.calls['create_timer'][0]
    assert period == 0.01
    assert callback == node.publish_pose

    fake_pm.read_result = _axes(x=1.0, roll=2.0)
    node.publish_pose()  # opens (fresh mode entry) and falls through to read()

    assert fake_pm.open_calls == 1
    assert node._is_open is True
    msg = node.publisher_.published[-1]
    assert msg.publisher == 'SPACEMOUSE'
    assert msg.pose_stamped.pose.position.x == pytest.approx(1.0 * hw.SPACEMOUSE_XY_MULT_MM)


# ── (c) device absent: every tick publishes home; open() rate-limited ──────

def test_device_absent_publishes_home_and_rate_limits_reopen(node, fake_pm, clock):
    fake_pm.open_result = False
    node.control_mode_callback(_mode('SPACEMOUSE'))

    node.publish_pose()
    assert fake_pm.open_calls == 1
    home = node.publisher_.published[-1]
    assert home.pose_stamped.pose.position.x == 0.0
    assert home.pose_stamped.pose.position.y == 0.0
    assert home.pose_stamped.pose.position.z == pytest.approx(hw.JB_OP_DEFAULT_ACTIVE_Z_MM)
    assert (home.pose_stamped.pose.orientation.x,
            home.pose_stamped.pose.orientation.y,
            home.pose_stamped.pose.orientation.z,
            home.pose_stamped.pose.orientation.w) == (0.0, 0.0, 0.0, 1.0)

    # Under 2 s since the last attempt: no new open() call.
    clock.advance(1.0)
    node.publish_pose()
    assert fake_pm.open_calls == 1

    # Past the 2 s cadence: one more attempt.
    clock.advance(1.5)
    node.publish_pose()
    assert fake_pm.open_calls == 2


# ── (d) a read() failure closes, homes, and recovers later ─────────────────

def test_read_failure_closes_homes_and_recovers(node, fake_pm, clock):
    node.control_mode_callback(_mode('SPACEMOUSE'))
    fake_pm.read_result = _axes(x=0.2)
    node.publish_pose()  # opens + streams
    assert node._is_open is True
    assert fake_pm.open_calls == 1

    fake_pm.read_result = RuntimeError('device unplugged')
    node.publish_pose()

    assert fake_pm.close_calls == 1
    assert node._is_open is False
    home = node.publisher_.published[-1]
    assert home.pose_stamped.pose.position.x == 0.0
    assert home.pose_stamped.pose.position.z == pytest.approx(hw.JB_OP_DEFAULT_ACTIVE_Z_MM)

    # The disconnect resets the limiter for a prompt first reconnect try —
    # the very next tick attempts to reopen even with no clock advance.
    fake_pm.open_result = False
    node.publish_pose()
    assert fake_pm.open_calls == 2
    assert node.publisher_.published[-1].pose_stamped.pose.position.z == \
        pytest.approx(hw.JB_OP_DEFAULT_ACTIVE_Z_MM)

    # That failed attempt re-arms the normal 2 s cadence.
    node.publish_pose()
    assert fake_pm.open_calls == 2
    clock.advance(2.1)
    node.publish_pose()
    assert fake_pm.open_calls == 3

    # Device comes back: next cadence-eligible tick resumes streaming.
    fake_pm.open_result = True
    fake_pm.read_result = _axes(x=0.5)
    clock.advance(2.1)
    node.publish_pose()

    assert node._is_open is True
    resumed = node.publisher_.published[-1]
    assert resumed.pose_stamped.pose.position.x == pytest.approx(0.5 * hw.SPACEMOUSE_XY_MULT_MM)


# ── (e) leaving the mode destroys the timer + closes the device ────────────

def test_leave_mode_destroys_timer_and_closes_device(node, fake_pm):
    node.control_mode_callback(_mode('SPACEMOUSE'))
    fake_pm.read_result = _axes(x=0.1)
    node.publish_pose()
    assert node._is_open is True
    timer_before = node.timer

    node.control_mode_callback(_mode('STANDBY'))

    assert node.timer is None
    assert node.calls['destroy_timer'] == [timer_before]
    assert fake_pm.close_calls == 1
    assert node._is_open is False

    # Idempotent: another non-SPACEMOUSE message is a no-op, not a re-close.
    node.control_mode_callback(_mode('GUI'))
    assert node.calls['destroy_timer'] == [timer_before]
    assert fake_pm.close_calls == 1


# ── (f) repeated identical mode messages: no extra timers, no reopen ───────

def test_repeated_identical_mode_messages_are_idempotent(node, fake_pm):
    node.control_mode_callback(_mode('SPACEMOUSE'))
    assert len(node.calls['create_timer']) == 1

    fake_pm.read_result = _axes(x=0.3)
    node.publish_pose()
    open_calls_after_first_tick = fake_pm.open_calls

    node.control_mode_callback(_mode('SPACEMOUSE'))
    node.control_mode_callback(_mode('SPACEMOUSE'))

    assert len(node.calls['create_timer']) == 1
    assert fake_pm.open_calls == open_calls_after_first_tick
    assert node.calls['destroy_timer'] == []


# ── (g) main() spins, it does not spin_once-poll ────────────────────────────

def test_main_uses_spin_not_spin_once(monkeypatch, fake_pm):
    """Mirrors test_skill_node.py's `test_main_runs_a_multi_threaded_executor_
    not_plain_spin` pattern: monkeypatch `rclpy.spin` (raising=False — the mock
    module has no such attribute by default) and drive `main()` for real
    rather than string-matching source, which is the less brittle pin."""
    spun = {}

    def _spin(node_arg):
        spun['node'] = node_arg

    monkeypatch.setattr(rclpy, 'init', lambda *a, **k: None)
    monkeypatch.setattr(rclpy, 'shutdown', lambda *a, **k: None)
    monkeypatch.setattr(rclpy, 'spin', _spin, raising=False)
    monkeypatch.setattr(
        rclpy, 'spin_once',
        lambda *a, **k: pytest.fail('spin_once is back in main()'))

    sm.main()

    assert isinstance(spun.get('node'), sm.SpaceMouseHandler)
