"""BUG-129: the bridge's ``stall_watchdog`` parameter starts the stall watchdog
only when true, feeds it the setpoint loop's age lock-free, and stops it in
``on_shutdown``.  ROS 2 is mocked by tests/ros/conftest.py; the module itself
is covered by tests/ros/test_stall_watchdog.py."""

from __future__ import annotations

import threading
import time

from tests.ros._bridge_harness import _build_paired_node, _teardown


def _params(monkeypatch, overrides):
    from jugglebot.teensy_bridge_node import TeensyBridgeNode
    orig = TeensyBridgeNode.declare_parameter

    def declare(self, name, default_value=None, *a, **kw):
        if name in overrides:
            default_value = overrides[name]
        return orig(self, name, default_value, *a, **kw)
    monkeypatch.setattr(TeensyBridgeNode, 'declare_parameter', declare)


def test_stall_watchdog_off_by_default_starts_nothing():
    teensy, client, node = _build_paired_node()
    try:
        assert node._stall_wd is None
        assert not any(t.name == 'stall_watchdog' for t in threading.enumerate())
    finally:
        _teardown(teensy, client, node)


def test_stall_watchdog_on_runs_and_on_shutdown_stops_it(monkeypatch, tmp_path):
    _params(monkeypatch, {'stall_watchdog': True, 'stall_watchdog_timeout_s': 0.2,
                          'stall_watchdog_dir': str(tmp_path)})
    teensy, client, node = _build_paired_node()
    wd = node._stall_wd
    try:
        assert wd is not None and wd.running
        assert wd.timeout_s == 0.2 and wd.out_dir == str(tmp_path)
    finally:
        _teardown(teensy, client, node)
    assert not wd.running


def test_setpoint_loop_age_is_none_unless_the_loop_runs():
    teensy, client, node = _build_paired_node()
    try:
        node._sp_thread = None
        assert node._setpoint_loop_age_s() is None
        stop = threading.Event()
        t = threading.Thread(target=stop.wait, name='fake_setpoint')
        t.start()
        try:
            node._sp_thread = t
            node._loop_gap.reset()
            assert node._setpoint_loop_age_s() is None        # no tick yet
            node._loop_gap.record(time.monotonic() - 0.5)
            age = node._setpoint_loop_age_s()
            assert age is not None and 0.5 <= age < 5.0
        finally:
            stop.set()
            t.join()
            node._sp_thread = None
    finally:
        _teardown(teensy, client, node)


def test_watchdog_on_during_normal_construction_reports_no_stall(monkeypatch, tmp_path):
    """The watchdog starts as the LAST statement of __init__ (BUG-129 audit): a node
    built the normal way, watchdog on, produces no report and no exception over
    several ticks, and the heartbeat never raised."""
    _params(monkeypatch, {'stall_watchdog': True, 'stall_watchdog_timeout_s': 0.2,
                          'stall_watchdog_dir': str(tmp_path)})
    teensy, client, node = _build_paired_node()
    wd = node._stall_wd
    try:
        assert wd is not None and wd.running
        for _ in range(8):
            wd.tick()
            time.sleep(0.05)
        assert wd.stalls == 0 and wd.reports == []
        assert wd._hb_failed is False
        assert node._setpoint_loop_age_s() is None
    finally:
        _teardown(teensy, client, node)
