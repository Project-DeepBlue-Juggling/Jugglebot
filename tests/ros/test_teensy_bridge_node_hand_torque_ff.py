"""Node-level tests for the hand torque-FF gain gate (C2FF spec, U3a).

Covers what U3a owns on the node side:

  * the ``hand_torque_ff_gain`` ROS param — declaration, initial-value
    validation, and live-update-with-refusal via
    ``TeensyBridgeNode._on_set_parameters`` (rclpy's
    ``add_on_set_parameters_callback`` idiom);
  * ``_set_hand_torque_scale_verified`` — the single setter for the
    fail-safe verified flag, and its push into the ``SetpointPump``;
  * the K>0-while-unverified WARN-once latch
    (``_maybe_warn_hand_ff_unverified``);
  * the ``hand_torque_ff_gain_requested`` / ``_effective`` /
    ``hand_torque_scale_verified`` ``/link_status`` diagnostics rows.

The pump's OWN packing logic (v2/HAS_V2/HAS_SCHED/accel[6]/hand_ff_gain) is
unit-tested in isolation in tests/teensy_link/test_setpoint_pump.py — this
file only tests the node's gate + plumbing into the pump.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from teensy_link.setpoint_pump import HAND_FF_GAIN_MAX

from tests.ros._bridge_harness import _build_paired_node, _teardown, _messages
from tests.ros.conftest import _MockParameter


@pytest.fixture
def bridge():
    teensy, client, node = _build_paired_node()
    yield teensy, node
    _teardown(teensy, client, node)


def _set(node, value):
    """Push one hand_torque_ff_gain param-set through the real callback path
    (mirrors what a `ros2 param set` / launch override does)."""
    return node.set_parameters([_MockParameter(value, name='hand_torque_ff_gain')])


# ── param declaration + initial value ──────────────────────────────────────

def test_default_gain_is_zero_and_unverified(bridge):
    _teensy, node = bridge
    assert node.get_parameter('hand_torque_ff_gain').value == 0.0
    assert node._sp_pump.hand_ff_gain == 0.0
    assert node._hand_torque_scale_verified is False


# ── live update via the param callback ──────────────────────────────────────

def test_valid_set_updates_pump_live(bridge):
    _teensy, node = bridge
    results = _set(node, 0.85)
    assert results[0].successful is True
    assert node.get_parameter('hand_torque_ff_gain').value == 0.85
    assert node._sp_pump.hand_ff_gain == 0.85


@pytest.mark.parametrize("bad", [-0.01, HAND_FF_GAIN_MAX + 0.01, float('nan'), float('inf')])
def test_out_of_range_or_nonfinite_set_is_refused(bridge, bad):
    _teensy, node = bridge
    node._sp_pump.set_hand_ff_gain(0.3)  # a known-good prior value
    results = _set(node, bad)
    assert results[0].successful is False
    # Refused -> the pump's prior value is kept (the pump is re-validated
    # independently — belt-and-suspenders per the module docstring). The
    # param's own stored value is untouched too (MockNode.set_parameters only
    # commits on success, mirroring rclpy).
    assert node._sp_pump.hand_ff_gain == 0.3
    assert node.get_parameter('hand_torque_ff_gain').value == 0.0  # untouched default


def test_boundary_values_accepted(bridge):
    _teensy, node = bridge
    assert _set(node, 0.0)[0].successful is True
    assert _set(node, HAND_FF_GAIN_MAX)[0].successful is True
    assert node._sp_pump.hand_ff_gain == HAND_FF_GAIN_MAX


def test_non_numeric_set_is_refused(bridge):
    _teensy, node = bridge
    results = _set(node, "not-a-number")
    assert results[0].successful is False


# ── _set_hand_torque_scale_verified: the single setter ───────────────────

def test_verified_setter_pushes_into_pump_and_logs(bridge):
    _teensy, node = bridge
    node._logger = MagicMock()
    node._sp_pump.set_hand_ff_gain(0.5)
    assert node._sp_pump.hand_torque_scale_verified is False
    node._set_hand_torque_scale_verified(True, 'readback matched 1000.0')
    assert node._hand_torque_scale_verified is True
    assert node._sp_pump.hand_torque_scale_verified is True
    assert any('True' in m for m in _messages(node._logger.info))

    node._logger = MagicMock()
    node._set_hand_torque_scale_verified(False, 'hand ODrive rebooted')
    assert node._hand_torque_scale_verified is False
    assert node._sp_pump.hand_torque_scale_verified is False
    assert any('False' in m for m in _messages(node._logger.warn))


# ── WARN-once: K>0 requested while unverified ────────────────────────────

def test_warn_once_on_set_while_unverified(bridge):
    _teensy, node = bridge
    node._logger = MagicMock()
    _set(node, 0.4)
    warns_after_first = len(_messages(node._logger.warn))
    assert warns_after_first >= 1
    # A second identical (still-unverified) condition must NOT warn again.
    _set(node, 0.5)
    assert len(_messages(node._logger.warn)) == warns_after_first


def test_no_warn_when_verified(bridge):
    _teensy, node = bridge
    node._set_hand_torque_scale_verified(True, 'ok')
    node._logger = MagicMock()
    _set(node, 0.4)
    assert _messages(node._logger.warn) == []


def test_no_warn_when_gain_is_zero(bridge):
    _teensy, node = bridge
    node._logger = MagicMock()
    _set(node, 0.0)
    assert _messages(node._logger.warn) == []


def test_warn_refires_after_verified_drops_again(bridge):
    _teensy, node = bridge
    node._logger = MagicMock()
    _set(node, 0.4)
    assert len(_messages(node._logger.warn)) >= 1
    node._set_hand_torque_scale_verified(True, 'ok')  # clears the latch
    node._logger = MagicMock()
    node._set_hand_torque_scale_verified(False, 'ODrive fault')  # re-arms it
    # _set_hand_torque_scale_verified(False) itself re-checks and should warn
    # (K is still 0.4 from the earlier set).
    assert len(_messages(node._logger.warn)) >= 1


# ── /link_status diagnostics rows ────────────────────────────────────────

def test_link_status_reports_requested_effective_and_verified(bridge):
    _teensy, node = bridge
    _set(node, 0.6)
    node._publish_link_status()
    kv = {v.key: v.value for v in node.link_status_pub.published[-1].values}
    assert kv['hand_torque_ff_gain_requested'] == '0.6000'
    # Unverified -> effective reads 0, even though 0.6 was requested.
    assert kv['hand_torque_ff_gain_effective'] == '0.0000'
    assert kv['hand_torque_scale_verified'] == '0'

    node._set_hand_torque_scale_verified(True, 'readback ok')
    node._publish_link_status()
    kv = {v.key: v.value for v in node.link_status_pub.published[-1].values}
    assert kv['hand_torque_ff_gain_requested'] == '0.6000'
    assert kv['hand_torque_ff_gain_effective'] == '0.6000'
    assert kv['hand_torque_scale_verified'] == '1'


# ── readback wiring at hand activation (C2FF, 2026-09-15) ──────────────────

import time as _time
from types import SimpleNamespace

import jugglebot.protocol_config as _proto


def _hand_state(node, state, *, readback=None, set_ok=True):
    """Drive _svc_set_hand_state with the axis-state RPC and the readback
    stubbed; returns the service response."""
    node.teensy_set_axis_state = MagicMock(return_value=(set_ok, 'refused', None))
    if readback is not None:
        node._rpc = MagicMock()
        if isinstance(readback, Exception):
            node._rpc.read_hand_input_torque_scale.side_effect = readback
        else:
            node._rpc.read_hand_input_torque_scale.return_value = readback
    node._HAND_TSCALE_RETRY_S = 0.0
    return node._svc_set_hand_state(SimpleNamespace(data=state),
                                    SimpleNamespace(success=None, message=None))


def _wait_for(pred, timeout=2.0):
    t_end = _time.monotonic() + timeout
    while _time.monotonic() < t_end:
        if pred():
            return True
        _time.sleep(0.005)
    return pred()


def test_closed_loop_with_matching_scale_verifies(bridge):
    _teensy, node = bridge
    res = _hand_state(node, 'CLOSED_LOOP',
                      readback=int(_proto.INPUT_SCALE_HAND_TOR))
    assert res.success is True
    assert _wait_for(lambda: node._hand_torque_scale_verified is True)
    assert node._sp_pump.hand_torque_scale_verified is True


def test_closed_loop_with_wrong_scale_stays_unverified(bridge):
    _teensy, node = bridge
    _hand_state(node, 'CLOSED_LOOP', readback=100)
    assert _wait_for(lambda: node._rpc.read_hand_input_torque_scale.called)
    assert _wait_for(lambda: '100' in node._hand_torque_scale_verified_detail)
    assert node._hand_torque_scale_verified is False


def test_readback_failure_retries_then_stays_unverified(bridge):
    _teensy, node = bridge
    _hand_state(node, 'CLOSED_LOOP', readback=RuntimeError('state 3'))
    assert _wait_for(lambda: node._rpc.read_hand_input_torque_scale.call_count
                     == node._HAND_TSCALE_ATTEMPTS)
    assert _wait_for(lambda: 'readback failed' in
                     node._hand_torque_scale_verified_detail)
    assert node._hand_torque_scale_verified is False


def test_any_hand_state_change_drops_verification(bridge):
    _teensy, node = bridge
    node._set_hand_torque_scale_verified(True, 'test')
    _hand_state(node, 'IDLE')
    assert node._hand_torque_scale_verified is False
    assert node._sp_pump.hand_torque_scale_verified is False


def test_failed_closed_loop_never_starts_a_readback(bridge):
    _teensy, node = bridge
    res = _hand_state(node, 'CLOSED_LOOP', readback=1000, set_ok=False)
    assert res.success is False
    _time.sleep(0.05)
    assert not node._rpc.read_hand_input_torque_scale.called
    assert node._hand_torque_scale_verified is False


def test_a_stale_readback_cannot_verify_after_a_later_state_change(bridge):
    _teensy, node = bridge
    gate = SimpleNamespace(release=False, entered=False)

    def _slow_read():
        gate.entered = True
        assert _wait_for(lambda: gate.release)
        return int(_proto.INPUT_SCALE_HAND_TOR)   # a MATCH — must still be ignored

    node._rpc = MagicMock()
    node._rpc.read_hand_input_torque_scale.side_effect = _slow_read
    _hand_state(node, 'CLOSED_LOOP')
    assert _wait_for(lambda: gate.entered)
    # A second state change lands while the first readback is still in flight.
    _hand_state(node, 'IDLE')
    gate.release = True
    _time.sleep(0.1)
    assert node._hand_torque_scale_verified is False
