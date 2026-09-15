"""Node-level tests for the hand torque-FF gain (C2FF spec, U3a).

Covers what U3a owns on the node side:

  * the ``hand_torque_ff_gain`` ROS param — declaration, initial-value
    validation, and live-update-with-refusal via
    ``TeensyBridgeNode._on_set_parameters`` (rclpy's
    ``add_on_set_parameters_callback`` idiom);
  * the ``hand_torque_ff_gain_requested`` / ``_effective`` ``/link_status``
    diagnostics rows.

The torque-scale readback gate (``hand_torque_scale_verified``) was REMOVED
2026-09-15 (see logbook/2026-09-15-hand-torque-ff-gate-removed.md): ACTIVATE
arms the hand through a path that never ran the readback, so the wire gain
was silently forced to 0 for a whole session. The owner confirmed the hand
ODrive's ``input_torque_scale=1000`` and decided to remove the gate — the
wire gain now follows the param directly. The RPC (``GET_HAND_TORQUE_SCALE`` /
``RpcClient.read_hand_input_torque_scale``) is retained as a manual
diagnostic; the node no longer calls it.

The pump's OWN packing logic (v2/HAS_V2/HAS_SCHED/accel[6]/hand_ff_gain) is
unit-tested in isolation in tests/teensy_link/test_setpoint_pump.py — this
file only tests the node's param plumbing into the pump.

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

def test_default_gain_is_zero(bridge):
    _teensy, node = bridge
    assert node.get_parameter('hand_torque_ff_gain').value == 0.0
    assert node._sp_pump.hand_ff_gain == 0.0


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


# ── /link_status diagnostics rows ────────────────────────────────────────

def test_link_status_reports_requested_and_effective(bridge):
    _teensy, node = bridge
    _set(node, 0.6)
    node._publish_link_status()
    kv = {v.key: v.value for v in node.link_status_pub.published[-1].values}
    assert kv['hand_torque_ff_gain_requested'] == '0.6000'
    # No gate anymore -> effective tracks requested directly (the pump only
    # ever forces 0 on a legs-only frame, tested in test_setpoint_pump.py).
    assert kv['hand_torque_ff_gain_effective'] == '0.6000'
    assert 'hand_torque_scale_verified' not in kv
