"""Tests for jugglebot.motion_bridge_node.MotionBridgeNode.

Focus: the fault-observability invariant on the IPC→ROS2 telemetry
poll.  A motor_guard ``GuardMode.ESTOP`` reaches the bridge via
``_publish_fault_telemetry``, whose dict carries ``fault_state`` but
deliberately OMITS ``cond_number``.  Pre-2026-05-19 the bridge gated
the whole ``DiagnosticStatus`` publish behind ``cond_number is not
None``, so an ESTOP was never published on ``motion/diagnostics`` —
invisible to operator / GUI / rosbag for the entire fault (root cause
of the 68 s blind window in
``logbook/2026-05-19-findingb-motor-guard-estop-latch-observability.md``).

INVARIANT under test: a fault / ESTOP signal is published regardless
of whether the optional monitoring field ``cond_number`` is present.

ROS2 is mocked by ``tests/ros/conftest.py``; ``BridgeIPC`` is patched
per-fixture so construction does not open ZMQ sockets (mirrors
``test_orchestrator_node``'s all-externals-mocked pattern).
"""

from unittest.mock import MagicMock, patch

import pytest

from diagnostic_msgs.msg import DiagnosticStatus


@pytest.fixture
def bridge():
    """A MotionBridgeNode with ZMQ IPC mocked out."""
    with patch('jugglebot.motion_bridge_node.BridgeIPC',
               return_value=MagicMock()):
        from jugglebot.motion_bridge_node import MotionBridgeNode
        return MotionBridgeNode()


def _set_telem(node, telem):
    node.ipc.recv_telemetry = MagicMock(return_value=telem)


def _published(node):
    """The DiagnosticStatus messages the bridge published this run."""
    return node._diagnostics_pub.published


def _kv(diag):
    return {kv.key: kv.value for kv in diag.values}


# Exact shape motor_guard._publish_fault_telemetry emits during ESTOP:
# fault_state + workspace_status present, NO cond_number / speed_scale.
_ESTOP_TELEM = {
    'type': 'telemetry',
    'leg_pos': None, 'leg_vel': None, 'leg_torques': None,
    'dt': 0.002,
    'fault_state': 'mpc_cmd_stale: 0.412s since last command',
    'workspace_status': 'ok',
    'timestamp': 1.0,
}


class TestFaultObservability:
    """The Finding-B regression: an ESTOP must be observable even
    though its telemetry path omits cond_number."""

    def test_estop_fault_published_without_cond_number(self, bridge):
        """ESTOP-shaped telem (no cond_number) ⇒ exactly one
        ERROR DiagnosticStatus naming the fault."""
        _set_telem(bridge, _ESTOP_TELEM)
        bridge._poll_telemetry()

        pub = _published(bridge)
        assert len(pub) == 1, (
            f"expected exactly one DiagnosticStatus; got {len(pub)} "
            "(pre-fix this was 0 — the fault was silently swallowed)"
        )
        diag = pub[-1]
        assert diag.level == DiagnosticStatus.ERROR
        assert diag.message == (
            'FAULT: mpc_cmd_stale: 0.412s since last command')
        kv = _kv(diag)
        assert kv['fault_state'] == _ESTOP_TELEM['fault_state']
        assert 'cond_number' not in kv, (
            "ESTOP path carries no cond_number; it must not be fabricated"
        )

    def test_fault_published_even_with_hard_workspace(self, bridge):
        """fault_state takes precedence over workspace status, and is
        still published with no cond_number."""
        t = dict(_ESTOP_TELEM, workspace_status='hard',
                 fault_state='workspace_hard_limit: leg 3')
        _set_telem(bridge, t)
        bridge._poll_telemetry()

        diag = _published(bridge)[-1]
        assert diag.level == DiagnosticStatus.ERROR
        assert diag.message == 'FAULT: workspace_hard_limit: leg 3'


class TestHappyPathUnchanged:
    """Regression guard: the normal cond-bearing path is byte-for-byte
    unchanged by the fix."""

    def _normal(self, **over):
        t = {
            'type': 'telemetry',
            'leg_pos': [0.0] * 6, 'leg_vel': [0.0] * 6,
            'leg_torques': [0.0] * 6, 'dt': 0.002,
            'cond_number': 4.2, 'workspace_status': 'ok',
            'workspace_speed_scale': 1.0, 'fault_state': None,
            'timestamp': 1.0,
        }
        t.update(over)
        return t

    def test_normal_ok(self, bridge):
        _set_telem(bridge, self._normal())
        bridge._poll_telemetry()
        diag = _published(bridge)[-1]
        assert diag.level == DiagnosticStatus.OK
        assert diag.message == 'OK'
        kv = _kv(diag)
        assert kv['cond_number'] == '4.2'
        assert kv['workspace_status'] == 'ok'
        assert kv['workspace_speed_scale'] == '1.000'
        assert 'fault_state' not in kv

    def test_normal_soft_workspace_warn(self, bridge):
        _set_telem(bridge, self._normal(workspace_status='soft',
                                        workspace_speed_scale=0.5))
        bridge._poll_telemetry()
        diag = _published(bridge)[-1]
        assert diag.level == DiagnosticStatus.WARN
        assert diag.message == 'Workspace soft limit'

    def test_normal_with_cond_and_fault_still_errors(self, bridge):
        """cond present AND fault present ⇒ ERROR, both fields in values."""
        _set_telem(bridge, self._normal(fault_state='max_deviation: leg 1'))
        bridge._poll_telemetry()
        diag = _published(bridge)[-1]
        assert diag.level == DiagnosticStatus.ERROR
        assert diag.message == 'FAULT: max_deviation: leg 1'
        kv = _kv(diag)
        assert kv['cond_number'] == '4.2'
        assert kv['fault_state'] == 'max_deviation: leg 1'


class TestNoSpuriousDiagnostics:
    def test_no_cond_no_fault_publishes_nothing(self, bridge):
        """Telem with neither cond_number nor fault_state (e.g. the
        pre-first-MPC feedback path) must not emit diagnostic spam."""
        _set_telem(bridge, {
            'type': 'telemetry', 'leg_pos': None, 'leg_vel': None,
            'leg_torques': None, 'dt': 0.002, 'timestamp': 1.0,
        })
        bridge._poll_telemetry()
        assert _published(bridge) == [], (
            "no fault and no cond ⇒ nothing to report"
        )

    def test_none_telemetry_is_a_noop(self, bridge):
        _set_telem(bridge, None)
        bridge._poll_telemetry()
        assert _published(bridge) == []
