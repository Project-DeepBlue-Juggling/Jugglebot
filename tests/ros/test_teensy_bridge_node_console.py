"""Operator-console (phase 3) log-shape tests for teensy_bridge_node.

Logging only: each operator-visible event is ONE short line, detail rides DEBUG,
and every call site keeps a single severity (the real MockLogger enforces rclpy's
per-call-site rule).
"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

from tests.ros._bridge_harness import _build_paired_node, _teardown


class _Rec:
    """Records (severity, text); accepts throttle kwargs like rclpy."""
    def __init__(self):
        self.lines = []
    def _add(self, sev):
        return lambda msg, **kw: self.lines.append((sev, str(msg)))
    def __getattr__(self, name):
        if name in ('info', 'warning', 'warn', 'error', 'debug'):
            return self._add({'warn': 'warning'}.get(name, name))
        raise AttributeError(name)
    def set_level(self, level):
        self.level = level
    def sev(self, sev):
        return [m for s, m in self.lines if s == sev]


def _node():
    teensy, client, node = _build_paired_node()
    return teensy, client, node


def test_node_records_debug_detail():
    teensy, client, node = _node()
    try:
        from rclpy.logging import LoggingSeverity
        assert node.get_logger().level == LoggingSeverity.DEBUG
    finally:
        _teardown(teensy, client, node)


def test_setpoint_arm_is_a_warning_and_disarm_is_info():
    teensy, client, node = _node()
    try:
        rec = _Rec()
        node._logger = rec
        node._set_mpc_active(True)
        node._set_mpc_active(False)
        # ARMED stays WARN (the leg path goes live: the operator's yellow cue);
        # DISARMED is INFO.
        assert rec.sev('warning')[-1:] == ['setpoint output ARMED']
        assert rec.sev('info')[-1:] == ['setpoint output DISARMED']
        assert rec.sev('warning') == ['setpoint output ARMED']
    finally:
        _teardown(teensy, client, node)


def test_service_outcome_lines_go_through_real_mock_logger():
    """success -> INFO, failure -> ERROR with the reason; both call sites are
    exercised on the REAL MockLogger so a severity flip would raise."""
    teensy, client, node = _node()
    try:
        ok = SimpleNamespace(success=True, message='done')
        bad = SimpleNamespace(success=False, message='because reasons')
        for _ in range(2):
            assert node._log_service_outcome('reboot_odrives', ok) is ok
            assert node._log_service_outcome('reboot_odrives', bad) is bad
            node._log_service_outcome('bb/reload', ok, quiet_success=True)
            node._log_service_outcome('bb/reload', bad, quiet_success=True)
    finally:
        _teardown(teensy, client, node)


def test_service_outcome_severities():
    teensy, client, node = _node()
    try:
        rec = _Rec()
        node._logger = rec
        node._log_service_outcome('reboot_odrives', SimpleNamespace(success=True, message='ok'))
        node._log_service_outcome('reboot_odrives', SimpleNamespace(success=False, message='nope'))
        node._log_service_outcome('bb/reload', SimpleNamespace(success=True, message='sent'),
                                  quiet_success=True)
        assert rec.sev('info') == ['reboot_odrives: ok']
        assert rec.sev('error') == ['reboot_odrives FAILED: nope']
        assert rec.sev('debug') == ['bb/reload: sent']
    finally:
        _teardown(teensy, client, node)


def test_recover_and_clear_errors_wrappers_log_failure_once():
    teensy, client, node = _node()
    try:
        rec = _Rec()
        node._logger = rec
        node._svc_recover = lambda req, res: (setattr(res, 'success', False)
                                               or setattr(res, 'message', 'refused') or res)
        node._svc_clear_errors = node._svc_recover
        res = SimpleNamespace(success=None, message='')
        node._svc_recover_logged(None, res)
        node._svc_clear_errors_logged(None, SimpleNamespace(success=None, message=''))
        assert rec.sev('error') == ['recover FAILED: refused',
                                    'clear_errors FAILED: refused']
        node._svc_recover = lambda req, res: (setattr(res, 'success', True)
                                               or setattr(res, 'message', 'recovered') or res)
        node._svc_recover_logged(None, SimpleNamespace(success=None, message=''))
        assert 'recover: recovered' in rec.sev('info')
    finally:
        _teardown(teensy, client, node)


def test_set_setpoint_output_refusal_is_a_warning_with_reason():
    from std_srvs.srv import SetBool
    teensy, client, node = _node()
    try:
        rec = _Rec()
        node._logger = rec
        node._arm_setpoint_output = lambda: (False, 'link down')
        req = SetBool.Request()
        req.data = True
        node._svc_set_setpoint_output(req, SetBool.Response())
        assert rec.sev('warning') == ['setpoint output NOT armed: link down']
    finally:
        _teardown(teensy, client, node)


def test_hand_gain_service_failure_logs_error_success_is_quiet():
    teensy, client, node = _node()
    try:
        rec = _Rec()
        node._logger = rec
        res = SimpleNamespace(success=None, message='')
        node._svc_set_hand_state(SimpleNamespace(data='NOT_A_STATE'), res)
        assert res.success is False
        assert any('Unknown hand state' in m for m in rec.sev('error'))
    finally:
        _teardown(teensy, client, node)


def test_main_destroys_action_servers_before_node(monkeypatch):
    """Shutdown fix: action servers are destroyed explicitly (guarded), before
    destroy_node, so Foxy's __del__ no longer hits InvalidHandle."""
    import jugglebot.teensy_bridge_node as tbn
    order = []

    class _Srv:
        def destroy(self):
            order.append('srv')
            raise RuntimeError('double destroy is harmless')

    class _Node:
        _home_action = _Srv()
        _bb_throw_action = _Srv()
        def on_shutdown(self): order.append('shutdown')
        def destroy_node(self): order.append('node')

    class _Exec:
        def add_node(self, n): pass
        def spin(self): raise KeyboardInterrupt
        def shutdown(self): order.append('exec')

    monkeypatch.setattr(tbn, 'TeensyBridgeNode', lambda: _Node())
    monkeypatch.setattr(tbn, 'MultiThreadedExecutor', lambda: _Exec())
    monkeypatch.setattr(tbn.rclpy, 'init', lambda args=None: None, raising=False)
    monkeypatch.setattr(tbn.rclpy, 'shutdown', lambda: order.append('rclpy'), raising=False)
    tbn.main()
    assert order == ['shutdown', 'exec', 'srv', 'srv', 'node', 'rclpy']
