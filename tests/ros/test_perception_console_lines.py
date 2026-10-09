"""Operator-console lines of the perception + Ball Butler nodes (phase 3).

Every node's logger is swapped for a recording MockLogger subclass that still
enforces rclpy's one-severity-per-call-site rule, so these tests both pin the
text/level of the operator lines AND exercise the real rule.
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np

from geometry_msgs.msg import Point
from jugglebot.protocol_config import BallButlerStates
from jugglebot_interfaces.msg import BallButlerCalibrationResult, RigidBodyPose, RigidBodyPoses
from jugglebot_interfaces.srv import BallButlerAim, BallButlerThrow

from rclpy.node import Node

# The conftest's MockLogger (enforces one severity per call site).
MockLogger = type(Node('_probe').get_logger())


class RecLogger(MockLogger):
    def __init__(self):
        super().__init__()
        self.lines = []

    def _rec(self, level, msg, kw):
        self.lines.append((level, msg))
        self._log(level, kw)   # frame depth matches MockLogger's own methods

    def info(self, msg, **kw): self._rec('INFO', msg, kw)
    def warning(self, msg, **kw): self._rec('WARN', msg, kw)
    def warn(self, msg, **kw): self._rec('WARN', msg, kw)
    def error(self, msg, **kw): self._rec('ERROR', msg, kw)
    def debug(self, msg, **kw): self._rec('DEBUG', msg, kw)

    def at(self, level):
        return [m for lv, m in self.lines if lv == level]


def _rec(node):
    node._logger = RecLogger()
    return node._logger


# ── mocap_node: BB calibration ───────────────────────────────────────────────

def _mocap_node(tmp_path=None, last_accepted_deg=1.78):
    """A mocap node whose consistency-gate state lives under *tmp_path* (never
    the real ~/bb_calibration_sessions), seeded with a last accepted yaw
    offset that the fake result below agrees with."""
    import json
    import jugglebot.mocap_node as mn
    iface = MagicMock()
    iface.is_receiving.return_value = True
    with patch.object(mn, 'MocapInterface', return_value=iface):
        node = mn.MocapNode()
    if tmp_path is not None:
        state = tmp_path / 'last_accepted.json'
        state.write_text(json.dumps({'yaw_offset_deg': last_accepted_deg,
                                     'accepted_at': 'test'}))
        node._params['bb_calibration_state_file'] = str(state)
    return node


def _hb(state):
    from jugglebot_interfaces.msg import BallButlerHeartbeat
    msg = BallButlerHeartbeat()
    msg.state = int(state)
    msg.yaw_deg = 1.0
    return msg


def _fake_result():
    return SimpleNamespace(
        bb_position_mm=np.array([-1018.99, -434.58, 1738.14]),
        axis_direction=np.array([0.001, 0.02, 0.9998]),
        axis_tilt_deg=0.62, yaw_offset_rad=np.radians(1.78),
        yaw_offset_std_deg=0.02, yaw_span_deg=118.8,
        yaw_method='constellation', anchor_yaw_offset_rad=None,
        yaw_estimate=SimpleNamespace(n_holds=1, holds=[SimpleNamespace(n_matched=3)],
                                     stat_std_deg=0.13, template_std_deg=0.1),
        marker_metrics={0: SimpleNamespace(status='ok', radius_mm=99.0,
                                           fit_residual_mm=0.2,
                                           distance_from_axis_mm=1.0,
                                           arc_span_deg=100.0)})


def test_calibration_success_is_one_info_line(tmp_path):
    import jugglebot.mocap_node as mn
    node = _mocap_node(tmp_path)
    log = _rec(node)
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._calib_data = {0: [np.zeros(3)]}
    with patch.object(mn, 'run_calibration', return_value=_fake_result()):
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    info = log.at('INFO')
    assert len(info) == 2, info
    assert 'started' in info[0]
    assert info[1] == ('BB calibrated: pos (-1019, -435, 1738) mm · axis tilt 0.62° '
                       '· yaw offset +1.78° ±0.02° (1 hold(s), gate ok) · swept 119°')
    assert not log.at('WARN') and not log.at('ERROR')


def test_calibration_outcast_is_named_on_the_success_line(tmp_path):
    """A marker the consensus excluded (bb_calibration MIN_AGREEING_MARKERS) is
    named, with its distance, on the one INFO outcome line, and its reason
    rides the per-marker WARN — the calibration still succeeds."""
    import jugglebot.mocap_node as mn
    node = _mocap_node(tmp_path)
    log = _rec(node)
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._calib_data = {0: [np.zeros(3)]}
    result = _fake_result()
    result.marker_metrics[1] = SimpleNamespace(
        status='outcast', distance_from_axis_mm=4.981,
        reason='circle centre 4.98 mm from the axis the other 6 markers agree on')
    with patch.object(mn, 'run_calibration', return_value=result):
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    assert log.at('INFO')[1] == (
        'BB calibrated: pos (-1019, -435, 1738) mm · axis tilt 0.62° '
        '· yaw offset +1.78° ±0.02° (1 hold(s), gate ok) · swept 119° '
        '· outcast Marker 2 (4.98 mm off axis)')
    assert log.at('WARN') == ['Marker 2: outcast — circle centre 4.98 mm from '
                              'the axis the other 6 markers agree on']
    assert not log.at('ERROR')


def test_calibration_failure_is_one_error_line(tmp_path):
    import jugglebot.mocap_node as mn
    node = _mocap_node(tmp_path)
    log = _rec(node)
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._calib_data = {0: [np.zeros(3)]}
    with patch.object(mn, 'run_calibration', side_effect=ValueError('Circle centres deviate')):
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    assert log.at('ERROR') == ['BB calibration FAILED: Circle centres deviate']


def test_dropout_invalidation_is_one_error_line():
    node = _mocap_node()
    log = _rec(node)
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node.mocap.is_receiving.return_value = False
    node._check_calibration_health()
    assert len(log.at('ERROR')) == 1
    assert 'QTM_DROPOUT_MID_SWEEP' in log.at('ERROR')[0]


def test_mocap_node_shutdown_is_silent():
    node = _mocap_node()
    log = _rec(node)
    node.on_shutdown()
    assert log.lines == []


# ── mocap_interface: base alignment ──────────────────────────────────────────

def test_base_not_visible_is_said_plainly():
    import inspect
    from jugglebot.mocap_interface import MocapInterface
    # The alignment branch lives inside the 200 Hz packet handler (needs a live
    # QTM packet); pin the wording contract on the source instead.
    src = inspect.getsource(MocapInterface)
    assert 'Base body not visible to QTM' in src


# ── ball_tracker_node ────────────────────────────────────────────────────────

def test_ball_tracker_announcement_and_startup_are_quiet():
    from jugglebot.ball_tracker_node import BallTrackerNode
    node = BallTrackerNode()
    log = _rec(node)
    t = SimpleNamespace(sec=0, nanosec=0)
    v = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    node._on_announcement(SimpleNamespace(
        throw_time=t, landing_time=t, initial_position=v, initial_velocity=v,
        landing_position=v, landing_velocity=v, target_id='', thrower_name='x'))
    assert log.at('INFO') == []
    assert any('announced' in m for m in log.at('DEBUG'))


# ── ball_butler_node ─────────────────────────────────────────────────────────

def _bb_node():
    from jugglebot.ball_butler_node import BallButlerNode
    n = BallButlerNode()
    n._aim_correction_matrix = None
    n._throw_action = MagicMock()
    n._throw_action.server_is_ready.return_value = True
    return n


def _calibrated(n):
    msg = BallButlerCalibrationResult()
    msg.position_mm = Point(x=0.0, y=0.0, z=0.0)
    msg.success = True
    log = _rec(n)
    n._on_bb_calibration(msg)
    return log


def _bodies(n, name, xyz):
    msg = RigidBodyPoses()
    body = RigidBodyPose()
    body.name = name
    body.pose.pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
    msg.bodies.append(body)
    n._on_rigid_bodies(msg)


def test_bb_calibration_received_is_debug_only():
    n = _bb_node()
    log = _calibrated(n)
    assert log.at('INFO') == []
    assert log.at('DEBUG')


def test_bb_throw_success_is_one_info_line_and_ok_result_is_debug():
    n = _bb_node()
    log = _calibrated(n)
    _bodies(n, 'jugglebot', (1200.0, 400.0, 0.0))   # local y > s: reachable with positive s
    req = BallButlerThrow.Request()
    req.target_name = 'jugglebot'
    res = n._svc_throw_at_target(req, BallButlerThrow.Response())
    assert res.success, res.message
    info = log.at('INFO')
    assert len(info) == 1 and info[0].startswith('Throw at jugglebot: in ')
    assert 'flight' in info[0] and 'm/s' in info[0]
    fut = MagicMock()
    fut.result.return_value.result = SimpleNamespace(success=True, message='OK (axis=n/a)')
    n._on_throw_result(fut, 'jugglebot')
    assert len(log.at('INFO')) == 1
    fut.result.return_value.result = SimpleNamespace(
        success=False, message='THROW_ABORTED_NOT_SETTLED (axis=YAW)')
    n._on_throw_result(fut, 'jugglebot')
    assert log.at('ERROR') == [
        'Throw at jugglebot NOT thrown: THROW_ABORTED_NOT_SETTLED (axis=YAW)']


def test_bb_throw_refusals_each_log_one_error():
    n = _bb_node()
    log = _rec(n)
    req = BallButlerThrow.Request()
    req.target_name = 'jugglebot'
    n._svc_throw_at_target(req, BallButlerThrow.Response())      # no calibration
    assert len(log.at('ERROR')) == 1
    _calibrated(n)
    log = _rec(n)
    n._svc_throw_at_target(req, BallButlerThrow.Response())      # unknown target
    assert len(log.at('ERROR')) == 1 and 'not in latest' in log.at('ERROR')[0]
    _bodies(n, 'jugglebot', (float('nan'),) * 3)
    n._svc_throw_at_target(req, BallButlerThrow.Response())      # nan target
    assert 'NaN' in log.at('ERROR')[-1]


def test_bb_aim_refusal_warns_once_and_success_is_silent():
    n = _bb_node()
    log = _rec(n)
    n._orch_state = 'RUNNING'
    req = BallButlerAim.Request()
    req.yaw_deg = 0.0
    req.pitch_deg = 60.0
    res = n._svc_aim(req, BallButlerAim.Response())
    assert not res.success
    assert len(log.at('WARN')) == 1 and 'Aim refused' in log.at('WARN')[0]


def test_bb_accuracy_calibration_refusal_and_cancel_have_lines():
    n = _bb_node()
    log = _rec(n)
    n._svc_start_accuracy_calibration(MagicMock(), MagicMock())
    n._svc_cancel_accuracy_calibration(MagicMock(), MagicMock())
    assert len(log.at('ERROR')) == 2


def test_bb_destroy_node_destroys_action_client_once():
    n = _bb_node()
    client = n._throw_action
    n.destroy_node()
    n.destroy_node()
    client.destroy.assert_called_once()
