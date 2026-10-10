"""mocap_node's calibration consistency gate, its bb_moved override and the
estimator inputs it collects (2026-10-09, sweep estimator).

* With NO state file the first calibration is accepted, persisted and logged
  at WARN (there is no reference; the template's pin is not one).
* Afterwards a calibration is refused when |Δyaw| > max(3·√(σ_new² + σ_ref²), 0.15°), the axis
  point moved > 1.5 mm, or the template residual > 0.5 mm, unless the
  one-shot ``bb_moved`` (BB moved or QTM recalibrated) is armed — which does
  not excuse the residual.
* An unreadable state file refuses (never a silent fallback) unless
  ``bb_moved`` is armed.
* The solver gets mocap frames at their QTM stamps (one per distinct stamp)
  and yaw samples on the ROS clock. The yaw source follows ``bb_yaw_source``
  (2026-10-10): ``auto`` by default — the stamped ``bb_yaw`` (degrees) on
  bb/axis_estimates when enough samples arrived, else the heartbeat (the
  two-joint message of older firmware); ``stamped`` insists on it (refused
  when absent); ``heartbeat`` ignores it. See
  ``logbook/2026-10-10-bb-yaw-offset-spread-stamped-source.md``.

See ``logbook/2026-10-09-bb-constellation-yaw-offset.md``.
"""
from __future__ import annotations

import json
import math
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from jugglebot.protocol_config import BallButlerStates

POS = [-975.75, -389.5, 1735.0]


def _node(tmp_path, state=None, template=True):
    import jugglebot.mocap_node as mn
    iface = MagicMock()
    iface.is_receiving.return_value = True
    with patch.object(mn, 'MocapInterface', return_value=iface):
        node = mn.MocapNode()
    path = tmp_path / 'last_accepted.json'
    if state is not None:
        path.write_text(json.dumps(state))
    node._params['bb_calibration_state_file'] = str(path)
    # These tests pin the WORLD gate: pose source sweep, and a private base
    # pool (never the live ~/bb_calibration_sessions state). Under the default
    # auto with the seeded pool, a window without the base leads its refusal
    # with BASE_FRAME_NOT_SEEN (tests/ros/test_mocap_node_base_frame.py).
    node._params['bb_pose_source'] = 'sweep'
    node._params['bb_base_frame_state_file'] = str(tmp_path / 'base_state.json')
    if not template:
        node._marker_template = None
        node._marker_template_error = 'BB marker template not found (x.json)'
    return node, path


def _ref(yaw=0.6, pos=POS):
    return {'yaw_offset_deg': yaw, 'position_mm': list(pos), 'accepted_at': 't0'}


def _hb(state, yaw=0.0):
    from jugglebot_interfaces.msg import BallButlerHeartbeat
    msg = BallButlerHeartbeat()
    msg.state = int(state)
    msg.yaw_deg = yaw
    return msg


def _result(yaw_deg, std=0.06, pos=POS, tres=0.3):
    return SimpleNamespace(
        bb_position_mm=np.array(pos, dtype=float),
        arc_position_mm=np.array(pos, dtype=float) + [0.1, -1.5, 0.0],
        axis_direction=np.array([0.0, 0.0, 1.0]), axis_tilt_deg=0.8,
        yaw_offset_rad=math.radians(yaw_deg), yaw_offset_std_deg=std,
        yaw_span_deg=125.0, marker_metrics={}, yaw_method='constellation',
        anchor_yaw_offset_rad=math.radians(0.9),
        yaw_estimate=SimpleNamespace(lag_s=0.082, template_residual_mm=tres,
                                     yaw_source='heartbeat', n_frames=1200,
                                     summary=lambda: 'sweep estimator: (fake)'))


def _calibrate(node, result):
    import jugglebot.mocap_node as mn
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._calib_data = {0: [np.zeros(3)]}
    with patch.object(mn, 'run_calibration', return_value=result) as solver:
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    # Every outcome is on bb/calibration_attempt; bb/calibration_result keeps the
    # last success once there is one (keep-last-good, 2026-10-10).
    return solver, node.pub_calibration_attempt.published[-1]


def _rec(node):
    lines = []

    class L:
        def __getattr__(self, level):
            return lambda m, *a, **k: lines.append((level, m))
    node._logger = L()
    return lines


def test_node_loads_the_shipped_template():
    import jugglebot.mocap_node as mn
    with patch.object(mn, 'MocapInterface', return_value=MagicMock()):
        node = mn.MocapNode()
    t = node._marker_template
    assert t is not None and len(t.points_mm) == 7
    # Re-pinned 2026-10-11 from landings (0.208 - 0.179; template gauge.repinned_on).
    assert t.pinned_yaw_offset_deg == pytest.approx(0.029)


def test_first_calibration_without_a_reference_is_accepted_persisted_and_warned(tmp_path):
    node, path = _node(tmp_path)
    log = _rec(node)
    _, msg = _calibrate(node, _result(5.0))      # far from any pin: not compared
    assert msg.success and 'no reference' in msg.message
    assert json.loads(path.read_text())['yaw_offset_deg'] == pytest.approx(5.0)
    warns = [m for lv, m in log if lv == 'warn']
    assert any('no reference calibration exists' in m for m in warns)


def test_consistent_calibration_is_accepted_and_persisted(tmp_path):
    node, path = _node(tmp_path, _ref(0.6))
    _, msg = _calibrate(node, _result(0.70))
    assert msg.success and 'sweep estimator' in msg.message and 'gate' in msg.message
    st = json.loads(path.read_text())
    assert st['yaw_offset_deg'] == pytest.approx(0.70)
    assert st['position_mm'] == pytest.approx(POS)
    assert st['lag_ms'] == pytest.approx(82.0) and st['yaw_source'] == 'heartbeat'


def test_inconsistent_yaw_is_refused_and_not_persisted(tmp_path):
    node, path = _node(tmp_path, _ref(0.6))
    _, msg = _calibrate(node, _result(0.85))     # 0.25° > max(3·0.06, 0.15)
    assert not msg.success and msg.message.startswith('CALIBRATION_INCONSISTENT')
    assert json.loads(path.read_text())['yaw_offset_deg'] == pytest.approx(0.6)


def test_gate_limit_uses_the_state_files_sigma(tmp_path):
    """The same Δ 0.25° that is refused against a σ-less reference passes when
    the state file carries σ_ref = 0.06: limit 3·√(0.06² + 0.06²) = 0.255°. The
    node must hand the gate the file's yaw_offset_std_deg (it did not, 2026-10-10)."""
    node, path = _node(tmp_path, dict(_ref(0.6), yaw_offset_std_deg=0.06))
    _, msg = _calibrate(node, _result(0.85, std=0.06))
    assert msg.success and 'limit ±0.255°' in msg.message
    assert json.loads(path.read_text())['yaw_offset_deg'] == pytest.approx(0.85)


def test_moved_axis_point_is_refused(tmp_path):
    node, _ = _node(tmp_path, _ref(0.6))
    _, msg = _calibrate(node, _result(0.6, pos=[POS[0] + 1.6, POS[1], POS[2]]))
    assert not msg.success and 'axis point' in msg.message


def test_template_residual_is_refused_even_with_bb_moved(tmp_path):
    node, _ = _node(tmp_path, _ref(0.6))
    node._on_set_parameters([SimpleNamespace(name='bb_moved', value=True)])
    _, msg = _calibrate(node, _result(0.6, tres=0.7))
    assert not msg.success and msg.message.startswith('TEMPLATE_RESIDUAL')


def test_bb_moved_override_is_one_shot_and_resets_the_reference(tmp_path):
    node, path = _node(tmp_path, _ref(0.6))
    node._on_set_parameters([SimpleNamespace(name='bb_moved', value=True)])
    _, msg = _calibrate(node, _result(3.0, pos=[POS[0] + 5, POS[1], POS[2]]))
    assert msg.success and 'bb_moved override' in msg.message
    assert json.loads(path.read_text())['yaw_offset_deg'] == pytest.approx(3.0)
    # consumed: the next disagreeing calibration is gated against the NEW reference
    _, msg = _calibrate(node, _result(0.6))
    assert not msg.success and 'CALIBRATION_INCONSISTENT' in msg.message
    _, msg = _calibrate(node, _result(3.05, pos=[POS[0] + 5, POS[1], POS[2]]))
    assert msg.success


def test_corrupt_state_file_refuses_unless_bb_moved(tmp_path):
    node, path = _node(tmp_path)
    path.write_text('{not json')
    _, msg = _calibrate(node, _result(0.6))
    assert not msg.success and msg.message.startswith('CALIBRATION_STATE_UNREADABLE')
    node._on_set_parameters([SimpleNamespace(name='bb_moved', value=True)])
    _, msg = _calibrate(node, _result(0.6))
    assert msg.success
    assert json.loads(path.read_text())['yaw_offset_deg'] == pytest.approx(0.6)
    assert not node._bb_moved_armed


def test_missing_template_refuses_the_calibration(tmp_path):
    node, _ = _node(tmp_path, template=False)
    solver, msg = _calibrate(node, _result(0.2))
    assert not msg.success and 'template not found' in msg.message
    solver.assert_not_called()


def test_an_anchor_result_is_never_published(tmp_path):
    node, _ = _node(tmp_path, _ref(0.6))
    r = _result(0.6)
    r.yaw_method = 'anchor'
    _, msg = _calibrate(node, r)
    assert not msg.success and 'anchor' in msg.message


def _bb_markers(stamp_ns, n=4):
    from jugglebot_interfaces.msg import MocapDataMulti, MocapDataSingle
    msg = MocapDataMulti()
    msg.stamp.sec = stamp_ns // 1_000_000_000
    msg.stamp.nanosec = stamp_ns % 1_000_000_000
    for i in range(n):
        s = MocapDataSingle()
        s.position.x, s.position.y, s.position.z = float(i), 0.0, 1.0
        msg.markers.append(s)
    return msg


def _axis_estimates(stamp_s, yaw_deg, with_yaw=True):
    from sensor_msgs.msg import JointState
    js = JointState()
    js.header.stamp.sec = int(stamp_s)
    js.header.stamp.nanosec = int(round((stamp_s % 1) * 1e9))
    js.name = ['bb_pitch', 'bb_hand'] + (['bb_yaw'] if with_yaw else [])
    js.position = [0.0, 0.0] + ([yaw_deg] if with_yaw else [])
    return js


def test_solver_gets_qtm_stamped_frames_once_each_and_heartbeat_yaw(tmp_path):
    import jugglebot.mocap_node as mn
    node, _ = _node(tmp_path, _ref(0.6))
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    t0 = 1_791_532_600_000_000_000
    for k in (0, 1, 1, 2):                      # frame 1 snapshotted twice
        node._accumulate_calibration_markers(_bb_markers(t0 + k * 5_000_000))
    node._accumulate_calibration_markers(_bb_markers(0))   # no stamp: arc fit only
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING, yaw=1.0))
    with patch.object(mn, 'run_calibration', return_value=_result(0.6)) as solver:
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    kw = solver.call_args.kwargs
    assert kw['template'] is node._marker_template
    assert [round(t, 3) for t, _ in kw['marker_frames']] == [
        round(t0 * 1e-9 + d, 3) for d in (0.0, 0.005, 0.010)]
    assert kw['yaw_source'] == 'heartbeat'
    assert kw['yaw_samples'] and all(len(s) == 2 for s in kw['yaw_samples'])
    assert len(kw['calibration_data'][0]) == 5



def test_the_first_calibrating_heartbeats_yaw_is_recorded(tmp_path):
    """The yaw sample was appended BEFORE the start edge was detected, so the
    first CALIBRATING heartbeat never reached the window (bag
    2026-10-09_23-49-07: one sample lost per sweep)."""
    node, _ = _node(tmp_path, _ref(0.6))
    node._on_bb_heartbeat(_hb(BallButlerStates.IDLE, yaw=359.6))
    assert node._calib_yaw_samples == [] and node._calib_yaw_readings == []
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING, yaw=359.7))
    assert node._calib_yaw_readings == [pytest.approx(359.7)]
    assert [y for _, y in node._calib_yaw_samples] == [pytest.approx(359.7)]
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING, yaw=3.0))
    assert [y for _, y in node._calib_yaw_samples] == [pytest.approx(359.7), pytest.approx(3.0)]

def _stamped_sweep(node, n):
    import jugglebot.mocap_node as mn
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._calib_data = {0: [np.zeros(3)]}
    for k in range(n):
        node._on_bb_axis_estimates(_axis_estimates(1000.0 + 0.01 * k, 36.0))
    with patch.object(mn, 'run_calibration', return_value=_result(0.6)) as solver:
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    return solver, node.pub_calibration.published[-1]


def test_auto_is_the_default_yaw_source_and_prefers_the_stamped_stream(tmp_path):
    """Default ``auto`` since 2026-10-10 12:26: bag 2026-10-10_12-26-22 (clean clock)
    replays at 0.021° SD stamped vs 0.038° heartbeat. With enough stamped samples
    the solver gets the stamped stream; with none it falls back to the heartbeat."""
    import jugglebot.mocap_node as mn
    node, _ = _node(tmp_path, _ref(0.6))
    assert node.get_parameter('bb_yaw_source').value == 'auto'
    solver, msg = _stamped_sweep(node, mn.MIN_STAMPED_YAW_SAMPLES + 50)
    assert solver.call_args.kwargs['yaw_source'] == 'stamped'
    assert len(solver.call_args.kwargs['yaw_samples']) == mn.MIN_STAMPED_YAW_SAMPLES + 50
    solver, _ = _stamped_sweep(node, 0)
    assert solver.call_args.kwargs['yaw_source'] == 'heartbeat'


def test_bb_yaw_source_stamped_uses_the_stamped_stream(tmp_path):
    import jugglebot.mocap_node as mn
    node, _ = _node(tmp_path, _ref(0.6))
    node._params['bb_yaw_source'] = 'stamped'
    solver, _ = _stamped_sweep(node, mn.MIN_STAMPED_YAW_SAMPLES)
    kw = solver.call_args.kwargs
    assert kw['yaw_source'] == 'stamped' and len(kw['yaw_samples']) == mn.MIN_STAMPED_YAW_SAMPLES


def test_bb_yaw_source_stamped_without_the_stream_is_refused_not_substituted(tmp_path):
    import jugglebot.mocap_node as mn
    node, _ = _node(tmp_path, _ref(0.6))
    node._params['bb_yaw_source'] = 'stamped'
    solver, msg = _stamped_sweep(node, mn.MIN_STAMPED_YAW_SAMPLES - 1)
    assert not solver.called
    assert not msg.success and 'BB_YAW_SOURCE_UNAVAILABLE' in msg.message


def test_an_invalid_bb_yaw_source_is_rejected_at_set_time_and_refused_at_calibration(tmp_path):
    import jugglebot.mocap_node as mn
    node, _ = _node(tmp_path, _ref(0.6))
    res = node._on_set_parameters([SimpleNamespace(name='bb_yaw_source', value='mocap')])
    assert not res.successful and 'heartbeat' in res.reason
    assert node._on_set_parameters([SimpleNamespace(name='bb_yaw_source', value='auto')]).successful
    node._params['bb_yaw_source'] = 'mocap'          # e.g. a launch-file typo
    solver, msg = _stamped_sweep(node, mn.MIN_STAMPED_YAW_SAMPLES)
    assert not solver.called
    assert not msg.success and 'BB_YAW_SOURCE_INVALID' in msg.message


def test_auto_prefers_the_stamped_yaw_stream_over_the_heartbeat(tmp_path):
    import jugglebot.mocap_node as mn
    node, _ = _node(tmp_path, _ref(0.6))
    node._params['bb_yaw_source'] = 'auto'
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._calib_data = {0: [np.zeros(3)]}
    for k in range(mn.MIN_STAMPED_YAW_SAMPLES):
        node._on_bb_axis_estimates(_axis_estimates(1000.0 + 0.01 * k, 36.0))
    with patch.object(mn, 'run_calibration', return_value=_result(0.6)) as solver:
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    kw = solver.call_args.kwargs
    assert kw['yaw_source'] == 'stamped'
    assert len(kw['yaw_samples']) == mn.MIN_STAMPED_YAW_SAMPLES
    assert kw['yaw_samples'][0] == (pytest.approx(1000.0), pytest.approx(36.0))


def test_auto_with_a_short_or_absent_stamped_stream_falls_back_to_the_heartbeat(tmp_path):
    import jugglebot.mocap_node as mn
    node, _ = _node(tmp_path, _ref(0.6))
    node._params['bb_yaw_source'] = 'auto'
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._calib_data = {0: [np.zeros(3)]}
    node._on_bb_axis_estimates(_axis_estimates(1000.0, 36.0))
    for k in range(200):                         # older firmware: two joints, no yaw
        node._on_bb_axis_estimates(_axis_estimates(1000.0 + 0.01 * k, 0.0, with_yaw=False))
    with patch.object(mn, 'run_calibration', return_value=_result(0.6)) as solver:
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    assert solver.call_args.kwargs['yaw_source'] == 'heartbeat'
