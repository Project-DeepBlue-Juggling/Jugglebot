"""mocap_node's yaw-offset consistency gate and its bb_moved override (2026-10-09).

A calibration whose constellation yaw offset disagrees with the last accepted
one by more than max(3σ_repeatability, 0.15°) is published as a FAILURE unless
the operator armed ``bb_moved``; the accepted result is persisted so the gate
survives restarts; with nothing persisted the template's pinned gauge (the
frame the aim correction was fitted in) is the reference. See
``logbook/2026-10-09-bb-constellation-yaw-offset.md``.
"""
from __future__ import annotations

import json
import math
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from jugglebot.protocol_config import BallButlerStates


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
    if not template:
        node._marker_template = None
        node._marker_template_error = 'BB marker template not found (x.json)'
    return node, path


def _hb(state, yaw=0.0):
    from jugglebot_interfaces.msg import BallButlerHeartbeat
    msg = BallButlerHeartbeat()
    msg.state = int(state)
    msg.yaw_deg = yaw
    return msg


def _result(yaw_deg, stat=0.13):
    return SimpleNamespace(
        bb_position_mm=np.array([-975.6, -389.3, 1734.9]),
        axis_direction=np.array([0.0, 0.0, 1.0]), axis_tilt_deg=0.8,
        yaw_offset_rad=math.radians(yaw_deg), yaw_offset_std_deg=math.hypot(stat, 0.1),
        yaw_span_deg=125.0, marker_metrics={}, yaw_method='constellation',
        anchor_yaw_offset_rad=math.radians(0.9),
        yaw_estimate=SimpleNamespace(n_holds=1, holds=[SimpleNamespace(n_matched=3)],
                                     stat_std_deg=stat, template_std_deg=0.1))


def _calibrate(node, result):
    import jugglebot.mocap_node as mn
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._calib_data = {0: [np.zeros(3)]}
    with patch.object(mn, 'run_calibration', return_value=result) as solver:
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    return solver, node.pub_calibration.published[-1]


def test_node_loads_the_shipped_template():
    import jugglebot.mocap_node as mn
    with patch.object(mn, 'MocapInterface', return_value=MagicMock()):
        node = mn.MocapNode()
    assert node._marker_template is not None
    assert node._marker_template.reference_yaw_offset_deg == pytest.approx(0.208)


def test_solver_gets_the_constellation_inputs(tmp_path):
    node, _ = _node(tmp_path)
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING, yaw=1.0))
    solver, _ = _calibrate(node, _result(0.25))
    kw = solver.call_args.kwargs
    assert kw['template'] is node._marker_template
    assert kw['yaw_samples'] and all(len(s) == 2 for s in kw['yaw_samples'])
    assert 'marker_frames' in kw


def test_consistent_calibration_is_accepted_and_persisted(tmp_path):
    node, path = _node(tmp_path, {'yaw_offset_deg': 0.208, 'accepted_at': 't0'})
    _, msg = _calibrate(node, _result(0.30))
    assert msg.success
    assert 'constellation yaw from 1 hold(s)' in msg.message and 'gate' in msg.message
    assert json.loads(path.read_text())['yaw_offset_deg'] == pytest.approx(0.30)


def test_inconsistent_calibration_is_refused_and_not_persisted(tmp_path):
    node, path = _node(tmp_path, {'yaw_offset_deg': 0.208, 'accepted_at': 't0'})
    _, msg = _calibrate(node, _result(0.681))
    assert not msg.success
    assert msg.message.startswith('YAW_OFFSET_INCONSISTENT')
    assert json.loads(path.read_text())['yaw_offset_deg'] == pytest.approx(0.208)


def test_without_a_persisted_calibration_the_template_gauge_is_the_reference(tmp_path):
    node, path = _node(tmp_path)
    _, msg = _calibrate(node, _result(0.95))
    assert not msg.success and 'template-pinned gauge' in msg.message
    assert not path.exists()


def test_bb_moved_override_is_one_shot(tmp_path):
    node, path = _node(tmp_path, {'yaw_offset_deg': 0.208, 'accepted_at': 't0'})
    from rcl_interfaces.msg import SetParametersResult  # noqa: F401  (mock present)
    node._on_set_parameters([SimpleNamespace(name='bb_moved', value=True)])
    _, msg = _calibrate(node, _result(3.0))
    assert msg.success and 'bb_moved override' in msg.message
    assert json.loads(path.read_text())['yaw_offset_deg'] == pytest.approx(3.0)
    # consumed: the next disagreeing calibration is gated again
    _, msg = _calibrate(node, _result(0.208))
    assert not msg.success and 'YAW_OFFSET_INCONSISTENT' in msg.message


def test_missing_template_refuses_the_calibration(tmp_path):
    node, _ = _node(tmp_path, template=False)
    solver, msg = _calibrate(node, _result(0.2))
    assert not msg.success and 'template not found' in msg.message
    solver.assert_not_called()


def test_an_anchor_result_is_never_published(tmp_path):
    node, _ = _node(tmp_path, {'yaw_offset_deg': 0.208, 'accepted_at': 't0'})
    r = _result(0.2)
    r.yaw_method = 'anchor'
    _, msg = _calibrate(node, r)
    assert not msg.success and 'anchor' in msg.message


def test_corrupt_state_file_falls_back_to_the_template_gauge(tmp_path):
    node, path = _node(tmp_path)
    path.write_text('{not json')
    _, msg = _calibrate(node, _result(0.25))
    assert msg.success and 'template-pinned gauge' in msg.message
