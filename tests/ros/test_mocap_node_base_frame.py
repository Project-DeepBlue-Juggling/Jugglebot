"""mocap_node and the BB base-marker frame (2026-10-10).

``bb_pose_source``: ``sweep`` (default; world gate unchanged, base frame a
diagnostic + pool input), ``base_frame`` (published pose = this window's base
pose ∘ pooled BB-in-base constants, gated in the base frame), ``auto`` (base
frame once the pool has ``bb_base_min_sweeps`` and the base is seen).
The solver is mocked (as in test_mocap_node_keep_last_good); the base frame's
unlabelled markers are synthetic frames fed to the window.
See logbook/2026-10-10-bb-base-marker-frame.md.
"""
from __future__ import annotations

import json
import math
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from jugglebot.protocol_config import BallButlerStates
from jugglebot import bb_base_frame as bf

NOM = bf.nominal_base_template()
H0 = -179.05                         # base heading (deg) at the reference sitting
O0 = np.array([-801.69, -236.79, 1659.98])
KAPPA = 179.49                       # BB-in-base yaw constant (deg)
P_B = np.array([177.54, 148.86, 76.71])
PIN = 0.208


def _Rz(h_deg):
    h = math.radians(h_deg)
    return np.array([[math.cos(h), -math.sin(h), 0], [math.sin(h), math.cos(h), 0], [0, 0, 1]])


def _bb_world(h=H0, o=O0, kappa=KAPPA, p_b=P_B):
    """BB's world (yaw deg, position) for a base at heading h / origin o."""
    return bf.wrap_deg(h + kappa), o + _Rz(h) @ p_b


def _base_frames(h=H0, o=O0, n=200, extra=6, seed=0):
    rng = np.random.default_rng(seed)
    out = []
    for k in range(n):
        P = NOM @ _Rz(h).T + o + rng.normal(0, 0.02, (4, 3))
        X = rng.uniform([-1500, -900, 0], [500, 900, 2400], (extra, 3))
        out.append((100.0 + 0.01 * k, np.vstack([X[:3], P, X[3:]])))
    return out


def _records(n, kappa=KAPPA, sd=0.04, seed=1):
    rng = np.random.default_rng(seed)
    ks = kappa + rng.normal(0, sd, n)
    ks += kappa - ks.mean()                     # pooled mean exactly kappa
    return [bf.KappaRecord(kappa_deg=float(k), sigma_deg=0.07, axis_point_b_mm=tuple(P_B),
                           pin_deg=PIN, yaw_offset_deg=float(bf.wrap_deg(H0 + k)),
                           base_heading_deg=H0) for k in ks]


def _node(tmp_path, *, pose_source='sweep', n_seed=10, state=None, base_state=None):
    import jugglebot.mocap_node as mn
    iface = MagicMock()
    iface.is_receiving.return_value = True
    with patch.object(mn, 'MocapInterface', return_value=iface):
        node = mn.MocapNode()
    node._params['bb_calibration_state_file'] = str(tmp_path / 'last.json')
    node._params['bb_base_frame_state_file'] = str(tmp_path / 'base.json')
    node._params['bb_pose_source'] = pose_source
    if state is not None:
        (tmp_path / 'last.json').write_text(json.dumps(state))
    if base_state is not None:
        (tmp_path / 'base.json').write_text(json.dumps(base_state))
    node._base_model = bf.BaseFrameModel(
        template_mm=NOM, nominal_mm=NOM, records=_records(n_seed) if n_seed else [],
        base_ref={'origin_mm': list(O0), 'heading_deg': H0, 'accepted_at': 'build'})
    node._base_monitor = bf.BaseFrameMonitor(NOM)
    node._marker_template.pinned_yaw_offset_deg = PIN
    return node


def _hb(state):
    from jugglebot_interfaces.msg import BallButlerHeartbeat
    msg = BallButlerHeartbeat()
    msg.state = int(state)
    msg.yaw_deg = 0.0
    return msg


def _result(yaw_deg, pos, std=0.07):
    return SimpleNamespace(
        bb_position_mm=np.array(pos, dtype=float), arc_position_mm=np.array(pos, dtype=float),
        axis_direction=np.array([0.0, 0.0, 1.0]), axis_tilt_deg=0.9,
        yaw_offset_rad=math.radians(yaw_deg), yaw_offset_std_deg=std,
        yaw_span_deg=123.0, marker_metrics={}, yaw_method='constellation',
        anchor_yaw_offset_rad=None,
        yaw_estimate=SimpleNamespace(lag_s=0.078, template_residual_mm=0.25,
                                     yaw_source='heartbeat', n_frames=850,
                                     summary=lambda: 'sweep estimator: (summary)'))


def _sweep(node, yaw_deg, pos, base=None):
    """One CALIBRATING → IDLE sweep, solver mocked; ``base`` = (h, o) of the
    base frame in this window, or None (not seen)."""
    import jugglebot.mocap_node as mn
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._calib_data = {0: [np.zeros(3)]}
    node._calib_unlabelled = [] if base is None else _base_frames(*base)
    with patch.object(mn, 'run_calibration', return_value=_result(yaw_deg, pos)):
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    return node.pub_calibration_attempt.published[-1]


def _rec(node):
    lines = []

    class L:
        def __getattr__(self, level):
            return lambda m, *a, **k: lines.append((level, m))
    node._logger = L()
    return lines


def _dir(tmp_path, name):
    d = tmp_path / name
    d.mkdir()
    return d


def _base_records(tmp_path):
    return json.loads((tmp_path / 'base.json').read_text())['records']


# ── Parameters ──────────────────────────────────────────────────────────────

def test_default_pose_source_is_sweep_and_an_invalid_value_is_refused(tmp_path):
    import jugglebot.mocap_node as mn
    assert mn.DEFAULT_BB_POSE_SOURCE == 'sweep'
    node = _node(tmp_path)
    node._params.pop('bb_pose_source')
    node.declare_parameter('bb_pose_source', mn.DEFAULT_BB_POSE_SOURCE)
    assert node.get_parameter('bb_pose_source').value == 'sweep'
    bad = node._on_set_parameters([SimpleNamespace(name='bb_pose_source', value='landings')])
    assert not bad.successful and 'bb_pose_source' in bad.reason
    ok = node._on_set_parameters([SimpleNamespace(name='bb_pose_source', value='auto')])
    assert ok.successful


def test_the_committed_resource_loads_in_the_node(tmp_path):
    import jugglebot.mocap_node as mn
    iface = MagicMock()
    with patch.object(mn, 'MocapInterface', return_value=iface):
        node = mn.MocapNode()
    assert node._base_model is not None and len(node._base_model.records) >= 5
    assert node._base_monitor is not None


# ── sweep (default) ─────────────────────────────────────────────────────────

def test_sweep_mode_publishes_the_sweep_and_pools_a_consistent_sweep(tmp_path):
    node = _node(tmp_path)
    yaw, pos = _bb_world()
    msg = _sweep(node, yaw + 0.02, pos, base=(H0, O0))
    assert msg.success
    assert math.degrees(msg.yaw_offset_rad) == pytest.approx(yaw + 0.02)
    recs = _base_records(tmp_path)
    assert len(recs) == 11 and recs[-1]['kappa_deg'] == pytest.approx(KAPPA + 0.02, abs=1e-3)
    st = json.loads((tmp_path / 'last.json').read_text())
    assert st['pose_source'] == 'sweep' and st['kappa_deg'] == pytest.approx(KAPPA + 0.02, abs=1e-3)


def test_sweep_mode_without_the_base_is_todays_behaviour(tmp_path):
    node = _node(tmp_path)
    yaw, pos = _bb_world()
    assert _sweep(node, yaw, pos).success
    assert not (tmp_path / 'base.json').exists()
    st = json.loads((tmp_path / 'last.json').read_text())
    assert st['base_frame'] is None and st['kappa_deg'] is None


def test_sweep_mode_world_gate_still_refuses_a_qtm_shift_and_names_it(tmp_path):
    yaw, pos = _bb_world()
    node = _node(tmp_path, state={'yaw_offset_deg': yaw, 'yaw_offset_std_deg': 0.07,
                                  'position_mm': list(pos), 'accepted_at': '2026-10-10T01:26:55'})
    yaw2, pos2 = _bb_world(h=H0 + 0.4)                 # QTM turned 0.4°: BB-in-base unchanged
    msg = _sweep(node, yaw2, pos2, base=(H0 + 0.4, O0))
    assert not msg.success and msg.message.startswith('CALIBRATION_INCONSISTENT')
    assert msg.message.endswith('base frame: BB unmoved, QTM frame shifted')
    assert len(msg.message) <= 240


def test_sweep_mode_does_not_pool_a_sweep_the_base_frame_disagrees_with(tmp_path):
    node = _node(tmp_path)
    lines = _rec(node)
    yaw, pos = _bb_world()
    assert _sweep(node, yaw + 0.4, pos, base=(H0, O0)).success     # world gate: no reference
    assert not (tmp_path / 'base.json').exists()
    assert any('BB_IN_BASE_MOVED' in m and 'not pooled' in m for lv, m in lines if lv == 'warn')


# ── base_frame ──────────────────────────────────────────────────────────────

def test_base_frame_mode_publishes_base_pose_composed_with_the_pool(tmp_path):
    node = _node(tmp_path, pose_source='base_frame')
    yaw, pos = _bb_world()
    msg = _sweep(node, yaw + 0.05, pos + [0.1, 0, 0], base=(H0, O0))
    assert msg.success
    # pool = 10 records at mean KAPPA + this sweep at KAPPA + 0.05
    expect = bf.wrap_deg(H0 + KAPPA + 0.05 / 11)
    assert math.degrees(msg.yaw_offset_rad) == pytest.approx(expect, abs=2e-3)
    assert msg.yaw_offset_std_deg < 0.07
    assert 'pose source base_frame' in msg.message and 'vs sweep' in msg.message
    st = json.loads((tmp_path / 'last.json').read_text())
    assert st['pose_source'] == 'base_frame' and st['method'] == 'base_frame'
    assert st['sweep_yaw_offset_deg'] == pytest.approx(yaw + 0.05)
    base = json.loads((tmp_path / 'base.json').read_text())
    assert len(base['records']) == 11
    assert base['base_ref']['heading_deg'] == pytest.approx(H0, abs=0.01)
    assert node.pub_calibration.published[-1] is msg


def test_base_frame_mode_accepts_a_qtm_frame_shift_and_reports_it(tmp_path):
    yaw, pos = _bb_world()
    node = _node(tmp_path, pose_source='base_frame',
                 state={'yaw_offset_deg': yaw, 'yaw_offset_std_deg': 0.07,
                        'position_mm': list(pos), 'accepted_at': '2026-10-10T01:26:55'})
    lines = _rec(node)
    shift = np.array([1.0, -0.6, 0.2])
    yaw2, pos2 = _bb_world(h=H0 + 0.4, o=O0 + shift)
    msg = _sweep(node, yaw2, pos2, base=(H0 + 0.4, O0 + shift))
    assert msg.success
    assert math.degrees(msg.yaw_offset_rad) == pytest.approx(yaw2, abs=2e-3)
    assert np.allclose([msg.position_mm.x, msg.position_mm.y, msg.position_mm.z], pos2, atol=0.05)
    assert any('QTM frame shift' in m for lv, m in lines if lv == 'warn')


def test_base_frame_mode_refuses_a_bb_move_on_the_shelf_unless_bb_moved(tmp_path):
    node = _node(tmp_path, pose_source='base_frame')
    yaw, pos = _bb_world()
    msg = _sweep(node, yaw + 0.4, pos, base=(H0, O0))
    assert not msg.success and msg.message.startswith('BB_IN_BASE_MOVED')
    assert not (tmp_path / 'base.json').exists() and not (tmp_path / 'last.json').exists()
    node._on_set_parameters([SimpleNamespace(name='bb_moved', value=True)])
    node._params['bb_moved'] = True
    msg = _sweep(node, yaw + 0.4, pos, base=(H0, O0))
    assert msg.success and 'bb_moved override' in msg.message
    assert len(_base_records(tmp_path)) == 1                 # a new pool
    assert math.degrees(msg.yaw_offset_rad) == pytest.approx(yaw + 0.4, abs=2e-3)
    assert node._bb_moved_armed is False                     # one-shot consumed
    assert not _sweep(node, yaw, pos, base=(H0, O0)).success  # the old κ is gone


def test_base_frame_mode_refuses_when_the_base_is_not_seen(tmp_path):
    node = _node(tmp_path, pose_source='base_frame')
    yaw, pos = _bb_world()
    msg = _sweep(node, yaw, pos)
    assert not msg.success and msg.message.startswith('BASE_FRAME_NOT_SEEN')
    assert 'set it to auto or sweep' in msg.message and len(msg.message) <= 240


def test_base_frame_mode_refuses_without_a_model(tmp_path):
    node = _node(tmp_path, pose_source='base_frame')
    node._base_model, node._base_model_error = None, 'BB base frame not found (x.json)'
    yaw, pos = _bb_world()
    msg = _sweep(node, yaw, pos, base=(H0, O0))
    assert not msg.success and msg.message.startswith('BASE_FRAME_MODEL_MISSING')


def test_base_frame_mode_refuses_an_unreadable_base_state(tmp_path):
    node = _node(tmp_path, pose_source='base_frame')
    (tmp_path / 'base.json').write_text('{not json')
    yaw, pos = _bb_world()
    msg = _sweep(node, yaw, pos, base=(H0, O0))
    assert not msg.success and msg.message.startswith('BASE_FRAME_STATE_UNREADABLE')


def test_a_window_without_the_base_uses_the_running_estimate(tmp_path):
    node = _node(tmp_path, pose_source='base_frame')
    for t, P in _base_frames(n=40):
        node._base_monitor.update(t, P)
    node._base_monitor_last_t = 100.4
    yaw, pos = _bb_world()
    import jugglebot.mocap_node as mn
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._calib_data = {0: [np.zeros(3)]}
    node._calib_unlabelled = [(101.0, np.zeros((0, 3)))] * 5
    with patch.object(mn, 'run_calibration', return_value=_result(yaw, pos)):
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    assert node.pub_calibration_attempt.published[-1].success


# ── auto ────────────────────────────────────────────────────────────────────

def test_auto_uses_the_base_frame_only_with_enough_pooled_sweeps_and_the_base_seen(tmp_path):
    yaw, pos = _bb_world()
    node = _node(_dir(tmp_path, 'a'), pose_source='auto', n_seed=3)
    msg = _sweep(node, yaw + 0.05, pos, base=(H0, O0))
    assert msg.success and 'pose source base_frame' not in msg.message     # pool 3 < 5: sweep
    msg = _sweep(node, yaw + 0.05, pos, base=(H0, O0))                     # pool 4 → still sweep
    assert 'pose source base_frame' not in msg.message
    msg = _sweep(node, yaw + 0.05, pos, base=(H0, O0))                     # pool 5 → base frame
    assert 'pose source base_frame' in msg.message
    msg = _sweep(node, yaw, pos)                                           # base unseen → sweep
    assert msg.success and 'pose source base_frame' not in msg.message
    assert math.degrees(msg.yaw_offset_rad) == pytest.approx(yaw)


# ── Persistence and gauge ───────────────────────────────────────────────────

def test_a_fresh_node_pools_from_the_state_file_not_the_seed(tmp_path):
    node = _node(tmp_path, pose_source='base_frame', n_seed=10)
    yaw, pos = _bb_world()
    _sweep(node, yaw, pos, base=(H0, O0))
    node2 = _node(tmp_path, pose_source='base_frame', n_seed=2)   # a different seed
    view = node2._base_frame_view(_result(yaw, pos))
    assert len(view['records']) == 11 and view['pooled'].n == 11
    assert view['base_ref']['heading_deg'] == pytest.approx(H0, abs=0.01)


def test_a_re_pinned_gauge_moves_the_base_frame_offset_by_the_same_delta(tmp_path):
    yaw, pos = _bb_world()
    a = _node(_dir(tmp_path, 'a'), pose_source='base_frame')
    b = _node(_dir(tmp_path, 'b'), pose_source='base_frame')
    b._marker_template.pinned_yaw_offset_deg = PIN + 0.06    # settle_yaw_gauge: δ = +0.06
    ya = math.degrees(_sweep(a, yaw, pos, base=(H0, O0)).yaw_offset_rad)
    # the sweep itself would also read +0.06 under the new pin
    yb = math.degrees(_sweep(b, yaw + 0.06, pos, base=(H0, O0)).yaw_offset_rad)
    assert yb - ya == pytest.approx(0.06, abs=1e-6)


# ── Collection, the running estimate and its report ─────────────────────────

def test_unlabelled_markers_feed_the_window_and_the_running_estimate(tmp_path):
    node = _node(tmp_path)
    frames = _base_frames(n=500)
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    for t, P in frames:
        U = np.c_[P, np.zeros(len(P))]
        node._feed_base_frame(U, int(round(t * 1e9)))
        node._feed_base_frame(U, int(round(t * 1e9)))        # a re-published frame: once
    assert len(node._calib_unlabelled) == 500
    assert node._base_monitor._frames == 25                  # 5 s at BASE_MONITOR_PERIOD_S
    lines = _rec(node)
    node._report_base_frame_once()
    node._report_base_frame_once()
    seen = [m for lv, m in lines if lv == 'info' and m.startswith('BB base frame seen')]
    assert len(seen) == 1 and 'QTM frame shift' in seen[0]
    node._end_calibration()
    assert node._calib_unlabelled == []
