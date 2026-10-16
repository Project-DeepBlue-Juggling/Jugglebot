"""mocap_node and the BB base-marker frame (2026-10-10; advisory since 2026-10-16).

The sweep's own yaw / axis point are always the published output. The base
frame, when seen, yields kappa (BB-in-base yaw) for the epoch bookkeeping: a
kappa within the band of the current epoch extends it, a larger one opens (or
rejoins) an epoch and is flagged KAPPA_EXCURSION - never a refusal. No base
(or an unreadable pool) is an advisory line and the sweep is still accepted.
The solver is mocked (as in test_mocap_node_keep_last_good); the base frame's
unlabelled markers are synthetic frames fed to the window.
See logbook/2026-10-10-bb-base-marker-frame.md and
plans/active/bb-calibration-one-attempt.md.
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


def _node(tmp_path, *, n_seed=10, state=None, base_state=None):
    import jugglebot.mocap_node as mn
    iface = MagicMock()
    iface.is_receiving.return_value = True
    with patch.object(mn, 'MocapInterface', return_value=iface):
        node = mn.MocapNode()
    node._params['bb_calibration_state_file'] = str(tmp_path / 'last.json')
    node._params['bb_base_frame_state_file'] = str(tmp_path / 'base.json')
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
        yaw_estimate=SimpleNamespace(lag_s=0.008, template_residual_mm=0.25,
                                     yaw_source='heartbeat', n_frames=850,
                                     summary=lambda: 'sweep estimator: (summary)'))


def _sweep(node, yaw_deg, pos, base=None):
    """One CALIBRATING → IDLE sweep, solver mocked; ``base`` = (h, o) of the
    base frame in this window, or None (not seen)."""
    import jugglebot.mocap_node as mn
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._calib_data = {0: [np.zeros(3)]}
    node._calib_base_points = [] if base is None else _base_frames(*base)
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

def test_no_pose_source_or_moved_parameter_is_declared(tmp_path):
    node = _node(tmp_path)
    for name in ('bb_moved', 'bb_pose_source', 'bb_base_min_sweeps'):
        assert name not in node._params
    for name in ('bb_base_frame_file', 'bb_base_frame_state_file', 'bb_yaw_source'):
        assert name in node._params


def test_the_startup_state_line_is_logged_once_at_init_and_names_the_pool(tmp_path):
    import jugglebot.mocap_node as mn
    iface = MagicMock()
    with patch.object(mn, 'MocapInterface', return_value=iface), \
            patch.object(mn.MocapNode, '_log_startup_state') as startup:
        mn.MocapNode()
    startup.assert_called_once_with()
    node = _node(tmp_path)                            # 10 seed records -> epoch 0
    lines = _rec(node)
    node._log_startup_state()
    info = [m for lv, m in lines if lv == 'info']
    assert len(info) == 1 and info[0].startswith('BB calibration state: template ')
    assert 'base model on' in info[0] and 'pool 10 records in 1 epochs (current K0 median 179.5' in info[0]
    assert 'n 10)' in info[0] and 'last accepted none' in info[0] and 'yaw source auto' in info[0]


def test_the_committed_resource_loads_in_the_node(tmp_path):
    import jugglebot.mocap_node as mn
    iface = MagicMock()
    with patch.object(mn, 'MocapInterface', return_value=iface):
        node = mn.MocapNode()
    assert node._base_model is not None and len(node._base_model.records) >= 5
    assert node._base_monitor is not None


# ── the sweep is published; the base frame is advisory ──────────────────────

def test_a_consistent_sweep_is_published_and_pooled_in_the_current_epoch(tmp_path):
    node = _node(tmp_path)
    yaw, pos = _bb_world()
    msg = _sweep(node, yaw + 0.02, pos, base=(H0, O0))
    assert msg.success
    assert math.degrees(msg.yaw_offset_rad) == pytest.approx(yaw + 0.02)
    base = json.loads((tmp_path / 'base.json').read_text())
    recs = base['records']
    assert len(recs) == 11 and recs[-1]['kappa_deg'] == pytest.approx(KAPPA + 0.02, abs=1e-3)
    assert recs[-1]['epoch'] == 0 and len(base['epochs']) == 1
    st = json.loads((tmp_path / 'last.json').read_text())
    assert st['pose_source'] == 'sweep' and st['kappa_deg'] == pytest.approx(KAPPA + 0.02, abs=1e-3)
    assert st['epoch'] == 0 and st['advisory_flags'] == []
    assert 'kappa 179.' in msg.message and '(epoch K0' in msg.message and 'ADVISORY' not in msg.message


def test_a_kappa_031_off_the_current_epoch_is_accepted_and_opens_a_new_epoch(tmp_path):
    """Was BB_IN_BASE_MOVED (a refusal). The 2026-10-14 day-to-day step is 0.25-0.31 deg."""
    node = _node(tmp_path)
    lines = _rec(node)
    yaw, pos = _bb_world()
    msg = _sweep(node, yaw + 0.31, pos, base=(H0, O0))
    assert msg.success
    assert math.degrees(msg.yaw_offset_rad) == pytest.approx(yaw + 0.31)   # the sweep's own value
    assert 'KAPPA_EXCURSION' not in msg.message          # the flag name is persisted, the LINE is shown
    assert 'opened epoch K1' in msg.message and 'ADVISORY' in msg.message
    base = json.loads((tmp_path / 'base.json').read_text())
    assert [e['id'] for e in base['epochs']] == [0, 1] and len(base['records']) == 11
    assert base['records'][-1]['epoch'] == 1 and base['epochs'][1]['n'] == 1
    st = json.loads((tmp_path / 'last.json').read_text())
    assert st['epoch'] == 1 and 'KAPPA_EXCURSION' in st['advisory_flags']
    outcome = [m for lv, m in lines if lv == 'warn' and m.startswith('BB calibrated: ')]
    assert len(outcome) == 1 and not [m for lv, m in lines if lv == 'info' and 'BB calibrated' in m]


def test_a_qtm_frame_shift_is_accepted_and_reported_not_refused(tmp_path):
    yaw, pos = _bb_world()
    node = _node(tmp_path, state={'yaw_offset_deg': yaw, 'yaw_offset_std_deg': 0.07,
                                  'position_mm': list(pos), 'accepted_at': '2026-10-10T01:26:55'})
    yaw2, pos2 = _bb_world(h=H0 + 0.4)                 # QTM turned 0.4 deg: BB-in-base unchanged
    msg = _sweep(node, yaw2, pos2, base=(H0 + 0.4, O0))
    assert msg.success and 'QTM frame moved since the last sweep' in msg.message
    assert math.degrees(msg.yaw_offset_rad) == pytest.approx(yaw2)
    flags = json.loads((tmp_path / 'last.json').read_text())['advisory_flags']
    assert 'QTM_FRAME_SHIFT' in flags and 'YAW_SHIFT' not in flags   # world deltas not comparable
    assert 'KAPPA_EXCURSION' not in flags


def test_a_window_without_the_base_is_accepted_and_not_pooled(tmp_path):
    node = _node(tmp_path)
    yaw, pos = _bb_world()
    msg = _sweep(node, yaw, pos)
    assert msg.success and 'shelf base not seen: kappa not pooled' in msg.message
    assert not (tmp_path / 'base.json').exists()
    st = json.loads((tmp_path / 'last.json').read_text())
    assert st['base_frame'] is None and st['kappa_deg'] is None and st['epoch'] is None
    assert 'BASE_NOT_SEEN' in st['advisory_flags']


def test_without_a_base_model_the_sweep_is_still_accepted(tmp_path):
    node = _node(tmp_path)
    node._base_model, node._base_model_error = None, 'BB base frame not found (x.json)'
    yaw, pos = _bb_world()
    msg = _sweep(node, yaw, pos, base=(H0, O0))
    assert msg.success and 'kappa not pooled' in msg.message


def test_an_unreadable_base_state_is_an_advisory_line_and_a_fresh_file_is_written(tmp_path):
    node = _node(tmp_path)
    (tmp_path / 'base.json').write_text('{not json')
    yaw, pos = _bb_world()
    msg = _sweep(node, yaw, pos, base=(H0, O0))
    assert msg.success and 'history unavailable: base.json: JSONDecodeError' in msg.message
    fresh = json.loads((tmp_path / 'base.json').read_text())
    assert fresh['schema'] == 2 and len(fresh['records']) == 11       # seed + this sweep
    assert list(tmp_path.glob('base.json.*.bak')), 'the corrupt file was not backed up'


def test_a_window_without_the_base_uses_the_running_estimate(tmp_path):
    node = _node(tmp_path)
    for t, P in _base_frames(n=40):
        node._base_monitor.update(t, P)
    node._base_monitor_last_t = 100.4
    yaw, pos = _bb_world()
    import jugglebot.mocap_node as mn
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._calib_data = {0: [np.zeros(3)]}
    node._calib_base_points = [(101.0, np.zeros((0, 3)))] * 5
    with patch.object(mn, 'run_calibration', return_value=_result(yaw, pos)):
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    assert node.pub_calibration_attempt.published[-1].success


# ── Persistence and gauge ───────────────────────────────────────────────────

def test_a_fresh_node_pools_from_the_state_file_not_the_seed(tmp_path):
    node = _node(tmp_path, n_seed=10)
    yaw, pos = _bb_world()
    _sweep(node, yaw, pos, base=(H0, O0))
    node2 = _node(tmp_path, n_seed=2)                  # a different seed
    state, err = node2._load_state()
    assert not err and len(state['records']) == 11
    assert state['base_ref']['heading_deg'] == pytest.approx(H0, abs=0.01)


# ── Collection, the running estimate and its report ─────────────────────────

def test_unlabelled_markers_feed_the_window_and_the_running_estimate(tmp_path):
    node = _node(tmp_path)
    frames = _base_frames(n=500)
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    for t, P in frames:
        U = np.c_[P, np.zeros(len(P))]
        node._feed_base_frame(U, int(round(t * 1e9)))
        node._feed_base_frame(U, int(round(t * 1e9)))        # a re-published frame: once
    assert len(node._calib_base_points) == 500
    assert node._base_monitor._frames == 25                  # 5 s at BASE_MONITOR_PERIOD_S
    lines = _rec(node)
    node._report_base_frame_once()
    node._report_base_frame_once()
    seen = [m for lv, m in lines if lv == 'info' and m.startswith('BB base frame seen')]
    assert len(seen) == 1 and 'QTM frame shift' in seen[0]
    node._end_calibration()
    assert node._calib_base_points == []


# ── Label-agnostic collection (2026-10-10 14:10) ────────────────────────────

def test_labelled_base_markers_reach_the_window_and_bbs_own_do_not(tmp_path):
    """QTM's re-enabled Base / Catching Cone bodies labelled two of the four base
    markers (logbook 2026-10-10-bb-base-frame-robustness): every labelled marker
    but BB's own feeds the window and the running estimate."""
    node = _node(tmp_path)
    P = NOM @ _Rz(H0).T + O0
    iface = node.mocap
    iface.is_receiving.return_value = True
    from jugglebot.mocap_interface import MocapFrame
    unlabelled = [tuple(P[i]) + (0.0,) for i in (0, 3)]
    labelled = [
        ('Catching Cone - 4', *P[1], 0.1), ('Catching Cone - 5', *P[2], 0.1),
        ('Ball Butler - 1', 0.0, 0.0, 0.0, 0.1), ('Ball Butler - 2', 10.0, 0.0, 0.0, 0.1)]
    nan_bb = ((math.nan,) * 4,) * 7
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    for k in range(60):
        iface.drain_frames.return_value = [MocapFrame(
            k, 0, int((100.0 + 0.01 * k) * 1e9), None, True, labelled, unlabelled, (), nan_bb)]
        node._publish_mocap_data()
    assert len(node._calib_base_points) == 60
    pts = node._calib_base_points[-1][1]
    assert pts.shape == (4, 3) and np.allclose(np.sort(pts, axis=0), np.sort(P, axis=0))
    pose = bf.estimate_base_pose(node._calib_base_points, NOM, min_frames=50)
    assert pose.heading_deg == pytest.approx(H0, abs=1e-6)


# ── bb/base_frame_pose publisher (magazine throw) ────────────────────────────

def test_base_frame_pose_is_published_in_mm_with_the_rotation_quaternion(tmp_path):
    node = _node(tmp_path)
    node.pub_base_frame_pose = MagicMock()
    node._publish_base_frame_pose()                       # no pose yet: silent
    node.pub_base_frame_pose.publish.assert_not_called()
    for t, pts in _base_frames():
        node._base_monitor.update(t, pts)
    node._publish_base_frame_pose()
    node.pub_base_frame_pose.publish.assert_called_once()
    msg = node.pub_base_frame_pose.publish.call_args[0][0]
    pose = node._base_monitor.pose()
    assert np.allclose([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                       pose.origin_mm) and abs(msg.pose.position.x - O0[0]) < 1.0   # mm
    q = msg.pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    assert np.allclose(R, pose.R, atol=1e-9)
    assert (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) == pytest.approx(pose.t_span_s[1])


def test_base_frame_pose_publisher_is_a_1hz_timer_and_survives_no_monitor(tmp_path):
    node = _node(tmp_path)
    node._base_monitor = None
    node.pub_base_frame_pose = MagicMock()
    node._publish_base_frame_pose()
    node.pub_base_frame_pose.publish.assert_not_called()
