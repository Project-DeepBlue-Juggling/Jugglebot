"""Keep-last-good BB calibration and the short failure line (2026-10-10).

Owner's requirement: once a calibration has succeeded, a later FAILED sweep
must not wipe it — the failed result is discarded, and the calibration is
updated by successful results only. And the failure line is one short line
(the 2026-10-09 live one was ~900 characters).

Mechanism (mocap_node): ``bb/calibration_result`` (latched) is the calibration
IN FORCE — a failure lands there only while the process has never succeeded;
``bb/calibration_attempt`` (latched) carries every sweep's outcome. Failure
text is bounded by ``MAX_CALIBRATION_FAILURE_CHARS`` at one enforcement point
(``_publish_calibration_failure``), and every reason the pipeline produces fits
it without truncation.

Under the mocked rclpy a latched topic's replay to a late subscriber is not
simulated; what TRANSIENT_LOCAL depth 1 replays is the LAST message published
on it, so "a restarted consumer gets the success" is asserted as: the last
message on ``pub_calibration`` is the success, the publisher is
TRANSIENT_LOCAL depth 1, and a FRESH ball_butler_node fed that message aims
from it.

See ``logbook/2026-10-10-bb-calibration-keep-last-good.md``.
"""
from __future__ import annotations

import json
import math
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from jugglebot.protocol_config import BallButlerStates
from jugglebot.bb_calibration import check_calibration_consistency

POS = [-975.75, -389.5, 1735.0]

#: A realistic estimator summary (the 2026-10-09 shape) — must reach DEBUG
#: only, never the published failure text.
LONG_SUMMARY = (
    'sweep estimator: 1187 frames (902 moving, 13 rejected, 5-7 markers, median fit '
    '0.21 mm, template residual 0.31 mm), yaw source heartbeat lag 83.2 ms (12 % one '
    'period stale) over 412 moving samples (residual 0.08° RMS), σ 0.052° (formal '
    '0.041° ⊕ repeatability 0.031°); arc-fit cross-check (-1019.40, -435.90, 1738.00) '
    'mm, 1.52 mm from the body-model axis point in x/y')


# ── Harness ─────────────────────────────────────────────────────────────────

def _node(tmp_path, state=None, state_name='last_accepted.json'):
    import jugglebot.mocap_node as mn
    iface = MagicMock()
    iface.is_receiving.return_value = True
    with patch.object(mn, 'MocapInterface', return_value=iface):
        node = mn.MocapNode()
    path = tmp_path / state_name
    if state is not None:
        path.write_text(json.dumps(state) if isinstance(state, dict) else state)
    node._params['bb_calibration_state_file'] = str(path)
    # These tests pin the WORLD gate: pose source sweep, and a private base
    # pool (never the live ~/bb_calibration_sessions state). Under the default
    # auto with the seeded pool, a window without the base leads its refusal
    # with BASE_FRAME_NOT_SEEN (tests/ros/test_mocap_node_base_frame.py).
    node._params['bb_pose_source'] = 'sweep'
    node._params['bb_base_frame_state_file'] = str(tmp_path / 'base_state.json')
    return node, path


def _ref(yaw=0.926, pos=POS):
    return {'yaw_offset_deg': yaw, 'position_mm': list(pos),
            'accepted_at': '2026-10-09T13:24:52.123456+00:00'}


def _hb(state, yaw=0.0):
    from jugglebot_interfaces.msg import BallButlerHeartbeat
    msg = BallButlerHeartbeat()
    msg.state = int(state)
    msg.yaw_deg = yaw
    return msg


def _result(yaw_deg, std=0.05, pos=POS, tres=0.3):
    return SimpleNamespace(
        bb_position_mm=np.array(pos, dtype=float),
        arc_position_mm=np.array(pos, dtype=float) + [0.1, -1.5, 0.0],
        axis_direction=np.array([0.0, 0.0, 1.0]), axis_tilt_deg=0.8,
        yaw_offset_rad=math.radians(yaw_deg), yaw_offset_std_deg=std,
        yaw_span_deg=125.0, marker_metrics={}, yaw_method='constellation',
        anchor_yaw_offset_rad=math.radians(0.9),
        yaw_estimate=SimpleNamespace(lag_s=0.083, template_residual_mm=tres,
                                     yaw_source='heartbeat', n_frames=1187,
                                     summary=lambda: LONG_SUMMARY))


def _sweep(node, result=None, error=None):
    """One CALIBRATING → IDLE sweep with the solver mocked."""
    import jugglebot.mocap_node as mn
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._calib_data = {0: [np.zeros(3)]}
    kw = {'side_effect': error} if error is not None else {'return_value': result}
    with patch.object(mn, 'run_calibration', **kw):
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))


def _rec(node):
    lines = []

    class L:
        def __getattr__(self, level):
            return lambda m, *a, **k: lines.append((level, m))
    node._logger = L()
    return lines


def _at(lines, level):
    return [m for lv, m in lines if lv == level]


# ── Keep-last-good ──────────────────────────────────────────────────────────

def test_a_failed_sweep_after_a_success_leaves_the_latched_result_alone(tmp_path):
    node, path = _node(tmp_path, _ref(0.926))
    _sweep(node, _result(0.95))                         # inside ±0.150°: accepted
    good = node.pub_calibration.published[-1]
    assert good.success
    state_before = path.read_text()

    _sweep(node, _result(0.56))                         # Δ -0.366°: refused
    assert node.pub_calibration.published == [good], \
        'a failure was published over the success on the latched topic'
    attempt = node.pub_calibration_attempt.published[-1]
    assert not attempt.success and attempt.message.startswith('CALIBRATION_INCONSISTENT')
    assert path.read_text() == state_before            # state file untouched


def test_every_failure_kind_after_a_success_is_discarded(tmp_path):
    """Solver refusal, ERROR exit, QTM dropout and timeout: none of them reach
    the latched result once there is a success."""
    node, _ = _node(tmp_path)
    _sweep(node, _result(0.9))                          # no reference: accepted
    good = node.pub_calibration.published[-1]
    _sweep(node, error=ValueError('CONSTELLATION_TOO_FEW_FRAMES: 12 mocap frames'))
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._on_bb_heartbeat(_hb(BallButlerStates.ERROR))
    node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node.mocap.is_receiving.return_value = False
    node._check_calibration_health()
    node.mocap.is_receiving.return_value = True
    node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._calib_start_mono -= 1000.0
    node._check_calibration_health()
    assert node.pub_calibration.published == [good]
    codes = [m.message.split(':')[0] for m in node.pub_calibration_attempt.published[1:]]
    assert codes == ['CONSTELLATION_TOO_FEW_FRAMES', 'BB entered ERROR state during calibration',
                     'QTM_DROPOUT_MID_SWEEP', 'CALIBRATION_TIMEOUT']
    assert not any(m.success for m in node.pub_calibration_attempt.published[1:])


def test_with_no_success_yet_a_failure_is_reported_as_before(tmp_path):
    node, _ = _node(tmp_path)
    _sweep(node, error=ValueError('CONSTELLATION_TOO_FEW_FRAMES: 12 mocap frames'))
    assert [m.success for m in node.pub_calibration.published] == [False]
    assert node.pub_calibration.published[-1] is node.pub_calibration_attempt.published[-1]


def test_a_later_success_replaces_the_calibration_and_clears_the_attempt(tmp_path):
    """The window FSM closes after a discarded failure, so the next sweep runs
    and a success replaces the calibration in force (no hang, no stale lock)."""
    node, _ = _node(tmp_path, _ref(0.926))
    _sweep(node, _result(0.95))
    _sweep(node, _result(0.56))                         # refused, discarded
    assert node._calibrating is False and node._calib_blocked is False
    _sweep(node, _result(0.93))
    assert [m.success for m in node.pub_calibration.published] == [True, True]
    assert node.pub_calibration.published[-1].yaw_offset_rad == pytest.approx(math.radians(0.93))
    assert node.pub_calibration_attempt.published[-1].success


def test_both_calibration_topics_are_latched_depth_one(tmp_path):
    from rclpy.qos import DurabilityPolicy
    node, _ = _node(tmp_path)
    for pub in (node.pub_calibration, node.pub_calibration_attempt):
        assert pub.qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert pub.qos.depth == 1
    assert node.pub_calibration_attempt.topic_name == 'bb/calibration_attempt'


def test_a_restarted_consumer_receives_the_success_not_the_failure(tmp_path):
    from jugglebot.ball_butler_node import BallButlerNode
    node, _ = _node(tmp_path, _ref(0.926))
    _sweep(node, _result(0.95, pos=[-975.0, -389.0, 1735.0]))   # 0.9 mm: accepted
    _sweep(node, _result(0.56))                         # refused
    replayed = node.pub_calibration.published[-1]       # what depth-1 latching replays
    fresh = BallButlerNode()
    fresh._on_bb_calibration(replayed)
    assert node.pub_calibration.published[-1].success
    assert fresh._bb_position_mm == pytest.approx((-975.0, -389.0, 1735.0))
    assert fresh._bb_yaw_offset_rad == pytest.approx(math.radians(0.95))


def test_a_running_consumer_keeps_its_calibration_through_a_failure(tmp_path):
    """ball_butler_node subscribed to the result topic; it also tolerates a
    success=False on it (a mocap_node restarted with no success yet)."""
    from jugglebot.ball_butler_node import BallButlerNode
    node, _ = _node(tmp_path)
    bb = BallButlerNode()
    _sweep(node, _result(0.9))
    for m in node.pub_calibration.published:
        bb._on_bb_calibration(m)
    _sweep(node, _result(0.9, tres=0.9))                # TEMPLATE_RESIDUAL refusal
    for m in node.pub_calibration_attempt.published:    # even a consumer of every outcome
        bb._on_bb_calibration(m)
    assert bb._bb_position_mm == pytest.approx(tuple(POS))
    assert bb._bb_yaw_offset_rad == pytest.approx(math.radians(0.9))


def test_success_message_carries_its_acceptance_time(tmp_path):
    import re
    node, path = _node(tmp_path)
    _sweep(node, _result(0.9))
    msg = node.pub_calibration.published[-1].message
    m = re.search(r'accepted (\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d)Z', msg)
    assert m and msg.startswith('Calibration successful (accepted ')
    assert json.loads(path.read_text())['accepted_at'].startswith(m.group(1))


# ── The short failure line ──────────────────────────────────────────────────

def test_inconsistent_failure_is_one_short_line_with_the_decisive_numbers(tmp_path):
    node, _ = _node(tmp_path, _ref(0.926))
    _sweep(node, _result(0.95))
    log = _rec(node)
    _sweep(node, _result(0.56))
    msg = node.pub_calibration_attempt.published[-1].message
    # Limit 3·√(σ_new² + σ_ref²) with both σ 0.05 (the reference's σ comes from the
    # state file written by the first sweep): 0.212°, not the 0.15° floor.
    assert msg == ('CALIBRATION_INCONSISTENT: Δyaw -0.390° exceeds ±0.212° vs 0.950° '
                   f'(accepted {node._last_good_at[:19]}) — set bb_moved:=true if BB or '
                   'QTM moved')
    errors = _at(log, 'error')
    assert errors == [f'BB calibration FAILED: {msg} (kept the calibration from '
                      f'{node._last_good_at})']
    assert len(errors[0]) < 250 and '\n' not in errors[0]
    assert 'sweep estimator' not in msg and 'sweep estimator' not in errors[0]
    assert any('sweep estimator' in m for m in _at(log, 'debug'))


def test_inconsistent_against_the_state_file_names_its_acceptance_time(tmp_path):
    node, _ = _node(tmp_path, _ref(0.926))
    _sweep(node, _result(0.56, pos=[POS[0] + 1.6, POS[1], POS[2]]))
    msg = node.pub_calibration_attempt.published[-1].message
    assert msg == ('CALIBRATION_INCONSISTENT: Δyaw -0.366° exceeds ±0.150°, axis point moved '
                   '1.60 mm > 1.5 mm vs 0.926° (accepted 2026-10-09T13:24:52) — set '
                   'bb_moved:=true if BB or QTM moved')


def _within_bound(msg):
    import jugglebot.mocap_node as mn
    assert '\n' not in msg and '…' not in msg, msg
    assert len(msg) <= mn.MAX_CALIBRATION_FAILURE_CHARS, (len(msg), msg)


@pytest.mark.parametrize('case', ['template', 'unreadable', 'timeout', 'dropout',
                                  'error_state', 'no_data', 'anchor'])
def test_node_failure_reasons_fit_the_bound(tmp_path, case):
    node, path = _node(tmp_path, _ref(0.926),
                       state_name='a_rather_long_state_file_name_for_bb_calibration.json')
    if case == 'template':
        _sweep(node, _result(0.926, tres=0.71))
    elif case == 'unreadable':
        path.write_text('{not json')
        _sweep(node, _result(0.926))
    elif case == 'timeout':
        node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
        node._calib_start_mono -= 1000.0
        node._check_calibration_health()
    elif case == 'dropout':
        node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
        node.mocap.is_receiving.return_value = False
        node._check_calibration_health()
    elif case == 'error_state':
        node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
        node._on_bb_heartbeat(_hb(BallButlerStates.ERROR))
    elif case == 'no_data':
        node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    elif case == 'anchor':
        r = _result(0.926)
        r.yaw_method = 'anchor'
        _sweep(node, r)
    msg = node.pub_calibration_attempt.published[-1]
    assert not msg.success
    _within_bound(msg.message)
    if case == 'unreadable':
        assert msg.message.startswith('CALIBRATION_STATE_UNREADABLE: '
                                      'a_rather_long_state_file_name_for_bb_calibration.json: '
                                      'JSONDecodeError')


def test_gate_refusals_fit_the_bound():
    ref = _ref(0.926)
    label = 'accepted 2026-10-09T13:24:52'
    for v in (check_calibration_consistency(-179.0, 0.4, [p + 30 for p in POS], 0.3, ref,
                                            reference_label=label),
              check_calibration_consistency(0.926, 0.05, POS, 0.99, ref, bb_moved=True,
                                            reference_label=label)):
        assert not v.accepted
        _within_bound(v.message)


def test_estimator_refusals_fit_the_bound():
    """Each CONSTELLATION_* refusal, driven by the estimator's own triggers
    (the recipes of tests/ros/test_bb_calibration_constellation.py)."""
    from jugglebot.bb_calibration import estimate_sweep_yaw_offset
    from tests.ros.test_bb_calibration_constellation import _synth, _tmpl
    cases = [
        (dict(seed=12), dict(), slice(0, 150), 'CONSTELLATION_TOO_FEW_FRAMES'),
        (dict(seed=13, keep=[0, 3]), dict(), None, 'CONSTELLATION_TOO_FEW_MARKERS'),
        (dict(seed=14, displace=(4, np.array([1.6, 0.0, 0.0]))), dict(), None,
         'CONSTELLATION_RESIDUAL'),
        (dict(seed=15, lag=0.33), dict(yaw_source='stamped'), None, 'CONSTELLATION_LAG_AT_EDGE'),
        (dict(seed=16, rate=100.0, lag=0.004, units=1.0 / 360.0), dict(yaw_source='stamped'),
         None, 'CONSTELLATION_'),
        (dict(seed=17, y_max=0.0), dict(), None, 'CONSTELLATION_TOO_FEW_MOVING'),
    ]
    for synth_kw, est_kw, cut, code in cases:
        frames, samples = _synth(**synth_kw)
        if cut is not None:
            frames = frames[cut]
        with pytest.raises(ValueError) as exc:
            estimate_sweep_yaw_offset(frames, samples, _tmpl(), **est_kw)
        assert str(exc.value).startswith(code)
        _within_bound(str(exc.value))


def test_axis_disagreement_refusal_fits_the_bound():
    from tests.ros.test_bb_calibration_consensus import _dataset, _run
    with pytest.raises(ValueError) as exc:
        _run(_dataset(shifted=(0, 1, 6)))
    _within_bound(str(exc.value))


def test_an_overlong_reason_is_collapsed_and_truncated_with_the_full_text_at_debug(tmp_path):
    import jugglebot.mocap_node as mn
    node, _ = _node(tmp_path)
    log = _rec(node)
    reason = 'CONSTELLATION_X: ' + 'detail\n' * 200
    node._publish_calibration_failure(reason)
    msg = node.pub_calibration_attempt.published[-1].message
    assert len(msg) == mn.MAX_CALIBRATION_FAILURE_CHARS and msg.endswith('…')
    assert '\n' not in msg and msg.startswith('CONSTELLATION_X: detail detail')
    assert any(m.startswith('Calibration failure (full text): CONSTELLATION_X')
               for m in _at(log, 'debug'))
    assert len(_at(log, 'error')) == 1
