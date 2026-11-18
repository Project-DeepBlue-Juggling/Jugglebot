"""`ball_tracker_node` publishes the Human Throw event on `human_throws`
(PWAY-11 P1, U2). The emitter itself is tested in
`tests/tracking/test_human_throw_event.py`; this file pins the node wiring:
silent while `detect_human_throws` is false, revisioned events keyed
(track_id, flight_seq) while true, and a clean stop/start on a live toggle.
"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from jugglebot.ball_tracker_node import BallTrackerNode
from jugglebot.tracking.ballistics import GRAVITY_MMPS2
from jugglebot.tracking.human_throw_event import HumanThrowEventConfig
from jugglebot_interfaces.msg import HumanThrowEvent, MocapDataMulti, MocapDataSingle

DT = 1.0 / 300.0
P0 = np.array([0.0, 0.0, 450.0])
V0 = np.array([100.0, -50.0, 4500.0])
T0 = 10.0  # nonzero mocap stamp origin


def _pos(t):
    return np.array([P0[0] + V0[0] * t, P0[1] + V0[1] * t,
                     P0[2] + V0[2] * t - 0.5 * GRAVITY_MMPS2 * t * t])


def _msg(pos, stamp_s):
    msg = MocapDataMulti()
    s = MocapDataSingle()
    s.position.x, s.position.y, s.position.z = (float(pos[0]), float(pos[1]), float(pos[2]))
    s.residual = 0.3
    s.label = ''
    msg.markers.append(s)
    ns = int(round(stamp_s * 1e9))
    msg.stamp.sec = ns // 1_000_000_000
    msg.stamp.nanosec = ns % 1_000_000_000
    return msg


def _node(detect):
    node = BallTrackerNode()
    node._tracker.detect_human_throws = detect
    node._human_pub = MagicMock()
    return node


def _fly(node, t_from, t_to, offset=(0.0, 0.0, 0.0)):
    t = t_from
    while t <= t_to:
        node._on_mocap(_msg(_pos(t - t_from) + np.array(offset), T0 + t))
        t += DT


def _events(node):
    return [c.args[0] for c in node._human_pub.publish.call_args_list]


def test_default_flag_is_false_and_publisher_is_idle():
    node = BallTrackerNode()
    assert node._tracker.detect_human_throws is False
    node._human_pub = MagicMock()
    _fly(node, 0.0, 0.9)
    node._human_pub.publish.assert_not_called()
    assert node._human_emitter is None


def test_flag_false_publishes_nothing_even_for_a_clean_flight():
    node = _node(False)
    _fly(node, 0.0, 0.9)
    assert _events(node) == []


def test_usable_human_flight_publishes_revision_zero_then_a_later_revision():
    node = _node(True)
    _fly(node, 0.0, 0.9)
    evs = _events(node)
    assert evs, 'a clean human Throw must publish'
    assert all(isinstance(e, HumanThrowEvent) for e in evs)
    first = evs[0]
    cfg = HumanThrowEventConfig()
    assert first.revision == 0
    assert first.n_samples >= 12
    assert first.rms_mm <= cfg.rms_mm
    assert first.start_speed_mm_s >= cfg.min_start_speed_mm_s
    land_t = first.landing_time.sec + first.landing_time.nanosec * 1e-9
    stamp = first.header.stamp.sec + first.header.stamp.nanosec * 1e-9
    assert land_t > stamp > T0
    assert any(e.revision >= 1 for e in evs[1:]), 'a later revision must follow revision 0'
    assert {(e.track_id, e.flight_seq) for e in evs} == {(first.track_id, first.flight_seq)}
    revs = [e.revision for e in evs]
    assert revs == sorted(revs) and len(set(revs)) == len(revs)


def test_toggling_off_and_on_mid_flight_does_not_publish_the_stale_flight():
    node = _node(True)
    _fly(node, 0.0, 0.30)
    n_before = len(_events(node))
    node._on_set_parameters([SimpleNamespace(name='detect_human_throws', value=False)])
    assert node._human_emitter is None
    # keep flying the same ball while off (continue the same trajectory)
    t = 0.30
    while t <= 0.45:
        node._on_mocap(_msg(_pos(t), T0 + t))
        t += DT
    assert len(_events(node)) == n_before
    node._on_set_parameters([SimpleNamespace(name='detect_human_throws', value=True)])
    while t <= 0.95:
        node._on_mocap(_msg(_pos(t), T0 + t))
        t += DT
    assert len(_events(node)) == n_before, 'a flight begun before the re-enable is stale'


def test_a_new_flight_after_a_re_enable_is_served():
    node = _node(True)
    _fly(node, 0.0, 0.30)
    node._on_set_parameters([SimpleNamespace(name='detect_human_throws', value=False)])
    node._on_set_parameters([SimpleNamespace(name='detect_human_throws', value=True)])
    base = len(_events(node))
    _fly(node, 3.0, 3.9, offset=(800.0, 300.0, 0.0))
    new = _events(node)[base:]
    assert new and new[0].revision == 0


def test_emitter_log_goes_through_the_node_logger_with_percent_args():
    node = _node(True)
    node.get_logger = MagicMock()
    node._ensure_human_emitter()._logger.info('HUMAN-THROW-REJECTED id %d seq %d', 41, 0)
    node.get_logger.return_value.info.assert_called_with('HUMAN-THROW-REJECTED id 41 seq 0')


def test_params_default_from_the_config_dataclass():
    node = BallTrackerNode()
    cfg = HumanThrowEventConfig()
    for name, attr in (
            ('human_throw_min_start_speed_mm_s', 'min_start_speed_mm_s'),
            ('human_throw_event_rms_mm', 'rms_mm'),
            ('human_throw_event_stable_mm', 'stable_mm'),
            ('human_throw_event_stable_gap_ms', 'stable_gap_ms'),
            ('human_throw_event_stable_time_ms', 'stable_time_ms'),
            ('human_throw_event_refine_mm', 'refine_mm'),
            ('human_throw_event_refine_ms', 'refine_ms')):
        assert float(node.get_parameter(name).value) == getattr(cfg, attr)
