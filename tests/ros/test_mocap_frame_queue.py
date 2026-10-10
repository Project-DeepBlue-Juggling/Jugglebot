"""The mocap frame queue (logbook 2026-10-10-mocap-frame-queue).

Until 2026-10-10 ``MocapInterface.on_packet`` overwrote ONE latest-frame
snapshot and a 5 ms timer published it, so when the process was starved every
packet received during the stall but the last was lost (180 of QTM's 300
frames/s on an idle Jetson, 70-116 loaded; logbook
2026-10-10-mocap-frame-loss-under-load). Now every frame is queued (bounded,
``FRAME_QUEUE_MAXLEN``) and the publisher tick drains it, one message per
frame with the frame's own stamp. These tests drive the REAL MocapInterface
(its asyncio thread patched out) into the real MocapNode under the mocked ROS
of tests/ros/conftest.py.
"""

from __future__ import annotations

import math
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from jugglebot.protocol_config import BallButlerStates
from jugglebot import mocap_interface as mi
from jugglebot.mocap_interface import FRAME_QUEUE_MAXLEN, MocapInterface, MocapFrame

from rclpy.node import Node

MockLogger = type(Node('_probe').get_logger())

LABELS = ['Ball Butler - %d' % i for i in range(1, 8)] + ['Catching Cone - 1']
BODIES = ['Base', 'Platform', 'Catching Cone', 'Ball Butler']


class RecLogger(MockLogger):
    """Records lines; still enforces rclpy's one-severity-per-call-site rule."""

    def __init__(self):
        super().__init__()
        self.lines = []

    def _rec(self, level, msg, kw):
        self.lines.append((level, msg))
        self._log(level, kw)

    def info(self, msg, **kw): self._rec('INFO', msg, kw)
    def warning(self, msg, **kw): self._rec('WARN', msg, kw)
    def warn(self, msg, **kw): self._rec('WARN', msg, kw)
    def error(self, msg, **kw): self._rec('ERROR', msg, kw)
    def debug(self, msg, **kw): self._rec('DEBUG', msg, kw)

    def at(self, level):
        return [m for lv, m in self.lines if lv == level]


# ── Harness ──────────────────────────────────────────────────────────────────

def _iface():
    with patch.object(MocapInterface, 'start', lambda self: None):
        iface = MocapInterface(host='127.0.0.1', port=22223,
                               logger=MagicMock(), node=MagicMock())
    iface.marker_dict = {name: i for i, name in enumerate(LABELS)}
    iface.body_dict = {name: i for i, name in enumerate(BODIES)}
    iface.ready_to_publish = True
    return iface


def _m(x, y, z, r=0.5):
    return SimpleNamespace(x=x, y=y, z=z, residual=r)


def _identity_body(x, y, z):
    return (SimpleNamespace(x=x, y=y, z=z),
            SimpleNamespace(matrix=[1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]))


def _packet(fn, *, qtm_us=None, drop_rate=0, ball=(500.0, 600.0, 700.0)):
    """A QTM packet of frame ``fn`` at 300 Hz: the 8 labelled markers (BB 1..7
    visible at x = 100 + i, the cone marker NaN), one unlabelled ball marker
    whose x moves with the frame number, and the 4 bodies."""
    q = 1_000_000 + int(fn * 3333.333) if qtm_us is None else qtm_us
    labelled = [_m(100.0 + i, 200.0, 300.0) for i in range(7)] + [_m(math.nan, 0, 0)]
    header = SimpleNamespace(marker_count=len(labelled), drop_rate=drop_rate, out_of_sync_rate=0)
    p = MagicMock()
    p.timestamp = q
    p.framenumber = fn
    p.get_3d_markers_residual.return_value = (header, labelled)
    p.get_3d_markers_no_label_residual.return_value = (
        header, [SimpleNamespace(x=ball[0] + fn, y=ball[1], z=ball[2], id=0, residual=0.3)])
    p.get_6d.return_value = (header, [_identity_body(1.0 * k, 0.0, 10.0) for k in range(4)])
    return p


def _receive(iface, fn, *, late_ms=2.0, base_ns=5_000_000_000, drift_ppm=0.0, **kw):
    """on_packet for frame ``fn``, received ``late_ms`` after its QTM time
    (on a ROS clock running ``drift_ppm`` off QTM's)."""
    p = _packet(fn, **kw)
    iface.node.get_clock.return_value.now.return_value.nanoseconds = (
        base_ns + int(p.timestamp * 1000 * (1 + drift_ppm * 1e-6)) + int(late_ms * 1e6))
    iface.on_packet(p)
    return p


def _node(iface):
    import jugglebot.mocap_node as mn
    with patch.object(mn, 'MocapInterface', return_value=iface):
        node = mn.MocapNode()
    node._logger = RecLogger()
    return node


def _stamp_ns(msg):
    return msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec


# ── The queue ────────────────────────────────────────────────────────────────

def test_a_stall_publishes_every_frame_with_its_own_monotonic_stamp():
    """The loss mechanism itself: N packets arrive back to back while the
    publisher is stalled, then ONE tick. Old code published 1 message; the
    queue publishes N, oldest first, each with its own frame's stamp."""
    iface = _iface()
    node = _node(iface)
    n = 45                                           # a 150 ms stall at 300 Hz
    packets = [_receive(iface, fn, late_ms=2.0 + 0.5 * k) for k, fn in enumerate(range(10, 10 + n))]
    node._publish_mocap_data()

    data = node.pub_mocap.published
    assert len(data) == n
    stamps = [_stamp_ns(m) for m in data]
    assert all(b > a for a, b in zip(stamps, stamps[1:]))           # strictly increasing
    assert len(set(stamps)) == n
    # each stamp is ITS frame's QTM time through the min-latency offset (the
    # late packets do not move the envelope): offset = base + 2 ms
    assert stamps == [5_000_000_000 + p.timestamp * 1000 + 2_000_000 for p in packets]
    # each message carries its own frame's markers (the ball x moves per frame)
    balls = [[mk.position.x for mk in m.markers if mk.label == ''] for m in data]
    assert balls == [[500.0 + fn] for fn in range(10, 10 + n)]
    # labelled: the 7 visible BB markers with their names, the NaN one dropped
    assert [mk.label for mk in data[0].markers if mk.label] == LABELS[:7]
    # bb/markers and rigid_body_poses: one per frame, the same stamps
    assert [_stamp_ns(m) for m in node.pub_bb_markers.published] == stamps
    assert len(node.pub_rigid_bodies.published) == n
    assert iface.drain_frames() == []


def test_an_empty_tick_publishes_nothing():
    """No new frame since the last tick: no message on any per-frame topic.
    (The old tick re-published the last frame with an empty marker list.)"""
    iface = _iface()
    node = _node(iface)
    _receive(iface, 1)
    node._publish_mocap_data()
    counts = (len(node.pub_mocap.published), len(node.pub_bb_markers.published),
              len(node.pub_rigid_bodies.published))
    assert counts == (1, 1, 1)
    node._publish_mocap_data()
    node._publish_mocap_data()
    assert (len(node.pub_mocap.published), len(node.pub_bb_markers.published),
            len(node.pub_rigid_bodies.published)) == counts


def test_overflow_drops_the_oldest_and_counts():
    iface = _iface()
    extra = 7
    for fn in range(FRAME_QUEUE_MAXLEN + extra):
        _receive(iface, fn)
    frames = iface.drain_frames()
    assert len(frames) == FRAME_QUEUE_MAXLEN
    assert [f.frame_number for f in frames] == list(range(extra, FRAME_QUEUE_MAXLEN + extra))
    st = iface.take_stream_stats()
    assert st['overflow_drops'] == extra
    assert st['frames_queued'] == FRAME_QUEUE_MAXLEN + extra
    assert st['max_queue_depth'] == FRAME_QUEUE_MAXLEN
    assert st['qtm_gap_frames'] == 0
    assert iface.take_stream_stats()['max_queue_depth'] == 0      # reset by the read


def test_queue_bound_covers_half_a_second_at_300_hz():
    assert FRAME_QUEUE_MAXLEN >= 0.5 * 300


def test_a_repeated_frame_number_is_queued_once():
    iface = _iface()
    _receive(iface, 5)
    _receive(iface, 5)
    _receive(iface, 6)
    assert [f.frame_number for f in iface.drain_frames()] == [5, 6]
    st = iface.take_stream_stats()
    assert st['duplicate_frames'] == 1 and st['qtm_gap_frames'] == 0


def test_frame_number_gaps_are_counted_and_a_qtm_restart_is_not_a_gap():
    iface = _iface()
    for fn in (1, 2, 5, 6, 10):                      # 2 + 3 frames never sent
        _receive(iface, fn)
    st = iface.take_stream_stats()
    assert (st['qtm_gap_frames'], st['qtm_gap_events']) == (5, 2)
    _receive(iface, 1, qtm_us=50_000_000)            # QTM restarted its counter
    st = iface.take_stream_stats()
    assert st['qtm_gap_frames'] == 5 and st['qtm_restarts'] == 1


def test_disconnect_discards_the_queue_and_restarts_the_frame_counter():
    iface = _iface()
    iface.loop = None
    _receive(iface, 100)
    iface._on_qtm_disconnect(None)
    assert iface.drain_frames() == []
    _receive(iface, 1, qtm_us=60_000_000)            # after the reconnect
    assert iface.take_stream_stats()['qtm_gap_frames'] == 0
    assert [f.frame_number for f in iface.drain_frames()] == [1]


def test_the_stamp_is_converted_once_at_receive_and_never_steps_backwards():
    """The 'stamps go backwards 164-490 times per bag' side finding: in the
    10-09 and 10-10 bags (stats of the 2026-10-10 investigation) EVERY backward
    step (0.2-21 µs) is a message with no markers right after a real one: the
    old empty tick re-published the previous frame's QTM time re-converted
    through an offset that had moved down a few µs since. Each frame is now
    converted once, at receive, and published once (an empty tick publishes
    nothing), so with the offset moving (a ROS clock 200 ppm slow: the
    envelope, and so the offset, steps down ~0.7 µs per frame, the direction
    that made the old re-published stamps go back) under jittered latency the
    published
    stamps only go forwards and each equals its frame's conversion at receive.
    (An offset STEP larger than a frame period, i.e. a clock-sync re-anchor,
    can still step them back: that is the sync's, not the queue's.)"""
    iface = _iface()
    node = _node(iface)
    rng = np.random.RandomState(7)
    expected = []
    for fn in range(1, 400):
        late = 2.0 if fn == 1 else 2.0 + rng.exponential(1.5)
        p = _receive(iface, fn, late_ms=late, drift_ppm=-200.0)
        expected.append(iface.qtm_timestamp_to_ros_ns(p.timestamp))
        if fn % 2:
            node._publish_mocap_data()
            node._publish_mocap_data()               # an empty tick
    node._publish_mocap_data()
    stamps = [_stamp_ns(m) for m in node.pub_mocap.published]
    assert stamps == expected
    back = [(k, b - a) for k, (a, b) in enumerate(zip(stamps, stamps[1:])) if b <= a]
    assert back == []
    offsets = {s - (1_000_000 + int(fn * 3333.333)) * 1000 for fn, s in zip(range(1, 400), stamps)}
    assert len(offsets) > 1                          # the offset did move


def test_name_lookups_follow_a_replaced_marker_dict():
    iface = _iface()
    _receive(iface, 1)
    assert iface.drain_frames()[0].labelled[0][0] == 'Ball Butler - 1'
    iface.marker_dict = {('Spare - %d' % i): i for i in range(8)}   # _refresh_parameters
    _receive(iface, 2)
    f = iface.drain_frames()[0]
    assert f.labelled[0][0] == 'Spare - 0'
    assert f.bb_markers == mi._NAN_BB_ROWS            # no BB labels in this setup


def test_bodies_carry_sanitised_names_frames_and_the_platform_offset():
    iface = _iface()
    iface.set_base_to_platform_offset(100.0)
    _receive(iface, 1)
    bodies = {b[0]: b for b in iface.drain_frames()[0].bodies}
    assert set(bodies) == {'Base', 'Platform', 'Catching_Cone', 'Ball_Butler'}
    assert bodies['Base'][1] == 'world' and bodies['Base'][4] == 10.0
    assert bodies['Platform'][1] == 'platform_start' and bodies['Platform'][4] == -90.0
    assert bodies['Catching_Cone'][1] == 'world'
    assert bodies['Base'][5:] == (0.0, 0.0, 0.0, 1.0)


def test_rigid_body_poses_keep_their_stamps_and_values():
    """Per fresh frame, as before: header at publish time, each pose stamped
    with its frame's receive time and carrying the body's position/quaternion."""
    iface = _iface()
    node = _node(iface)
    p = _receive(iface, 1, late_ms=3.0)
    node._publish_mocap_data()
    msg = node.pub_rigid_bodies.published[0]
    assert [b.name for b in msg.bodies] == ['Base', 'Platform', 'Catching_Cone', 'Ball_Butler']
    pose = msg.bodies[2].pose
    rx = 5_000_000_000 + p.timestamp * 1000 + 3_000_000
    assert (pose.header.stamp.sec, pose.header.stamp.nanosec) == divmod(rx, 1_000_000_000)
    assert pose.header.frame_id == 'world'
    assert (pose.pose.position.x, pose.pose.position.z) == (2.0, 10.0)
    assert pose.pose.orientation.w == 1.0


# ── Downstream: calibration window and base-frame cadence ────────────────────

def test_the_calibration_collector_still_takes_each_frame_once():
    """One point set per distinct stamp: a frame seen twice (a re-delivered
    record) is collected once; each queued frame is collected."""
    iface = _iface()
    node = _node(iface)
    node._on_bb_heartbeat(SimpleNamespace(state=int(BallButlerStates.CALIBRATING), yaw_deg=0.0))
    for fn in range(1, 31):
        _receive(iface, fn)
    frames = iface.drain_frames()
    for f in frames:
        node._publish_frame(f)
    node._publish_frame(frames[-1])                  # the same frame again
    assert len(node._calib_frames) == 30
    assert len(node._calib_data[0]) == 31            # the arc fit takes every message (as before)
    assert len(node._calib_base_points) == 30


def test_base_monitor_cadence_stays_every_0_2_s_of_frame_time():
    """300 frames over 1 s of QTM time feed the slow base estimate 5 times;
    the candidate array is not built for the other frames."""
    import jugglebot.mocap_node as mn
    iface = _iface()
    node = _node(iface)
    node._base_monitor = MagicMock()
    node._base_monitor_last_t = None
    with patch.object(mn, 'base_candidate_points', wraps=mn.base_candidate_points) as cand:
        for fn in range(300):
            _receive(iface, fn)
            node._publish_mocap_data()
    assert node._base_monitor.update.call_count == 5
    assert cand.call_count == 5


# ── Loss visibility: the 1 Hz line and the throttled WARN ────────────────────

def _health(node, monotonic):
    with patch('jugglebot.mocap_node.time.monotonic', return_value=monotonic):
        return node._check_stream_health()


def test_the_1hz_debug_line_carries_the_counters():
    iface = _iface()
    node = _node(iface)
    for fn in (1, 2, 3, 7):
        _receive(iface, fn, drop_rate=12)
    with patch('jugglebot.mocap_node.time.monotonic', return_value=100.0):
        node._publish_clock_offset()
    line = [m for m in node._logger.at('DEBUG') if m.startswith('qtm clock sync:')][-1]
    assert 'excess latency' in line                  # the existing content stays
    assert 'stream 0 frames/s' in line               # no previous read: no rate yet
    assert 'queue max 4/%d' % FRAME_QUEUE_MAXLEN in line
    assert 'node overflow 0 (total 0)' in line
    assert 'QTM gaps 3 frames (total 3)' in line
    assert 'QTM 2D drop 12‰' in line
    for fn in range(8, 308):
        _receive(iface, fn)
    with patch('jugglebot.mocap_node.time.monotonic', return_value=101.0):
        node._publish_clock_offset()
    line = [m for m in node._logger.at('DEBUG') if m.startswith('qtm clock sync:')][-1]
    assert 'stream 300 frames/s' in line and 'QTM gaps 0 frames (total 3)' in line


def _window(iface, node, start_fn, received, missing_every=0, gap=1, drain=True):
    """Receive `received` frames from number `start_fn`, skipping `gap` frame
    numbers after every `missing_every`-th; drain so the queue never overflows.
    Returns the next frame number."""
    fn = start_fn
    for i in range(received):
        _receive(iface, fn)
        fn += 1
        if missing_every and (i + 1) % missing_every == 0:
            fn += gap
        if drain and i % 100 == 99:
            iface.drain_frames()
    iface.drain_frames()
    return fn


def test_loss_at_or_below_5_percent_is_silent_and_in_the_debug_line():
    """4 % of ~3000 expected frames lost to QTM single-frame skips: no WARN."""
    iface = _iface()
    node = _node(iface)
    _health(node, 10.0)
    _window(iface, node, 0, 2880, missing_every=24)   # 119 seen skipped of ~3000 = 4 %
    line = _health(node, 20.0)
    assert node._logger.at('WARN') == []
    assert 'QTM gaps 119 frames' in line and 'node overflow 0' in line


def test_qtm_loss_above_5_percent_warns_naming_qtm_not_the_jetson():
    import jugglebot.mocap_node as mn
    iface = _iface()
    node = _node(iface)
    _health(node, 10.0)
    _window(iface, node, 0, 2820, missing_every=15)   # 187 seen skipped of 3007 = 6.2 %
    _health(node, 10.0 + mn.MOCAP_LOSS_WARN_PERIOD_S - 1)
    assert node._logger.at('WARN') == []              # judged only when the window ends
    _health(node, 10.0 + mn.MOCAP_LOSS_WARN_PERIOD_S)
    warns = node._logger.at('WARN')
    assert len(warns) == 1
    w = warns[0]
    assert w.startswith('mocap: 187 of 3007 expected QTM frames lost (6.2 %) in the last 10 s')
    assert '187 never received from QTM (187 frame-number gaps)' in w
    assert 'dropped by mocap_node' not in w and 'Jetson' not in w     # the node dropped nothing
    assert 'QTM' in w and 'frame-drop' in w


def test_node_overflow_above_5_percent_warns_with_the_starvation_wording():
    import jugglebot.mocap_node as mn
    iface = _iface()
    node = _node(iface)
    _health(node, 10.0)
    fn = 0
    for _ in range(10):                               # ten > 0.5 s stalls, 20 overflow each
        for _ in range(FRAME_QUEUE_MAXLEN + 20):
            _receive(iface, fn)
            fn += 1
        iface.drain_frames()
    _window(iface, node, fn, 1500)                    # 200 of 3200 = 6.25 %
    _health(node, 10.0 + mn.MOCAP_LOSS_WARN_PERIOD_S)
    warns = node._logger.at('WARN')
    assert len(warns) == 1
    w = warns[0]
    assert '200 dropped by mocap_node (frame queue full: starved for over 0.5 s)' in w
    assert 'never received from QTM' not in w
    assert 'check its load' in w


def test_a_multi_frame_gap_burst_is_judged_by_the_same_rule():
    """A single 200-frame QTM outage in a window (6.7 %) warns; the window is
    then cleared, and the next quiet window is silent."""
    import jugglebot.mocap_node as mn
    iface = _iface()
    node = _node(iface)
    _health(node, 10.0)
    fn = _window(iface, node, 0, 1400)
    fn += 200
    fn = _window(iface, node, fn, 1400)
    _health(node, 20.0)
    assert len(node._logger.at('WARN')) == 1
    assert '200 never received from QTM (1 frame-number gaps)' in node._logger.at('WARN')[0]
    _window(iface, node, fn, 3000)
    _health(node, 30.0)
    assert len(node._logger.at('WARN')) == 1


def test_no_loss_no_warn():
    iface = _iface()
    node = _node(iface)
    for t in range(5):
        for fn in range(t * 300, (t + 1) * 300):
            _receive(iface, fn)
            if fn % 100 == 99:                       # a 333 ms stall, inside the queue
                node._publish_mocap_data()
        _health(node, 100.0 + t)
    assert len(node.pub_mocap.published) == 1500
    assert node._logger.at('WARN') == []


# ── Cheaper per-frame work: same results ─────────────────────────────────────

def _quat_numpy(R_list):
    """The pre-2026-10-10 numpy implementation, verbatim, as the reference."""
    return MocapInterface.rotation_list_to_quaternion(None, R_list)


def _rot(axis, ang):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def test_float_quaternion_matches_the_numpy_one_on_every_branch():
    rng = np.random.RandomState(1)
    mats = [_rot(rng.normal(size=3), rng.uniform(0, math.pi)) for _ in range(300)]
    mats += [_rot(a, math.pi * 0.999) for a in ([1, 0, 0], [0, 1, 0], [0, 0, 1])]   # trace <= 0, each branch
    mats += [np.diag([-1.0, -1.0, 1.0]), np.diag([1.0, -1.0, -1.0]), np.diag([-1.0, 1.0, -1.0])]
    for R in mats:
        R_list = list(R.flatten(order='F'))
        assert mi.quaternion_from_rotation_list(R_list) == pytest.approx(
            tuple(_quat_numpy(R_list)), abs=1e-12)
    nan = mi.quaternion_from_rotation_list([math.nan] * 9)
    assert all(math.isnan(v) for v in nan)
    assert all(math.isnan(v) for v in _quat_numpy([math.nan] * 9))


def test_a_base_body_out_of_view_reads_not_aligned():
    iface = _iface()
    iface.is_aligned = True
    iface._params_need_refresh = True                # one body vs four in body_dict
    p = _packet(1)
    p.get_6d.return_value = (None, [(SimpleNamespace(x=math.nan, y=math.nan, z=math.nan),
                                     SimpleNamespace(matrix=[math.nan] * 9))])
    iface.node.get_clock.return_value.now.return_value.nanoseconds = 5_000_000_000
    iface.on_packet(p)
    assert iface.is_aligned is False
    assert 'Base body not visible to QTM' in iface.logger.warning.call_args[0][0]


def test_the_fast_marker_builder_is_off_under_the_message_stand_ins():
    """The slot-writing builder is used only on rosidl's slot layout, after
    an equality self-check; the tests' dataclass stand-ins fail it, so the
    checked (old) path runs here."""
    import jugglebot.mocap_node as mn
    assert mn.MARKER_FAST_PATH is False
    assert mn._make_marker is mn._marker_checked


def test_the_fast_marker_builder_self_check_accepts_rosidl_style_classes():
    import jugglebot.mocap_node as mn

    def _cls(name, fields, defaults):
        def init(self):
            for f, d in zip(fields, defaults):
                setattr(self, '_' + f, d() if callable(d) else d)
        ns = {'__slots__': ['_' + f for f in fields], '__init__': init,
              '__repr__': lambda self: name + repr(tuple(getattr(self, '_' + f) for f in fields))}
        for f in fields:
            ns[f] = property(lambda self, f=f: getattr(self, '_' + f),
                             lambda self, v, f=f: setattr(self, '_' + f, v))
        return type(name, (), ns)

    P = _cls('Point', ['x', 'y', 'z'], [0.0, 0.0, 0.0])
    S = _cls('MocapDataSingle', ['position', 'residual', 'label'], [P, 0.0, ''])
    with patch.object(mn, 'Point', P), patch.object(mn, 'MocapDataSingle', S):
        assert mn._marker_fast_is_safe() is True
        a = mn._marker_checked(1.0, 2.0, 3.0, 0.5, 'x')
        b = mn._marker_fast(1.0, 2.0, 3.0, 0.5, 'x')
        assert repr(a) == repr(b)
    Bad = _cls('Point', ['x', 'y'], [0.0, 0.0])
    with patch.object(mn, 'Point', Bad), patch.object(mn, 'MocapDataSingle', S):
        assert mn._marker_fast_is_safe() is False


def test_mocap_frame_fields():
    assert MocapFrame._fields == ('frame_number', 'qtm_us', 'stamp_ns', 'receive_ns', 'aligned',
                                  'labelled', 'unlabelled', 'bodies', 'bb_markers')
