"""mocap_node's decimated GUI copies and the ``qtm_frame_hz`` status key (JB-76).

rosbridge deserialises every message it ingests before the client's throttle
applies, so the GUI takes ``gui/mocap_data`` and ``gui/rigid_body_poses`` (every
``gui_copy_divisor``-th frame, default 15 -> 20 Hz at 300 Hz QTM) instead of the
300 Hz originals. Pinned here: the copy goes out on EXACTLY every Nth frame,
as the IDENTICAL message object the original topic got, the default divisor is
15, and ``mocap/status`` carries ``qtm_frame_hz`` (the GUI reads only that key
for the 300 Hz rows of its topic monitor).
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import numpy as np


def _frame(stamp_ns=None, *, number=1, bodies=()):
    from jugglebot.mocap_interface import MocapFrame
    bb = np.full((7, 4), np.nan)
    return MocapFrame(number, 0, stamp_ns, None, True, [], [], tuple(bodies),
                      tuple(map(tuple, bb)))


def _fake_iface():
    iface = MagicMock()
    iface.is_receiving.return_value = True
    iface.is_aligned = True
    iface.get_qtm_sync_status.return_value = {'synced': True}
    iface.get_ball_butler_markers_base_frame.return_value = np.zeros((5, 4))
    iface.latest_frame_ros_ns.return_value = None
    iface.drain_frames.return_value = []
    return iface


def _make_node(iface):
    import jugglebot.mocap_node as mn
    with patch.object(mn, 'MocapInterface', return_value=iface):
        return mn.MocapNode(solve_inline=True)


BODY = ('Catching_Cone', 'world', 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)


def _publish(node, n, bodies=(BODY,)):
    for i in range(1, n + 1):
        node._publish_frame(_frame(stamp_ns=1_000_000 * i, number=i, bodies=bodies))


def test_default_divisor_is_15():
    node = _make_node(_fake_iface())
    assert node._params['gui_copy_divisor'] == 15
    assert node._gui_copy_divisor == 15


def test_copy_on_exactly_every_nth_frame():
    node = _make_node(_fake_iface())
    n = node._gui_copy_divisor
    _publish(node, 3 * n + 4)
    assert len(node.pub_mocap.published) == 3 * n + 4
    assert len(node.pub_rigid_bodies.published) == 3 * n + 4
    assert len(node.pub_gui_mocap.published) == 3
    assert len(node.pub_gui_rigid_bodies.published) == 3
    # Frame numbers 1-based: the copies are frames n, 2n, 3n.
    for k, copy in enumerate(node.pub_gui_mocap.published, start=1):
        assert copy is node.pub_mocap.published[k * n - 1]


def test_copy_is_the_identical_object():
    node = _make_node(_fake_iface())
    _publish(node, 15)
    assert node.pub_gui_mocap.published[0] is node.pub_mocap.published[14]
    assert node.pub_gui_rigid_bodies.published[0] is node.pub_rigid_bodies.published[14]


def test_divisor_parameter_is_honoured():
    node = _make_node(_fake_iface())
    node._gui_copy_divisor = 4
    _publish(node, 10)
    assert len(node.pub_gui_mocap.published) == 2


def test_bb_markers_have_no_gui_copy():
    node = _make_node(_fake_iface())
    _publish(node, 15)
    assert len(node.pub_bb_markers.published) == 15
    assert not hasattr(node, 'pub_gui_bb_markers')


def test_status_carries_qtm_frame_hz():
    node = _make_node(_fake_iface())
    with patch('jugglebot.mocap_node.time.monotonic', return_value=100.0):
        node._publish_mocap_status()           # primes the period
    _publish(node, 60)
    with patch('jugglebot.mocap_node.time.monotonic', return_value=100.2):
        node._publish_mocap_status()           # 60 frames in 0.2 s = 300 Hz
    kv = {v.key: v.value for v in node.pub_mocap_status.published[-1].values}
    assert 'qtm_frame_hz' in kv
    assert kv['qtm_frame_hz'] == '300.0'
    first = {v.key: v.value for v in node.pub_mocap_status.published[0].values}
    assert first['qtm_frame_hz'] == '0.0'


def test_a_due_frame_without_bodies_copies_mocap_but_no_rigid_bodies():
    """The rigid-body copy rides inside the existing ``if bodies`` publish
    block: a due frame with no bodies copies ``gui/mocap_data`` only."""
    node = _make_node(_fake_iface())
    _publish(node, node._gui_copy_divisor, bodies=())
    assert len(node.pub_gui_mocap.published) == 1
    assert len(node.pub_rigid_bodies.published) == 0
    assert len(node.pub_gui_rigid_bodies.published) == 0
