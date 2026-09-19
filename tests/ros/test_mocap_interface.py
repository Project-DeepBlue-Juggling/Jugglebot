"""Outage-logging tests for MocapInterface (the QTM connection handler).

Covers the state-transition logging contract for task (c): the "QTM
unavailable" WARNING logs the FIRST failure of an outage and then stays silent
until QTM returns (no throttled repeat), and the state TRANSITIONS
(available→unavailable, unavailable→available) are the loud events — matching
the connection-handling pattern used by the SpaceMouse handler and mocap's own
_on_qtm_disconnect.

ROS 2 is mocked by tests/ros/conftest.py (geometry_msgs in particular). qtm_rt
is the real library (installed on the Jetson). MocapInterface.start() spins an
asyncio thread that would attempt real network connects, so we patch it to a
no-op and drive the logging methods directly.
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

from jugglebot.mocap_interface import MocapInterface


def _make_iface():
    """Construct a MocapInterface without starting its asyncio thread."""
    with patch.object(MocapInterface, 'start', lambda self: None):
        return MocapInterface(host='127.0.0.1', port=22223,
                              logger=MagicMock(), node=MagicMock())


# ── Frame-stamp source: latest_frame_ros_ns (2026-09-20) ───────────────────
# See mocap_interface.py's on_packet / _update_qtm_clock_sync /
# latest_frame_ros_ns and MocapDataMulti.msg's `stamp` field.

def _set_ros_ns(iface, ros_ns):
    """Pin the ROS clock reading `_update_qtm_clock_sync` will read next."""
    iface.node.get_clock.return_value.now.return_value.nanoseconds = ros_ns


def _marker(x, y, z, residual=0.5):
    m = MagicMock()
    m.x, m.y, m.z, m.residual = x, y, z, residual
    return m


def _packet(qtm_us, *, unlabelled=None):
    """A QTM packet stand-in carrying only what on_packet needs: its own
    timestamp and (optionally) unlabelled markers. Labelled/6dof are None so
    on_packet's parameter-refresh check (which needs real marker_dict/
    body_dict bookkeeping) never fires."""
    p = MagicMock()
    p.timestamp = qtm_us
    p.get_3d_markers_residual.return_value = None
    p.get_6d.return_value = None
    if unlabelled is None:
        p.get_3d_markers_no_label_residual.return_value = None
    else:
        p.get_3d_markers_no_label_residual.return_value = (MagicMock(), unlabelled)
    return p


def test_latest_frame_ros_ns_is_none_before_any_packet():
    iface = _make_iface()
    assert iface.latest_frame_ros_ns() is None


def test_latest_frame_ros_ns_follows_the_packet_timestamp_through_the_offset():
    """`latest_frame_ros_ns()` must track whichever packet was processed
    LAST, converted through the SAME smoothed offset `qtm_timestamp_to_ros_ns`
    exposes — not a value frozen at first sync."""
    iface = _make_iface()
    iface.ready_to_publish = True

    _set_ros_ns(iface, 5_000_000_000)
    iface.on_packet(_packet(1_000_000))
    first = iface.latest_frame_ros_ns()
    assert first == iface.qtm_timestamp_to_ros_ns(1_000_000)

    _set_ros_ns(iface, 7_030_000_000)
    iface.on_packet(_packet(2_000_000))
    second = iface.latest_frame_ros_ns()
    assert second == iface.qtm_timestamp_to_ros_ns(2_000_000)
    assert second != first


def test_marker_snapshot_and_frame_time_update_together():
    """The frame's markers and its own time are written under the SAME lock
    in on_packet (2026-09-20) — a reader must never see one packet's markers
    paired with a different packet's time."""
    iface = _make_iface()
    iface.ready_to_publish = True

    _set_ros_ns(iface, 1_000_000_000)
    iface.on_packet(_packet(100_000, unlabelled=[_marker(1.0, 2.0, 3.0)]))
    markers_a = iface.get_all_markers_base_frame()
    frame_a_ns = iface.latest_frame_ros_ns()
    assert markers_a.shape[0] == 1 and markers_a[0, 0] == 1.0

    _set_ros_ns(iface, 1_050_000_000)
    iface.on_packet(_packet(150_000, unlabelled=[_marker(9.0, 8.0, 7.0)]))
    markers_b = iface.get_all_markers_base_frame()
    frame_b_ns = iface.latest_frame_ros_ns()
    assert markers_b.shape[0] == 1 and markers_b[0, 0] == 9.0
    assert frame_b_ns == iface.qtm_timestamp_to_ros_ns(150_000)
    assert frame_b_ns != frame_a_ns


def test_qtm_outage_logs_first_then_silent():
    """Repeated connect failures within one outage log exactly one WARNING."""
    iface = _make_iface()
    iface._log_qtm_outage('connection refused')
    iface._log_qtm_outage('connection refused')
    iface._log_qtm_outage('connection refused')
    assert iface.logger.warning.call_count == 1
    assert iface._qtm_outage_active is True
    assert 'QTM unavailable' in iface.logger.warning.call_args[0][0]


def test_qtm_reconnect_clears_outage_and_relogs_next_outage():
    """The unavailable→available edge clears the latch so a NEW outage logs its
    first failure again (state-transition logging, not a one-shot mute)."""
    iface = _make_iface()
    iface._log_qtm_outage('down')
    assert iface.logger.warning.call_count == 1
    # connect() clears this on the "Connected to QTM." edge.
    iface._qtm_outage_active = False
    iface._log_qtm_outage('down again')
    assert iface.logger.warning.call_count == 2


def test_qtm_disconnect_marks_outage_and_silences_reconnect():
    """A mid-session drop is the available→unavailable transition: it logs one
    'QTM disconnected' WARNING and marks the outage active so the reconnect
    loop's connect() failures stay silent (no duplicate 'QTM unavailable')."""
    iface = _make_iface()
    iface.loop = None  # no running loop → _on_qtm_disconnect schedules no task
    iface._on_qtm_disconnect(RuntimeError('cable pulled'))
    assert iface.logger.warning.call_count == 1
    assert 'QTM disconnected' in iface.logger.warning.call_args[0][0]
    assert iface._qtm_outage_active is True
    # The reconnect loop now retries silently.
    iface._log_qtm_outage('still down')
    assert iface.logger.warning.call_count == 1
