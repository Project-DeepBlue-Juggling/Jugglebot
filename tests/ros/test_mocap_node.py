"""Tests for jugglebot.mocap_node.MocapNode — the QTM-side half of F4.

FIRST direct coverage of this node. Until 2026-08-21 it could not even be
IMPORTED under the mocked-ROS conftest: ``from geometry_msgs.msg import
TransformStamped`` and ``import tf2_ros`` both fell through the mock layer
(tf2_ros is unusable here in principle — its first import line is
``from rclpy.duration import Duration`` and this suite replaces ``rclpy`` with
a non-package module). Both stubs were added to tests/ros/conftest.py with
this file.

What is covered:

* **Q1** — the ``mocap/status`` DiagnosticStatus publisher: every KeyValue, the
  level, and the fact that it reads a SNAPSHOT (no side effects on the 200 Hz
  marker path, no gating of any existing publication).
* **Q5b** — a mid-sweep QTM dropout latches the collection window invalid,
  publishes ``QTM_DROPOUT_MID_SWEEP``, stops accumulating, and makes the
  state-exit edge skip the solver.
* **Q5c** — the 60 s wall-clock cap on a CALIBRATING window that never ends:
  publishes ``CALIBRATION_TIMEOUT``, unlatches ``_calibrating``, and fences
  against a heartbeat still frozen at CALIBRATING immediately restarting it.

Timers are MagicMocks under the mock node, so nothing fires on its own — each
test calls the timer callback directly, which is also what makes the 60 s cap
testable without waiting 60 s (``_calib_start_mono`` is pushed into the past).
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from jugglebot.protocol_config import BallButlerStates
from jugglebot import mocap_status as ms


# ── Harness ──────────────────────────────────────────────────────────────────

def _fake_iface(*, receiving=True, aligned=True, synced=True, markers=None):
    """A MocapInterface stand-in.

    The real one spins an asyncio thread that connects to QTM on construction,
    so it is replaced wholesale rather than patched method-by-method.
    """
    iface = MagicMock()
    iface.is_receiving.return_value = receiving
    iface.is_aligned = aligned
    iface.get_qtm_sync_status.return_value = {'synced': synced}
    if markers is None:
        markers = np.zeros((5, 4))
    iface.get_ball_butler_markers_base_frame.return_value = markers
    return iface


def _make_node(iface):
    import jugglebot.mocap_node as mn
    with patch.object(mn, 'MocapInterface', return_value=iface):
        return mn.MocapNode()


@pytest.fixture
def node():
    return _make_node(_fake_iface())


def _hb(state):
    from jugglebot_interfaces.msg import BallButlerHeartbeat
    msg = BallButlerHeartbeat()
    msg.state = int(state)
    msg.yaw_deg = 12.0
    return msg


def _kv(published_msg):
    return {kv.key: kv.value for kv in published_msg.values}


def _last_calibration(node):
    assert node.pub_calibration.published, 'no bb/calibration_result published'
    return node.pub_calibration.published[-1]


def _markers_with_visible(indices):
    """(5, 4) BB marker array with only *indices* resolved (rest NaN rows)."""
    arr = np.full((5, 4), np.nan)
    for i in indices:
        arr[i] = [100.0 + i, 200.0, 300.0, 0.5]
    return arr


# ════════════════════════════════════════════════════════════════
# Q1 — mocap/status
# ════════════════════════════════════════════════════════════════

def test_status_publishes_all_five_keys(node):
    node._publish_mocap_status()
    assert len(node.pub_mocap_status.published) == 1
    kv = _kv(node.pub_mocap_status.published[0])
    assert set(kv) == {
        ms.KEY_QTM_RECEIVING, ms.KEY_BB_MARKERS_VISIBLE,
        ms.KEY_MARKER3_VISIBLE, ms.KEY_ALIGNED, ms.KEY_QTM_SYNCED,
    }


def test_status_healthy_snapshot_is_ready_by_the_shared_predicate():
    """End-to-end on the CONTRACT, not just the fields: what this node
    publishes when all is well must be what mocap_status.evaluate() calls
    ready. A field renamed on one side and not the other would pass a
    key-by-key test and still leave every consumer refusing forever."""
    node = _make_node(_fake_iface(markers=_markers_with_visible(range(5))))
    node._publish_mocap_status()
    kv = _kv(node.pub_mocap_status.published[0])
    ready, code, detail = ms.evaluate(kv, age_s=0.0)
    assert ready is True, f'{code}: {detail}'


def test_status_reports_qtm_not_receiving():
    node = _make_node(_fake_iface(receiving=False))
    node._publish_mocap_status()
    msg = node.pub_mocap_status.published[0]
    assert _kv(msg)[ms.KEY_QTM_RECEIVING] == '0'
    from diagnostic_msgs.msg import DiagnosticStatus
    assert msg.level == DiagnosticStatus.ERROR


def test_status_level_ok_when_receiving(node):
    node._publish_mocap_status()
    from diagnostic_msgs.msg import DiagnosticStatus
    assert node.pub_mocap_status.published[0].level == DiagnosticStatus.OK


def test_status_counts_only_non_nan_markers():
    node = _make_node(_fake_iface(markers=_markers_with_visible([0, 2, 4])))
    node._publish_mocap_status()
    kv = _kv(node.pub_mocap_status.published[0])
    assert kv[ms.KEY_BB_MARKERS_VISIBLE] == '3'
    assert kv[ms.KEY_MARKER3_VISIBLE] == '1'   # index 2 IS Marker 3


def test_status_marker3_false_when_index_2_is_nan():
    node = _make_node(_fake_iface(markers=_markers_with_visible([0, 1, 3, 4])))
    node._publish_mocap_status()
    kv = _kv(node.pub_mocap_status.published[0])
    assert kv[ms.KEY_BB_MARKERS_VISIBLE] == '4'
    assert kv[ms.KEY_MARKER3_VISIBLE] == '0'


def test_status_handles_empty_marker_array():
    """MocapInterface returns ``np.empty((0, 4))`` when it has nothing; the
    count must be 0 and Marker 3 absent, not an IndexError."""
    node = _make_node(_fake_iface(markers=np.empty((0, 4))))
    node._publish_mocap_status()
    kv = _kv(node.pub_mocap_status.published[0])
    assert kv[ms.KEY_BB_MARKERS_VISIBLE] == '0'
    assert kv[ms.KEY_MARKER3_VISIBLE] == '0'


def test_status_reports_aligned_and_synced_flags():
    node = _make_node(_fake_iface(aligned=False, synced=False))
    node._publish_mocap_status()
    kv = _kv(node.pub_mocap_status.published[0])
    assert kv[ms.KEY_ALIGNED] == '0'
    assert kv[ms.KEY_QTM_SYNCED] == '0'


def test_status_publisher_does_not_disturb_existing_publications(node):
    """The spec's hard constraint: no existing mocap publication may change
    behaviour. The status timer must not consume markers, clear caches, or
    publish on any other topic."""
    node._publish_mocap_status()
    assert node.pub_mocap.published == []
    assert node.pub_bb_markers.published == []
    assert node.pub_rigid_bodies.published == []
    assert node.pub_clock_offset.published == []
    node.mocap.clear_markers.assert_not_called()
    node.mocap.clear_body_poses.assert_not_called()


def test_status_publisher_does_no_blocking_io(node):
    """Determinism doctrine (owner): a periodic callback reads snapshots only.

    Enforced by construction — the only interface calls allowed are the three
    pure cache reads. Anything else on this list would be a new I/O path
    smuggled into a 5 Hz timer.
    """
    node.mocap.reset_mock()   # drop the constructor's setup calls
    node._publish_mocap_status()
    called = {c[0] for c in node.mocap.method_calls}
    assert called <= {'is_receiving', 'get_qtm_sync_status',
                      'get_ball_butler_markers_base_frame'}


def test_status_survives_an_interface_exception(node):
    """Observability must never take the node down: a throwing interface logs
    and returns rather than propagating into the executor."""
    node.mocap.is_receiving.side_effect = RuntimeError('boom')
    node._publish_mocap_status()          # must not raise
    assert node.pub_mocap_status.published == []


# ════════════════════════════════════════════════════════════════
# Q5b — mid-sweep QTM dropout
# ════════════════════════════════════════════════════════════════

def _start_sweep(node):
    node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    assert node._calibrating is True
    assert node._calib_invalid is None


def test_dropout_mid_sweep_latches_invalid_and_publishes(node):
    _start_sweep(node)
    node.mocap.is_receiving.return_value = False
    node._check_calibration_health()

    assert node._calib_invalid == 'QTM_DROPOUT_MID_SWEEP'
    result = _last_calibration(node)
    assert result.success is False
    assert 'QTM_DROPOUT_MID_SWEEP' in result.message


def test_dropout_publishes_exactly_once(node):
    """Latched, not level-triggered: the health check runs at 5 Hz, so a
    re-publish per tick would spam the latched topic for the whole outage."""
    _start_sweep(node)
    node.mocap.is_receiving.return_value = False
    for _ in range(5):
        node._check_calibration_health()
    assert len(node.pub_calibration.published) == 1


def test_dropout_stops_marker_accumulation(node):
    """More points cannot repair an arc with a hole in it — and a growing dict
    would only make the garbage look better-supported to the solver."""
    _start_sweep(node)
    node.mocap.is_receiving.return_value = False
    node._check_calibration_health()
    node.mocap.is_receiving.return_value = True   # QTM comes back
    node._publish_mocap_data()
    assert all(not pts for pts in node._calib_data.values())


def test_dropout_stops_yaw_accumulation(node):
    _start_sweep(node)
    before = len(node._calib_yaw_readings)
    node.mocap.is_receiving.return_value = False
    node._check_calibration_health()
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    assert len(node._calib_yaw_readings) == before


def test_invalidated_window_skips_the_solver_at_the_exit_edge(node):
    """The exit edge must NOT run run_calibration: the named failure is already
    published, and a solver success would overwrite it on the latched topic
    with a plausible pose fitted to a fragment."""
    _start_sweep(node)
    node.mocap.is_receiving.return_value = False
    node._check_calibration_health()

    import jugglebot.mocap_node as mn
    with patch.object(mn, 'run_calibration') as solver:
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    solver.assert_not_called()
    assert node._calibrating is False
    assert _last_calibration(node).message.startswith('QTM_DROPOUT_MID_SWEEP')


def test_healthy_sweep_is_not_invalidated(node):
    _start_sweep(node)
    for _ in range(10):
        node._check_calibration_health()
    assert node._calib_invalid is None
    assert node.pub_calibration.published == []


def test_health_check_is_inert_outside_a_sweep(node):
    """QTM being down while BB is idle is not a calibration failure and must
    not publish one."""
    node.mocap.is_receiving.return_value = False
    node._check_calibration_health()
    assert node.pub_calibration.published == []
    assert node._calib_invalid is None


# ════════════════════════════════════════════════════════════════
# Q5c — collection timeout
# ════════════════════════════════════════════════════════════════

def _age_the_window(node, seconds):
    node._calib_start_mono -= seconds


def test_timeout_publishes_named_failure_and_unlatches(node):
    import jugglebot.mocap_node as mn
    _start_sweep(node)
    _age_the_window(node, mn.CALIBRATION_TIMEOUT_S + 1.0)
    node._check_calibration_health()

    assert node._calib_invalid == 'CALIBRATION_TIMEOUT'
    assert node._calibrating is False
    assert node._calib_data == {}
    result = _last_calibration(node)
    assert result.success is False
    assert 'CALIBRATION_TIMEOUT' in result.message


def test_window_just_under_the_cap_is_left_alone(node):
    import jugglebot.mocap_node as mn
    _start_sweep(node)
    _age_the_window(node, mn.CALIBRATION_TIMEOUT_S - 1.0)
    node._check_calibration_health()
    assert node._calibrating is True
    assert node.pub_calibration.published == []


def test_timeout_fences_a_heartbeat_still_frozen_at_calibrating(node):
    """The wedge that caused the timeout is usually still there. Without the
    fence the very next CALIBRATING heartbeat would restart the window and the
    node would publish a timeout every 60 s forever."""
    import jugglebot.mocap_node as mn
    _start_sweep(node)
    _age_the_window(node, mn.CALIBRATION_TIMEOUT_S + 1.0)
    node._check_calibration_health()

    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    assert node._calibrating is False
    assert node._calib_blocked is True


def test_fence_clears_on_any_non_calibrating_state(node):
    """...but the fence must not be permanent: once BB reports anything else
    the wedge has cleared and the next genuine sweep has to be collected."""
    import jugglebot.mocap_node as mn
    _start_sweep(node)
    _age_the_window(node, mn.CALIBRATION_TIMEOUT_S + 1.0)
    node._check_calibration_health()

    node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    assert node._calib_blocked is False
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    assert node._calibrating is True
    assert node._calib_invalid is None


def test_timeout_closes_a_window_that_already_dropped_out(node):
    """REGRESSION (the ordering defect this file was written against).

    The timeout is evaluated BEFORE the 'already invalidated' early return. Get
    that backwards and a sweep that loses QTM and then wedges at CALIBRATING
    stays ``_calibrating=True`` for the life of the process — and because the
    start edge requires ``not self._calibrating``, EVERY later calibration is
    silently ignored. Nothing else in the node can close that window.
    """
    import jugglebot.mocap_node as mn
    _start_sweep(node)
    node.mocap.is_receiving.return_value = False
    node._check_calibration_health()
    assert node._calib_invalid == 'QTM_DROPOUT_MID_SWEEP'

    _age_the_window(node, mn.CALIBRATION_TIMEOUT_S + 1.0)
    node._check_calibration_health()
    assert node._calibrating is False
    assert node._calib_blocked is True


def test_timeout_does_not_overwrite_a_more_specific_cause(node):
    """bb/calibration_result is LATCHED. Re-publishing CALIBRATION_TIMEOUT over
    an already-published QTM_DROPOUT_MID_SWEEP would replace the message that
    names the actual cause with the vaguer one, in the one place the operator
    (and the GUI) reads it."""
    import jugglebot.mocap_node as mn
    _start_sweep(node)
    node.mocap.is_receiving.return_value = False
    node._check_calibration_health()
    _age_the_window(node, mn.CALIBRATION_TIMEOUT_S + 1.0)
    node._check_calibration_health()

    assert len(node.pub_calibration.published) == 1
    assert 'QTM_DROPOUT_MID_SWEEP' in _last_calibration(node).message


# ════════════════════════════════════════════════════════════════
# Unchanged pre-existing behaviour (guard against F4 regressions)
# ════════════════════════════════════════════════════════════════

def test_clean_sweep_still_reaches_the_solver(node):
    """The normal path is untouched: start, collect, exit CALIBRATING → the
    solver runs."""
    import jugglebot.mocap_node as mn
    _start_sweep(node)
    with patch.object(mn.MocapNode, '_finalize_calibration') as fin:
        node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    fin.assert_called_once()
    assert node._calibrating is False


def test_bb_error_during_sweep_still_publishes_its_own_failure(node):
    _start_sweep(node)
    node._on_bb_heartbeat(_hb(BallButlerStates.ERROR))
    result = _last_calibration(node)
    assert result.success is False
    assert 'ERROR state' in result.message
