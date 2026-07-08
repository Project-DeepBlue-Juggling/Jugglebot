"""catch_coordinator_node tests (Phase 5) — feedback-topic swap.

The coordinator's accept/reject feedback moved from the dormant MPC process's ZMQ
:5559 SUB to the ``trajectory/target_feedback`` ROS topic published by
trajectory_node. These tests assert the swap (subscription present, ZMQ sub gone)
and that the feasibility-blacklist semantics are preserved unchanged.

ROS 2 is mocked by ``tests/ros/conftest.py``.
"""

from __future__ import annotations

import numpy as np

from jugglebot_interfaces.msg import TargetFeedback

from jugglebot.catch_coordinator_node import CatchCoordinatorNode


def _fb(accepted, arrival_time, code='TOO_FAST', reason='too tight', source='catch'):
    fb = TargetFeedback()
    fb.accepted = accepted
    fb.code = code
    fb.reason = reason
    fb.arrival_time = float(arrival_time)
    fb.source = source
    return fb


def _armed_node():
    """A coordinator with a submitted target primed for correlation."""
    node = CatchCoordinatorNode()
    node._last_submitted_ball_id = 7
    node._last_arrival_time = 100.0
    node._last_landing_position = np.array([50.0, 0.0, 574.3])
    return node


# ── Topic swap ────────────────────────────────────────────────

def test_subscribes_to_target_feedback_topic():
    node = CatchCoordinatorNode()
    assert 'trajectory/target_feedback' in node._subscriptions


def test_no_zmq_feedback_ipc():
    """The dormant MPC ZMQ :5559 feedback SUB is gone (swapped for the topic)."""
    node = CatchCoordinatorNode()
    assert not hasattr(node, '_feedback_ipc')


# ── Blacklist semantics preserved ─────────────────────────────

def test_rejection_increments_blacklist_count():
    node = _armed_node()
    node._on_target_feedback(_fb(False, 100.0))
    assert node._coordinator._rejection_counts.get(7) == 1


def test_repeated_rejections_blacklist_ball():
    node = _armed_node()
    thr = node._coordinator.blacklist_rejection_threshold
    for _ in range(thr):
        node._on_target_feedback(_fb(False, 100.0))
    assert 7 in node._coordinator._blacklist
    # The blacklist entry snapshots the submitted landing position.
    entry = node._coordinator._blacklist[7]
    assert np.allclose(entry.landing_position_snapshot, [50.0, 0.0, 574.3])


def test_acceptance_clears_rejection_count():
    node = _armed_node()
    node._on_target_feedback(_fb(False, 100.0))
    assert node._coordinator._rejection_counts.get(7) == 1
    node._on_target_feedback(_fb(True, 100.0, code='OK', reason=''))
    assert 7 not in node._coordinator._rejection_counts


def test_mismatched_arrival_time_ignored():
    node = _armed_node()
    node._on_target_feedback(_fb(False, 100.0 + 5.0))   # far outside the 0.1 s window
    assert node._coordinator._rejection_counts.get(7) is None


def test_feedback_ignored_without_submitted_ball():
    node = CatchCoordinatorNode()
    node._last_submitted_ball_id = None
    node._on_target_feedback(_fb(False, 100.0))   # no active submission → no-op
    assert node._coordinator._rejection_counts == {}


# ── Source filter + non-blacklist codes ───────────────────────

def test_timed_source_feedback_ignored():
    """trajectory/target_feedback carries timed-service decisions too; a timed-source
    reject must NOT touch the catch blacklist."""
    node = _armed_node()
    node._on_target_feedback(_fb(False, 100.0, code='TOO_FAST', source='timed'))
    assert node._coordinator._rejection_counts.get(7) is None


def test_stale_state_reject_not_blacklist_counted():
    """A STALE_STATE reject (the target's reachability was never evaluated) must not
    count toward the blacklist."""
    node = _armed_node()
    node._on_target_feedback(_fb(False, 100.0, code='STALE_STATE'))
    assert node._coordinator._rejection_counts.get(7) is None


def test_frozen_reject_not_blacklist_counted():
    """A FROZEN reject (a committed reach was held) must not count toward the
    blacklist."""
    node = _armed_node()
    node._on_target_feedback(_fb(False, 100.0, code='FROZEN'))
    assert node._coordinator._rejection_counts.get(7) is None


def test_workspace_reject_still_blacklist_counted():
    """A feasibility-class reject (WORKSPACE) DOES count — the position is genuinely
    unreachable."""
    node = _armed_node()
    node._on_target_feedback(_fb(False, 100.0, code='WORKSPACE'))
    assert node._coordinator._rejection_counts.get(7) == 1
