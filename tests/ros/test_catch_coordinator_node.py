"""catch_coordinator_node tests (Phase 5) — feedback-topic swap.

The coordinator's accept/reject feedback moved from the dormant MPC process's ZMQ
:5559 SUB to the ``trajectory/target_feedback`` ROS topic published by
trajectory_node. These tests assert the swap (subscription present, ZMQ sub gone)
and that the feasibility-blacklist semantics are preserved unchanged.

ROS 2 is mocked by ``tests/ros/conftest.py``.
"""

from __future__ import annotations

import types

import numpy as np
import pytest

from std_msgs.msg import Bool
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


# ── Catch-armed latch gates hand actuation (Phase 2) ──────────
# Without CATCH mode as the implicit "operator intends to catch" signal, the hand
# prime/arm is gated on the reload action's catch-armed latch (catch/armed) so it
# actuates ONLY during a reload — never on a stray tracked ball.


def _catchable_cmd(ball_id=5):
    """A coordinator command that would drive a hand prime + arm."""
    return types.SimpleNamespace(
        ball_id=ball_id,
        target_pos=np.array([0.0, 0.0, 809.08]),
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        target_vel=np.array([0.0, 0.0, 0.0]),
        landing_time=5.0,          # current_time is 0.0 (MockClock) → event_delay 5.0 s
        arm_hand=True,
        event_vel_mps=1.2,
    )


def _balls_msg():
    return types.SimpleNamespace(balls=[])


def test_subscribes_to_catch_armed():
    node = CatchCoordinatorNode()
    assert 'catch/armed' in node._subscriptions
    assert node._catch_armed is False        # disarmed at construction


def test_hand_not_actuated_when_disarmed(monkeypatch):
    """A catchable ball with the latch DOWN (no reload) must not prime or arm the hand."""
    node = CatchCoordinatorNode()
    assert node._catch_armed is False
    primed, armed = [], []
    monkeypatch.setattr(node, '_prime_hand', lambda: primed.append(1))
    monkeypatch.setattr(node, '_arm_hand_catch', lambda d, v: armed.append((d, v)))
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time: _catchable_cmd())
    node._on_balls(_balls_msg())
    assert primed == [] and armed == []      # latch down → hand untouched


def test_hand_actuated_when_armed(monkeypatch):
    """With the catch-armed latch UP (a reload in progress) the same catchable ball
    primes + arms the hand — the reactive-fire timing stays in the coordinator."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    assert node._catch_armed is True
    primed, armed = [], []
    monkeypatch.setattr(node, '_prime_hand', lambda: primed.append(1))
    monkeypatch.setattr(node, '_arm_hand_catch', lambda d, v: armed.append((d, v)))
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time: _catchable_cmd())
    node._on_balls(_balls_msg())
    assert primed == [1]
    assert len(armed) == 1
    assert armed[0][1] == pytest.approx(1.2)   # event_vel carried through


def test_disarm_resets_hand_one_shots():
    """Disarming (reload ended / aborted) resets the prime/arm one-shots so the NEXT
    reload re-primes + re-arms from a clean state."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    node._hand_primed = True
    node._hand_traj_armed_for_ball = 5
    node._on_catch_armed(Bool(data=False))
    assert node._catch_armed is False
    assert node._hand_primed is False
    assert node._hand_traj_armed_for_ball is None


def test_catch_armed_same_state_is_noop():
    """A repeat arm (no edge) does not reset the one-shots — only a true disarm edge does."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    node._hand_primed = True
    node._on_catch_armed(Bool(data=True))    # same state → no reset
    assert node._hand_primed is True


def test_arm_rising_edge_primes_hand_immediately(monkeypatch):
    """The ARM edge primes the hand THEN AND THERE — it must not wait for a
    catchable ball to appear on ``balls``. Hardware 2026-07-23: the bottom→top
    smooth-move is ~0.7 s against a 0.878 s flight, so a ball-triggered prime is
    a coin flip, and a hand still mid-prime at fire time makes the Teensy
    silently drop the whole catch stroke (its prelude time-budget check). This
    edge-prime serves both the reload action's catch/armed publish and the
    manual static-catch recipe (publish catch/armed true, throw by hand)."""
    node = CatchCoordinatorNode()
    primed = []
    monkeypatch.setattr(node, '_prime_hand', lambda: primed.append(1))
    node._on_catch_armed(Bool(data=True))
    assert primed == [1]                      # primed on the edge, no ball involved
    # A repeat arm (no edge) does not re-prime.
    node._on_catch_armed(Bool(data=True))
    assert primed == [1]
