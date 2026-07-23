"""catch_coordinator_node tests (Phase 5) — feedback-topic swap.

The coordinator's accept/reject feedback moved from the dormant MPC process's ZMQ
:5559 SUB to the ``trajectory/target_feedback`` ROS topic published by
trajectory_node. These tests assert the swap (subscription present, ZMQ sub gone)
and that the feasibility-blacklist semantics are preserved unchanged.

ROS 2 is mocked by ``tests/ros/conftest.py``.
"""

from __future__ import annotations

import time
import types

import numpy as np
import pytest

from std_msgs.msg import Bool, Float64
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
    """With the catch-armed latch UP (a reload in progress) a catchable ball ARMS the
    hand catch — but NEVER dispatches a prime from the balls path: a kind-3
    smooth-move sent while a catch sequence is live clears the Platform Teensy's
    armed catch stroke (last-writer-wins; 3/6 strokes lost to that race,
    2026-07-23). Priming belongs to the armed edge + the off-path retry timer."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    assert node._catch_armed is True
    primed, armed = [], []
    monkeypatch.setattr(node, '_prime_hand', lambda: primed.append(1))
    monkeypatch.setattr(node, '_arm_hand_catch',
                        lambda d, v: armed.append((d, v)) or True)
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time: _catchable_cmd())
    node._on_balls(_balls_msg())
    assert primed == []                        # NO prime from the balls path
    assert len(armed) == 1
    assert armed[0][1] == pytest.approx(1.2)   # event_vel carried (scale 1.0)
    assert node._hand_traj_armed_for_ball == 5
    # The balls tick stamped the quiet window that suppresses the retry timer.
    assert time.perf_counter() - node._last_cmd_mono < 1.0


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


# ── catch/vel_scale — the operator's per-attempt catch-speed knob ─────────────

def test_vel_scale_scales_armed_event_velocity(monkeypatch):
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    node._on_vel_scale(Float64(data=0.5))
    armed = []
    monkeypatch.setattr(node, '_arm_hand_catch',
                        lambda d, v: armed.append(v) or True)
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time: _catchable_cmd())
    node._on_balls(_balls_msg())
    assert armed == [pytest.approx(0.6)]       # 1.2 × 0.5


def test_vel_scale_clamped_to_safe_range():
    """Below ~0.3 the Teensy's windup budget silently drops the stroke; above 1.5
    the event-velocity ceiling binds — out-of-range values are clamped, loudly."""
    node = CatchCoordinatorNode()
    node._on_vel_scale(Float64(data=0.05))
    assert node._catch_vel_scale == pytest.approx(0.3)
    node._on_vel_scale(Float64(data=9.0))
    assert node._catch_vel_scale == pytest.approx(1.5)


def test_vel_scale_resets_on_disarm():
    """One reload's tuning value must never leak into the next attempt."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    node._on_vel_scale(Float64(data=0.7))
    assert node._catch_vel_scale == pytest.approx(0.7)
    node._on_catch_armed(Bool(data=False))
    assert node._catch_vel_scale == pytest.approx(1.0)


def test_scaled_event_vel_reclamped_to_teensy_bounds(monkeypatch):
    """scale × raw must stay inside the Teensy's [0.3, 7.0] validation range."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    node._on_vel_scale(Float64(data=1.5))
    armed = []
    monkeypatch.setattr(node, '_arm_hand_catch',
                        lambda d, v: armed.append(v) or True)
    cmd = _catchable_cmd()
    cmd.event_vel_mps = 6.0                    # 6.0 × 1.5 = 9.0 → clamp 7.0
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time: cmd)
    node._on_balls(_balls_msg())
    assert armed == [pytest.approx(7.0)]


# ── prime-retry timer (off the balls path) ────────────────────────────────────

def test_prime_retry_fires_when_armed_unprimed_and_quiet(monkeypatch):
    node = CatchCoordinatorNode()
    primed = []
    monkeypatch.setattr(node, '_prime_hand', lambda: primed.append(1))
    node._catch_armed = True
    node._hand_primed = False
    node._last_cmd_mono = 0.0                  # far in the past → quiet
    node._prime_retry_tick()
    assert primed == [1]


def test_prime_retry_suppressed_while_catch_sequence_live(monkeypatch):
    """A retry prime during a live catch sequence is the exact race that erased
    3/6 catch strokes on 2026-07-23 (kind-3 clears the Teensy's armed catch)."""
    node = CatchCoordinatorNode()
    primed = []
    monkeypatch.setattr(node, '_prime_hand', lambda: primed.append(1))
    node._catch_armed = True
    node._hand_primed = False
    node._last_cmd_mono = time.perf_counter()  # a catch cmd JUST went out
    node._prime_retry_tick()
    assert primed == []


def test_prime_retry_noop_when_primed_or_disarmed(monkeypatch):
    node = CatchCoordinatorNode()
    primed = []
    monkeypatch.setattr(node, '_prime_hand', lambda: primed.append(1))
    node._catch_armed = False
    node._prime_retry_tick()
    node._catch_armed = True
    node._hand_primed = True
    node._prime_retry_tick()
    assert primed == []


# ── announcement pre-tilt ─────────────────────────────────────────────────────

def _announcement(target_id='jugglebot', landing_z=809.08, sec=100, nanosec=0):
    return types.SimpleNamespace(
        target_id=target_id,
        landing_position=types.SimpleNamespace(x=0.0, y=0.0, z=landing_z),
        landing_velocity=types.SimpleNamespace(x=-1000.0, y=0.0, z=-4800.0),
        landing_time=types.SimpleNamespace(sec=sec, nanosec=nanosec),
    )


def test_announcement_pretilt_published_while_armed():
    """OUR announcement, while armed, drives a one-shot predicted catch target —
    the platform settles into the receive tilt during the ~3 s countdown instead
    of reaching mid-flight (2026-07-23: the reactive reach was only ~95% settled
    at contact). The pose math is single-sourced with the reactive path."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    n0 = len(node._dyn_target_pub.published)
    node._on_throw_announcement(_announcement())
    assert len(node._dyn_target_pub.published) == n0 + 1
    msg = node._dyn_target_pub.published[-1]
    # Stow-relative pose near the active hold, receive tilt present (non-identity).
    assert 150.0 < msg.target_pos.z < 200.0
    tilt = float(np.hypot(msg.target_quat.x, msg.target_quat.y))
    assert tilt > 1e-3
    # Correlation state untouched: the synthetic target has no tracker ball and
    # must never feed the blacklist or suppress the real ball's hand-arm.
    assert node._last_submitted_ball_id is None


def test_announcement_pretilt_gated_on_armed_and_target():
    node = CatchCoordinatorNode()
    n0 = len(node._dyn_target_pub.published)
    node._on_throw_announcement(_announcement())               # disarmed → skip
    assert len(node._dyn_target_pub.published) == n0
    node._on_catch_armed(Bool(data=True))
    node._on_throw_announcement(_announcement(target_id='someone_else'))
    assert len(node._dyn_target_pub.published) == n0           # not our ball


def test_arm_one_shot_latched_only_on_dispatch(monkeypatch):
    """A service-not-ready arm attempt must be RETRIED next tick — the old code
    latched the one-shot unconditionally, permanently suppressing the retry."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    monkeypatch.setattr(node, '_arm_hand_catch', lambda d, v: False)  # not dispatched
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time: _catchable_cmd())
    node._on_balls(_balls_msg())
    assert node._hand_traj_armed_for_ball is None              # NOT latched
    monkeypatch.setattr(node, '_arm_hand_catch', lambda d, v: True)   # dispatched
    node._on_balls(_balls_msg())
    assert node._hand_traj_armed_for_ball == 5                 # latched now


def test_prime_retry_blocked_while_stroke_armed(monkeypatch):
    """AUDIT (2026-07-23): the quiet window anchors to the last EMITTED command,
    which stops ~0.3 s before landing — a slow disarm round-trip could unblock the
    retry while the armed stroke (or its settle) is still live. An armed one-shot
    blocks the retry outright; it clears on disarm and on arm failure."""
    node = CatchCoordinatorNode()
    primed = []
    monkeypatch.setattr(node, '_prime_hand', lambda: primed.append(1))
    node._catch_armed = True
    node._hand_primed = False
    node._last_cmd_mono = 0.0                  # quiet window long expired
    node._hand_traj_armed_for_ball = 5         # but a stroke is ARMED
    node._prime_retry_tick()
    assert primed == []
    node._hand_traj_armed_for_ball = None      # disarm edge / arm failure cleared it
    node._prime_retry_tick()
    assert primed == [1]


def test_announcement_untagged_target_does_not_pretilt():
    """AUDIT: the reload path always names the target, so an announcement with an
    EMPTY target_id is not ours — it must not move the platform."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    n0 = len(node._dyn_target_pub.published)
    node._on_throw_announcement(_announcement(target_id=''))
    assert len(node._dyn_target_pub.published) == n0
