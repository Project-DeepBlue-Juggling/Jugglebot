"""catch_coordinator_node tests (Phase 5) — feedback-topic swap.

The coordinator's accept/reject feedback moved from the dormant MPC process's ZMQ
:5559 SUB to the ``trajectory/target_feedback`` ROS topic published by
trajectory_node. These tests assert the swap (subscription present, ZMQ sub gone)
and that the feasibility-blacklist semantics are preserved unchanged.

ROS 2 is mocked by ``tests/ros/conftest.py``.
"""

from __future__ import annotations

import math
import time
import types

import numpy as np
import pytest

from std_msgs.msg import Bool, Float64
from jugglebot_interfaces.msg import TargetFeedback

import jugglebot.hardware_config as hw
from jugglebot.catch_coordinator_node import CatchCoordinatorNode
from jugglebot.motion.trajectory import hand_stroke


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
                        lambda balls, current_time, exclude_ids=None: _catchable_cmd())
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
                        lambda balls, current_time, exclude_ids=None: _catchable_cmd())
    node._on_balls(_balls_msg())
    assert primed == []                        # NO prime from the balls path
    assert len(armed) == 1
    # event_vel carried, scaled by the config default (0.8 locked in 2026-07-23)
    assert armed[0][1] == pytest.approx(1.2 * hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)
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
                        lambda balls, current_time, exclude_ids=None: _catchable_cmd())
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
    """One reload's tuning value must never leak into the next attempt — the
    disarm edge restores the config default (0.8, locked in 2026-07-23)."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    node._on_vel_scale(Float64(data=0.7))
    assert node._catch_vel_scale == pytest.approx(0.7)
    node._on_catch_armed(Bool(data=False))
    assert node._catch_vel_scale == pytest.approx(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)


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
                        lambda balls, current_time, exclude_ids=None: cmd)
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
                        lambda balls, current_time, exclude_ids=None: _catchable_cmd())
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


def test_pretilt_arrival_scheduled_early():
    """The pre-tilt target must schedule its arrival _PRETILT_EARLY_S (1.5 s)
    BEFORE the predicted landing. The old arrival == landing made trajectory_node
    span the whole announce→land window with one min-jerk reach completing AT
    contact — third sitting (2026-07-23): tilt still >1° off until 0.24–0.49 s
    before landing on all 12 attempts."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    node._on_throw_announcement(_announcement())          # landing at ros t=100
    msg = node._dyn_target_pub.published[-1]
    landing_perf = 100.0 + node._ros_to_perf_offset
    assert msg.arrival_time == pytest.approx(landing_perf - 1.5, abs=0.05)


def test_pretilt_arrival_clamped_to_min_lead():
    """A short-countdown announcement must still get a feasible profiled traverse:
    arrival is clamped to now + _PRETILT_MIN_LEAD_S (1.0 s), never demanding a
    violent reach — and never scheduled after the landing itself."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    before = time.perf_counter()
    node._on_throw_announcement(_announcement(sec=2))     # landing only ~2 s out
    msg = node._dyn_target_pub.published[-1]
    landing_perf = 2.0 + node._ros_to_perf_offset
    assert msg.arrival_time == pytest.approx(before + 1.0, abs=0.1)
    assert msg.arrival_time < landing_perf


# ── anti-stutter prime in-flight window ───────────────────────────────────────

def test_prime_retry_suppressed_during_prime_ascent(monkeypatch):
    """The third sitting's stutter: the 0.5 s retry tick re-dispatched 0.5 s into
    a ~0.8 s ascent whose ack had failed, rebuilding the Teensy profile mid-move
    (velocity reversal to −4 rev/s) on 5/12 attempts. No re-prime may be
    dispatched inside _PRIME_INFLIGHT_S of the last dispatch; after the window a
    re-dispatch at top is a Teensy no-op, so a lost dispatch still recovers."""
    node = CatchCoordinatorNode()
    primed = []
    monkeypatch.setattr(node, '_prime_hand', lambda: primed.append(1))
    node._catch_armed = True
    node._hand_primed = False
    node._last_cmd_mono = 0.0                       # quiet window clear
    node._prime_dispatch_mono = time.perf_counter()  # a prime JUST dispatched
    node._prime_retry_tick()
    assert primed == []                             # ascent protected
    node._prime_dispatch_mono = time.perf_counter() - 1.5  # ascent over
    node._prime_retry_tick()
    assert primed == [1]                            # recovery retry allowed


def test_edge_prime_skipped_while_prime_ascent_inflight(monkeypatch):
    """The reload coordinator primes at CHECKING ~0.1 s before the catch/armed
    edge reaches this node — the pair restarted a just-started ascent on 3/12
    third-sitting attempts. The edge prime defers to a fresh dispatch window;
    the retry tick re-primes after the window if the ascent never happened."""
    node = CatchCoordinatorNode()
    primed = []
    monkeypatch.setattr(node, '_prime_hand', lambda: primed.append(1))
    node._prime_dispatch_mono = time.perf_counter()  # reload's prime just went out
    node._on_catch_armed(Bool(data=True))
    assert primed == []                              # live ascent not restarted
    node._on_catch_armed(Bool(data=False))
    node._prime_dispatch_mono = 0.0                  # no recent dispatch
    node._on_catch_armed(Bool(data=True))
    assert primed == [1]                             # normal edge prime intact


def test_prime_inflight_window_covers_the_commanded_prime_ascent():
    """``_PRIME_INFLIGHT_S`` is sized from the PRIME ASCENT DURATION, so it is a
    window the prime rev moves — and it does not name the constant, which is why
    a grep-of-the-constant sweep misses it.

    Added 2026-07-26 with the prime move (9.858 → the derived stroke top
    9.9594 rev): the commanded full-stroke ascent lengthened 0.7544 → 0.7583 s
    (``T = sqrt(Δ·QUINTIC_S2_MAX/A)``, ``Trajectory.h:257``). That is nowhere
    near 1.2 s, but nothing pinned the relationship, so a later prime raise or a
    Phase-4 duration-formula change could push the commanded ascent past the
    window silently.

    **What that would cost.** The retry tick at ``catch_coordinator_node.py:717``
    would fire mid-ascent and re-dispatch a kind-3, which rebuilds the Teensy
    profile from the live position at ``v(0) = 0`` and yanks the moving hand
    backwards — the 2026-07-23 stutter (5/12 ascents stalled 60-70 ms with
    velocity reversals to −4 rev/s).

    Pinned against the COMMANDED duration with 1.5x of headroom, not against the
    0.68-1.05 s OBSERVED band the constant's comment cites: the observed upper
    bound already includes dispatch and settle time the model does not claim to
    cover, so 1.2 s vs 1.05 s (0.15 s, 12.5 % of the window) is the real bench
    margin and belongs in the comment rather than in an assertion that would go
    red on ordinary telemetry scatter.
    """
    from jugglebot.catch_coordinator_node import (_PRIME_INFLIGHT_S,
                                                  _PRIME_RETRY_QUIET_S)
    ascent = hand_stroke.smooth_move_duration_s(
        float(hw.JB_OP_HAND_CATCH_PRIME_REV))
    assert ascent == pytest.approx(0.7583, abs=1e-3)
    assert _PRIME_INFLIGHT_S >= 1.5 * ascent
    # ...and against ANY commanded prime, not just the rest-to-rest one.  Phase 4
    # made the prelude velocity-continuous, so a prime dispatched into a live
    # retract (which never stamps _prime_dispatch_mono, so it is not suppressed)
    # is seeded with the hand's live descent velocity and takes LONGER: at the
    # retract's own peak, start 5.04 rev at -24.6 rev/s, the accel-limited
    # solution is 1.206 s — past this window outright.  The firmware bounds it
    # (Trajectory.h::smoothMoveMaxDuration = the longest rest-to-rest move the
    # stroke admits), so the window still covers every profile the Teensy can
    # emit; this asserts the two have not drifted apart.  Without the cap the
    # bound below is 1.206 s and this fails, which is the point.
    cap = math.sqrt(float(hw.GEOM_HAND_MOTOR_MAX_POSITION_REVS)
                    * float(hw.TEENSY_TRAJ_QUINTIC_S2_MAX)
                    / float(hw.TEENSY_TRAJ_MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2))
    assert cap == pytest.approx(0.80054, abs=1e-4)
    assert _PRIME_INFLIGHT_S >= cap, (
        'the in-flight window no longer covers the longest hand move the '
        'firmware can command')
    # And the quiet window — which guards the kind-3-clobbers-a-live-catch path —
    # must not be shorter than the in-flight window it composes with.
    assert _PRIME_RETRY_QUIET_S >= _PRIME_INFLIGHT_S


def test_prime_dispatched_topic_stamps_window():
    """catch/prime_dispatched (published by the reload coordinator on every
    ACTION_PRIME_HAND dispatch) stamps the same window — the two prime owners
    cannot see each other's service calls."""
    node = CatchCoordinatorNode()
    assert 'catch/prime_dispatched' in node._subscriptions
    assert node._prime_dispatch_mono == 0.0
    node._on_prime_dispatched(Bool(data=True))
    assert (time.perf_counter() - node._prime_dispatch_mono) < 0.5


def test_prime_hand_stamps_dispatch_window():
    """This node's own prime dispatch stamps the window too (on DISPATCH, not on
    the ack — failed acks have been observed with the frame still transmitted)."""
    node = CatchCoordinatorNode()
    assert node._prime_dispatch_mono == 0.0
    node._prime_hand()
    assert (time.perf_counter() - node._prime_dispatch_mono) < 0.5


# ── catch/prime_hold — the toss coordinator's prime-suppression gate ──────────
# From toss PREPARE to terminal the ball rides the hand at the stroke bottom;
# an auto-prime (kind-3 ascent) mid-toss would carry the ball-laden hand up and
# clear an armed throw stroke on the Teensy's last-writer-wins queue. The hold
# gates ONLY this node's prime dispatch paths (armed-edge + retry tick); the
# catch arm and all other behaviour are untouched, and the absent-topic default
# is bit-identical to the hardware-proven reload path.


def test_prime_hold_absent_topic_defaults_false(monkeypatch):
    """No catch/prime_hold ever published (every reload today): the flag is
    False and the armed-edge prime fires exactly as the reload-path tests pin —
    the gate is invisible when the topic is absent."""
    node = CatchCoordinatorNode()
    assert 'catch/prime_hold' in node._subscriptions
    assert node._prime_hold is False
    primed = []
    monkeypatch.setattr(node, '_prime_hand', lambda: primed.append(1))
    node._on_catch_armed(Bool(data=True))
    assert primed == [1]


def test_prime_hold_true_before_armed_suppresses_edge_and_retry(monkeypatch):
    """The toss choreography: prime_hold True at PREPARE entry, BEFORE
    catch/armed rises. The armed-edge prime is suppressed AND the 0.5 s retry
    tick never re-primes while the hold is up."""
    node = CatchCoordinatorNode()
    primed = []
    monkeypatch.setattr(node, '_prime_hand', lambda: primed.append(1))
    node._on_prime_hold(Bool(data=True))       # PREPARE: hold raised before armed
    node._on_catch_armed(Bool(data=True))
    assert primed == []                        # edge prime suppressed
    # Retry-tick preconditions all clear (armed, unprimed, quiet window expired,
    # no ascent in flight) — the hold alone must keep suppressing.
    assert node._hand_primed is False
    node._last_cmd_mono = 0.0
    node._prime_dispatch_mono = 0.0
    node._prime_retry_tick()
    node._prime_retry_tick()
    assert primed == []


def test_prime_hold_does_not_gate_catch_arm(monkeypatch):
    """prime_hold gates ONLY the prime dispatch paths: with the hold raised the
    kind-1 catch ARM still dispatches normally — the toss catch depends on it
    (hand-stroke catch timing stays tracker-driven through this node)."""
    node = CatchCoordinatorNode()
    node._on_prime_hold(Bool(data=True))
    node._on_catch_armed(Bool(data=True))
    armed = []
    monkeypatch.setattr(node, '_arm_hand_catch',
                        lambda d, v: armed.append((d, v)) or True)
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None: _catchable_cmd())
    node._on_balls(_balls_msg())
    assert len(armed) == 1
    assert node._hand_traj_armed_for_ball == 5   # one-shot latched as normal


def test_prime_hold_release_reenables_priming(monkeypatch):
    """prime_hold False again (toss terminal): priming is re-enabled — a
    still-armed unprimed node's retry tick recovers, and the next armed edge
    primes normally."""
    node = CatchCoordinatorNode()
    primed = []
    monkeypatch.setattr(node, '_prime_hand', lambda: primed.append(1))
    node._on_prime_hold(Bool(data=True))
    node._on_catch_armed(Bool(data=True))
    assert primed == []                        # suppressed during the toss
    # Release while still armed: the 0.5 s retry tick recovers the prime.
    node._on_prime_hold(Bool(data=False))
    node._prime_retry_tick()
    assert primed == [1]
    # And a fresh armed edge primes normally again.
    node._on_catch_armed(Bool(data=False))
    node._on_catch_armed(Bool(data=True))
    assert primed == [1, 1]


def test_prime_hold_survives_disarm_stale_true_fails_safe(monkeypatch):
    """The flag is owned by its publisher and never reset locally: a stale True
    (a toss that died before terminal) keeps failing SAFE — no auto-prime on the
    next armed edge; the reload action primes proactively itself."""
    node = CatchCoordinatorNode()
    primed = []
    monkeypatch.setattr(node, '_prime_hand', lambda: primed.append(1))
    node._on_prime_hold(Bool(data=True))
    node._on_catch_armed(Bool(data=True))
    node._on_catch_armed(Bool(data=False))     # disarm does NOT reset the flag
    assert node._prime_hold is True
    node._on_catch_armed(Bool(data=True))
    assert primed == []                        # suppressed until False is published


# ── catch/pretilt_hold — the Tier-8b toss's platform pre-tilt-suppression gate ─
# The stock announcement pre-tilt (arrival clamped to ~now + 1 s) would complete
# the A→B translate + un-tilt BEFORE a toss releases (announced >= 1 s pre-release)
# — aim destroyed, moving platform under a seated ball mid-windup. pretilt_hold
# suppresses ONLY the platform pre-tilt PUBLISH while still latching the
# announcement for the hand-arm window + open-loop freeze; the toss coordinator
# publishes the ONE deferred A→B reach at release. Absent topic = the reload path
# bit-identical; the flag is publisher-owned (never reset locally) and stale-True
# fails DEGRADED-BUT-SAFE (a reload loses only its pre-tilt; the platform holds).


def test_pretilt_hold_absent_topic_defaults_false():
    """No catch/pretilt_hold ever published (every reload today): the flag is
    False, the subscription exists, and OUR announcement drives the platform
    pre-tilt exactly as the reload-path tests pin — the gate is invisible when
    the topic is absent."""
    node = CatchCoordinatorNode()
    assert 'catch/pretilt_hold' in node._subscriptions
    assert node._pretilt_hold is False
    node._on_catch_armed(Bool(data=True))
    n0 = len(node._dyn_target_pub.published)
    node._on_throw_announcement(_announcement())
    assert len(node._dyn_target_pub.published) == n0 + 1        # pre-tilt published
    assert node._pretilt_cmd is not None


def test_pretilt_hold_true_suppresses_platform_but_latches_and_arms(monkeypatch):
    """The Tier-8b toss: pretilt_hold True at PREPARE, BEFORE the announcement.
    The announcement publishes NO platform target and caches _pretilt_cmd = None
    (so a balls tick's _republish_pretilt no-ops), BUT still latches
    _announcement_seen + _announced_landing_time (the open-loop freeze + the
    hand-arm window keep working) — and the hand-arm still fires tracker-driven."""
    node = CatchCoordinatorNode()
    node._on_pretilt_hold(Bool(data=True))       # PREPARE: raised before armed
    node._on_catch_armed(Bool(data=True))
    n0 = len(node._dyn_target_pub.published)
    node._on_throw_announcement(_announcement())               # landing ros t=100
    # NO platform target from the announcement; latched for hand-arm + open-loop.
    assert len(node._dyn_target_pub.published) == n0
    assert node._announcement_seen is True
    assert node._announced_landing_time == pytest.approx(100.0)
    assert node._pretilt_cmd is None
    # A balls tick under open-loop: _republish_pretilt no-ops (_pretilt_cmd None)
    # — the toss coordinator owns the platform reach — but the hand-arm fires.
    armed = []
    monkeypatch.setattr(node, '_arm_hand_catch',
                        lambda d, v: armed.append((d, v)) or True)
    cmd = _catchable_cmd()
    cmd.landing_time = 100.0                                    # within the arm window
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None: cmd)
    node._on_balls(_balls_msg())
    assert len(node._dyn_target_pub.published) == n0           # still no platform target
    assert len(armed) == 1                                     # hand-arm reactive fire intact
    assert node._last_submitted_ball_id is None                # reactive correlation dormant


def test_pretilt_hold_does_not_gate_edge_prime(monkeypatch):
    """pretilt_hold gates ONLY the platform pre-tilt publish — NOT the armed-edge
    hand prime (that is prime_hold's job) nor the catch arm. With ONLY pretilt_hold
    up, the armed edge still primes; in production the toss raises BOTH gates."""
    node = CatchCoordinatorNode()
    primed = []
    monkeypatch.setattr(node, '_prime_hand', lambda: primed.append(1))
    node._on_pretilt_hold(Bool(data=True))
    node._on_catch_armed(Bool(data=True))
    assert primed == [1]                          # pretilt_hold does not gate priming


def test_pretilt_hold_release_reenables_pretilt():
    """pretilt_hold False again (toss terminal, or a reload after a toss): OUR
    announcement drives the platform pre-tilt normally again."""
    node = CatchCoordinatorNode()
    node._on_pretilt_hold(Bool(data=True))
    node._on_pretilt_hold(Bool(data=False))
    node._on_catch_armed(Bool(data=True))
    n0 = len(node._dyn_target_pub.published)
    node._on_throw_announcement(_announcement())
    assert len(node._dyn_target_pub.published) == n0 + 1
    assert node._pretilt_cmd is not None


def test_pretilt_hold_survives_disarm_stale_true_degrades_safe():
    """The flag is publisher-owned and never reset locally: a stale True (a toss
    that died before terminal) keeps suppressing the platform pre-tilt — a reload
    announcement then loses only its pre-tilt (the platform holds; the hand-arm
    stays tracker-driven), DEGRADED-BUT-SAFE, never a hazard."""
    node = CatchCoordinatorNode()
    node._on_pretilt_hold(Bool(data=True))
    node._on_catch_armed(Bool(data=True))
    node._on_catch_armed(Bool(data=False))        # disarm does NOT reset pretilt_hold
    assert node._pretilt_hold is True
    node._on_catch_armed(Bool(data=True))
    n0 = len(node._dyn_target_pub.published)
    node._on_throw_announcement(_announcement())
    assert len(node._dyn_target_pub.published) == n0    # still suppressed
    assert node._announcement_seen is True              # but still latched (open-loop)
    assert node._pretilt_cmd is None


def test_pretilt_hold_forces_open_loop_independent_of_reload_flag(monkeypatch):
    """FIX-2: the toss's reactive-platform suppression is SELF-CONTAINED, NOT
    co-dependent on JB_OP_RELOAD_PLATFORM_OPEN_LOOP. With the reload flag forced
    FALSE, a held 8b toss (pretilt_hold True) must STILL suppress the reactive
    per-ball catch/dynamic_target — otherwise the tracker-derived target would
    compete with the toss coordinator's deferred A->B reach mid-flight. The
    hand-arm still engages (its timing stays tracker-driven)."""
    monkeypatch.setattr(hw, 'JB_OP_RELOAD_PLATFORM_OPEN_LOOP', False)
    node = CatchCoordinatorNode()
    node._on_pretilt_hold(Bool(data=True))       # PREPARE: raised before armed
    node._on_catch_armed(Bool(data=True))
    node._on_throw_announcement(_announcement())               # latched, no platform target
    assert node._announcement_seen is True
    assert node._pretilt_cmd is None
    n0 = len(node._dyn_target_pub.published)
    armed = []
    monkeypatch.setattr(node, '_arm_hand_catch',
                        lambda d, v: armed.append((d, v)) or True)
    cmd = _catchable_cmd()
    cmd.landing_time = 100.0                                    # within the arm window
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None: cmd)
    node._on_balls(_balls_msg())
    assert len(node._dyn_target_pub.published) == n0           # NO reactive platform target
    assert len(armed) == 1                                     # hand-arm engaged
    assert node._last_submitted_ball_id is None                # reactive correlation dormant


def test_reactive_reload_path_unchanged_when_pretilt_hold_false(monkeypatch):
    """FIX-2 regression: with pretilt_hold False the open-loop condition reduces
    EXACTLY to the pre-existing `flag and armed and announcement_seen`. With the
    reload flag forced FALSE (the pre-existing NON-open-loop reactive reload) the
    per-ball platform target IS still published — the toss suppression must not
    leak into an ordinary reactive reload."""
    monkeypatch.setattr(hw, 'JB_OP_RELOAD_PLATFORM_OPEN_LOOP', False)
    node = CatchCoordinatorNode()
    assert node._pretilt_hold is False
    node._on_catch_armed(Bool(data=True))
    node._on_throw_announcement(_announcement())               # flag off ⇒ reactive refines
    n0 = len(node._dyn_target_pub.published)
    monkeypatch.setattr(node, '_arm_hand_catch', lambda d, v: True)
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None: _catchable_cmd())
    node._on_balls(_balls_msg())
    assert len(node._dyn_target_pub.published) == n0 + 1       # reactive platform target published
    assert node._dyn_target_pub.published[-1].target_pos.z == pytest.approx(809.08)
    assert node._last_submitted_ball_id == 5                   # reactive correlation stamped


# ── open-loop reload platform (JB_OP_RELOAD_PLATFORM_OPEN_LOOP) ────────────────
# Once OUR throw is announced during an armed reload, the platform holds the
# announcement pre-tilt pose and IGNORES live per-ball reactive refinements — a bad
# ball prediction must never move the platform mid-reload (2026-07-24: a corrupt
# track's sweep got ONE 78 mm target accepted at land−0.67 s, dragging the platform
# 83.7 mm in the last 0.8 s and costing the catch). Only the PLATFORM reach is
# frozen; the hand-arm stays reactive.


class _RecLogger:
    """A logger that records (level, message) so a test can assert log level/count."""
    def __init__(self):
        self.records = []

    def info(self, msg, **kw): self.records.append(('info', msg))
    def warning(self, msg, **kw): self.records.append(('warning', msg))
    def warn(self, msg, **kw): self.records.append(('warning', msg))
    def error(self, msg, **kw): self.records.append(('error', msg))
    def debug(self, msg, **kw): self.records.append(('debug', msg))
    def fatal(self, msg, **kw): self.records.append(('fatal', msg))


def test_open_loop_holds_pretilt_ignores_reactive_platform(monkeypatch):
    """Armed + OUR announcement seen + open-loop: a reactive per-ball cmd must NOT move
    the platform. The hand-arm still fires (reactive timing preserved), _last_cmd_mono is
    stamped (guards the kind-1 stroke on the Teensy queue), the per-ball correlation
    stays dormant (nothing feeds the blacklist), and the only platform target published
    is the PRE-TILT pose (stow-relative z ~170), NOT the reactive cmd's pose (z 809)."""
    assert hw.JB_OP_RELOAD_PLATFORM_OPEN_LOOP is True
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    node._on_throw_announcement(_announcement())          # sets announcement_seen + pre-tilt
    assert node._announcement_seen is True
    n0 = len(node._dyn_target_pub.published)
    armed = []
    monkeypatch.setattr(node, '_arm_hand_catch',
                        lambda d, v: armed.append((d, v)) or True)
    cmd = _catchable_cmd()
    cmd.landing_time = 100.0                               # within the arm window (announced t=100)
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None: cmd)
    node._on_balls(_balls_msg())
    assert len(armed) == 1                                 # hand-arm reactive timing preserved
    assert time.perf_counter() - node._last_cmd_mono < 1.0  # quiet window stamped
    assert node._last_submitted_ball_id is None            # reactive correlation dormant
    # The only platform target on this tick is the pre-tilt refresh, not the reactive pose.
    assert len(node._dyn_target_pub.published) == n0 + 1
    assert 150.0 < node._dyn_target_pub.published[-1].target_pos.z < 200.0


def test_reactive_platform_published_without_announcement(monkeypatch):
    """Armed but NO announcement yet (or a manual/bench throw): the reactive platform
    path stays live — open-loop only engages after OUR throw is announced."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    assert node._announcement_seen is False
    n0 = len(node._dyn_target_pub.published)
    monkeypatch.setattr(node, '_arm_hand_catch', lambda d, v: True)
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None: _catchable_cmd())
    node._on_balls(_balls_msg())
    assert len(node._dyn_target_pub.published) == n0 + 1
    assert node._dyn_target_pub.published[-1].target_pos.z == pytest.approx(809.08)
    assert node._last_submitted_ball_id == 5               # reactive correlation stamped


def test_disarm_resets_open_loop_state():
    """Disarm clears the open-loop latch so a stale announcement never freezes the
    platform before the NEXT reload's throw is announced."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    node._on_throw_announcement(_announcement())
    assert node._announcement_seen is True
    assert node._announced_landing_time is not None
    assert node._pretilt_cmd is not None
    node._on_catch_armed(Bool(data=False))
    assert node._announcement_seen is False
    assert node._announced_landing_time is None
    assert node._pretilt_cmd is None


# ── stale-track hand-arm guard (fix 2) ────────────────────────────────────────

def test_arm_window_rejects_ball_far_off_announced_landing(monkeypatch):
    """Once OUR throw is announced, a corrupt track whose predicted landing is far off
    the announced landing must NOT arm the one-shot hand stroke (arming off garbage
    timing would block the real ball's arm). A ball within the window still arms."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    node._on_throw_announcement(_announcement())          # announced landing ros t=100
    armed = []
    monkeypatch.setattr(node, '_arm_hand_catch',
                        lambda d, v: armed.append((d, v)) or True)
    far = _catchable_cmd()
    far.landing_time = 100.0 + 5.0                         # 5 s off the announced landing
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None: far)
    node._on_balls(_balls_msg())
    assert armed == []                                     # far-off landing → not armed
    near = _catchable_cmd()
    near.landing_time = 100.0 + 0.3                        # within 0.75 s
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None: near)
    node._on_balls(_balls_msg())
    assert len(armed) == 1


def test_arm_window_inert_before_announcement(monkeypatch):
    """With no announcement seen (manual/bench throw), the arm-window guard is inert —
    a catchable ball arms as before."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    assert node._announced_landing_time is None
    armed = []
    monkeypatch.setattr(node, '_arm_hand_catch',
                        lambda d, v: armed.append((d, v)) or True)
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None: _catchable_cmd())
    node._on_balls(_balls_msg())
    assert len(armed) == 1


def test_arm_edge_snapshots_preexisting_flight_ids():
    """The catch-armed rising edge snapshots the ids currently in flight (excluded from
    this reload's catch candidates); disarm clears the snapshot."""
    node = CatchCoordinatorNode()
    node._latest_in_flight_ids = {14, 15}                  # leftovers from a prior attempt
    node._on_catch_armed(Bool(data=True))
    assert node._preexisting_flight_ids == {14, 15}
    node._on_catch_armed(Bool(data=False))
    assert node._preexisting_flight_ids == set()


def test_preexisting_ids_passed_to_update(monkeypatch):
    """_on_balls passes the arm-edge snapshot to update(exclude_ids=...) so a
    prior-attempt leftover track can never be selected as the catch candidate."""
    node = CatchCoordinatorNode()
    node._latest_in_flight_ids = {14}
    node._on_catch_armed(Bool(data=True))                  # snapshot {14}
    captured = {}

    def _cap(balls, current_time, exclude_ids=None):
        captured['exclude'] = set(exclude_ids) if exclude_ids else set()
        return None

    monkeypatch.setattr(node._coordinator, 'update', _cap)
    node._on_balls(_balls_msg())
    assert captured['exclude'] == {14}


# ── hand-arm re-dispatch cap + WARN hygiene (fix 5) ───────────────────────────

class _FailAck:
    """A completed hand-traj future whose ack failed (the ERR_TIMEOUT epidemic)."""
    def result(self):
        return types.SimpleNamespace(success=False, message='ERR_TIMEOUT')


def test_arm_redispatch_capped(monkeypatch):
    """A failed (lying) ack re-opens the one-shot latch for a retry — but only up to
    _MAX_ARM_DISPATCHES per ball; after the cap the latch STAYS set (assume the lying
    ack armed) rather than churning the Teensy's last-writer-wins queue with
    near-identical repacks forever."""
    from jugglebot.catch_coordinator_node import _MAX_ARM_DISPATCHES
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    dispatched = []
    monkeypatch.setattr(node, '_arm_hand_catch',
                        lambda d, v: dispatched.append(1) or True)
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None: _catchable_cmd())
    fail = _FailAck()
    node._on_balls(_balls_msg())                           # dispatch 1
    assert node._hand_traj_armed_for_ball == 5 and node._arm_dispatch_count == 1
    node._on_hand_traj_done(fail)
    assert node._hand_traj_armed_for_ball is None          # re-opened (1 < cap)
    node._on_balls(_balls_msg())                           # dispatch 2
    assert node._hand_traj_armed_for_ball == 5 and node._arm_dispatch_count == 2
    node._on_hand_traj_done(fail)
    assert node._hand_traj_armed_for_ball == 5             # KEPT — capped, assume armed
    node._on_balls(_balls_msg())                           # no dispatch 3
    assert len(dispatched) == _MAX_ARM_DISPATCHES


def test_arm_failed_ack_logged_at_debug():
    """The expected-epidemic failed ack is DEBUG, not WARN — a working reload was reading
    as 30/51 arm-failure spam."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    node._arm_dispatch_count = 1                            # within the cap
    rec = _RecLogger()
    node._logger = rec
    node._on_hand_traj_done(_FailAck())
    assert any(lvl == 'debug' for lvl, _ in rec.records)
    assert not any(lvl == 'warning' for lvl, _ in rec.records)


# ══════════════════════════════════════════════════════════════════════════════
#  C-HAND-1 — the hand-catch arm is gated until OUR throw stroke completes
# ══════════════════════════════════════════════════════════════════════════════
#
# The defect (2026-07-25, seven self-tosses across three sessions): the kind-1
# catch arm landed 8-18 ms after release, INSIDE the throw's 65 ms deceleration
# ramp. Teensy_code.ino:539 clears the whole packed queue on any kind-0/1/2
# command and Trajectory.h:242-301 seeds the replacement prelude from
# current_hand_position with v = 0, a = 0 (current_hand_velocity is declared
# extern at :47 and never read) — so the queue was replaced by a rest-to-rest
# quintic computed from a position the hand was travelling through at ~120 rev/s.
# The hand overshot to 10.17-10.33 rev against an 11.1 rev guard, was yanked
# 0.34-1.75 rev (10.7-55.3 mm) BELOW the stroke end, and recovered over ~300 ms.
# It also discarded the THROW's own decel ramp: 0.887 s and 1.091 s of achieved
# flight against a commanded 0.800 s.
#
# The fix is timing only — no commanded magnitude changes. Arming after the
# stroke costs nothing because x3 (the throw's end) IS the catch trajectory's
# first sample, algebraically and for every commanded velocity.

_V_THROW_080 = 3.930820          # m/s — nominal 0.80 s flight (compute_release_state)
_V_LAND_080 = 3.913980           # m/s — |arrival velocity| of the same toss
_T_ANNOUNCE = 100.0              # ROS s
_T_RELEASE = 101.0               # ROS s (announced throw_time)
_T_LANDING = _T_RELEASE + 0.80   # ROS s
# throw_decel_s(3.930820) = 65.104 ms; + the 40 ms margin
_T_CLEAR = _T_RELEASE + 0.105104


class _FakeClock:
    """A settable ROS clock (the conftest MockClock is frozen at 0)."""
    def __init__(self, t=0.0):
        self.t = float(t)

    def now(self):
        return types.SimpleNamespace(nanoseconds=int(round(self.t * 1e9)))


def _self_toss_announcement(v_throw_mps=_V_THROW_080, throw_time=_T_RELEASE,
                            landing_time=_T_LANDING, thrower='jugglebot',
                            target_id='jugglebot'):
    """The real self-toss wire shape: thrower_name AND target_id are this robot,
    throw_time is the absolute ROS release instant, initial_velocity is the ball's
    launch vector in mm/s whose magnitude IS the commanded event_vel."""
    return types.SimpleNamespace(
        thrower_name=thrower,
        target_id=target_id,
        initial_position=types.SimpleNamespace(x=0.0, y=0.0, z=800.0),
        initial_velocity=types.SimpleNamespace(x=0.0, y=0.0,
                                               z=v_throw_mps * 1000.0),
        throw_time=types.SimpleNamespace(
            sec=int(throw_time), nanosec=int(round((throw_time % 1) * 1e9))),
        landing_position=types.SimpleNamespace(x=0.0, y=0.0, z=809.08),
        landing_velocity=types.SimpleNamespace(x=0.0, y=0.0,
                                               z=-_V_LAND_080 * 1000.0),
        landing_time=types.SimpleNamespace(
            sec=int(landing_time), nanosec=int(round((landing_time % 1) * 1e9))),
    )


def _toss_node(t=_T_ANNOUNCE, announce=True, **ann_kw):
    """A CCN in the Tier-8b toss configuration at ROS time ``t``: prime_hold and
    pretilt_hold raised, armed, and (optionally) our own announcement delivered."""
    node = CatchCoordinatorNode()
    node._clock = _FakeClock(t)
    node._on_prime_hold(Bool(data=True))
    node._on_pretilt_hold(Bool(data=True))
    node._on_catch_armed(Bool(data=True))
    if announce:
        node._on_throw_announcement(_self_toss_announcement(**ann_kw))
    return node


def _toss_cmd(ball_id=5, event_vel_mps=_V_LAND_080, landing_time=_T_LANDING):
    cmd = _catchable_cmd(ball_id=ball_id)
    cmd.landing_time = landing_time
    cmd.event_vel_mps = event_vel_mps
    return cmd


def _drive_balls(node, cmd, monkeypatch):
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None: cmd)
    node._on_balls(_balls_msg())


def _capture_arm_dispatch(node, monkeypatch):
    """Record every SetHandTrajCmd request the REAL _arm_hand_catch emits."""
    sent = []
    monkeypatch.setattr(node._hand_traj_client, 'call_async',
                        lambda req: sent.append(req) or _NeverFuture())
    return sent


class _NeverFuture:
    def add_done_callback(self, cb):
        pass


# ── latching the window off the announcement ────────────────────────────────

def test_self_toss_announcement_latches_the_stroke_window():
    """Both inputs come off the wire the toss already publishes — no new field,
    no new topic. throw_time is the kind-0 event instant, which IS ball release
    (makeThrow's shiftTime(-t2) puts t = 0 at the end of the velocity hold), and
    |initial_velocity| is exactly the event_vel the sequencer commands."""
    node = _toss_node()
    assert node._throw_stroke_v_throw == pytest.approx(_V_THROW_080, rel=1e-6)
    assert node._throw_stroke_clear_ros == pytest.approx(_T_CLEAR, abs=1e-5)
    # Derived, not fixed: the same arithmetic at the other end of the shipped
    # flight band gives a materially different window (2x the decel ramp), which
    # is why a fixed conservative delay was rejected.
    slow = _toss_node(v_throw_mps=2.708897)
    assert (slow._throw_stroke_clear_ros - _T_RELEASE) == pytest.approx(
        0.094471 + 0.040, abs=1e-5)


def test_reload_announcement_leaves_the_stroke_window_inert():
    """A BB reload has NO Jugglebot throw stroke: the hand is parked at the top at
    rest, a repack there is genuinely harmless, and delaying the arm would eat
    lead the catch needs. target_id alone cannot discriminate — a BB throw aimed
    at us carries target_id == robot_name too — so the gate keys on thrower_name.
    """
    node = CatchCoordinatorNode()
    node._clock = _FakeClock(_T_ANNOUNCE)
    node._on_pretilt_hold(Bool(data=True))
    node._on_catch_armed(Bool(data=True))
    node._on_throw_announcement(_self_toss_announcement(thrower='ball_butler'))
    assert node._announcement_seen is True          # hand-arm window still latched
    assert node._throw_stroke_clear_ros is None     # but the stroke gate is inert


def test_stroke_window_latches_on_the_non_pretilt_hold_branch_too():
    """Tier 8a / pretilt_hold off still runs a real throw stroke, so the latch
    sits ahead of the branch rather than inside one of them."""
    node = CatchCoordinatorNode()
    node._clock = _FakeClock(_T_ANNOUNCE)
    node._on_catch_armed(Bool(data=True))
    node._on_throw_announcement(_self_toss_announcement())
    assert node._throw_stroke_clear_ros == pytest.approx(_T_CLEAR, abs=1e-5)


def test_malformed_self_announcement_leaves_the_window_inert():
    """A window synthesized from garbage would suppress the catch arm for an
    arbitrary time. Refuse to latch and say so; the behaviour degrades to
    exactly today's."""
    node = CatchCoordinatorNode()
    node._clock = _FakeClock(_T_ANNOUNCE)
    node._on_pretilt_hold(Bool(data=True))
    node._on_catch_armed(Bool(data=True))
    rec = _RecLogger()
    node._logger = rec
    node._on_throw_announcement(_self_toss_announcement(v_throw_mps=0.0))
    assert node._throw_stroke_clear_ros is None
    assert any(lvl == 'warning' for lvl, _ in rec.records)


def test_stroke_window_cleared_on_both_latch_edges():
    """A stale window must never suppress the NEXT ball-op's catch arm. It would
    also self-expire, but the reload path must not depend on that.

    BOTH edges are driven independently. Driving only the disarm edge and then
    re-arming would leave the arm-edge clear unpinned: nothing is latched by that
    point, so the closing assertion passes on the disarm clear alone and a
    mutation that deletes the arm-edge block goes undetected (verified — that
    mutation left the file's suite fully green before this test was extended).
    The arm edge is therefore exercised from a hand-seeded stale window, which is
    the only state that can reach it.
    """
    node = _toss_node()
    assert node._throw_stroke_clear_ros is not None
    # ── disarm edge ──
    node._on_catch_armed(Bool(data=False))
    assert node._throw_stroke_clear_ros is None
    node._on_throw_announcement(_self_toss_announcement())   # disarmed → ignored
    assert node._throw_stroke_clear_ros is None

    # ── arm edge, driven on its own ──
    # Seed the residue an arm edge must scrub. Unreachable today (a latch
    # requires _catch_armed, and _on_catch_armed early-returns unless the flag
    # actually transitions), which is exactly why it needs pinning: the clear is
    # defence-in-depth, and defence-in-depth that no test drives is deleted by
    # the next refactor that notices it is redundant.
    node._throw_stroke_clear_ros = _T_CLEAR + 999.0
    node._throw_stroke_v_throw = _V_THROW_080
    node._stroke_gate_logged_for_ball = 7
    node._stroke_gate_forced_for_ball = 7
    node._on_catch_armed(Bool(data=True))            # disarmed → armed
    assert node._throw_stroke_clear_ros is None
    assert node._throw_stroke_v_throw is None
    assert node._stroke_gate_logged_for_ball is None
    assert node._stroke_gate_forced_for_ball is None


# ── the gate itself ─────────────────────────────────────────────────────────

def test_arm_withheld_while_our_throw_stroke_is_still_decelerating(monkeypatch):
    """release + 10 ms — where every observed arm landed — is now WITHHELD.

    Nothing reaches the service, so nothing clears the Teensy's packed queue and
    the throw plays its own deceleration ramp to x3."""
    node = _toss_node()
    node._clock.t = _T_RELEASE + 0.010
    sent = _capture_arm_dispatch(node, monkeypatch)
    _drive_balls(node, _toss_cmd(), monkeypatch)
    assert sent == []
    assert node._hand_traj_armed_for_ball is None      # latch left open → retried
    assert node._arm_dispatch_count == 0               # no dispatch burned


def test_arm_withheld_before_release_too(monkeypatch):
    """The window opens at the ANNOUNCEMENT, not at release: a kind-1 landing
    between the kind-0 dispatch and release would silently un-arm the throw on
    the last-writer-wins queue (ABORTED_NO_RELEASE), and one landing before the
    kind-0 would itself be clobbered by it."""
    node = _toss_node()
    node._clock.t = _T_RELEASE - 0.050
    sent = _capture_arm_dispatch(node, monkeypatch)
    _drive_balls(node, _toss_cmd(), monkeypatch)
    assert sent == []


def test_arm_dispatched_on_the_first_tick_after_the_stroke_ends(monkeypatch):
    """Withholding is a DEFERRAL, not a drop. One tick before the clear instant:
    nothing. One tick after: the arm goes out, with the hand standing at rest on
    x3 = the catch trajectory's own first sample."""
    node = _toss_node()
    sent = _capture_arm_dispatch(node, monkeypatch)
    node._clock.t = _T_CLEAR - 0.005
    _drive_balls(node, _toss_cmd(), monkeypatch)
    assert sent == []
    node._clock.t = _T_CLEAR + 0.005
    _drive_balls(node, _toss_cmd(), monkeypatch)
    assert len(sent) == 1
    assert sent[0].traj_type == 1                       # kind-1 catch
    assert sent[0].event_vel == pytest.approx(
        _V_LAND_080 * hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)
    assert node._hand_traj_armed_for_ball == 5
    assert node._arm_dispatch_count == 1


def test_gate_inert_when_no_self_throw_is_live(monkeypatch):
    """No latched window (a reload, a bench throw) ⇒ the arm dispatches exactly as
    before. This is the reload path's regression net."""
    node = CatchCoordinatorNode()
    node._clock = _FakeClock(_T_ANNOUNCE)
    node._on_catch_armed(Bool(data=True))
    sent = _capture_arm_dispatch(node, monkeypatch)
    _drive_balls(node, _toss_cmd(landing_time=_T_ANNOUNCE + 0.8), monkeypatch)
    assert len(sent) == 1


def test_suppression_logs_once_per_ball(monkeypatch):
    """The balls tick runs at mocap rate; an un-keyed log would emit ~20 lines
    per suppression and bury the rest of the sequence."""
    node = _toss_node()
    node._clock.t = _T_RELEASE + 0.010
    _capture_arm_dispatch(node, monkeypatch)
    rec = _RecLogger()
    node._logger = rec
    for _ in range(8):
        _drive_balls(node, _toss_cmd(), monkeypatch)
    withheld = [m for lvl, m in rec.records
                if lvl == 'info' and 'withheld' in str(m)]
    assert len(withheld) == 1


# ── step 3: the window must still FIT ───────────────────────────────────────

def test_min_event_delay_floor_pinned():
    """tests/motion/test_hand_stroke.py restates this value (importing the node
    there would drag rclpy into a pure-motion test); pin them together."""
    from jugglebot.catch_coordinator_node import _MIN_EVENT_DELAY_S
    assert _MIN_EVENT_DELAY_S == 0.3


def test_window_closed_dispatches_immediately_and_loudly(monkeypatch):
    """When waiting would push the arm past the Teensy's build deadline, ARM
    ANYWAY and shout.

    Teensy_code.ino:533 refuses the WHOLE command when
    now + smoothDur + SAFETY_GAP > firstMainAbs and prints the refusal to serial
    only (:534) — so an arm deferred past that point does not arrive late, the
    catch silently never fires and the ball hits the floor with no ROS-visible
    signal. The dip is ugly and recoverable; a silently-refused catch is neither.

    Driven the way it can actually happen: a LOW tracker landing-speed estimate.
    t_acc_catch = 0.404 / v_armed, so a slow estimate lengthens the required lead
    and moves the deadline earlier.
    """
    node = _toss_node()
    node._clock.t = _T_RELEASE + 0.010
    sent = _capture_arm_dispatch(node, monkeypatch)
    rec = _RecLogger()
    node._logger = rec
    _drive_balls(node, _toss_cmd(event_vel_mps=0.5), monkeypatch)
    assert len(sent) == 1                       # dispatched despite the live stroke
    warns = [m for lvl, m in rec.records if lvl == 'warning']
    assert any('CLOSED' in str(m) for m in warns)
    # ...and only once per ball.
    _drive_balls(node, _toss_cmd(event_vel_mps=0.5), monkeypatch)
    assert len([m for m in rec.records
                if m[0] == 'warning' and 'CLOSED' in str(m[1])]) == 1


def test_window_still_fits_at_the_shortest_shipped_flight(monkeypatch):
    """FLIGHT_TIME_MIN_S = 0.55 is the binding case (slower throw ⇒ longer decel
    ramp, shorter flight ⇒ less room). The arm is withheld at release + 10 ms and
    dispatches by release + 135 ms, ~115 ms inside the deadline."""
    from jugglebot.toss_sequencer import FLIGHT_TIME_MIN_S
    v_throw, v_land = 2.708897, 2.684366
    landing = _T_RELEASE + FLIGHT_TIME_MIN_S
    node = _toss_node(v_throw_mps=v_throw, landing_time=landing)
    clear = _T_RELEASE + 0.094471 + 0.040
    assert node._throw_stroke_clear_ros == pytest.approx(clear, abs=1e-5)
    sent = _capture_arm_dispatch(node, monkeypatch)
    node._clock.t = _T_RELEASE + 0.010
    _drive_balls(node, _toss_cmd(event_vel_mps=v_land, landing_time=landing),
                 monkeypatch)
    assert sent == []                                   # withheld, window OPEN
    rec = _RecLogger()
    node._logger = rec
    node._clock.t = clear + 0.001
    _drive_balls(node, _toss_cmd(event_vel_mps=v_land, landing_time=landing),
                 monkeypatch)
    assert len(sent) == 1
    assert not [m for lvl, m in rec.records
                if lvl == 'warning' and 'CLOSED' in str(m)]
    # And the arm still lands well inside the caller's own 0.3 s floor.
    assert landing - node._clock.t > 0.3


# ── Phase 2: the repack guard ───────────────────────────────────────────────

def test_failed_ack_retry_is_deferred_until_the_stroke_ends(monkeypatch):
    """_on_hand_traj_done re-opens the one-shot latch on a failed ack (the ack is
    unreliable AND lies), so the retry is a genuine re-dispatch. Before this gate
    it landed wherever the retry tick fell — during a toss, inside the stroke.

    Now the retry is refused while the stroke is live and DEFERRED to the first
    tick after it, which is what makes _on_hand_traj_done's "further repacks just
    churn" premise true again: the repack happens with the hand at rest at x3.
    """
    node = _toss_node()
    sent = _capture_arm_dispatch(node, monkeypatch)
    # First arm goes out cleanly after the stroke...
    node._clock.t = _T_CLEAR + 0.005
    _drive_balls(node, _toss_cmd(), monkeypatch)
    assert len(sent) == 1 and node._arm_dispatch_count == 1
    # ...its ack fails, re-opening the latch.
    node._on_hand_traj_done(_FailAck())
    assert node._hand_traj_armed_for_ball is None
    # A retry BEFORE the stroke would have cleared: rewind into the ramp and the
    # re-dispatch is withheld, not lost.
    node._clock.t = _T_RELEASE + 0.020
    _drive_balls(node, _toss_cmd(), monkeypatch)
    assert len(sent) == 1
    assert node._arm_dispatch_count == 1                # deferral burns nothing
    # First tick clear of the stroke: the deferred retry goes out.
    node._clock.t = _T_CLEAR + 0.010
    _drive_balls(node, _toss_cmd(), monkeypatch)
    assert len(sent) == 2
    assert node._arm_dispatch_count == 2


def test_dispatch_cap_and_keep_the_latch_behaviour_preserved(monkeypatch):
    """_MAX_ARM_DISPATCHES and the keep-the-latch-after-the-cap posture are
    correct and hard-won — a withheld arm must not erode either. Withholding
    never increments the counter, so the two allowed dispatches remain two REAL
    dispatches however many ticks were suppressed."""
    from jugglebot.catch_coordinator_node import _MAX_ARM_DISPATCHES
    node = _toss_node()
    sent = _capture_arm_dispatch(node, monkeypatch)
    node._clock.t = _T_RELEASE + 0.010
    for _ in range(10):                                  # ten suppressed ticks
        _drive_balls(node, _toss_cmd(), monkeypatch)
    assert sent == [] and node._arm_dispatch_count == 0
    node._clock.t = _T_CLEAR + 0.010
    for _ in range(4):
        _drive_balls(node, _toss_cmd(), monkeypatch)
        node._on_hand_traj_done(_FailAck())
    assert len(sent) == _MAX_ARM_DISPATCHES
    assert node._hand_traj_armed_for_ball == 5           # latch KEPT after the cap


# ── the abort path stays exempt ─────────────────────────────────────────────

def test_kind3_smooth_move_is_not_gated_by_the_stroke_window(monkeypatch):
    """A kind-3 replacing whatever is queued is the ONLY un-arm mechanism the
    Teensy offers, and a pre-release SAFE_ABORT's retract depends on it clobbering
    an armed kind-0 throw stroke (toss_sequencer's ORDERING PRINCIPLE). Anything
    that made a kind-3 refuse to clobber would be a safety regression, so the gate
    is confined to _arm_hand_catch — the node's only kind-0/1/2 dispatch.

    (The retract itself is dispatched by reload_coordinator_node through its own
    smooth_move_hand client and never passes through this node at all. The toss's
    own prime-during-stroke hazard is owned by catch/prime_hold, a separate gate
    raised for the entire PREPARE→terminal span.)
    """
    node = _toss_node()
    node._clock.t = _T_RELEASE + 0.010                   # mid decel ramp
    assert node._throw_stroke_clear_ros > node._clock.t  # gate is live
    moved = []
    monkeypatch.setattr(node._smooth_move_client, 'call_async',
                        lambda req: moved.append(req) or _NeverFuture())
    node._prime_hold = False                             # isolate: only this gate
    node._prime_hand()
    assert len(moved) == 1                               # kind-3 NOT refused


def test_arm_dispatch_is_always_kind1(monkeypatch):
    """The gate only ever sees kind-1 traffic — pinned so a future kind-0/2
    dispatch added here is a deliberate decision, not an accident."""
    node = _toss_node()
    node._clock.t = _T_CLEAR + 0.010
    sent = _capture_arm_dispatch(node, monkeypatch)
    _drive_balls(node, _toss_cmd(), monkeypatch)
    assert [r.traj_type for r in sent] == [1]


def test_absurd_throw_time_cannot_hang_the_catch(monkeypatch):
    """A window is an absolute instant computed from wire data, so a wildly wrong
    ``throw_time`` would otherwise suppress the arm for as long as it is wrong.

    The fit check is what bounds that: once waiting would push the arm past the
    Teensy's build deadline the gate dispatches anyway and warns, so the worst a
    garbage announcement can do is restore today's behaviour with a loud log —
    never a silently withheld catch.
    """
    node = _toss_node(throw_time=_T_RELEASE + 3600.0)
    node._clock.t = _T_RELEASE + 0.010
    sent = _capture_arm_dispatch(node, monkeypatch)
    rec = _RecLogger()
    node._logger = rec
    _drive_balls(node, _toss_cmd(), monkeypatch)
    assert len(sent) == 1
    assert any('CLOSED' in str(m) for lvl, m in rec.records if lvl == 'warning')
