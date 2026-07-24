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

import jugglebot.hardware_config as hw
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
