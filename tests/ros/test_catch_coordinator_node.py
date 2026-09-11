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


def test_disarm_resets_hand_one_shots():
    """Disarming (reload ended / aborted) resets the prime one-shot so the NEXT
    reload re-primes from a clean state."""
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    node._hand_primed = True
    node._on_catch_armed(Bool(data=False))
    assert node._catch_armed is False
    assert node._hand_primed is False


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


def test_pretilt_hold_true_suppresses_platform_but_latches(monkeypatch):
    """The Tier-8b toss: pretilt_hold True at PREPARE, BEFORE the announcement.
    The announcement publishes NO platform target and caches _pretilt_cmd = None
    (so a balls tick's _republish_pretilt no-ops), BUT still latches
    _announcement_seen + _announced_landing_time (the open-loop freeze keeps
    working)."""
    node = CatchCoordinatorNode()
    node._on_pretilt_hold(Bool(data=True))       # PREPARE: raised before armed
    node._on_catch_armed(Bool(data=True))
    n0 = len(node._dyn_target_pub.published)
    node._on_throw_announcement(_announcement())               # landing ros t=100
    # NO platform target from the announcement; latched for open-loop.
    assert len(node._dyn_target_pub.published) == n0
    assert node._announcement_seen is True
    assert node._announced_landing_time == pytest.approx(100.0)
    assert node._pretilt_cmd is None
    # A balls tick under open-loop: _republish_pretilt no-ops (_pretilt_cmd None)
    # — the toss coordinator owns the platform reach.
    cmd = _catchable_cmd()
    cmd.landing_time = 100.0
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None: cmd)
    node._on_balls(_balls_msg())
    assert len(node._dyn_target_pub.published) == n0           # still no platform target
    assert node._last_submitted_ball_id is None                # reactive correlation dormant


def test_pretilt_hold_does_not_gate_edge_prime(monkeypatch):
    """pretilt_hold gates ONLY the platform pre-tilt publish — NOT the armed-edge
    hand prime (that is prime_hold's job). With ONLY pretilt_hold up, the armed
    edge still primes; in production the toss raises BOTH gates."""
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
    compete with the toss coordinator's deferred A->B reach mid-flight."""
    monkeypatch.setattr(hw, 'JB_OP_RELOAD_PLATFORM_OPEN_LOOP', False)
    node = CatchCoordinatorNode()
    node._on_pretilt_hold(Bool(data=True))       # PREPARE: raised before armed
    node._on_catch_armed(Bool(data=True))
    node._on_throw_announcement(_announcement())               # latched, no platform target
    assert node._announcement_seen is True
    assert node._pretilt_cmd is None
    n0 = len(node._dyn_target_pub.published)
    cmd = _catchable_cmd()
    cmd.landing_time = 100.0
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None: cmd)
    node._on_balls(_balls_msg())
    assert len(node._dyn_target_pub.published) == n0           # NO reactive platform target
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
# 83.7 mm in the last 0.8 s and costing the catch). R1: the hand-arm reactive path
# is retired (owner decision 3) — this covers the PLATFORM reach only.


def test_open_loop_holds_pretilt_ignores_reactive_platform(monkeypatch):
    """Armed + OUR announcement seen + open-loop: a reactive per-ball cmd must NOT move
    the platform. _last_cmd_mono is still stamped (guards the balls-path quiet window),
    the per-ball correlation stays dormant (nothing feeds the blacklist), and the only
    platform target published is the PRE-TILT pose (stow-relative z ~170), NOT the
    reactive cmd's pose (z 809)."""
    assert hw.JB_OP_RELOAD_PLATFORM_OPEN_LOOP is True
    node = CatchCoordinatorNode()
    node._on_catch_armed(Bool(data=True))
    node._on_throw_announcement(_announcement())          # sets announcement_seen + pre-tilt
    assert node._announcement_seen is True
    n0 = len(node._dyn_target_pub.published)
    cmd = _catchable_cmd()
    cmd.landing_time = 100.0                               # within the announced window
    monkeypatch.setattr(node._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None: cmd)
    node._on_balls(_balls_msg())
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

