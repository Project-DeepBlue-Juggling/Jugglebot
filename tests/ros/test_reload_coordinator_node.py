"""reload_coordinator_node tests — the thin ROS wrapper.

The FSM itself is exhaustively covered by test_reload_sequencer.py; here we test the
node's seams: the wiring surface (action / clients / publishers / subscriptions),
observation assembly from cached messages (fresh vs stale), and that _step_sequence
executes the FSM's requested action (PRIME_HAND / call_reload / PREPARE / send_throw /
RECENTER / SAFE_ABORT) and publishes phase feedback.

The RELOAD action OWNS the platform + hand for its duration (reload-action-catch-latch
plan): it runs within the active streaming mode (``TRAJECTORY``), primes the hand at
COMMAND time (PRIME_HAND → smooth_move_hand), raises the catch-armed latch BEFORE the
throw is committed (PREPARE → trajectory/arm_catch; BB's firmware countdown has no
abort opcode), retracts on abort, and re-centers on terminal (trajectory/go_home). It
publishes the catch-armed state on ``catch/armed`` to gate catch_coordinator's
hand-arm.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

import time
import types

import pytest

import jugglebot.hardware_config as hw
from jugglebot.reload_coordinator_node import ReloadCoordinatorNode
from jugglebot.reload_sequencer import (
    ACTION_CALL_RELOAD,
    ACTION_PREPARE_CATCH,
    ACTION_PRIME_HAND,
    ACTION_RECENTER,
    ACTION_SAFE_ABORT,
    ACTION_SEND_THROW,
    BB_STATE_IDLE,
    BB_STATE_THROWING,
    RELOAD_CONTROL_MODE,
    ReloadDecision,
    ReloadResult,
    ReloadSequencer,
)
from tests.ros import possession_fixtures as fx


def _seq_in_flight(node):
    """Drive a sequencer through the armed-before-throw walk to BALL_IN_FLIGHT:
    PRIME (0.0) → PREPARE ok (0.1) → SEND_THROW (0.2) → accept → THROW_PENDING
    (0.3) → announcement → BALL_IN_FLIGHT. Release-fallback landing = 0.2 + 3.0."""
    seq = ReloadSequencer(catch_point_mm=node._catch_point_mm, throw_delay_s=3.0)
    seq.start(0.0)
    seq.step(0.0, _obs_stub())
    seq.note_prime_result(True)
    seq.step(0.1, _obs_stub())
    seq.note_prepare_result(True)
    seq.step(0.2, _obs_stub())            # SEND_THROW
    seq.note_throw_result(True)
    seq.step(0.3, _obs_stub())            # THROW_PENDING
    seq.note_announcement(0.5)
    seq.step(0.5, _obs_stub())            # BALL_IN_FLIGHT
    assert seq.phase == 'BALL_IN_FLIGHT'
    return seq


class _Hb:
    def __init__(self, connected=True, state=BB_STATE_IDLE, ball_in_hand=True):
        self.connected = connected
        self.state = state
        self.ball_in_hand = ball_in_hand


class _Pos:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Ball:
    def __init__(self, status, destination='jugglebot', x=0.0, y=0.0, z=809.08, id=5):
        self.id = id
        self.status = status
        self.destination = destination
        self.position = _Pos(x, y, z)


def _node_fresh(now, commanded_pos=(0.0, 0.0, 170.0)):
    """A node with every observation cached FRESH at `now` and preconditions met.

    ``commanded_pos`` feeds the trajectory/commanded_position cache the reload's
    REJECTED_NOT_CENTERED gate reads (Phase E, 2026-07-29); the default is the
    workspace centre, i.e. the state a healthy reload session starts from."""
    node = ReloadCoordinatorNode()
    with node._lock:
        node._hb = _Hb()
        node._hb_mono = now
        node._control_mode = RELOAD_CONTROL_MODE
        node._streaming = True
        node._commanded_pos_mm = tuple(float(v) for v in commanded_pos)
        node._commanded_pos_mono = now
        node._mocap_mono = now
        node._balls = []
        node._balls_mono = now
    return node


# ── Wiring surface ─────────────────────────────────────────────

def test_wiring_surface():
    node = ReloadCoordinatorNode()
    assert 'bb/heartbeat' in node._subscriptions
    assert 'control_mode_topic' in node._subscriptions
    assert 'trajectory/status' in node._subscriptions
    assert 'rigid_body_poses' in node._subscriptions
    assert 'balls' in node._subscriptions
    assert 'throw_announcements' in node._subscriptions
    assert 'trajectory/target_feedback' in node._subscriptions
    assert 'bb/reload' in node._clients
    assert 'bb/throw_at_target' in node._clients
    # Platform + hand ownership clients (Phase 2).
    assert 'trajectory/arm_catch' in node._clients
    assert 'smooth_move_hand' in node._clients
    assert 'trajectory/go_home' in node._clients
    # Catch-armed state publisher (gates catch_coordinator's hand-arm).
    assert 'catch/armed' in node._publishers
    # Prime-dispatch announcements (catch_coordinator's anti-stutter window).
    assert 'catch/prime_dispatched' in node._publishers
    assert 'jugglebot/reload' in node._action_servers


def test_catch_point_includes_hand_cup_offset():
    """The reload aim must be the CUP plane (platform centroid + HAND_CATCH_OFFSET_MM),
    matching throw_ballistics/_landing_z/catch_coordinator — NOT the bare centroid.
    Regression for the 2026-07-20 fix: aiming at the centroid (744.3) delivered the ball
    64.78 mm below where the hand intercepts it (809.08)."""
    node = ReloadCoordinatorNode()
    expected_z = (hw.GEOM_INITIAL_HEIGHT_MM + hw.JB_OP_DEFAULT_ACTIVE_Z_MM
                  + hw.HAND_CATCH_OFFSET_MM)
    assert node._catch_point_mm[0] == pytest.approx(0.0)
    assert node._catch_point_mm[1] == pytest.approx(0.0)
    assert node._catch_point_mm[2] == pytest.approx(expected_z)
    assert node._catch_point_mm[2] == pytest.approx(809.08)


# ── Observation assembly ───────────────────────────────────────

def test_build_observations_fresh():
    now = 100.0
    node = _node_fresh(now)
    obs = node._build_observations(now)
    assert obs.control_mode == RELOAD_CONTROL_MODE
    assert obs.bb_connected is True
    assert obs.bb_state == BB_STATE_IDLE
    assert obs.ball_in_hand is True
    assert obs.mocap_fresh is True
    assert obs.streaming is True
    assert obs.ball_caught is False


def test_build_observations_stale_mocap_and_heartbeat():
    now = 100.0
    node = _node_fresh(now)
    # Age everything past the freshness windows.
    obs = node._build_observations(now + 1.0)
    assert obs.mocap_fresh is False
    assert obs.bb_connected is False        # stale heartbeat → not connected
    assert obs.bb_state == 0                # stale → conservative BOOT


def test_build_observations_detects_caught_ball_with_error():
    now = 100.0
    node = _node_fresh(now)
    # The tracker first puts OUR ball in flight → the coordinator latches its id.
    with node._lock:
        node._balls = [_Ball(status=1, id=5)]             # IN_FLIGHT
        node._balls_mono = now
    node._build_observations(now)                          # latches announced id = 5
    # Then it's caught, 25 mm off centre.
    with node._lock:
        node._balls = [_Ball(status=2, id=5, x=15.0, y=20.0)]   # CAUGHT
        node._balls_mono = now
    obs = node._build_observations(now)
    assert obs.ball_caught is True
    assert obs.catch_error_mm == pytest.approx(25.0, abs=1e-6)


def test_build_observations_ignores_other_robots_ball():
    now = 100.0
    node = _node_fresh(now)
    with node._lock:
        node._balls = [_Ball(status=2, id=5, destination='someone_else')]
        node._balls_mono = now
    obs = node._build_observations(now)
    assert obs.ball_caught is False


def test_build_observations_stray_caught_ball_different_id_ignored():
    """A caught ball whose id is NOT our announced ball's (e.g. a leftover from a prior
    throw) does not confirm the reload's catch (fix 9)."""
    now = 100.0
    node = _node_fresh(now)
    # Our ball (id 5) goes in flight → latched.
    with node._lock:
        node._balls = [_Ball(status=1, id=5)]
        node._balls_mono = now
    node._build_observations(now)
    # A STRAY caught ball (id 99) appears while ours is still airborne.
    with node._lock:
        node._balls = [_Ball(status=2, id=99, x=0.0, y=0.0), _Ball(status=1, id=5)]
        node._balls_mono = now
    obs = node._build_observations(now)
    assert obs.ball_caught is False


# ── _step_sequence executes actions + publishes feedback ───────

class _GoalHandle:
    def __init__(self):
        self.feedbacks = []
        self.is_cancel_requested = False

    def publish_feedback(self, fb):
        self.feedbacks.append(fb.phase)


def test_step_sequence_primes_then_calls_reload_and_publishes_feedback(monkeypatch):
    now = 100.0
    node = _node_fresh(now)
    with node._lock:              # empty hand → FSM asks for a reload (after the prime)
        node._hb = _Hb(ball_in_hand=False)
        node._hb_mono = now
    calls = []
    monkeypatch.setattr(node, '_smooth_move_hand',
                        lambda p: calls.append(('prime', p)) or True)
    monkeypatch.setattr(node, '_call_reload', lambda: calls.append('reload') or True)
    seq = ReloadSequencer(catch_point_mm=node._catch_point_mm, throw_delay_s=3.0)
    seq.start(now)
    gh = _GoalHandle()
    decision = node._step_sequence(seq, now, gh)
    assert decision.action == ACTION_PRIME_HAND      # hand prime FIRST, at command
    decision = node._step_sequence(seq, now + 0.1, gh)
    assert decision.action == ACTION_CALL_RELOAD
    assert calls == [('prime', hw.JB_OP_HAND_CATCH_PRIME_REV), 'reload']
    assert gh.feedbacks == ['CHECKING', 'CHECKING']


def test_step_sequence_arms_then_sends_throw_and_feeds_result(monkeypatch):
    now = 100.0
    node = _node_fresh(now)            # ball in hand → prime → prepare → throw
    sent = []
    monkeypatch.setattr(node, '_smooth_move_hand', lambda p: True)
    monkeypatch.setattr(node, '_prepare_catch', lambda: True)
    monkeypatch.setattr(node, '_send_throw',
                        lambda s: (sent.append(s.throw_delay_s) or (True, 'ok')))
    seq = ReloadSequencer(catch_point_mm=node._catch_point_mm, throw_delay_s=3.0)
    seq.start(now)
    gh = _GoalHandle()
    d = node._step_sequence(seq, now, gh)
    assert d.action == ACTION_PRIME_HAND
    d = node._step_sequence(seq, now + 0.1, gh)
    assert d.action == ACTION_PREPARE_CATCH          # latch BEFORE the throw
    d = node._step_sequence(seq, now + 0.2, gh)
    assert d.action == ACTION_SEND_THROW
    assert sent == [3.0]
    # The BB accept was fed back into the FSM → next step reaches THROW_PENDING
    # with no further action (all arming already ran).
    d2 = node._step_sequence(seq, now + 0.3, gh)
    assert d2.phase == 'THROW_PENDING'
    assert d2.action == 'none'


def test_step_sequence_dispatches_prepare_recenter_safe_abort(monkeypatch):
    """_step_sequence routes each new terminal/prepare action to the matching executor —
    INCLUDING terminal (done) decisions (RECENTER / SAFE_ABORT run before the caller
    returns on done)."""
    now = 100.0
    node = _node_fresh(now)
    called = []
    monkeypatch.setattr(node, '_smooth_move_hand',
                        lambda p: called.append('prime') or True)
    monkeypatch.setattr(node, '_prepare_catch',
                        lambda: called.append('prepare') or True)
    monkeypatch.setattr(node, '_recenter', lambda: called.append('recenter'))
    monkeypatch.setattr(node, '_safe_abort', lambda: called.append('safe_abort'))
    cases = [
        (ACTION_PRIME_HAND, False, 'prime'),
        (ACTION_PREPARE_CATCH, False, 'prepare'),
        (ACTION_RECENTER, True, 'recenter'),
        (ACTION_SAFE_ABORT, True, 'safe_abort'),
    ]
    for action, done, _tag in cases:
        seq = ReloadSequencer(catch_point_mm=node._catch_point_mm)
        seq.step = lambda now, obs, _a=action, _d=done: ReloadDecision(
            'X', _a, _d, ReloadResult(False, 'x'))
        node._step_sequence(seq, now)
    assert called == ['prime', 'prepare', 'recenter', 'safe_abort']


# ── Terminal-action executors ──────────────────────────────────

def test_prepare_catch_raises_latch_and_publishes_armed(monkeypatch):
    """PREPARE is now latch-raise + publish ONLY — the hand prime moved to
    PRIME_HAND at sequence start (the 2026-07-23 hardware session showed a prime
    racing the flight is a coin flip)."""
    node = ReloadCoordinatorNode()
    calls = []
    monkeypatch.setattr(node, '_smooth_move_hand',
                        lambda p: calls.append(('hand', p)) or True)
    monkeypatch.setattr(node, '_arm_catch', lambda a: calls.append(('arm', a)) or True)
    assert node._prepare_catch() is True
    assert ('arm', True) in calls                              # raise the latch
    assert not any(c[0] == 'hand' for c in calls)              # no prime here anymore
    # catch/armed published True so catch_coordinator can actuate the hand.
    assert node._publishers['catch/armed'].published[-1].data is True


def test_prepare_catch_failure_reports_false_and_never_publishes_armed(monkeypatch):
    """A failed latch raise returns False (→ the FSM aborts BEFORE the throw is
    committed) and must NOT publish catch/armed=True — catch_coordinator would
    otherwise arm the hand stroke for a sequence that is being aborted."""
    node = ReloadCoordinatorNode()
    monkeypatch.setattr(node, '_arm_catch', lambda a: False)
    n_pub = len(node._publishers['catch/armed'].published)
    assert node._prepare_catch() is False
    assert len(node._publishers['catch/armed'].published) == n_pub


def test_recenter_lowers_latch_and_go_home_no_retract(monkeypatch):
    node = ReloadCoordinatorNode()
    calls = []
    monkeypatch.setattr(node, '_smooth_move_hand',
                        lambda p: calls.append(('hand', p)) or True)
    monkeypatch.setattr(node, '_arm_catch', lambda a: calls.append(('arm', a)) or True)
    monkeypatch.setattr(node, '_go_home', lambda: calls.append(('home',)) or True)
    node._recenter()
    assert ('arm', False) in calls          # lower the latch
    assert ('home',) in calls               # re-center
    # A successful catch keeps the ball — the hand is NOT retracted.
    assert not any(c[0] == 'hand' for c in calls)
    assert node._publishers['catch/armed'].published[-1].data is False


def test_safe_abort_retracts_hand_lowers_latch_recenters(monkeypatch):
    node = ReloadCoordinatorNode()
    calls = []
    monkeypatch.setattr(node, '_smooth_move_hand',
                        lambda p: calls.append(('hand', p)) or True)
    monkeypatch.setattr(node, '_arm_catch', lambda a: calls.append(('arm', a)) or True)
    monkeypatch.setattr(node, '_go_home', lambda: calls.append(('home',)) or True)
    node._safe_abort()
    assert ('hand', hw.JB_OP_HAND_RETRACT_REV) in calls    # retract to BOTTOM
    assert ('arm', False) in calls                          # lower the latch
    assert ('home',) in calls                               # re-center
    assert node._publishers['catch/armed'].published[-1].data is False


def test_hand_targets_are_within_bridge_service_range():
    """CLASS GUARD (2026-07-23): every hand target this node dispatches must be
    inside the bridge's smooth_move_hand validation range [0, hand max position].
    The abort retract used to target HOMING_HAND_ABS_POS_REV = -0.1 rev — BELOW the
    range — so the bridge rejected it and every abort left the hand parked at top,
    silently (the return value was ignored too). Pin both dispatch targets."""
    assert 0.0 <= hw.JB_OP_HAND_RETRACT_REV <= hw.GEOM_HAND_MOTOR_MAX_POSITION_REVS
    assert 0.0 <= hw.JB_OP_HAND_CATCH_PRIME_REV <= hw.GEOM_HAND_MOTOR_MAX_POSITION_REVS
    # The homing reference is intentionally NOT a valid smooth-move target.
    assert hw.HOMING_HAND_ABS_POS_REV < 0.0
    # Headroom, not just membership: the prime moved 3.2 mm UP on 2026-07-26 when
    # it became the derived stroke top, and the overextension guard is the wall it
    # moved toward. 1.1406 rev = 36.1 mm of clearance (was 39.3 mm).
    assert (hw.GEOM_HAND_MOTOR_MAX_POSITION_REVS
            - hw.JB_OP_HAND_CATCH_PRIME_REV) == pytest.approx(1.1406, abs=1e-3)


def test_prime_move_leaves_the_park_band_windows_open():
    """The prime became the derived stroke top on 2026-07-26 (9.858 -> 9.9594 rev,
    3.2 mm UP). Every window that keys off it must still hold.

    PROBE-DRIVEN, not asserted: ``/tmp/probe_prime_rev_windows.py`` (2026-07-26)
    computed each margin at both values; the numbers below are its output. Ground
    truth for the parked-top position spread is the same 2026-07-24 bag reading
    the module header records at ``reload_coordinator_node.py:205`` —
    ``[9.675, 10.044]`` rev, MEASURED WITH THE PRIME COMMANDED TO 9.858.

    The hand parks where it is commanded, so post-change the spread translates up
    with the target and every margin returns to its measured value (0.3170 /
    0.3140 rev). What is pinned here is the pessimistic TRANSITIONAL case — a hand
    still standing at the OLD top while the NEW band is applied — because that is
    the only reading of the measured spread that is not an assumption about
    physics the change itself alters.

    Why W1/W2 are the ones that would hurt: too LOW a near-band edge and a
    genuinely-parked hand reads as "not at target", so the ladder re-dispatches
    into a live ascent (the 2026-07-23 stutter, and the 4/4 lying-ack ladder that
    produced a false ABORTED_PRIME_FAILED). Too HIGH a retract qualifier and
    parked-top |vel| noise — 5.39 rev/s p99, above the 2.0 rev/s moving threshold
    — fakes a descent, so a genuinely lost retract frame is never re-sent and the
    safing path silently no-ops.
    """
    from jugglebot.reload_coordinator_node import (_HAND_NEAR_TARGET_REV,
                                                   _HAND_MOVING_VEL_RPS)
    prime = float(hw.JB_OP_HAND_CATCH_PRIME_REV)
    spread_lo, spread_hi = 9.675, 10.044        # measured at the OLD prime

    # W1 — _hand_dispatch_confirmed's at-target near-band, prime ladder.
    assert prime - _HAND_NEAR_TARGET_REV < spread_lo
    assert spread_lo - (prime - _HAND_NEAR_TARGET_REV) == pytest.approx(
        0.2156, abs=1e-3)                                    # 6.8 mm
    assert spread_hi < prime + _HAND_NEAR_TARGET_REV
    assert (prime + _HAND_NEAR_TARGET_REV) - spread_hi == pytest.approx(
        0.4154, abs=1e-3)                                    # 13.1 mm

    # W2 — the retract-descending qualifier (pos < prime - near-band) must still
    # EXCLUDE a hand parked at top, or velocity noise fakes "descending".
    assert spread_lo - (prime - _HAND_NEAR_TARGET_REV) > 0.0
    assert _HAND_MOVING_VEL_RPS == 2.0        # the noise threshold it pairs with

    # W3 — toss_sequencer's hand_parked band is the BOTTOM one, keyed to
    # JB_OP_HAND_RETRACT_REV, so a kind-0 stroke's off-band hazard is untouched by
    # this change. Pinned so a future edit re-keying it to the prime is caught.
    assert hw.JB_OP_HAND_RETRACT_REV == 0.0
    assert abs(prime - hw.JB_OP_HAND_RETRACT_REV) > _HAND_NEAR_TARGET_REV


def test_safe_on_early_exit_safes_only_when_armed(monkeypatch):
    """A node-level early exit (cancel / timeout / shutdown) bypasses the FSM's own
    terminal SAFE_ABORT — the node must safe the robot itself, but ONLY once
    something was armed (the hand prime dispatched or the latch raise requested).
    A pre-arming exit safes nothing. Note the prime is the FIRST action of the
    sequence, so 'armed' begins one step in."""
    node = ReloadCoordinatorNode()
    called = []
    monkeypatch.setattr(node, '_safe_abort', lambda: called.append('safe'))
    seq = ReloadSequencer(catch_point_mm=node._catch_point_mm, throw_delay_s=3.0)
    seq.start(0.0)
    # Nothing armed yet (no step ran) → early exit safes nothing.
    node._safe_on_early_exit(seq)
    assert called == []
    # One step: the PRIME_HAND dispatch — the hand may be mid-move → must safe.
    d = seq.step(0.0, _obs_stub())
    assert d.action == ACTION_PRIME_HAND
    assert seq.prepared is True
    node._safe_on_early_exit(seq)
    assert called == ['safe']


# ── Async event forwarding to the active sequencer ─────────────

def test_announcement_forwards_to_active_sequencer():
    node = ReloadCoordinatorNode()
    seq = ReloadSequencer(catch_point_mm=node._catch_point_mm, throw_delay_s=3.0)
    seq.start(0.0)
    seq.step(0.0, _obs_stub())        # PRIME
    seq.note_prime_result(True)
    seq.step(0.1, _obs_stub())        # PREPARE
    seq.note_prepare_result(True)
    seq.step(0.2, _obs_stub())        # SEND_THROW
    seq.note_throw_result(True)
    seq.step(0.3, _obs_stub())        # → THROW_PENDING
    with node._lock:
        node._active_seq = seq
    ann = types.SimpleNamespace(target_id='jugglebot')
    node._on_announcement(ann)
    d = seq.step(0.4, _obs_stub())
    assert d.phase == 'BALL_IN_FLIGHT'


def test_announcement_ignored_when_no_active_sequence():
    node = ReloadCoordinatorNode()          # no active sequence
    ann = types.SimpleNamespace(target_id='jugglebot')
    node._on_announcement(ann)              # must not raise


def test_target_feedback_reject_forwards_infeasibility():
    node = ReloadCoordinatorNode()
    seq = _seq_in_flight(node)
    with node._lock:
        node._active_seq = seq
    fb = types.SimpleNamespace(source='catch', accepted=False, code='WORKSPACE')
    node._on_target_feedback(fb)
    # The standing infeasibility does NOT terminate mid-flight (the primed cup may
    # still catch); it resolves the outcome at settle.
    d = seq.step(1.0, _obs_stub())
    assert not d.done
    d = seq.step(4.5, _obs_stub())          # settle (fallback landing 3.2 + 0.7)
    assert d.done and d.result.outcome == 'MISSED_INFEASIBLE_WORKSPACE'


def test_target_feedback_frozen_and_stale_are_ignored():
    """trajectory_node emits FROZEN for every late catch target in the reach-freeze
    window and STALE_STATE on transient races — the node must drop them so the catch
    proceeds (fix 5)."""
    node = ReloadCoordinatorNode()
    seq = _seq_in_flight(node)
    with node._lock:
        node._active_seq = seq
    for code in ('FROZEN', 'STALE_STATE'):
        node._on_target_feedback(
            types.SimpleNamespace(source='catch', accepted=False, code=code))
    d = seq.step(1.0, _obs_stub())
    assert not d.done                       # neither latched an infeasibility
    d = seq.step(4.5, _obs_stub())          # settle: plain MISSED, not INFEASIBLE
    assert d.done and d.result.outcome == 'MISSED'


# ── Concurrent-goal rejection (fix 4) ──────────────────────────

def test_second_goal_rejected_while_one_active():
    from rclpy.action import GoalResponse
    node = ReloadCoordinatorNode()
    # No active sequence → accept.
    assert node._goal_callback(object()) == GoalResponse.ACCEPT
    # A reload in progress → reject the concurrent goal (would double-throw).
    with node._lock:
        node._active_seq = ReloadSequencer(catch_point_mm=node._catch_point_mm)
    assert node._goal_callback(object()) == GoalResponse.REJECT


def test_platform_centered_reads_the_live_commanded_pose():
    """The node-side half of REJECTED_NOT_CENTERED: measured against the RELOAD
    catch point (read from _catch_point_mm, not hard-coded) with
    _RELOAD_CENTERED_TOL_MM as the tolerance.

    xy only, and frame-exact: _catch_point_mm is world while the commanded pose
    is STOW-relative, but STOW→world adds only z."""
    import jugglebot.reload_coordinator_node as _rcn
    tol = _rcn._RELOAD_CENTERED_TOL_MM
    now = 100.0
    assert _node_fresh(now)._platform_centered(now) is True
    assert _node_fresh(now, (tol - 1.0, 0.0, 170.0))._platform_centered(now) is True
    assert _node_fresh(now, (tol + 1.0, 0.0, 170.0))._platform_centered(now) is False
    # A caught toss parked at the +-150 mm working range: refused.
    assert _node_fresh(now, (150.0, -150.0, 170.0))._platform_centered(now) is False
    # z is NOT part of the test (the reload's z is the ACTIVE plane by
    # construction; a z difference is not what makes the catch unreachable).
    assert _node_fresh(now, (0.0, 0.0, 500.0))._platform_centered(now) is True


def test_centered_tolerance_leaves_room_for_the_reload_pretilt_swing():
    """THE gate's reason for existing: an ADMITTED park must still be able to
    reach the ball BB is about to throw.

    The reload declares no reach centre (C-REACH-1), so its arm raise centres
    the 80 mm envelope on the PARK — but the target that envelope judges is the
    pre-tilt catch pose, whose centroid sits hand_catch_offset*sin(tilt) off the
    catch point, and compute_catch_orientation CLAMPS at MAX_TILT_DEG for every
    real BB arrival (18-40 deg off vertical). So the shift SATURATES at
    64.78*sin(12 deg) = 13.47 mm on every single reload.

    With the tolerance set to the bare envelope radius (what shipped in review)
    a park at 70 mm was ADMITTED and the resulting excursion was 83.5 mm ->
    rejected WORKSPACE with BB's countdown already started and the ball
    unsavable. This test fails against that version."""
    import math
    import jugglebot.hardware_config as hw
    import jugglebot.reload_coordinator_node as _rcn
    from jugglebot.motion.trajectory.tilt_geometry import MAX_TILT_DEG

    env = float(hw.JB_TRAJ_CATCH_REACH_ENVELOPE_MM)
    swing = float(hw.HAND_CATCH_OFFSET_MM) * math.sin(math.radians(MAX_TILT_DEG))
    assert swing == pytest.approx(13.469, abs=0.01)
    # The tolerance is the envelope MINUS the worst-case swing — derived, not a
    # magic number, and strictly tighter than the envelope.
    assert _rcn._RELOAD_CENTERED_TOL_MM == pytest.approx(env - swing)
    assert _rcn._RELOAD_CENTERED_TOL_MM < env

    # The invariant: worst-case admitted park + worst-case adverse swing still
    # fits inside the envelope, so an admitted reload can never be rejected
    # WORKSPACE mid-flight for being off centre.
    now = 100.0
    worst = _rcn._RELOAD_CENTERED_TOL_MM
    assert _node_fresh(now, (worst - 1e-6, 0.0, 170.0))._platform_centered(now) is True
    assert (worst + swing) <= env + 1e-9
    # And the band that used to be admitted-then-doomed is now refused.
    for park in (70.0, 75.0, 79.0, 80.0):
        assert _node_fresh(now, (park, 0.0, 170.0))._platform_centered(now) is False, park


def test_platform_centered_is_fail_closed_when_the_pose_is_unknown_or_stale():
    """No commanded pose, or a stale one, reads NOT centred. trajectory_node
    publishes the topic only while seeded+streaming, so silence genuinely means
    "no commanded pose exists" — the state where assuming centre would arm a
    reload catch against an unknown platform position."""
    import jugglebot.reload_coordinator_node as _rcn
    now = 100.0
    node = _node_fresh(now)
    with node._lock:
        node._commanded_pos_mm = None
        node._commanded_pos_mono = 0.0
    assert node._platform_centered(now) is False
    node = _node_fresh(now)
    with node._lock:
        node._commanded_pos_mono = now - (_rcn._TRAJ_STATUS_STALE_S + 1.0)
    assert node._platform_centered(now) is False


def test_reload_does_not_declare_a_reach_centre(monkeypatch):
    """The RELOAD choreography is untouched by contract C-REACH-1: its catch IS
    the held pose, so the commanded-pose fallback is already the right centre and
    it must publish nothing on catch/reach_center. A declaration here would be a
    behaviour change on the shipping path — the toss declares, the reload does
    not, and that asymmetry is the whole reason the reload stays byte-identical."""
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    now = 100.0
    node = _node_fresh(now)
    monkeypatch.setattr(node, '_prime_hand_with_retries', lambda: True)
    monkeypatch.setattr(node, '_prepare_catch', lambda: True)
    monkeypatch.setattr(node, '_call_reload', lambda: True)
    monkeypatch.setattr(node, '_send_throw', lambda s: (True, ''))
    monkeypatch.setattr(node, '_recenter', lambda: None)
    monkeypatch.setattr(node, '_safe_abort', lambda: None)
    seq = ReloadSequencer(catch_point_mm=node._catch_point_mm, throw_delay_s=3.0)
    seq.start(now)
    pub = node._publishers['catch/reach_center']
    for t in (now, now + 0.1, now + 0.2, now + 0.3):
        node._step_sequence(seq, t)
    assert seq.prepared is True              # it really did arm
    assert pub.published == []


def _obs_stub():
    from jugglebot.reload_sequencer import ReloadObservations, BB_STATE_IDLE
    return ReloadObservations(
        now=0.0, control_mode=RELOAD_CONTROL_MODE, bb_connected=True,
        bb_state=BB_STATE_IDLE, ball_in_hand=True, mocap_fresh=True,
        streaming=True, platform_centered=True)


def test_sequence_deadline_never_lands_inside_the_flight_window():
    """The node-level hard ceiling is derived per-goal from the FSM's own budgets.
    A fixed 30 s ceiling with an operator-supplied throw_delay_s ~ 16+ s would fire
    MID-FLIGHT and SAFE_ABORT under the airborne ball (the hazard class the
    settle-resolution fix closed). Audit finding, 2026-07-23."""
    from jugglebot.reload_coordinator_node import (
        _MAX_SEQUENCE_S, _sequence_deadline_s)
    short = ReloadSequencer(catch_point_mm=(0, 0, 809.08), throw_delay_s=3.0)
    assert _sequence_deadline_s(short) == _MAX_SEQUENCE_S      # floor holds
    long = ReloadSequencer(catch_point_mm=(0, 0, 809.08), throw_delay_s=20.0)
    ceiling = _sequence_deadline_s(long)
    # Must clear reload wait (10) + delay (20) + grace (0.5) + confirm (0.7).
    assert ceiling > 10.0 + 20.0 + 0.5 + 0.7
    assert ceiling >= _MAX_SEQUENCE_S


# ── CAUGHT plausibility + phantom exclusion (2026-07-23 re-test hardening) ─────

def test_implausible_caught_not_counted():
    """The tracker's split-track corruption flips CAUGHT on tracks coasting below
    the floor near BB — 5 of 6 reloads on 2026-07-23 spuriously reported SUCCESS.
    A caught ball only confirms if its final estimate is physically near the catch
    point."""
    now = 100.0
    node = _node_fresh(now)
    with node._lock:
        node._balls = [_Ball(status=1, id=5)]
        node._balls_mono = now
    node._build_observations(now)                          # latch id 5
    # The corrupt track flips CAUGHT below the floor near BB.
    with node._lock:
        node._balls = [_Ball(status=2, id=5, x=-539.0, y=-323.0, z=-532.0)]
        node._balls_mono = now
    obs = node._build_observations(now)
    assert obs.ball_caught is False                        # implausible → not counted


def test_plausible_caught_still_counts():
    now = 100.0
    node = _node_fresh(now)
    with node._lock:
        node._balls = [_Ball(status=1, id=5)]
        node._balls_mono = now
    node._build_observations(now)
    with node._lock:
        node._balls = [_Ball(status=2, id=5, x=30.0, y=15.0, z=805.0)]
        node._balls_mono = now
    obs = node._build_observations(now)
    assert obs.ball_caught is True
    assert obs.catch_error_mm == pytest.approx((30.0**2 + 15.0**2) ** 0.5)


def test_preexisting_flight_ball_never_latched():
    """A ball already IN_FLIGHT when the throw was accepted is NOT our ball —
    attempt 5 of the re-test latched a phantom untagged track that predated the
    throw and rode it to a wrong MISSED verdict."""
    now = 100.0
    node = _node_fresh(now)
    with node._lock:
        node._preexisting_flight_ids = {14}
        node._balls = [_Ball(status=1, id=14, destination=''),   # the phantom
                       _Ball(status=1, id=13, destination='')]   # the real ball
        node._balls_mono = now
    node._build_observations(now)
    assert node._announced_ball_id == 13


def test_destination_match_preferred_over_untagged():
    now = 100.0
    node = _node_fresh(now)
    with node._lock:
        node._balls = [_Ball(status=1, id=8, destination=''),
                       _Ball(status=1, id=9, destination='jugglebot')]
        node._balls_mono = now
    node._build_observations(now)
    assert node._announced_ball_id == 9


# ── The possession seam (C-POSSESS-1, ros_ws/docs/ball_possession_contract.md) ─
# The verdict surface itself is scored on measured fixtures in
# tests/ros/test_ball_possession.py; here we test the NODE's half — the tolerance
# it constructs the source with, that every caller routes through the one seam,
# and that the log an operator scores by eye says what was observed.


def _latch(node, now, ball_id=5):
    """Put ball_id IN_FLIGHT so _update_announced_ball_latch adopts it."""
    with node._lock:
        node._balls = [_Ball(status=1, id=ball_id)]
        node._balls_mono = now
    node._build_observations(now)


def test_possession_tolerance_is_the_catching_aperture():
    """Drift guard on the bound: it is the catching structure's entry aperture,
    not a tuned number, so a geometry change moves it. The 200.0 it replaced sat
    4.9 mm under the measured corrupt-track floor (204.9 mm) — a 1.02x margin
    against minting a FALSE CAUGHT."""
    from jugglebot.ball_possession import lateral_miss_mm
    from jugglebot.reload_coordinator_node import _CAUGHT_MAX_XY_ERROR_MM
    assert _CAUGHT_MAX_XY_ERROR_MM == pytest.approx(hw.GEOM_ARM_RADIUS_MM)
    node = ReloadCoordinatorNode()
    assert node._possession_source.arrival_tol_mm == pytest.approx(
        _CAUGHT_MAX_XY_ERROR_MM)
    # The margins, asserted against the SHIPPED constant rather than the geometry
    # it is sourced from — otherwise a re-tune of the constant alone would leave
    # the margin claim green while the shipped gate degraded.
    ref = node._catch_point_mm
    clean = max(lateral_miss_mm((x, y, z), ref)
                for _i, x, y, z in fx.SELF_TOSS_CAUGHT)
    corrupt = min(lateral_miss_mm((x, y, z), ref)
                  for _i, x, y, z in fx.RELOAD_CAUGHT)
    assert _CAUGHT_MAX_XY_ERROR_MM / clean > 18.0        # above real catches
    assert corrupt / _CAUGHT_MAX_XY_ERROR_MM > 2.9       # under corrupt tracks


@pytest.mark.parametrize('ball_id,x,y,z', fx.SELF_TOSS_CAUGHT)
def test_node_counts_every_real_self_toss_catch(ball_id, x, y, z):
    """End to end through the node: all 17 real catches of 2026-07-27 reported
    MISSED because the deleted z bound (305-1007 mm of dead-reckoning) could never
    pass. RED for every one of the 17 against the pre-2026-07-28 gate."""
    now = 100.0
    node = _node_fresh(now)
    _latch(node, now)
    with node._lock:
        node._balls = [_Ball(status=2, id=5, x=x, y=y, z=z)]
        node._balls_mono = now
    obs = node._build_observations(now)
    assert obs.ball_caught is True
    assert obs.catch_error_mm < hw.GEOM_ARM_RADIUS_MM


@pytest.mark.parametrize('ball_id,x,y,z', fx.RELOAD_CAUGHT)
def test_node_refuses_every_corrupt_reload_track(ball_id, x, y, z):
    """The negative set stays rejected end to end."""
    now = 100.0
    node = _node_fresh(now)
    _latch(node, now)
    with node._lock:
        node._balls = [_Ball(status=2, id=5, x=x, y=y, z=z)]
        node._balls_mono = now
    assert node._build_observations(now).ball_caught is False


def test_possession_source_is_pluggable_at_one_seam():
    """C-POSSESS-1 § 3: the ball-in-cup hand sensor becomes the PRIMARY source by
    replacing ONE attribute — no call site changes.

    Proven, not asserted: a substitute source flips the verdict on a fixture the
    tracker source refuses, and it is reached from all THREE consumers (the
    possession latch in _on_balls, the reload observation builder, and the toss
    observation builder), so none of them has its own private copy of the rule."""
    from jugglebot.ball_possession import (
        PossessionVerdict, RETENTION_CONFIRMED, SOURCE_HAND_BALL_SENSOR)

    class _AlwaysYes:
        name = SOURCE_HAND_BALL_SENSOR

        def __init__(self):
            self.calls = []

        def judge(self, ball_xyz_mm, ref_point_mm):
            self.calls.append(tuple(ball_xyz_mm))
            return PossessionVerdict(self.name, True, RETENTION_CONFIRMED,
                                     0.0, 0.0, 'SENSOR_HELD')

    corrupt = fx.RELOAD_CAUGHT[0][1:]              # 726 mm out — tracker refuses
    now = 100.0
    node = _node_fresh(now)
    _latch(node, now)
    with node._lock:
        node._balls = [_Ball(status=2, id=5, x=corrupt[0], y=corrupt[1],
                             z=corrupt[2])]
        node._balls_mono = now
    assert node._build_observations(now).ball_caught is False   # tracker source

    fake = _AlwaysYes()
    node._possession_source = fake
    assert node._build_observations(now).ball_caught is True
    node._on_balls(types.SimpleNamespace(
        balls=[_Ball(status=2, id=5, x=corrupt[0], y=corrupt[1], z=corrupt[2])]))
    assert node._ball_possession is True
    node._build_toss_observations(now)
    assert len(fake.calls) >= 3            # all three consumers reached the seam


def test_possession_verdict_logged_once_per_outcome():
    """One INFO line per (ball, verdict) — the accepted line is what the bench
    scores the gate against by eye (runbook row POSS-1), and the refused line must
    name the dead-reckoning error model rather than reading as an error in a
    working sequence."""
    now = 100.0
    node = _node_fresh(now)
    rec = _RecLogger()
    node._logger = rec
    _latch(node, now)
    clean = fx.SELF_TOSS_CAUGHT[0][1:]
    with node._lock:
        node._balls = [_Ball(status=2, id=5, x=clean[0], y=clean[1], z=clean[2])]
        node._balls_mono = now
    for _ in range(4):
        node._build_observations(now)
    ok_lines = [m for lvl, m in rec.records
                if 'Ball 5' in m and 'CONFIRMED' in m]
    assert len(ok_lines) == 1
    assert all(lvl == 'info' for lvl, m in rec.records if 'Ball 5' in m)
    assert 'UNKNOWN' in ok_lines[0]          # retention is stated, never implied

    corrupt = fx.RELOAD_CAUGHT[0][1:]
    with node._lock:
        node._balls = [_Ball(status=2, id=7, x=corrupt[0], y=corrupt[1],
                             z=corrupt[2])]
        node._balls_mono = now
        node._announced_ball_id = 7
    for _ in range(4):
        node._build_observations(now)
    no_lines = [m for _l, m in rec.records if 'Ball 7' in m and 'REFUSED' in m]
    assert len(no_lines) == 1
    assert 'dead-reckoned' in no_lines[0]


def test_catch_error_is_horizontal_only():
    """`catch_error_mm` is a LATERAL miss. Folding the vertical component in would
    report 305-1007 mm of dead-reckoning as catch inaccuracy on every real catch —
    and it is the same formula the gate decides on (one computation, not two)."""
    node = ReloadCoordinatorNode()
    ball = _Ball(status=2, id=5, x=30.0, y=40.0, z=-500.0)
    assert node._catch_error_from_ball(ball) == pytest.approx(50.0)
    assert node._possession_confirmed(ball) is True


# ── catch_vel_scale relay + prime retry ────────────────────────────────────────

def test_prepare_publishes_vel_scale_before_armed(monkeypatch):
    """The goal's catch-speed knob is published BEFORE the armed edge so
    catch_coordinator holds it before any hand-arm can fire."""
    node = ReloadCoordinatorNode()
    with node._lock:
        node._catch_vel_scale = 0.75
    monkeypatch.setattr(node, '_arm_catch', lambda a: True)
    assert node._prepare_catch() is True
    assert node._publishers['catch/vel_scale'].published[-1].data == pytest.approx(0.75)
    assert node._publishers['catch/armed'].published[-1].data is True


def test_goal_vel_scale_zero_defaults_to_config(monkeypatch):
    """Field default 0 (unset) means the config default (0.8, locked in from the
    2026-07-23 third sitting) — plumbed per-goal, never sticky."""
    node = ReloadCoordinatorNode()
    calls = []
    monkeypatch.setattr(node, '_step_sequence',
                        lambda seq, now, gh=None: (_ for _ in ()).throw(SystemExit))
    goal = types.SimpleNamespace(
        request=types.SimpleNamespace(throw_delay_s=3.0, catch_vel_scale=0.0),
        is_cancel_requested=False)
    try:
        node._execute_reload(goal)
    except SystemExit:
        pass
    with node._lock:
        assert node._catch_vel_scale == pytest.approx(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)


def test_prime_retry_ladder_on_transient_failures(monkeypatch):
    """The HAND_TRAJ_CMD dispatch path failed ~40-60% PER CALL across the third
    sitting (firmware CAN-enqueue errors); goal 1 died to two back-to-back
    failures. The ladder (_HAND_DISPATCH_ATTEMPTS = 4) rides out a streak: here
    three failures then a success still primes."""
    node = _node_fresh(100.0)
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)   # skip the ack-settle wait
    results = iter([False, False, False, True])
    attempts = []
    monkeypatch.setattr(node, '_smooth_move_hand',
                        lambda p: attempts.append(p) or next(results))
    seq = ReloadSequencer(catch_point_mm=node._catch_point_mm, throw_delay_s=3.0)
    seq.start(100.0)
    d = node._step_sequence(seq, 100.0)
    assert d.action == ACTION_PRIME_HAND
    assert len(attempts) == 4                  # failed 3×, 4th succeeded
    # Every dispatch was announced on catch/prime_dispatched BEFORE it went out,
    # so catch_coordinator's anti-stutter window covers the whole ladder.
    assert len(node._publishers['catch/prime_dispatched'].published) == 4
    d = node._step_sequence(seq, 100.1)
    assert not d.done                          # sequence continues (prime ok)


def test_prime_ladder_exhaustion_still_aborts(monkeypatch):
    node = _node_fresh(100.0)
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)   # skip the ack-settle wait
    attempts = []
    monkeypatch.setattr(node, '_smooth_move_hand',
                        lambda p: attempts.append(p) or False)
    monkeypatch.setattr(node, '_safe_abort', lambda: None)
    seq = ReloadSequencer(catch_point_mm=node._catch_point_mm, throw_delay_s=3.0)
    seq.start(100.0)
    node._step_sequence(seq, 100.0)            # PRIME: all 4 dispatches fail
    assert len(attempts) == 4
    d = node._step_sequence(seq, 100.1)
    assert d.done and d.result.outcome == 'ABORTED_PRIME_FAILED'


def test_safe_abort_retract_retries(monkeypatch):
    """The third sitting's SAFE_ABORT retract failed outright on 7/12 aborts
    (same dispatch epidemic) and the hand silently stayed at top. The safing
    path gets the same retry ladder as the prime — and the catch/armed disarm
    goes out FIRST (audit): with the node still armed and _hand_primed
    unlatched, catch_coordinator's 0.5 s retry tick could re-prime mid-ladder
    and erase the in-flight retract on the Teensy's last-writer-wins queue."""
    node = _node_fresh(100.0)
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)   # skip the ack-settle wait
    results = iter([False, True])
    order = []
    monkeypatch.setattr(node, '_smooth_move_hand',
                        lambda p: order.append(('retract', p)) or next(results))
    monkeypatch.setattr(node, '_publish_catch_armed',
                        lambda armed: order.append(('disarm', armed)))
    monkeypatch.setattr(node, '_arm_catch', lambda armed: True)
    monkeypatch.setattr(node, '_go_home', lambda: True)
    node._safe_abort()
    assert order[0] == ('disarm', False)                 # tick stands down first
    assert order[1:] == [('retract', hw.JB_OP_HAND_RETRACT_REV)] * 2  # retried, ok


def test_untagged_latch_displaced_by_destination_match():
    """AUDIT (2026-07-23): an untagged latch is PROVISIONAL — a phantom spawning
    during the ~3 s countdown grabs the empty-destination fallback first; when the
    real destination-tagged track appears at release it must displace the phantom
    (attempt 5's wrong-verdict shape, one spawn-time shift away)."""
    now = 100.0
    node = _node_fresh(now)
    with node._lock:
        node._balls = [_Ball(status=1, id=14, destination='')]   # countdown phantom
        node._balls_mono = now
    node._build_observations(now)
    assert node._announced_ball_id == 14                          # provisional
    with node._lock:
        node._balls = [_Ball(status=1, id=14, destination=''),
                       _Ball(status=1, id=13, destination='jugglebot')]  # real ball
        node._balls_mono = now
    node._build_observations(now)
    assert node._announced_ball_id == 13                          # displaced
    # A tagged latch is NOT provisional — another tagged ball never displaces it.
    with node._lock:
        node._balls = [_Ball(status=1, id=99, destination='jugglebot'),
                       _Ball(status=1, id=13, destination='jugglebot')]
        node._balls_mono = now
    node._build_observations(now)
    assert node._announced_ball_id == 13


def test_negative_vel_scale_warns_and_defaults(monkeypatch):
    """AUDIT: a sign typo (-0.8 intending slow) must not silently become full
    speed; it warns and uses the default."""
    node = ReloadCoordinatorNode()
    monkeypatch.setattr(node, '_step_sequence',
                        lambda seq, now, gh=None: (_ for _ in ()).throw(SystemExit))
    goal = types.SimpleNamespace(
        request=types.SimpleNamespace(throw_delay_s=3.0, catch_vel_scale=-0.8),
        is_cancel_requested=False)
    try:
        node._execute_reload(goal)
    except SystemExit:
        pass
    with node._lock:
        assert node._catch_vel_scale == pytest.approx(hw.JB_OP_CATCH_VEL_SCALE_DEFAULT)


# ── telemetry-verified prime / retract ladders (fixes 3 + 4) ───────────────────
# The HAND_TRAJ_CMD ack LIES — failed 'ERR_TIMEOUT' acks were observed with the frame
# still transmitted and the hand moving. A blind re-dispatch on a lied ack rebuilds the
# min-jerk profile from the live mid-move position at v=0, yanking the moving hand — the
# fourth sitting's (2026-07-24) "hand jump up-down then abort" (a full 4/4 ladder
# restarting a live ascent -> false ABORTED_PRIME_FAILED). After a failed ack the ladder
# consults hand_telemetry: only re-dispatch if the hand is positively stationary away
# from the target, or telemetry is absent (fall back to today's blind ladder).

class _RecLogger:
    def __init__(self):
        self.records = []

    def info(self, msg, **kw): self.records.append(('info', msg))
    def warning(self, msg, **kw): self.records.append(('warning', msg))
    def warn(self, msg, **kw): self.records.append(('warning', msg))
    def error(self, msg, **kw): self.records.append(('error', msg))
    def debug(self, msg, **kw): self.records.append(('debug', msg))
    def fatal(self, msg, **kw): self.records.append(('fatal', msg))


def _set_hand_telemetry(node, pos, vel):
    with node._lock:
        node._hand_pos_meas = pos
        node._hand_vel_meas = vel
        node._hand_telemetry_mono = time.perf_counter()


def test_hand_telemetry_wiring():
    node = ReloadCoordinatorNode()
    assert 'hand_telemetry' in node._subscriptions


def test_prime_lying_ack_ascending_telemetry_succeeds(monkeypatch):
    """The ack lies (fails while the frame transmits and the hand ascends). Telemetry
    shows the hand rising off the bottom → the prime SUCCEEDS without a re-dispatch, so a
    lying-ack ladder no longer produces a false ABORTED_PRIME_FAILED."""
    node = _node_fresh(100.0)
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    attempts = []
    monkeypatch.setattr(node, '_smooth_move_hand',
                        lambda p: attempts.append(p) or False)   # every ack lies
    _set_hand_telemetry(node, pos=3.0, vel=4.0)                  # rising toward the top
    assert node._prime_hand_with_retries() is True
    assert len(attempts) == 1                                    # no restart of the live ascent


def test_prime_no_motion_telemetry_redispatches_then_aborts(monkeypatch):
    """Telemetry positively shows the hand stationary at the bottom (frame genuinely
    lost) → the ladder re-dispatches every attempt and aborts on genuine failure."""
    from jugglebot.reload_coordinator_node import _HAND_DISPATCH_ATTEMPTS
    node = _node_fresh(100.0)
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    attempts = []
    monkeypatch.setattr(node, '_smooth_move_hand', lambda p: attempts.append(p) or False)
    _set_hand_telemetry(node, pos=0.0, vel=0.0)                  # stationary at the bottom
    assert node._prime_hand_with_retries() is False
    assert len(attempts) == _HAND_DISPATCH_ATTEMPTS


def test_prime_stale_telemetry_preserves_blind_ladder(monkeypatch):
    """Absent/stale telemetry → cannot verify → fall back to today's blind ladder (a
    strict superset): 3 fails then a success still primes."""
    node = _node_fresh(100.0)
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    results = iter([False, False, False, True])
    attempts = []
    monkeypatch.setattr(node, '_smooth_move_hand',
                        lambda p: attempts.append(p) or next(results))
    # telemetry_mono left at 0.0 → unavailable → blind ladder
    assert node._prime_hand_with_retries() is True
    assert len(attempts) == 4


def test_retract_accepts_on_descending_telemetry(monkeypatch):
    """A lied retract ack with the hand already descending is accepted — no spurious
    exhausted-ladder ERROR / re-dispatch churn on the safing path."""
    node = _node_fresh(100.0)
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    attempts = []
    monkeypatch.setattr(node, '_smooth_move_hand', lambda p: attempts.append(p) or False)
    _set_hand_telemetry(node, pos=6.0, vel=-3.0)                 # still high, but descending
    assert node._retract_hand_with_retries() is True
    assert len(attempts) == 1                                    # accepted after the first lied ack


def test_retract_rejects_top_park_velocity_noise(monkeypatch):
    """PROBE-DRIVEN (2026-07-24 bag): a hand parked at TOP shows |vel| noise up to
    5.39 rev/s p99 (gravity-hold dither) — above the 2.0 rev/s moving threshold. A
    lost retract frame must NOT false-accept as 'descending' off that noise: the
    moving check requires the hand to have LEFT the top park band."""
    from jugglebot.reload_coordinator_node import _HAND_DISPATCH_ATTEMPTS
    node = _node_fresh(100.0)
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    attempts = []
    monkeypatch.setattr(node, '_smooth_move_hand', lambda p: attempts.append(p) or False)
    # Parked at top, a −3 rev/s noise spike at the sample instant.
    _set_hand_telemetry(node, pos=hw.JB_OP_HAND_CATCH_PRIME_REV - 0.05, vel=-3.0)
    assert node._retract_hand_with_retries() is False            # NOT accepted
    assert len(attempts) == _HAND_DISPATCH_ATTEMPTS              # kept re-dispatching


def test_retract_redispatches_on_stationary_telemetry(monkeypatch):
    """A retract with the hand positively stationary at the top re-dispatches the whole
    ladder and returns False (genuine failure → the exhausted-ladder ERROR stands)."""
    from jugglebot.reload_coordinator_node import _HAND_DISPATCH_ATTEMPTS
    node = _node_fresh(100.0)
    monkeypatch.setattr(time, 'sleep', lambda *a, **k: None)
    attempts = []
    monkeypatch.setattr(node, '_smooth_move_hand', lambda p: attempts.append(p) or False)
    _set_hand_telemetry(node, pos=hw.JB_OP_HAND_CATCH_PRIME_REV, vel=0.0)   # parked at top
    assert node._retract_hand_with_retries() is False
    assert len(attempts) == _HAND_DISPATCH_ATTEMPTS


# ── per-attempt outcome log (fix 6) ────────────────────────────────────────────

class _TerminalGoalHandle:
    is_cancel_requested = False
    request = types.SimpleNamespace(throw_delay_s=3.0, catch_vel_scale=0.0)

    def succeed(self): pass
    def abort(self): pass
    def publish_feedback(self, fb): pass


def test_reload_outcome_logged_on_terminal(monkeypatch):
    """The node logs ONE authoritative outcome line on a terminal decision — a working
    reload previously logged NO outcome (0 lines) and read as pure ladder spam."""
    node = _node_fresh(100.0)
    rec = _RecLogger()
    node._logger = rec
    done = ReloadDecision('SETTLING', 'none', True,
                          ReloadResult(False, 'MISSED', float('nan')))
    monkeypatch.setattr(node, '_step_sequence', lambda seq, now, gh=None: done)
    node._execute_reload(_TerminalGoalHandle())
    outcome = [(lvl, m) for lvl, m in rec.records if 'Reload MISSED' in m]
    assert len(outcome) == 1                                     # exactly one outcome line
    assert outcome[0][0] == 'warning'                           # not-CAUGHT → WARN


def test_reload_outcome_log_reports_catch_error():
    """A CAUGHT outcome logs at INFO with the catch error."""
    node = ReloadCoordinatorNode()
    rec = _RecLogger()
    node._logger = rec
    node._log_reload_outcome(ReloadResult(True, 'CAUGHT', 12.0))
    hits = [(lvl, m) for lvl, m in rec.records if 'Reload CAUGHT' in m]
    assert len(hits) == 1 and hits[0][0] == 'info'
    assert '12' in hits[0][1]
