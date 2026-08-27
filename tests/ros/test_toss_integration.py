"""Integration tests: the toss's self-announcement through the REAL catch path.

Two cross-component seams that no mocked-ROS unit test covers:

1. **Thrower-agnosticism of the announcement → correlation → catch-target path**
   (the plan's Phase-1 mandate). The toss publishes a ThrowAnnouncement with
   ``thrower_name='jugglebot'`` — a name the pipeline has never carried (BB owns
   every announcement to date). The path must key on ``target_id`` alone:
   destination comes solely from it in the tracker, and the coordinator's
   candidacy check reads destination. This test drives the ACTUAL message the
   coordinator node publishes through the REAL ``BallTracker`` correlation engine
   (mapped exactly as ``ball_tracker_node._on_announcement`` maps the wire fields)
   and the REAL ``CatchCoordinator`` policy, asserting a catch command (the
   would-be ``catch/dynamic_target``) comes out the other end — with the
   round-trip frame identity: nominated stow pose in ⇒ same stow pose out.

2. **prime_hold suppression, end-to-end in real publish order.** The toss's
   PREPARE bundle and terminal teardown are replayed into a REAL
   ``CatchCoordinatorNode`` in the exact order the toss node emits the messages,
   pinning that no auto-prime is dispatched anywhere between PREPARE and
   terminal (edge AND retry tick) while the kind-1 catch arm still fires.

3. **T-I2 — the same suppression under S6's SESSION-scoped holds.** The holds
   are raised once for a whole chained run and must survive every cycle's STAY
   terminal; the pin is an absence across three cycles, which is what makes the
   97 %→100 % armed duty cycle S5′ point 1 accepts a safe trade.

4. **T-I3 — the reach-envelope centre under a standing latch**, driven into the
   REAL ``trajectory_node``. It asserts the CAPTURED ``_catch_envelope_center``
   rather than publish ordering, because ``_svc_arm_catch`` read-and-clears the
   pending declaration BEFORE its idempotent early return: a per-cycle
   declaration under a standing latch is consumed and DISCARDED while every
   ordering check stays green (the Q-3 false green).

Why synthesized mocap, not a rosbag: same rationale as test_reload_integration
(no rosbag2 reader under the mocked-ROS pytest environment; synthesized markers
driving the real engines is the sanctioned deterministic integration).

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

import time as _time
import types

import numpy as np
import pytest

from std_msgs.msg import Bool

import jugglebot.hardware_config as hw
from jugglebot.tracking.matcher import BallTracker
from jugglebot.tracking.ball import BallStatus, TrackingConfidence
from jugglebot.catch_coordinator import CatchCoordinator
from jugglebot.catch_coordinator_node import CatchCoordinatorNode
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.reload_coordinator_node import (
    ReloadCoordinatorNode,
    _TOSS_SESSION_REACH_DRIFT_TOL_MM,
)
from jugglebot.toss_sequencer import TIER_8B, TossSequencer
from jugglebot.motion.trajectory.toss_release import compute_release_state_tilted
from tests.ros.test_toss_coordinator import _install_toss_goal
from tests.ros.test_catch_coordinator_node import _balls_msg, _catchable_cmd
from tests.ros.test_trajectory_node import (
    _arm_catch,
    _declare_reach_center,
    _dyn_target,
    _traj_mode_node,
)

# Tracker-side gravity (the mocap world the tracker fits against) — deliberately
# the 9810 the tracking stack uses, vs the 9806 the announcement's ballistics
# used: the ~0.2 mm divergence over a flight is far inside the 100 mm match
# threshold, and running both constants IS part of the integration being tested.
GRAVITY_MMPS2 = 9810.0
DT = hw.TRACKING_MOCAP_DT_S
LANDING_Z = (hw.GEOM_INITIAL_HEIGHT_MM + hw.JB_OP_DEFAULT_ACTIVE_Z_MM
             + hw.HAND_CATCH_OFFSET_MM)


def _ballistic_pos(pos0, vel0, t):
    return np.array([
        pos0[0] + vel0[0] * t,
        pos0[1] + vel0[1] * t,
        pos0[2] + vel0[2] * t - 0.5 * GRAVITY_MMPS2 * t * t,
    ])


def _make_tracker():
    """A BallTracker configured exactly as ball_tracker_node configures it."""
    return BallTracker(
        dt=DT,
        landing_z=LANDING_Z,
        match_threshold_base_mm=hw.TRACKING_MATCH_THRESHOLD_BASE_MM,
        parabolic_min_frames=hw.TRACKING_MIN_MATCHES_TO_CONFIRM,
        missed_frames_to_lose=10,
        max_frames_without_measurement=hw.TRACKING_MAX_FRAMES_WITHOUT_MEASUREMENT,
        process_noise=hw.TRACKING_PROCESS_NOISE,
        measurement_noise=hw.TRACKING_MEASUREMENT_NOISE,
        min_height_above_landing_mm=hw.TRACKING_MIN_HEIGHT_ABOVE_LANDING_MM,
    )


def _make_coordinator():
    """A CatchCoordinator configured exactly as catch_coordinator_node does."""
    return CatchCoordinator(
        robot_name="jugglebot",
        initial_height_mm=hw.GEOM_INITIAL_HEIGHT_MM,
        landing_z_offset_mm=hw.JB_OP_DEFAULT_ACTIVE_Z_MM + hw.HAND_CATCH_OFFSET_MM,
        hand_catch_offset_mm=hw.HAND_CATCH_OFFSET_MM,
        catch_angle_limit_deg=30.0,
    )


def _publish_real_toss_announcement(pose=(0.0, 0.0, 170.0), flight=0.8):
    """Run the toss node's ACTUAL announce executor and return the published
    wire message — the genuine article, not a hand-built lookalike."""
    node = ReloadCoordinatorNode()
    _install_toss_goal(node, pose=pose, flight=flight)
    seq = TossSequencer(catch_pose_stow_mm=pose, flight_time_s=flight,
                        throw_delay_s=5.0)
    import time as _time
    seq.start(_time.perf_counter())
    seq._prepare_dispatched = True        # satisfy note_announcement's gate
    node._announce_toss(seq)
    return node._publishers['throw_announcements'].published[-1]


def _feed_announcement_to_tracker(tracker, ann):
    """Map the wire message into the tracker EXACTLY as
    ball_tracker_node._on_announcement does: throw_time from the stamp (the
    mock clock packs ns ints), source from thrower_name, destination SOLELY
    from target_id, landing kwargs when the landing stamp is positive."""
    throw_time_s = float(ann.throw_time) * 1e-9
    assert throw_time_s > 1.0             # the node's "immediate" sentinel not taken
    landing_time_s = float(ann.landing_time) * 1e-9
    tracker.handle_announcement(
        initial_position=np.array([ann.initial_position.x,
                                   ann.initial_position.y,
                                   ann.initial_position.z]),
        initial_velocity=np.array([ann.initial_velocity.x,
                                   ann.initial_velocity.y,
                                   ann.initial_velocity.z]),
        throw_time=throw_time_s,
        source=ann.thrower_name or "ball_butler",
        destination=ann.target_id if ann.target_id else "",
        landing_position=np.array([ann.landing_position.x,
                                   ann.landing_position.y,
                                   ann.landing_position.z]),
        landing_velocity=np.array([ann.landing_velocity.x,
                                   ann.landing_velocity.y,
                                   ann.landing_velocity.z]),
        landing_time=landing_time_s,
    )
    return throw_time_s


def _confirm_toss_ball(tracker, ann):
    """Feed synthesized mocap markers along the ANNOUNCED arc until the tracker
    CONFIRMS the ball. The vertical toss rises from below the min-track height,
    so feeding starts once the arc clears landing + min_height (the tracker
    ignores markers below it) and stops before the descent re-crosses it —
    mirroring test_reload_integration's _confirm_ball."""
    throw_time_s = _feed_announcement_to_tracker(tracker, ann)
    pos0 = np.array([ann.initial_position.x, ann.initial_position.y,
                     ann.initial_position.z])
    vel0 = np.array([ann.initial_velocity.x, ann.initial_velocity.y,
                     ann.initial_velocity.z])
    floor = LANDING_Z + hw.TRACKING_MIN_HEIGHT_ABOVE_LANDING_MM + 5.0
    t_rel = DT
    ball = None
    t = throw_time_s
    for _ in range(400):
        pos = _ballistic_pos(pos0, vel0, t_rel)
        descending = vel0[2] - GRAVITY_MMPS2 * t_rel < 0.0
        if descending and pos[2] < floor:
            break                          # stop before the catch plane
        t = throw_time_s + t_rel
        if pos[2] >= floor:
            tracker.process_frame([pos], t)
            b = tracker.get_ball(1)
            if b is not None and b.tracking == TrackingConfidence.CONFIRMED:
                # A few more frames for a stable landing estimate, still well
                # before landing (the coordinator needs > min_lead remaining).
                for _ in range(6):
                    t_rel += DT
                    t = throw_time_s + t_rel
                    tracker.process_frame(
                        [_ballistic_pos(pos0, vel0, t_rel)], t)
                ball = tracker.get_ball(1)
                break
        t_rel += DT
    return ball, t


def test_self_announcement_correlates_and_emits_catch_target():
    """THE thrower-agnosticism pin: the toss node's real announcement
    (thrower_name='jugglebot' — a name the pipeline never carried) correlates
    through the REAL tracker to a CONFIRMED ball destined for us, and the REAL
    coordinator emits a catch command. Destination comes SOLELY from target_id;
    thrower_name is a cosmetic source label."""
    ann = _publish_real_toss_announcement(pose=(0.0, 0.0, 170.0), flight=0.8)
    tracker = _make_tracker()
    ball, t_now = _confirm_toss_ball(tracker, ann)

    assert ball is not None, "tracker never confirmed the toss's announced ball"
    assert ball.status == BallStatus.IN_FLIGHT
    assert ball.tracking == TrackingConfidence.CONFIRMED
    assert ball.destination == "jugglebot"    # from target_id — the load-bearing tag
    assert ball.source == "jugglebot"         # thrower_name carried as the label

    coord = _make_coordinator()
    cmd = coord.update([ball], current_time=t_now)
    assert cmd is not None, "coordinator produced no catch command for our toss"
    assert cmd.ball_id == ball.id
    # Vertical arrival ⇒ (near) level catch — build_catch would take the level
    # reach branch, exactly the Tier-8a premise (platform stays level).
    tilt_component = float(np.linalg.norm(cmd.target_quat[1:4]))
    assert tilt_component < 5e-2, "a vertical self-toss should be a level catch"
    # Round-trip frame identity: the nominated STOW pose (0, 0, 170) went out as
    # a global announcement and comes back as the same STOW platform pose — the
    # single-conversion-point discipline, end to end (the 2026-07-23 z-bug class).
    assert cmd.target_pos[2] == pytest.approx(170.0, abs=20.0)
    assert np.hypot(cmd.target_pos[0], cmd.target_pos[1]) < 20.0


def test_offcentre_toss_lands_catch_target_at_nominated_xy():
    """The nominated (x, y) carries through the whole real path: announce at the
    corner pose, correlate, and the coordinator's catch target comes back AT the
    nominated platform (x, y) — not at the centre reload point."""
    ann = _publish_real_toss_announcement(pose=(60.0, -60.0, 170.0), flight=0.8)
    assert ann.landing_position.x == pytest.approx(60.0)
    assert ann.landing_position.y == pytest.approx(-60.0)
    tracker = _make_tracker()
    ball, t_now = _confirm_toss_ball(tracker, ann)
    assert ball is not None and ball.destination == "jugglebot"
    cmd = _make_coordinator().update([ball], current_time=t_now)
    assert cmd is not None
    assert cmd.target_pos[0] == pytest.approx(60.0, abs=20.0)
    assert cmd.target_pos[1] == pytest.approx(-60.0, abs=20.0)


def test_untagged_announcement_never_becomes_catchable():
    """The inverse pin: strip target_id and the same physics never produces a
    catch candidate — destination stays empty, the coordinator ignores it. This
    is what makes target_id='jugglebot' MANDATORY in the announce executor."""
    ann = _publish_real_toss_announcement()
    ann.target_id = ""                        # the one field under test
    tracker = _make_tracker()
    ball, t_now = _confirm_toss_ball(tracker, ann)
    assert ball is not None
    assert ball.destination == ""
    assert _make_coordinator().update([ball], current_time=t_now) is None


def test_prepare_to_terminal_replay_suppresses_ccn_autoprime(monkeypatch):
    """END-TO-END prime_hold: capture the toss node's PREPARE-choreography and
    terminal publishes in their REAL emission order — the hold goes out ALONE
    on the verified-arrival tick, the bundle (vel_scale → prime_dispatched
    stamp → armed) one FSM tick later — replay them into a REAL
    CatchCoordinatorNode, and pin: no auto-prime anywhere from PREPARE to
    terminal (edge AND retry tick — a kind-3 ascent would launch the seated
    ball and clear the armed throw stroke), while the kind-1 catch arm still
    dispatches (hand-stroke catch timing stays tracker-driven), the vel_scale
    relay lands, and the teardown releases the hold strictly AFTER the disarm."""
    toss_node = ReloadCoordinatorNode()
    _install_toss_goal(toss_node, vel_scale=0.75)
    monkeypatch.setattr(toss_node, '_set_soft_catch_gains', lambda: True)
    monkeypatch.setattr(toss_node, '_arm_catch', lambda a: True)
    monkeypatch.setattr(toss_node, '_go_home', lambda: True)
    events = []
    for topic in ('catch/prime_hold', 'catch/vel_scale',
                  'catch/prime_dispatched', 'catch/armed'):
        pub = toss_node._publishers[topic]
        monkeypatch.setattr(
            pub, 'publish',
            (lambda _topic, _orig: lambda msg: (events.append((_topic, msg)),
                                                _orig(msg)))(topic, pub.publish))
    # The verified-arrival tick publishes the hold ALONE (see
    # _step_toss_sequence); the bundle runs on the next tick.
    toss_node._publish_prime_hold(True)
    assert toss_node._prepare_toss_catch() is True
    assert [t for t, _ in events][0] == 'catch/prime_hold'

    ccn = CatchCoordinatorNode()
    primed = []
    monkeypatch.setattr(ccn, '_prime_hand', lambda: primed.append(1))

    def _replay():
        while events:
            topic, msg = events.pop(0)
            if topic == 'catch/prime_hold':
                ccn._on_prime_hold(msg)
            elif topic == 'catch/vel_scale':
                ccn._on_vel_scale(msg)
            elif topic == 'catch/prime_dispatched':
                ccn._on_prime_dispatched(msg)
            elif topic == 'catch/armed':
                ccn._on_catch_armed(msg)
    _replay()
    assert ccn._catch_armed is True
    assert ccn._catch_vel_scale == pytest.approx(0.75)   # the relay landed
    assert primed == []                                  # armed-edge prime suppressed
    # Retry tick with every OTHER precondition satisfied (armed, unprimed, quiet
    # window expired, no ascent in flight): the hold alone keeps suppressing.
    ccn._last_cmd_mono = 0.0
    ccn._prime_dispatch_mono = 0.0
    ccn._prime_retry_tick()
    assert primed == []
    # The kind-1 catch arm is NOT gated — the toss catch depends on it.
    armed = []
    monkeypatch.setattr(ccn, '_arm_hand_catch',
                        lambda d, v: armed.append((d, v)) or True)
    monkeypatch.setattr(ccn._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None:
                        _catchable_cmd())
    ccn._on_balls(_balls_msg())
    assert len(armed) == 1
    # Terminal teardown (CAUGHT): replay in real order — the disarm must land
    # BEFORE the hold release, so a still-armed CCN never meets a released hold
    # with the caught ball resting in the cup.
    toss_node._toss_recenter()
    assert [t for t, _ in events] == ['catch/armed', 'catch/prime_hold']
    _replay()
    assert ccn._catch_armed is False
    assert ccn._prime_hold is False
    # And priming works again for the next ball-op (a fresh armed edge primes).
    from std_msgs.msg import Bool
    ccn._prime_dispatch_mono = 0.0
    ccn._on_catch_armed(Bool(data=True))
    assert primed == [1]


def test_prime_dispatched_belt_covers_lost_hold(monkeypatch):
    """The FIX-4 BELT, end-to-end: if the prime_hold publish is LOST (or
    reordered past the armed edge), the ONE catch/prime_dispatched stamp the
    bundle emits immediately before catch/armed still suppresses the
    armed-edge auto-prime via CCN's hardware-proven 1.2 s prime-inflight
    window — the only auto-prime that can fire before the hold's next chance
    to land. A belt, not a heartbeat: no periodic re-stamping exists."""
    toss_node = ReloadCoordinatorNode()
    _install_toss_goal(toss_node)
    monkeypatch.setattr(toss_node, '_set_soft_catch_gains', lambda: True)
    monkeypatch.setattr(toss_node, '_arm_catch', lambda a: True)
    events = []
    for topic in ('catch/prime_dispatched', 'catch/armed'):
        pub = toss_node._publishers[topic]
        monkeypatch.setattr(
            pub, 'publish',
            (lambda _topic, _orig: lambda msg: (events.append((_topic, msg)),
                                                _orig(msg)))(topic, pub.publish))
    # NOTE: the hold publish is deliberately NOT replayed — the lost-message
    # scenario under test. Only the bundle's stamp + armed edge arrive at CCN.
    assert toss_node._prepare_toss_catch() is True
    assert [t for t, _ in events] == ['catch/prime_dispatched', 'catch/armed']
    ccn = CatchCoordinatorNode()
    primed = []
    monkeypatch.setattr(ccn, '_prime_hand', lambda: primed.append(1))
    for topic, msg in events:
        if topic == 'catch/prime_dispatched':
            ccn._on_prime_dispatched(msg)
        else:
            ccn._on_catch_armed(msg)
    assert ccn._catch_armed is True
    assert ccn._prime_hold is False              # the hold never arrived
    assert primed == []                          # the stamp's window covered the edge


def test_session_holds_suppress_ccn_autoprime_for_a_whole_session(monkeypatch):
    """**T-I2** — S6's standing latch replayed into a REAL CatchCoordinatorNode.

    The toss node's SESSION arming (catch/prime_hold + catch/reach_center +
    catch/pretilt_hold, then the ONE arm_catch raise and vel_scale) is emitted
    once; three chained cycles then emit only their per-cycle remainder
    (prime_dispatched → armed True … armed False at STAY). Every message is
    replayed into CCN in the real order the node emits it, and the pin is an
    ABSENCE across the WHOLE session: no auto-prime fires anywhere — not on any
    of the three armed edges, not on the retry tick between cycles — and no
    announcement pre-tilt is ever installed.

    That absence is the whole safety argument for S5′ point 1. Under S6 the
    armed duty cycle goes from ~97 % to 100 %, and what makes that acceptable is
    that both suppressors STAND for the entire run rather than being re-raised
    per cycle: a prime_hold that came down between cycles would let the next
    armed edge ascend the hand with the caught ball resting in the cup."""
    toss_node = ReloadCoordinatorNode()
    _install_toss_goal(toss_node, vel_scale=0.75)
    toss_node._toss_session_live = True
    monkeypatch.setattr(toss_node, '_set_soft_catch_gains', lambda: True)
    monkeypatch.setattr(toss_node, '_arm_catch', lambda a: True)
    events = []
    for topic in ('catch/prime_hold', 'catch/pretilt_hold', 'catch/vel_scale',
                  'catch/prime_dispatched', 'catch/armed'):
        pub = toss_node._publishers[topic]
        monkeypatch.setattr(
            pub, 'publish',
            (lambda _topic, _orig: lambda msg: (events.append((_topic, msg)),
                                                _orig(msg)))(topic, pub.publish))

    ccn = CatchCoordinatorNode()
    primed = []
    monkeypatch.setattr(ccn, '_prime_hand', lambda: primed.append(1))

    def _replay():
        while events:
            topic, msg = events.pop(0)
            {'catch/prime_hold': ccn._on_prime_hold,
             'catch/pretilt_hold': ccn._on_pretilt_hold,
             'catch/vel_scale': ccn._on_vel_scale,
             'catch/prime_dispatched': ccn._on_prime_dispatched,
             'catch/armed': ccn._on_catch_armed}[topic](msg)

    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0))
    # ── the SESSION arm, once ──
    toss_node._arm_session_declare(seq)
    assert [t for t, _ in events] == ['catch/prime_hold', 'catch/pretilt_hold']
    assert toss_node._arm_session(seq) is True
    _replay()
    assert ccn._prime_hold is True and ccn._pretilt_hold is True
    assert ccn._catch_vel_scale == pytest.approx(0.75)
    assert primed == []

    # ── three chained cycles, per-cycle remainder only ──
    for _ in range(3):
        assert toss_node._prepare_toss_catch(seq) is True
        assert [t for t, _ in events] == ['catch/prime_dispatched',
                                          'catch/armed']
        _replay()
        assert ccn._catch_armed is True
        assert primed == [], 'an armed edge under a standing hold must not prime'
        # The retry tick between cycles, with every OTHER precondition met
        # (armed, unprimed, quiet window expired, no ascent in flight): the
        # standing hold alone keeps suppressing.
        ccn._last_cmd_mono = 0.0
        ccn._prime_dispatch_mono = 0.0
        ccn._prime_retry_tick()
        assert primed == []
        # …and the CHAINING terminal (STAY): catch/armed False and NOTHING else.
        toss_node._toss_stay()
        assert [t for t, _ in events] == ['catch/armed']
        _replay()
        assert ccn._catch_armed is False
        assert ccn._prime_hold is True, 'the hold must OUTLIVE the cycle (S6)'
        assert ccn._pretilt_hold is True

    # No announcement pre-tilt was ever installed, either: the pretilt hold has
    # stood since before cycle 1 (the toss node owns the deferred A->B reach).
    n0 = len(ccn._dyn_target_pub.published)
    ann = _publish_real_toss_announcement()
    lt_ns = int(ann.landing_time)
    ccn._on_throw_announcement(types.SimpleNamespace(
        target_id=ann.target_id,
        landing_position=ann.landing_position,
        landing_velocity=ann.landing_velocity,
        landing_time=types.SimpleNamespace(sec=lt_ns // 1_000_000_000,
                                           nanosec=lt_ns % 1_000_000_000)))
    assert len(ccn._dyn_target_pub.published) == n0
    assert ccn._pretilt_cmd is None

    # ── the SESSION terminal: ONE lower, armed False BEFORE the hold release ──
    toss_node._disarm_session()
    assert [t for t, _ in events] == ['catch/armed', 'catch/prime_hold',
                                      'catch/pretilt_hold']
    _replay()
    assert ccn._catch_armed is False and ccn._prime_hold is False
    assert ccn._pretilt_hold is False
    assert primed == []                       # nothing primed anywhere, all run
    # And priming works again for the next ball-op (a fresh armed edge primes).
    ccn._prime_dispatch_mono = 0.0
    ccn._on_catch_armed(Bool(data=True))
    assert primed == [1]


def test_the_session_declaration_is_captured_once_and_admits_every_cycle(
        monkeypatch):
    """**T-I3** — the reach-envelope centre under a standing latch, through the
    REAL trajectory_node (Q-3, resolved 2026-08-27).

    It asserts the CAPTURED VALUE, never publish ordering, and that distinction
    is the finding: `_svc_arm_catch` read-and-CLEARS the pending declaration
    BEFORE its idempotent early return, so under a standing latch every
    per-cycle declaration is consumed and DISCARDED while the bench trace
    recorder's CS-4 (one declaration per cycle, ≥1 tick before the arm) stays
    green. CS-4 alone is a FALSE GREEN, and only reading `_catch_envelope_center`
    catches it.

    So: the session's ONE declaration is captured at its ONE raise, a catch
    target at B is accepted across three simulated cycles with no re-raise, a
    later declaration cannot move the centre, and the drift guard refuses the
    divergent-B cycle the frozen centre forecloses."""
    b = (150.0, 0.0, 190.0)
    traj = _traj_mode_node()
    toss_node = ReloadCoordinatorNode()
    _install_toss_goal(toss_node)
    toss_node._toss_session_live = True
    monkeypatch.setattr(toss_node, '_set_soft_catch_gains', lambda: True)
    # Wire the toss node's declaration and its arm_catch call into the REAL
    # trajectory node — the two transports whose ordering C-REACH-1 depends on.
    monkeypatch.setattr(toss_node._reach_center_pub, 'publish',
                        traj._on_reach_center)
    raises = []
    monkeypatch.setattr(
        toss_node, '_arm_catch',
        lambda a: raises.append(bool(a)) or bool(_arm_catch(traj, a).success))
    monkeypatch.setattr(toss_node, '_publish_pretilt_hold', lambda h: None)
    monkeypatch.setattr(toss_node, '_publish_prime_hold', lambda h: None)

    seq = TossSequencer(catch_pose_stow_mm=b)
    toss_node._arm_session_declare(seq)
    assert traj._catch_envelope_center is None      # pending only, not applied
    assert toss_node._arm_session(seq) is True

    # THE captured value — B, not the pose the platform is holding.
    assert np.allclose(traj._catch_envelope_center, b, atol=1e-9)
    assert traj._pending_reach_center is None       # consumed by the ONE raise
    captured = np.array(traj._catch_envelope_center, dtype=float)

    for _ in range(3):
        assert toss_node._prepare_toss_catch(seq) is True
        traj._on_dynamic_target(_dyn_target(traj, x=b[0], y=b[1], z=b[2],
                                            lead_s=0.8))
        fb = traj.target_feedback_pub.published[-1]
        assert fb.accepted is True, 'a catch at the declared B must be admitted'
        assert np.allclose(traj._catch_envelope_center, captured, atol=1e-9)
    assert raises == [True], 'S6: exactly ONE arm_catch raise for the whole run'

    # A LATER declaration cannot move the centre — the Q-3 finding, stated as a
    # value assertion. This is why the declaration is session-scoped.
    _declare_reach_center(traj, x=0.0, y=0.0, z=190.0)
    _arm_catch(traj, True)                          # the idempotent no-op
    assert np.allclose(traj._catch_envelope_center, captured, atol=1e-9)
    assert traj._pending_reach_center is None       # consumed and DISCARDED

    # …so a cycle nominating a different B is refused BEFORE anything is armed,
    # by the node-side drift guard — because the alternative is the mid-flight
    # WORKSPACE reject demonstrated on the line below, ball already airborne.
    drifted = (b[0] + _TOSS_SESSION_REACH_DRIFT_TOL_MM + 20.0, b[1], b[2])
    seq2 = TossSequencer(catch_pose_stow_mm=drifted)
    assert toss_node._prepare_toss_catch(seq2) is False
    assert toss_node._toss_prepare_reject == 'REACH_CENTER_DRIFT'
    traj._on_dynamic_target(_dyn_target(traj, x=drifted[0], y=drifted[1],
                                        z=drifted[2], lead_s=0.8))
    fb = traj.target_feedback_pub.published[-1]
    assert fb.accepted is False and fb.code == feas.WORKSPACE


def _toss_node_8b_announce(pose=(50.0, 0.0, 170.0), throw_site=(0.0, 0.0),
                           flight=0.8):
    """A real toss node with Tier-8b goal state installed (tilted release) and
    its ACTUAL announce executor run — populating BOTH the deferred-reach stash
    (the cycle state's ``announced_reach``) and the wire ThrowAnnouncement."""
    node = ReloadCoordinatorNode()
    release = compute_release_state_tilted(pose, flight, throw_site_xy_mm=throw_site)
    with node._lock:
        node._toss_committed.release_state = release
        node._toss_committed.landing_global_mm = tuple(
            float(v) for v in release.catch_point_global_mm)
        node._toss_committed.platform_target_mm = tuple(float(v) for v in pose)
        node._toss_committed.prepare_pending = False
        node._toss_committed.throw_dispatched = False
        node._toss_committed.pretilt_hold_raised = False
        node._toss_committed.announced_reach = None
    seq = TossSequencer(catch_pose_stow_mm=pose, flight_time_s=flight,
                        throw_delay_s=5.0, tier=TIER_8B, throw_site_xy_mm=throw_site,
                        throw_site_known=True,
                        event_vel_mps=float(release.event_vel_mps))
    seq.start(_time.perf_counter())
    seq._prepare_dispatched = True             # satisfy note_announcement's gate
    node._announce_toss(seq)
    ann = node._publishers['throw_announcements'].published[-1]
    return node, ann


def test_8b_pretilt_hold_suppresses_ccn_and_deferred_reach_is_sole_target(monkeypatch):
    """END-TO-END Tier 8b (the central Phase-4 choreography): a REAL 8b toss
    raises catch/pretilt_hold True (with catch/armed) into a REAL
    CatchCoordinatorNode, then its real announcement arrives. CCN publishes NO
    platform target from the announcement (it latches the open-loop freeze +
    hand-arm window instead), and a balls tick under open-loop publishes no
    platform target either while the hand-arm still fires tracker-driven. The
    toss node's OWN deferred A→B reach is then the SOLE platform target for the
    flight — the stock CCN pre-tilt (which would complete the A→B translate
    before release) is fully suppressed."""
    toss_node, ann = _toss_node_8b_announce()
    ccn = CatchCoordinatorNode()
    primed = []
    monkeypatch.setattr(ccn, '_prime_hand', lambda: primed.append(1))

    # PREPARE (8b): the toss raises catch/prime_hold AND catch/pretilt_hold
    # together (prime_hold suppresses the ball-laden auto-prime; pretilt_hold
    # suppresses the platform pre-tilt), then catch/armed one FSM tick later —
    # replay them into CCN in that real emission order.
    ccn._on_prime_hold(Bool(data=True))
    ccn._on_pretilt_hold(Bool(data=True))
    ccn._on_catch_armed(Bool(data=True))
    assert ccn._prime_hold is True and ccn._pretilt_hold is True
    assert ccn._catch_armed is True
    assert primed == []                                    # ball-laden auto-prime suppressed

    # The real 8b announcement (landing = the displaced B, lateral arrival). The
    # mock packs builtin_interfaces/Time as an int-ns via to_msg(); reshape it
    # into the .sec/.nanosec form CCN reads (the physics fields stay the real
    # wire Point/Vector3).
    n0 = len(ccn._dyn_target_pub.published)
    lt_ns = int(ann.landing_time)
    ccn_ann = types.SimpleNamespace(
        target_id=ann.target_id,
        landing_position=ann.landing_position,
        landing_velocity=ann.landing_velocity,
        landing_time=types.SimpleNamespace(sec=lt_ns // 1_000_000_000,
                                           nanosec=lt_ns % 1_000_000_000))
    ccn._on_throw_announcement(ccn_ann)
    assert len(ccn._dyn_target_pub.published) == n0        # NO platform target from CCN
    assert ccn._announcement_seen is True                  # latched for hand-arm window
    assert ccn._announced_landing_time is not None
    assert ccn._pretilt_cmd is None                        # no cached pre-tilt to republish

    # A balls tick under open-loop: STILL no platform target from CCN, hand arms.
    armed = []
    monkeypatch.setattr(ccn, '_arm_hand_catch',
                        lambda d, v: armed.append((d, v)) or True)
    cmd = _catchable_cmd()
    cmd.landing_time = ccn._announced_landing_time         # within the arm window
    monkeypatch.setattr(ccn._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None: cmd)
    ccn._on_balls(_balls_msg())
    assert len(ccn._dyn_target_pub.published) == n0        # STILL no platform target
    assert len(armed) == 1                                 # hand-arm engaged (reactive)
    assert primed == []                                    # no auto-prime meanwhile

    # The toss node's OWN deferred A→B reach is the SOLE platform target: it
    # publishes ONE catch/dynamic_target from the stashed announced landing via
    # the same predicted_catch_command policy CCN would have used.
    m0 = len(toss_node._dyn_target_pub.published)
    toss_node._publish_toss_reach()
    assert len(toss_node._dyn_target_pub.published) == m0 + 1
    out = toss_node._dyn_target_pub.published[-1]
    cmd_reach = toss_node._toss_catch_policy.predicted_catch_command(
        *toss_node._toss_committed.announced_reach)
    assert cmd_reach is not None
    assert out.target_pos.x == pytest.approx(cmd_reach.target_pos[0])
    assert out.target_pos.y == pytest.approx(cmd_reach.target_pos[1])
    assert out.target_pos.z == pytest.approx(cmd_reach.target_pos[2])
