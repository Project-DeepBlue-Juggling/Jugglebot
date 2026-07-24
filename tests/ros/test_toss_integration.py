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

Why synthesized mocap, not a rosbag: same rationale as test_reload_integration
(no rosbag2 reader under the mocked-ROS pytest environment; synthesized markers
driving the real engines is the sanctioned deterministic integration).

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.tracking.matcher import BallTracker
from jugglebot.tracking.ball import BallStatus, TrackingConfidence
from jugglebot.catch_coordinator import CatchCoordinator
from jugglebot.catch_coordinator_node import CatchCoordinatorNode
from jugglebot.reload_coordinator_node import ReloadCoordinatorNode
from jugglebot.toss_sequencer import TossSequencer
from tests.ros.test_toss_coordinator import _install_toss_goal
from tests.ros.test_catch_coordinator_node import _balls_msg, _catchable_cmd

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
