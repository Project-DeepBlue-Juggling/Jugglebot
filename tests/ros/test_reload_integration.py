"""Integration test: the reload announcement → correlation → coordinator path.

The reload sequence relies on the EXISTING catch pipeline to convert a Ball Butler
throw into a platform catch target:

    ThrowAnnouncement  →  BallTracker (correlation: tag the tracked ball with its
                          announced destination + confirm it against mocap markers)
                       →  CatchCoordinator (policy: pick the catchable ball, compute the
                          receive-tilt catch pose)  →  catch/dynamic_target (which
                          trajectory_node turns into a build_catch plan while the RELOAD
                          action's catch-armed latch is raised — no persistent CATCH
                          mode; the action owns the latch for the flight window).

This test drives the REAL correlation engine (``BallTracker``) and the REAL coordinator
policy (``CatchCoordinator``) — the pure-Python logic the thin ``ball_tracker_node`` /
``catch_coordinator_node`` wrappers forward to — with SYNTHESIZED announcement + mocap.

Why synthesized, not a recorded rosbag replay: the ~/Desktop/rosbags captures predate
this trajectory/build_catch pipeline (their DynamicTargetCommand was produced by the
old MPC catch path), there is no rosbag2 reader in the ROS-mocked pytest environment,
and the per-node message seams are already covered by the per-node tests. Synthesized
messages driving the real engines is the robust, deterministic integration the task's
fallback sanctions.
"""

from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.tracking.matcher import BallTracker
from jugglebot.tracking.ball import BallStatus, TrackingConfidence
from jugglebot.catch_coordinator import CatchCoordinator

GRAVITY_MMPS2 = 9810.0
DT = hw.TRACKING_MOCAP_DT_S
LANDING_Z = hw.GEOM_INITIAL_HEIGHT_MM + hw.JB_OP_DEFAULT_ACTIVE_Z_MM + hw.HAND_CATCH_OFFSET_MM


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
    """A CatchCoordinator configured exactly as catch_coordinator_node configures it."""
    return CatchCoordinator(
        robot_name="jugglebot",
        initial_height_mm=hw.GEOM_INITIAL_HEIGHT_MM,
        landing_z_offset_mm=hw.JB_OP_DEFAULT_ACTIVE_Z_MM + hw.HAND_CATCH_OFFSET_MM,
        hand_catch_offset_mm=hw.HAND_CATCH_OFFSET_MM,
        catch_angle_limit_deg=30.0,
    )


def _confirm_ball(tracker, pos0, vel0, destination):
    """Announce a throw at ``destination`` and feed mocap markers along the arc until
    the tracker CONFIRMS an in-flight ball. Returns (ball, last_time)."""
    tracker.handle_announcement(
        initial_position=np.asarray(pos0, dtype=float),
        initial_velocity=np.asarray(vel0, dtype=float),
        throw_time=0.0,
        source="ball_butler",
        destination=destination,
    )
    t = DT
    ball = None
    for _ in range(400):
        pos = _ballistic_pos(pos0, vel0, t)
        if pos[2] < LANDING_Z + hw.TRACKING_MIN_HEIGHT_ABOVE_LANDING_MM + 5.0:
            break                       # stop before the ball reaches the catch plane
        tracker.process_frame([pos], t)
        b = tracker.get_ball(1)
        if b is not None and b.tracking == TrackingConfidence.CONFIRMED:
            # A few more frames for a stable landing estimate, then done — still well
            # before landing so the coordinator sees > min_lead of flight remaining.
            for _ in range(6):
                t += DT
                pos = _ballistic_pos(pos0, vel0, t)
                tracker.process_frame([pos], t)
            ball = tracker.get_ball(1)
            break
        t += DT
    return ball, t


def test_announcement_correlation_to_coordinator_emits_catch_target():
    """End-to-end: an announced BB throw at 'jugglebot' is correlated to a confirmed
    in-flight ball, and the coordinator emits a catch command with a receive tilt."""
    tracker = _make_tracker()
    # A descending throw with lateral velocity so the arrival velocity is off-vertical
    # → the coordinator computes a real receive tilt (the geometry build_catch carries
    # through the seat).
    pos0 = [250.0, 0.0, 2200.0]
    vel0 = [-250.0, 0.0, -500.0]
    ball, t_now = _confirm_ball(tracker, pos0, vel0, destination="jugglebot")

    assert ball is not None, "tracker never confirmed the announced ball"
    assert ball.status == BallStatus.IN_FLIGHT
    assert ball.tracking == TrackingConfidence.CONFIRMED
    assert ball.destination == "jugglebot"        # the correlation tag from the announcement

    coord = _make_coordinator()
    cmd = coord.update([ball], current_time=t_now)
    assert cmd is not None, "coordinator produced no catch command for the confirmed ball"
    assert cmd.ball_id == ball.id
    # Stationary catch: the coordinator never asks for an arrival velocity (velocity
    # matching is the hand's job; build_catch forces translational arrival vel to zero).
    assert np.allclose(cmd.target_vel, 0.0)
    # Arrival time is in the future (a positive lead the catch planner can meet).
    assert cmd.landing_time > t_now
    # A receive tilt is present (non-identity quaternion) — this is the tilt the
    # trajectory node feeds build_catch's tilt-through-seat.
    tilt_component = float(np.linalg.norm(cmd.target_quat[1:4]))
    assert tilt_component > 1e-3, "expected a non-identity receive tilt from lateral arrival"


def test_coordinator_ignores_ball_for_other_destination():
    """The correlation tag is load-bearing: a ball announced for another robot is not
    caught (the destination filter is what routes a reload to THIS robot)."""
    tracker = _make_tracker()
    pos0 = [250.0, 0.0, 2200.0]
    vel0 = [-250.0, 0.0, -500.0]
    ball, t_now = _confirm_ball(tracker, pos0, vel0, destination="someone_else")
    assert ball is not None and ball.destination == "someone_else"

    coord = _make_coordinator()
    assert coord.update([ball], current_time=t_now) is None


def test_vertical_throw_gives_near_level_catch():
    """A straight-down arrival yields a (near) level catch pose — the tilt-to-receive
    degenerates to identity, so build_catch would take the level (reach+hold) branch."""
    tracker = _make_tracker()
    pos0 = [0.0, 0.0, 2200.0]
    vel0 = [0.0, 0.0, -500.0]
    ball, t_now = _confirm_ball(tracker, pos0, vel0, destination="jugglebot")
    assert ball is not None
    coord = _make_coordinator()
    cmd = coord.update([ball], current_time=t_now)
    assert cmd is not None
    tilt_component = float(np.linalg.norm(cmd.target_quat[1:4]))
    assert tilt_component < 5e-2, "a vertical arrival should be a near-level catch"
