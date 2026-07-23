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


def test_catch_command_crosses_wire_into_accepted_build_catch():
    """THE CROSS-PROCESS FRAME TEST (closes the gap that hid the 2026-07-23 z bug).

    The pre-existing integration tests stop at the CatchCommand; the per-node
    trajectory tests synthesize their own wire messages. NOTHING checked that the
    coordinator's actual output, packed into DynamicTargetCommand exactly as
    catch_coordinator_node packs it, is interpreted in the SAME FRAME by
    trajectory_node — so a stow-relative producer met an
    assumed-active-relative consumer, every hardware catch reach commanded
    z ≈ 341 mm (out of stroke), and no mocked-ROS test could see it.

    This test drives the REAL tracker → REAL coordinator → the REAL wire packing →
    a REAL TrajectoryNode (seeded at the active hold, TRAJECTORY, latch armed) and
    asserts the catch is ACCEPTED with the reach arriving AT the wire pose."""
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from geometry_msgs.msg import Point, Quaternion, Vector3
    from jugglebot_interfaces.msg import (
        DynamicTargetCommand, MotorStateSingle, RobotState)
    from jugglebot.trajectory_node import TrajectoryNode
    import time

    # A near-centre BB-like throw: lands ~33 mm off-centre on the cup plane with a
    # small receive tilt — the nominal reload geometry.
    tracker = _make_tracker()
    pos0 = [60.0, 0.0, 2000.0]
    vel0 = [-60.0, 0.0, -400.0]
    ball, t_now = _confirm_ball(tracker, pos0, vel0, destination="jugglebot")
    assert ball is not None
    cmd = _make_coordinator().update([ball], current_time=t_now)
    assert cmd is not None

    # Pack the wire message EXACTLY as catch_coordinator_node._on_balls does
    # (verbatim target_pos; w-first quaternion into named fields), with a perf-domain
    # arrival a realistic flight-lead ahead.
    msg = DynamicTargetCommand()
    msg.target_pos = Point(x=float(cmd.target_pos[0]), y=float(cmd.target_pos[1]),
                           z=float(cmd.target_pos[2]))
    msg.target_quat = Quaternion(w=float(cmd.target_quat[0]),
                                 x=float(cmd.target_quat[1]),
                                 y=float(cmd.target_quat[2]),
                                 z=float(cmd.target_quat[3]))
    msg.target_vel = Vector3(x=0.0, y=0.0, z=0.0)
    lead_s = 1.5
    msg.arrival_time = time.perf_counter() + lead_s

    # A real TrajectoryNode at the active hold, TRAJECTORY, catch latch armed.
    node = TrajectoryNode(start_emitter=False)
    rs = RobotState()
    rs.motor_states = [
        MotorStateSingle(pos_estimate=float(r))
        for r in hw.JB_OP_ACTIVATE_POSITION_REVS] + [MotorStateSingle()]
    rs.is_homed = True
    node._on_robot_state(rs)
    node._on_control_mode(String(data='TRAJECTORY'))
    arm = SetBool.Request()
    arm.data = True
    node._svc_arm_catch(arm, SetBool.Response())

    node._on_dynamic_target(msg)
    fb = node.target_feedback_pub.published[-1]
    assert fb.accepted is True, (
        f"coordinator wire output REJECTED by trajectory_node: {fb.code} {fb.reason}")
    plan = node._active_plan
    assert plan.kind == 'move'
    # The reach arrives AT the wire pose — no frame conversion anywhere between the
    # coordinator's output and the commanded catch (z ≈ active + a few mm, in-stroke).
    p_arr = plan.state_at(lead_s)[0]
    assert abs(p_arr[2] - float(cmd.target_pos[2])) < 1e-6
    assert 150.0 < p_arr[2] < 200.0
    assert np.allclose(p_arr[:2], cmd.target_pos[:2], atol=1e-6)
