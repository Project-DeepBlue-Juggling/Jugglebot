"""Scripted test trajectories and catch sequences for MPC validation.

Each trajectory is a list of ``(pose, arrival_time)`` waypoints.  The MPC
plans optimal motion between waypoints internally (variable-resolution
horizon), so no external trajectory generator is needed.

Trajectories
------------
T1 : Linear translation
    active pose -> [0,0,50,0,0,0] -> active pose, 1s each segment.
T2 : Circular orbit
    80 mm radius circle in XY at z=50, 2s period.
T3 : Multi-axis
    Simultaneous translation + tilt, 1.5s segments.
T4 : Speed test
    Fast point-to-point, ~400 ms transit.
T5 : Grand tour — 10 poses spanning the workspace.
T6 : Extreme tour — large sweeping motions.

Waypoint format
---------------
``list[tuple[np.ndarray, float]]`` — each entry is ``(pose_6dof, arrival_time)``.
The MPC targets each waypoint in sequence, advancing to the next once the
current arrival_time is reached.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    Waypoints = list[tuple[np.ndarray, float]]


def _waypoints_from_poses(
    poses: list[np.ndarray],
    seg_durations: list[float],
    start_time: float = 0.5,
) -> tuple[list[tuple[np.ndarray, float]], float]:
    """Build a waypoint list from poses and per-segment durations.

    The first pose is the initial hold (arrival = start_time).  Each
    subsequent pose arrives ``seg_durations[i]`` seconds after the previous.

    Returns (waypoints, total_duration_with_settle).
    """
    assert len(seg_durations) == len(poses) - 1
    waypoints: list[tuple[np.ndarray, float]] = []
    t = start_time
    waypoints.append((poses[0].copy(), t))
    for i, dur in enumerate(seg_durations):
        t += dur
        waypoints.append((poses[i + 1].copy(), t))
    return waypoints, t + 0.5  # 0.5s settle after last waypoint


def make_T1(start_time: float = 0.5) -> tuple[list[tuple[np.ndarray, float]], float]:
    """T1: Linear translation — low -> z+50mm -> low, 1s each.

    Returns (waypoints, total_duration).
    """
    low = np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])
    raised = np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0])
    return _waypoints_from_poses([low, raised, low], [1.0, 1.0], start_time)


def make_T2(start_time: float = 0.5) -> tuple[list[tuple[np.ndarray, float]], float]:
    """T2: Circular orbit — 80mm radius in XY at z=50, 2s period.

    Includes a 1s ramp from active pose to the orbit start position, then 2 full
    periods of circular motion (4s), sampled at the control rate (50 Hz).
    The MPC plans the ramp internally; the orbit phase uses dense waypoints
    so the MPC always has a nearby target to track.

    Returns (waypoints, total_duration).
    """
    radius = 80.0
    z_offset = 50.0
    period = 2.0
    omega = 2.0 * np.pi / period
    n_periods = 2
    orbit_time = period * n_periods
    ramp_duration = 1.0
    dt = 0.02  # control rate sampling for orbit

    orbit_start_pose = np.array([radius, 0.0, z_offset, 0.0, 0.0, 0.0])
    orbit_start_time = start_time + ramp_duration

    waypoints: list[tuple[np.ndarray, float]] = []

    # Phase 1: MPC plans a smooth ramp from active pose to orbit start
    waypoints.append((orbit_start_pose.copy(), orbit_start_time))

    # Phase 2: dense circle waypoints at control rate
    n_orbit_steps = int(orbit_time / dt)
    for i in range(1, n_orbit_steps + 1):
        t_orbit = i * dt
        theta = omega * t_orbit
        pose = np.array([
            radius * np.cos(theta),
            radius * np.sin(theta),
            z_offset, 0.0, 0.0, 0.0,
        ])
        waypoints.append((pose, orbit_start_time + t_orbit))

    # Phase 3: hold at orbit end (= orbit start pose after 2 full periods)
    hold_time = orbit_start_time + orbit_time + 0.5
    waypoints.append((orbit_start_pose.copy(), hold_time))

    total_duration = hold_time + 0.5
    return waypoints, total_duration


def make_T3(start_time: float = 0.5) -> tuple[list[tuple[np.ndarray, float]], float]:
    """T3: Multi-axis — simultaneous translation + tilt, 1.5s segments.

    Path: low -> [30, -20, 60, 3deg, -2deg, 0] -> [-20, 30, 40, -2deg, 3deg, 0] -> low

    Returns (waypoints, total_duration).
    """
    low = np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])
    pose_a = np.array([30.0, -20.0, 60.0, np.radians(3.0), np.radians(-2.0), 0.0])
    pose_b = np.array([-20.0, 30.0, 40.0, np.radians(-2.0), np.radians(3.0), 0.0])
    return _waypoints_from_poses(
        [low, pose_a, pose_b, low], [1.5, 1.5, 1.5], start_time)


def make_T4(start_time: float = 0.5) -> tuple[list[tuple[np.ndarray, float]], float]:
    """T4: Speed test — fast point-to-point, 400ms transit.

    Uses a raised start position so lateral motion is feasible.
    Path: active pose -> [0,0,80,0,0,0] -> [40,-30,60,2deg,-1deg,0], then hold.

    Returns (waypoints, total_duration).
    """
    low = np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])
    pose_start = np.array([0.0, 0.0, 80.0, 0.0, 0.0, 0.0])
    pose_end = np.array([40.0, -30.0, 60.0, np.radians(2.0), np.radians(-1.0), 0.0])
    return _waypoints_from_poses(
        [low, pose_start, pose_end], [1.0, 0.4], start_time)


def make_T5(start_time: float = 0.5) -> tuple[list[tuple[np.ndarray, float]], float]:
    """T5: Grand tour — 10 poses spanning the workspace, 1.2s per segment.

    Exercises all 6 DoFs with a mix of high/low Z, lateral excursions,
    combined tilts, and yaw. Returns to active pose at the end.

    Returns (waypoints, total_duration).
    """
    d = np.radians

    low = np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])
    poses = [
        low,                                                                # low (just above STOW)
        np.array([  0.0,   0.0, 100.0,      0.0,      0.0,      0.0]),     # straight up
        np.array([ 60.0,  40.0,  80.0,   d(4.0),   d(-3.0),     0.0]),     # front-right, tilted
        np.array([-50.0,  50.0,  60.0,  d(-3.0),    d(4.0),  d(2.0)]),     # back-left, yaw
        np.array([  0.0, -70.0, 110.0,   d(5.0),      0.0,  d(-3.0)]),     # far side, high
        np.array([ 70.0,   0.0,  50.0,      0.0,   d(-5.0),     0.0]),     # right, big roll
        np.array([-60.0, -40.0,  90.0,  d(-4.0),    d(3.0),  d(3.0)]),     # back-left-high, yaw
        np.array([ 30.0,  60.0,  40.0,   d(3.0),   d(-2.0), d(-2.0)]),     # front-left, low
        np.array([-40.0,   0.0, 120.0,  d(-2.0),   d(-4.0),     0.0]),     # left, highest
        np.array([ 50.0, -50.0,  70.0,   d(4.0),    d(4.0),  d(1.0)]),     # front-right-low, dual tilt
        np.array([  0.0,   0.0, 100.0,      0.0,      0.0,      0.0]),     # centre high (smooth exit)
        low,                                                                # low
    ]
    seg_duration = 1.2
    return _waypoints_from_poses(
        poses, [seg_duration] * (len(poses) - 1), start_time)


def make_T6(start_time: float = 0.5) -> tuple[list[tuple[np.ndarray, float]], float]:
    """T6: Extreme tour — big sweeping XY motions across the full workspace.

    Pushes lateral reach to ~120 mm while combining height changes, tilts,
    and yaw.  The platform covers huge distances between poses, crossing
    through the centre and visiting all four XY quadrants.

    Returns (waypoints, total_duration).
    """
    d = np.radians

    low = np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])
    poses = [
        low,                                                                  # low (just above STOW)
        np.array([  0.0,   0.0, 120.0,       0.0,      0.0,      0.0]),      # rise to cruise height
        np.array([120.0,   0.0, 100.0,       0.0,   d(-5.0),     0.0]),      # far right, leaning
        np.array([-100.0, 100.0, 100.0,   d(-4.0),    d(4.0), d(-3.0)]),     # opposite corner
        np.array([  0.0,-120.0, 130.0,    d(5.0),      0.0,   d(4.0)]),      # far back
        np.array([100.0, 100.0, 120.0,    d(3.0),   d(-3.0),     0.0]),      # front-right diagonal
        np.array([-120.0,  0.0,  80.0,      0.0,    d(5.0),  d(-4.0)]),      # far left, low
        np.array([ 80.0, -80.0,  60.0,   d(-5.0),   d(-5.0),  d(3.0)]),     # front-right-low, big tilt
        np.array([-90.0, -90.0, 100.0,    d(4.0),    d(4.0), d(-3.0)]),      # back-left
        np.array([  0.0, 120.0, 140.0,   d(-4.0),      0.0,   d(5.0)]),      # far front, high
        np.array([100.0, -80.0, 140.0,    d(3.0),   d(-3.0),  d(2.0)]),      # opposite again
        np.array([  0.0,   0.0, 260.0,    d(5.0),    d(5.0),     0.0]),      # sky high + tilt
        np.array([  0.0,   0.0, 120.0,       0.0,      0.0,      0.0]),      # back to cruise
        low,                                                                  # low
    ]
    durations = [1.0] + [1.2] * (len(poses) - 3) + [1.0]
    return _waypoints_from_poses(poses, durations, start_time)


# Registry for easy lookup
TRAJECTORIES = {
    'T1': make_T1,
    'T2': make_T2,
    'T3': make_T3,
    'T4': make_T4,
    'T5': make_T5,
    'T6': make_T6,
}


def get_trajectory(
    name: str, **kwargs,
) -> tuple[list[tuple[np.ndarray, float]], float]:
    """Look up a scripted trajectory by name.

    Parameters
    ----------
    name : str
        Trajectory name (T1, T2, T3, T4, T5, T6).

    Returns
    -------
    waypoints : list of (pose_6dof, arrival_time)
    duration : float
        Recommended simulation duration.
    """
    if name not in TRAJECTORIES:
        raise ValueError(
            f"Unknown trajectory '{name}'. Available: {list(TRAJECTORIES.keys())}"
        )
    return TRAJECTORIES[name](**kwargs)


# ---------------------------------------------------------------------------
# Dynamic target (catch) test sequences — Phase 5C
# ---------------------------------------------------------------------------

from sim.hand.coordinator import DynamicTarget, BallSpawn


def _ball_landing(
    spawn_pos_mm: np.ndarray,
    spawn_vel_mms: np.ndarray,
    target_z_mm: float,
) -> tuple[float, np.ndarray, np.ndarray]:
    """Predict ball's position and velocity at a given Z height.

    Returns (flight_time_s, landing_pos_mm, landing_vel_mms).
    Uses basic ballistic equations (no drag).
    """
    g = 9806.0  # mm/s²
    z0 = spawn_pos_mm[2]
    vz0 = spawn_vel_mms[2]

    # Solve z0 + vz0*t - 0.5*g*t^2 = target_z for t
    # -0.5*g*t^2 + vz0*t + (z0 - target_z) = 0
    a = -0.5 * g
    b = vz0
    c = z0 - target_z_mm

    disc = b**2 - 4 * a * c
    if disc < 0:
        raise ValueError("Ball never reaches target Z")

    # Take the positive root where ball is DESCENDING (vz < 0).
    # For upward throws that cross target_z twice, this correctly
    # selects the downward crossing rather than the upward one.
    t1 = (-b + np.sqrt(disc)) / (2 * a)
    t2 = (-b - np.sqrt(disc)) / (2 * a)
    candidates = [t for t in [t1, t2]
                  if t > 0 and (vz0 - g * t) < 0]
    if not candidates:
        raise ValueError("Ball never descends through target Z")
    t = min(candidates)

    landing_pos = spawn_pos_mm.copy()
    landing_pos[0] += spawn_vel_mms[0] * t
    landing_pos[1] += spawn_vel_mms[1] * t
    landing_pos[2] = target_z_mm

    landing_vel = spawn_vel_mms.copy()
    landing_vel[2] -= g * t

    return float(t), landing_pos, landing_vel


# Catch angle limit (matches production catch_coordinator.py)
_CATCH_ANGLE_LIMIT_RAD = np.radians(30.0)

# Hand catch offset: distance from platform centroid to ball COM along the
# platform's local Z axis (mm) at the moment of catch.
# x5 is in effective-stroke coords; add STROKE_MARGIN to get physical pos.
# Ball COM offset from hand body = BALL_SEAT_OFFSET (44.4mm).
from sim.hand.trajectory import (
    INERTIA_RATIO as _IR,
    CATCH_VEL_HOLD_PCT as _CVH,
    _TOTAL_STROKE_M as _TS,
    STROKE_MARGIN_M as _SM,
)
from sim.hand.ballistics import compute_hand_offset_mm as _compute_hand_offset_mm
_HAND_AXIS_BOTTOM_OFFSET_MM = -129.0   # from hardware_config.yaml
_x5_m = _TS - (_TS - _CVH * _TS) * _IR / (1.0 + _IR)
_HAND_CATCH_X5_PHYSICAL_MM = _SM * 1000.0 + _x5_m * 1000.0
_HAND_CATCH_OFFSET_MM = _compute_hand_offset_mm(_HAND_CATCH_X5_PHYSICAL_MM)

_PLATFORM_HEIGHT_MM = 574.3


def _rodrigues(rv: np.ndarray) -> np.ndarray:
    """Rotation vector → 3×3 rotation matrix via Rodrigues' formula (numpy).

    Same formula used in controller/mpc.py ``_numerical_ik``.  Avoids a
    scipy dependency for a single rotation.
    """
    angle = np.linalg.norm(rv)
    if angle < 1e-10:
        return np.eye(3)
    K = np.array([
        [0, -rv[2], rv[1]],
        [rv[2], 0, -rv[0]],
        [-rv[1], rv[0], 0],
    ])
    return np.eye(3) + (np.sin(angle) / angle) * K + ((1 - np.cos(angle)) / (angle * angle)) * K @ K


def _compute_catch_orientation(landing_vel_mms: np.ndarray) -> np.ndarray | None:
    """Compute rotation vector to tilt platform normal toward ball velocity.

    The platform's Z-down axis is aligned toward the ball's approach
    direction so the hand/cone catches the ball head-on.

    Parameters
    ----------
    landing_vel_mms : (3,) array
        Ball velocity at landing height (mm/s).

    Returns
    -------
    rv : (3,) rotation vector, or None if angle exceeds limit.

    Mirrors production ``catch_coordinator.compute_catch_orientation()``.
    Since the simulation uses rotation vectors natively, we compute
    axis * angle directly without going through quaternions.
    """
    v = landing_vel_mms
    speed = np.linalg.norm(v)
    if speed < 1e-6:
        return np.zeros(3)

    v_hat = v / speed
    reference = np.array([0.0, 0.0, -1.0])  # platform Z-down
    dot = np.clip(np.dot(reference, v_hat), -1.0, 1.0)
    angle = np.arccos(dot)

    if angle > _CATCH_ANGLE_LIMIT_RAD:
        return None

    if angle < 1e-9:
        return np.zeros(3)

    axis = np.cross(reference, v_hat)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-9:
        # Anti-parallel: ball coming straight up (unlikely but handle it)
        return np.array([np.pi, 0.0, 0.0])

    axis /= axis_norm
    return axis * angle


def _compute_catch_target(
    spawn_pos_mm: np.ndarray,
    spawn_vel_mms: np.ndarray,
    spawn_time: float,
    platform_height_mm: float = 574.3,
    hand_catch_offset_mm: float = 64.78,
    active_z_offset_mm: float = 0.0,
) -> tuple[DynamicTarget, BallSpawn]:
    """Compute a DynamicTarget from a ball trajectory.

    Computes the platform pose that positions the hand opening at the
    ball's predicted landing point, with the platform tilted to align
    the catching axis with the ball's approach direction.

    The catch Z is the hand opening position in world frame when the
    platform is at ``active_z_offset_mm`` above the active pose.

    Parameters
    ----------
    hand_catch_offset_mm : float
        Distance from platform centroid to ball catch point along the
        platform's local Z axis.  Production value: 64.78 mm, computed as
        ``hand_axis_bottom_offset_mm + x5_m * 1000`` where x5 is the hand
        slider position at ball contact (from Trajectory.h 3-segment timing).
    """

    # First pass: predict landing at the untilted catch height to get
    # an initial estimate of the landing velocity (needed for tilt calc).
    catch_z_world = platform_height_mm + active_z_offset_mm + hand_catch_offset_mm

    flight_time, landing_pos, landing_vel = _ball_landing(
        spawn_pos_mm, spawn_vel_mms, catch_z_world)

    # Compute catch orientation from landing velocity
    rv = _compute_catch_orientation(landing_vel)
    if rv is None:
        raise ValueError(
            f"Approach angle exceeds {np.degrees(_CATCH_ANGLE_LIMIT_RAD):.0f} "
            f"degree limit")

    # Hand offset correction: when the platform is tilted, the hand opening
    # is offset along the *tilted* Z axis, not the world Z axis.
    # Platform centroid = ball landing position - hand_catch_offset * tilted_Z
    #
    # For small tilts (< 30 deg), the Z component dominates and the XY
    # correction is small but physically important for lateral catches.
    R = _rodrigues(rv)
    platform_z = R @ np.array([0.0, 0.0, 1.0])

    # Platform centroid in world frame
    centroid_world = landing_pos - hand_catch_offset_mm * platform_z

    # Convert to platform offset coordinates (relative to active pose)
    target_z_offset = centroid_world[2] - platform_height_mm

    arrival_time = spawn_time + flight_time

    target_pose = np.array([
        centroid_world[0],
        centroid_world[1],
        target_z_offset,
        rv[0], rv[1], rv[2],
    ])

    # Compute event_vel from ball landing speed (mm/s → m/s, clamped)
    landing_speed_mps = float(np.linalg.norm(landing_vel)) / 1000.0
    event_vel_mps = max(0.3, min(7.0, landing_speed_mps))

    target = DynamicTarget(
        pose_6dof=target_pose,
        arrival_time=arrival_time,
        arrival_twist=None,  # catch mode
        hold_duration=0.5,
        event_vel_mps=event_vel_mps,
    )

    ball = BallSpawn(
        position_mm=spawn_pos_mm.copy(),
        velocity_mms=spawn_vel_mms.copy(),
        spawn_time=spawn_time,
    )

    return target, ball


def _ball_landing_speed_mps(
    spawn_pos_mm: np.ndarray,
    spawn_vel_mms: np.ndarray,
    target_z_mm: float,
) -> float:
    """Compute ball speed at landing height, clamped to Teensy event_vel range."""
    _, _, landing_vel = _ball_landing(spawn_pos_mm, spawn_vel_mms, target_z_mm)
    speed_mps = float(np.linalg.norm(landing_vel)) / 1000.0
    return max(0.3, min(7.0, speed_mps))


# ---------------------------------------------------------------------------
# Ballistic catch tests (DT1-DT5, DT7): computed from _compute_catch_target()
#
# These tests derive the platform target pose from ball ballistics, exercising
# the full catch pipeline: ballistic prediction → tilt computation → hand
# offset correction → DynamicTarget.  The spawn parameters define the test
# scenario; everything else (pose, arrival time, event_vel) is computed.
#
# Spawn height is 3000 mm with zero initial velocity (pure gravity drop).
# This gives ~0.68 s of flight time to the catch height, plus spawn_time
# delay, yielding ~1.0 s total arrival time — enough for the hand catch
# trajectory prelude (~0.85 s lead required).
# ---------------------------------------------------------------------------

# Default platform Z offset for catch height computation (mm above active pose).
# The catch intercept height is platform_height + active_z + hand_catch_offset.
_DT_ACTIVE_Z_MM = 80.0


def make_DT1() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """DT1: Single catch — ball with lateral offset and horizontal velocity.

    The horizontal velocity exercises the tilt computation: the platform
    tilts to align the hand axis with the ball's approach direction.

    Returns ([(target, ball_spawn)], total_duration).
    """
    target, ball = _compute_catch_target(
        np.array([30.0, -20.0, 3000.0]),
        np.array([-50.0, 30.0, 0.0]),   # horizontal drift → small tilt
        spawn_time=0.3,
        active_z_offset_mm=_DT_ACTIVE_Z_MM,
    )
    return [(target, ball)], 3.0


def make_DT2() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """DT2: Rapid succession — two catches at different XY positions.

    Ball 2 spawns after ball 1's catch-hold-return cycle completes (~2.3 s).
    """
    target1, ball1 = _compute_catch_target(
        np.array([20.0, 0.0, 3000.0]),
        np.array([0.0, 0.0, 0.0]),
        spawn_time=0.2,
        active_z_offset_mm=_DT_ACTIVE_Z_MM,
    )
    target1.hold_duration = 0.3

    target2, ball2 = _compute_catch_target(
        np.array([-30.0, 20.0, 3000.0]),
        np.array([0.0, 0.0, 0.0]),
        spawn_time=2.5,
        active_z_offset_mm=_DT_ACTIVE_Z_MM,
    )
    target2.hold_duration = 0.3

    return [(target1, ball1), (target2, ball2)], 6.0


def make_DT3() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """DT3: Edge of workspace — ball dropped near the reachable boundary.

    No horizontal velocity so the ball lands directly at [80, -60], which
    is near the ~120 mm lateral reach limit.
    """
    target, ball = _compute_catch_target(
        np.array([80.0, -60.0, 3000.0]),
        np.array([0.0, 0.0, 0.0]),
        spawn_time=0.3,
        active_z_offset_mm=_DT_ACTIVE_Z_MM,
    )
    return [(target, ball)], 3.5


# ---------------------------------------------------------------------------
# Hardcoded-pose tests (DT4, DT6, DT8): NOT computed from ballistics
#
# These tests have deliberately decoupled target poses:
#   DT4 — Infeasible: hardcoded extreme pose + tight deadline tests MPC
#         graceful degradation.  Using _compute_catch_target() would produce
#         a different (possibly feasible) target, defeating the test.
#   DT6 — Throw: no ball involved; tests nonzero arrival velocity + deceleration.
#   DT8 — Deliberate miss: ball spawned far off-axis but target is centred.
#         The test verifies the system handles a ball that doesn't land near
#         the target.  Using ballistics would move the target to the ball.
# ---------------------------------------------------------------------------


def make_DT4() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """DT4: Infeasible — target requiring impossibly fast motion.

    Platform can't reach [120, 120, 200] with 5° combined tilt in 200 ms from active pose.
    The MPC should do its best but the ball should miss.
    """
    spawn_pos = np.array([120.0, 120.0, 1500.0])
    spawn_vel = np.array([0.0, 0.0, -2000.0])
    catch_z = _PLATFORM_HEIGHT_MM + _DT_ACTIVE_Z_MM + _HAND_CATCH_OFFSET_MM

    target = DynamicTarget(
        pose_6dof=np.array([120.0, 120.0, 200.0,
                            np.radians(5.0), np.radians(5.0), 0.0]),
        arrival_time=0.3,  # way too soon
        arrival_twist=None,
        hold_duration=0.5,
        event_vel_mps=_ball_landing_speed_mps(spawn_pos, spawn_vel, catch_z),
    )
    ball = BallSpawn(position_mm=spawn_pos, velocity_mms=spawn_vel, spawn_time=0.05)

    return [(target, ball)], 3.0


def make_DT5() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """DT5: Early arrival — ball spawned late so the platform has ample lead time.

    The platform should reach the target well before the ball arrives and hold.
    """
    target, ball = _compute_catch_target(
        np.array([0.0, 0.0, 3000.0]),
        np.array([0.0, 0.0, 0.0]),
        spawn_time=1.5,
        active_z_offset_mm=_DT_ACTIVE_Z_MM,
    )
    return [(target, ball)], 5.0


def make_DT6() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """DT6: Throw — arrive with nonzero velocity, then decelerate.

    Platform moves downward at 200 mm/s at arrival. Ball released with that velocity.
    No ball is spawned — this tests the throw/deceleration path only.
    """
    target = DynamicTarget(
        pose_6dof=np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0]),
        arrival_time=1.0,
        arrival_twist=np.array([0.0, 0.0, -200.0, 0.0, 0.0, 0.0]),
        hold_duration=0.0,
    )

    return [(target, None)], 3.0


def make_DT7() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """DT7: Catch then throw — catch a ball, hold, then throw downward.

    The catch phase uses ballistic computation.  The throw phase is hardcoded
    (no ball to track — tests nonzero arrival velocity + deceleration).
    """
    catch_target, catch_ball = _compute_catch_target(
        np.array([0.0, 0.0, 3000.0]),
        np.array([0.0, 0.0, 0.0]),
        spawn_time=0.3,
        active_z_offset_mm=_DT_ACTIVE_Z_MM,
    )
    catch_target.hold_duration = 0.3

    # Throw phase: hardcoded (no ball to track — tests velocity-at-arrival)
    throw_target = DynamicTarget(
        pose_6dof=np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0]),
        arrival_time=3.0,
        arrival_twist=np.array([0.0, 0.0, -150.0, 0.0, 0.0, 0.0]),
        hold_duration=0.0,
    )

    return [(catch_target, catch_ball), (throw_target, None)], 5.0


# ---------------------------------------------------------------------------
# Throw-catch sequences (TC1-TC4): use ThrowCatchPlanner
# ---------------------------------------------------------------------------

def make_TC1():
    """TC1: Vertical throw-catch — throw ball up, catch it as it comes down.

    Throw from [0, 0, 800] upward, ball arcs and lands at [0, 0, 800].
    No tilt (vertical throw), ~1s flight time.

    Returns (ThrowCatchPlan, total_duration).
    """
    from sim.hand.planner import ThrowCatchPlanner
    planner = ThrowCatchPlanner()
    plan = planner.plan(
        throw_pos_mm=np.array([0.0, 0.0, 800.0]),
        catch_pos_mm=np.array([0.0, 0.0, 800.0]),
        throw_time=1.5,
        catch_time=2.5,
    )
    return plan, 5.0


def make_TC2():
    """TC2: Angled throw-catch — 400mm lateral traverse.

    Throw and catch at platform_height + 255mm (829mm).  Flight time
    1.15s gives a high arc and gentle tilts (~3.5° vs original ~11°),
    leaving ample stroke headroom for the lateral traverse.

    Returns (ThrowCatchPlan, total_duration).
    """
    from sim.hand.planner import ThrowCatchPlanner
    planner = ThrowCatchPlanner()
    ball_z = 574.3 + 255.0   # halfway between original 170mm and 340mm offsets
    flight_time = 1.15        # high arc, gentle tilts, plenty of transit time
    plan = planner.plan(
        throw_pos_mm=np.array([-200.0, 0.0, ball_z]),
        catch_pos_mm=np.array([200.0, 0.0, ball_z]),
        throw_time=2.0,
        catch_time=2.0 + flight_time,
    )
    return plan, 7.0


def make_TC3():
    """TC3: Fast throw-catch — short flight time, tests timing precision.

    Returns (ThrowCatchPlan, total_duration).
    """
    from sim.hand.planner import ThrowCatchPlanner
    planner = ThrowCatchPlanner()
    plan = planner.plan(
        throw_pos_mm=np.array([0.0, 0.0, 800.0]),
        catch_pos_mm=np.array([0.0, 0.0, 800.0]),
        throw_time=1.5,
        catch_time=2.1,
    )
    return plan, 4.0


def make_TC4():
    """TC4: Longer arc — 1.4s flight time (6.86 m/s, near max).

    Returns (ThrowCatchPlan, total_duration).
    """
    from sim.hand.planner import ThrowCatchPlanner
    planner = ThrowCatchPlanner()
    plan = planner.plan(
        throw_pos_mm=np.array([0.0, 0.0, 800.0]),
        catch_pos_mm=np.array([0.0, 0.0, 800.0]),
        throw_time=1.5,
        catch_time=2.9,
    )
    return plan, 5.0


THROW_CATCH_SEQUENCES = {
    'TC1': make_TC1,
    'TC2': make_TC2,
    'TC3': make_TC3,
    'TC4': make_TC4,
}


def get_throw_catch_sequence(name: str):
    """Look up a scripted throw-catch sequence by name.

    Returns (ThrowCatchPlan, duration).
    """
    if name not in THROW_CATCH_SEQUENCES:
        raise ValueError(
            f"Unknown throw-catch sequence '{name}'. "
            f"Available: {list(THROW_CATCH_SEQUENCES.keys())}"
        )
    return THROW_CATCH_SEQUENCES[name]()


def make_DT8() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """DT8: Ball miss — ball spawned far from reachable platform workspace.

    The ball is spawned at x=300mm (well outside the ~120mm reachable radius).
    The target is centred at the origin — deliberately NOT computed from
    ballistics so the ball and target are misaligned.  The system should
    attempt the catch, fail, and return to active pose gracefully.
    """
    spawn_pos = np.array([300.0, 0.0, 1500.0])
    spawn_vel = np.array([0.0, 0.0, -800.0])
    catch_z = _PLATFORM_HEIGHT_MM + _DT_ACTIVE_Z_MM + _HAND_CATCH_OFFSET_MM

    target = DynamicTarget(
        pose_6dof=np.array([0.0, 0.0, 80.0, 0.0, 0.0, 0.0]),
        arrival_time=0.8,
        arrival_twist=None,
        hold_duration=0.5,
        event_vel_mps=_ball_landing_speed_mps(spawn_pos, spawn_vel, catch_z),
    )
    ball = BallSpawn(position_mm=spawn_pos, velocity_mms=spawn_vel, spawn_time=0.1)

    return [(target, ball)], 3.0


# ---------------------------------------------------------------------------
# Ball Butler throw sequences (BB1-BB4)
#
# These use BallButlerSim to compute realistic ball trajectories originating
# from BB's physical location, then feed through the same _compute_catch_target
# pipeline as DT1-DT8.
# ---------------------------------------------------------------------------

# Default BB placement (approximate — will be updated with real measurements)
_BB_DEFAULT_POS_MM = np.array([300.0, -400.0, 1500.0])
_BB_DEFAULT_YAW_RAD = float(np.arctan2(
    0.0 - _BB_DEFAULT_POS_MM[1],
    0.0 - _BB_DEFAULT_POS_MM[0],
))


def _make_bb_sim():
    from sim.ball_butler.sim import BallButlerSim
    return BallButlerSim.from_hardware_config(
        _BB_DEFAULT_POS_MM, _BB_DEFAULT_YAW_RAD)


def make_BB1() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """BB1: Single throw from Ball Butler at Jugglebot centre.

    Ball arcs from ~1.5m height to catch plane.  Basic throw-catch validation.

    Returns ([(target, ball_spawn)], total_duration).
    """
    bb = _make_bb_sim()
    spawn = bb.throw_at_jugglebot(spawn_time=0.3)
    target, ball = _compute_catch_target(
        spawn.position_mm, spawn.velocity_mms, spawn.spawn_time,
        active_z_offset_mm=_DT_ACTIVE_Z_MM,
    )
    return [(target, ball)], 4.0


def make_BB2() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """BB2: Two throws 3s apart at different XY offsets.

    Tests succession with realistic inter-throw timing.
    """
    bb = _make_bb_sim()

    spawn1 = bb.throw_at_jugglebot(spawn_time=0.3,
                                   landing_xy_mm=np.array([20.0, 0.0]))
    target1, ball1 = _compute_catch_target(
        spawn1.position_mm, spawn1.velocity_mms, spawn1.spawn_time,
        active_z_offset_mm=_DT_ACTIVE_Z_MM,
    )
    target1.hold_duration = 0.3

    spawn2 = bb.throw_at_jugglebot(spawn_time=3.3,
                                   landing_xy_mm=np.array([-30.0, 20.0]))
    target2, ball2 = _compute_catch_target(
        spawn2.position_mm, spawn2.velocity_mms, spawn2.spawn_time,
        active_z_offset_mm=_DT_ACTIVE_Z_MM,
    )
    target2.hold_duration = 0.3

    return [(target1, ball1), (target2, ball2)], 7.0


def make_BB3() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """BB3: Throw near edge of catch envelope — 80mm lateral offset.

    Exercises the platform's lateral reach and approach-angle tilt.
    """
    bb = _make_bb_sim()
    spawn = bb.throw_at_jugglebot(spawn_time=0.3,
                                  landing_xy_mm=np.array([80.0, -50.0]))
    target, ball = _compute_catch_target(
        spawn.position_mm, spawn.velocity_mms, spawn.spawn_time,
        active_z_offset_mm=_DT_ACTIVE_Z_MM,
    )
    return [(target, ball)], 4.0


def make_BB4() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """BB4: Rapid succession — two throws 1.5s apart.

    Tests throughput: second ball arrives while platform is still returning
    from the first catch.
    """
    bb = _make_bb_sim()

    spawn1 = bb.throw_at_jugglebot(spawn_time=0.3)
    target1, ball1 = _compute_catch_target(
        spawn1.position_mm, spawn1.velocity_mms, spawn1.spawn_time,
        active_z_offset_mm=_DT_ACTIVE_Z_MM,
    )
    target1.hold_duration = 0.2

    spawn2 = bb.throw_at_jugglebot(spawn_time=1.8,
                                   landing_xy_mm=np.array([20.0, -10.0]))
    target2, ball2 = _compute_catch_target(
        spawn2.position_mm, spawn2.velocity_mms, spawn2.spawn_time,
        active_z_offset_mm=_DT_ACTIVE_Z_MM,
    )
    target2.hold_duration = 0.2

    return [(target1, ball1), (target2, ball2)], 5.0


# Dynamic target registry
CATCH_SEQUENCES = {
    'DT1': make_DT1,
    'DT2': make_DT2,
    'DT3': make_DT3,
    'DT4': make_DT4,
    'DT5': make_DT5,
    'DT6': make_DT6,
    'DT7': make_DT7,
    'DT8': make_DT8,
    'BB1': make_BB1,
    'BB2': make_BB2,
    'BB3': make_BB3,
    'BB4': make_BB4,
}


def get_catch_sequence(
    name: str,
) -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """Look up a scripted catch sequence by name.

    Returns
    -------
    sequence : list of (DynamicTarget, BallSpawn | None)
    duration : float
        Recommended simulation duration.
    """
    if name not in CATCH_SEQUENCES:
        raise ValueError(
            f"Unknown catch sequence '{name}'. "
            f"Available: {list(CATCH_SEQUENCES.keys())}"
        )
    return CATCH_SEQUENCES[name]()
