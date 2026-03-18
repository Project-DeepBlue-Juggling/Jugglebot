"""Scripted test trajectories for MPC trajectory tracking validation.

Each trajectory is a ``ReferenceGenerator`` that can be passed directly to the
MPC simulation loop.

Trajectories
------------
T1 : Linear translation
    home -> [0,0,50,0,0,0] -> home, 1s each segment.
T2 : Circular orbit
    80 mm radius circle in XY at z=50, 2s period.
T3 : Multi-axis
    Simultaneous translation + tilt, 1.5s segments.
T4 : Speed test
    Fast point-to-point, ~300 ms transit.
"""

from __future__ import annotations

import numpy as np

import os, sys
_sim_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)

from controller.reference import ReferenceGenerator


def make_T1(start_time: float = 0.5) -> tuple[ReferenceGenerator, float]:
    """T1: Linear translation — home -> z+50mm -> home, 1s each.

    Returns (reference_generator, total_duration).
    """
    home = np.zeros(6)
    raised = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])

    ref = ReferenceGenerator.from_waypoints(
        waypoints=[home, raised, home],
        durations=[1.0, 1.0],
        start_time=start_time,
    )
    return ref, start_time + 2.0 + 0.5  # extra settle time


def make_T2(start_time: float = 0.5) -> tuple[ReferenceGenerator, float]:
    """T2: Circular orbit — 80mm radius in XY at z=50, 2s period.

    Includes a 1s quintic ramp-up from home to the orbit start position
    [radius, 0, z_offset] before circular motion begins.

    Returns (reference_generator, total_duration).
    The trajectory runs for 2 full periods (4s) after ramp-up.
    """
    radius = 80.0
    z_offset = 50.0
    period = 2.0
    omega = 2.0 * np.pi / period
    n_periods = 2
    orbit_time = period * n_periods
    ramp_duration = 1.0  # smooth ramp from home to orbit start

    from jugglebot.motion.quintic import create_trajectory, evaluate as quintic_evaluate

    # Quintic ramp: home → [radius, 0, z_offset, 0, 0, 0] with zero twist at
    # start and matching circular velocity + centripetal acceleration at end
    orbit_start_pose = np.array([radius, 0.0, z_offset, 0.0, 0.0, 0.0])
    # Velocity at orbit start: vx=0, vy=radius*omega (tangent to circle at theta=0)
    orbit_start_twist = np.array([0.0, radius * omega, 0.0, 0.0, 0.0, 0.0])
    # Centripetal acceleration at theta=0: ax=-r*omega^2, ay=0 (points inward)
    orbit_start_accel = np.array([-radius * omega ** 2, 0.0, 0.0, 0.0, 0.0, 0.0])

    ramp_traj = create_trajectory(
        start_pose=np.zeros(6),
        start_twist=np.zeros(6),
        start_accel=np.zeros(6),
        end_pose=orbit_start_pose,
        end_twist=orbit_start_twist,
        end_accel=orbit_start_accel,
        duration=ramp_duration,
        t_start=start_time,
    )

    orbit_start = start_time + ramp_duration

    def _eval(t):
        if t < start_time:
            return np.zeros(6), np.zeros(6)

        if t < orbit_start:
            # Ramp-up phase
            pose, twist, _ = quintic_evaluate(ramp_traj, t)
            return pose, twist

        t_orbit = t - orbit_start
        if t_orbit > orbit_time:
            # After orbit: hold at orbit start with zero twist.
            # NOTE: velocity discontinuity here — orbit ends with vy=r*omega
            # (~251 mm/s) but hold reference has zero twist. MPC handles
            # deceleration naturally via its prediction horizon.
            return orbit_start_pose.copy(), np.zeros(6)

        theta = omega * t_orbit
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        vx = -radius * omega * np.sin(theta)
        vy = radius * omega * np.cos(theta)

        pose = np.array([x, y, z_offset, 0.0, 0.0, 0.0])
        twist = np.array([vx, vy, 0.0, 0.0, 0.0, 0.0])
        return pose, twist

    total_duration = start_time + ramp_duration + orbit_time + 0.5
    ref = ReferenceGenerator.from_function(_eval, duration=total_duration)
    return ref, total_duration


def make_T3(start_time: float = 0.5) -> tuple[ReferenceGenerator, float]:
    """T3: Multi-axis — simultaneous translation + tilt, 1.5s segments.

    Path: home -> [30, -20, 60, 3deg, -2deg, 0] -> [-20, 30, 40, -2deg, 3deg, 0] -> home

    Returns (reference_generator, total_duration).
    """
    home = np.zeros(6)
    pose_a = np.array([30.0, -20.0, 60.0, np.radians(3.0), np.radians(-2.0), 0.0])
    pose_b = np.array([-20.0, 30.0, 40.0, np.radians(-2.0), np.radians(3.0), 0.0])

    ref = ReferenceGenerator.from_waypoints(
        waypoints=[home, pose_a, pose_b, home],
        durations=[1.5, 1.5, 1.5],
        start_time=start_time,
    )
    return ref, start_time + 4.5 + 0.5


def make_T4(start_time: float = 0.5) -> tuple[ReferenceGenerator, float]:
    """T4: Speed test — fast point-to-point, 400ms transit.

    Uses a raised start position so lateral motion is feasible.

    Path: [0,0,80,0,0,0] -> [40,-30,60,2deg,-1deg,0] in 400ms, then hold.

    Returns (reference_generator, total_duration).
    """
    pose_start = np.array([0.0, 0.0, 80.0, 0.0, 0.0, 0.0])
    pose_end = np.array([40.0, -30.0, 60.0, np.radians(2.0), np.radians(-1.0), 0.0])

    # First ramp up to starting position, then do the fast move
    ref = ReferenceGenerator.from_waypoints(
        waypoints=[np.zeros(6), pose_start, pose_end],
        durations=[1.0, 0.4],
        start_time=start_time,
    )
    return ref, start_time + 1.4 + 0.5


def make_T5(start_time: float = 0.5) -> tuple[ReferenceGenerator, float]:
    """T5: Grand tour — 10 poses spanning the workspace, 1.2s per segment.

    Exercises all 6 DoFs with a mix of high/low Z, lateral excursions,
    combined tilts, and yaw. Returns to home at the end.

    Returns (reference_generator, total_duration).
    """
    d = np.radians  # shorthand

    waypoints = [
        np.zeros(6),                                            # home
        np.array([  0.0,   0.0, 100.0,       0.0,      0.0,      0.0]),  # straight up
        np.array([ 60.0,  40.0,  80.0,    d(4.0),   d(-3.0),     0.0]),  # front-right, tilted
        np.array([-50.0,  50.0,  60.0,   d(-3.0),    d(4.0),  d(2.0)]),  # back-left, yaw
        np.array([  0.0, -70.0, 110.0,    d(5.0),      0.0,  d(-3.0)]), # far side, high
        np.array([ 70.0,   0.0,  50.0,      0.0,   d(-5.0),     0.0]),  # right, big roll
        np.array([-60.0, -40.0,  90.0,   d(-4.0),    d(3.0),  d(3.0)]),  # back-left-high, yaw
        np.array([ 30.0,  60.0,  40.0,    d(3.0),   d(-2.0), d(-2.0)]), # front-left, low
        np.array([-40.0,  -0.0, 120.0,   d(-2.0),   d(-4.0),     0.0]), # left, highest
        np.array([ 50.0, -50.0,  70.0,    d(4.0),    d(4.0),  d(1.0)]),  # front-right-low, dual tilt
        np.array([  0.0,   0.0, 100.0,      0.0,      0.0,      0.0]),  # centre high (smooth exit)
        np.zeros(6),                                            # home
    ]

    seg_duration = 1.2
    durations = [seg_duration] * (len(waypoints) - 1)

    ref = ReferenceGenerator.from_waypoints(
        waypoints=waypoints,
        durations=durations,
        start_time=start_time,
    )
    total = start_time + sum(durations) + 0.5
    return ref, total


def make_T6(start_time: float = 0.5) -> tuple[ReferenceGenerator, float]:
    """T6: Extreme tour — big sweeping XY motions across the full workspace.

    Pushes lateral reach to ~120 mm while combining height changes, tilts,
    and yaw.  The platform covers huge distances between poses, crossing
    through the centre and visiting all four XY quadrants.

    Returns (reference_generator, total_duration).
    """
    d = np.radians

    waypoints = [
        np.zeros(6),                                                          # home
        np.array([  0.0,   0.0, 120.0,       0.0,      0.0,      0.0]),      # rise to cruise height
        np.array([120.0,   0.0, 100.0,       0.0,   d(-5.0),     0.0]),      # far right, leaning
        np.array([-100.0, 100.0, 100.0,   d(-4.0),    d(4.0), d(-3.0)]),     # opposite corner (220mm diagonal!)
        np.array([  0.0,-120.0, 130.0,    d(5.0),      0.0,   d(4.0)]),      # far back
        np.array([100.0, 100.0, 120.0,    d(3.0),   d(-3.0),     0.0]),      # front-right diagonal
        np.array([-120.0,  0.0,  80.0,      0.0,    d(5.0),  d(-4.0)]),      # far left, low
        np.array([ 80.0, -80.0,  60.0,   d(-5.0),   d(-5.0),  d(3.0)]),     # front-right-low, big tilt
        np.array([-90.0, -90.0, 100.0,    d(4.0),    d(4.0), d(-3.0)]),      # back-left (170mm diagonal cross!)
        np.array([  0.0, 120.0, 140.0,   d(-4.0),      0.0,   d(5.0)]),      # far front, high
        np.array([100.0, -80.0, 140.0,    d(3.0),   d(-3.0),  d(2.0)]),      # opposite again
        np.array([  0.0,   0.0, 260.0,    d(5.0),    d(5.0),     0.0]),      # sky high + tilt (near stroke limit)
        np.array([  0.0,   0.0, 120.0,       0.0,      0.0,      0.0]),      # back to cruise
        np.zeros(6),                                                          # home
    ]

    durations = [1.0] + [1.2] * (len(waypoints) - 3) + [1.0]

    ref = ReferenceGenerator.from_waypoints(
        waypoints=waypoints,
        durations=durations,
        start_time=start_time,
    )
    total = start_time + sum(durations) + 0.5
    return ref, total


# Registry for easy lookup
TRAJECTORIES = {
    'T1': make_T1,
    'T2': make_T2,
    'T3': make_T3,
    'T4': make_T4,
    'T5': make_T5,
    'T6': make_T6,
}


def get_trajectory(name: str, **kwargs) -> tuple[ReferenceGenerator, float]:
    """Look up a scripted trajectory by name.

    Parameters
    ----------
    name : str
        Trajectory name (T1, T2, T3, T4, T5, T6).

    Returns
    -------
    ref : ReferenceGenerator
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

from catch.coordinator import DynamicTarget, BallSpawn


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

    # Take the positive root (first time ball reaches target Z)
    t1 = (-b + np.sqrt(disc)) / (2 * a)
    t2 = (-b - np.sqrt(disc)) / (2 * a)
    t = min(t for t in [t1, t2] if t > 0)

    landing_pos = spawn_pos_mm.copy()
    landing_pos[0] += spawn_vel_mms[0] * t
    landing_pos[1] += spawn_vel_mms[1] * t
    landing_pos[2] = target_z_mm

    landing_vel = spawn_vel_mms.copy()
    landing_vel[2] -= g * t

    return float(t), landing_pos, landing_vel


def _compute_catch_target(
    spawn_pos_mm: np.ndarray,
    spawn_vel_mms: np.ndarray,
    spawn_time: float,
    platform_height_mm: float = 574.3,
    hand_prime_mm: float = 322.7,
    hand_bottom_z_mm: float = -129.0,
    cone_height_mm: float = 109.0,
    active_z_offset_mm: float = 0.0,
) -> tuple[DynamicTarget, BallSpawn]:
    """Compute a DynamicTarget from a ball trajectory.

    The catch Z is the hand opening position in world frame when the
    platform is at ``active_z_offset_mm`` above home.
    """
    # Hand opening Z in world frame = platform_height + platform_z_offset + hand_bottom + hand_prime + cone_top
    # At home (z_offset=0): world_z = platform_height + hand_bottom + hand_pos + cone_height
    # But the platform moves, so we compute the target Z we want the ball to arrive at
    # The hand opening is at platform local z = hand_bottom + hand_prime + cone_height
    hand_opening_local_z = hand_bottom_z_mm + hand_prime_mm + cone_height_mm
    # Target Z in platform offset coords = hand_opening_local_z (since platform is at home + offset)

    # We want the ball to land at the hand opening. The hand opening world Z
    # depends on the platform Z offset. For simplicity, we compute the target
    # platform Z offset that puts the hand opening at the ball's predicted landing Z.

    # Catch Z = hand opening world Z when platform is at active_z_offset
    catch_z_world = platform_height_mm + active_z_offset_mm + hand_opening_local_z

    flight_time, landing_pos, landing_vel = _ball_landing(
        spawn_pos_mm, spawn_vel_mms, catch_z_world)

    arrival_time = spawn_time + flight_time

    # Target pose: XY from ball landing, Z offset to put hand at catch height.
    # The platform centroid must be at catch_z_world - hand_opening_local_z
    # in world frame.  z_offset is relative to home (initial_height), so:
    target_z_offset = catch_z_world - hand_opening_local_z - platform_height_mm
    target_pose = np.array([
        landing_pos[0],
        landing_pos[1],
        target_z_offset,
        0.0, 0.0, 0.0,  # no tilt for basic catches
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


def make_DT1() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """DT1: Single catch — ball dropped from above with slight lateral offset.

    Returns ([(target, ball_spawn)], total_duration).
    """
    spawn_pos = np.array([30.0, -20.0, 1500.0])
    spawn_vel = np.array([0.0, 0.0, -500.0])
    # Approximate hand opening Z in world frame (~870 mm)
    catch_z_world = 574.3 + (-135.0 + 322.7 + 109.0) + 80.0
    event_vel = _ball_landing_speed_mps(spawn_pos, spawn_vel, catch_z_world)

    target = DynamicTarget(
        pose_6dof=np.array([30.0, -20.0, 80.0,
                            np.radians(3.0), np.radians(-2.0), 0.0]),
        arrival_time=0.9,
        arrival_twist=None,
        hold_duration=0.5,
        event_vel_mps=event_vel,
    )

    ball = BallSpawn(
        position_mm=spawn_pos.copy(),
        velocity_mms=spawn_vel.copy(),
        spawn_time=0.1,
    )

    return [(target, ball)], 3.0


def make_DT2() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """DT2: Rapid succession — two catches 800 ms apart, different poses."""
    spawn1 = np.array([20.0, 0.0, 1500.0])
    vel1 = np.array([0.0, 0.0, -500.0])
    catch_z1 = 574.3 + (-135.0 + 322.7 + 109.0) + 80.0

    target1 = DynamicTarget(
        pose_6dof=np.array([20.0, 0.0, 80.0, 0.0, 0.0, 0.0]),
        arrival_time=0.8,
        arrival_twist=None,
        hold_duration=0.3,
        event_vel_mps=_ball_landing_speed_mps(spawn1, vel1, catch_z1),
    )
    ball1 = BallSpawn(position_mm=spawn1, velocity_mms=vel1, spawn_time=0.1)

    spawn2 = np.array([-20.0, 30.0, 1500.0])
    vel2 = np.array([0.0, 0.0, -500.0])
    catch_z2 = 574.3 + (-135.0 + 322.7 + 109.0) + 60.0

    target2 = DynamicTarget(
        pose_6dof=np.array([-20.0, 30.0, 60.0, 0.0, 0.0, 0.0]),
        arrival_time=2.5,
        arrival_twist=None,
        hold_duration=0.3,
        event_vel_mps=_ball_landing_speed_mps(spawn2, vel2, catch_z2),
    )
    ball2 = BallSpawn(position_mm=spawn2, velocity_mms=vel2, spawn_time=1.8)

    return [(target1, ball1), (target2, ball2)], 5.0


def make_DT3() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """DT3: Edge of workspace — target near workspace boundary."""
    spawn_pos = np.array([80.0, -60.0, 1500.0])
    spawn_vel = np.array([0.0, 0.0, -500.0])
    catch_z = 574.3 + (-135.0 + 322.7 + 109.0) + 100.0

    target = DynamicTarget(
        pose_6dof=np.array([80.0, -60.0, 100.0,
                            np.radians(4.0), np.radians(-3.0), 0.0]),
        arrival_time=1.0,
        arrival_twist=None,
        hold_duration=0.5,
        event_vel_mps=_ball_landing_speed_mps(spawn_pos, spawn_vel, catch_z),
    )
    ball = BallSpawn(position_mm=spawn_pos, velocity_mms=spawn_vel, spawn_time=0.1)

    return [(target, ball)], 3.5


def make_DT4() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """DT4: Infeasible — target requiring impossibly fast motion.

    Platform can't reach [120, 120, 200] with 5° combined tilt in 200 ms from home.
    The MPC should do its best but the ball should miss.
    """
    spawn_pos = np.array([120.0, 120.0, 1500.0])
    spawn_vel = np.array([0.0, 0.0, -2000.0])
    catch_z = 574.3 + (-135.0 + 322.7 + 109.0) + 200.0

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
    """DT5: Early arrival — target with 2s lead time. Platform should arrive early and hold."""
    spawn_pos = np.array([0.0, 0.0, 1500.0])
    spawn_vel = np.array([0.0, 0.0, -300.0])
    catch_z = 574.3 + (-135.0 + 322.7 + 109.0) + 60.0

    target = DynamicTarget(
        pose_6dof=np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0]),
        arrival_time=2.5,
        arrival_twist=None,
        hold_duration=0.5,
        event_vel_mps=_ball_landing_speed_mps(spawn_pos, spawn_vel, catch_z),
    )
    ball = BallSpawn(position_mm=spawn_pos, velocity_mms=spawn_vel, spawn_time=1.5)

    return [(target, ball)], 5.0


def make_DT6() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """DT6: Throw — arrive with nonzero velocity, then decelerate.

    Platform moves downward at 200 mm/s at arrival. Ball released with that velocity.
    """
    target = DynamicTarget(
        pose_6dof=np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0]),
        arrival_time=1.0,
        arrival_twist=np.array([0.0, 0.0, -200.0, 0.0, 0.0, 0.0]),
        hold_duration=0.0,
    )

    return [(target, None)], 3.0


def make_DT7() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """DT7: Catch then throw — catch at t=0.9, hold 0.3s, throw at t=1.5."""
    spawn_pos = np.array([0.0, 0.0, 1500.0])
    spawn_vel = np.array([0.0, 0.0, -500.0])
    catch_z = 574.3 + (-135.0 + 322.7 + 109.0) + 80.0

    catch_target = DynamicTarget(
        pose_6dof=np.array([0.0, 0.0, 80.0, 0.0, 0.0, 0.0]),
        arrival_time=0.9,
        arrival_twist=None,
        hold_duration=0.3,
        event_vel_mps=_ball_landing_speed_mps(spawn_pos, spawn_vel, catch_z),
    )
    catch_ball = BallSpawn(position_mm=spawn_pos, velocity_mms=spawn_vel, spawn_time=0.1)

    throw_target = DynamicTarget(
        pose_6dof=np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0]),
        arrival_time=2.5,
        arrival_twist=np.array([0.0, 0.0, -150.0, 0.0, 0.0, 0.0]),
        hold_duration=0.0,
    )

    return [(catch_target, catch_ball), (throw_target, None)], 5.0


def make_DT8() -> tuple[list[tuple[DynamicTarget, BallSpawn | None]], float]:
    """DT8: Ball miss — ball spawned far from reachable platform workspace.

    Platform tries to reach target but ball flies past. System returns to home.
    """
    spawn_pos = np.array([300.0, 0.0, 1500.0])
    spawn_vel = np.array([0.0, 0.0, -800.0])
    catch_z = 574.3 + (-135.0 + 322.7 + 109.0) + 80.0

    target = DynamicTarget(
        pose_6dof=np.array([0.0, 0.0, 80.0, 0.0, 0.0, 0.0]),
        arrival_time=0.8,
        arrival_twist=None,
        hold_duration=0.5,
        event_vel_mps=_ball_landing_speed_mps(spawn_pos, spawn_vel, catch_z),
    )
    ball = BallSpawn(position_mm=spawn_pos, velocity_mms=spawn_vel, spawn_time=0.1)

    return [(target, ball)], 3.0


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
