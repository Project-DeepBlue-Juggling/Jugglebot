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
    # start and matching circular velocity at end
    orbit_start_pose = np.array([radius, 0.0, z_offset, 0.0, 0.0, 0.0])
    # Velocity at orbit start: vx=0, vy=radius*omega (tangent to circle at theta=0)
    orbit_start_twist = np.array([0.0, radius * omega, 0.0, 0.0, 0.0, 0.0])

    ramp_traj = create_trajectory(
        start_pose=np.zeros(6),
        start_twist=np.zeros(6),
        start_accel=np.zeros(6),
        end_pose=orbit_start_pose,
        end_twist=orbit_start_twist,
        end_accel=np.zeros(6),
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
            # After orbit: hold at orbit start
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
        Trajectory name (T1, T2, T3, T4).

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
