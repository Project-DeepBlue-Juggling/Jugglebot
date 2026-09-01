"""Pure ballistic geometry helpers — no drag, constant gravity.

Hardware-safe (numpy only). Computes throw / catch platform poses from throw
position, catch position, and an apex-height constraint. (The original caller
was ``run_mpc.py``'s ``--toss-motion`` path, removed 2026-09-01 with the MPC
chain — tag ``mpc-final``. The geometry is caller-agnostic and is now consumed
by ``sim/`` and the toss planner.)

The symbols here were originally defined inside ``sim/hand/ballistics.py``
but are equally applicable to the hardware layer (they are plain ballistics
with no simulation-specific behaviour). ``sim/hand/ballistics.py`` now
re-exports ``compute_launch_velocity``, ``compute_arrival_velocity``,
``compute_orientation``, and ``rodrigues`` from here; hand-specific helpers
(``compute_hand_offset_mm``) stay in the sim module.
"""

from __future__ import annotations

import numpy as np


GRAVITY_MMS2 = 9806.0

TILT_LIMIT_RAD = np.radians(30.0)


def compute_launch_velocity(
    release_pos_mm: np.ndarray,
    target_pos_mm: np.ndarray,
    flight_time_s: float,
) -> np.ndarray:
    """Ballistic inverse: velocity at release required for the ball to reach
    ``target_pos_mm`` from ``release_pos_mm`` after ``flight_time_s``.

    Assumes no drag, constant gravity ``[0, 0, -GRAVITY_MMS2]``. Returns
    a (3,) ndarray in mm/s.
    """
    dt = flight_time_s
    return ((target_pos_mm - release_pos_mm) / dt
            + np.array([0.0, 0.0, 0.5 * GRAVITY_MMS2 * dt]))


def compute_arrival_velocity(
    launch_vel_mms: np.ndarray,
    flight_time_s: float,
) -> np.ndarray:
    """Ball velocity at catch time, given launch velocity. (3,) ndarray mm/s."""
    return launch_vel_mms + np.array(
        [0.0, 0.0, -GRAVITY_MMS2 * flight_time_s])


def flight_time_from_apex(
    throw_pos_mm: np.ndarray,
    catch_pos_mm: np.ndarray,
    apex_height_mm: float,
) -> float:
    """Flight time for a throw that reaches ``apex_height_mm`` above the
    throw position and lands at ``catch_pos_mm``.

    Solves the quadratic ``½g·t² − v_z·t + Δz = 0`` where
    ``v_z = √(2·g·apex)`` and ``Δz = catch.z − throw.z``. Returns the later
    (descent) root — the earlier root corresponds to the ascent crossing,
    which is only physical for catches on the way up.

    Raises ``ValueError`` if the catch is higher than the apex (the ball
    never reaches that height from the given throw) or if ``apex_height_mm``
    is non-positive.
    """
    if apex_height_mm <= 0:
        raise ValueError(
            f"apex_height_mm must be positive (got {apex_height_mm})")

    throw_pos = np.asarray(throw_pos_mm, dtype=float)
    catch_pos = np.asarray(catch_pos_mm, dtype=float)
    dz = float(catch_pos[2] - throw_pos[2])

    if dz > apex_height_mm:
        raise ValueError(
            f"catch above apex: Δz={dz:.1f} mm > apex={apex_height_mm:.1f} mm — "
            f"the ball cannot reach the catch height from this throw")

    v_z = np.sqrt(2.0 * GRAVITY_MMS2 * apex_height_mm)
    # Discriminant: v_z² − 2g·Δz. With the Δz ≤ apex guard above, this is
    # always non-negative (equality when the catch is exactly at the apex,
    # where the ball has zero vertical velocity — a single-root landing).
    disc = v_z * v_z - 2.0 * GRAVITY_MMS2 * dz
    disc = max(disc, 0.0)  # clamp tiny negative from float roundoff
    t_descent = (v_z + np.sqrt(disc)) / GRAVITY_MMS2
    return float(t_descent)


def compute_orientation(target_direction: np.ndarray) -> np.ndarray:
    """Rotation vector that aligns the platform +Z with ``target_direction``.

    Returns a (3,) rotation vector (axis × angle in rad). Returns zeros when
    the direction already coincides with +Z. Raises ``ValueError`` if the
    resulting tilt would exceed ``TILT_LIMIT_RAD`` (the platform workspace
    limit).
    """
    d = target_direction
    norm = np.linalg.norm(d)
    if norm < 1e-9:
        raise ValueError("Zero-length direction vector")

    d_hat = d / norm
    reference = np.array([0.0, 0.0, 1.0])
    dot = float(np.clip(np.dot(reference, d_hat), -1.0, 1.0))
    angle = np.arccos(dot)

    if angle > TILT_LIMIT_RAD:
        raise ValueError(
            f"Tilt angle {np.degrees(angle):.1f}° exceeds "
            f"{np.degrees(TILT_LIMIT_RAD):.0f}° limit")

    if angle < 1e-9:
        return np.zeros(3)

    axis = np.cross(reference, d_hat)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-9:
        # Anti-parallel: direction is straight down (d_hat ≈ -Z).  Unreachable
        # today because the TILT_LIMIT_RAD=30° guard above rejects any angle
        # > 30° first, so we never get here for angles near 180°.  Kept as a
        # defensive branch so compute_orientation remains correct if the tilt
        # limit is ever loosened.
        return np.array([np.pi, 0.0, 0.0])

    axis /= axis_norm
    return axis * angle


def rodrigues(rv: np.ndarray) -> np.ndarray:
    """Rotation vector → 3×3 rotation matrix (Rodrigues' formula)."""
    angle = np.linalg.norm(rv)
    if angle < 1e-10:
        return np.eye(3)
    K = np.array([
        [0, -rv[2], rv[1]],
        [rv[2], 0, -rv[0]],
        [-rv[1], rv[0], 0],
    ])
    return (np.eye(3)
            + (np.sin(angle) / angle) * K
            + ((1 - np.cos(angle)) / (angle * angle)) * K @ K)
