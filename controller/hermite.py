"""Quintic Hermite interpolation for C2-continuous motion planning.

Provides position/velocity/acceleration-matching interpolation between
two boundary states.  Used by the EventScheduler and TossLoopController
to build smooth, jerk-minimised platform trajectories.

This module has NO sim or ROS2 dependencies — pure Python + numpy.
"""

from __future__ import annotations

import numpy as np


def quintic_interp(
    p0: np.ndarray,
    v0: np.ndarray,
    a0: np.ndarray,
    p1: np.ndarray,
    v1: np.ndarray,
    a1: np.ndarray,
    duration: float,
    t: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Quintic Hermite interpolation matching pos, vel, accel at both ends.

    The polynomial in normalised time s = t/T is::

        p(s) = h0·p0 + h1·T·v0 + h2·T²·a0 + h3·p1 + h4·T·v1 + h5·T²·a1

    where h0..h5 are the quintic Hermite basis functions.

    Parameters
    ----------
    p0, p1 : (N,) start and end poses
    v0, v1 : (N,) start and end twists (real-time units, e.g. mm/s)
    a0, a1 : (N,) start and end accelerations (real-time units, e.g. mm/s²)
    duration : segment duration in seconds
    t : time since segment start (clamped to [0, duration])

    Returns
    -------
    pose : (N,) interpolated pose
    twist : (N,) interpolated twist (velocity in real-time units)
    """
    if duration <= 0:
        return p1.copy(), v1.copy()

    t = max(0.0, min(t, duration))
    T = duration
    s = t / T

    # Scale derivatives to normalised time
    V0 = v0 * T
    V1 = v1 * T
    A0 = a0 * (T * T)
    A1 = a1 * (T * T)

    s2 = s * s
    s3 = s2 * s
    s4 = s3 * s
    s5 = s4 * s

    # Quintic Hermite basis functions
    h0 = 1 - 10 * s3 + 15 * s4 - 6 * s5
    h1 = s - 6 * s3 + 8 * s4 - 3 * s5
    h2 = 0.5 * s2 - 1.5 * s3 + 1.5 * s4 - 0.5 * s5
    h3 = 10 * s3 - 15 * s4 + 6 * s5
    h4 = -4 * s3 + 7 * s4 - 3 * s5
    h5 = 0.5 * s3 - s4 + 0.5 * s5

    pose = h0 * p0 + h1 * V0 + h2 * A0 + h3 * p1 + h4 * V1 + h5 * A1

    # First derivative of basis (w.r.t. s), divided by T for real-time velocity
    inv_T = 1.0 / T
    dh0 = (-30 * s2 + 60 * s3 - 30 * s4) * inv_T
    dh1 = (1 - 18 * s2 + 32 * s3 - 15 * s4) * inv_T
    dh2 = (s - 4.5 * s2 + 6 * s3 - 2.5 * s4) * inv_T
    dh3 = (30 * s2 - 60 * s3 + 30 * s4) * inv_T
    dh4 = (-12 * s2 + 28 * s3 - 15 * s4) * inv_T
    dh5 = (1.5 * s2 - 4 * s3 + 2.5 * s4) * inv_T

    twist = dh0 * p0 + dh1 * V0 + dh2 * A0 + dh3 * p1 + dh4 * V1 + dh5 * A1

    return pose, twist


def quintic_interp_with_accel(
    p0: np.ndarray,
    v0: np.ndarray,
    a0: np.ndarray,
    p1: np.ndarray,
    v1: np.ndarray,
    a1: np.ndarray,
    duration: float,
    t: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Like :func:`quintic_interp` but also returns acceleration.

    Needed by the scheduler to read the current acceleration when replanning
    mid-segment (preserving C2 continuity across replan boundaries).

    Returns
    -------
    pose : (N,) interpolated pose
    twist : (N,) interpolated twist
    accel : (N,) interpolated acceleration (real-time units, e.g. mm/s²)
    """
    if duration <= 0:
        return p1.copy(), v1.copy(), a1.copy()

    t = max(0.0, min(t, duration))
    T = duration
    s = t / T

    V0 = v0 * T
    V1 = v1 * T
    A0 = a0 * (T * T)
    A1 = a1 * (T * T)

    s2 = s * s
    s3 = s2 * s
    s4 = s3 * s
    s5 = s4 * s

    # Position basis
    h0 = 1 - 10 * s3 + 15 * s4 - 6 * s5
    h1 = s - 6 * s3 + 8 * s4 - 3 * s5
    h2 = 0.5 * s2 - 1.5 * s3 + 1.5 * s4 - 0.5 * s5
    h3 = 10 * s3 - 15 * s4 + 6 * s5
    h4 = -4 * s3 + 7 * s4 - 3 * s5
    h5 = 0.5 * s3 - s4 + 0.5 * s5

    pose = h0 * p0 + h1 * V0 + h2 * A0 + h3 * p1 + h4 * V1 + h5 * A1

    # Velocity basis (dp/ds / T)
    inv_T = 1.0 / T
    dh0 = (-30 * s2 + 60 * s3 - 30 * s4) * inv_T
    dh1 = (1 - 18 * s2 + 32 * s3 - 15 * s4) * inv_T
    dh2 = (s - 4.5 * s2 + 6 * s3 - 2.5 * s4) * inv_T
    dh3 = (30 * s2 - 60 * s3 + 30 * s4) * inv_T
    dh4 = (-12 * s2 + 28 * s3 - 15 * s4) * inv_T
    dh5 = (1.5 * s2 - 4 * s3 + 2.5 * s4) * inv_T

    twist = dh0 * p0 + dh1 * V0 + dh2 * A0 + dh3 * p1 + dh4 * V1 + dh5 * A1

    # Acceleration basis (d²p/ds² / T²)
    inv_T2 = inv_T * inv_T
    ddh0 = (-60 * s + 180 * s2 - 120 * s3) * inv_T2
    ddh1 = (-36 * s + 96 * s2 - 60 * s3) * inv_T2
    ddh2 = (1 - 9 * s + 18 * s2 - 10 * s3) * inv_T2
    ddh3 = (60 * s - 180 * s2 + 120 * s3) * inv_T2
    ddh4 = (-24 * s + 84 * s2 - 60 * s3) * inv_T2
    ddh5 = (3 * s - 12 * s2 + 10 * s3) * inv_T2

    accel = ddh0 * p0 + ddh1 * V0 + ddh2 * A0 + ddh3 * p1 + ddh4 * V1 + ddh5 * A1

    return pose, twist, accel


def quintic_jerk_integral(
    p0: np.ndarray,
    v0: np.ndarray,
    a0: np.ndarray,
    p1: np.ndarray,
    v1: np.ndarray,
    a1: np.ndarray,
    duration: float,
) -> float:
    """Analytical integral of squared jerk over a quintic Hermite segment.

    Used by the catch height optimizer to score candidate catch poses by
    platform effort without running the MPC.

    For the quintic polynomial p(s) with normalised time s = t/T, the jerk
    is d³p/dt³ = p'''(s) / T³.  The integral ∫₀ᵀ ||j(t)||² dt can be
    computed analytically since p'''(s) is quadratic in s.

    Returns
    -------
    float
        Sum over all dimensions of ∫₀ᵀ jerk²(t) dt.  Lower is smoother.
    """
    if duration <= 0:
        return float('inf')

    T = duration

    # Boundary values in normalised-time coordinates
    V0 = v0 * T
    V1 = v1 * T
    A0 = a0 * (T * T)
    A1 = a1 * (T * T)

    dp = p1 - p0

    # Third derivative of the quintic basis (constant + linear + quadratic in s):
    # p'''(s) = c0 + c1·s + c2·s²
    # where c0, c1, c2 are vectors derived from the 6 boundary conditions.
    #
    # From the quintic Hermite:
    #   h0'''(s) = -60 + 360s - 360s²   →  for p0
    #   h1'''(s) = -36 + 192s - 180s²   →  for V0
    #   h2'''(s) =  -9 +  36s -  30s²   →  for A0
    #   h3'''(s) =  60 - 360s + 360s²   →  for p1
    #   h4'''(s) = -24 + 168s - 180s²   →  for V1
    #   h5'''(s) =   3 -  24s +  30s²   →  for A1

    c0 = -60 * p0 - 36 * V0 - 9 * A0 + 60 * p1 - 24 * V1 + 3 * A1
    c1 = 360 * p0 + 192 * V0 + 36 * A0 - 360 * p1 + 168 * V1 - 24 * A1
    c2 = -360 * p0 - 180 * V0 - 30 * A0 + 360 * p1 - 180 * V1 + 30 * A1

    # jerk(t) = p'''(s) / T³, so jerk² = (p'''(s))² / T⁶
    # ∫₀ᵀ jerk² dt = ∫₀¹ (p'''(s))² / T⁶ · T ds = (1/T⁵) ∫₀¹ (c0 + c1·s + c2·s²)² ds
    #
    # ∫₀¹ (c0 + c1·s + c2·s²)² ds  (per-dimension, then sum)
    #   = c0² + c0·c1 + (2·c0·c2 + c1²)/3 + c1·c2/2 + c2²/5

    per_dim = (
        c0 * c0
        + c0 * c1
        + (2 * c0 * c2 + c1 * c1) / 3.0
        + c1 * c2 / 2.0
        + c2 * c2 / 5.0
    )

    return float(np.sum(per_dim)) / (T ** 5)
