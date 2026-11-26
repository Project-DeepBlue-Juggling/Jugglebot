"""Ballistic trajectory prediction — pure Python.

Solves the quadratic for when a ball (under gravity only) crosses a
horizontal plane at a given Z height. Returns landing position, velocity,
and time-to-land.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np

# Gravity in mm/s² — canonical value from hardware_config.
# Hardcoded here to keep the tracking subpackage free of jugglebot imports
# so it can be tested standalone. Must match hw.GRAVITY_MPS2 * 1000.
GRAVITY_MMPS2 = 9806.0


def predict_landing_state(
    position: np.ndarray,
    velocity: np.ndarray,
    landing_z: float,
    g_xy_mmps2: Optional[Sequence[float]] = None,
) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
    """Predict where and when a ball will cross a horizontal plane.

    Uses ballistic equations (gravity only, no drag).

    ``g_xy_mmps2`` is gravity's HORIZONTAL part in the caller's frame, mm/s^2
    (FEAT-27, 2026-11-26): the Mocap world frame is tilted 0.55 deg from
    gravity, measured (+43, +83) mm/s^2, so a flight there curves sideways
    as well as down. It moves the landing xy by ``0.5 g_xy t^2`` and the
    landing velocity by ``g_xy t``; the crossing time (the z root) is
    unchanged, with the vertical part still ``GRAVITY_MMPS2``. None or
    (0, 0) is the pre-FEAT-27 arithmetic exactly, bit for bit.

    Args:
        position: Current [x, y, z] in mm.
        velocity: Current [vx, vy, vz] in mm/s.
        landing_z: Z-height of the landing plane in mm.

    Returns:
        (landing_pos, landing_vel, time_to_land) or None if the ball
        never reaches the plane (no real positive solution).
        - landing_pos: [x, y, z] in mm (z == landing_z).
        - landing_vel: [vx, vy, vz] in mm/s at landing.
        - time_to_land: seconds from now until landing (> 0).
    """
    x, y, z = float(position[0]), float(position[1]), float(position[2])
    vx, vy, vz = float(velocity[0]), float(velocity[1]), float(velocity[2])

    # Solve: z + vz*t - 0.5*g*t² = landing_z
    # Rearranged: -0.5*g*t² + vz*t + (z - landing_z) = 0
    # Standard form: a*t² + b*t + c = 0
    a = -0.5 * GRAVITY_MMPS2
    b = vz
    c = z - landing_z

    discriminant = b * b - 4.0 * a * c
    if discriminant < 0:
        return None

    sqrt_disc = math.sqrt(discriminant)
    t1 = (-b + sqrt_disc) / (2.0 * a)
    t2 = (-b - sqrt_disc) / (2.0 * a)

    # Pick the smallest positive root
    positive_roots = [t for t in (t1, t2) if t > 1e-9]
    if not positive_roots:
        return None

    time_to_land = min(positive_roots)

    landing_x = x + vx * time_to_land
    landing_y = y + vy * time_to_land

    landing_vx = vx
    landing_vy = vy
    landing_vz = vz - GRAVITY_MMPS2 * time_to_land

    if g_xy_mmps2 is not None:
        gx, gy = float(g_xy_mmps2[0]), float(g_xy_mmps2[1])
        if gx != 0.0 or gy != 0.0:
            half_t2 = 0.5 * time_to_land * time_to_land
            landing_x += gx * half_t2
            landing_y += gy * half_t2
            landing_vx += gx * time_to_land
            landing_vy += gy * time_to_land

    landing_pos = np.array([landing_x, landing_y, landing_z])
    landing_vel = np.array([landing_vx, landing_vy, landing_vz])

    return landing_pos, landing_vel, time_to_land
