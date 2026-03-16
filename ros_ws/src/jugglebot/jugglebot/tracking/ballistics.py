"""Ballistic trajectory prediction — pure Python.

Solves the quadratic for when a ball (under gravity only) crosses a
horizontal plane at a given Z height. Returns landing position, velocity,
and time-to-land.
"""

import math
from typing import Optional, Tuple

import numpy as np

# Gravity in mm/s² — canonical value from hardware_config.
# Hardcoded here to keep the tracking subpackage free of jugglebot imports
# so it can be tested standalone. Must match hw.GRAVITY_MPS2 * 1000.
GRAVITY_MMPS2 = 9806.0


def predict_landing_state(
    position: np.ndarray,
    velocity: np.ndarray,
    landing_z: float,
) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
    """Predict where and when a ball will cross a horizontal plane.

    Uses ballistic equations (gravity only, no drag).

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

    landing_pos = np.array([landing_x, landing_y, landing_z])
    landing_vel = np.array([landing_vx, landing_vy, landing_vz])

    return landing_pos, landing_vel, time_to_land
