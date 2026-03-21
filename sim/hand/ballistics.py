"""Ballistic utilities for throw-catch planning.

Provides inverse ballistics (required launch velocity for a given flight),
orientation computation (align platform Z with launch/catch direction),
and hand offset calculation.
"""

from __future__ import annotations

import numpy as np

# Gravity in mm/s² (matching sim convention)
_GRAVITY_MMS2 = 9806.0

# Hand offset constants (from hardware_config.yaml + MuJoCo model geometry)
_HAND_AXIS_BOTTOM_OFFSET_MM = -129.0
# Distance from hand body origin to ball COM when held (mm).
# The hand_opening site is at +40mm; ball COM sits 4.4mm above the cone rim.
_BALL_SEAT_OFFSET_MM = 44.4

# Maximum tilt angle (radians) — workspace limit
_TILT_LIMIT_RAD = np.radians(30.0)


def compute_launch_velocity(
    release_pos_mm: np.ndarray,
    target_pos_mm: np.ndarray,
    flight_time_s: float,
) -> np.ndarray:
    """Ballistic inverse: required velocity at release for ball to reach target.

    Returns (3,) velocity in mm/s.
    Assumes no drag, constant gravity = [0, 0, -9806] mm/s².
    """
    dt = flight_time_s
    return ((target_pos_mm - release_pos_mm) / dt
            + np.array([0.0, 0.0, 0.5 * _GRAVITY_MMS2 * dt]))


def compute_arrival_velocity(
    launch_vel_mms: np.ndarray,
    flight_time_s: float,
) -> np.ndarray:
    """Velocity at catch time given launch velocity.

    Returns (3,) velocity in mm/s.
    """
    return launch_vel_mms + np.array([0.0, 0.0, -_GRAVITY_MMS2 * flight_time_s])


def compute_orientation(target_direction: np.ndarray) -> np.ndarray:
    """Rotation vector to align platform +Z with target_direction.

    Returns (3,) rotation vector.  Returns zeros if direction is already +Z.
    Raises ValueError if angle exceeds 30 degrees (workspace limit).
    """
    d = target_direction
    norm = np.linalg.norm(d)
    if norm < 1e-9:
        raise ValueError("Zero-length direction vector")

    d_hat = d / norm
    reference = np.array([0.0, 0.0, 1.0])
    dot = float(np.clip(np.dot(reference, d_hat), -1.0, 1.0))
    angle = np.arccos(dot)

    if angle > _TILT_LIMIT_RAD:
        raise ValueError(
            f"Tilt angle {np.degrees(angle):.1f}° exceeds "
            f"{np.degrees(_TILT_LIMIT_RAD):.0f}° limit"
        )

    if angle < 1e-9:
        return np.zeros(3)

    axis = np.cross(reference, d_hat)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-9:
        # Anti-parallel: direction is straight down
        return np.array([np.pi, 0.0, 0.0])

    axis /= axis_norm
    return axis * angle


def compute_hand_offset_mm(hand_pos_mm: float) -> float:
    """Offset from platform centroid to ball COM along platform local Z.

    hand_pos_mm: hand position in mm from physical bottom of stroke.
    Returns: offset in mm (positive = above centroid).

    Formula: HAND_AXIS_BOTTOM_OFFSET_MM + hand_pos_mm + BALL_SEAT_OFFSET_MM
             = -129.0 + hand_pos_mm + 44.4
    """
    return _HAND_AXIS_BOTTOM_OFFSET_MM + hand_pos_mm + _BALL_SEAT_OFFSET_MM


def rodrigues(rv: np.ndarray) -> np.ndarray:
    """Rotation vector → 3×3 rotation matrix via Rodrigues' formula."""
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
