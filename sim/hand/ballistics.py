"""Hand/ball ballistic utilities.

Inverse ballistics, orientation computation and Rodrigues are provided by
``controller/ballistics.py`` (hardware-safe, pure numpy). This module
re-exports them for backward compatibility and adds the hand-specific
centroid-offset helper used by the sim's throw-catch planner.
"""

from __future__ import annotations

from controller.ballistics import (  # noqa: F401  (re-exported for compat)
    GRAVITY_MMS2 as _GRAVITY_MMS2,
    TILT_LIMIT_RAD as _TILT_LIMIT_RAD,
    compute_arrival_velocity,
    compute_launch_velocity,
    compute_orientation,
    flight_time_from_apex,
    rodrigues,
)

# Hand offset constants (from hardware_config.yaml + MuJoCo model geometry)
_HAND_AXIS_BOTTOM_OFFSET_MM = -129.0
# Distance from hand body origin to ball COM when held (mm).
# The hand_opening site is at +40mm; ball COM sits 4.4mm above the cone rim.
_BALL_SEAT_OFFSET_MM = 44.4


def compute_hand_offset_mm(hand_pos_mm: float) -> float:
    """Offset from platform centroid to ball COM along platform local Z.

    hand_pos_mm: hand position in mm from physical bottom of stroke.
    Returns: offset in mm (positive = above centroid).

    Formula: HAND_AXIS_BOTTOM_OFFSET_MM + hand_pos_mm + BALL_SEAT_OFFSET_MM
             = -129.0 + hand_pos_mm + 44.4
    """
    return _HAND_AXIS_BOTTOM_OFFSET_MM + hand_pos_mm + _BALL_SEAT_OFFSET_MM
