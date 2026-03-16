"""Ballistic trajectory computation for Ball Butler throw announcements.

Pure Python / math only — no ROS2 dependency.  Given the throw parameters
(yaw, pitch, speed) and the BB's calibrated position + yaw offset, computes
the initial velocity vector in the global frame and predicts the landing
state (position, velocity, time-of-flight) at a configurable catch height.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import jugglebot.hardware_config as hw

# Gravity in mm/s² (canonical value from hardware_config)
_G_MMPS2 = hw.GRAVITY_MPS2 * 1000.0

# Default landing plane: platform home height + vertical offset for catch
_DEFAULT_CATCH_HEIGHT_MM = hw.GEOM_INITIAL_HEIGHT_MM + 160.0


@dataclass
class ThrowPrediction:
    """Result of a ballistic throw prediction.

    All positions in mm (global frame), velocities in mm/s, times in seconds.
    """
    # Initial state at throw time
    initial_position: Tuple[float, float, float]
    initial_velocity: Tuple[float, float, float]

    # Predicted landing state
    landing_position: Tuple[float, float, float]
    landing_velocity: Tuple[float, float, float]
    tof_s: float  # time of flight from throw to landing


def predict_throw(
    yaw_rad: float,
    pitch_rad: float,
    speed_mps: float,
    bb_position_mm: Tuple[float, float, float],
    yaw_offset_rad: float = 0.0,
    catch_height_mm: Optional[float] = None,
) -> Optional[ThrowPrediction]:
    """Predict where a thrown ball will land given throw parameters.

    Args:
        yaw_rad: BB yaw command in radians (-π to π), in BB local frame.
        pitch_rad: BB pitch command in radians (0 to π/2).
        speed_mps: Throw speed in m/s.
        bb_position_mm: BB global position (x, y, z) in mm from calibration.
        yaw_offset_rad: Offset from BB local yaw to global frame (from calibration).
        catch_height_mm: Z-height of the landing/catch plane (mm).
            Defaults to initial_height + 160 mm.

    Returns:
        ThrowPrediction with all fields populated, or None if the ball
        never reaches the catch plane (e.g. thrown too slowly or downward).
    """
    if catch_height_mm is None:
        catch_height_mm = _DEFAULT_CATCH_HEIGHT_MM

    # Decompose speed into global-frame velocity components (mm/s)
    cos_pitch = math.cos(pitch_rad)
    sin_pitch = math.sin(pitch_rad)
    global_yaw = yaw_rad + yaw_offset_rad
    cos_yaw = math.cos(global_yaw)
    sin_yaw = math.sin(global_yaw)

    vx = speed_mps * cos_pitch * cos_yaw * 1000.0
    vy = speed_mps * cos_pitch * sin_yaw * 1000.0
    vz = speed_mps * sin_pitch * 1000.0

    # Vertical kinematics: solve for time when z(t) = catch_height_mm
    #   z(t) = z0 + vz*t - 0.5*g*t²
    #   0.5*g*t² - vz*t + (catch_height_mm - z0) = 0
    z0 = bb_position_mm[2]
    dz = catch_height_mm - z0

    a = 0.5 * _G_MMPS2
    b = -vz
    c = dz

    discriminant = b * b - 4.0 * a * c
    if discriminant < 0:
        return None  # Ball never reaches catch plane

    sqrt_disc = math.sqrt(discriminant)
    # Two solutions; we want the positive, larger one (ball going up then coming down)
    t1 = (-b - sqrt_disc) / (2.0 * a)
    t2 = (-b + sqrt_disc) / (2.0 * a)

    # Pick the latest positive root (the descending crossing)
    tof = t2 if t2 > 0 else t1
    if tof <= 0:
        return None  # No valid future crossing

    # Landing position
    x_land = bb_position_mm[0] + vx * tof
    y_land = bb_position_mm[1] + vy * tof
    z_land = catch_height_mm

    # Landing velocity (horizontal unchanged, vertical decays under gravity)
    vx_land = vx
    vy_land = vy
    vz_land = vz - _G_MMPS2 * tof

    return ThrowPrediction(
        initial_position=tuple(bb_position_mm),
        initial_velocity=(vx, vy, vz),
        landing_position=(x_land, y_land, z_land),
        landing_velocity=(vx_land, vy_land, vz_land),
        tof_s=tof,
    )
