"""Ballistic trajectory computation for Ball Butler throws.

Pure Python / math only — no ROS2 dependency.  Two complementary directions:

* ``predict_throw`` — forward:   (yaw, pitch, speed) → landing position
* ``solve_throw_local`` — inverse: (target x,y,z) → (yaw, pitch, speed, tof)

Plus ``global_to_bb_local`` for converting world-frame target positions
into BB local frame given the BB's calibrated origin + yaw offset.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import jugglebot.hardware_config as hw

# Gravity in mm/s² (canonical value from hardware_config)
_G_MMPS2 = hw.GRAVITY_MPS2 * 1000.0

# Default landing plane: platform active height + vertical offset for catch
_DEFAULT_CATCH_HEIGHT_MM = (hw.GEOM_INITIAL_HEIGHT_MM
                           + hw.JB_OP_DEFAULT_ACTIVE_Z_MM
                           + hw.HAND_CATCH_OFFSET_MM)


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


# ═══════════════════════════════════════════════════════════════════
# Inverse: target → throw parameters
# ═══════════════════════════════════════════════════════════════════


def _wrap_pi(angle: float) -> float:
    """Wrap angle to (-π, π]."""
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return a if a != -math.pi else math.pi


def yaw_solve_thetas(x: float, y: float, s: float) -> Tuple[float, float, float]:
    """Solve the BB yaw geometry.

    The release point is offset by ``s`` along the BB's lateral axis from the
    yaw axis, so the throw-direction yaw ``θ`` satisfies::

        x = r·cos(θ) − s·sin(θ)
        y = r·sin(θ) + s·cos(θ)

    where ``r = √(x²+y²−s²)`` is the horizontal range along the throw direction.

    Returns ``(t1, t2, chosen)`` in radians wrapped to (-π, π].  ``chosen`` is
    the solution with the smaller magnitude.  If no solution exists (target
    is inside the s-offset circle, or x == y == 0) all three return NaN.
    """
    hyp = math.hypot(x, y)
    r_sq = x * x + y * y - s * s
    if r_sq < 0.0 or hyp == 0.0:
        nan = float('nan')
        return nan, nan, nan
    base = math.atan2(y, x)
    s_over_hyp = max(-1.0, min(1.0, s / hyp))
    delta = math.asin(s_over_hyp)
    t1 = _wrap_pi(base - delta)
    t2 = _wrap_pi(base - math.pi + delta)
    chosen = t1 if abs(t1) <= abs(t2) else t2
    return t1, t2, chosen


@dataclass
class ThrowSolution:
    """Inverse-ballistics solution for a target position.

    All angles in radians (BB local frame), speed in m/s, tof in seconds,
    heights in mm.  ``peak_height_mm`` is measured *above the release point*.
    """
    yaw_rad: float
    pitch_rad: float
    speed_mps: float
    tof_s: float
    peak_height_mm: float


def global_to_bb_local(
    x_global_mm: float, y_global_mm: float, z_global_mm: float,
    bb_position_mm: Tuple[float, float, float],
    yaw_offset_rad: float,
) -> Tuple[float, float, float]:
    """Transform a world-frame point into BB local frame.

    BB calibration gives the BB origin position in world frame
    (``bb_position_mm``) and the rotation from BB-local-yaw=0 to the world
    +x axis (``yaw_offset_rad``).  The local frame's x axis points along
    BB-local-yaw=0 (i.e. straight forward when BB yaw is commanded to zero).
    """
    dx = x_global_mm - bb_position_mm[0]
    dy = y_global_mm - bb_position_mm[1]
    dz = z_global_mm - bb_position_mm[2]

    cos_off = math.cos(yaw_offset_rad)
    sin_off = math.sin(yaw_offset_rad)
    # Inverse rotation: rotate world delta by −yaw_offset into BB local.
    x_local = dx * cos_off + dy * sin_off
    y_local = -dx * sin_off + dy * cos_off
    return x_local, y_local, dz


def solve_throw_local(
    x_mm: float, y_mm: float, z_mm: float,
    *,
    yaw_s_offset_mm: float = hw.BB_GEOM_YAW_S_OFFSET_MM,
    pitch_d_offset_mm: float = hw.BB_GEOM_PITCH_D_OFFSET_MM,
    release_l_mm: float = hw.BB_GEOM_RELEASE_L_POSITION_MM,
    pitch_min_deg: float = hw.BB_GEOM_PITCH_MIN_DEG,
    pitch_max_deg: float = hw.BB_GEOM_PITCH_MAX_DEG,
    yaw_min_deg: float = hw.BB_GEOM_YAW_MIN_DEG,
    yaw_max_deg: float = hw.BB_GEOM_YAW_MAX_DEG,
    max_speed_mps: float = hw.BB_OP_MAX_THROW_SPEED_MPS,
    max_height_mm: float = hw.BB_OP_MAX_THROW_HEIGHT_M * 1000.0,
    pitch_step_deg: float = 0.5,
    g_mps2: float = hw.GRAVITY_MPS2,
) -> ThrowSolution:
    """Solve BB throw parameters to land at ``(x, y, z)`` in BB local frame.

    BB kinematics: yaw axis → pitch axis (offset ``d``) → linear axis →
    release point (offset ``s`` lateral, ``l`` along the linear axis).  The
    yaw solution is independent of pitch / l / d (it's a 2-D ground-plane
    geometry).  Pitch is searched on a coarse grid, picking the pitch that
    minimises horizontal landing velocity (= the most-vertical landing) while
    respecting speed and height limits.

    All distance arguments are mm; angle arguments are degrees.  Mixing
    units keeps the call site readable against ``hardware_config`` defaults.

    Raises ``ValueError`` if no feasible solution exists.  The message names
    the binding constraint so the caller can surface it (e.g. "yaw out of
    range", "no pitch satisfies speed+height limits", "target inside
    s-offset circle").
    """
    s = yaw_s_offset_mm
    d = pitch_d_offset_mm
    l = release_l_mm
    v_max_mmps = max_speed_mps * 1000.0
    h_max_mm = max_height_mm
    g_mmps2 = g_mps2 * 1000.0

    yaw_min_rad = math.radians(yaw_min_deg)
    yaw_max_rad = math.radians(yaw_max_deg)
    pitch_min_rad = math.radians(pitch_min_deg)
    pitch_max_rad = math.radians(pitch_max_deg)

    # ---- Yaw from 2-D geometry (independent of pitch, l, d) ----
    _, _, yaw_rad = yaw_solve_thetas(x_mm, y_mm, s)
    if not math.isfinite(yaw_rad):
        raise ValueError(
            f"No yaw solution for target (x={x_mm:.0f}, y={y_mm:.0f} mm) "
            f"with s={s:.1f} mm — target inside the s-offset circle.")

    if yaw_rad < yaw_min_rad or yaw_rad > yaw_max_rad:
        raise ValueError(
            f"Yaw {math.degrees(yaw_rad):.1f}° out of BB range "
            f"[{yaw_min_deg:.1f}°, {yaw_max_deg:.1f}°].")

    # ---- A = horizontal range along the throw direction ----
    A_sq = x_mm * x_mm + y_mm * y_mm - s * s
    if A_sq < 0:
        raise ValueError(
            f"Target (x={x_mm:.0f}, y={y_mm:.0f} mm) inside s-offset "
            f"circle (s={s:.1f} mm).")
    A = math.sqrt(A_sq)

    # ---- Degenerate: target directly above/below yaw axis ----
    if A <= 1e-9:
        if z_mm >= 0:
            raise ValueError(
                "Target is directly above BB origin — cannot reach with "
                "projectile motion.")
        pitch_rad = -math.pi / 2
        v_mmps = math.sqrt(2 * g_mmps2 * (-z_mm))
        if v_mmps > v_max_mmps:
            raise ValueError(
                f"Target too far below: need {v_mmps / 1000.0:.2f} m/s, "
                f"max {max_speed_mps:.2f} m/s.")
        t = math.sqrt(-2 * z_mm / g_mmps2)
        return ThrowSolution(
            yaw_rad=yaw_rad, pitch_rad=pitch_rad,
            speed_mps=v_mmps / 1000.0, tof_s=t, peak_height_mm=0.0)

    # ---- Search pitch on a coarse grid ----
    pitch_step_rad = math.radians(pitch_step_deg)
    n_steps = max(int((pitch_max_rad - pitch_min_rad) / pitch_step_rad), 1) + 1

    best = None  # (h_vel, pitch_rad, v_mmps, h_peak_mm)
    for i in range(n_steps):
        pitch_rad = pitch_min_rad + i * (pitch_max_rad - pitch_min_rad) / max(n_steps - 1, 1)
        cos_p = math.cos(pitch_rad)
        sin_p = math.sin(pitch_rad)
        if abs(cos_p) < 1e-6:
            continue

        # Range from release point to target (horizontal)
        R = A - l * cos_p + d
        if R <= 0:
            continue

        # Throw height: vertical distance from release point to target
        z_throw_mm = z_mm - l * sin_p

        # Standard projectile: v² = g·R² / (R·sin(2φ) − Δz·(1+cos(2φ)))
        sin_2p = 2.0 * sin_p * cos_p
        cos_2p = cos_p * cos_p - sin_p * sin_p
        denom = R * sin_2p - z_throw_mm * (1.0 + cos_2p)
        if denom <= 0:
            continue
        v_sq = g_mmps2 * R * R / denom
        if v_sq <= 0:
            continue
        v_mmps = math.sqrt(v_sq)
        if v_mmps > v_max_mmps:
            continue

        # Peak above release point
        v_vert = v_mmps * sin_p
        h_peak_mm = (v_vert * v_vert) / (2.0 * g_mmps2) if v_vert > 0 else 0.0
        if h_peak_mm > h_max_mm:
            continue

        # Minimise horizontal landing velocity (= most-vertical landing)
        h_vel = v_mmps * cos_p
        if best is None or h_vel < best[0]:
            best = (h_vel, pitch_rad, v_mmps, h_peak_mm)

    if best is None:
        raise ValueError(
            f"No feasible trajectory for target (A={A:.0f}, z={z_mm:.0f} mm). "
            f"Limits: v_max={max_speed_mps:.2f} m/s, "
            f"h_max={max_height_mm:.0f} mm, "
            f"pitch [{pitch_min_deg:.0f}°, {pitch_max_deg:.0f}°].")

    _, pitch_rad, v_mmps, h_peak_mm = best
    # Time of flight (using release-point-corrected range)
    R_final = A - l * math.cos(pitch_rad) + d
    tof_s = R_final / (v_mmps * math.cos(pitch_rad))

    return ThrowSolution(
        yaw_rad=yaw_rad,
        pitch_rad=pitch_rad,
        speed_mps=v_mmps / 1000.0,
        tof_s=tof_s,
        peak_height_mm=h_peak_mm,
    )
