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


def bb_release_state(
    yaw_rad: float,
    pitch_rad: float,
    speed_mps: float,
    bb_position_mm: Tuple[float, float, float],
    yaw_offset_rad: float = 0.0,
    *,
    yaw_s_offset_mm: float = hw.BB_GEOM_YAW_S_OFFSET_MM,
    pitch_d_offset_mm: float = hw.BB_GEOM_PITCH_D_OFFSET_MM,
    release_l_mm: float = hw.BB_GEOM_RELEASE_L_POSITION_MM,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """Analytic BB forward kinematics: actual launch (yaw, pitch, speed) →
    ``(release_point, release_velocity)`` in the WORLD frame (mm, mm/s).

    THE single source of truth for where and how the ball leaves BB.
    ``predict_throw`` propagates a ballistic arc from here; ``solve_throw_local``
    is its analytic inverse — the round-trip is asserted in the tests so the two
    can never silently drift again (this function exists to kill exactly that
    drift, where the old point-launch ``predict_throw`` disagreed with the
    serial-chain solver by ~100 mm / tens of ms).

    Geometry (identical to ``solve_throw_local``): from the yaw axis the release
    point lies ``a = l·cosφ − d`` along the throw azimuth, ``s`` lateral, and
    ``l·sinφ`` above the pitch-axis plane.  ``yaw_offset_rad`` rotates BB-local
    yaw into world.  ``pitch_z_offset`` is already baked into ``bb_position_mm.z``
    by calibration (see ``solve_throw_local`` frame convention), so it is NOT
    re-added here.

    Inputs are *actual* launch quantities: apply any axis calibration (cmd→actual)
    in the caller — that map is deliberately kept out of the geometry so this
    stays the one true forward model.
    """
    global_yaw = yaw_rad + yaw_offset_rad
    cos_p = math.cos(pitch_rad)
    sin_p = math.sin(pitch_rad)
    cos_y = math.cos(global_yaw)
    sin_y = math.sin(global_yaw)

    a = release_l_mm * cos_p - pitch_d_offset_mm   # along-throw dist from yaw axis
    s = yaw_s_offset_mm
    release = (
        bb_position_mm[0] + a * cos_y - s * sin_y,
        bb_position_mm[1] + a * sin_y + s * cos_y,
        bb_position_mm[2] + release_l_mm * sin_p,
    )
    v_mmps = speed_mps * 1000.0
    velocity = (
        v_mmps * cos_p * cos_y,
        v_mmps * cos_p * sin_y,
        v_mmps * sin_p,
    )
    return release, velocity


def predict_throw(
    yaw_rad: float,
    pitch_rad: float,
    speed_mps: float,
    bb_position_mm: Tuple[float, float, float],
    yaw_offset_rad: float = 0.0,
    catch_height_mm: Optional[float] = None,
    *,
    yaw_s_offset_mm: float = hw.BB_GEOM_YAW_S_OFFSET_MM,
    pitch_d_offset_mm: float = hw.BB_GEOM_PITCH_D_OFFSET_MM,
    release_l_mm: float = hw.BB_GEOM_RELEASE_L_POSITION_MM,
) -> Optional[ThrowPrediction]:
    """Predict where a thrown ball will land given *actual* launch parameters.

    Launches from the geometric RELEASE POINT (``bb_release_state``) — NOT the BB
    origin — so it is consistent with ``solve_throw_local`` (asserted by the
    round-trip test).  ``yaw/pitch/speed`` are actual launch quantities; apply any
    axis calibration (cmd→actual) before calling.

    Args:
        yaw_rad: actual launch azimuth, BB local frame (rad).
        pitch_rad: actual launch elevation (rad).
        speed_mps: actual launch speed (m/s).
        bb_position_mm: BB origin in world frame (mm) from calibration.
        yaw_offset_rad: BB-local-yaw → world rotation (from calibration).
        catch_height_mm: world z of the landing/catch plane; defaults to the
            platform catch height.
        yaw_s_offset_mm / pitch_d_offset_mm / release_l_mm: BB release geometry
            (defaults from config; pass the same values used by the solver).

    Returns ``ThrowPrediction`` (``initial_position`` is the release point), or
    None if the ball never reaches the catch plane.
    """
    if catch_height_mm is None:
        catch_height_mm = _DEFAULT_CATCH_HEIGHT_MM

    release, velocity = bb_release_state(
        yaw_rad, pitch_rad, speed_mps, bb_position_mm, yaw_offset_rad,
        yaw_s_offset_mm=yaw_s_offset_mm, pitch_d_offset_mm=pitch_d_offset_mm,
        release_l_mm=release_l_mm)
    vx, vy, vz = velocity

    # Vertical kinematics from the RELEASE point:  z(t) = rz + vz·t − ½g·t²
    rz = release[2]
    a_q = 0.5 * _G_MMPS2
    b_q = -vz
    c_q = catch_height_mm - rz

    discriminant = b_q * b_q - 4.0 * a_q * c_q
    if discriminant < 0:
        return None  # Ball never reaches catch plane

    sqrt_disc = math.sqrt(discriminant)
    t1 = (-b_q - sqrt_disc) / (2.0 * a_q)
    t2 = (-b_q + sqrt_disc) / (2.0 * a_q)
    tof = t2 if t2 > 0 else t1          # latest positive root = descending crossing
    if tof <= 0:
        return None

    return ThrowPrediction(
        initial_position=release,
        initial_velocity=velocity,
        landing_position=(release[0] + vx * tof, release[1] + vy * tof, catch_height_mm),
        landing_velocity=(vx, vy, vz - _G_MMPS2 * tof),
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


# Near-field guard constants (see _steepest_feasible_pitch): the maxima over
# [0, π/2] of  −cosφ·cos2φ  and  2·sinφ·cos²φ.
_NEAR_K1 = 2.0 / (3.0 * math.sqrt(6.0))
_NEAR_K2 = 4.0 / (3.0 * math.sqrt(3.0))


class _NoFeasiblePitch(Exception):
    """No pitch in range satisfies the speed + height limits."""


class _NearFieldTarget(Exception):
    """Target so close that the steepest throw is not the softest landing."""


def _pitch_eval(A: float, z_mm: float, l: float, d: float, g: float,
                pitch_rad: float) -> Optional[Tuple[float, float, float, float]]:
    """The unique ballistic arc through the target at one pitch.

    Returns ``(v, h_peak, h_vel, tof)`` in mm / s units, or None if no arc
    exists at this pitch (release point beyond the target, or the target above
    the launch line).  The release point moves with pitch: horizontal range
    ``R = A − l·cosφ + d``, and target height above release ``Δz = z − l·sinφ``
    (z is already in the pitch-axis frame — see ``solve_throw_local``).
    """
    cos_p = math.cos(pitch_rad)
    sin_p = math.sin(pitch_rad)
    if abs(cos_p) < 1e-6:
        return None
    R = A - l * cos_p + d
    if R <= 0:
        return None
    z_throw = z_mm - l * sin_p
    # Standard projectile: v² = g·R² / (R·sin(2φ) − Δz·(1+cos(2φ)))
    sin_2p = 2.0 * sin_p * cos_p
    cos_2p = cos_p * cos_p - sin_p * sin_p
    denom = R * sin_2p - z_throw * (1.0 + cos_2p)
    if denom <= 0:
        return None
    v = math.sqrt(g * R * R / denom)
    v_vert = v * sin_p
    h_peak = (v_vert * v_vert) / (2.0 * g) if v_vert > 0 else 0.0
    h_vel = v * cos_p
    return v, h_peak, h_vel, R / h_vel


def _steepest_feasible_pitch(
    A: float, z_mm: float, l: float, d: float,
    pitch_min_rad: float, pitch_max_rad: float,
    v_max: float, h_max: float, g: float,
) -> Tuple[float, float, float, float]:
    """Pitch minimising horizontal landing velocity, found without a search.

    Returns ``(pitch_rad, v_mmps, tof_s, h_peak_mm)``.  All lengths mm.

    Why no search is needed.  Horizontal velocity is ``R / tof``, and for a
    given release and target, time of flight grows monotonically with apex
    height — so the softest horizontal landing is simply the HIGHEST throw the
    limits allow, i.e. the steepest feasible pitch.  Both the apex
    ``h_peak(φ)`` and (above the minimum-energy angle) the speed ``v(φ)`` rise
    with pitch, so that pitch is the smallest of three upper bounds:

    * ``pitch_max``;
    * the apex cap — the peak is measured above the RELEASE point, so the cap
      fixes the vertical launch speed at exactly ``√(2·g·h_max)`` and the pitch
      follows in closed form; because the release point itself moves with pitch
      (``l``), that closed form is a fixed-point equation, solved by Newton
      from ``pitch_max`` (3–5 passes);
    * the speed cap's upper root (only binds at the edge of the range envelope;
      bisected between the minimum-energy angle and the bound above).

    This replaced a 0.5° grid sweep (2026-09-18): the result lies within one
    grid step ABOVE the old answer, never has a larger horizontal velocity, and
    lands on the target exactly.  ``tests/ros/test_throw_ballistics.py`` pins
    all three against the retired sweep, kept as a test oracle.

    Near field (owner decision 2026-09-18: refuse outright — no real target is
    that close).  Within a few hundred mm of the yaw axis the release point's
    travel is comparable to the range itself; a flatter barrel then parks the
    release point almost over the target and "steepest = softest" fails — the
    true optimum can even sit at an INTERIOR pitch, so comparing the two ends
    of the pitch range is not a sufficient guard (probed 2026-09-18).  The
    guard is instead a sufficient condition for the claim: with ``s, c`` the
    sine / cosine of pitch and ``v_h² = g·R² / (2·(R·tanφ − Δz))``,

        d(v_h²)/dφ < 0   ⇔   R² + R·l·c·cos2φ + 2·l·z·s·c² − 2·l²·s²·c² > 0

    and bounding each trig factor by its maximum over [0, π/2] gives

        R_lo² − K1·l·R_lo − K2·l·max(−z, 0) − l²/2 > 0,    R_lo = A + d − l·cos(pitch_min)

    (``R_lo`` is the shortest range any pitch sees).  It is conservative —
    at the default geometry it refuses A below ~235 mm for a level target and
    ~555 mm for a 1.5 m drop, against a true boundary near 290 mm — and it is
    one evaluation, not a sweep.  Fails → ``_NearFieldTarget``.
    """
    r_lo = A + d - l * math.cos(pitch_min_rad)
    if (r_lo <= 0.5 * _NEAR_K1 * l
            or r_lo * r_lo - _NEAR_K1 * l * r_lo
            - _NEAR_K2 * l * max(-z_mm, 0.0) - 0.5 * l * l <= 0):
        raise _NearFieldTarget()

    top = _pitch_eval(A, z_mm, l, d, g, pitch_max_rad)
    if top is None:
        # No arc even at the steepest pitch: every flatter pitch has a shorter
        # range and a lower launch line, so none exists there either.
        raise _NoFeasiblePitch()

    pitch = pitch_max_rad
    best = top

    if top[1] > h_max:
        # ---- Apex cap binds: vertical launch speed is pinned ----
        vz = math.sqrt(2.0 * g * h_max)
        # Unknown: the pitch whose cap-height arc (descending root) passes
        # through the target, i.e. the zero of
        #     r(φ) = φ − atan2(vz·tof(φ), R(φ)),   tof = (vz + √disc)/g,
        #     disc(φ) = vz² − 2·g·(z − l·sinφ).
        # r is convex on the descending-arrival set (−√disc is convex in φ) and
        # r(pitch_max) > 0, so Newton from pitch_max descends monotonically
        # onto the LARGEST root without overshooting it.  That matters within
        # millimetres of the cap, where the moving release point gives r two
        # roots (a window of feasible pitches) and plain fixed-point iteration
        # stops contracting (√disc has unbounded slope) — probed 2026-09-18.
        # Running out of domain (disc < 0) or of slope (r′ ≤ 0) before a zero
        # means r has none: the target sits above the apex cap.
        converged = False
        for _ in range(60):
            sin_p = math.sin(pitch)
            cos_p = math.cos(pitch)
            R = A - l * cos_p + d
            disc = vz * vz - 2.0 * g * (z_mm - l * sin_p)
            if disc <= 0:
                break
            root = math.sqrt(disc)
            Y = vz * (vz + root) / g
            resid = pitch - math.atan2(Y, R)
            dY = vz * l * cos_p / root
            slope = 1.0 - (R * dY - Y * l * sin_p) / (R * R + Y * Y)
            if slope <= 0:
                break
            step = resid / slope
            pitch -= step
            if abs(step) < 1e-13:
                converged = True
                break
        if not converged:
            raise _NoFeasiblePitch()      # target sits above the apex cap
        if pitch > pitch_max_rad:
            # Among DESCENDING arrivals the apex rises with pitch, so the cap
            # pitch can only exceed pitch_max if pitch_max itself reaches the
            # target still rising (a high, close target: over the cap at
            # pitch_max because the launch line nearly passes through it).
            # Every flatter pitch then arrives rising too — nothing lands.
            raise _NoFeasiblePitch()
        best = _pitch_eval(A, z_mm, l, d, g, pitch)
        # The arc is re-derived from the pitch so the landing is exact; back
        # off to the feasible side of the cap if rounding left it a hair over.
        for _ in range(8):
            if best is None or best[1] <= h_max:
                break
            pitch -= 1e-10
            best = _pitch_eval(A, z_mm, l, d, g, pitch)
        if best is None or best[1] > h_max:
            raise _NoFeasiblePitch()

    if best[0] > v_max:
        # ---- Speed cap binds: upper root of v(φ) = v_max ----
        # v(φ) is U-shaped with its minimum at the minimum-energy angle
        # (45° + half the line-of-sight elevation).  Settle that angle against
        # the moving release point, then bisect [φ_me, pitch] keeping ``lo``
        # on the feasible side.
        phi_me = pitch
        for _ in range(4):
            R = A - l * math.cos(phi_me) + d
            phi_me = 0.25 * math.pi + 0.5 * math.atan2(
                z_mm - l * math.sin(phi_me), R)
        lo = max(phi_me, pitch_min_rad)
        lo_eval = _pitch_eval(A, z_mm, l, d, g, lo) if lo < pitch else None
        if lo_eval is None or lo_eval[0] > v_max:
            raise _NoFeasiblePitch()
        hi = pitch
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            mid_eval = _pitch_eval(A, z_mm, l, d, g, mid)
            if mid_eval is not None and mid_eval[0] <= v_max:
                lo, lo_eval = mid, mid_eval
            else:
                hi = mid
        pitch, best = lo, lo_eval

    if pitch < pitch_min_rad:
        raise _NoFeasiblePitch()

    v, h_peak, _, tof = best
    # The ball must ARRIVE DESCENDING.  A high, close target can lie on the
    # rising limb of the only arc the limits allow — that is a fly-through, not
    # a landing, and ``predict_throw`` (descending crossing) would disagree
    # about where it comes down.  The steepest pitch has the longest flight, so
    # if it arrives rising every flatter pitch does too.  (The retired sweep
    # returned these; unreachable inside the default envelope, found 2026-09-18
    # by the pitch-max-bound oracle regime.)
    if v * math.sin(pitch) - g * tof >= 0:
        raise _NoFeasiblePitch()
    return pitch, v, tof, h_peak


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
    g_mps2: float = hw.GRAVITY_MPS2,
) -> ThrowSolution:
    """Solve BB throw parameters to land at ``(x, y, z)`` in BB local frame.

    Analytic INVERSE of ``bb_release_state`` (the forward model); their
    consistency is asserted by ``test_predict_throw_round_trips_with_solver`` —
    keep the two in lock-step (this pairing is what prevents the forward/inverse
    drift that the point-launch ``predict_throw`` once had).

    BB kinematics: yaw axis → pitch axis (offset ``d`` along x-local,
    ``pitch_z_offset`` above the yaw axis) → linear axis → release
    point (offset ``s`` lateral, ``l`` along the linear axis).  The yaw
    solution is independent of pitch / l / d (it's a 2-D ground-plane
    geometry).  Pitch minimises horizontal landing velocity (= the most-
    vertical landing) within the speed, height and pitch limits — which is
    the STEEPEST feasible pitch, found directly by ``_steepest_feasible_pitch``
    (no search; see its docstring for why, and for the near-field exception
    that is refused rather than solved).

    Frame convention (load-bearing — read before changing z kinematics).
    ``bb_calibration.py`` adds ``pitch_z_offset_mm`` to the marker-derived
    avg_z when computing ``bb_mocap_position.z``.  The result is a mixed-
    frame origin: ``(yaw_axis_x, yaw_axis_y, pitch_axis_z)``.  When
    ``global_to_bb_local`` subtracts ``bb_mocap_position`` from a world
    target, the resulting z is **already in pitch-axis frame** — the
    release point's z above the input z=0 plane is just ``l*sin(pitch)``,
    NOT ``pitch_z_offset + l*sin(pitch)``.  Adding pitch_z_offset here
    would double-count it.  (For historical context: an earlier version
    of this solver did add pitch_z_offset, which biased throws by 17.5 mm
    vertically — caught when re-checking the working 2026-02-17 model.)

    All distance arguments are mm; angle arguments are degrees.  Mixing
    units keeps the call site readable against ``hardware_config`` defaults.

    Raises ``ValueError`` if no feasible solution exists.  The message names
    the binding constraint so the caller can surface it (e.g. "yaw out of
    range", "no pitch satisfies speed+height limits", "target inside
    s-offset circle", "target too close").
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

    # ---- Degenerate: target directly above the yaw axis ----
    # (directly BELOW is a near-field target; _steepest_feasible_pitch refuses it)
    if A <= 1e-9 and z_mm >= 0:
        raise ValueError(
            "Target is directly above BB origin — cannot reach with "
            "projectile motion.")

    # ---- Pitch + speed: the steepest feasible throw (no search) ----
    try:
        pitch_rad, v_mmps, tof_s, h_peak_mm = _steepest_feasible_pitch(
            A, z_mm, l, d, pitch_min_rad, pitch_max_rad,
            v_max_mmps, h_max_mm, g_mmps2)
    except _NearFieldTarget:
        raise ValueError(
            f"Target too close to BB (A={A:.0f}, z={z_mm:.0f} mm): in the near "
            f"field the release point's own travel (l={l:.0f} mm) dominates the "
            f"range and the steepest throw is no longer the softest landing. "
            f"Near-field targets are refused by design.")
    except _NoFeasiblePitch:
        raise ValueError(
            f"No feasible trajectory for target (A={A:.0f}, z={z_mm:.0f} mm). "
            f"Limits: v_max={max_speed_mps:.2f} m/s, "
            f"h_max={max_height_mm:.0f} mm, "
            f"pitch [{pitch_min_deg:.0f}°, {pitch_max_deg:.0f}°].")

    return ThrowSolution(
        yaw_rad=yaw_rad,
        pitch_rad=pitch_rad,
        speed_mps=v_mmps / 1000.0,
        tof_s=tof_s,
        peak_height_mm=h_peak_mm,
    )
