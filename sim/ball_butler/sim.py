"""Ball Butler simulation — ballistic spawner for throw-catch testing.

``BallButlerSim`` models Ball Butler as a pure-kinematic throw source.
Given a world-frame target, it computes the release point and velocity
using the same IK + ballistics as the real Ball Butler firmware, then
returns a ``BallSpawn`` that the sim's catch pipeline consumes directly.

No MuJoCo bodies are created — BB exists only as a computation.

The ballistics solver is ported from the production code in the retired
``ball_butler_node.py`` (now ``attic/ros-jugglebot-archived/``)
(``compute_command_for_target`` and ``global_to_bb_frame``).
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

import numpy as np
import yaml

import sys

from sim.hand.coordinator import BallSpawn

# Gravity in mm/s² (same as hand.ballistics)
_GRAVITY_MMS2 = 9806.0


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class BallButlerConfig:
    """Ball Butler geometry, limits, and world-frame placement.

    Geometry fields come from ``hardware_config.yaml`` (sections
    ``ball_butler_geometry`` and ``ball_butler_operational``).
    Position and yaw offset are caller-supplied (determined by
    physical mounting / mocap calibration in production).
    """
    # World-frame placement
    position_mm: np.ndarray          # [x, y, z] BB yaw axis origin
    yaw_offset_rad: float            # rotation from world +X to BB local +X

    # Serial-chain geometry (from ball_butler_geometry)
    yaw_s_offset_mm: float           # ball centroid off-axis offset
    pitch_d_offset_mm: float         # pitch axis behind BB origin
    release_l_position_mm: float     # release point along arm
    pitch_z_offset_mm: float         # pitch axis above BB origin

    # Joint limits
    pitch_min_deg: float
    pitch_max_deg: float
    yaw_max_deg: float               # local frame, min is 0

    # Throw limits
    max_throw_speed_mps: float
    max_throw_height_m: float


def _load_hardware_config() -> dict:
    """Load hardware_config.yaml from standard locations."""
    candidates = [
        os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'hardware_config.yaml'),
        os.path.join('config', 'hardware_config.yaml'),
        os.path.join('..', 'config', 'hardware_config.yaml'),
    ]
    for path in candidates:
        if os.path.exists(path):
            with open(path) as f:
                return yaml.safe_load(f)
    raise FileNotFoundError("Cannot find config/hardware_config.yaml")


def _config_from_yaml(
    position_mm: np.ndarray,
    yaw_offset_rad: float,
) -> BallButlerConfig:
    """Build a BallButlerConfig from hardware_config.yaml + caller position."""
    hw = _load_hardware_config()
    geom = hw['ball_butler_geometry']
    ops = hw['ball_butler_operational']

    return BallButlerConfig(
        position_mm=np.asarray(position_mm, dtype=float),
        yaw_offset_rad=float(yaw_offset_rad),
        yaw_s_offset_mm=geom['yaw_s_offset_mm'],
        pitch_d_offset_mm=geom['pitch_d_offset_mm'],
        release_l_position_mm=geom['release_l_position_mm'],
        pitch_z_offset_mm=geom['pitch_z_offset_mm'],
        pitch_min_deg=geom['pitch_min_deg'],
        pitch_max_deg=geom['pitch_max_deg'],
        yaw_max_deg=geom['yaw_max_deg'],
        max_throw_speed_mps=ops['max_throw_speed_mps'],
        max_throw_height_m=ops['max_throw_height_m'],
    )


# ---------------------------------------------------------------------------
# Yaw solver (ported from ball_butler_node.py)
# ---------------------------------------------------------------------------

def _wrap_pi(angle: float) -> float:
    """Wrap angle to (-pi, pi]."""
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return a if a != -math.pi else math.pi


def _yaw_solve(x: float, y: float, s: float) -> float:
    """Solve for yaw angle given target (x, y) and off-axis offset s.

    Returns the yaw angle in radians (choosing the solution with smallest
    absolute value).  Raises ``ValueError`` if no solution exists.
    """
    hyp = math.hypot(x, y)
    r_sq = x * x + y * y - s * s
    if r_sq < 0.0 or hyp == 0.0:
        raise ValueError(
            f"No yaw solution for target (x={x:.0f}, y={y:.0f} mm) "
            f"with s={s:.1f} mm")
    base = math.atan2(y, x)
    s_over_hyp = max(-1.0, min(1.0, s / hyp))
    delta = math.asin(s_over_hyp)
    t1 = _wrap_pi(base - delta)
    t2 = _wrap_pi(base - math.pi + delta)
    return t1 if abs(t1) <= abs(t2) else t2


# ---------------------------------------------------------------------------
# Steepest-feasible-pitch core — a VERBATIM twin of the block of the same names
# in ros_ws/src/jugglebot/jugglebot/can/throw_ballistics.py (sim has no ROS
# package dependency, so it is copied, not imported).  Edit both together:
# tests/sim/test_ball_butler_sim.py pins the two to bit-identical output.
# ---------------------------------------------------------------------------

# Near-field guard constants (see _steepest_feasible_pitch): the maxima over
# [0, π/2] of  −cosφ·cos2φ  and  2·sinφ·cos²φ.
_NEAR_K1 = 2.0 / (3.0 * math.sqrt(6.0))
_NEAR_K2 = 4.0 / (3.0 * math.sqrt(3.0))


class _NoFeasiblePitch(Exception):
    """No pitch in range satisfies the speed + height limits."""


class _NearFieldTarget(Exception):
    """Target so close that the steepest throw is not the softest landing."""


def _pitch_eval(A: float, z_mm: float, l: float, d: float, g: float,
                pitch_rad: float) -> tuple[float, float, float, float] | None:
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
) -> tuple[float, float, float, float]:
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


# ---------------------------------------------------------------------------
# Ballistics solver (ported from ball_butler_node.py compute_command_for_target)
# ---------------------------------------------------------------------------

def _solve_throw(
    x_bb: float, y_bb: float, z_bb: float,
    cfg: BallButlerConfig,
) -> tuple[float, float, float, float]:
    """Compute (yaw_rad, pitch_rad, speed_mms, time_of_flight_s) for a target
    in BB's local frame (mm).

    Minimises horizontal landing velocity subject to speed, height, and
    pitch constraints — the steepest feasible pitch, found without a sweep.
    Raises ``ValueError`` if no feasible solution or the target is near-field.
    """
    s = cfg.yaw_s_offset_mm
    d = cfg.pitch_d_offset_mm
    l = cfg.release_l_position_mm
    v_max = cfg.max_throw_speed_mps * 1000.0  # mm/s
    h_max = cfg.max_throw_height_m * 1000.0    # mm
    g = _GRAVITY_MMS2

    # Yaw (geometry-only, independent of pitch)
    yaw = _yaw_solve(x_bb, y_bb, s)
    yaw_max_rad = math.radians(cfg.yaw_max_deg)
    if yaw < 0 or yaw > yaw_max_rad:
        raise ValueError(
            f"Yaw {math.degrees(yaw):.1f}° outside limits "
            f"[0°, {cfg.yaw_max_deg:.1f}°]")

    # Horizontal arm-line distance
    A_sq = x_bb * x_bb + y_bb * y_bb - s * s
    if A_sq < 0:
        raise ValueError(
            f"Target ({x_bb:.0f}, {y_bb:.0f} mm) is inside the "
            f"s-offset circle (s={s:.1f} mm)")
    A = math.sqrt(A_sq)

    # Special case: directly above (directly BELOW is a near-field target,
    # refused by _steepest_feasible_pitch)
    if A <= 1e-9 and z_bb >= 0:
        raise ValueError("Target directly above launch point")

    # Steepest feasible pitch = minimum horizontal landing velocity (no sweep)
    try:
        pitch, speed, t, _h_peak = _steepest_feasible_pitch(
            A, z_bb, l, d,
            math.radians(cfg.pitch_min_deg), math.radians(cfg.pitch_max_deg),
            v_max, h_max, g)
    except _NearFieldTarget:
        raise ValueError(
            f"Target too close to BB (A={A:.0f} mm, z={z_bb:.0f} mm): "
            f"near-field targets are refused by design")
    except _NoFeasiblePitch:
        raise ValueError(
            f"No valid trajectory for target (A={A:.0f} mm, z={z_bb:.0f} mm). "
            f"v_max={v_max/1000:.1f} m/s, h_max={h_max/1000:.1f} m, "
            f"pitch=[{cfg.pitch_min_deg:.0f}°, {cfg.pitch_max_deg:.0f}°]")

    return yaw, pitch, speed, t


# ---------------------------------------------------------------------------
# BallButlerSim
# ---------------------------------------------------------------------------

class BallButlerSim:
    """Simulated Ball Butler — computes throw release states.

    Placed at a fixed world-frame position, uses BB's real kinematics
    to compute where and how fast the ball leaves the mechanism.

    Usage::

        bb = BallButlerSim.from_hardware_config(
            position_mm=[40, -80, 1500],
            yaw_offset_rad=0.0,
        )
        spawn = bb.throw_at_jugglebot(spawn_time=0.5)
        plant.spawn_ball(spawn.position_mm, spawn.velocity_mms)
    """

    def __init__(self, config: BallButlerConfig) -> None:
        self._cfg = config

    @classmethod
    def from_hardware_config(
        cls,
        position_mm: np.ndarray | list[float],
        yaw_offset_rad: float = 0.0,
    ) -> BallButlerSim:
        """Create from hardware_config.yaml.  Caller provides position."""
        cfg = _config_from_yaml(np.asarray(position_mm, dtype=float), yaw_offset_rad)
        return cls(cfg)

    @property
    def config(self) -> BallButlerConfig:
        return self._cfg

    # -- Coordinate transforms ------------------------------------------------

    def _world_to_local(self, point_mm: np.ndarray) -> np.ndarray:
        """World frame → BB local frame."""
        d = point_mm - self._cfg.position_mm
        c = math.cos(self._cfg.yaw_offset_rad)
        s = math.sin(self._cfg.yaw_offset_rad)
        return np.array([
            d[0] * c + d[1] * s,
            -d[0] * s + d[1] * c,
            d[2],
        ])

    def _local_to_world(self, point_mm: np.ndarray) -> np.ndarray:
        """BB local frame → world frame."""
        c = math.cos(self._cfg.yaw_offset_rad)
        s = math.sin(self._cfg.yaw_offset_rad)
        rotated = np.array([
            point_mm[0] * c - point_mm[1] * s,
            point_mm[0] * s + point_mm[1] * c,
            point_mm[2],
        ])
        return rotated + self._cfg.position_mm

    def _local_to_world_dir(self, dir_mm: np.ndarray) -> np.ndarray:
        """Rotate a direction vector from BB local → world (no translation)."""
        c = math.cos(self._cfg.yaw_offset_rad)
        s = math.sin(self._cfg.yaw_offset_rad)
        return np.array([
            dir_mm[0] * c - dir_mm[1] * s,
            dir_mm[0] * s + dir_mm[1] * c,
            dir_mm[2],
        ])

    # -- Core API -------------------------------------------------------------

    def compute_release_state(
        self, target_mm: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """Compute ball release state for a throw aimed at *target_mm* (world frame).

        Returns
        -------
        release_pos_mm : (3,) world-frame release position
        release_vel_mms : (3,) world-frame release velocity
        time_of_flight_s : float
        """
        target_local = self._world_to_local(target_mm)
        yaw, pitch, speed, tof = _solve_throw(
            target_local[0], target_local[1], target_local[2], self._cfg)

        cfg = self._cfg
        l = cfg.release_l_position_mm
        d = cfg.pitch_d_offset_mm
        s = cfg.yaw_s_offset_mm

        cos_p = math.cos(pitch)
        sin_p = math.sin(pitch)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        # Release position in BB local frame
        # x_rel is the horizontal distance along the arm direction from yaw axis
        x_rel = l * cos_p - d
        release_local = np.array([
            x_rel * cos_y - s * sin_y,
            x_rel * sin_y + s * cos_y,
            cfg.pitch_z_offset_mm + l * sin_p,
        ])

        # Release velocity direction: throw speed at pitch angle, in yaw direction
        vel_local = speed * np.array([
            cos_p * cos_y,
            cos_p * sin_y,
            sin_p,
        ])

        release_world = self._local_to_world(release_local)
        vel_world = self._local_to_world_dir(vel_local)

        return release_world, vel_world, tof

    def throw_at(
        self,
        target_mm: np.ndarray,
        spawn_time: float,
    ) -> BallSpawn:
        """Compute a ``BallSpawn`` for a throw aimed at *target_mm* (world frame).

        Parameters
        ----------
        target_mm : (3,) target position in world frame (mm)
        spawn_time : absolute sim time at which the ball is released
        """
        release_pos, release_vel, _ = self.compute_release_state(
            np.asarray(target_mm, dtype=float))
        return BallSpawn(
            position_mm=release_pos,
            velocity_mms=release_vel,
            spawn_time=spawn_time,
        )

    def throw_at_jugglebot(
        self,
        spawn_time: float,
        landing_xy_mm: np.ndarray | None = None,
        scatter_mm: float = 0.0,
        catch_z_mm: float = 783.5,
        rng: np.random.Generator | None = None,
    ) -> BallSpawn:
        """Throw at Jugglebot's catch height (convenience).

        Parameters
        ----------
        spawn_time : absolute sim time at which ball is released
        landing_xy_mm : (2,) optional lateral aim offset from centre (mm)
        scatter_mm : gaussian scatter sigma applied to landing XY (mm)
        catch_z_mm : world-frame Z of catch height.  Default is
            574.3 (platform_height) + 80.0 (active_z) + 129.2 (hand_catch_offset).
        rng : optional seeded ``np.random.Generator`` for reproducible
            scatter. When ``None`` a fresh unseeded generator is used (so
            ``scatter_mm > 0`` runs are non-deterministic) — pass a seeded
            generator (e.g. from the demo runner) for reproducibility.
        """
        xy = np.zeros(2)
        if landing_xy_mm is not None:
            xy = np.asarray(landing_xy_mm, dtype=float)
        if scatter_mm > 0:
            gen = rng if rng is not None else np.random.default_rng()
            xy = xy + gen.normal(0.0, scatter_mm, size=2)

        target = np.array([xy[0], xy[1], catch_z_mm])
        return self.throw_at(target, spawn_time)
