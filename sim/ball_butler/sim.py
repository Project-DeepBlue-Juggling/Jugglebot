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
_sim_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)

from hand.coordinator import BallSpawn

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
# Ballistics solver (ported from ball_butler_node.py compute_command_for_target)
# ---------------------------------------------------------------------------

def _solve_throw(
    x_bb: float, y_bb: float, z_bb: float,
    cfg: BallButlerConfig,
) -> tuple[float, float, float, float]:
    """Compute (yaw_rad, pitch_rad, speed_mms, time_of_flight_s) for a target
    in BB's local frame (mm).

    Minimises horizontal landing velocity subject to speed, height, and
    pitch constraints.  Raises ``ValueError`` if no feasible solution.
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

    # Special case: directly above/below
    if A <= 1e-9:
        if z_bb >= 0:
            raise ValueError("Target directly above launch point")
        v = math.sqrt(2 * g * (-z_bb))
        if v > v_max:
            raise ValueError(f"Target too far below (need {v/1000:.2f} m/s)")
        t = math.sqrt(-2 * z_bb / g)
        return yaw, -math.pi / 2, v, t

    # Sweep pitch to find minimum horizontal landing velocity
    pitch_min_rad = math.radians(cfg.pitch_min_deg)
    pitch_max_rad = math.radians(cfg.pitch_max_deg)
    n_steps = int((pitch_max_rad - pitch_min_rad) / math.radians(0.5)) + 1

    best_pitch = None
    best_speed = None
    best_h_vel = float('inf')

    for i in range(n_steps):
        pitch = pitch_min_rad + i * (pitch_max_rad - pitch_min_rad) / max(n_steps - 1, 1)
        cos_p = math.cos(pitch)
        sin_p = math.sin(pitch)

        if abs(cos_p) < 1e-6:
            continue

        # Throw range (horizontal distance from release to target)
        R = A - l * cos_p + d
        if R <= 0:
            continue

        # Throw height (vertical distance from release to target)
        z_throw = z_bb - l * sin_p

        # Projectile equation: v² = g·R² / (R·sin2φ − z_throw·(1+cos2φ))
        sin_2p = 2 * sin_p * cos_p
        cos_2p = cos_p * cos_p - sin_p * sin_p
        D = R * sin_2p - z_throw * (1 + cos_2p)
        if D <= 0:
            continue

        v_sq = g * R * R / D
        if v_sq <= 0:
            continue
        v = math.sqrt(v_sq)

        if v > v_max:
            continue

        # Peak height above release point
        v_vert = v * sin_p
        h_peak = (v_vert * v_vert / (2 * g)) if v_vert > 0 else 0.0
        if h_peak > h_max:
            continue

        h_vel = v * cos_p
        if h_vel < best_h_vel:
            best_h_vel = h_vel
            best_pitch = pitch
            best_speed = v

    if best_pitch is None:
        raise ValueError(
            f"No valid trajectory for target (A={A:.0f} mm, z={z_bb:.0f} mm). "
            f"v_max={v_max/1000:.1f} m/s, h_max={h_max/1000:.1f} m, "
            f"pitch=[{cfg.pitch_min_deg:.0f}°, {cfg.pitch_max_deg:.0f}°]")

    R_final = A - l * math.cos(best_pitch) + d
    t = R_final / (best_speed * math.cos(best_pitch))

    return yaw, best_pitch, best_speed, t


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
