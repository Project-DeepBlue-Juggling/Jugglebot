"""Geometry and tempo specification for the two-ball oval juggling pattern.

A ``JugglePattern`` captures the small set of primary parameters that define
the demo's pattern (throw height, column separation, platform working
height, oval width) and derives from them the throw/catch keyframe poses and
the periodic timing the trajectory layer needs.

The derived timing reproduces the Phase 1 feasibility study in
``plans/active/bb-led-two-ball-juggle-demo.md`` — see that document for the
derivation of the hand stroke-duration coefficients below.

Pure NumPy — no ROS2, no CasADi, no hardware.
"""
from __future__ import annotations

import dataclasses
import math

import numpy as np

GRAVITY_MPS2 = 9.806

# Hand throw/catch stroke-duration coefficients, units (s . m/s):
#     t_throw = THROW_DUR_COEF / v ,   t_catch = CATCH_DUR_COEF / v
# where v is the ball release speed (m/s). Derived in the Phase 1 feasibility
# study from Trajectory.h with the hardware_config `teensy_trajectory`
# constants: effective stroke 0.315 m, inertia_ratio 0.747,
# throw_vel_hold_pct 0.05, catch_vel_hold_pct 0.10, catch_vel_ratio 0.6.
#
# NOTE: catch_vel_ratio 0.6 is the hardware_config value; sim/hand/trajectory.py
# currently uses 0.9. That discrepancy is tracked in both plan documents and
# owned by the hand-generator side-quest; if it resolves to 0.9, CATCH_DUR_COEF
# must be recomputed (a faster catch -> smaller coefficient).
THROW_DUR_COEF = 0.614
CATCH_DUR_COEF = 0.998


@dataclasses.dataclass(frozen=True)
class JugglePattern:
    """Primary parameters of the two-ball oval juggling pattern.

    Coordinates are platform-pose units: millimetres and radians, with the
    pose 6-vector ordered ``[x, y, z, rx, ry, rz]`` as an offset from STOW.
    """

    apex_height_mm: float = 1300.0   # ball apex above the hand at release
    separation_mm: float = 100.0     # THROW <-> CATCH horizontal distance
    active_z_mm: float = 170.0       # platform working height (offset from STOW)
    oval_width_mm: float = 30.0      # lateral (y) semi-axis of the oval

    def __post_init__(self) -> None:
        if self.apex_height_mm <= 0:
            raise ValueError("apex_height_mm must be positive")
        if self.separation_mm <= 0:
            raise ValueError("separation_mm must be positive")
        if self.transit_window_s <= 0.0:
            raise ValueError(
                "infeasible tempo: transit window <= 0 at apex "
                f"{self.apex_height_mm:.0f} mm — throw higher "
                "(see Phase 1 feasibility study)")

    # ---- ballistics --------------------------------------------------
    @property
    def throw_speed_mps(self) -> float:
        """Ball vertical launch speed for the apex: ``v = sqrt(2 g h)``."""
        return math.sqrt(2.0 * GRAVITY_MPS2 * self.apex_height_mm / 1000.0)

    @property
    def flight_time_s(self) -> float:
        """Symmetric flight time (throw height == catch height)."""
        return 2.0 * self.throw_speed_mps / GRAVITY_MPS2

    # ---- hand stroke timing -----------------------------------------
    @property
    def throw_stroke_s(self) -> float:
        return THROW_DUR_COEF / self.throw_speed_mps

    @property
    def catch_stroke_s(self) -> float:
        return CATCH_DUR_COEF / self.throw_speed_mps

    # ---- tempo -------------------------------------------------------
    @property
    def transit_window_s(self) -> float:
        """Hand-idle window for one THROW<->CATCH platform transit (Phase 1).

        ``(t_flight - t_throw - 1.5 t_catch) / 2``. Must be positive for the
        two-ball pattern to close.
        """
        return 0.5 * (self.flight_time_s
                      - self.throw_stroke_s
                      - 1.5 * self.catch_stroke_s)

    @property
    def ball_period_s(self) -> float:
        """One ball's full cycle: 2 throws + 2 catches + 2 transit windows."""
        return 2.0 * (self.throw_stroke_s + self.catch_stroke_s
                      + self.transit_window_s)

    @property
    def platform_period_s(self) -> float:
        """Platform oval-loop period.

        Half the ball period: per ball period the platform traces the oval
        (THROW -> CATCH -> THROW) twice, so the platform pose trajectory is
        periodic with this shorter period.
        """
        return 0.5 * self.ball_period_s

    # ---- keyframes ---------------------------------------------------
    @property
    def throw_pose(self) -> np.ndarray:
        """Level platform pose at the THROW point — ``[x,y,z,rx,ry,rz]``."""
        return np.array([0.5 * self.separation_mm, 0.0, self.active_z_mm,
                         0.0, 0.0, 0.0])

    @property
    def catch_pose(self) -> np.ndarray:
        """Level platform pose at the CATCH point — ``[x,y,z,rx,ry,rz]``."""
        return np.array([-0.5 * self.separation_mm, 0.0, self.active_z_mm,
                         0.0, 0.0, 0.0])
