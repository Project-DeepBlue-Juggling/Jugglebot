"""MPC tuning parameters.

All values use the simulation's native units: mm, rad, seconds.
"""

from __future__ import annotations

import functools
from dataclasses import dataclass, field
from typing import Tuple

import numpy as np


def _default_dt_schedule() -> Tuple[float, ...]:
    """Default variable-resolution horizon: 5 fine + 5 coarse steps."""
    return (
        0.02, 0.02, 0.02, 0.02, 0.02,   # fine:   5 × 20 ms  = 100 ms
        0.25, 0.25, 0.25, 0.25, 0.25,    # coarse: 5 × 250 ms = 1250 ms
    )                                      # total: 10 steps, 1.35 s horizon


@dataclass
class MPCParams:
    """Tunable parameters for the NMPC controller.

    Defaults are reasonable starting points for Jugglebot's workspace.
    Weights were chosen so that 1 mm position error ≈ 1° orientation error
    in the cost function (Q_ori ≈ Q_pos / 0.017²).
    """

    # ---- Horizon --------------------------------------------------------
    dt_schedule: Tuple[float, ...] = field(default_factory=_default_dt_schedule)

    @property
    def N(self) -> int:
        """Number of prediction steps."""
        return len(self.dt_schedule)

    @property
    def dt_fine(self) -> float:
        """Finest timestep — used as the control loop rate."""
        return self.dt_schedule[0]

    @property
    def horizon_s(self) -> float:
        """Total horizon duration in seconds."""
        return sum(self.dt_schedule)

    @functools.cached_property
    def cumulative_times(self) -> np.ndarray:
        """(N+1,) cumulative time at each horizon node, starting at 0.

        Cached on first access (safe because dt_schedule is an immutable tuple).
        """
        result = np.concatenate(([0.0], np.cumsum(self.dt_schedule)))
        result.flags.writeable = False
        return result

    @staticmethod
    def uniform_schedule(N: int = 10, dt: float = 0.02) -> Tuple[float, ...]:
        """Build a uniform timestep schedule (convenience for tests/feasibility)."""
        return tuple(dt for _ in range(N))

    # ---- Actuator model -------------------------------------------------
    tau: float = 0.03        # first-order lag time constant (s)

    # ---- Tracking cost --------------------------------------------------
    Q_pos: float = 10.0      # position tracking weight (per mm²)
    Q_ori: float = 1000.0    # orientation tracking weight (per rad²)
    Qf_pos: float = 50.0     # terminal position weight
    Qf_ori: float = 5000.0   # terminal orientation weight

    # Velocity tracking (finite-difference twist vs reference twist)
    # At these weights: 1mm position error ≈ 100 mm/s velocity error in cost.
    # Higher values degrade position tracking without improving velocity
    # (actuator lag τ is the bottleneck, not the cost weight).
    Q_vel_lin: float = 0.001   # linear velocity (per (mm/s)²)
    Q_vel_ang: float = 0.1     # angular velocity (per (rad/s)²)

    # ---- Arrival urgency ------------------------------------------------
    urgency_ramp_s: float = 0.5   # ramp window: tracking weight ramps base→max
                                   # over this many seconds before the deadline
    urgency_base: float = 0.05    # base multiplier for timed targets (nodes far
                                   # from deadline).  Low value lets the MPC choose
                                   # its own path without penalty for being far from
                                   # the target; terminal cost still provides pull.
                                   # ASAP mode always uses 1.0.
    urgency_max: float = 10.0     # peak multiplier at/past the deadline

    # ---- Control cost ---------------------------------------------------
    R: float = 1e-4          # control effort — small regulariser
    S: float = 0.02          # control smoothness (penalises Δu rate, dt-normalised)
    A: float = 0.004         # acceleration smoothness (penalises ΔΔu rate, dt-normalised)

    # ---- Constraints ----------------------------------------------------
    stroke_mm: float = 280.0
    stroke_margin_mm: float = 5.0    # safety margin from each end of stroke range
    max_leg_vel_mmps: float = 300.0  # mm/s per leg (hardware max ~1060)

    # ---- IPOPT options --------------------------------------------------
    max_iter: int = 200
    max_cpu_time: float = 0.018  # 18 ms (90 % of 20 ms budget)
    tol: float = 1e-4
    warm_start: bool = True
    print_level: int = 0        # 0 = silent

    # ---- Failure handling -----------------------------------------------
    max_consecutive_failures: int = 10
