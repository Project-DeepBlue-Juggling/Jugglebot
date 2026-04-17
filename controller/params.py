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
        0.025, 0.025, 0.025, 0.025, 0.025,  # fine:   5 × 25 ms  = 125 ms
        0.25, 0.25, 0.25, 0.25, 0.25,       # coarse: 5 × 250 ms = 1250 ms
    )                                         # total: 10 steps, 1.375 s horizon


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
    # The Phase 4.3 calibration (2026-04-01, commit 1aaf9b7) measured tau=65ms,
    # but the calibration run had a latent bug: the HardwarePlant's ZMQ SUB
    # set CONFLATE after connect (silently ignored), so get_state() returned
    # telemetry drained one-message-per-poll from a 500 Hz FIFO backlog with
    # hundreds of ms of lag.  That calibration therefore captured
    # actuator_tau + fifo_latency, not the actuator alone.  With the drain-to-
    # latest fix (2026-04-16, commit TBD) feedback latency is ~2ms, so the
    # true actuator tau is closer to the pre-calibration default of ~30ms.
    # Sim is model-consistent at either value (MuJoCo has no hidden lag),
    # but on hardware tau=65ms under-predicts actuator response, causing MPC
    # over-correction and ~8.8 Hz limit-cycle oscillation.
    tau: float = 0.040

    # ---- Tracking cost --------------------------------------------------
    Q_pos: float = 2.0       # position tracking weight (per mm²)
    Q_ori: float = 1000.0    # orientation tracking weight (per rad²)
    Qf_pos: float = 50.0     # terminal position weight
    Qf_ori: float = 5000.0   # terminal orientation weight

    # Velocity tracking (finite-difference twist vs reference twist)
    # These are the default weights used for flat-reference callers
    # (spacemouse, keyboard, catch coordinator).  Conservative to avoid
    # penalizing motion toward the target (twist_ref=0 on flat path).
    Q_vel_lin: float = 0.001   # linear velocity (per (mm/s)²)
    Q_vel_ang: float = 0.1     # angular velocity (per (rad/s)²)

    # Terminal velocity tracking — default weights for flat reference.
    Qf_vel_lin: float = 0.001  # terminal linear velocity weight
    Qf_vel_ang: float = 0.1    # terminal angular velocity weight

    # Event-boosted velocity weights (used when ref_events is provided).
    # With event-based references, twist_ref is kinematically consistent
    # with the position reference, so higher weights cooperate rather than
    # fight.  These are injected as NLP parameters per-solve.
    Q_vel_lin_events: float = 0.05    # boosted linear velocity weight
    Q_vel_ang_events: float = 0.1     # boosted angular velocity weight
    Qf_vel_lin_events: float = 1.0    # boosted terminal linear velocity
    Qf_vel_ang_events: float = 0.1    # boosted terminal angular velocity

    # Cartesian acceleration tracking (penalises finite-difference accel vs
    # reference accel from quintic interpolation).  Defaults to zero (opt-in)
    # to preserve existing behavior.  Suggested starting values when enabled:
    # Q_accel_lin=0.0005, Q_accel_ang=0.05.
    Q_accel_lin: float = 0.0          # linear accel (per (mm/s²)²)
    Q_accel_ang: float = 0.0          # angular accel (per (rad/s²)²)

    # ---- Control cost ---------------------------------------------------
    R: float = 1e-4          # control effort — small regulariser
    S: float = 0.05          # control smoothness (penalises Δu rate, dt-normalised)
    A: float = 0.01          # acceleration smoothness (penalises ΔΔu rate, dt-normalised)

    # ---- Constraints ----------------------------------------------------
    stroke_mm: float = 280.0
    stroke_margin_mm: float = 5.0    # safety margin from each end of stroke range
    max_leg_vel_mmps: float = 140.0  # mm/s per leg �� raised from 70 after
    # quintic reference trajectory shaping (2026-04-17).  The S-curve
    # reference prevents the aggressive accelerations that drove 2.7G peak
    # at v_max=280.  Raise further toward 280–700 once validated on hardware.

    # ---- IPOPT options --------------------------------------------------
    max_iter: int = 200
    max_cpu_time: float = 0.018  # 18 ms (72% of budget, leaves headroom for overhead)
    tol: float = 1e-4
    warm_start: bool = True
    print_level: int = 0        # 0 = silent

    # ---- Failure handling -----------------------------------------------
    max_consecutive_failures: int = 3

    # ---- Solver priming -------------------------------------------------
    # If True, MPCController.__init__ invokes the solver once with neutral
    # parameters after NLP construction.  This amortises CasADi's first-
    # call warmup (function tables, linear-solver symbolic factorisation,
    # page faults, etc.) into startup time, so the first operator-visible
    # control-loop solve does not eat the ~27-100ms cold-start penalty.
    # Set False in CI/tests to avoid the ~50-150ms construction overhead
    # when the solver will not actually be driven in real time.
    prime_solver: bool = True

    # ---- AOT solver loading ---------------------------------------------
    # If True, try to load the ahead-of-time-compiled NLP from
    # ``controller/generated/mpc_gen.so`` when present.  Set False for
    # MPC instances that intentionally use a non-default NLP structure
    # (e.g. ``sim/hand/feasibility.py``'s coarse-horizon checker): the
    # main AOT binary is built for the production horizon and would
    # (correctly) hash-mismatch against a one-off variant.
    use_aot_solver: bool = True

    def __post_init__(self):
        # Coerce to tuple so cached_property on cumulative_times is safe
        if not isinstance(self.dt_schedule, tuple):
            object.__setattr__(self, 'dt_schedule', tuple(self.dt_schedule))
        if any(dt <= 0 for dt in self.dt_schedule):
            raise ValueError(
                f"dt_schedule must contain only positive values, "
                f"got {self.dt_schedule}"
            )
