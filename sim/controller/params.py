"""MPC tuning parameters.

All values use the simulation's native units: mm, rad, seconds.
"""

from dataclasses import dataclass


@dataclass
class MPCParams:
    """Tunable parameters for the NMPC controller.

    Defaults are reasonable starting points for Jugglebot's workspace.
    Weights were chosen so that 1 mm position error ≈ 1° orientation error
    in the cost function (Q_ori ≈ Q_pos / 0.017²).
    """

    # ---- Horizon --------------------------------------------------------
    N: int = 10              # prediction steps (at 50 Hz → 200 ms lookahead)
    dt: float = 0.02         # control timestep (s)

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

    # ---- Control cost ---------------------------------------------------
    R: float = 1e-4          # control effort — small regulariser
    S: float = 1.0           # control smoothness (penalises Δu per step)
    A: float = 0.2           # acceleration smoothness (penalises ΔΔu per step)

    # ---- Constraints ----------------------------------------------------
    stroke_mm: float = 280.0
    max_leg_vel_mmps: float = 300.0  # mm/s per leg (hardware max ~1060)

    # ---- IPOPT options --------------------------------------------------
    max_iter: int = 200
    max_cpu_time: float = 0.018  # 18 ms (90 % of 20 ms budget)
    tol: float = 1e-4
    warm_start: bool = True
    print_level: int = 0        # 0 = silent

    # ---- Failure handling -----------------------------------------------
    max_consecutive_failures: int = 10
