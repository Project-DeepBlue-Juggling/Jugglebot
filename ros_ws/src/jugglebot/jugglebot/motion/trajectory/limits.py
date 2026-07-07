"""Actuator / pose limits for the MVP trajectory generator.

``TrajectoryLimits`` bundles the leg-space kinematic ceilings (vel/acc/jerk) the
feasibility gate enforces, the fixed 40 Hz knot spacing, and the per-knot motor
step bound. It is constructed from the generated ``JB_TRAJ_*`` /
``JB_OP_MAX_POSITION_STEP_REV`` config constants so the single source of truth
stays ``config/hardware_config.yaml``.

Two tiers of leg limits:
  * the **session limit** (``leg_vel/acc/jerk``) — the currently-commanded ceiling,
    ramped up across hardware sessions (Phase 4) via ``trajectory/set_limits``;
  * the **hard ceiling** (``*_ceiling``) — the YAML-pinned absolute maximum a
    ``set_limits`` request is clamped to (a session limit can never exceed it).

Pure Python + numpy. No ROS2 / repo-root imports.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TrajectoryLimits:
    """Immutable leg-space limits + knot spacing for one plan/validate pass.

    All leg-space limits are in mm-extension units (mm/s, mm/s², mm/s³);
    the feasibility gate measures true sampled leg extensions/velocities via
    the IK Jacobian chain and compares against these.
    """
    leg_vel_mmps: float          # session velocity ceiling (mm/s)
    leg_acc_mmps2: float         # session acceleration ceiling (mm/s²)
    leg_jerk_mmps3: float        # session jerk ceiling (mm/s³) — the binding constraint
    leg_vel_ceiling_mmps: float  # YAML hard ceiling (mm/s)
    leg_acc_ceiling_mmps2: float
    leg_jerk_ceiling_mmps3: float
    knot_dt_s: float             # fixed 40 Hz knot spacing (0.025 s)
    max_step_rev: float          # per-knot |Δu0| motor bound (rev)
    min_move_duration_s: float
    min_timed_lead_s: float

    @staticmethod
    def from_config(hw, *,
                    leg_vel_mmps: float | None = None,
                    leg_acc_mmps2: float | None = None,
                    leg_jerk_mmps3: float | None = None) -> 'TrajectoryLimits':
        """Build from the generated ``hardware_config`` module (``JB_TRAJ_*``).

        Optional overrides replace the YAML session-default limits (used by the
        in-session ``trajectory/set_limits`` ramp); each override is clamped to
        the corresponding YAML hard ceiling so a runtime request can never
        exceed the pinned maximum.
        """
        v_ceil = float(hw.JB_TRAJ_LEG_VEL_CEILING_MMPS)
        a_ceil = float(hw.JB_TRAJ_LEG_ACC_CEILING_MMPS2)
        j_ceil = float(hw.JB_TRAJ_LEG_JERK_CEILING_MMPS3)

        v = float(hw.JB_TRAJ_LEG_VEL_LIMIT_MMPS if leg_vel_mmps is None
                  else leg_vel_mmps)
        a = float(hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2 if leg_acc_mmps2 is None
                  else leg_acc_mmps2)
        j = float(hw.JB_TRAJ_LEG_JERK_LIMIT_MMPS3 if leg_jerk_mmps3 is None
                  else leg_jerk_mmps3)

        return TrajectoryLimits(
            leg_vel_mmps=min(v, v_ceil),
            leg_acc_mmps2=min(a, a_ceil),
            leg_jerk_mmps3=min(j, j_ceil),
            leg_vel_ceiling_mmps=v_ceil,
            leg_acc_ceiling_mmps2=a_ceil,
            leg_jerk_ceiling_mmps3=j_ceil,
            knot_dt_s=float(hw.JB_TRAJ_KNOT_DT_S),
            max_step_rev=float(hw.JB_OP_MAX_POSITION_STEP_REV),
            min_move_duration_s=float(hw.JB_TRAJ_MIN_MOVE_DURATION_S),
            min_timed_lead_s=float(hw.JB_TRAJ_MIN_TIMED_LEAD_S),
        )
