"""Actuator / pose limits for the MVP trajectory generator.

``TrajectoryLimits`` bundles the leg-space kinematic ceilings (vel/acc/jerk) the
feasibility gate enforces, the fixed 40 Hz knot spacing, and the per-knot motor
step bound. It is constructed from the generated ``JB_TRAJ_*`` /
``JB_OP_MAX_POSITION_STEP_REV`` config constants so the single source of truth
stays ``config/hardware_config.yaml``.

Two tiers of limits, for the legs and (since the unified 7-DoF planner) for the
hand:
  * the **session limit** (``leg_vel/acc/jerk``, ``hand_vel/acc``) — the
    currently-commanded ceiling, ramped up across hardware sessions (Phase 4)
    via ``trajectory/set_limits``;
  * the **hard ceiling** (``*_ceiling``) — the YAML-pinned absolute maximum a
    ``set_limits`` request is clamped to (a session limit can never exceed it).

The hand pair (``hand_vel_limit_rps`` / ``hand_vel_ceiling_rps`` and
``hand_acc_limit_rps2`` / ``hand_acc_ceiling_rps2``, owner-signed 2026-08-30 —
``logbook/2026-08-30-unified-7dof-planner-phase0-probes.md`` decision 4) is the
SAME shape for the same reason: a raise has to be bounded by something the YAML
pins, or "ramp the limit" becomes "there is no envelope". Units are hand MOTOR
revs, matching the firmware's homed frame and everything in ``hand_stroke.py``.
``feasibility.validate_cycle`` is the enforcement point on a ``CyclePlan``'s
7th channel.

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
    hand_vel_limit_rps: float    # session hand velocity ceiling (motor rev/s)
    hand_acc_limit_rps2: float   # session hand acceleration ceiling (motor rev/s²)
    hand_vel_ceiling_rps: float  # YAML hard ceiling (rev/s)
    hand_acc_ceiling_rps2: float # YAML hard ceiling (rev/s²)
    knot_dt_s: float             # fixed 40 Hz knot spacing (0.025 s)
    max_step_rev: float          # per-knot |Δu0| motor bound (rev)
    min_move_duration_s: float
    min_timed_lead_s: float
    max_timed_lead_s: float       # upper bound on a timed arrival lead (clock-domain guard)

    @staticmethod
    def from_config(hw, *,
                    leg_vel_mmps: float | None = None,
                    leg_acc_mmps2: float | None = None,
                    leg_jerk_mmps3: float | None = None,
                    hand_vel_rps: float | None = None,
                    hand_acc_rps2: float | None = None) -> 'TrajectoryLimits':
        """Build from the generated ``hardware_config`` module (``JB_TRAJ_*``).

        Optional overrides replace the YAML session-default limits (used by the
        in-session ``trajectory/set_limits`` ramp); each override is clamped to
        the corresponding YAML hard ceiling so a runtime request can never
        exceed the pinned maximum. The hand pair follows the identical rule.
        """
        v_ceil = float(hw.JB_TRAJ_LEG_VEL_CEILING_MMPS)
        a_ceil = float(hw.JB_TRAJ_LEG_ACC_CEILING_MMPS2)
        j_ceil = float(hw.JB_TRAJ_LEG_JERK_CEILING_MMPS3)
        hv_ceil = float(hw.JB_TRAJ_HAND_VEL_CEILING_RPS)
        ha_ceil = float(hw.JB_TRAJ_HAND_ACC_CEILING_RPS2)

        v = float(hw.JB_TRAJ_LEG_VEL_LIMIT_MMPS if leg_vel_mmps is None
                  else leg_vel_mmps)
        a = float(hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2 if leg_acc_mmps2 is None
                  else leg_acc_mmps2)
        j = float(hw.JB_TRAJ_LEG_JERK_LIMIT_MMPS3 if leg_jerk_mmps3 is None
                  else leg_jerk_mmps3)
        hv = float(hw.JB_TRAJ_HAND_VEL_LIMIT_RPS if hand_vel_rps is None
                   else hand_vel_rps)
        ha = float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2 if hand_acc_rps2 is None
                   else hand_acc_rps2)

        return TrajectoryLimits(
            leg_vel_mmps=min(v, v_ceil),
            leg_acc_mmps2=min(a, a_ceil),
            leg_jerk_mmps3=min(j, j_ceil),
            leg_vel_ceiling_mmps=v_ceil,
            leg_acc_ceiling_mmps2=a_ceil,
            leg_jerk_ceiling_mmps3=j_ceil,
            hand_vel_limit_rps=min(hv, hv_ceil),
            hand_acc_limit_rps2=min(ha, ha_ceil),
            hand_vel_ceiling_rps=hv_ceil,
            hand_acc_ceiling_rps2=ha_ceil,
            knot_dt_s=float(hw.JB_TRAJ_KNOT_DT_S),
            max_step_rev=float(hw.JB_OP_MAX_POSITION_STEP_REV),
            min_move_duration_s=float(hw.JB_TRAJ_MIN_MOVE_DURATION_S),
            min_timed_lead_s=float(hw.JB_TRAJ_MIN_TIMED_LEAD_S),
            max_timed_lead_s=float(hw.JB_TRAJ_MAX_TIMED_LEAD_S),
        )

    def with_session_limits(self, *,
                            leg_vel_mmps: float | None = None,
                            leg_acc_mmps2: float | None = None,
                            leg_jerk_mmps3: float | None = None,
                            hand_vel_rps: float | None = None,
                            hand_acc_rps2: float | None = None
                            ) -> 'TrajectoryLimits':
        """Return a copy with new session limits, each clamped to its ceiling.

        This is the in-session ``trajectory/set_limits`` ramp: ``None`` keeps the
        current session value; any supplied value is clamped to the corresponding
        YAML hard ceiling so a runtime request can NEVER raise a limit above the
        pinned maximum (the safety invariant — the ceilings are the physical
        envelope the operator ramps *within*, never past). The ceilings and knot /
        step / duration fields are preserved unchanged.

        The hand pair rides the same machinery deliberately: the unified 7-DoF
        cycle puts the hand on the platform's clock, so a hand ramp is the same
        kind of act as a leg ramp and must be bounded the same way. NOTE the ROS
        ``SetTrajectoryLimits`` service still carries only the three leg fields —
        widening it belongs with the wire/orchestrator work (plan Phases 2/4), so
        today the hand pair moves by YAML + regenerate + relaunch.
        """
        v = self.leg_vel_mmps if leg_vel_mmps is None else float(leg_vel_mmps)
        a = self.leg_acc_mmps2 if leg_acc_mmps2 is None else float(leg_acc_mmps2)
        j = self.leg_jerk_mmps3 if leg_jerk_mmps3 is None else float(leg_jerk_mmps3)
        hv = (self.hand_vel_limit_rps if hand_vel_rps is None
              else float(hand_vel_rps))
        ha = (self.hand_acc_limit_rps2 if hand_acc_rps2 is None
              else float(hand_acc_rps2))
        return TrajectoryLimits(
            leg_vel_mmps=min(v, self.leg_vel_ceiling_mmps),
            leg_acc_mmps2=min(a, self.leg_acc_ceiling_mmps2),
            leg_jerk_mmps3=min(j, self.leg_jerk_ceiling_mmps3),
            leg_vel_ceiling_mmps=self.leg_vel_ceiling_mmps,
            leg_acc_ceiling_mmps2=self.leg_acc_ceiling_mmps2,
            leg_jerk_ceiling_mmps3=self.leg_jerk_ceiling_mmps3,
            hand_vel_limit_rps=min(hv, self.hand_vel_ceiling_rps),
            hand_acc_limit_rps2=min(ha, self.hand_acc_ceiling_rps2),
            hand_vel_ceiling_rps=self.hand_vel_ceiling_rps,
            hand_acc_ceiling_rps2=self.hand_acc_ceiling_rps2,
            knot_dt_s=self.knot_dt_s,
            max_step_rev=self.max_step_rev,
            min_move_duration_s=self.min_move_duration_s,
            min_timed_lead_s=self.min_timed_lead_s,
            max_timed_lead_s=self.max_timed_lead_s,
        )
