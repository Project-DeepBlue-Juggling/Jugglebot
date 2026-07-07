"""The single canonical feasibility gate for all platform motion.

Every plan constructor in ``planner.py`` calls :func:`validate` before returning,
so **nothing that moves the platform bypasses this gate** — that single-enforcement
-point property is what makes the loud-rejection guarantee hold (see the MVP plan's
"single most load-bearing convention").

Phase 1 scope (this file): the **minimal** gate — leg stroke / workspace bounds
and leg velocity / acceleration ceilings, sampled densely along the plan. This is
trivially satisfied by a hold, which is all Phase 1 streams. The full gate
(reachability, closed-form leg jerk, timing duration-stretch, per-knot step bound)
lands in Phase 2; the ``FeasibilityReport`` code enum is already the full set so
Phase 2 only fills in checks, never reshapes the contract.

Pure Python + numpy. No ROS2 / repo-root imports.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from jugglebot.motion.ik_solver import (
    compute_jacobian,
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
    twist_to_leg_velocities,
    accel_to_leg_accels,
)
from jugglebot.motion.workspace import check_leg_extensions


# Machine-readable feasibility codes. The full set is declared now (Phase 1 only
# emits a subset) so downstream consumers pin against a stable enum.
OK = 'OK'
UNREACHABLE = 'UNREACHABLE'
WORKSPACE = 'WORKSPACE'
LIMIT_VEL = 'LIMIT_VEL'
LIMIT_ACC = 'LIMIT_ACC'
LIMIT_JERK = 'LIMIT_JERK'
TOO_FAST = 'TOO_FAST'
STEP_BOUND = 'STEP_BOUND'
STALE_STATE = 'STALE_STATE'
WRONG_MODE = 'WRONG_MODE'


@dataclass
class FeasibilityReport:
    """Outcome of a :func:`validate` pass over a plan."""
    ok: bool
    code: str
    reasons: list = field(default_factory=list)
    min_duration_s: float = 0.0
    # Measured per-axis peaks over the sampled path (leg-extension units).
    peak_leg_vel_mmps: float = 0.0
    peak_leg_acc_mmps2: float = 0.0
    peak_leg_ext_mm: float = 0.0


class TrajectoryInfeasible(Exception):
    """Raised by ``planner.py`` constructors when a plan fails :func:`validate`.

    Carries the machine ``code``, human ``reasons``, and (when the failure is a
    timing/limit one that a longer duration would fix) ``min_duration_s`` so the
    caller can surface an achievable duration to the operator — the plan's
    "loudly rejected, never silently dropped" requirement.
    """

    def __init__(self, code: str, reasons, min_duration_s: float = 0.0):
        self.code = code
        self.reasons = list(reasons)
        self.min_duration_s = float(min_duration_s)
        super().__init__(f"{code}: {'; '.join(self.reasons)}")


def _pose_to_pos_rot(pose: np.ndarray):
    pos = np.asarray(pose[:3], dtype=float)
    rot = rotvec_to_rot_matrix(np.asarray(pose[3:6], dtype=float))
    return pos, rot


def validate(plan, limits, geom, *, samples_per_segment: int = 200
             ) -> FeasibilityReport:
    """Validate ``plan`` against ``limits`` by dense sampling of the leg chain.

    Phase-1 checks (in order; first failure wins the ``code``):
      1. **Stroke / workspace** — every sampled pose's leg extensions within the
         hard stroke margins (``check_leg_extensions``).
      2. **Leg velocity** — peak sampled leg extension rate ≤ ``leg_vel_mmps``.
      3. **Leg acceleration** — peak sampled leg extension accel ≤ ``leg_acc_mmps2``.

    A :class:`HoldPlan` (no segments) samples its single fixed pose. Returns a
    :class:`FeasibilityReport`; ``ok`` False carries the failing ``code`` +
    human ``reasons``.
    """
    # Build the sample time grid: dense per segment, plus the terminal pose.
    sample_times = []
    if plan.segments:
        for start, seg in zip(plan._starts, plan.segments):
            n = max(2, int(samples_per_segment))
            for k in range(n):
                sample_times.append(start + seg.duration * k / (n - 1))
        sample_times.append(plan.total_duration)
    else:
        sample_times.append(0.0)

    peak_vel = 0.0
    peak_acc = 0.0
    peak_ext = 0.0

    for t in sample_times:
        pose, twist, accel = plan.state_at(t)
        pos, rot = _pose_to_pos_rot(pose)

        ext = pose_to_leg_lengths(pos, rot, geom)
        peak_ext = max(peak_ext, float(np.max(np.abs(ext))))
        valid, states = check_leg_extensions(ext, geom)
        if not valid:
            bad = [i for i, s in enumerate(states) if s != 0]
            reasons = [
                f"leg {i} out of stroke ({ext[i]:.1f} mm) at t={t:.3f}s"
                for i in bad]
            return FeasibilityReport(
                ok=False, code=WORKSPACE, reasons=reasons,
                peak_leg_ext_mm=peak_ext)

        J = compute_jacobian(pos, rot, geom)
        leg_vel = twist_to_leg_velocities(twist, pos, rot, geom, J=J)
        vmax = float(np.max(np.abs(leg_vel)))
        peak_vel = max(peak_vel, vmax)
        if vmax > limits.leg_vel_mmps:
            return FeasibilityReport(
                ok=False, code=LIMIT_VEL,
                reasons=[f"leg velocity {vmax:.1f} mm/s > "
                         f"{limits.leg_vel_mmps:.1f} at t={t:.3f}s"],
                peak_leg_vel_mmps=peak_vel, peak_leg_ext_mm=peak_ext)

        leg_acc = accel_to_leg_accels(accel, twist, pos, rot, geom, J=J)
        amax = float(np.max(np.abs(leg_acc)))
        peak_acc = max(peak_acc, amax)
        if amax > limits.leg_acc_mmps2:
            return FeasibilityReport(
                ok=False, code=LIMIT_ACC,
                reasons=[f"leg acceleration {amax:.1f} mm/s² > "
                         f"{limits.leg_acc_mmps2:.1f} at t={t:.3f}s"],
                peak_leg_vel_mmps=peak_vel, peak_leg_acc_mmps2=peak_acc,
                peak_leg_ext_mm=peak_ext)

    return FeasibilityReport(
        ok=True, code=OK, reasons=[],
        peak_leg_vel_mmps=peak_vel, peak_leg_acc_mmps2=peak_acc,
        peak_leg_ext_mm=peak_ext)
