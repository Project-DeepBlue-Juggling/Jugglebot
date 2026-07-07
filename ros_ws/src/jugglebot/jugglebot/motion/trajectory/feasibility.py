"""The single canonical feasibility gate for all platform motion.

Every plan constructor in ``planner.py`` calls :func:`validate` before returning,
so **nothing that moves the platform bypasses this gate** — that single-enforcement
-point property is what makes the loud-rejection guarantee hold (see the MVP plan's
"single most load-bearing convention").

Phase 2 scope (this file): the **full** gate. In order (first failure wins the
``code``):

1. **Geometry** per dense sample (200/segment): non-finite reject (``UNREACHABLE``);
   leg extensions within the hard stroke margins (``WORKSPACE``); Jacobian
   condition number under the workspace hard bound (``UNREACHABLE`` — a
   near-singular pose is unrealisable no matter the timing).
2. **Leg kinematic peaks** via the ``ik_solver`` Jacobian chain: peak leg velocity
   (``LIMIT_VEL``), acceleration (``LIMIT_ACC``), and jerk (``LIMIT_JERK``, from
   the finite difference of the analytic leg acceleration — a second-order-accurate
   stand-in for the third difference of leg position).
3. **Knot-step bound** (``STEP_BOUND``): the actual wire ``u0`` sequence is the plan
   sampled at the 25 ms knot spacing and mapped ``ext × mm_to_rev``; the max
   per-knot ``|Δu0|`` must stay under ``max_step_rev`` with a 20 % margin so a
   move is rejected here, before motion, rather than mid-stream at the pump.

Timing (``TOO_FAST``) and the duration-stretch loop live in ``planner.build_move``,
which drives :func:`validate` iteratively; this function is a **pure predicate over
a fully-specified plan** — it never mutates durations. It always computes *all*
peaks (a full pass, no early return on a limit failure) so the planner can read the
worst-ratio off one report and stretch in a single step. Only the geometry checks
early-return (a non-finite / out-of-stroke / singular pose makes the Jacobian peaks
meaningless).

Pure Python + numpy. No ROS2 / repo-root imports.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from jugglebot.motion.ik_solver import (
    accel_to_leg_accels,
    compute_jacobian,
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
    twist_to_leg_velocities,
)
from jugglebot.motion.workspace import (
    WorkspaceLimits,
    check_leg_extensions,
    compute_condition_number,
)


# Machine-readable feasibility codes. The full set is declared for downstream
# consumers to pin against a stable enum.
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

# The per-knot |Δu0| bound is enforced with a 20 % safety margin below the pump's
# hard step gate, so the gate rejects a step-heavy move BEFORE it reaches the pump
# (which would otherwise reject it mid-stream, one tick after motion started).
STEP_BOUND_MARGIN = 0.80

# A jerk finite difference needs at least this many samples per segment.
_MIN_SAMPLES = 4


@dataclass
class FeasibilityReport:
    """Outcome of a :func:`validate` pass over a plan.

    Carries **all** measured leg-space peaks (not just the failing one) so the
    planner's duration-stretch loop can compute the worst limit ratio from a
    single report and converge in one step.
    """
    ok: bool
    code: str
    reasons: list = field(default_factory=list)
    min_duration_s: float = 0.0
    # Measured peaks over the sampled path.
    peak_leg_vel_mmps: float = 0.0
    peak_leg_acc_mmps2: float = 0.0
    peak_leg_jerk_mmps3: float = 0.0
    peak_leg_ext_mm: float = 0.0
    peak_step_rev: float = 0.0


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


def _segment_grids(plan, samples_per_segment: int):
    """Per-segment uniform time grids ``(times, dt)`` for jerk finite differences.

    A uniform grid *within* each segment is what lets the jerk finite difference
    use a single ``dt``; jerk is only C0 across segment joins (the plan is C2, not
    C3), so a per-segment grid is correct — a cross-join difference would fabricate
    a spurious jerk spike at the seam.
    """
    grids = []
    if plan.segments:
        for start, seg in zip(plan._starts, plan.segments):
            n = max(_MIN_SAMPLES, int(samples_per_segment))
            times = start + seg.duration * np.arange(n) / (n - 1)
            grids.append((times, seg.duration / (n - 1)))
    else:
        # HoldPlan: a single fixed pose, no time axis.
        grids.append((np.array([0.0]), 0.0))
    return grids


def validate(plan, limits, geom, *, samples_per_segment: int = 200
             ) -> FeasibilityReport:
    """Validate ``plan`` against ``limits`` by dense sampling of the leg chain.

    Returns a :class:`FeasibilityReport`. On a geometry failure the report
    early-returns with the offending ``code`` and populated ``reasons``. Otherwise
    it always carries every measured peak, and ``code`` is the first limit/step
    check to fail (priority: vel → acc → jerk → step) or ``OK``.
    """
    wlimits = WorkspaceLimits.from_geometry(geom)
    mm_to_rev = np.asarray(geom.mm_to_rev, dtype=float)

    peak_vel = 0.0
    peak_acc = 0.0
    peak_jerk = 0.0
    peak_ext = 0.0

    # ── Passes 1 + 2: geometry + leg vel/acc/jerk, per segment ──
    for times, dt in _segment_grids(plan, samples_per_segment):
        acc_samples = []
        for t in times:
            pose, twist, accel = plan.state_at(float(t))
            # Reject non-finite input up front: a NaN/Inf pose sails through every
            # numeric comparison below (all comparisons against NaN are False), so
            # the gate must catch it explicitly. Reuse UNREACHABLE.
            if not np.all(np.isfinite(pose)):
                return FeasibilityReport(
                    ok=False, code=UNREACHABLE,
                    reasons=[f"pose contains non-finite values (NaN/Inf) "
                             f"at t={t:.3f}s"])
            pos, rot = _pose_to_pos_rot(pose)

            ext = pose_to_leg_lengths(pos, rot, geom)
            peak_ext = max(peak_ext, float(np.max(np.abs(ext))))
            valid, states = check_leg_extensions(ext, geom)
            if not valid:
                bad = [i for i, s in enumerate(states) if s != 0]
                return FeasibilityReport(
                    ok=False, code=WORKSPACE,
                    reasons=[f"leg {i} out of stroke ({ext[i]:.1f} mm) "
                             f"at t={t:.3f}s" for i in bad],
                    peak_leg_ext_mm=peak_ext)

            J = compute_jacobian(pos, rot, geom)
            cond = compute_condition_number(pos, rot, geom, J=J)
            if cond > wlimits.cond_hard:
                # Near-singular: unrealisable no matter the timing.
                return FeasibilityReport(
                    ok=False, code=UNREACHABLE,
                    reasons=[f"Jacobian condition {cond:.1f} > "
                             f"{wlimits.cond_hard:.1f} (near singularity) "
                             f"at t={t:.3f}s"],
                    peak_leg_ext_mm=peak_ext)

            leg_vel = twist_to_leg_velocities(twist, pos, rot, geom, J=J)
            peak_vel = max(peak_vel, float(np.max(np.abs(leg_vel))))
            leg_acc = accel_to_leg_accels(accel, twist, pos, rot, geom, J=J)
            peak_acc = max(peak_acc, float(np.max(np.abs(leg_acc))))
            acc_samples.append(leg_acc)

        # Jerk from the finite difference of the analytic leg acceleration.
        if dt > 0.0 and len(acc_samples) >= 2:
            arr = np.asarray(acc_samples)
            jerk = np.diff(arr, axis=0) / dt
            peak_jerk = max(peak_jerk, float(np.max(np.abs(jerk))))

    # ── Pass 3: knot-step bound (the actual wire u0 sequence) ──
    peak_step = 0.0
    if plan.total_duration > 0.0:
        kdt = float(limits.knot_dt_s)
        n_knots = int(np.floor(plan.total_duration / kdt)) + 2
        prev_rev = None
        for k in range(n_knots):
            tk = min(k * kdt, plan.total_duration)
            pose_k, _, _ = plan.state_at(tk)
            pos_k, rot_k = _pose_to_pos_rot(pose_k)
            motor_rev = pose_to_leg_lengths(pos_k, rot_k, geom) * mm_to_rev
            if prev_rev is not None:
                peak_step = max(
                    peak_step, float(np.max(np.abs(motor_rev - prev_rev))))
            prev_rev = motor_rev
            if tk >= plan.total_duration:
                break

    # ── Decide the code (priority order; first failure wins) ──
    step_bound = STEP_BOUND_MARGIN * float(limits.max_step_rev)
    code = OK
    reasons: list = []
    if peak_vel > limits.leg_vel_mmps:
        code = LIMIT_VEL
        reasons = [f"peak leg velocity {peak_vel:.1f} mm/s > "
                   f"{limits.leg_vel_mmps:.1f}"]
    elif peak_acc > limits.leg_acc_mmps2:
        code = LIMIT_ACC
        reasons = [f"peak leg acceleration {peak_acc:.1f} mm/s² > "
                   f"{limits.leg_acc_mmps2:.1f}"]
    elif peak_jerk > limits.leg_jerk_mmps3:
        code = LIMIT_JERK
        reasons = [f"peak leg jerk {peak_jerk:.0f} mm/s³ > "
                   f"{limits.leg_jerk_mmps3:.0f}"]
    elif peak_step > step_bound:
        code = STEP_BOUND
        reasons = [f"peak per-knot step {peak_step:.3f} rev > "
                   f"{step_bound:.3f} ({int(STEP_BOUND_MARGIN * 100)}% of "
                   f"{limits.max_step_rev:.3f})"]

    return FeasibilityReport(
        ok=(code == OK), code=code, reasons=reasons,
        peak_leg_vel_mmps=peak_vel, peak_leg_acc_mmps2=peak_acc,
        peak_leg_jerk_mmps3=peak_jerk, peak_leg_ext_mm=peak_ext,
        peak_step_rev=peak_step)
