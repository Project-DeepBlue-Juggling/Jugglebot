"""Plan constructors — the ONLY way a plan reaches the emitter.

Every constructor here runs :func:`feasibility.validate` before returning and
raises :class:`TrajectoryInfeasible` on failure, so all platform motion is gated
at one point (the plan's load-bearing convention). Constructors:

  * :func:`build_hold`   — freeze at the current pose (a C2 decel-to-rest when the
    seed state is moving; a degenerate ``HoldPlan`` when already at rest).
  * :func:`build_return_to_neutral` — a single profiled quintic back to the neutral
    active pose ``(0, 0, 170, 0, 0, 0)`` (``trajectory/go_home``); a no-op-shaped
    degenerate segment when already at neutral and at rest.
  * :func:`build_move` (Phase 2) — an arbitrary-target profiled point-to-point move
    with the duration-stretch loop: honour a requested duration or find the minimal
    feasible one, and **loudly reject** a too-tight requested duration (``TOO_FAST``
    with the achievable minimum).

The ``build_timed`` / ``build_catch`` constructors land in later phases; they will
use the same ``validate`` + ``TrajectoryInfeasible`` contract.

A plan seed state is ``(pose, twist, accel)`` — each a 6-vector, matching
``TrajectoryPlan.state_at``'s return — so a replan chains continuously (C2) off
the previous plan's sampled state.

Pure Python + numpy. No ROS2 / repo-root imports.
"""

from __future__ import annotations

import numpy as np

from jugglebot.motion.trajectory.feasibility import (
    LIMIT_ACC,
    LIMIT_JERK,
    LIMIT_VEL,
    STEP_BOUND,
    STEP_BOUND_MARGIN,
    TOO_FAST,
    TrajectoryInfeasible,
    validate,
)
from jugglebot.motion.trajectory.plan import HoldPlan, TrajectoryPlan
from jugglebot.motion.trajectory.segment import POSE_DIM, QuinticSegment

# A moving seed is treated as "at rest" below these thresholds (per pose axis),
# so a hold from a genuinely stationary platform yields a trivial HoldPlan rather
# than a needless (but harmless) decel segment.
_REST_TWIST_EPS = 1e-6      # mm/s and rad/s
_REST_ACCEL_EPS = 1e-6      # mm/s² and rad/s²

# Codes the duration-stretch loop CAN fix by lengthening the move (they scale with
# 1/T, 1/T², 1/T³). A WORKSPACE / UNREACHABLE failure is spatial — the rest-to-rest
# quintic traces the same set of poses regardless of duration — so stretching can
# never fix it, and the planner re-raises immediately.
_STRETCHABLE = (LIMIT_VEL, LIMIT_ACC, LIMIT_JERK, STEP_BOUND)

# Fixed-point cap for the stretch loop. From a rest start the leg peaks scale
# EXACTLY as 1/Tⁿ (the spatial path is fixed; only its timing rescales), so the
# worst-ratio factor lands feasible in ONE step and the 1.05 margin confirms it in
# the second — convergence in ≤2 iters. A moving seed breaks the exact scaling, so
# a handful of extra iters is budgeted before giving up with TOO_FAST.
_MAX_STRETCH_ITERS = 12
_STRETCH_MARGIN = 1.05


def _as6(vec, name: str) -> np.ndarray:
    a = np.asarray(vec, dtype=float)
    if a.shape != (POSE_DIM,):
        raise ValueError(f"{name} must be shape ({POSE_DIM},), got {a.shape}")
    return a


def _is_at_rest(twist: np.ndarray, accel: np.ndarray) -> bool:
    return (bool(np.all(np.abs(twist) <= _REST_TWIST_EPS))
            and bool(np.all(np.abs(accel) <= _REST_ACCEL_EPS)))


def build_hold(state0, limits, geom, *, stop_duration_s: float | None = None
               ) -> TrajectoryPlan:
    """Freeze at the current pose.

    ``state0`` is ``(pose, twist, accel)``. When the seed is already at rest,
    returns a degenerate :class:`HoldPlan` (constant pose). When the seed is
    moving, returns a single C2 quintic that decelerates back to the seed pose at
    rest over ``stop_duration_s`` (defaults to ``min_move_duration_s``) — smooth,
    rest-terminating, gated by :func:`validate`.
    """
    pose, twist, accel = (_as6(state0[0], 'pose'),
                          _as6(state0[1], 'twist'),
                          _as6(state0[2], 'accel'))
    if _is_at_rest(twist, accel):
        plan = HoldPlan(pose)
        _gate(plan, limits, geom)
        return plan

    dur = float(stop_duration_s if stop_duration_s is not None
                else limits.min_move_duration_s)
    seg = QuinticSegment(
        p0=pose, v0=twist, a0=accel,
        p1=pose, v1=np.zeros(POSE_DIM), a1=np.zeros(POSE_DIM),
        duration=dur)
    plan = TrajectoryPlan(segments=(seg,), final_pose=pose)
    _gate(plan, limits, geom)
    return plan


def build_return_to_neutral(state0, neutral_pose, duration_s, limits, geom
                            ) -> TrajectoryPlan:
    """A single profiled quintic from the seed state back to ``neutral_pose``.

    ``neutral_pose`` is the active pose ``(0, 0, 170, 0, 0, 0)``. If the seed is
    already at ``neutral_pose`` and at rest the segment is degenerate (no motion),
    so ``trajectory/go_home`` from the held active pose is a genuine no-op.
    Raises :class:`TrajectoryInfeasible` if the move violates the gate.
    """
    pose, twist, accel = (_as6(state0[0], 'pose'),
                          _as6(state0[1], 'twist'),
                          _as6(state0[2], 'accel'))
    target = _as6(neutral_pose, 'neutral_pose')
    dur = max(float(duration_s), float(limits.min_move_duration_s))
    seg = QuinticSegment(
        p0=pose, v0=twist, a0=accel,
        p1=target, v1=np.zeros(POSE_DIM), a1=np.zeros(POSE_DIM),
        duration=dur)
    plan = TrajectoryPlan(segments=(seg,), final_pose=target)
    _gate(plan, limits, geom)
    return plan


def build_move(state0, target_pose, duration_s, limits, geom
               ) -> TrajectoryPlan:
    """Profiled point-to-point move from ``state0`` to ``target_pose`` at rest.

    ``state0`` is ``(pose, twist, accel)`` (the seed continues the previous plan's
    sampled state — C2 by construction). The move ends at rest (zero twist/accel)
    at ``target_pose``.

    ``duration_s`` semantics:
      * ``None`` or ``<= 0`` — use the **minimal feasible** duration (the
        duration-stretch loop finds the shortest T the gate accepts).
      * ``> 0`` — honour the requested duration if it is ``>= min feasible``;
        otherwise raise :class:`TrajectoryInfeasible` (``TOO_FAST``) carrying the
        minimal feasible duration in ``min_duration_s`` so the caller can surface
        an achievable time. A too-tight duration is **loudly rejected**, never
        silently stretched — the operator asked for a specific timing.

    Raises :class:`TrajectoryInfeasible` immediately (no stretch) on a spatial
    ``WORKSPACE`` / ``UNREACHABLE`` failure, which a longer duration cannot fix.
    """
    pose, twist, accel = (_as6(state0[0], 'pose'),
                          _as6(state0[1], 'twist'),
                          _as6(state0[2], 'accel'))
    target = _as6(target_pose, 'target_pose')

    t_min, plan_min = _min_feasible_move(pose, twist, accel, target, limits, geom)

    if duration_s is None or float(duration_s) <= 0.0:
        return plan_min

    requested = float(duration_s)
    if requested < t_min - 1e-9:
        raise TrajectoryInfeasible(
            TOO_FAST,
            [f"requested duration {requested:.3f}s < minimal feasible "
             f"{t_min:.3f}s for this move at the current limits"],
            min_duration_s=t_min)

    # Requested duration is at or above the minimal feasible one: the same spatial
    # path at a longer time has strictly smaller leg peaks, so it re-validates.
    seg = QuinticSegment(
        p0=pose, v0=twist, a0=accel,
        p1=target, v1=np.zeros(POSE_DIM), a1=np.zeros(POSE_DIM),
        duration=requested)
    plan = TrajectoryPlan(segments=(seg,), final_pose=target)
    report = validate(plan, limits, geom)
    if not report.ok:
        # Only reachable if the failure is spatial (a longer T cannot fix it) —
        # _min_feasible_move already raised on a spatial failure, so this is a
        # defensive backstop.
        raise TrajectoryInfeasible(report.code, report.reasons, t_min)
    return plan


def _build_rest_move(pose, twist, accel, target, duration_s):
    seg = QuinticSegment(
        p0=pose, v0=twist, a0=accel,
        p1=target, v1=np.zeros(POSE_DIM), a1=np.zeros(POSE_DIM),
        duration=float(duration_s))
    return TrajectoryPlan(segments=(seg,), final_pose=target)


def _stretch_factor(report, limits) -> float:
    """Worst-ratio duration-stretch factor from a failed gate report.

    Each leg peak scales with a power of 1/T (vel ∝ 1/T, acc ∝ 1/T², jerk ∝ 1/T³,
    per-knot step ∝ 1/T like velocity), so the T-multiplier that brings each under
    its ceiling is the ratio raised to the reciprocal power. Taking the max over
    all four makes the single worst constraint binding; the 1.05 margin lands the
    move strictly inside the gate.
    """
    r_vel = report.peak_leg_vel_mmps / max(limits.leg_vel_mmps, 1e-9)
    r_acc = report.peak_leg_acc_mmps2 / max(limits.leg_acc_mmps2, 1e-9)
    r_jerk = report.peak_leg_jerk_mmps3 / max(limits.leg_jerk_mmps3, 1e-9)
    step_bound = STEP_BOUND_MARGIN * max(limits.max_step_rev, 1e-9)
    r_step = report.peak_step_rev / step_bound
    factor = max(r_vel,
                 np.sqrt(max(r_acc, 0.0)),
                 np.cbrt(max(r_jerk, 0.0)),
                 r_step)
    return max(float(factor) * _STRETCH_MARGIN, 1.0)


def _min_feasible_move(pose, twist, accel, target, limits, geom):
    """Shortest duration the gate accepts for a rest-terminating move.

    Starts at the ``min_move_duration_s`` floor and stretches by the exact
    worst-ratio factor until feasible (≤2 iters from a rest start). Returns
    ``(duration_s, plan)``. Raises :class:`TrajectoryInfeasible` on a spatial
    failure (immediately) or if the loop fails to converge (``TOO_FAST``).
    """
    T = float(limits.min_move_duration_s)
    report = None
    for _ in range(_MAX_STRETCH_ITERS):
        plan = _build_rest_move(pose, twist, accel, target, T)
        report = validate(plan, limits, geom)
        if report.ok:
            return T, plan
        if report.code not in _STRETCHABLE:
            # Spatial failure — a longer duration cannot fix it.
            raise TrajectoryInfeasible(report.code, report.reasons)
        T *= _stretch_factor(report, limits)

    raise TrajectoryInfeasible(
        TOO_FAST,
        [f"could not find a feasible duration after {_MAX_STRETCH_ITERS} "
         f"iterations (last failure {report.code if report else 'n/a'})"],
        min_duration_s=T)


def _gate(plan, limits, geom) -> None:
    """Run the canonical gate; raise :class:`TrajectoryInfeasible` on failure."""
    report = validate(plan, limits, geom)
    if not report.ok:
        raise TrajectoryInfeasible(
            report.code, report.reasons, report.min_duration_s)
