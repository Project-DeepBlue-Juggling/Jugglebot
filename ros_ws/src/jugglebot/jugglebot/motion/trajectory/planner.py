"""Plan constructors — the ONLY way a plan reaches the emitter.

Every constructor here runs :func:`feasibility.validate` before returning and
raises :class:`TrajectoryInfeasible` on failure, so all platform motion is gated
at one point (the plan's load-bearing convention). Phase 1 exposes only the
constructors the streaming-foundation node needs:

  * :func:`build_hold`   — freeze at the current pose (a C2 decel-to-rest when the
    seed state is moving; a degenerate ``HoldPlan`` when already at rest).
  * :func:`build_return_to_neutral` — a single profiled quintic back to the neutral
    active pose ``(0, 0, 170, 0, 0, 0)`` (``trajectory/go_home``); a no-op-shaped
    degenerate segment when already at neutral and at rest.

The arbitrary-target ``build_move`` / ``build_timed`` / ``build_catch``
constructors + the duration-stretch loop land in later phases; they will use the
same ``validate`` + ``TrajectoryInfeasible`` contract.

A plan seed state is ``(pose, twist, accel)`` — each a 6-vector, matching
``TrajectoryPlan.state_at``'s return — so a replan chains continuously (C2) off
the previous plan's sampled state.

Pure Python + numpy. No ROS2 / repo-root imports.
"""

from __future__ import annotations

import numpy as np

from jugglebot.motion.trajectory.feasibility import (
    TrajectoryInfeasible, validate,
)
from jugglebot.motion.trajectory.plan import HoldPlan, TrajectoryPlan
from jugglebot.motion.trajectory.segment import POSE_DIM, QuinticSegment

# A moving seed is treated as "at rest" below these thresholds (per pose axis),
# so a hold from a genuinely stationary platform yields a trivial HoldPlan rather
# than a needless (but harmless) decel segment.
_REST_TWIST_EPS = 1e-6      # mm/s and rad/s
_REST_ACCEL_EPS = 1e-6      # mm/s² and rad/s²


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


def _gate(plan, limits, geom) -> None:
    """Run the canonical gate; raise :class:`TrajectoryInfeasible` on failure."""
    report = validate(plan, limits, geom)
    if not report.ok:
        raise TrajectoryInfeasible(
            report.code, report.reasons, report.min_duration_s)
