"""Streaming-target follower for SpaceMouse / GUI flight (Phase 3).

A ``TargetFollower`` turns a continuously-updated *target pose* (from the
SpaceMouse, GUI, or shell) into a stream of C2-continuous plans that the emitter
samples — the "continuous target following through the same layer" MVP goal. It
is a pure, testable object: the ROS wiring (subscriptions, mode gating, gravity
composition, install) lives in ``trajectory_node``; the tracking policy lives
here.

Per 40 Hz emitter tick the node hands the follower the **current commanded
state** ``(pose, twist, accel)`` (sampled from the active plan — so the new plan
is C2 by construction) and the **latest** target (drain-to-latest: only the newest
target matters). :meth:`TargetFollower.follow` then:

  1. **Saturation clamp** — if the target is outside the reachable leg-stroke
     workspace, clamp it to the furthest reachable point along the
     ``current → target`` ray (``saturated=True`` so the node logs a throttled
     WARN). This uses the EXISTING stroke envelope (``check_leg_extensions``) — it
     invents no new limit — and every clamped plan is still independently gated. It
     exists so a shove past the edge tracks *up to* the boundary instead of
     freezing (an unreachable target would fail the gate's stroke check and the
     platform would not move at all).

     Scope of the guarantee: the clamp enforces **stroke only** — it does NOT
     guarantee every other spatial check the full gate makes. In particular a
     target that is in-stroke but *near-singular* (Jacobian condition over the
     workspace hard bound) is not caught by the clamp; ``build_follow`` gate-rejects
     it and the follower falls back to keep-last-plan (a freeze) with a throttled
     WARN. That is acceptable here: in this platform's geometry stroke provably
     binds before ill-conditioning (see the Phase-2 condition-check discussion), so
     the clamp's stroke bound is reached first — adding the per-sample condition SVD
     to this emitter-thread bisection is deliberately avoided (it is the ~9 % cost
     the fast gate already decimates).
  2. **Deadband** — if the clamped target is within ``pos_deadband_mm`` /
     ``rot_deadband_rad`` of the last target we planned toward, do nothing (keep the
     current plan running to its terminal hold). This stops SpaceMouse jitter from
     churning a fresh plan every tick when the operator is holding still.
  3. **Replan** — otherwise build a follower move toward the clamped target over
     ``max(min_feasible, horizon_s)`` via ``planner.build_follow`` (the FAST
     ``validate_follow`` gate). On a gate rejection the follower **keeps its last
     valid plan** (the plan's "spacemouse rejections … keep the last valid plan"
     rule) and reports the rejection for a throttled WARN — it never raises into
     the hot path.

Pure Python + numpy. No ROS2 / repo-root imports.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from jugglebot.motion.ik_solver import (
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
)
from jugglebot.motion.workspace import check_leg_extensions
import jugglebot.motion.trajectory.planner as planner
from jugglebot.motion.trajectory.feasibility import TrajectoryInfeasible
from jugglebot.motion.trajectory.segment import POSE_DIM

# Defaults from the plan: 0.5 mm / 0.1° deadband.
_DEFAULT_POS_DEADBAND_MM = 0.5
_DEFAULT_ROT_DEADBAND_RAD = float(np.deg2rad(0.1))


@dataclass
class FollowResult:
    """Outcome of one :meth:`TargetFollower.follow` call.

    ``plan`` is the new plan to install, or ``None`` to keep the current one
    (deadbanded, or a gate rejection). The flags/strings drive the node's
    throttled logging and diagnostics.
    """
    plan: object = None
    report: object = None
    saturated: bool = False
    deadbanded: bool = False
    rejection: str = ''


class TargetFollower:
    """Stateful streaming-target tracker (one per follower-mode session)."""

    def __init__(self, geom, horizon_s: float, *,
                 pos_deadband_mm: float = _DEFAULT_POS_DEADBAND_MM,
                 rot_deadband_rad: float = _DEFAULT_ROT_DEADBAND_RAD):
        self._geom = geom
        self._horizon_s = float(horizon_s)
        self._pos_db = float(pos_deadband_mm)
        self._rot_db = float(rot_deadband_rad)
        self._last_target = None          # last clamped pose we planned toward
        self._last_rejected_target = None  # last clamped pose the gate REJECTED
        self._last_rejection_msg = ''      # its rejection string (for status/WARN)

    def reset(self) -> None:
        """Forget the last target — call on follower-mode entry / re-seed so the
        first target after a (re)seed always replans (never spuriously deadbanded
        against a stale target from a previous session)."""
        self._last_target = None
        self._last_rejected_target = None
        self._last_rejection_msg = ''

    def follow(self, state0, target_pose, limits) -> FollowResult:
        """Track ``target_pose`` from the current commanded ``state0``.

        ``state0`` is ``(pose, twist, accel)`` (the active plan sampled at now).
        Returns a :class:`FollowResult` — see the class docstring for the
        clamp → deadband → replan policy.
        """
        current_pose = np.asarray(state0[0], dtype=float)
        target = np.asarray(target_pose, dtype=float)
        if target.shape != (POSE_DIM,):
            raise ValueError(
                f"target_pose must be shape ({POSE_DIM},), got {target.shape}")

        clamped, saturated = self._clamp_to_workspace(current_pose, target)

        # Sustained-reject skip: if the last replan REJECTED a target and this one
        # is still within deadband of that rejected pose, skip re-running the
        # (up-to-6-pass) build_follow — it would only reject again, spamming the
        # gate every 40 Hz tick while the operator holds an infeasible target. The
        # skip is cleared the moment the target moves beyond deadband (below).
        if self._last_rejected_target is not None and self._within_deadband(
                clamped, self._last_rejected_target):
            return FollowResult(plan=None, saturated=saturated,
                                rejection=self._last_rejection_msg)

        if self._last_target is not None and self._within_deadband(
                clamped, self._last_target):
            return FollowResult(plan=None, saturated=saturated, deadbanded=True)

        try:
            plan, report = planner.build_follow(
                state0, clamped, limits, self._geom, self._horizon_s)
        except TrajectoryInfeasible as e:
            # Keep the last valid plan; the node logs this throttled. Remember the
            # rejected target so a sustained hold on it skips the gate next tick.
            self._last_rejected_target = clamped
            self._last_rejection_msg = str(e)
            return FollowResult(plan=None, saturated=saturated, rejection=str(e))

        self._last_target = clamped
        self._last_rejected_target = None   # accepted → clear the reject memory
        return FollowResult(plan=plan, report=report, saturated=saturated)

    # ── helpers ──────────────────────────────────────────────────

    def _within_deadband(self, a: np.ndarray, b: np.ndarray) -> bool:
        pos_close = bool(np.linalg.norm(a[:3] - b[:3]) <= self._pos_db)
        # rotvec-difference norm ≈ relative rotation angle at MVP tilt scales
        # (≤12°); the small-angle error is far below a 0.1° deadband.
        rot_close = bool(np.linalg.norm(a[3:6] - b[3:6]) <= self._rot_db)
        return pos_close and rot_close

    def _reachable(self, pose: np.ndarray) -> bool:
        pos = pose[:3]
        rot = rotvec_to_rot_matrix(pose[3:6])
        ext = pose_to_leg_lengths(pos, rot, self._geom)
        valid, _ = check_leg_extensions(ext, self._geom)
        return valid

    def _clamp_to_workspace(self, current_pose: np.ndarray,
                            target_pose: np.ndarray):
        """Clamp ``target_pose`` to the furthest reachable point on the
        ``current → target`` ray. Returns ``(clamped_pose, saturated)``.

        If the target is already reachable, returns it unchanged
        (``saturated=False``). Otherwise binary-searches the interpolation factor
        ``alpha`` (``pose = current + alpha·(target − current)``) for the largest
        reachable ``alpha`` and returns that pose (``saturated=True``). The current
        commanded pose is reachable by construction (it came from a gated plan), so
        ``alpha = 0`` is always a valid lower bound.
        """
        if self._reachable(target_pose):
            return target_pose, False

        delta = target_pose - current_pose
        lo, hi = 0.0, 1.0
        # 24 bisections → sub-µm / sub-µrad resolution on the ray; the interior of
        # the resulting current→clamped quintic stays reachable (both endpoints are
        # in-stroke and the ray is short) and the gate double-checks it anyway.
        for _ in range(24):
            mid = 0.5 * (lo + hi)
            if self._reachable(current_pose + mid * delta):
                lo = mid
            else:
                hi = mid
        return current_pose + lo * delta, True
