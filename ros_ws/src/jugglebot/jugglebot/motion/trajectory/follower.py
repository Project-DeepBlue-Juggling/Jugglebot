"""Streaming-target follower for SpaceMouse / GUI flight (Phase 3).

A ``TargetFollower`` turns a continuously-updated *target pose* (from the
SpaceMouse, GUI, or shell) into a stream of C2-continuous plans that the emitter
samples — the "continuous target following through the same layer" MVP goal. It
is a pure, testable object: the ROS wiring (subscriptions, mode gating, gravity
composition, install) lives in ``trajectory_node``; the tracking policy lives
here.

Chase-and-cap tracking (post-2026-07-09 S3 rewrite)
---------------------------------------------------
The original policy asked the gate "can I reach the stick this tick?" and, on a
reject, kept the last plan. The 2026-07-09 S3 spacemouse forensics showed that
design failing structurally: a stick reversal hands ``build_follow`` a
high-energy seed whose *decel* half violates the leg limits; the stretch ladder
cannot fix a moving-seed candidate, so the follower limit-cycled between accept
and keep-last (legs sustained 10-45 % OVER the velocity limit) and — worst —
deadlocked permanently when the old workspace clamp parked the commanded state on
the exact hard stroke boundary the gate rejects at (the clamp and gate disagreed
*by construction*).

The rewrite never rejects in steady state. Each tick it computes, BEFORE
building the candidate, *how far toward the stick this tick's plan can feasibly
go* (:func:`chase.chase_alpha` — a frozen-Jacobian, linear-in-α feasible-progress
limiter) and plans to that chased point over the energy/distance-scaled horizon
the chase returns. The platform then always tracks the stick at max feasible
speed; a shove past the workspace edge tracks *up to* the (margin-tightened)
boundary instead of freezing. The gate stays canonical — every plan is still
``validate_follow``-checked through ``planner`` — the chase is advisory and
merely makes the candidate nearly-always acceptable (empirically ≥ 97 % single-
validate at every session tier; see ``chase.py``'s A6 sweep docstring).

Per 40 Hz emitter tick the node hands the follower the **current commanded
state** ``(pose, twist, accel)`` (sampled from the active plan — so the new plan
is C2 by construction) and the **latest** target (drain-to-latest). The pipeline
in :meth:`TargetFollower.follow`:

  1. **Margin ray-clamp** (:meth:`_clamp_to_workspace`) — bisect the target onto
     the furthest reachable point on the ``current → target`` ray, against a
     stroke envelope tightened by ``CLAMP_MARGIN_MM``. Clamping into a space
     STRICTLY inside the gate-accept space is the deadlock fix: a terminal pose
     the clamp constructs can never sit on the exact hard boundary the gate
     rejects at. If the *current* commanded pose itself sits in the (hard−margin,
     hard) band (a legitimate seed — the gate enforces only HARD bounds on plan
     interiors), the bisection's ``lo = 0`` invariant fails, so the clamp returns
     the current pose (zero progress, saturated) and lets keep-last pull the state
     back inside (A5).
  2. **Deadband** vs ``_last_target`` (the CHASED pose we last planned toward). A
     far parked stick deadbands only once the chase has converged onto it — the
     desired behaviour.
  3. **Chase clamp** (:func:`chase.chase_alpha`) — the feasible-progress limiter.
     An EMPTY α interval (an over-energetic / corrupt seed the delta term cannot
     unload — ``feasible=False``) routes the tick to a graceful stop instead of a
     doomed ``build_follow`` (A2).
  4. **Plan once** over the chase's scaled horizon via ``planner.build_follow``
     (fast ``validate_follow`` gate); the stretch cap is ``_MAX_FOLLOW_ITERS``
     (build → validate → one-or-two stretch → validate, all ≤ ~10 ms post-publish).
  5. **On a residual reject** (a rare spatial class — e.g. ``WORKSPACE`` from a
     seed whose decel overshoots the boundary): keep the last valid plan for this
     tick and count ``consecutive_rejects``. The chase recomputes next tick from
     the evolving seed — the old absorbing state is gone because terminal poses
     are now ≥ ``CLAMP_MARGIN_MM`` inside the envelope and the seed's energy
     decays under the still-running plan.
  6. **Escalation** (A3, belt-and-braces) — while ``consecutive_rejects`` stays
     ≥ ``CHASE_ESCALATE_TICKS`` (≈ 0.3 s), the :class:`FollowResult` carries
     ``escalate=True`` (level-triggered). The node latches a graceful stop off
     that flag; the counter resets on any accept or deadband (and in
     :meth:`reset`).

Pure Python + numpy. No ROS2 / repo-root imports.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from jugglebot.motion.ik_solver import (
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
)
import jugglebot.motion.trajectory.planner as planner
from jugglebot.motion.trajectory.chase import chase_alpha
from jugglebot.motion.trajectory.feasibility import (
    TrajectoryInfeasible,
    _workspace_limits,
)
from jugglebot.motion.trajectory.segment import POSE_DIM

# Defaults from the plan: 0.5 mm / 0.1° deadband.
_DEFAULT_POS_DEADBAND_MM = 0.5
_DEFAULT_ROT_DEADBAND_RAD = float(np.deg2rad(0.1))

# Stroke-envelope margin (mm) the ray-clamp tightens against (A5). The clamp
# target space must be STRICTLY inside the gate-accept space — the gate rejects a
# plan whose interior touches the hard stroke bound, so a clamp that constructed a
# rest target ON that bound (the old behaviour) parked the commanded state exactly
# where every subsequent replan's seed sample fails the stroke check → permanent
# keep-last deadlock (the 2026-07-09 S3 lockup). 0.5 mm is far above the bisection
# residual and negligible against the ~490 mm stroke.
CLAMP_MARGIN_MM = 0.5

# Consecutive-reject count at which the follower asks the node to escalate to a
# graceful stop (A3). ≈ 0.3 s at 40 Hz — long enough that a transient spatial
# reject (the seed's decel overshooting, which the chase clears within a few ticks
# as the seed decays) never trips it, short enough that a genuinely wedged /
# corrupt seed is stopped loudly well before the 250 ms staleness E-STOP matters.
CHASE_ESCALATE_TICKS = 12

# Bound on the A2 graceful-stop build (this runs on the emitter thread post-publish
# via the follower tick, so it must stay well inside the emit budget). The
# energy-informed start (planner.build_graceful_stop, A4) converges in ≤ ~3
# validates; the retry resumes next tick from the decayed seed if it doesn't.
_A2_STOP_MAX_ITERS = 6


@dataclass
class FollowResult:
    """Outcome of one :meth:`TargetFollower.follow` call.

    ``plan`` is the new plan to install, or ``None`` to keep the current one
    (deadbanded, or a gate rejection). The flags/strings drive the node's
    throttled logging, diagnostics, and escalation latch:

      * ``alpha`` — the chase's feasible-progress fraction this tick (1.0 =
        reached the clamped target, 0.0 = no progress / graceful stop). Published
        as a diagnostic so ``/diagnose`` sees how hard the platform is chasing.
      * ``escalate`` — True while ``consecutive_rejects`` ≥ ``CHASE_ESCALATE_TICKS``
        (level-triggered): the node latches a graceful stop off it (A3).
    """
    plan: object = None
    report: object = None
    saturated: bool = False
    deadbanded: bool = False
    rejection: str = ''
    alpha: float = 1.0
    escalate: bool = False


class TargetFollower:
    """Stateful streaming-target tracker (one per follower-mode session)."""

    def __init__(self, geom, horizon_s: float, *,
                 pos_deadband_mm: float = _DEFAULT_POS_DEADBAND_MM,
                 rot_deadband_rad: float = _DEFAULT_ROT_DEADBAND_RAD):
        self._geom = geom
        self._horizon_s = float(horizon_s)
        self._pos_db = float(pos_deadband_mm)
        self._rot_db = float(rot_deadband_rad)
        self._last_target = None          # last CHASED pose we planned toward
        self._last_rejection_msg = ''      # last rejection string (for status/WARN)
        self._consecutive_rejects = 0      # A3 escalation counter
        # Cache the hard stroke bounds once (the SVD-bearing workspace-limits build
        # is hoisted here — the ray-clamp predicate runs 24× per bisection per tick).
        wl = _workspace_limits(geom)
        self._hard_min_mm = float(wl.leg_hard_min_mm)
        self._hard_max_mm = float(wl.leg_hard_max_mm)

    @property
    def consecutive_rejects(self) -> int:
        """Current consecutive-reject streak (the node reads it for diagnostics)."""
        return self._consecutive_rejects

    def reset(self) -> None:
        """Forget the last target + reject streak — call on follower-mode entry /
        re-seed so the first target after a (re)seed always replans (never
        spuriously deadbanded against a stale target from a previous session) and a
        fresh session never inherits a stale escalation count."""
        self._last_target = None
        self._last_rejection_msg = ''
        self._consecutive_rejects = 0

    def follow(self, state0, target_pose, limits) -> FollowResult:
        """Track ``target_pose`` from the current commanded ``state0``.

        ``state0`` is ``(pose, twist, accel)`` (the active plan sampled at now).
        Returns a :class:`FollowResult` — see the module docstring for the
        clamp → deadband → chase → plan / keep-last policy.
        """
        current_pose = np.asarray(state0[0], dtype=float)
        target = np.asarray(target_pose, dtype=float)
        if target.shape != (POSE_DIM,):
            raise ValueError(
                f"target_pose must be shape ({POSE_DIM},), got {target.shape}")

        clamped, saturated = self._clamp_to_workspace(current_pose, target)

        # Deadband vs the last CHASED pose we planned toward. With the chase, a far
        # parked stick keeps replanning (each tick's clamped stick pose is far from
        # last tick's partway chased pose) until the chase converges onto it, then
        # deadbands — exactly the "track, then hold" behaviour we want. A deadband is
        # a non-reject outcome, so it resets the escalation counter (A3).
        if self._last_target is not None and self._within_deadband(
                clamped, self._last_target):
            self._consecutive_rejects = 0
            return FollowResult(plan=None, saturated=saturated, deadbanded=True,
                                alpha=1.0)

        # Chase clamp: the largest feasible progress toward the clamped target this
        # tick, and the energy/distance-scaled horizon to build the candidate over.
        chase = chase_alpha(state0, clamped, limits, self._geom, self._horizon_s)

        if not chase.feasible:
            # A2: the α interval is EMPTY — even α = 0 (decel-to-rest-in-place) is
            # infeasible at the scaled horizon (an over-energetic / corrupt seed the
            # forward delta cannot unload). Building a follow plan here is doomed, so
            # route to a graceful stop instead. The stop's in-place ladder converges
            # for every gate-limited seed; a genuine spatial failure (near-boundary
            # outward decel overshoot) falls through to keep-last + count below.
            try:
                stop = planner.build_graceful_stop(
                    state0, limits, self._geom, max_iters=_A2_STOP_MAX_ITERS)
            except TrajectoryInfeasible as e:
                return self._reject(str(e), saturated, chase.alpha)
            # A stop is a fresh valid plan (a non-reject outcome): reset the streak.
            # It is not a tracked target, so drop _last_target — the next fresh
            # target must replan (never deadband against a pose we never chased to).
            self._consecutive_rejects = 0
            self._last_target = None
            return FollowResult(plan=stop, saturated=saturated, alpha=0.0)

        # Plan once to the chased pose over the chase's scaled horizon (the α
        # interval was computed AT that T — building at another T invalidates it),
        # with the fast validate_follow gate + a bounded stretch fallback.
        try:
            plan, report = planner.build_follow(
                state0, chase.pose, limits, self._geom, chase.horizon_s)
        except TrajectoryInfeasible as e:
            # Residual spatial reject (rare): keep the last valid plan this tick and
            # count. The chase recomputes next tick from the evolving/decaying seed.
            return self._reject(str(e), saturated, chase.alpha)

        self._last_target = chase.pose
        self._consecutive_rejects = 0
        return FollowResult(plan=plan, report=report, saturated=saturated,
                            alpha=chase.alpha)

    # ── helpers ──────────────────────────────────────────────────

    def _reject(self, msg: str, saturated: bool, alpha: float) -> FollowResult:
        """Keep-last-plan outcome: record the rejection, bump the escalation streak,
        and surface ``escalate`` (level-triggered) once it crosses the threshold."""
        self._last_rejection_msg = msg
        self._consecutive_rejects += 1
        return FollowResult(
            plan=None, saturated=saturated, rejection=msg, alpha=alpha,
            escalate=self._consecutive_rejects >= CHASE_ESCALATE_TICKS)

    def _within_deadband(self, a: np.ndarray, b: np.ndarray) -> bool:
        pos_close = bool(np.linalg.norm(a[:3] - b[:3]) <= self._pos_db)
        # rotvec-difference norm ≈ relative rotation angle at MVP tilt scales
        # (≤12°); the small-angle error is far below a 0.1° deadband.
        rot_close = bool(np.linalg.norm(a[3:6] - b[3:6]) <= self._rot_db)
        return pos_close and rot_close

    def _reachable(self, pose: np.ndarray, margin: float) -> bool:
        """True iff every leg extension is inside ``[hard_min + margin,
        hard_max − margin]`` — the margin-tightened stroke envelope (A5)."""
        pos = pose[:3]
        rot = rotvec_to_rot_matrix(pose[3:6])
        ext = pose_to_leg_lengths(pos, rot, self._geom)
        return bool(np.all((ext >= self._hard_min_mm + margin)
                           & (ext <= self._hard_max_mm - margin)))

    def _clamp_to_workspace(self, current_pose: np.ndarray,
                            target_pose: np.ndarray):
        """Clamp ``target_pose`` to the furthest MARGIN-reachable point on the
        ``current → target`` ray. Returns ``(clamped_pose, saturated)``.

        The predicate is tightened by ``CLAMP_MARGIN_MM`` so a constructed rest
        target is STRICTLY inside the gate-accept space (never on the hard bound the
        gate rejects at — the deadlock fix). If the target is already inside the
        margin envelope it is returned unchanged (``saturated=False``).

        A5 — in-band current pose: the gate enforces only HARD bounds on plan
        interiors, so a sampled seed can legitimately sit in the (hard−margin, hard)
        band. There the bisection's ``lo = 0`` invariant (current pose reachable)
        fails, so we do NOT bisect (it would search a ray with no reachable lower
        end); we return the current pose (zero progress, ``saturated=True``). The
        in-band replan gate-rejects, keep-last pulls the commanded state back inside
        the margin, and tracking resumes — instead of constructing a target on the
        very bound that deadlocks.
        """
        margin = CLAMP_MARGIN_MM
        if self._reachable(target_pose, margin):
            return target_pose, False

        if not self._reachable(current_pose, margin):
            # Current pose is in the (hard−margin, hard) band: bisection invalid.
            return current_pose, True

        delta = target_pose - current_pose
        lo, hi = 0.0, 1.0
        # 24 bisections → sub-µm / sub-µrad resolution on the ray; the interior of
        # the resulting current→clamped quintic stays reachable (both endpoints are
        # in-stroke and the ray is short) and the gate double-checks it anyway.
        for _ in range(24):
            mid = 0.5 * (lo + hi)
            if self._reachable(current_pose + mid * delta, margin):
                lo = mid
            else:
                hi = mid
        return current_pose + lo * delta, True
