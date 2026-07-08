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
    validate_follow,
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
#
# The "spatial path is duration-invariant" argument holds only for a REST seed: a
# rest-to-rest quintic's pose set is fixed and only its timing rescales. A MOVING
# seed (nonzero twist/accel) bends the path with T, so the spatial checks are NOT
# duration-invariant there — which is why the stretch loop re-validates the full
# gate on every candidate T (it does) rather than trusting a single spatial pass.
_STRETCHABLE = (LIMIT_VEL, LIMIT_ACC, LIMIT_JERK, STEP_BOUND)

# Fixed-point cap for the stretch loop. From a rest start the leg peaks scale
# EXACTLY as 1/Tⁿ (the spatial path is fixed; only its timing rescales), so the
# worst-ratio factor lands feasible in ONE step and the 1.05 margin confirms it in
# the second — convergence in ≤2 iters. A moving seed breaks the exact scaling, so
# a handful of extra iters is budgeted before giving up with TOO_FAST.
_MAX_STRETCH_ITERS = 12
_STRETCH_MARGIN = 1.05

# Shaped-path refinement (Phase-4 audit). The worst-ratio stretch factor is derived
# from the base plan's exact 1/Tⁿ leg-peak scaling, but the superposed lean terms
# scale FASTER (tilt-rate ∝ base jerk ∝ 1/T³, tilt-curvature ∝ base snap ∝ 1/T⁵), so
# from a tight failing T the factor OVER-corrects and the first passing T overshoots
# the true minimum badly (measured x+20 @ gain 0.3: 0.2 s fails → 1.758 s passes, a
# 3.1× overshoot of the 0.563 s true minimum). A few bisections between the last
# failing T and the first passing T recover the honest minimum (~0.59 s here, <5 %
# over) at the cost of a handful of extra validate() passes — shaper-path only; the
# plain path's ~5 % overshoot needs no refinement.
_SHAPED_REFINE_ITERS = 4


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


def build_move(state0, target_pose, duration_s, limits, geom, *, shaper=None):
    """Profiled point-to-point move from ``state0`` to ``target_pose`` at rest.

    ``state0`` is ``(pose, twist, accel)`` (the seed continues the previous plan's
    sampled state — C2 by construction). The move ends at rest (zero twist/accel)
    at ``target_pose``.

    ``shaper`` (optional, Phase 4) is a ``shaping.LeanShaper``: when supplied with a
    positive gain it superposes lean tilt + lever-arm compensation on every
    candidate plan **before** :func:`validate` runs, so the gate always measures the
    SHAPED leg peaks and the duration-stretch loop sizes the shaped motion. A None
    shaper (or a zero-gain one) is the identity — the default, lean OFF.

    Returns ``(plan, report)`` — the accepting :class:`FeasibilityReport` is handed
    back alongside the plan so the caller records the measured leg peaks (for
    ``trajectory/diagnostics``) WITHOUT re-running the ~350 ms gate a second time.

    ``duration_s`` semantics:
      * ``None`` or ``<= 0`` — use the **minimal feasible** duration (the
        duration-stretch loop finds the shortest T the gate accepts).
      * ``> 0`` — validate the REQUESTED duration first and honour it if the gate
        accepts it. Only on a failure that a longer T could fix does the stretch
        loop run, to populate ``min_duration_s`` for a **loud** ``TOO_FAST``
        rejection. A too-tight duration is loudly rejected, never silently
        stretched — the operator asked for a specific timing.

    Validating the requested duration first (rather than comparing it to the
    stretch loop's ``t_min``) is deliberate: ``t_min`` carries a 1.05 stretch
    margin, so a requested duration BELOW ``t_min`` but which the gate still
    accepts must not be false-rejected. The gate itself is the source of truth.

    Raises :class:`TrajectoryInfeasible` immediately (no stretch) on a spatial
    ``WORKSPACE`` / ``UNREACHABLE`` failure, which a longer duration cannot fix.
    """
    pose, twist, accel = (_as6(state0[0], 'pose'),
                          _as6(state0[1], 'twist'),
                          _as6(state0[2], 'accel'))
    target = _as6(target_pose, 'target_pose')

    if duration_s is None or float(duration_s) <= 0.0:
        _t_min, plan_min, report_min = _min_feasible_move(
            pose, twist, accel, target, limits, geom, shaper=shaper)
        return plan_min, report_min

    # Explicit requested duration: validate it directly. If the gate accepts it we
    # are done — no stretch loop, no false-reject on the 1.05 margin.
    requested = float(duration_s)
    plan = _build_rest_move(pose, twist, accel, target, requested, shaper=shaper)
    report = validate(plan, limits, geom)
    if report.ok:
        return plan, report

    # The requested duration failed. A spatial failure (WORKSPACE / UNREACHABLE) is
    # not fixable by a longer T — re-raise it immediately. A stretchable failure
    # means the request was too tight: run the stretch loop only now, to find and
    # advertise the minimal feasible duration in a loud TOO_FAST.
    if report.code not in _STRETCHABLE:
        raise TrajectoryInfeasible(report.code, report.reasons)
    t_min, _plan_min, _report_min = _min_feasible_move(
        pose, twist, accel, target, limits, geom, shaper=shaper)
    raise TrajectoryInfeasible(
        TOO_FAST,
        [f"requested duration {requested:.3f}s < minimal feasible "
         f"{t_min:.3f}s for this move at the current limits"],
        min_duration_s=t_min)


def _build_rest_move(pose, twist, accel, target, duration_s, *, shaper=None):
    seg = QuinticSegment(
        p0=pose, v0=twist, a0=accel,
        p1=target, v1=np.zeros(POSE_DIM), a1=np.zeros(POSE_DIM),
        duration=float(duration_s))
    plan = TrajectoryPlan(segments=(seg,), final_pose=target)
    # Shaping runs BEFORE validate (the canonical ordering invariant): the gate
    # must always see the shaped plan, so wrap here — every candidate the stretch
    # loop validates is shaped, and the loop sizes the shaped motion.
    return shaper.shape(plan) if shaper is not None else plan


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


def _min_feasible_move(pose, twist, accel, target, limits, geom, *, shaper=None):
    """Shortest duration the gate accepts for a rest-terminating move.

    Starts at the ``min_move_duration_s`` floor and stretches by the exact
    worst-ratio factor until feasible (≤2 iters from a rest start). Returns
    ``(duration_s, plan, report)`` — the accepting report rides along so callers
    avoid a redundant ``validate``. Raises :class:`TrajectoryInfeasible` on a
    spatial failure (immediately) or if the loop fails to converge (``TOO_FAST``).

    With ``shaper`` supplied, every candidate is the SHAPED plan (lean superposed
    before validate); the shaped move still shrinks monotonically with T, so the loop
    still converges. But the lean terms scale faster than the 1/Tⁿ the stretch factor
    assumes, so from a tight failing T the first passing T OVERSHOOTS the true minimum
    (~3× measured); when a stretch actually happened, a bisection refinement recovers
    the honest minimum (see :data:`_SHAPED_REFINE_ITERS`). The plain path keeps its
    exact one-step convergence — no refinement.
    """
    T = float(limits.min_move_duration_s)
    report = None
    t_fail = None    # the largest T the gate rejected (a stretchable failure)
    shaped = shaper is not None and getattr(shaper, 'gain', 0.0) > 0.0
    for _ in range(_MAX_STRETCH_ITERS):
        plan = _build_rest_move(pose, twist, accel, target, T, shaper=shaper)
        report = validate(plan, limits, geom)
        if report.ok:
            # Shaped move that required at least one stretch: the passing T likely
            # overshoots (lean terms scale faster than the stretch factor assumes).
            # Bisect [t_fail, T] to recover the honest minimum. The plain path (and a
            # shaped move that passed on the very first T) returns unrefined.
            if shaped and t_fail is not None:
                return _refine_shaped_min(
                    pose, twist, accel, target, limits, geom, shaper,
                    t_fail, T, plan, report)
            return T, plan, report
        if report.code not in _STRETCHABLE:
            # Spatial failure — a longer duration cannot fix it.
            raise TrajectoryInfeasible(report.code, report.reasons)
        t_fail = T
        T *= _stretch_factor(report, limits)

    raise TrajectoryInfeasible(
        TOO_FAST,
        [f"could not find a feasible duration after {_MAX_STRETCH_ITERS} "
         f"iterations (last failure {report.code if report else 'n/a'})"],
        min_duration_s=T)


def _refine_shaped_min(pose, twist, accel, target, limits, geom, shaper,
                       t_fail, t_pass, pass_plan, pass_report):
    """Bisect the shaped min-feasible duration in ``[t_fail, t_pass]``.

    ``t_fail`` is the largest duration the gate REJECTED and ``t_pass`` the first it
    ACCEPTED, so the monotone gate boundary (any T ≥ true-min passes) is bracketed.
    Runs ``_SHAPED_REFINE_ITERS`` bisections, keeping the smallest PASSING candidate
    (and its plan+report so the caller still skips a redundant ~350 ms validate).
    Shaper-path only — see :data:`_SHAPED_REFINE_ITERS` for why the plain path never
    needs this.
    """
    best_T, best_plan, best_report = t_pass, pass_plan, pass_report
    lo, hi = float(t_fail), float(t_pass)
    for _ in range(_SHAPED_REFINE_ITERS):
        mid = 0.5 * (lo + hi)
        plan = _build_rest_move(pose, twist, accel, target, mid, shaper=shaper)
        report = validate(plan, limits, geom)
        if report.ok:
            best_T, best_plan, best_report = mid, plan, report
            hi = mid
        else:
            lo = mid
    return best_T, best_plan, best_report


def _gate(plan, limits, geom) -> None:
    """Run the canonical gate; raise :class:`TrajectoryInfeasible` on failure."""
    report = validate(plan, limits, geom)
    if not report.ok:
        raise TrajectoryInfeasible(
            report.code, report.reasons, report.min_duration_s)


# ═══════════════════════════════════════════════════════════════════════════
# Phase 3 — follower + graceful stop (the fast gate path)
# ═══════════════════════════════════════════════════════════════════════════

# Bound on the follower's per-tick stretch. From a MOVING seed the exact 1/Tⁿ
# scaling does not hold, so a couple of extra iterations may be needed; each is a
# ~1.6 ms validate_follow call, so a small cap keeps the whole per-tick replan well
# inside the 25 ms emit budget. On non-convergence the follower keeps its last
# valid plan (build_follow raises TOO_FAST, which the follower swallows).
_MAX_FOLLOW_ITERS = 6


def build_follow(state0, target_pose, limits, geom, horizon_s):
    """A streaming-follower move toward ``target_pose`` over ~``horizon_s``.

    Used by the SpaceMouse/GUI follower (``follower.py``), which calls this every
    40 Hz tick from the CURRENT commanded state (a MOVING seed — C2 by
    construction). Gated by the **fast** :func:`validate_follow`, not the ~377 ms
    analytic :func:`validate`, so the whole replan fits the emit budget.

    Horizon semantics mirror the plan's ``max(min_feasible, horizon_s)``: build at
    ``horizon_s`` (floored at ``min_move_duration_s``); if the fast gate rejects it
    with a stretchable (vel/acc/jerk/step) failure, stretch the duration up toward
    the minimal feasible one and re-gate (bounded iterations). A spatial
    ``WORKSPACE`` / ``UNREACHABLE`` failure is re-raised immediately (a longer
    horizon cannot reach an unreachable target — the follower's saturation clamp is
    what keeps the target reachable). Returns ``(plan, report)``.
    """
    pose, twist, accel = (_as6(state0[0], 'pose'),
                          _as6(state0[1], 'twist'),
                          _as6(state0[2], 'accel'))
    target = _as6(target_pose, 'target_pose')
    T = max(float(horizon_s), float(limits.min_move_duration_s))
    report = None
    for _ in range(_MAX_FOLLOW_ITERS):
        plan = _build_rest_move(pose, twist, accel, target, T)
        report = validate_follow(plan, limits, geom)
        if report.ok:
            return plan, report
        if report.code not in _STRETCHABLE:
            raise TrajectoryInfeasible(report.code, report.reasons)
        T *= _stretch_factor(report, limits)
    raise TrajectoryInfeasible(
        TOO_FAST,
        [f"follower could not find a feasible horizon in {_MAX_FOLLOW_ITERS} "
         f"iterations (last failure {report.code if report else 'n/a'})"],
        min_duration_s=T)


def build_timed(state0, target_pose, target_twist, duration_s, limits, geom, *,
                hold_after=True, neutral_pose=None):
    """A timed target: reach ``target_pose`` at ``target_twist`` after exactly
    ``duration_s`` seconds (the arrival lead), then come to rest.

    Unlike :func:`build_move`, the duration is **fixed** — the arrival lead is the
    operator's (or the catch's) hard timing constraint, so the plan is NEVER
    silently stretched to arrive late. A too-tight lead is loudly rejected
    (``TOO_FAST``) with the minimal feasible lead in ``min_duration_s``, never
    quietly slowed.

    Boundary conditions: the reach segment starts at ``state0``
    (``(pose, twist, accel)`` — the seed continues the previous plan's sampled
    state, C2 by construction) and ends at ``(target_pose, target_twist,
    zero-accel)`` at ``t = duration_s``. That segment end is the timing-accuracy
    -critical knot; the arrival pose error is zero by construction and the emitted
    knot nearest ``t_arrival`` is within one 25 ms knot of the target.

    **Rest-termination is a safety invariant (non-negotiable).** The plan ALWAYS
    ends at rest. An implicit terminal hold snaps twist to zero, so a final segment
    with a nonzero end velocity would be a velocity discontinuity ⇒ unbounded leg
    jerk ⇒ dangerous hardware jerk (the exact class ``CLAUDE.md`` warns about).
    Therefore, when the arrival velocity is nonzero, a decel-to-rest continuation
    (the audited :func:`build_graceful_stop` primitive) is appended after the
    arrival knot.

    ``hold_after``:
      * ``True`` — hold at the target after arriving (for a moving arrival: decel to
        rest first). The catch path uses this (arrive at rest, hold quiescent).
      * ``False`` — after arriving (and coming to rest), profile back to
        ``neutral_pose`` and hold there (a "reach out, then return" one-shot);
        ``neutral_pose`` is REQUIRED when ``False``.

    Gated by the **fast** :func:`validate_follow`, not the ~377 ms analytic
    :func:`validate`. The timed-target service supersedes an in-flight plan, so the
    gate must run in a few ms for the plan to install shortly after its seed sample —
    single-digit ms typical, guard-bounded (the caller's install-continuity check
    rejects ``STALE_STATE`` on a seed drift > 0.06 rev), worst case tens of ms with an
    appended stop-stretch. The drift over that window is ≪ the pump/firmware step
    gates, so the supersede is a clean C2 replan rather than a u0 jump (the TOCTOU
    class the Phase-2 install-continuity guard closed on the ~377 ms path). Timed
    plans are never lean-shaped, so ``validate_follow`` (shaping-blind) is exactly
    right. Returns ``(plan, report)``.
    """
    pose, twist, accel = (_as6(state0[0], 'pose'),
                          _as6(state0[1], 'twist'),
                          _as6(state0[2], 'accel'))
    target = _as6(target_pose, 'target_pose')
    v1 = _as6(target_twist, 'target_twist')
    if not hold_after and neutral_pose is None:
        raise ValueError("build_timed(hold_after=False) requires neutral_pose")

    D = float(duration_s)
    lead_floor = float(limits.min_timed_lead_s)
    lead_ceiling = float(limits.max_timed_lead_s)
    _zero = np.zeros(POSE_DIM)

    # Hard lead CEILING (clock-domain guard): a lead beyond max_timed_lead is almost
    # always a clock-domain confusion — e.g. an absolute epoch timestamp (~1.75e9 s)
    # reaching here as a "lead". Building the reach at that duration would size the
    # knot-step sampling at ~D/knot_dt elements (np.arange(~7e10) → MemoryError; even a
    # finite hour-scale lead stalls the executor ~30 s). Reject BEFORE building anything.
    if D > lead_ceiling:
        raise TrajectoryInfeasible(
            TOO_FAST,
            [f"arrival lead {D:.1f}s exceeds maximum timed lead {lead_ceiling:.1f}s "
             f"— wrong clock domain?"],
            min_duration_s=lead_ceiling)

    # Hard lead floor: below min_timed_lead there is not enough time for ANY move —
    # reject before building anything, advertising the minimal achievable lead.
    if D < lead_floor:
        t_min = _min_feasible_timed(pose, twist, accel, target, v1, limits, geom)
        raise TrajectoryInfeasible(
            TOO_FAST,
            [f"arrival lead {D:.3f}s < minimum timed lead {lead_floor:.3f}s"],
            min_duration_s=max(t_min, lead_floor))

    # Gate the timing-critical reach FIRST (as a single-segment plan), so a
    # too-tight arrival is rejected TOO_FAST before any (free-duration) continuation
    # is built. A spatial failure (WORKSPACE / UNREACHABLE) is not a timing problem —
    # re-raise it as-is.
    reach = QuinticSegment(p0=pose, v0=twist, a0=accel,
                           p1=target, v1=v1, a1=_zero, duration=D)
    reach_report = validate_follow(TrajectoryPlan((reach,), target), limits, geom)
    if not reach_report.ok:
        if reach_report.code not in _STRETCHABLE:
            raise TrajectoryInfeasible(reach_report.code, reach_report.reasons)
        t_min = _min_feasible_timed(pose, twist, accel, target, v1, limits, geom)
        raise TrajectoryInfeasible(
            TOO_FAST,
            [f"arrival lead {D:.3f}s < minimal feasible {t_min:.3f}s to reach the "
             f"target at this velocity under the current limits"],
            min_duration_s=t_min)

    # Reach feasible — assemble the rest-terminating continuation.
    segments = [reach]
    if not _is_at_rest(v1, _zero):
        # Nonzero arrival velocity: append a decel-to-rest. build_graceful_stop
        # decelerates in place at the target (its segments continue C2 off the
        # reach: v0 = v1, a0 = 0 = reach's a1) and is itself validate_follow-gated
        # and duration-stretched, so the whole plan stays rest-terminating & smooth.
        stop = build_graceful_stop((target, v1, _zero), limits, geom)
        segments.extend(stop.segments)   # HoldPlan (0 segments) impossible: moving
        rest_pose = stop.final_pose
    else:
        rest_pose = target

    final_pose = rest_pose
    if not hold_after:
        # Profiled return to neutral from the (now at-rest) pose. Reuse the
        # validate_follow-based rest-to-rest builder so build_timed stays fast.
        ret_plan, _ = build_follow(
            (rest_pose, _zero, _zero), _as6(neutral_pose, 'neutral_pose'),
            limits, geom, limits.min_move_duration_s)
        segments.extend(ret_plan.segments)
        final_pose = ret_plan.final_pose

    plan = TrajectoryPlan(tuple(segments), final_pose)
    # Single-gate contract: gate the ASSEMBLED plan (the exact object the emitter
    # runs). Each piece was gated feasible individually and the joins are C2, so
    # this confirms the whole; a residual stretchable failure can only be the
    # fixed-duration reach interacting at a join — surface it TOO_FAST with the
    # minimal feasible lead rather than accepting an over-limit plan.
    report = validate_follow(plan, limits, geom)
    if not report.ok:
        if report.code in _STRETCHABLE:
            t_min = _min_feasible_timed(pose, twist, accel, target, v1, limits, geom)
            raise TrajectoryInfeasible(TOO_FAST, report.reasons, min_duration_s=t_min)
        raise TrajectoryInfeasible(report.code, report.reasons)
    return plan, report


def _min_feasible_timed(pose, twist, accel, target, v1, limits, geom):
    """Smallest arrival lead the fast gate accepts for the reach to ``(target, v1)``.

    Stretches the reach duration (``v1`` FIXED — the arrival velocity is a hard BC;
    a longer lead only lowers the leg peaks) from the ``min_timed_lead_s`` floor
    until :func:`validate_follow` passes. Bounded iterations (a moving seed breaks
    the exact 1/Tⁿ leg-peak scaling the stretch factor assumes, so a couple of extra
    passes may be needed); returns the best duration found. Only the reach is gated
    here — the decel/return continuations are free-duration and never constrain the
    arrival lead.
    """
    T = float(limits.min_timed_lead_s)
    _zero = np.zeros(POSE_DIM)
    report = None
    for _ in range(_MAX_FOLLOW_ITERS):
        reach = QuinticSegment(p0=pose, v0=twist, a0=accel,
                               p1=target, v1=v1, a1=_zero, duration=T)
        report = validate_follow(TrajectoryPlan((reach,), target), limits, geom)
        if report.ok:
            return T
        if report.code not in _STRETCHABLE:
            raise TrajectoryInfeasible(report.code, report.reasons)
        T *= _stretch_factor(report, limits)

    # Loop exhausted without a clean pass (a moving seed breaks the exact 1/Tⁿ
    # leg-peak scaling the stretch factor assumes). The final T from the last
    # iteration was multiplied but NEVER re-validated, so validate it once more; if it
    # still fails-stretchable, apply one more stretch so the advertised minimum is a
    # genuine (best-effort) upper bound on the feasible lead rather than an unvalidated
    # guess. Kept to a single extra pass — this is only ever an advertised retry lead.
    reach = QuinticSegment(p0=pose, v0=twist, a0=accel,
                           p1=target, v1=v1, a1=_zero, duration=T)
    final = validate_follow(TrajectoryPlan((reach,), target), limits, geom)
    if not final.ok and final.code in _STRETCHABLE:
        T *= _stretch_factor(final, limits)
    return T


def build_graceful_stop(state0, limits, geom, *, start_duration_s=None):
    """A duration-stretched decel-to-rest at the current pose.

    Unlike :func:`build_hold` (which builds ONE min-duration decel and RAISES if
    that decel is too aggressive for the gate at a high velocity), this stretches
    the stop's horizon until the gate passes. Because it decelerates **in place**
    (``p1 == p0``, the current reachable pose), the only possible *timing* failures
    are the stretchable vel/acc/jerk/step ones — a longer stop is monotonically
    gentler — so it **converges for all gate-limited seeds** by lengthening the
    horizon.

    It is **not** unconditionally always-valid, though: the decel overshoots the
    seed pose by ~0.2·v·T (the excursion of a rest-terminating quintic), which
    *grows* with the stretched horizon T. So a seed whose boundary margin is smaller
    than that overshoot fails the gate's stroke check spatially — no stretch fixes it
    (a longer stop overshoots more). This raises :class:`TrajectoryInfeasible`
    (``WORKSPACE``); the caller keeps its current gated, rest-terminating plan and
    retries from the (decaying-velocity) live state, which converges within a few
    ticks as the boundary margin grows. A future structural fix is a *retargeted*
    stop (``p1 = p0 + alpha·v0``, decelerating along the motion, stop-soonest) that
    never overshoots outward — deferred.

    This is the primitive the plan/task require for the two "stop now" cases:
    leaving a streaming mode mid-move (STANDBY-exit) and follower input-loss. Gated
    by the fast :func:`validate_follow` so it is cheap enough to run on the emitter
    thread (input-loss) without threatening the 250 ms staleness window. Returns the
    stop :class:`TrajectoryPlan` (a :class:`HoldPlan` when the seed is already at
    rest).
    """
    pose, twist, accel = (_as6(state0[0], 'pose'),
                          _as6(state0[1], 'twist'),
                          _as6(state0[2], 'accel'))
    if _is_at_rest(twist, accel):
        # At rest: the stop is a plain hold at the seed pose. Gate it exactly as
        # build_hold gates the identical case — an out-of-stroke / non-finite seed
        # must be rejected here, not returned as an ungated HoldPlan (which would
        # otherwise reach the emitter unchecked — the finding this closes).
        plan = HoldPlan(pose)
        report = validate_follow(plan, limits, geom)
        if not report.ok:
            raise TrajectoryInfeasible(report.code, report.reasons)
        return plan

    T = float(start_duration_s if start_duration_s is not None
              else limits.min_move_duration_s)
    report = None
    # Generous cap: the stop is a decel-in-place, so it converges monotonically;
    # the cap only guards a pathological (e.g. absurd-velocity) seed.
    for _ in range(24):
        seg = QuinticSegment(
            p0=pose, v0=twist, a0=accel,
            p1=pose, v1=np.zeros(POSE_DIM), a1=np.zeros(POSE_DIM),
            duration=T)
        plan = TrajectoryPlan(segments=(seg,), final_pose=pose)
        report = validate_follow(plan, limits, geom)
        if report.ok:
            return plan
        if report.code not in _STRETCHABLE:
            # A decel-in-place cannot fail spatially (p1 == p0 is the seed's own
            # reachable pose); if it somehow does, surface it rather than loop.
            raise TrajectoryInfeasible(report.code, report.reasons)
        T *= _stretch_factor(report, limits)
    raise TrajectoryInfeasible(
        TOO_FAST,
        [f"graceful stop did not converge in 24 iterations "
         f"(last {report.code if report else 'n/a'})"],
        min_duration_s=T)
