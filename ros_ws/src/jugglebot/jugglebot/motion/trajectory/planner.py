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

from jugglebot.motion.ik_solver import compute_jacobian, rotvec_to_rot_matrix
from jugglebot.motion.trajectory.feasibility import (
    LIMIT_ACC,
    LIMIT_JERK,
    LIMIT_VEL,
    STEP_BOUND,
    STEP_BOUND_MARGIN,
    TOO_FAST,
    WORKSPACE,
    TrajectoryInfeasible,
    validate,
    validate_follow,
)
from jugglebot.motion.trajectory import retime
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

# Retiming-model diagnostics (Phase 2, shaped-planning-efficiency). The model path
# proposes a duration and the ONE verify pass through the unchanged validate() is the
# source of truth; on the ~never verify rejection (or a model that cannot bracket a
# feasible T) the planner falls back to the proven legacy stretch+bisection loop.
# These counters make that fallback observable (a persistent nonzero count on the
# hardware A/B would flag a model pathology) without changing behaviour.
_RETIME_STATS = {'model_used': 0, 'verify_rejected': 0, 'model_no_solution': 0}


def _as6(vec, name: str) -> np.ndarray:
    a = np.asarray(vec, dtype=float)
    if a.shape != (POSE_DIM,):
        raise ValueError(f"{name} must be shape ({POSE_DIM},), got {a.shape}")
    return a


def _as_tilt2(vec, name: str) -> np.ndarray:
    """A two-component ``(rx, ry)`` tilt, in radians.

    Strict about the shape on purpose: the natural mistakes are handing over a
    3-component rotation vector or a whole 6-DOF pose, and either would silently
    aim a catch's through-seat off the wrong two numbers — the exact class of frame
    error C-CATCH-1 exists to close.
    """
    a = np.asarray(vec, dtype=float)
    if a.shape != (2,):
        raise ValueError(f"{name} must be shape (2,) — the gravity-referenced "
                         f"(rx, ry) receive tilt in rad — got {a.shape}")
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

    ``neutral_pose`` is the neutral pose the caller supplies. With a levelling
    correction loaded that is the **corrected** active pose, not the bare
    ``(0, 0, 170, 0, 0, 0)`` — ``trajectory_node`` corrects it at use (contract
    C-LEVEL-1, ``ros_ws/docs/levelling_frame.md``). So ``trajectory/go_home`` from
    the held active pose is a genuine no-op **only when the correction is
    identity**; with the 2026-07-25 session offset loaded it is a real ~2.77 mm
    worst-leg move. Do not reason from "go_home is a no-op" when diagnosing
    unexplained motion — that inference is what sent the 2026-07-25 self-toss
    investigation looking in the wrong place.

    If the seed is already at ``neutral_pose`` and at rest the segment is
    degenerate (no motion). Raises :class:`TrajectoryInfeasible` if the move
    violates the gate.
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


def build_move(state0, target_pose, duration_s, limits, geom, *, shaper=None,
               retime_model=True):
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
            pose, twist, accel, target, limits, geom, shaper=shaper,
            retime_model=retime_model)
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
        pose, twist, accel, target, limits, geom, shaper=shaper,
        retime_model=retime_model)
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


def _min_feasible_move(pose, twist, accel, target, limits, geom, *, shaper=None,
                       retime_model=True):
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

    Retiming-model fast path (Phase 2): for a SHAPED move from a REST seed with
    ``retime_model`` on, the minimal feasible duration is proposed by a per-sample
    u-polynomial model (:mod:`retime`) and confirmed by ONE ``validate`` pass instead
    of the 6-pass stretch+bisection search. The model only proposes T — the unchanged
    ``validate`` remains the source of truth — so a verify rejection (or a move the
    model cannot bracket) falls back to the legacy loop below, never returning an
    unvalidated plan. Moving seeds and the plain (unshaped) path always use the legacy
    loop: the exact 1/Tⁿ leg-peak scaling the model exploits holds only for a rest
    seed, and the plain path already converges in ≤2 passes.
    """
    shaped = shaper is not None and getattr(shaper, 'gain', 0.0) > 0.0
    if retime_model and shaped and _is_at_rest(twist, accel):
        result = _try_retime_model(pose, twist, accel, target, limits, geom, shaper)
        if result is not None:
            return result
        # else: model could not propose or the verify rejected — fall through to the
        # proven legacy stretch+bisection loop (the fallback is already counted).

    T = float(limits.min_move_duration_s)
    report = None
    t_fail = None    # the largest T the gate rejected (a stretchable failure)
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


def _try_retime_model(pose, twist, accel, target, limits, geom, shaper):
    """Propose a shaped-move duration via the retiming model, then VERIFY it once.

    Returns ``(T_final, plan, report)`` when the model's proposed duration passes the
    unchanged :func:`validate` (the common case — the model matches the gate boundary
    to ~1 ms and the 1.02 inflation covers the residual). Returns ``None`` — signalling
    the caller to fall back to the legacy loop — when the model cannot propose a
    duration OR the mandatory verify pass rejects the proposed plan (both counted in
    :data:`_RETIME_STATS`). Never returns an unvalidated plan: the single returned
    plan is exactly the object ``validate`` accepted, so the K-contract holds.
    """
    T_model = retime.propose_move_duration(pose, target, limits, geom, shaper)
    if T_model is None:
        _RETIME_STATS['model_no_solution'] += 1
        return None
    plan = _build_rest_move(pose, twist, accel, target, T_model, shaper=shaper)
    report = validate(plan, limits, geom)
    if report.ok:
        _RETIME_STATS['model_used'] += 1
        return T_model, plan, report
    # The model proposed a duration the unchanged gate rejects — fall back to the
    # legacy loop (the caller re-runs the stretch+bisection search, which is the
    # proven source of truth, and re-raises any spatial failure). Only a STRETCHABLE
    # rejection is a genuine model mis-size (should be ~never on the corpus); a spatial
    # WORKSPACE/UNREACHABLE reject is a legitimately-infeasible target, not a model
    # pathology, so it is not counted against the model.
    if report.code in _STRETCHABLE:
        _RETIME_STATS['verify_rejected'] += 1
    return None


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


def build_follow(state0, target_pose, limits, geom, horizon_s, *,
                 max_iters: int = _MAX_FOLLOW_ITERS):
    """A streaming-follower move toward ``target_pose`` over ~``horizon_s``.

    Used by the SpaceMouse/GUI follower (``follower.py``), which calls this every
    40 Hz tick from the CURRENT commanded state (a MOVING seed — C2 by
    construction). Gated by the **fast** :func:`validate_follow`, not the ~377 ms
    analytic :func:`validate`, so the whole replan fits the emit budget.

    Horizon semantics mirror the plan's ``max(min_feasible, horizon_s)``: build at
    ``horizon_s`` (floored at ``min_move_duration_s``); if the fast gate rejects it
    with a stretchable (vel/acc/jerk/step) failure, stretch the duration up toward
    the minimal feasible one and re-gate (up to ``max_iters`` iterations). A spatial
    ``WORKSPACE`` / ``UNREACHABLE`` failure is re-raised immediately (a longer
    horizon cannot reach an unreachable target — the follower's saturation clamp is
    what keeps the target reachable). Returns ``(plan, report)``.

    ``max_iters`` caps the build-validate-stretch loop. The chase follower passes a
    small value (3): the chase already sizes the horizon so the FIRST validate
    nearly always passes, and a stretch factor derived from a decel-dominated
    moving-seed candidate under-corrects, so the extra iters are belt-and-braces
    (each is a ~1.6 ms ``validate_follow``, kept well inside the emit budget). Other
    callers keep the default ``_MAX_FOLLOW_ITERS``.
    """
    pose, twist, accel = (_as6(state0[0], 'pose'),
                          _as6(state0[1], 'twist'),
                          _as6(state0[2], 'accel'))
    target = _as6(target_pose, 'target_pose')
    T = max(float(horizon_s), float(limits.min_move_duration_s))
    report = None
    for _ in range(int(max_iters)):
        plan = _build_rest_move(pose, twist, accel, target, T)
        report = validate_follow(plan, limits, geom)
        if report.ok:
            return plan, report
        if report.code not in _STRETCHABLE:
            raise TrajectoryInfeasible(report.code, report.reasons)
        T *= _stretch_factor(report, limits)
    raise TrajectoryInfeasible(
        TOO_FAST,
        [f"follower could not find a feasible horizon in {int(max_iters)} "
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


# Decel-in-place base peak coefficients (the rest-terminating quintic basis
# extrema, numerically verified 2026-07-09; mirrors chase._B_ACC_V / _B_JERK_V —
# duplicated here rather than imported so the gate layer stays independent of the
# advisory chase layer). Used only to SEED the stop-stretch horizon (A4); the gate
# is the source of truth for feasibility.
_DECEL_ACC_V = 3.941     # peak base acc per (v_pk / T)
_DECEL_JERK_V = 36.0     # peak base jerk per (v_pk / T²)
# Bleed the energy-informed start below the true feasible T so the ladder still
# stretches UP to it (never overshoots downward). An OVER-estimated start would
# permanently inflate the decel's outward overshoot (~0.2·v·T grows with T) and can
# flip a convergent near-boundary stop into a WORKSPACE deferral (A4).
_DECEL_START_BLEED = 1.3

# Default cap on the stop-stretch loop. Generous: the stop is a decel-in-place, so
# it converges monotonically; the cap only guards a pathological (absurd-velocity)
# seed. The node's per-tick retry passes a small value so the post-publish block
# stays bounded (the retry resumes next tick from the decayed seed).
_MAX_STOP_ITERS = 24


def _decel_energy_start(pose, twist, accel, limits, geom) -> float:
    """Energy-informed START horizon for :func:`build_graceful_stop` (A4).

    Estimates the decel-in-place duration from the seed's frozen-J leg-space peak
    velocity via the base-quintic acc/jerk extrema, then UNDER-estimates it
    (``/ _DECEL_START_BLEED``) so the ladder stretches up to — never past — the true
    feasible T (an over-estimate inflates the outward overshoot). Floored at
    ``min_move_duration_s``. Uses the DECEL forms only (never the rest-to-rest
    1.875 delta form): the stop has zero net motion, so its peaks are the base's.
    """
    J = compute_jacobian(pose[:3], rotvec_to_rot_matrix(pose[3:6]), geom)
    v_pk = float(np.max(np.abs(J @ twist)))
    a_lim = max(float(limits.leg_acc_mmps2), 1e-9)
    j_lim = max(float(limits.leg_jerk_mmps3), 1e-9)
    t_est = max(_DECEL_ACC_V * v_pk / a_lim,
                float(np.sqrt(_DECEL_JERK_V * v_pk / j_lim)))
    return max(float(limits.min_move_duration_s), t_est / _DECEL_START_BLEED)


def _stretch_stop(pose, twist, accel, T, limits, geom, max_iters):
    """Stretch a decel-in-place stop from start horizon ``T`` until the gate passes.

    Raises :class:`TrajectoryInfeasible` on a spatial (``WORKSPACE``) failure — the
    decel overshoot exceeds the seed's boundary margin and GROWS with T, so no
    stretch fixes it — or ``TOO_FAST`` if the loop exhausts ``max_iters``.
    """
    report = None
    for _ in range(int(max_iters)):
        seg = QuinticSegment(
            p0=pose, v0=twist, a0=accel,
            p1=pose, v1=np.zeros(POSE_DIM), a1=np.zeros(POSE_DIM),
            duration=T)
        plan = TrajectoryPlan(segments=(seg,), final_pose=pose)
        report = validate_follow(plan, limits, geom)
        if report.ok:
            return plan
        if report.code not in _STRETCHABLE:
            # A decel-in-place fails spatially only by overshooting the boundary
            # (~0.2·v·T, growing with T) — surface it rather than loop.
            raise TrajectoryInfeasible(report.code, report.reasons)
        T *= _stretch_factor(report, limits)
    raise TrajectoryInfeasible(
        TOO_FAST,
        [f"graceful stop did not converge in {int(max_iters)} iterations "
         f"(last {report.code if report else 'n/a'})"],
        min_duration_s=T)


def build_graceful_stop(state0, limits, geom, *, start_duration_s=None,
                        max_iters: int = _MAX_STOP_ITERS):
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

    Start horizon (A4): with no explicit ``start_duration_s`` the loop starts from
    an *energy-informed* estimate (:func:`_decel_energy_start`) so an over-energetic
    seed converges in ≤ ~3 validates instead of climbing the whole ladder from
    ``min_move_duration_s`` (the ~100 ms in-loop stall that fed the S3 146 ms emit
    gap). The estimate deliberately UNDER-shoots the true feasible T so it never
    inflates the overshoot. Because a too-large start can push the overshoot past a
    tight boundary margin (WORKSPACE) where a SMALLER start would have passed, a
    WORKSPACE raise triggers a ONE-shot fallback: retry from the plain
    ``min_move_duration_s`` ladder start (smallest overshoot) before re-raising.

    ``max_iters`` caps the stretch loop (default ``_MAX_STOP_ITERS`` = 24). The
    node's per-tick pending-stop retry passes a small value (~6) so the post-publish
    block stays far under the ~117 ms firmware decay→sprint threshold; the retry
    resumes next tick from the decayed seed.

    This is the primitive the plan/task require for the "stop now" cases (leaving a
    streaming mode mid-move, follower input-loss, and the chase follower's A2
    over-energetic-seed routing). Gated by the fast :func:`validate_follow` so it is
    cheap enough to run on the emitter thread. Returns the stop
    :class:`TrajectoryPlan` (a :class:`HoldPlan` when the seed is already at rest).
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

    min_start = float(limits.min_move_duration_s)
    if start_duration_s is not None:
        T0 = float(start_duration_s)
    else:
        T0 = _decel_energy_start(pose, twist, accel, limits, geom)
    try:
        return _stretch_stop(pose, twist, accel, T0, limits, geom, max_iters)
    except TrajectoryInfeasible as e:
        # One-shot fallback (A4): a start above min_move can overshoot a tight
        # boundary margin (WORKSPACE) where the plain min_move ladder — smaller
        # overshoot per candidate — still passes. Retry once before re-raising.
        if e.code == WORKSPACE and T0 > min_start:
            return _stretch_stop(pose, twist, accel, min_start, limits, geom,
                                 max_iters)
        raise


# ═══════════════════════════════════════════════════════════════════════════
# Phase 6 — the catch trajectory (reach / tilt-through-seat / quiescent hold)
# ═══════════════════════════════════════════════════════════════════════════

# Residual tilt rate at the catch arrival (rad/s), in the receive-tilt-increasing
# direction, that "ramps the tilt through the seat" — a parked tilted rim deflects
# the ball (the bb-sim geometry finding), so the tilt would still be MOVING through
# the seat angle at the instant of contact, then decayed to rest over
# `tilt_decay_s`.
#
# ── SHIPPED AT ZERO since 2026-07-26 (operator decision) ─────────────────────
# The planner MANUFACTURES nothing. With this at 0.0 a catch that nobody gave an
# opinion about arrives at its target with zero twist, so every catch is
# reach + quiescent hold and the platform is stationary at ball contact and
# through the throw that follows it.
#
# This is a DEFAULT, not a rule, and the distinction is the whole point. Platform
# motion during a catch or a throw is PERMITTED — it is simply never MANDATED by a
# constant in the trajectory builder. The seam that makes it permitted is
# `build_catch(tilt_through_rate_radps=...)`: an explicitly-supplied rate is
# REQUESTED motion, returned verbatim and unbounded by `_catch_arrival_rate`, and
# that is what a future optimising planner uses when it decides a moving rim
# catches better. Nothing here asserts a catch commands no motion; setting the
# manufactured fallback to zero is just the cleanest way to say "the builder has no
# opinion of its own".
#
# WHY 0.07 EXISTED, AND WHY ZEROING IT IS NOT FREE. The bb-sim geometry finding
# above is about a parked *tilted* rim. A gravity-level catch (the self-toss path)
# seats at zero tilt, so a zero rate costs it nothing — under C-CATCH-1 its
# receive-tilt magnitude is already zero and the through-seat never engaged. The
# RELOAD catch is the one that pays: it seats at 11.08° and its rim is now
# stationary at contact. Two facts that bound that risk, neither of which removes
# it: (a) 0.07 rad/s has NEVER been validated on hardware — it is an estimate, and
# its own leg-velocity note here was 2× wrong until it was measured; (b) until
# commit 407154f the seat was aimed off the PLAN-frame tilt, so on every levelled
# catch it pointed along the levelling correction rather than at the ball — any
# bench impression formed before that commit was formed on a mis-aimed seat.
# Bench-scored by `tests/hardware/session_anomaly_fixes.md` § Section ZSEAT.
#
# TO BRING IT BACK (the seat-tuning session this constant has always anticipated):
# set this to the rate to try and re-run the suite. C-CATCH-1 (`_catch_arrival_rate`)
# then bounds it against the catch's physical scale exactly as before — the contract
# is dormant at 0.0, not removed, and `tests/motion/test_trajectory_planner_catch.py`
# monkeypatches this constant to a non-zero value precisely so the bound stays
# tested while the shipped default cannot reach it.
#
# Sizing notes kept for that session: at 0.07 rad/s (~4°/s) the overshoot over a
# 0.15 s decay is ~0.3°, well inside MAX_TILT (12°) and the hold-quiescence tilt
# budget (<1°), and the leg velocities it induces are negligible against the session
# ceilings — 14.24 mm/s, measured 2026-07-26 through the production gate (an earlier
# estimate of "~plat_radius·rate ≈ 7 mm/s" here was 2× low; the tilt axis is not
# through the platform centre, so the lever arm is not plat_radius).
_CATCH_TILT_THROUGH_RATE_RADPS = 0.0

# Fraction of `rate·decay_s` the tilt overshoots past the seat during the decay
# (the mean-velocity displacement of a rate→0 decel). Only sets the settle pose;
# the exact value is not physically critical (the gate is the backstop).
_CATCH_TILT_OVERSHOOT_FRAC = 0.5

# ── C-CATCH-1 (`ros_ws/docs/catch_arrival_contract.md`) ──────────────────────
# The largest `|v1|·T / |p1 − p0|` for which a rest-seeded quintic never leaves its
# seed pose on the FAR side from its target. Derived from the quintic's own
# geometry, not fitted: with `p(s) = p0 + d·ψ(s) + v1·T·φ(s)` the reach crosses to
# the wrong side of `p0` iff `|v1|T/|d| > min_s ψ(s)/|φ(s)|`, and
# `ψ/|φ| = (10−15s+6s²)/(4−7s+3s²)` is strictly increasing on [0,1) with infimum
# **5/2** as s → 0⁺ (verified numerically 2026-07-26: 2.500000000625 at s = 1e-9,
# and the excursion is exactly 0 at 2.50 / +1.6e-8 at 2.51).
#
# Equivalently, in the excursion units the plan and the replay probe use: the peak
# unrequested excursion `(16/81)·|v1|·T` may not exceed `40/81 ≈ 0.4938` of the
# catch's physical tilt scale (`_catch_scale`). NOTE the 2026-07-26 measurement
# above corrects a claim carried by
# `plans/active/catch-reach-degenerate-overshoot.md` and
# `tools/probes/catch_reach_replay.py` that the sign reverses above `ψ(2/3) =
# 0.790`: that is where the value AT s = 2/3 crosses zero, not where the reach
# first leaves the park the wrong way. The true first crossing is 40/81.
_CATCH_ARRIVAL_RATE_BOUND = 2.5

# The same 40/81, expressed directly as a fraction of the catch scale. Used to
# bound the OTHER departure the arrival rate manufactures — the settle overshoot
# past the seat during the decay — so the contract's two halves share one factor
# and no second tuning constant enters. `(16/81)·(5/2) = 40/81` exactly.
_CATCH_EXCURSION_FRAC_BOUND = 40.0 / 81.0


def _catch_scale(seed_tilt, target_tilt, seat_mag):
    """The catch's physical tilt SCALE (rad) — the denominator C-CATCH-1 bounds against.

    The larger of the two quantities a catch is physically sized by:

    * ``|target_tilt − seed_tilt|`` — how far the rim must *travel* on this plan;
    * ``seat_mag`` — how tilted the rim must *end up* (the receive-tilt magnitude),
      which is the quantity that justifies a nonzero arrival rate at all.

    Taking the MAX, rather than the displacement alone, is load-bearing and was
    established by measurement (2026-07-26, finalize review of the C-CATCH-1
    landing). The displacement-only reading degenerates precisely where its own
    physical meaning evaporates: the bound's reading is "the reach must not leave
    its seed on the far side from its target", which presumes there IS a
    meaningful displacement. On a C2 supersede that is already ON the target — the
    shipping reload path, where ``catch_coordinator._republish_pretilt``
    re-installs the catch every balls tick from ``arrival + settle_hold`` until
    ``arrival − reach_freeze`` — the target *is* the seed, so every nonzero arrival
    velocity is "wrong-side" by definition and a displacement-only bound collapses
    to "no arrival velocity at all".

    Measured consequence of getting this wrong, replayed through this planner on
    the recorded reload geometry of bag ``2026-07-25_15-17-48``: the arrival rate
    of the plan that is actually FROZEN through ball contact fell from
    ``0.070000 rad/s`` (4.011 °/s) to ``0.004460 rad/s`` (0.256 °/s) — a 15.7×
    de-rate that silently deletes the through-seat on the one path that has a real
    seat, at the one instant it exists for. A parked tilted rim deflects the ball
    (the bb-sim geometry finding this whole feature rests on), so that is a
    behaviour change at ball contact, not a diagnostic nicety.

    With the seat magnitude in the max, the reload keeps its full seat (scale
    10.87°, bound 0.200 rad/s ≫ the 0.07 default) while the 2026-07-25 defect
    stays closed by construction: its wire receive tilt was exactly zero, so
    ``seat_mag`` is 0 and the scale is the displacement, unchanged.
    """
    disp = float(np.linalg.norm(np.asarray(target_tilt, dtype=float)
                                - np.asarray(seed_tilt, dtype=float)))
    return max(disp, float(seat_mag))


def _catch_arrival_rate(seed_tilt, target_tilt, seat_mag, duration_s, decay_s,
                        requested_rate):
    """The catch reach's arrival tilt-rate magnitude (rad/s) — the C-CATCH-1 point.

    This is the ONE place a catch plan's arrival twist magnitude is decided, and
    the invariant it enforces is that :func:`build_catch` may not MANUFACTURE
    commanded motion:

    * ``requested_rate is not None`` — the caller specified the arrival twist.
      That is requested motion; it is returned verbatim and is **not** bounded.
      (This is the seam a future planner uses to command a deliberately moving
      platform at ball contact from an optimisation, and the seam the offline
      replay probes use to rebuild a pre-C-CATCH-1 capture faithfully.)
    * ``requested_rate is None`` — the caller has no opinion, so the planner falls
      back to ``_CATCH_TILT_THROUGH_RATE_RADPS``. That rate is the planner's own
      constant, not a request, so every departure from the target it manufactures
      is bounded against the catch's physical scale (:func:`_catch_scale`).

    **The fallback ships at 0.0 (operator decision, 2026-07-26)**, so today this
    function returns 0.0 for every unopinionated caller and the bound below never
    binds. It is kept live, and kept tested, because it is the guard for the moment
    someone raises the constant again — which its own docstring anticipates as a
    seat-tuning session. A bound that is deleted the day it stops binding is a bound
    that will not be there when the value moves; the tests reach it by monkeypatching
    ``_CATCH_TILT_THROUGH_RATE_RADPS``, never by passing ``requested_rate`` (which
    takes the deliberately-unbounded branch above and would prove nothing).

    TWO departures are manufactured by the same rate and BOTH are bounded, by the
    same ``40/81`` factor and with no second free parameter:

    * the **reach** excursion ``(16/81)·|v1|·T`` — the wrong-side swing;
    * the **settle** overshoot ``_CATCH_TILT_OVERSHOOT_FRAC·rate·decay`` — how far
      past the seat the rim drifts while the rate decays to rest, which
      ``hold_after=True`` then holds *through release*.

    Bounding only the reach would leave this contract's own headline evidence
    unenforced: it is the SETTLE residual (0.3008° off gravity, held through
    release) that predicts the session's 16.5 mm throw-direction error, and
    ``decay`` does not appear in the reach bound at all. A future seat-tuning
    session raising ``tilt_decay_s`` from 0.15 s to 0.6 s would quadruple that
    residual with every ``test_ccatch1_*`` still green.

    The failure mode this closes, measured on bag ``2026-07-25_15-17-48``: a
    −0.78° catch target produced a **+2.32° commanded excursion in the opposite
    direction**, peaking 2.0 s before release, because the manufactured arrival
    rate was a constant while the requested displacement was not — the ratio goes
    as ``1/|tilt|`` and so blows up exactly where the request is smallest.

    Bounding the RATE rather than rejecting the plan is deliberate: the violation
    is manufactured by the builder itself, so rejecting would refuse a catch the
    caller asked for perfectly reasonably — no catch at all is strictly worse than
    a rim that rotates through the seat more slowly. Every effect of the bound is a
    REDUCTION in commanded motion.
    """
    rate = float(requested_rate if requested_rate is not None
                 else _CATCH_TILT_THROUGH_RATE_RADPS)
    rate = max(rate, 0.0)
    if requested_rate is not None:
        return rate
    scale = _catch_scale(seed_tilt, target_tilt, seat_mag)
    T = float(duration_s)
    if T <= 0.0:
        return 0.0
    # Reach half: (16/81)·|v1|·T ≤ (40/81)·scale  ⟺  |v1| ≤ (5/2)·scale/T.
    rate = min(rate, _CATCH_ARRIVAL_RATE_BOUND * scale / T)
    # Settle half: frac·rate·decay ≤ (40/81)·scale. Same factor, so no new tuning
    # constant enters the contract; at the shipped decay (0.15 s) this is slack by
    # ~18× on the reload and never binds before the reach half.
    decay = float(decay_s)
    if decay > 0.0:
        rate = min(rate, (_CATCH_EXCURSION_FRAC_BOUND * scale
                          / (_CATCH_TILT_OVERSHOOT_FRAC * decay)))
    return rate


def build_catch(state0, catch_pose, duration_s, limits, geom, *,
                settle_hold_s, tilt_decay_s=0.15, receive_tilt=None,
                tilt_through_rate_radps=None,
                hold_after=True, neutral_pose=None):
    """The reload catch trajectory: reach the catch pose at ``t = duration_s``,
    ramp the tilt through the seat, then hold quiescent (and optionally return).

    ``catch_pose`` is the full ``[x, y, z, rx, ry, rz]`` catch pose the caller
    computed from the observed ball: reach xy at the observed landing, ``z`` at the
    catch height (STOW-relative), and the receive tilt ``(rx, ry)`` from
    :func:`tilt_geometry.tilt_to_receive` (cup axis collinear with the arrival
    velocity). ``rz`` is normally 0. The reach envelope (≤ 80 mm from the held
    pose) is the caller's responsibility to respect; an out-of-stroke catch is
    rejected ``WORKSPACE`` by the gate here regardless.

    ``receive_tilt`` is the **gravity-referenced** receive tilt ``(rx, ry)`` in
    radians — the physical quantity the through-seat aims along, straight from the
    ball's arrival direction. It is a SEPARATE argument from ``catch_pose`` because
    the two are not the same vector once a gravity-levelling correction is loaded:
    ``catch_pose[3:5]`` is then a *plan-frame* tilt carrying the correction, so
    reading the seat direction out of it aims the through-seat along the correction
    and ramps a **gravity-level** catch off level exactly at ball contact
    (``ros_ws/docs/levelling_frame.md`` C-LEVEL-1 rewrites the rotation of every
    external pose; it deliberately does not rewrite this). ``None`` means "the
    catch pose is already gravity-referenced" and falls back to ``catch_pose[3:5]``
    — correct for every caller with no levelling concept (``sim/``), and wrong for
    any caller that has one. A level receive tilt yields a **zero** arrival twist
    and hence a reach with no excursion at all, by construction rather than by a
    ``|tilt| ≈ 0`` threshold.

    ``tilt_through_rate_radps`` is the requested seat rate. ``None`` (the default)
    means the caller has no opinion: the planner falls back to
    ``_CATCH_TILT_THROUGH_RATE_RADPS`` — **which ships at 0.0 since 2026-07-26**, so
    an unopinionated catch arrives with zero twist, carries no decay segment, and
    settles exactly on its target — and **bounds** it under C-CATCH-1
    (:func:`_catch_arrival_rate`), so no departure from the target it manufactures
    — during the reach or during the settle — exceeds ``40/81`` of the catch's
    physical tilt SCALE (:func:`_catch_scale`: the larger of the seed → target
    travel and the receive-tilt magnitude, *not* the travel alone — see that
    function for why). An explicit value is caller-requested motion and is
    honoured verbatim.

    Segments (all C2-joined, gated as one assembled plan):

      1. **Reach** — ``state0`` → ``catch_pose`` over the FIXED ``duration_s`` (the
         arrival lead). Translational velocity at arrival is **always zero**
         (a baked-in safety invariant: *velocity matching is the hand's job*, so
         the platform is translationally still at contact — a moving platform at
         seat would fight the hand's velocity-matched catch). The tilt arrives with
         a residual rate along the *gravity-referenced* receive tilt — zero at the
         shipped default, and zero for a level receive tilt (see below).
      2. **Tilt-through-seat decay** — *present only when the arrival tilt rate is
         non-zero, which at the shipped default means only when a caller asked for
         one.* The residual tilt rate decays to rest over ``tilt_decay_s`` (default
         0.15 s), the tilt drifting a small overshoot past the seat, so at
         ``t = duration_s`` (contact) the rim is still *moving* through the seat
         angle rather than parked. **A default catch has two segments, not three.**
      3. **Quiescent hold** — a literal zero-twist hold at the settled pose for
         ``settle_hold_s`` (the ball seats undisturbed; the hold-quiescence gate
         criterion is measured over this window).
      4. **Return** (``hold_after=False`` only) — a slow profiled return to
         ``neutral_pose`` after the hold.

    **Fixed lead, loud rejection.** Like :func:`build_timed`, the arrival lead is a
    hard constraint — a too-tight ``duration_s`` is rejected ``TOO_FAST`` with the
    minimal feasible lead in ``min_duration_s`` (never silently slowed). A spatial
    ``WORKSPACE`` / ``UNREACHABLE`` failure (out-of-stroke reach or tilt) is
    re-raised as-is (a longer lead cannot reach an unreachable pose).

    **Gate choice — ``validate_follow`` (the fast gate).** The catch is the
    CATCH-mode reach, exactly as the Phase-5 ``catch/dynamic_target`` path
    (``build_timed``) is; using the same fast gate keeps the catch supersede cheap
    (a pre-freeze target update installs a C2 replan shortly after its seed) and the
    catch is *never* lean-shaped, so ``validate_follow``'s shaping-blindness is a
    guard, not a limitation. Its knot-step bound is bit-identical to the analytic
    ``validate``, so every emitted catch knot stays pump-accepted. Returns
    ``(plan, report)``.
    """
    pose, twist, accel = (_as6(state0[0], 'pose'),
                          _as6(state0[1], 'twist'),
                          _as6(state0[2], 'accel'))
    target = _as6(catch_pose, 'catch_pose')
    if not hold_after and neutral_pose is None:
        raise ValueError("build_catch(hold_after=False) requires neutral_pose")

    D = float(duration_s)
    lead_floor = float(limits.min_timed_lead_s)
    lead_ceiling = float(limits.max_timed_lead_s)
    decay = max(float(tilt_decay_s), 0.0)
    hold = max(float(settle_hold_s), 0.0)
    _zero = np.zeros(POSE_DIM)

    # Clock-domain guard (mirrors build_timed): a lead beyond the ceiling is almost
    # always a clock-domain confusion (an absolute epoch reaching here as a "lead"),
    # which would size the knot-step sampling at ~D/knot_dt elements → MemoryError.
    if D > lead_ceiling:
        raise TrajectoryInfeasible(
            TOO_FAST,
            [f"catch lead {D:.1f}s exceeds maximum timed lead {lead_ceiling:.1f}s "
             f"— wrong clock domain?"],
            min_duration_s=lead_ceiling)
    if D < lead_floor:
        raise TrajectoryInfeasible(
            TOO_FAST,
            [f"catch lead {D:.3f}s < minimum timed lead {lead_floor:.3f}s"],
            min_duration_s=lead_floor)

    # Residual tilt rate at arrival: SMALL, in the RECEIVE-tilt-increasing direction
    # (so the rim keeps moving through the seat, not parked). Zero for a level
    # receive tilt — there is no seat to continue through. Translational arrival
    # velocity is always zero.
    #
    # `seat` is the GRAVITY-REFERENCED receive tilt, not `target[3:5]`. With a
    # levelling correction loaded those differ by the correction, and aiming off the
    # plan-frame tilt is what made a gravity-level catch ramp 0.30° off level at
    # contact (bag 2026-07-25_15-17-48; `ros_ws/docs/catch_arrival_contract.md`).
    seat = (target[3:5] if receive_tilt is None
            else _as_tilt2(receive_tilt, 'receive_tilt'))
    smag = float(np.hypot(seat[0], seat[1]))
    rate = _catch_arrival_rate(pose[3:5], target[3:5], smag, D, decay,
                               tilt_through_rate_radps)
    arrival_twist = _zero.copy()
    settle_pose = target.copy()
    if smag > 1e-9 and rate > 0.0 and decay > 0.0:
        tdir = seat / smag
        arrival_twist[3] = rate * tdir[0]
        arrival_twist[4] = rate * tdir[1]
        # Overshoot the seat by the mean-velocity displacement of the rate→0 decay.
        overshoot = _CATCH_TILT_OVERSHOOT_FRAC * rate * decay
        settle_pose[3] = target[3] + overshoot * tdir[0]
        settle_pose[4] = target[4] + overshoot * tdir[1]

    # Gate the timing-critical reach FIRST (single-segment), so a too-tight lead is
    # rejected TOO_FAST before the (free-duration) settle/hold continuation is built.
    reach = QuinticSegment(p0=pose, v0=twist, a0=accel,
                           p1=target, v1=arrival_twist, a1=_zero, duration=D)
    reach_report = validate_follow(TrajectoryPlan((reach,), target), limits, geom)
    if not reach_report.ok:
        if reach_report.code not in _STRETCHABLE:
            raise TrajectoryInfeasible(reach_report.code, reach_report.reasons)
        t_min = _min_feasible_catch(pose, twist, accel, target, arrival_twist,
                                    limits, geom)
        raise TrajectoryInfeasible(
            TOO_FAST,
            [f"catch lead {D:.3f}s < minimal feasible {t_min:.3f}s to reach the "
             f"catch pose under the current limits"],
            min_duration_s=t_min)

    segments = [reach]
    # Tilt-through-seat decay (only when there is a residual rate to decay).
    if not _is_at_rest(arrival_twist, _zero):
        decay_dur = decay if decay > 0.0 else float(limits.min_move_duration_s)
        segments.append(QuinticSegment(
            p0=target, v0=arrival_twist, a0=_zero,
            p1=settle_pose, v1=_zero, a1=_zero, duration=decay_dur))

    # Literal quiescent hold segment (zero twist) — the window the hold-quiescence
    # criterion is measured over. A degenerate no-motion quintic (p0 == p1).
    if hold > 0.0:
        segments.append(QuinticSegment(
            p0=settle_pose, v0=_zero, a0=_zero,
            p1=settle_pose, v1=_zero, a1=_zero, duration=hold))

    final_pose = settle_pose
    if not hold_after:
        ret_plan, _ = build_follow(
            (settle_pose, _zero, _zero), _as6(neutral_pose, 'neutral_pose'),
            limits, geom, limits.min_move_duration_s)
        segments.extend(ret_plan.segments)
        final_pose = ret_plan.final_pose

    plan = TrajectoryPlan(tuple(segments), final_pose)
    # Single-gate contract: gate the ASSEMBLED plan (the object the emitter runs).
    report = validate_follow(plan, limits, geom)
    if not report.ok:
        if report.code in _STRETCHABLE:
            t_min = _min_feasible_catch(pose, twist, accel, target, arrival_twist,
                                        limits, geom)
            raise TrajectoryInfeasible(TOO_FAST, report.reasons, min_duration_s=t_min)
        raise TrajectoryInfeasible(report.code, report.reasons)
    return plan, report


def _min_feasible_catch(pose, twist, accel, target, arrival_twist, limits, geom):
    """Smallest catch lead the fast gate accepts for the reach to ``target``.

    Stretches the reach duration (the residual ``arrival_twist`` FIXED — a longer
    lead only lowers the leg peaks) from the ``min_timed_lead_s`` floor until
    :func:`validate_follow` passes. Bounded iterations; returns the best duration
    found (mirrors :func:`_min_feasible_timed`).

    Holding the arrival twist fixed while T moves is deliberately conservative
    under C-CATCH-1: the reach bound is ``2.5·scale/T`` (:func:`_catch_scale` —
    NOT the seed → target travel alone), and ``scale`` does not depend on T, so a
    LONGER candidate lead can only shrink the rate the real rebuild uses. The
    settle half of the bound does not involve T at all. This search therefore
    over-states the peaks slightly and reports a lead no shorter than the one that
    will actually work — never one that then fails on the caller's retry."""
    T = float(limits.min_timed_lead_s)
    _zero = np.zeros(POSE_DIM)
    report = None
    for _ in range(_MAX_FOLLOW_ITERS):
        reach = QuinticSegment(p0=pose, v0=twist, a0=accel,
                               p1=target, v1=arrival_twist, a1=_zero, duration=T)
        report = validate_follow(TrajectoryPlan((reach,), target), limits, geom)
        if report.ok:
            return T
        if report.code not in _STRETCHABLE:
            raise TrajectoryInfeasible(report.code, report.reasons)
        T *= _stretch_factor(report, limits)
    reach = QuinticSegment(p0=pose, v0=twist, a0=accel,
                           p1=target, v1=arrival_twist, a1=_zero, duration=T)
    final = validate_follow(TrajectoryPlan((reach,), target), limits, geom)
    if not final.ok and final.code in _STRETCHABLE:
        T *= _stretch_factor(final, limits)
    return T
