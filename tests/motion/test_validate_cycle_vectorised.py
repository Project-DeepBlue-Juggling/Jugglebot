"""T-R2-A — the VECTORISED ``feasibility.validate_cycle`` against its scalar twin.

WHY THIS FILE EXISTS
--------------------
``validate_cycle`` is the runtime safety assert on six legs and the hand: the skill
stack gates every ≤ 40-knot segment with it before the segment is dispatched on the
CAN wall clock.  R2 replaced its per-sample Python loop with a batched numpy chain
because the loop cost ~2.5 ms/knot (probe 2026-09-11) — ~157 ms on the 1.4 s
reference cycle, ~88 % of a ``plan_cycle`` call — and the plan pins < 10 ms per
40-knot segment (``plans/active/two-ball-skill-stack.md`` § 2.6).

A faster gate that quietly ACCEPTS a plan the old one refused is the one failure
this change must not ship.  So the old implementation is kept here verbatim as
:func:`_validate_cycle_scalar` (copied from ``67445f3``, the commit before the
vectorisation, lines 1058-1323 of ``feasibility.py``) and the two are run against
each other over a battery that covers every code the gate can emit, on real
planner output as well as hand-built pathological plans, and — the part that
actually binds — at limits set within 1e-6 relative of a MEASURED peak, where the
two implementations' last-bit differences would show up as a flipped verdict if
they existed.

PARITY BOUND (owner, 2026-09-12): ``ok`` and ``code`` identical, ``reasons``
identical strings, every ``peak_*`` field within 1e-9 relative.

WHERE THE ARITHMETIC DIFFERS IN EXPRESSION (and why 1e-9 is still the right bound)
---------------------------------------------------------------------------------
The batched chain is the SAME law, not an approximation, but three operations are
spelled differently and are therefore equal only to ~1 ulp, not bit-for-bit:

* leg vectors — scalar ``(rot @ plat_nodes.T).T``, batched
  ``einsum('nij,kj->nki', R, plat_nodes)``;
* ``J @ twist`` / ``J @ accel`` — scalar matmul, batched ``einsum('nij,nj->ni')``;
* the Rodrigues rotation — scalar ``rotvec_to_rot_matrix``, batched
  ``_batched_rotvec_to_R`` (same formula, same ``< 1e-12`` identity branch).

Each is the same expression the SHAPED gate has been shipping since Phase 1a
(``test_shaped_batch.py``), measured there at ≤ 2.84e-14 absolute.  1e-9 relative
is four orders above that and still far below any physical threshold.

THE ONE RESIDUAL, STATED HONESTLY: a plan whose leg extension or Jacobian
condition sits within ~1 ulp of a HARD workspace bound could in principle be
classified differently by the two chains.  That is inherent to any re-expression
of the geometry and is not closed by a tolerance; it is bounded by the fact that
both bounds carry a 5 mm / 1.25× physical margin, so a 1e-13 mm disagreement is
not a disagreement about the machine.  The limit ladder — the part a caller tunes
per session, and the part that CAN sit exactly on a peak — is what the
near-threshold sweep pins, at 1e-6 relative, in both directions.

I-PLAN-3's twin is asserted here too: nothing the 4-samples/knot gate accepts is
refused at 16 samples/knot, on the same battery.
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    accel_to_leg_accels,
    compute_jacobian,
    pose_to_leg_lengths,
    twist_to_leg_velocities,
)
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as fz
from jugglebot.motion.trajectory.cycle_plan import CyclePlan
from jugglebot.motion.trajectory.limits import TrajectoryLimits
from jugglebot.motion.workspace import check_leg_extensions, compute_condition_number
from jugglebot.outcome_detail import bound_msg

# Module globals the verbatim scalar reference below reads, bound here under the
# names it uses so the copy needs no edit beyond its own ``def`` line.
FeasibilityReport = fz.FeasibilityReport
OK = fz.OK
UNREACHABLE = fz.UNREACHABLE
WORKSPACE = fz.WORKSPACE
LIMIT_VEL = fz.LIMIT_VEL
LIMIT_ACC = fz.LIMIT_ACC
LIMIT_JERK = fz.LIMIT_JERK
STEP_BOUND = fz.STEP_BOUND
HAND_STROKE = fz.HAND_STROKE
HAND_LIMIT_VEL = fz.HAND_LIMIT_VEL
HAND_LIMIT_ACC = fz.HAND_LIMIT_ACC
STEP_BOUND_MARGIN = fz.STEP_BOUND_MARGIN
HAND_STROKE_MIN_REV = fz.HAND_STROKE_MIN_REV
CATCH_RUNWAY_MARGIN_REV = fz.CATCH_RUNWAY_MARGIN_REV
_CYCLE_SAMPLES_PER_KNOT = fz._CYCLE_SAMPLES_PER_KNOT
_VALIDATE_JERK_MARGIN = fz._VALIDATE_JERK_MARGIN
_workspace_limits = fz._workspace_limits
_pose_to_pos_rot = fz._pose_to_pos_rot
_pose_to_rev = fz._pose_to_rev
_cycle_sample_times = fz._cycle_sample_times
_hand_span_extrema = fz._hand_span_extrema
_hand_stroke_reason = fz._hand_stroke_reason
_cycle_stroke_floor = fz._cycle_stroke_floor


# ═══════════════════════════════════════════════════════════════════════════
# The reference: ``validate_cycle`` as it stood at 67445f3, VERBATIM
# ═══════════════════════════════════════════════════════════════════════════
#
# Copied unchanged from ``feasibility.py`` lines 1058-1323 at commit 67445f3,
# with only the function NAME altered.  Do not tidy it, do not re-flow it, do not
# "fix" anything in it — the moment it stops being the shipped-then code, it stops
# being a reference and this whole file measures nothing.

def _validate_cycle_scalar(cycle_plan, limits, geom, *,
                   samples_per_knot: int = _CYCLE_SAMPLES_PER_KNOT,
                   runway_margin_rev: float = CATCH_RUNWAY_MARGIN_REV
                   ) -> FeasibilityReport:
    """Feasibility gate for a 7-channel :class:`~cycle_plan.CyclePlan`.

    Same contract as :func:`validate` — a pure predicate over a fully-specified
    plan, returning a :class:`FeasibilityReport` whose ``code`` is the first
    failure in priority order and whose peaks are all populated — extended to the
    hand. It never mutates the plan and never stretches a duration.

    **It does not, and must not, delegate.** ``CyclePlan.segments`` is empty, so
    both gates above measure nothing on one; see the module block above this
    function for the two silent-pass paths that closes.

    The checks, in the order they can fail:

    1. **Geometry, per sample, first-failure-wins** (early return, exactly as
       :func:`validate`): non-finite pose or hand value → ``UNREACHABLE`` (a NaN
       sails through every numeric comparison below, so it is caught explicitly —
       ``CyclePlan``'s constructor rejects non-finite arrays too, and this is the
       defence-in-depth layer for a duck-typed plan or a post-construction
       mutation); leg extensions outside the hard stroke → ``WORKSPACE``; Jacobian
       condition past the workspace bound → ``UNREACHABLE``; slider outside
       ``[HAND_STROKE_MIN_REV, HAND_STROKE_MAX_REV]`` → ``HAND_STROKE``, naming the
       hard stop separately when that is crossed too. The FLOOR of that band is
       the plan's own first knot when the hand is parked below the homed zero
       (:func:`_cycle_stroke_floor`); the ceiling is never relaxed.
    2. **The catch runway, re-checked with the ACHIEVED catch velocity** →
       ``HAND_STROKE``. ``cup_cycle`` bounds this inside the QP with the TARGET
       catch speed (``catch_slider_vel_ratio × |v_ball,z|``) against a cup-frame
       floor that defaults to ``z_min_m``; the soft vertical match only approaches
       that target, and the real floor is the slider's own bottom of travel, which
       in the default configuration is ~0.21 m HIGHER than ``z_min_m``. So this is
       not a duplicate of the planner's bound — it is the first time the
       requirement is evaluated against both the achieved speed and the true
       floor. It shares ``HAND_STROKE`` because the fact it reports is a stroke
       fact: the travel below the catch is insufficient.
    3. **Limit ladder** (all peaks measured first, then prioritised): leg vel →
       leg acc → leg jerk → leg step, then hand vel → hand acc → hand step. The leg
       half is first and unchanged so a pose-track failure reports exactly the code
       it reports today.

    ``samples_per_knot`` meshes the POSE track only (see
    :data:`_CYCLE_SAMPLES_PER_KNOT`); the hand channel's extrema are closed-form
    per span and independent of it. ``runway_margin_rev`` is the reserve below the
    computed stopping distance.
    """
    m = max(1, int(samples_per_knot))
    wlimits = _workspace_limits(geom)
    mm_to_rev = np.asarray(geom.mm_to_rev, dtype=float)

    dt = float(getattr(cycle_plan, 'dt', limits.knot_dt_s))
    total = float(cycle_plan.total_duration)
    n_knots = int(getattr(cycle_plan, 'n_knots', int(round(total / dt)) + 1))
    if n_knots < 2 or dt <= 0.0 or total <= 0.0:
        return FeasibilityReport(
            ok=False, code=UNREACHABLE,
            reasons=["cycle plan spans no time (%d knots, dt=%.4f s, "
                     "duration=%.4f s)" % (n_knots, dt, total)])
    catch_k = int(getattr(cycle_plan, 'catch_k', -1))

    ts = _cycle_sample_times(n_knots, dt, total, m)
    hand_limit_v = float(limits.hand_vel_limit_rps)
    hand_limit_a = float(limits.hand_acc_limit_rps2)
    # The stroke floor for THIS plan (see `_cycle_stroke_floor`). The catch
    # runway below deliberately keeps HAND_STROKE_MIN_REV instead: the runway
    # asks how much travel is left to STOP a catch in, and the answer is bounded
    # by the physical bottom of travel, not by wherever the hand happened to be
    # parked before the window started.
    stroke_min = _cycle_stroke_floor(cycle_plan)

    peak_vel = 0.0
    peak_acc = 0.0
    peak_ext = 0.0
    peak_hand_rev = 0.0
    acc_samples = []

    # ── Pass 1: geometry + hand stroke + analytic leg vel/acc, per sample ──
    for t in ts:
        t = float(t)
        pose, twist, accel = cycle_plan.state_at(t)
        hand_rev, hand_vel = cycle_plan.hand_at(t)
        if not np.all(np.isfinite(pose)) or not np.all(np.isfinite(twist)) \
                or not np.all(np.isfinite(accel)):
            return FeasibilityReport(
                ok=False, code=UNREACHABLE,
                reasons=["pose state contains non-finite values (NaN/Inf) "
                         "at t=%.3fs" % t])
        if not (np.isfinite(hand_rev) and np.isfinite(hand_vel)):
            return FeasibilityReport(
                ok=False, code=UNREACHABLE,
                reasons=["hand channel contains non-finite values (NaN/Inf) "
                         "at t=%.3fs" % t])

        peak_hand_rev = max(peak_hand_rev, abs(float(hand_rev)))
        bad_stroke = _hand_stroke_reason(float(hand_rev), t, stroke_min)
        if bad_stroke is not None:
            return FeasibilityReport(
                ok=False, code=HAND_STROKE, reasons=[bad_stroke],
                peak_leg_ext_mm=peak_ext, peak_hand_rev=peak_hand_rev)

        pos, rot = _pose_to_pos_rot(pose)
        ext = pose_to_leg_lengths(pos, rot, geom)
        peak_ext = max(peak_ext, float(np.max(np.abs(ext))))
        valid, states = check_leg_extensions(ext, geom)
        if not valid:
            bad = [j for j, s in enumerate(states) if s != 0]
            return FeasibilityReport(
                ok=False, code=WORKSPACE,
                reasons=["leg %d out of stroke (%.1f mm) at t=%.3fs"
                         % (j, ext[j], t) for j in bad],
                peak_leg_ext_mm=peak_ext, peak_hand_rev=peak_hand_rev)

        J = compute_jacobian(pos, rot, geom)
        cond = compute_condition_number(pos, rot, geom, J=J)
        if cond > wlimits.cond_hard:
            return FeasibilityReport(
                ok=False, code=UNREACHABLE,
                reasons=["Jacobian condition %.1f > %.1f (near singularity) "
                         "at t=%.3fs" % (cond, wlimits.cond_hard, t)],
                peak_leg_ext_mm=peak_ext, peak_hand_rev=peak_hand_rev)

        leg_vel = twist_to_leg_velocities(twist, pos, rot, geom, J=J)
        peak_vel = max(peak_vel, float(np.max(np.abs(leg_vel))))
        leg_acc = accel_to_leg_accels(accel, twist, pos, rot, geom, J=J)
        peak_acc = max(peak_acc, float(np.max(np.abs(leg_acc))))
        acc_samples.append(leg_acc)

    # Leg jerk from the finite difference of the analytic leg acceleration on the
    # sub-knot mesh, inflated by the same _VALIDATE_JERK_MARGIN the segment gate
    # applies. NB the margin was calibrated on a rest-to-rest quintic at 80
    # samples/segment, not on this plan family; it is applied because a FD jerk
    # peak always under-measures, and the whole-cycle jerk characterisation is
    # WP4's sim harness, not this constant.
    peak_jerk = 0.0
    sub_dt = dt / m
    if len(acc_samples) >= 2:
        jerk = np.diff(np.asarray(acc_samples), axis=0) / sub_dt
        peak_jerk = float(np.max(np.abs(jerk))) * _VALIDATE_JERK_MARGIN

    # ── Pass 2: hand span extrema (closed form) + the per-span position extrema ──
    peak_hand_vel = 0.0
    peak_hand_acc = 0.0
    for k in range(n_knots - 1):
        t0 = ts[k * m]
        t1 = ts[(k + 1) * m]
        p0, v0 = cycle_plan.hand_at(float(t0))
        p1, v1 = cycle_plan.hand_at(float(t1))
        s_stationary, span_vel, span_acc = _hand_span_extrema(
            float(p0), float(v0), float(p1), float(v1), dt)
        peak_hand_vel = max(peak_hand_vel, span_vel)
        peak_hand_acc = max(peak_hand_acc, span_acc)
        # Interior POSITION extrema are read back through the plan's own hand_at,
        # so the gate can never disagree with the curve the emitter will sample.
        for s in s_stationary:
            t_s = float(t0) + s * dt
            rev_s, _ = cycle_plan.hand_at(t_s)
            if not np.isfinite(rev_s):
                return FeasibilityReport(
                    ok=False, code=UNREACHABLE,
                    reasons=["hand channel contains non-finite values (NaN/Inf) "
                             "at t=%.3fs" % t_s])
            peak_hand_rev = max(peak_hand_rev, abs(float(rev_s)))
            bad_stroke = _hand_stroke_reason(float(rev_s), t_s, stroke_min)
            if bad_stroke is not None:
                return FeasibilityReport(
                    ok=False, code=HAND_STROKE, reasons=[bad_stroke],
                    peak_leg_ext_mm=peak_ext, peak_hand_rev=peak_hand_rev)

    # ── Pass 3: the catch runway, with the ACHIEVED catch velocity ──
    if 0 <= catch_k <= n_knots - 1:
        t_catch = float(ts[min(catch_k * m, len(ts) - 1)])
        rev_c, vel_c = cycle_plan.hand_at(t_catch)
        available = float(rev_c) - HAND_STROKE_MIN_REV
        needed = (float(vel_c) ** 2) / (2.0 * hand_limit_a) + float(runway_margin_rev)
        if available < needed:
            return FeasibilityReport(
                ok=False, code=HAND_STROKE,
                reasons=[bound_msg(
                    'catch runway', available, '<', needed, unit='rev',
                    digits=3, limit_label='required',
                    tail=('achieved catch speed %.1f rev/s needs %.3f rev to stop '
                          'at %.0f rev/s^2 plus %.3f rev margin, and the slider '
                          'sits %.3f rev above its homed bottom at t=%.3fs — plan '
                          'the catch higher or slow it'
                          % (abs(float(vel_c)),
                             (float(vel_c) ** 2) / (2.0 * hand_limit_a),
                             hand_limit_a, float(runway_margin_rev),
                             available, t_catch)))],
                peak_leg_ext_mm=peak_ext, peak_hand_rev=peak_hand_rev)

    # ── Pass 4: per-knot step bounds, on the wire's own knot sequence ──
    peak_step = 0.0
    peak_hand_step = 0.0
    prev_rev = None
    prev_hand = None
    for k in range(n_knots):
        tk = float(ts[k * m])
        pose_k, _, _ = cycle_plan.state_at(tk)
        motor_rev = _pose_to_rev(pose_k, geom, mm_to_rev)
        hand_k, _ = cycle_plan.hand_at(tk)
        if prev_rev is not None:
            peak_step = max(peak_step,
                            float(np.max(np.abs(motor_rev - prev_rev))))
            peak_hand_step = max(peak_hand_step, abs(float(hand_k) - prev_hand))
        prev_rev = motor_rev
        prev_hand = float(hand_k)

    # ── Decide the code (priority order; first failure wins) ──
    step_bound = STEP_BOUND_MARGIN * float(limits.max_step_rev)
    # The hand's wire step gate landed with plan Phase 2: the pump's
    # per-channel ``max_step_hand_rev`` (``DEFAULT_MAX_STEP_HAND_REV`` =
    # hand_vel_limit_rps × knot dt — the same derivation as this bound, so
    # this sits at the same 20 % validate-below-pump margin the legs use
    # (0.8 × 5.0 = 4.0 vs pump 5.0) and the gate refuses a step-heavy cycle
    # BEFORE motion rather than mid-stream). Only the firmware half
    # (``MAX_DEVIATION_HAND_REV`` / ``MAX_LEAD_HAND_REV``) remains Phase 3.
    hand_step_bound = (STEP_BOUND_MARGIN * hand_limit_v * dt)
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
    elif peak_hand_vel > hand_limit_v:
        code = HAND_LIMIT_VEL
        reasons = [bound_msg('peak hand velocity', peak_hand_vel, '>',
                             hand_limit_v, unit='rev/s',
                             knob='trajectory_op.hand_vel_limit_rps', digits=2)]
    elif peak_hand_acc > hand_limit_a:
        code = HAND_LIMIT_ACC
        reasons = [bound_msg('peak hand acceleration', peak_hand_acc, '>',
                             hand_limit_a, unit='rev/s^2',
                             knob='trajectory_op.hand_acc_limit_rps2', digits=1)]
    elif peak_hand_step > hand_step_bound:
        code = STEP_BOUND
        reasons = [bound_msg(
            'peak per-knot hand step', peak_hand_step, '>', hand_step_bound,
            unit='rev', knob='trajectory_op.hand_vel_limit_rps', digits=3,
            tail=("%d%% of hand_vel_limit_rps %.1f x the plan's knot dt %.3f — "
                  'the same margin below the pump max_step_hand_rev gate that '
                  'the legs use; the firmware MAX_DEVIATION_HAND_REV half '
                  'lands in plan Phase 3'
                  % (int(STEP_BOUND_MARGIN * 100), hand_limit_v, dt)))]

    return FeasibilityReport(
        ok=(code == OK), code=code, reasons=reasons,
        peak_leg_vel_mmps=peak_vel, peak_leg_acc_mmps2=peak_acc,
        peak_leg_jerk_mmps3=peak_jerk, peak_leg_ext_mm=peak_ext,
        peak_step_rev=peak_step, peak_hand_rev=peak_hand_rev,
        peak_hand_vel_rps=peak_hand_vel, peak_hand_acc_rps2=peak_hand_acc,
        peak_hand_step_rev=peak_hand_step)


# ═══════════════════════════════════════════════════════════════════════════
# Rig — the reference operating point, shared with test_unified_cycle.py
# ═══════════════════════════════════════════════════════════════════════════

THROW_MM = np.array([0.0, 0.0, 860.0])
CATCH_MM = np.array([20.0, 0.0, 830.0])
CATCH_V_MM_S = np.array([100.0, -50.0, -2500.0])
REST_MM = np.array([0.0, 0.0, 750.0])
NEUTRAL = np.array([0.0, 0.0, float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM), 0.0, 0.0, 0.0])
DT = 0.025

#: Every ``peak_*`` on a :class:`FeasibilityReport` — all of them are compared,
#: so a new field cannot be added without this battery seeing it.
_PEAKS = tuple(f.name for f in dataclasses.fields(fz.FeasibilityReport)
               if f.name.startswith('peak_'))


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=250.0, leg_acc_mmps2=3000.0, leg_jerk_mmps3=150000.0)


def _goals(**kw) -> uc.CycleGoals:
    base = dict(period_s=1.4, throw_site_mm=THROW_MM, throw_target_mm=THROW_MM,
                flight_s=0.6, catch_site_mm=CATCH_MM,
                catch_vel_mm_s=CATCH_V_MM_S, catch_frac=0.55,
                settle_site_mm=REST_MM)
    base.update(kw)
    return uc.CycleGoals(**base)


def _rest_state(cup_mm=REST_MM, cfg=None) -> uc.CycleState:
    cfg = cr.RealizeConfig() if cfg is None else cfg
    slider_mm = float(cup_mm[2]) - cfg.cup_z_base_mm
    rev = ((slider_mm - cfg.slider_rev_zero_mm) / 1000.0 * cr.HAND_REV_PER_M)
    pose = np.array([cup_mm[0], cup_mm[1], cfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, cfg)


@pytest.fixture(scope='module')
def accepted(limits, geom):
    """``[(label, plan)]`` — every plan shape the skill stack gates, plus one driver.

    The first six are real planner output: the four window kinds and both splices.
    The seventh is synthetic and is there for one reason — on real cup plans the
    hand's WIRE STEP binds before its velocity ceiling (pulling
    ``hand_vel_limit_rps`` down to the measured velocity peak takes the derived
    ``0.8 × hand_vel_limit_rps × dt`` step bound below the measured step, so the
    near-threshold sweep lands on ``STEP_BOUND`` and ``HAND_LIMIT_VEL`` is never
    reached).  :func:`_hand_wiggle` inverts that ratio, so the sweep exercises the
    hand velocity ceiling at threshold like every other code.

    Module-scoped: six real solves plus two whole-plan re-gates, measured at
    ~3 s (2026-09-12).  The near-threshold sweep below re-gates each of these
    fourteen times, so building them once is the difference between a 30 s file
    and a 60 s one.
    """
    launch_plan, launch_meta = uc.plan_launch(_goals(period_s=0.6),
                                              _rest_state(), limits, geom)
    rel = uc.release_state_from_meta(launch_meta, launch_plan)
    steady_plan, steady_meta = uc.plan_steady(_goals(), rel, limits, geom)
    landing_plan, _ = uc.plan_landing(
        _goals(period_s=1.0, catch_frac=None, catch_t_s=0.6), rel, limits, geom)
    settle_plan, _ = uc.plan_settle(
        _goals(period_s=0.6, catch_site_mm=None, catch_vel_mm_s=None,
               catch_frac=None), rel, limits, geom)

    # A chained ring: LAUNCH + STEADY joined at the shared release knot.
    joined, _ = uc.extend(launch_plan, launch_meta, steady_plan, steady_meta,
                          limits, geom)
    # A catch-side re-aim spliced into the committed STEADY head.
    spliced, _ = uc.replan_tail(steady_plan, steady_meta, 0.0,
                                CATCH_MM + np.array([15.0, 8.0, 0.0]),
                                CATCH_V_MM_S, limits, geom, lead_s=0.10)
    return [('launch-0.6', launch_plan), ('steady-1.4', steady_plan),
            ('landing-1.0', landing_plan), ('settle-0.6', settle_plan),
            ('extend-chain', joined), ('replan-tail-splice', spliced),
            ('hand-wiggle', _hand_wiggle())]


def _held(hand_rev, hand_vel_rps, *, dt=DT, catch_k=-1):
    """A cycle holding NEUTRAL with a prescribed hand track (test_validate_cycle's
    minimal driver): every leg peak is exactly zero, so only a hand gate can fire."""
    hand_rev = np.asarray(hand_rev, dtype=float)
    n = hand_rev.shape[0]
    return CyclePlan(pose=np.tile(NEUTRAL, (n, 1)), pose_vel=np.zeros((n, 6)),
                     hand_rev=hand_rev,
                     hand_vel_rps=np.asarray(hand_vel_rps, dtype=float),
                     dt=dt, catch_k=catch_k)


def _hand_wiggle(n=5, amp=0.1, *, dt=DT):
    """A held platform with a zig-zag hand track: interior velocity >> knot step.

    Knot velocities are zero, so each span's cubic has to swing to cover ``amp``
    and back: its interior velocity peak is ``1.5 × amp / dt`` while the per-knot
    step is only ``amp``.  That is the ratio real cup plans do not have, and it is
    what lets the near-threshold sweep reach ``HAND_LIMIT_VEL``.
    """
    rev = 5.0 + amp * (np.arange(n) % 2)
    return _held(rev, np.zeros(n), dt=dt)


def _sweep(amp_mm, n=9, *, dt=DT, axis=0, hand=5.0):
    """An x (or y) sinusoidal sweep of the platform — the leg-channel driver."""
    t = np.arange(n) * dt
    w = 2 * np.pi / (n * dt)
    pose = np.tile(NEUTRAL, (n, 1))
    pose[:, axis] = amp_mm * np.sin(w * t)
    pose_vel = np.zeros((n, 6))
    pose_vel[:, axis] = amp_mm * w * np.cos(w * t)
    return CyclePlan(pose=pose, pose_vel=pose_vel,
                     hand_rev=np.full(n, float(hand)),
                     hand_vel_rps=np.zeros(n), dt=dt)


@pytest.fixture(scope='module')
def refused():
    """``[(label, plan)]`` — one hand-built plan per EARLY-RETURN path.

    The CyclePlan constructor rejects non-finite arrays, so the NaN case is
    injected after construction — which is exactly the post-construction mutation
    the gate's finite check exists as defence-in-depth against.
    """
    nan_plan = _sweep(30.0)
    nan_plan.pose[5, 0] = np.nan
    nan_hand = _sweep(30.0)
    nan_hand.hand_vel_rps[4] = np.inf
    out = [
        ('nan-pose', nan_plan),
        ('inf-hand-vel', nan_hand),
        # Below the stroke floor and past the end stop.
        ('hand-below-floor', _held([5.0, 5.0, -3.0, 5.0], np.zeros(4))),
        ('hand-past-end-stop', _held([5.0, 5.0, 12.0, 5.0], np.zeros(4))),
        # Far outside the leg workspace (a 400 mm z step off NEUTRAL).
        ('workspace', _sweep(0.0)),
        # A violent sweep: huge per-knot steps and leg velocity.
        ('step-bound', _sweep(120.0, n=7)),
    ]
    ws = out[4][1]
    ws.pose[:, 2] += 400.0
    return out


def _assert_parity(vec, ref, ctx):
    """The owner's 2026-09-12 bound: verdict identical, every peak < 1e-9 rel."""
    assert vec.code == ref.code, f"{ctx}: code {vec.code} != {ref.code}"
    assert vec.ok == ref.ok, f"{ctx}: ok {vec.ok} != {ref.ok}"
    assert list(vec.reasons) == list(ref.reasons), (
        f"{ctx}: reasons\n  vec {vec.reasons}\n  ref {ref.reasons}")
    for f in _PEAKS:
        a = float(getattr(vec, f))
        b = float(getattr(ref, f))
        rel = abs(a - b) / max(abs(b), 1e-9)
        assert rel < 1e-9, f"{ctx}: {f} {a!r} vs {b!r} (rel {rel:.3e})"


def _both(plan, lims, geom, **kw):
    return (fz.validate_cycle(plan, lims, geom, **kw),
            _validate_cycle_scalar(plan, lims, geom, **kw))


# ═══════════════════════════════════════════════════════════════════════════
# (a) Parity on real planner output and on every early-return path
# ═══════════════════════════════════════════════════════════════════════════

def test_parity_on_every_accepted_plan_shape(accepted, limits, geom):
    """The six shapes a sitting actually gates — four window kinds plus both splices.

    These all come back ``OK``, so what this pins is the PEAKS: every measured
    channel agrees to 1e-9 relative.  The peaks are what the near-threshold sweep
    below then sets its limits against, so a drift here would silently weaken
    that sweep rather than fail it.
    """
    for label, plan in accepted:
        vec, ref = _both(plan, limits, geom)
        assert ref.ok, f"{label}: reference refused a planner-accepted plan " \
                       f"({ref.code}: {ref.reasons})"
        _assert_parity(vec, ref, label)


def test_parity_on_every_early_return_path(refused, geom):
    """One plan per refusal the per-sample geometry pass can emit.

    The early returns are where a vectorised gate is most likely to diverge: the
    scalar loop stops at the FIRST offending sample and reports that sample's
    ``t`` and the peaks accumulated up to it, while the batched chain measures
    every sample and has to reconstruct the same prefix.  ``reasons`` is compared
    string-for-string, so the reported ``t=%.3fs`` and the per-leg extension in
    the WORKSPACE text have to be the same sample, not merely the same code.
    """
    lims = TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=250.0, leg_acc_mmps2=3000.0, leg_jerk_mmps3=150000.0)
    seen = set()
    for label, plan in refused:
        vec, ref = _both(plan, lims, geom)
        assert not ref.ok, f"{label}: reference accepted it — driver is wrong"
        _assert_parity(vec, ref, label)
        seen.add(ref.code)
    assert seen >= {fz.UNREACHABLE, fz.WORKSPACE, fz.HAND_STROKE}, seen


def test_parity_when_the_peaks_are_reported_on_an_early_return(geom):
    """``peak_leg_ext_mm`` / ``peak_hand_rev`` on a HAND_STROKE are a PREFIX max.

    The scalar loop updates ``peak_hand_rev`` BEFORE its stroke check and
    ``peak_ext`` AFTER it, so a HAND_STROKE at sample *i* reports the hand peak
    over ``0..i`` and the leg peak over ``0..i-1``.  That off-by-one is invisible
    to a code comparison and is exactly the kind of thing a rewrite loses, so it
    gets its own driver: a sweep that grows monotonically in leg extension with a
    stroke breach partway through, where the two prefixes are different numbers.
    """
    n = 9
    pose = np.tile(NEUTRAL, (n, 1))
    pose[:, 0] = np.linspace(0.0, 80.0, n)        # leg ext grows every knot
    hand = np.full(n, 5.0)
    hand[5] = 12.5                                 # past the end stop, mid-plan
    plan = CyclePlan(pose=pose, pose_vel=np.zeros((n, 6)), hand_rev=hand,
                     hand_vel_rps=np.zeros(n), dt=DT)
    lims = TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=250.0, leg_acc_mmps2=3000.0, leg_jerk_mmps3=150000.0)
    vec, ref = _both(plan, lims, geom)
    assert ref.code == fz.HAND_STROKE
    assert ref.peak_leg_ext_mm > 0.0
    _assert_parity(vec, ref, 'prefix-peaks-on-hand-stroke')


def test_parity_at_a_coarser_and_a_denser_mesh(accepted, limits, geom):
    """Parity is not mesh-special: it holds at 1, 4 and 16 samples per knot.

    ``samples_per_knot`` is an accuracy knob on the leg-jerk finite difference,
    and the batched chain has to reproduce the scalar one at whatever mesh it is
    handed — including ``m = 1``, where the sub-knot grid collapses onto the knots
    themselves and the knot-step pass reads the same samples as pass 1.
    """
    for label, plan in accepted[:4]:
        for m in (1, 4, 16):
            vec, ref = _both(plan, limits, geom, samples_per_knot=m)
            _assert_parity(vec, ref, f"{label} m={m}")


# ═══════════════════════════════════════════════════════════════════════════
# (b) Near-threshold: the limits set within 1e-6 of a measured peak
# ═══════════════════════════════════════════════════════════════════════════
#
# This is the part that actually binds.  Anywhere above, a 1e-13 disagreement
# between the two chains is absorbed by a limit that sits far from the peak; here
# the limit is placed one part in a million from the measured peak, in both
# directions, so the two implementations have to agree about which side of it the
# plan falls on.  Seven knobs × two directions × six plans.

def _knobs(plan, report):
    """``[(limits field, limit value at the peak, expected code)]`` for ``plan``.

    The value returned is the limit at which the plan sits EXACTLY on the bound:
    the peak itself for the four direct ceilings, the peak divided by
    :data:`STEP_BOUND_MARGIN` for the leg step (the gate refuses above
    ``0.8 × max_step_rev``), and the hand step's peak divided by
    ``0.8 × dt`` (the hand's wire step bound is derived from the hand VELOCITY
    limit, so that is the knob it is reached through).
    """
    dt = float(plan.dt)
    return [
        ('leg_vel_mmps', report.peak_leg_vel_mmps, fz.LIMIT_VEL),
        ('leg_acc_mmps2', report.peak_leg_acc_mmps2, fz.LIMIT_ACC),
        ('leg_jerk_mmps3', report.peak_leg_jerk_mmps3, fz.LIMIT_JERK),
        ('max_step_rev', report.peak_step_rev / STEP_BOUND_MARGIN,
         fz.STEP_BOUND),
        ('hand_vel_limit_rps', report.peak_hand_vel_rps, fz.HAND_LIMIT_VEL),
        ('hand_acc_limit_rps2', report.peak_hand_acc_rps2, fz.HAND_LIMIT_ACC),
        ('hand_vel_limit_rps',
         report.peak_hand_step_rev / (STEP_BOUND_MARGIN * dt), fz.STEP_BOUND),
    ]


def test_near_threshold_verdicts_flip_identically(accepted, limits, geom):
    """Every limit code, reached from both sides at 1e-6 relative, on every plan.

    For each accepted plan and each of the seven limit codes the ladder can emit,
    the relevant limit is set to ``peak × (1 ± 1e-6)`` and BOTH gates are run.
    They must return the same verdict, the same reason string and the same peaks —
    and when the loose side still accepts, the tight side must refuse with the
    code that knob owns, which is what makes this a test of the LADDER and not
    just of the arithmetic.

    The hand's wire step is reached through ``hand_vel_limit_rps`` (the bound is
    ``0.8 × hand_vel_limit_rps × dt``), so on a plan whose hand velocity peak
    binds first that case lands on ``HAND_LIMIT_VEL`` instead; the assertion is
    therefore conditioned on the loose side still being OK.
    """
    fired = set()
    cases = 0
    for label, plan in accepted:
        base = fz.validate_cycle(plan, limits, geom)
        for knob, at_peak, expect in _knobs(plan, base):
            if not (at_peak > 0.0):
                continue
            got = {}
            for side, eps in (('tight', -1e-6), ('loose', +1e-6)):
                lims = dataclasses.replace(
                    limits, **{knob: float(at_peak) * (1.0 + eps)})
                vec, ref = _both(plan, lims, geom)
                _assert_parity(vec, ref, f"{label} {knob} {side}")
                got[side] = vec
                cases += 1
            if got['loose'].ok:
                assert got['tight'].code == expect, (
                    "%s: %s at peak×(1-1e-6) gave %s, expected %s"
                    % (label, knob, got['tight'].code, expect))
                fired.add(got['tight'].code)
    assert fired == {fz.LIMIT_VEL, fz.LIMIT_ACC, fz.LIMIT_JERK, fz.STEP_BOUND,
                     fz.HAND_LIMIT_VEL, fz.HAND_LIMIT_ACC}, fired
    # 74 knob-cases = 148 gate calls (measured 2026-09-12); the floor guards
    # against a fixture change silently emptying the sweep.
    assert cases >= 70, cases


# ═══════════════════════════════════════════════════════════════════════════
# (c) I-PLAN-3's twin on the new path
# ═══════════════════════════════════════════════════════════════════════════

def test_the_shipped_mesh_under_measures_nothing_but_the_finite_difference_jerk(
        accepted, limits, geom):
    """I-PLAN-3 (``skills/INVARIANTS.md``) restated for the vectorised gate.

    The gate's default mesh is four sub-samples per knot, and a coarse mesh can
    only UNDER-measure a peak — so the failure this closes is a plan the shipped
    mesh calls feasible and a denser one calls over-limit, i.e. a real violation
    the operator never saw.  The ``m = 16`` sample set is a strict SUPERSET of the
    ``m = 4`` one (both are ``k·dt/m`` grids sharing the same nudged last sample),
    so every peak that is a maximum over samples must be ``>=`` its coarse twin
    exactly, and the span-closed-form hand peaks and the knot-step peaks must be
    EQUAL — they do not read the sub-knot mesh at all.

    THE ONE CHANNEL THAT GENUINELY GROWS WITH THE MESH, stated rather than hidden:
    leg jerk is a finite difference of the analytic leg acceleration, so its
    denominator is ``dt/m``.  Refining the mesh resolves more of the true jerk and
    the measured peak rises — on ``steady-1.4`` it reads under 150 000 mm/s³ at
    ``m = 4`` and **229 537 mm/s³ at m = 16** (2026-09-12, this battery).  That is
    a property of the FD, is why :data:`feasibility._VALIDATE_JERK_MARGIN` exists,
    and is unchanged by the vectorisation — ``test_parity_at_a_coarser_and_a_denser_mesh``
    pins that the scalar reference reports the SAME growth sample for sample.  So
    the jerk code is the one admitted exception here, and a coarse-accepts /
    dense-refuses on any OTHER code is a hard failure.
    """
    for label, plan in accepted:
        coarse = fz.validate_cycle(plan, limits, geom, samples_per_knot=4)
        dense = fz.validate_cycle(plan, limits, geom, samples_per_knot=16)
        for f in ('peak_leg_vel_mmps', 'peak_leg_acc_mmps2', 'peak_leg_ext_mm',
                  'peak_hand_rev'):
            assert getattr(dense, f) >= getattr(coarse, f), f"{label}: {f}"
        for f in ('peak_step_rev', 'peak_hand_vel_rps', 'peak_hand_acc_rps2',
                  'peak_hand_step_rev'):
            assert getattr(dense, f) == getattr(coarse, f), f"{label}: {f}"
        if coarse.ok:
            assert dense.ok or dense.code == fz.LIMIT_JERK, (
                "%s: accepted at 4 samples/knot, refused at 16 (%s: %s) — the "
                "shipped mesh is under-measuring this plan family on a channel "
                "that is NOT the finite-difference jerk"
                % (label, dense.code, dense.reasons))


def test_parity_on_the_condition_refusal(geom):
    """The near-singularity early return, driven by lowering ``cond_hard``.

    Driven rather than found on purpose.  Reaching the SHIPPED bound
    (``2.0 x cond_reference``) needs a pose so extreme that the leg stroke check —
    which runs first — answers instead, so a "realistic" driver for this branch
    tests ``WORKSPACE``.  Lowering the bound under a plan whose condition is
    genuinely measured drives the exact branch with the exact numbers the refusal
    text formats, and both implementations read the SAME
    :func:`feasibility._workspace_limits`, so neither is being handed a different
    fact from the other.
    """
    plan = _sweep(40.0)
    key = id(geom)
    saved = fz._WLIMITS_CACHE.get(key)
    wl = fz._workspace_limits(geom)
    # The condition at the sweep's extreme, less a hair: the plan is above the
    # bound somewhere and below it at t = 0, so the refusal has a real FIRST
    # offending sample rather than failing at knot 0.
    _, leg_vecs, R = fz._batched_jacobian(plan.pose, geom)
    conds = fz._batched_condition(leg_vecs, R, geom)
    assert conds.max() > conds.min(), conds
    try:
        fz._WLIMITS_CACHE[key] = (geom, dataclasses.replace(
            wl, cond_hard=float(0.5 * (conds.min() + conds.max()))))
        lims = TrajectoryLimits.from_config(hw).with_session_limits(
            leg_vel_mmps=1e6, leg_acc_mmps2=1e6, leg_jerk_mmps3=1e9)
        vec, ref = _both(plan, lims, geom)
        assert ref.code == fz.UNREACHABLE, (ref.code, ref.reasons)
        assert 'condition' in ref.reasons[0]
        _assert_parity(vec, ref, 'condition-bound')
    finally:
        if saved is None:
            fz._WLIMITS_CACHE.pop(key, None)
        else:
            fz._WLIMITS_CACHE[key] = saved


def test_parity_on_the_catch_runway_refusal(geom):
    """Pass 3, which now reads the sampled hand arrays instead of calling ``hand_at``.

    The recipe is ``test_validate_cycle.py``'s: ``catch_k = 1``, hand
    ``[3.5, 1.0]`` rev at a constant -100 rev/s, which needs 1.429 rev to stop
    plus the 0.614 rev margin against 1.000 rev of travel below the catch.  The
    runway is the one pass whose input moved from a scalar accessor to an array
    index, so its reason string — which quotes the achieved speed, the stopping
    distance and ``t`` — is the thing to compare.
    """
    plan = _held([3.5, 3.5 - 2.5], [-100.0, -100.0], catch_k=1)
    lims = TrajectoryLimits.from_config(hw)
    vec, ref = _both(plan, lims, geom)
    assert ref.code == fz.HAND_STROKE
    assert 'catch runway' in ref.reasons[0]
    _assert_parity(vec, ref, 'catch-runway')
