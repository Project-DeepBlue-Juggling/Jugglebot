"""The late catch re-aim as a platform-xy BUMP on the installed plan (I-CATCH-5).

Plan: ``plans/archived/late-catch-reaim-bump.md``; design and offline
characterisation: ``logbook/2026-10-16-late-catch-reaim-bump-design.md``.
Past the snap band the raw re-solve (catch-with-throw from a mid-swing seed)
was refused 87/87 on 2026-10-16.  The bump re-aims the SAME installed record
instead: an offset ``b(t)`` added to ``pose[:, 0:2]`` and its exact derivative
``ḃ(t)`` to ``pose_vel[:, 0:2]``.  The hand arrays are never touched, the
record keeps its ``t0``, nothing is re-solved, and the bumped plan must pass
``feasibility.validate_cycle`` at the live limits or the bump is DECLINED —
never clipped, never scaled down (a partial bump is a silent clip).

:func:`apply_catch_bump` is the ONE enforcement point of I-CATCH-5.

The 40 Hz walk (written before the code, CLAUDE.md)
---------------------------------------------------
Knots are Hermite ``(pos, vel)`` on the 25 ms grid; trajectory_node streams
them and the can-bridge Teensy interpolates the Hermite at 500 Hz.  At an
install instant ``t_inst`` the wire may already hold every knot up to
``k_wire = floor((t_inst - t0)/dt) + WIRE_READ_KNOTS`` (3 knots ahead).

* ``k_b = k_wire + 2`` (:data:`BUMP_LEAD_KNOTS` = 5 knots after the dispatch
  knot).  The ramp starts AT ``t_b = k_b·dt`` with a quintic whose position,
  velocity and acceleration are all zero there, so knots ``0..k_b`` are
  BIT-IDENTICAL to the installed record (the function asserts it before
  returning an accepted plan), and so is every Hermite segment up to and
  including ``[k_b-1, k_b]``.  The first changed knot is ``k_b + 1``: two knots
  past anything the wire can have read when the executor dispatched, which is
  the 25-50 ms install allowance (knot quantisation).  The node re-derives
  ``k_wire`` at apply under its stream lock and refuses ``SPLICE_TOO_LATE`` if
  ``k_b <= k_wire_node`` (one knot more conservative than strictly needed, the
  plan's rule); that refusal is non-terminal (a latency fact, record unchanged).
* Tick by tick around the install: ticks ``n <= k_b`` emit the old setpoints
  (position AND velocity feed-forward unchanged, so no step on either); tick
  ``k_b+1`` emits ``pose + b(t_{k_b+1})`` with ``pose_vel + ḃ`` — the velocity
  feed-forward is the analytic derivative of the same offset, so position and
  FF stay consistent and the interpolant is C1 across every knot by
  construction.  Its acceleration and jerk between knots are what
  ``validate_cycle`` measures on the bumped slice; a peak over the sitting
  limit (LIMIT_VEL first on the ramp-in, then ACC/JERK — every offline refusal
  was there) declines the bump and the old record keeps streaming untouched.
* Every breakpoint of ``b`` is at least C2 (ramp-in end at ``t_c - pre_hold``:
  quintic ends with zero velocity/acceleration; carry velocity ramp starts and
  ends with zero acceleration; post-release rest quintic starts from the
  constant-velocity state with zero acceleration and ends at zero offset,
  velocity and acceleration on the record's last knot), so the knot samples
  carry no hidden discontinuity at 40 Hz.

The shape (per lateral axis; W1-corrected 2026-10-16)
----------------------------------------------------
1. Ramp-in: quintic ``0 -> Δ`` over ``[t_b, t_c - BUMP_PRE_HOLD_S]``.  0.050 s
   covers the largest measured timing slip (|Δt| <= 41 ms), so the bump is
   xy-only; a timing move is absorbed by the pre-hold, not re-planned.
2. Carry (the DELIBERATE, BOUNDED exception to the 2026-09-28 carry rule): over
   ``T_a = [t_c, t_rel - pre_s]`` a quintic lateral VELOCITY ramp to
   ``v_b = -Δ/(T_f + T_a/2 + pre_s)``, held constant through the pre/post-
   release hold (no added acceleration across the detach cone).  The release
   offset is then ``b(t_rel) = Δ + v_b·(T_a/2 + pre_s)`` and the landing offset
   ``b(t_rel) + v_b·T_f`` is exactly 0: the throw lands on its unbumped target.
3. Settle: after the post-release hold a rest-to-rest quintic returns offset
   AND velocity to 0 on the record's last knot, so the rest-terminal is the
   plan's own ``rest_site_mm`` (I-PLAN-7) and a later install seeds from it.

The launch bound (owner decision 2026-10-16)
--------------------------------------------
The TOTAL lateral launch velocity ``|v_plan + v_b|`` (the unbumped plan's own
release lateral velocity ``rm.vel_mm_s[:2]`` plus the Euclidean bias ``v_b`` the
retarget adds) is bounded by :data:`BUMP_MAX_LAUNCH_LATERAL_MM_S` = 50 mm/s;
above it the bump is declined :data:`BUMP_LAUNCH` (non-terminal: a smaller Δ
can pass).  ``BumpResult.launch_bias_xy_mm_s`` still reports the bias alone.
The executor's "<= 25 mm/s" figures (``SkillExecutor._catch_terminal`` /
``_release_pos_mm``) describe the UNBUMPED carried throw — the release rides
the caught xy and the ballistics absorb the <= 20 mm offset as <= 25 mm/s —
so the total is the plan's own <= ~25 mm/s plus ``v_b``; ``v_b`` is a SEPARATE,
planned bias that exists only on a bumped catch (design typical <= 38 mm/s at
columns timing, T_f 0.88, T_a 0.17; 46 mm/s for any ``T_a`` at the clamped
``|Δ| <= 40.3`` mm).  The 2026-09-28 rule exists because an UNPLANNED 85-110
mm/s launch made throws miss; this one is planned and accounted for exactly.

Per-pattern switch (owner decision 2026-10-16)
----------------------------------------------
:data:`BUMP_ENABLED_PATTERNS` = the columns family only.  Hop and reload (sep
250) stay OFF until a matching-record capture exists (the offline columns gate
plans no catch at sep 250, so hop was only a columns proxy) — that capture,
replayed through :func:`apply_catch_bump`, is the unpark condition.

Pure functions, no I/O, no ROS; Python 3.8.
"""
from __future__ import annotations

import dataclasses
import math
from typing import Optional, Tuple

import numpy as np

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.skills.schedule import WIRE_READ_KNOTS
from jugglebot.motion.trajectory import feasibility as fz
from jugglebot.motion.trajectory.cycle_plan import CyclePlan, shift_catch_evidence

#: Knots past the wire's read horizon at dispatch before the first knot the bump
#: may change is ``k_b + 1``: ``k_b = k_wire + 2`` (the 25-50 ms install
#: allowance, plan § 2).  Measured: at 9 knots the same bump installs 5 %.
BUMP_EXTRA_KNOTS = 2
BUMP_LEAD_KNOTS = WIRE_READ_KNOTS + BUMP_EXTRA_KNOTS
#: The ramp-in ends this long before the catch (|Δt| <= 41 ms measured).
BUMP_PRE_HOLD_S = 0.050
#: Owner bound on the retarget's lateral launch-velocity bias (2026-10-16).
BUMP_MAX_LAUNCH_LATERAL_MM_S = 50.0
#: Shortest ramp / carry / settle the shape is built over (two knots).
BUMP_MIN_PHASE_KNOTS = 2
#: Gate context knots before ``k_b`` (validate_cycle's stencils are knot-local;
#: the knots before ``k_b - 4`` are unchanged and already passed the gate).
BUMP_GATE_CONTEXT_KNOTS = 4
#: Patterns the bump is enabled for.  Hop / reload (sep 250) stay off until a
#: matching-record capture passes the replay (owner decision 2026-10-16).
BUMP_ENABLED_PATTERNS = frozenset({'columns', 'columns_1ball_fed'})

#: Decline codes the bump adds (the gate's own codes pass through unchanged).
BUMP_FREEZE = 'BUMP-FREEZE'        # no runway left for the ramp: terminal
BUMP_NO_CARRY = 'BUMP-NO-CARRY'    # no carried release / no carry or settle room: terminal
BUMP_LAUNCH = 'BUMP-LAUNCH'        # |v_b| above the owner bound: non-terminal
BUMP_SPLICE_TOO_LATE = 'SPLICE_TOO_LATE'  # k_b <= k_wire at apply: non-terminal
#: Node-side (trajectory_node, G2): an I-INSTALL-1 StreamHandoff is pending, so
#: the wire is not yet playing the record the bump would edit: non-terminal.
BUMP_HANDOFF = 'BUMP_HANDOFF'
#: Node-side: the active record no longer holds this catch (a later install
#: superseded it, or nothing is streaming): terminal.
BUMP_NO_RECORD = 'BUMP-NO-RECORD'
#: Non-terminal declines that are NOT gate refusals: retried on the next tick
#: as-is (the executor's BUMP-WINDOW shrink rule applies to gate codes and
#: BUMP-LAUNCH only, whose answer cannot change for the same delta).
BUMP_RETRY_CODES = frozenset({BUMP_SPLICE_TOO_LATE, BUMP_HANDOFF})

#: Executor-side terminal freeze: past ``t_c - BUMP_FREEZE_LEAD_S`` a ramp
#: starting ``BUMP_LEAD_KNOTS`` knots out cannot finish ``BUMP_PRE_HOLD_S``
#: before the catch with ``BUMP_MIN_PHASE_KNOTS`` of ramp (0.225 s today).
BUMP_FREEZE_LEAD_S = ((BUMP_LEAD_KNOTS + BUMP_MIN_PHASE_KNOTS)
                      * float(hw.JB_TRAJ_KNOT_DT_S) + BUMP_PRE_HOLD_S)


def bump_enabled_for(pattern_name: Optional[str]) -> bool:
    """The per-pattern switch (columns family only, 2026-10-16)."""
    return pattern_name in BUMP_ENABLED_PATTERNS


@dataclasses.dataclass(frozen=True)
class BumpResult:
    """One bump attempt: accepted (a full-length bumped plan + meta) or a decline.

    ``terminal`` declines end bumping for the catch (no runway, no carry); the
    others (gate refusals = the executor's ``BUMP-WINDOW``, ``BUMP-LAUNCH``,
    ``SPLICE_TOO_LATE``) leave the committed catch and the cap untouched.
    """

    accepted: bool
    code: str
    terminal: bool
    message: str
    k_b: int
    plan: Optional[CyclePlan] = None
    meta: Optional[uc.CycleMeta] = None
    report: Optional[fz.FeasibilityReport] = None
    delta_xy_mm: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(2))
    #: The REALISED release offset ``b(t_rel)`` (mm, xy) — what the learner row
    #: and ``_carried_release_mm`` must record (NOT Δ).
    release_offset_xy_mm: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(2))
    #: The lateral launch-velocity bias ``v_b`` (mm/s, xy).
    launch_bias_xy_mm_s: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(2))


def _q5(u):
    """Rest-to-rest quintic ``s(u)`` and ``ds/du`` on ``u`` clipped to [0, 1]."""
    u = np.clip(u, 0.0, 1.0)
    return (10 * u**3 - 15 * u**4 + 6 * u**5), (30 * u**2 - 60 * u**3 + 30 * u**4)


def _q5_dd(u):
    """``d²s/du²`` of :func:`_q5` on ``u`` clipped to [0, 1] (zero at both ends,
    so the clip leaves no step: the analytic acceleration of the same profile)."""
    u = np.clip(u, 0.0, 1.0)
    return 60 * u - 180 * u**2 + 120 * u**3


def _vel_ramp(t, t_a, T, v):
    """Position/velocity of a quintic VELOCITY ramp ``0 -> v`` over
    ``[t_a, t_a + T]`` (C2 in velocity), continued at constant ``v`` after."""
    t = np.asarray(t, float)
    u = np.clip((t - t_a) / T, 0.0, 1.0)
    pos = T * (2.5 * u**4 - 3 * u**5 + u**6) + np.maximum(t - (t_a + T), 0.0)
    vel = 10 * u**3 - 15 * u**4 + 6 * u**5
    return pos[:, None] * v[None, :], vel[:, None] * v[None, :]


def _vel_ramp_acc(t, t_a, T, v):
    """Acceleration of :func:`_vel_ramp` (analytic; zero outside the ramp)."""
    u = np.clip((np.asarray(t, float) - t_a) / T, 0.0, 1.0)
    acc = (30 * u**2 - 60 * u**3 + 30 * u**4) / T
    return acc[:, None] * v[None, :]


def _rest_quintic(t, t_d, t_e, p0, v0):
    """Per-axis quintic from ``(p0, v0, a=0)`` at ``t_d`` to ``(0, 0, 0)`` at
    ``t_e``, zero after ``t_e``: the post-release re-centre (the ball has left)."""
    T = t_e - t_d
    A = np.array([[T**3, T**4, T**5], [3 * T**2, 4 * T**3, 5 * T**4],
                  [6 * T, 12 * T**2, 20 * T**3]])
    tau = np.clip(np.asarray(t, float) - t_d, 0.0, T)
    pos = np.zeros((len(tau), len(p0)))
    vel = np.zeros_like(pos)
    acc = np.zeros_like(pos)
    for ax in range(len(p0)):
        c3, c4, c5 = np.linalg.solve(A, [-p0[ax] - v0[ax] * T, -v0[ax], 0.0])
        pos[:, ax] = p0[ax] + v0[ax] * tau + c3 * tau**3 + c4 * tau**4 + c5 * tau**5
        vel[:, ax] = v0[ax] + 3 * c3 * tau**2 + 4 * c4 * tau**3 + 5 * c5 * tau**4
        # The third boundary row pins acc(T) = 0, so the clip at T leaves no step.
        acc[:, ax] = 6 * c3 * tau + 12 * c4 * tau**2 + 20 * c5 * tau**3
    return pos, vel, acc


def retarget_parts(delta, t_c, t_rel, pre_s, flight_s):
    """``(v_b, b_rel)``: the launch-velocity bias and the realised release offset.

    Exact retarget (landing offset 0): ``b_rel + v_b·T_f = 0`` with
    ``b_rel = Δ + v_b·(T_a/2 + pre_s)``, so ``v_b = -Δ/(T_f + T_a/2 + pre_s)``."""
    delta = np.asarray(delta, float)
    T_a = (t_rel - pre_s) - t_c
    v_b = -delta / (flight_s + T_a / 2.0 + pre_s)
    return v_b, delta + v_b * (T_a / 2.0 + pre_s)


def retarget_profile(t, t_b, t_in_end, t_c, t_rel, pre_s, post_s, t_end,
                     flight_s, delta, *, with_acc: bool = False):
    """``(b, ḃ)`` at plan-clock times ``t``, shape ``(len(t), 2)`` (module doc).

    ``with_acc=True`` returns ``(b, ḃ, b̈)``: the ANALYTIC second derivative of
    the same three pieces (I-PLAN-10 — the overlay's cup-track update must not
    finite-difference the profile it already knows in closed form).
    """
    delta = np.asarray(delta, float)
    t = np.asarray(t, float)
    T1 = t_in_end - t_b
    u1 = (t - t_b) / T1
    s1, ds1 = _q5(u1)
    b = s1[:, None] * delta[None, :]
    bd = (ds1 / T1)[:, None] * delta[None, :]
    bdd = (_q5_dd(u1) / T1**2)[:, None] * delta[None, :]
    v_b, _ = retarget_parts(delta, t_c, t_rel, pre_s, flight_s)
    T_a = (t_rel - pre_s) - t_c
    p1, v1 = _vel_ramp(t, t_c, T_a, v_b)
    b, bd = b + p1, bd + v1
    bdd = bdd + _vel_ramp_acc(t, t_c, T_a, v_b)
    t_d = t_rel + post_s
    after = t >= t_d - 1e-12
    p_d = delta + v_b * (T_a / 2.0 + pre_s + post_s)
    q, qd, qdd = _rest_quintic(t, t_d, t_end, p_d, v_b)
    b[after] = q[after]
    bd[after] = qd[after]
    bdd[after] = qdd[after]
    if with_acc:
        return b, bd, bdd
    return b, bd


def _plan_slice(pose, pose_vel, plan: CyclePlan, k0: int, k1: int) -> CyclePlan:
    """Knots ``[k0, k1]`` of a plan with the given pose arrays, carrying the
    contact windows and catch knot across (shifted and clipped, never dropped:
    a dropped ``contact_knots`` disarms CUP_CONTACT_ACC)."""
    ck = None
    if plan.contact_knots:
        ck = tuple((max(a, k0) - k0, min(b_, k1) - k0)
                   for a, b_ in plan.contact_knots if b_ >= k0 and a <= k1) or None
    catch_k = plan.catch_k - k0 if k0 <= plan.catch_k <= k1 else -1
    sl = slice(k0, k1 + 1)
    return CyclePlan(pose[sl], pose_vel[sl], plan.hand_rev[sl],
                     plan.hand_vel_rps[sl], plan.dt, catch_k=catch_k,
                     contact_knots=ck,
                     catch_evidence=shift_catch_evidence(
                         plan.catch_evidence, -k0, plan.dt, k1 - k0 + 1))


def _bumped_cup(cup, b, bd, bdd):
    """``cup`` (SI metres) with the bump's ANALYTIC ``(b, ḃ, b̈)`` (mm) added to
    its xy pos/vel/acc on every knot, so ``cup_realize.decompose(cup, tilts)``
    reproduces the bumped ``plan.pose[:, :2]`` (I-PLAN-10).

    ``decompose`` builds ``pose_xy = cup_xy·1000 − shift`` and ``pose_vel_xy =
    vel_xy·1000 − shift_dot``, with ``shift`` / ``shift_dot`` functions of
    ``cup_z`` and the tilts only: a pure xy overlay with tilts and z untouched
    therefore needs no shift term, and adding ``b/1000`` to the cup xy is the
    exact preimage.  ``jerk`` is the cup track's per-SPAN quantity
    (``acc[k+1] = acc[k] + jerk[k]·dt``, ``CupCyclePlan.state_at``), so it takes
    the span difference of the analytic ``b̈`` — the exact span-mean of the
    profile's third derivative, keeping the cup track's own rollout
    self-consistent.  ``takeoff_vel`` is left alone here: the caller mirrors
    ``meta.takeoff_vel_mps``'s rule.
    """
    if cup is None:
        return None
    n = cup.pos.shape[0]
    if b.shape[0] != n:
        raise ValueError('bump profile has %d knots, cup track %d'
                         % (b.shape[0], n))
    pos = np.array(cup.pos, dtype=float, copy=True)
    vel = np.array(cup.vel, dtype=float, copy=True)
    acc = np.array(cup.acc, dtype=float, copy=True)
    jerk = np.array(cup.jerk, dtype=float, copy=True)
    pos[:, :2] += b / 1000.0
    vel[:, :2] += bd / 1000.0
    acc[:, :2] += bdd / 1000.0
    jerk[:, :2] += np.diff(bdd, axis=0) / 1000.0 / float(cup.dt)
    return dataclasses.replace(cup, pos=pos, vel=vel, acc=acc, jerk=jerk)


def _bumped_meta(meta: uc.CycleMeta, i_rel: int, i_catch: Optional[int],
                 delta, b_rel, v_b, profile=None) -> uc.CycleMeta:
    """The meta with the bumped catch's site moved by Δ and the carried
    release's site / cup velocity moved by ``b_rel`` / ``v_b`` — so the splice
    guard, the report and the next seed read the release the plan executes.

    ``profile = (b, ḃ, b̈)`` (mm, per knot, the SAME arrays the plan was bumped
    with) also moves ``meta.cup_plan`` (I-PLAN-10): the cup track is the second
    representation of the plan, and ``unified_cycle._seam_velocity`` /
    ``state_at_knot`` read it at the next splice.  ``meta.tilts`` is untouched
    because the bump does not touch ``pose[:, 3:5]``.
    """
    rels = list(meta.releases)
    rm = rels[i_rel]
    off3 = np.array([b_rel[0], b_rel[1], 0.0])
    vel3 = np.array([v_b[0], v_b[1], 0.0])
    rels[i_rel] = dataclasses.replace(
        rm, site_mm=np.asarray(rm.site_mm, float) + off3,
        vel_mm_s=np.asarray(rm.vel_mm_s, float) + vel3)
    cats = list(meta.catches)
    if i_catch is not None:
        cm = cats[i_catch]
        cats[i_catch] = dataclasses.replace(
            cm, site_mm=np.asarray(cm.site_mm, float)
            + np.array([delta[0], delta[1], 0.0]))
    kw = dict(releases=tuple(rels), catches=tuple(cats))
    last_rel = i_rel == len(rels) - 1
    if last_rel and getattr(meta, 'takeoff_vel_mps', None) is not None:
        kw['takeoff_vel_mps'] = (np.asarray(meta.takeoff_vel_mps, float)
                                 + vel3 / 1000.0)
    if profile is not None and meta.cup_plan is not None:
        cup = _bumped_cup(meta.cup_plan, *profile)
        if last_rel:
            # The same rule as ``takeoff_vel_mps`` (which is read straight off
            # ``CupCyclePlan.takeoff_vel``): the terminal window's release.
            cup = dataclasses.replace(
                cup, takeoff_vel=np.asarray(cup.takeoff_vel, float) + vel3 / 1000.0)
        kw['cup_plan'] = cup
    return dataclasses.replace(meta, **kw)


def apply_catch_bump(plan: CyclePlan, meta: uc.CycleMeta, k_wire: int,
                     delta_xy_mm, t_catch_s: float, limits, geom, *,
                     pre_hold_s: float = BUMP_PRE_HOLD_S,
                     max_launch_lateral_mm_s: float = BUMP_MAX_LAUNCH_LATERAL_MM_S,
                     post_hold_knots: Optional[int] = None,
                     t_ramp_start_s: Optional[float] = None) -> BumpResult:
    """Bump the installed record's catch at ``t_catch_s`` (PLAN clock) by
    ``delta_xy_mm`` and retarget the release it carries — or decline.

    ``k_wire`` is the highest knot the wire may have read at the DISPATCH
    instant (``floor((t - t0)/dt) + WIRE_READ_KNOTS``); the bump is zero with
    zero derivatives through ``k_b = k_wire + BUMP_EXTRA_KNOTS``.  ``delta`` is
    the CLAMPED landing's xy minus the committed one (the caller's
    ``_clamp_lateral_to_schedule`` is the lateral authority).  The carried
    release is the first release after the catch; its flight and pre-hold are
    read off ``meta``.  ``t_ramp_start_s`` (plan clock, default ``k_b·dt``)
    may only move the ramp start LATER than ``k_b·dt`` (the offline probe's
    pre-production continuous start, kept to reproduce the design row); an
    earlier value is a contract violation and raises.  The returned plan is full-length (``plan.n_knots``),
    same ``dt``, hand arrays the SAME objects' values (bit-identical).
    """
    dt = float(plan.dt)
    delta = np.asarray(delta_xy_mm, float).reshape(2)
    k_b = int(k_wire) + BUMP_EXTRA_KNOTS
    t_b = k_b * dt
    if t_ramp_start_s is not None:
        if float(t_ramp_start_s) < t_b - 1e-12:
            raise ValueError('t_ramp_start_s %.4f before k_b*dt %.4f breaks I-CATCH-5'
                             % (t_ramp_start_s, t_b))
        t_b = float(t_ramp_start_s)
    t_c = float(t_catch_s)

    def decline(code, terminal, msg, **kw):
        return BumpResult(False, code, terminal, msg, k_b, delta_xy_mm=delta, **kw)

    if not np.all(np.isfinite(delta)):
        return decline(BUMP_NO_CARRY, True, 'non-finite delta %r' % (delta,))
    t_in_end = t_c - float(pre_hold_s)
    if t_in_end - t_b < BUMP_MIN_PHASE_KNOTS * dt - 1e-9:
        return decline(BUMP_FREEZE, True,
                       'ramp-in %.3f s from k_b %d to t_c - %.3f s < %d knots'
                       % (t_in_end - t_b, k_b, pre_hold_s, BUMP_MIN_PHASE_KNOTS))
    after = [(i, r) for i, r in enumerate(meta.releases) if float(r.t_s) > t_c + 1e-9]
    if not after:
        return decline(BUMP_NO_CARRY, True, 'no carried release after t_c %.3f s' % t_c)
    i_rel, rm = min(after, key=lambda ir: float(ir[1].t_s))
    t_rel = float(rm.t_s)
    flight = float(rm.flight_s)
    pre_s = int(rm.pre_hold_knots) * dt
    post_s = (uc.post_release_hold_knots() if post_hold_knots is None
              else int(post_hold_knots)) * dt
    t_end = float(plan.t[-1])
    if (t_rel - pre_s - t_c < BUMP_MIN_PHASE_KNOTS * dt - 1e-9
            or t_end - (t_rel + post_s) < BUMP_MIN_PHASE_KNOTS * dt - 1e-9):
        return decline(BUMP_NO_CARRY, True,
                       'carry %.3f s / settle %.3f s shorter than %d knots'
                       % (t_rel - pre_s - t_c, t_end - t_rel - post_s,
                          BUMP_MIN_PHASE_KNOTS))
    v_b, b_rel = retarget_parts(delta, t_c, t_rel, pre_s, flight)
    # The bound is on the TOTAL lateral launch velocity: the unbumped plan's own
    # release lateral velocity plus the bias (owner reading 2026-10-16).
    v_tot = np.asarray(rm.vel_mm_s, dtype=float)[:2] + v_b
    v_norm = float(np.hypot(v_tot[0], v_tot[1]))
    if v_norm > float(max_launch_lateral_mm_s) + 1e-9:
        return decline(BUMP_LAUNCH, False,
                       'total lateral launch velocity %.1f mm/s (plan + bias '
                       '%.1f) > %.1f mm/s'
                       % (v_norm, float(np.hypot(v_b[0], v_b[1])),
                          max_launch_lateral_mm_s),
                       release_offset_xy_mm=b_rel, launch_bias_xy_mm_s=v_b)
    b, bd, bdd = retarget_profile(plan.t, t_b, t_in_end, t_c, t_rel, pre_s,
                                  post_s, t_end, flight, delta, with_acc=True)
    # I-CATCH-5: every knot the wire may have read, and k_b itself, unchanged.
    b[:k_b + 1] = 0.0
    bd[:k_b + 1] = 0.0
    bdd[:k_b + 1] = 0.0
    pose = np.array(plan.pose, dtype=float, copy=True)
    pose_vel = np.array(plan.pose_vel, dtype=float, copy=True)
    pose[:, 0:2] += b
    pose_vel[:, 0:2] += bd
    assert np.array_equal(pose[:k_b + 1], plan.pose[:k_b + 1])
    assert np.array_equal(pose_vel[:k_b + 1], plan.pose_vel[:k_b + 1])
    k0 = max(0, k_b - BUMP_GATE_CONTEXT_KNOTS)
    rep = fz.validate_cycle(_plan_slice(pose, pose_vel, plan, k0, plan.n_knots - 1),
                            limits, geom)
    if rep.code != 'OK':
        return decline(rep.code, False, 'bumped plan refused: %s'
                       % ('; '.join(map(str, rep.reasons)) or rep.code),
                       report=rep, release_offset_xy_mm=b_rel,
                       launch_bias_xy_mm_s=v_b)
    new_plan = CyclePlan(pose, pose_vel, plan.hand_rev, plan.hand_vel_rps, dt,
                         catch_k=plan.catch_k, contact_knots=plan.contact_knots,
                         catch_evidence=plan.catch_evidence)
    # The bumped catch's mark: the touch-down nearest t_c within half a knot
    # (t_c is the caller's committed landing; the mark is the interpolated one).
    near = [(abs(float(cm.t_s) - t_c), i) for i, cm in enumerate(meta.catches)
            if abs(float(cm.t_s) - t_c) <= dt / 2.0]
    i_catch = min(near)[1] if near else None
    return BumpResult(True, 'OK', False,
                      'bump (%.1f, %.1f) mm from k_b %d; release offset '
                      '(%.2f, %.2f) mm, launch bias %.1f mm/s'
                      % (delta[0], delta[1], k_b, b_rel[0], b_rel[1], v_norm),
                      k_b, plan=new_plan,
                      meta=_bumped_meta(meta, i_rel, i_catch, delta, b_rel, v_b,
                                        profile=(b, bd, bdd)),
                      report=rep, delta_xy_mm=delta, release_offset_xy_mm=b_rel,
                      launch_bias_xy_mm_s=v_b)


def k_wire_at(t0_s: float, t_s: float, dt: float) -> int:
    """The highest knot the wire may have read at wall instant ``t_s`` for a
    record with origin ``t0_s`` — the same arithmetic as ``install_segment``."""
    return int(math.floor((float(t_s) - float(t0_s)) / float(dt))) + WIRE_READ_KNOTS


def bump_record(plan: CyclePlan, meta: uc.CycleMeta, t0_s: float,
                t_dispatch_s: float, delta_xy_mm, t_catch_abs_s: float,
                limits, geom, *, t_apply_s: Optional[float] = None,
                **kw) -> Tuple[BumpResult, int]:
    """Wall-clock wrapper: ``k_wire`` from the DISPATCH instant, plus the node's
    apply-time check (``k_b <= k_wire_at(t_apply_s)`` -> non-terminal
    ``SPLICE_TOO_LATE``, record unchanged).  Returns ``(result, k_wire)``."""
    dt = float(plan.dt)
    k_wire = k_wire_at(t0_s, t_dispatch_s, dt)
    if t_apply_s is not None:
        k_now = k_wire_at(t0_s, t_apply_s, dt)
        if k_wire + BUMP_EXTRA_KNOTS <= k_now:
            return BumpResult(False, BUMP_SPLICE_TOO_LATE, False,
                              'k_b %d <= k_wire %d at apply'
                              % (k_wire + BUMP_EXTRA_KNOTS, k_now),
                              k_wire + BUMP_EXTRA_KNOTS,
                              delta_xy_mm=np.asarray(delta_xy_mm, float)), k_wire
    return apply_catch_bump(plan, meta, k_wire, delta_xy_mm,
                            float(t_catch_abs_s) - float(t0_s), limits, geom,
                            **kw), k_wire
