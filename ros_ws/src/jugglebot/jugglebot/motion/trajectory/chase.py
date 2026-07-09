"""Chase clamp — the follower's feasible-progress limiter (frozen-J, linear-in-α).

The 2026-07-09 S3 spacemouse forensics showed the reject-and-keep-last-plan
follower design failing structurally: stick reversals hand ``build_follow``
high-energy seeds its stretch ladder cannot fix (the candidate's *decel* half is
what violates, and stretching bends a moving-seed path), producing ~0.43 s
accept/reject limit cycles with legs sustained 10–45 % OVER the velocity limit.
The fix is to stop asking the gate "can I reach the stick?" and instead compute,
BEFORE building the candidate, *how far toward the stick this tick's plan can
feasibly go* — then plan to that chased point. The gate stays canonical (every
plan is still ``validate_follow``-checked); the chase is advisory and merely
makes the candidate nearly-always acceptable.

Decomposition (exact under a Jacobian frozen at the seed pose): the candidate is
a single rest-terminating quintic from the seed state ``(p0, v0, a0)`` to
``p0 + α·Δ`` over a horizon ``T``. Per pose axis it splits as

    pose(t) = base(t; p0, v0, a0 → p0, rest)  +  α · s(t) · Δ

where ``base`` is the decel-to-rest-in-place quintic (same BCs, zero net motion)
and ``s(t)`` the 0→1 rest-to-rest quintic shape. Both terms are quintics with the
candidate's exact boundary conditions, and quintic interpolation is linear in its
endpoints, so the split is EXACT — the leg-space derivative at any sample is
``w_n(t) + α·u_n(t)`` (n = vel/acc/jerk) under the frozen J. Each sample ×
constraint × leg then yields an α interval from ``|w + α·u| ≤ safety·L``; the
intersection over all of them (and the knot-step bound, expressed as a leg-rate
limit) is the feasible progress band. We take its upper end: maximum progress.

Horizon (A1 — energy + distance scaled, replaces the prototype's fixed T):
``T_chase = clip(max(horizon_s, T_base, T_delta), horizon_s, CHASE_T_MAX)``.

  * ``T_base`` makes the decel-in-place base feasible BY CONSTRUCTION at
    ``safety·limits``: its peaks follow the quintic basis-polynomial extrema
    (verified numerically to 5 digits, 2026-07-09) — acc ≈ 3.94·v_pk/T
    (max|h1''|), jerk(0) = 36·v_pk/T² + 9·a_pk/T (|h1'''(0)|, |h2'''(0)|).
  * ``T_delta`` sizes the α=1 rest-to-rest delta term: vel 1.875·|Δl|/T,
    acc 5.774·|Δl|/T², jerk 60·|Δl|/T³ (the s(t) shape extrema).
  * The ``CHASE_T_MAX`` cap bounds ``validate_follow``'s O(T) knot loop
    (~85 knots ≈ 9 ms at 2.0 s); α < 1 absorbs whatever the cap cut off.

Without the T scaling the prototype tracked at only ~20–35 % of the leg velocity
limit (the fixed-T base decel ate the whole budget); with it, cruise rides just
under ``safety`` × the session velocity limit.

Empty intersection (A2): when even α = 0 is infeasible (a sample where the base
alone violates and the delta term cannot unload it — pathological/corrupt seeds,
or a seed a hair over ``safety·L`` where the delta basis is exactly zero), there
is no feasible candidate at ALL; the follower routes the tick to
``build_graceful_stop`` instead of building a doomed plan. ``feasible=False``
signals that. Every chase error degrades to a gate reject downstream — never to
un-gated motion.

CHASE_SAFETY absorbs the frozen-J error (J drifts over the plan) and the gate's
1.05 jerk inflation. Re-swept (A6) on THIS Jetson through the production
``TargetFollower.follow`` (``temp/probes/chase_sweep.py``, run 2026-07-09), two
regimes per session tier:

  PRODUCTION regime — incremental random-walk target (the real stick, ~4 mm/tick),
  2000 ticks. This is the operating point that matters, and it is clean at every
  tier:
    * defaults 100/400/8000:    0 reject streak, p99 10.9 ms, max 19.5 ms
    * S4 156/660/10331:         0 reject streak, p99  8.6 ms
    * ceilings 280/4000/200000: 0 reject streak, p99  6.2 ms

  STRESS regime — 500 far-out-of-envelope (160 mm) random targets, a fresh one
  EVERY tick (reject_hunt style), moving seeds chained through accepted plans.
  Deliberately adversarial (no real stick teleports 160 mm each 25 ms):
    * defaults:  98.2 % single-validate, 100 % ≤ 1 stretch, 0 rejects;
                 cost p99 26 ms — the O(T) gate at the 2.0 s cap, EVERY tick.
    * S4:        98.8 % single-validate, 100 % ≤ 1 stretch, 0 rejects, p99 18 ms.
    * ceilings:  99.8 % single-validate, 100 % ≤ 1 stretch, but ~70/500 residual
                 WORKSPACE rejects — a moving-seed quintic BULGING past the hard
                 bound as α≈1 tracking pushes the chased pose onto the workspace
                 margin. These are SPATIAL (workspace), NOT leg-limit, so
                 CHASE_SAFETY does not move them (verified: 0.85 vs 0.70 → identical
                 count); they are the "rare spatial class → keep-last, self-heal
                 next tick" the follower is designed around (spec C2.5) and do NOT
                 appear in the production regime above. The stress cost p99 is
                 non-blocking: publish-first (C3) puts the replan AFTER the wire, so
                 a 26 ms block delays only the next publish by ~1 ms — far under the
                 ~117 ms firmware decay→sprint threshold.

ACCEPTED A6 LIMITATION (not a defect to chase): the binding A6 gate ("0 residual
rejects at EACH tier") is met at every tier in the PRODUCTION random-walk regime
but NOT in the adversarial far-jump STRESS regime at the ceilings tier (~70/500
WORKSPACE rejects). That residual is the self-healing spatial class above — outside
CHASE_SAFETY's reach (it governs only leg-limit rejects, of which both regimes have
zero). It is documented and accepted, not tuned away; the deferred STRUCTURAL fix is
a workspace-aware chase term (clamp α against the leg-extension bound directly, not
only the leg vel/acc/jerk/step limits), which would drive the stress ceilings tier to
zero too. Until then the keep-last self-heal (spec C2.5) covers it in production.

CHASE_SAFETY stays 0.85 (the prototype-validated value): the A6 knob only governs
leg-limit rejects (LIMIT_VEL/ACC/JERK/STEP), of which BOTH regimes have zero at
every tier — the only residual rejects are the workspace-overshoot class above,
which the safety factor provably cannot address. CHASE_T_MAX stays 2.0 s: the
production cost is well inside budget and publish-first makes the stress-worst-case
non-blocking; a lower cap would only slow the far-target tracking A1 restored.

Pure Python + numpy. No ROS2 / repo-root imports.
"""

from __future__ import annotations

from typing import NamedTuple

import numpy as np

from jugglebot.motion.ik_solver import compute_jacobian, rotvec_to_rot_matrix
from jugglebot.motion.trajectory.feasibility import STEP_BOUND_MARGIN

# Fraction of each leg-space limit the chase plans against. NOT a new limit: the
# gate still validates every plan at the FULL session limits — this is headroom
# so the frozen-J approximation error (and the gate's 1.05 jerk margin) land the
# candidate inside the gate instead of a hair over it. See the module docstring
# for the empirical sweep that pinned 0.85.
CHASE_SAFETY = 0.85

# Horizon cap (s): bounds the fast gate's O(T) knot-step loop (~9 ms at 2.0 s)
# so the post-publish follow block stays far under the ~117 ms firmware
# decay→sprint threshold. Always above T_base for every session tier
# (T_base(v=L_v) = 3.94·L_v/(0.85·L_a) ≈ 1.16 s at defaults, less at S4 and the
# ceilings), so the cap only ever trims T_delta — absorbed by α < 1.
CHASE_T_MAX = 2.0

# Normalised-time sample count for the interval intersection. 41 matches the
# validated prototype; the quintic derivative extrema are broad, so 41 samples
# under-measure a true peak by well under the safety headroom.
_CHASE_SAMPLES = 41

# Quintic basis-polynomial extrema (numerically verified 2026-07-09; see the
# module docstring). _B_* are the decel-in-place base's peaks per unit v0/a0;
# _D_* the 0→1 rest-to-rest shape's derivative peaks per unit Δ.
_B_ACC_V = 3.941     # max|h1''|  — base acc per (v_pk/T)
_B_JERK_V = 36.0     # |h1'''(0)| — base jerk per (v_pk/T²)
_B_JERK_A = 9.0      # |h2'''(0)| — base jerk per (a_pk/T)
# Adverse alignment of the a0 basis at the h1'' interior peak (h2''(0.243) ≈
# −0.27; 0.3 is a mildly conservative round-up). Only steers the T pick — the
# interval intersection enforces the true constraint sample-by-sample.
_B_ACC_A = 0.3
_D_VEL = 1.875       # max s'    — delta vel per (|Δl|/T)
_D_ACC = 5.774       # max|s''|  — delta acc per (|Δl|/T²)
_D_JERK = 60.0       # |s'''(0)| — delta jerk per (|Δl|/T³)

# Normalised sample grid + the delta-shape derivative arrays (precomputed once;
# real-time derivatives divide by T, T², T³ per call).
_TS = np.linspace(0.0, 1.0, _CHASE_SAMPLES)
_S1 = 30.0 * _TS**2 - 60.0 * _TS**3 + 30.0 * _TS**4
_S2 = 60.0 * _TS - 180.0 * _TS**2 + 120.0 * _TS**3
_S3 = 60.0 - 360.0 * _TS + 360.0 * _TS**2


class ChaseResult(NamedTuple):
    """Outcome of one :func:`chase_alpha` call.

    ``pose`` is the chased target ``seed + alpha·(target − seed)``;
    ``horizon_s`` the energy/distance-scaled duration the candidate plan must be
    built over (the α interval was computed AT this T — building at another T
    invalidates it); ``feasible=False`` flags an empty α interval (no feasible
    candidate exists at any α — route to a graceful stop, A2).
    """
    pose: np.ndarray
    alpha: float
    horizon_s: float
    feasible: bool


def _base_derivs(v0: float, a0: float, T: float, ts: np.ndarray):
    """Vel/acc/jerk samples of the decel-in-place quintic from ``(0, v0, a0)``
    to ``(0, 0, 0)`` over ``T``, at normalised times ``ts``.

    Kept as the prototype's 3×3 boundary-condition solve (exact, ~0.1 ms total
    across the ≤ 6 nonzero axes) rather than a hand-expanded closed form — the
    solve IS the closed form, without transcription risk.
    """
    t = ts * T
    c1 = v0
    c2 = a0 / 2.0
    A = np.array([[T**3, T**4, T**5],
                  [3 * T**2, 4 * T**3, 5 * T**4],
                  [6 * T, 12 * T**2, 20 * T**3]])
    b = np.array([-(c1 * T + c2 * T**2), -(c1 + 2 * c2 * T), -(2 * c2)])
    c3, c4, c5 = np.linalg.solve(A, b)
    v = c1 + 2 * c2 * t + 3 * c3 * t**2 + 4 * c4 * t**3 + 5 * c5 * t**4
    a = 2 * c2 + 6 * c3 * t + 12 * c4 * t**2 + 20 * c5 * t**3
    j = 6 * c3 + 24 * c4 * t + 60 * c5 * t**2
    return v, a, j


def _interval_bounds(w: np.ndarray, u: np.ndarray, limit: float):
    """Per-element α interval from ``|w + α·u| ≤ limit`` → ``(lo, hi)`` arrays.

    ``u > 0``: α ∈ [(−L−w)/u, (L−w)/u]; ``u < 0``: the same endpoints swapped;
    ``u == 0``: unconstrained if ``|w| ≤ L``, EMPTY otherwise (encoded as the
    inverted interval (+inf, −inf) so the caller's max/min intersection turns it
    into lo > hi). This true intersection is what replaces the prototype's
    ``|w| > L → α = 0`` forcing: forward delta motion UNLOADS the base's braking
    on many samples, so the interval is often nonempty (α strictly > 0) exactly
    where the old forcing collapsed to zero progress.
    """
    with np.errstate(divide='ignore', invalid='ignore'):
        hi = np.where(u > 0, (limit - w) / u,
                      np.where(u < 0, (-limit - w) / u,
                               np.where(np.abs(w) <= limit, np.inf, -np.inf)))
        lo = np.where(u > 0, (-limit - w) / u,
                      np.where(u < 0, (limit - w) / u,
                               np.where(np.abs(w) <= limit, -np.inf, np.inf)))
    return lo, hi


def _chase_horizon(v_pk: float, a_pk: float, dl_pk: float, vel_limit: float,
                   acc_limit: float, jerk_limit: float, horizon_s: float
                   ) -> float:
    """The energy + distance scaled candidate horizon (A1).

    ``v_pk/a_pk`` are the frozen-J leg-space seed peaks, ``dl_pk`` the peak leg
    delta ``max|J·Δ|``; the limits are already safety-scaled. Returns
    ``clip(max(horizon_s, T_base, T_delta), horizon_s, CHASE_T_MAX)``.
    """
    # T_base — decel-in-place feasibility. Acc: the interior 3.94·v_pk/T peak
    # with the a0 basis adversely aligned (≤ _B_ACC_A·a_pk there); if a_pk alone
    # eats the budget no T helps (the t=0 acc IS a0) — the intersection handles
    # that sample, so just saturate at the cap.
    acc_room = acc_limit - _B_ACC_A * a_pk
    if v_pk > 1e-12:
        t_acc = (_B_ACC_V * v_pk / acc_room) if acc_room > 1e-9 else CHASE_T_MAX
        # Jerk peaks at t=0 where both bases peak together:
        # 36·v_pk/T² + 9·a_pk/T ≤ L_j — solve the quadratic in x = 1/T.
        x = ((-_B_JERK_A * a_pk
              + np.sqrt((_B_JERK_A * a_pk)**2
                        + 4.0 * _B_JERK_V * v_pk * jerk_limit))
             / (2.0 * _B_JERK_V * v_pk))
        t_jerk = (1.0 / x) if x > 1e-12 else CHASE_T_MAX
        t_base = max(t_acc, t_jerk)
    elif a_pk > 1e-12:
        t_base = _B_JERK_A * a_pk / max(jerk_limit, 1e-9)
    else:
        t_base = 0.0

    # T_delta — the α=1 rest-to-rest delta term within the safety-scaled limits.
    t_delta = max(
        _D_VEL * dl_pk / max(vel_limit, 1e-9),
        float(np.sqrt(_D_ACC * dl_pk / max(acc_limit, 1e-9))),
        float(np.cbrt(_D_JERK * dl_pk / max(jerk_limit, 1e-9))))

    return float(np.clip(max(horizon_s, t_base, t_delta),
                         horizon_s, CHASE_T_MAX))


def chase_alpha(state0, target_pose, limits, geom, horizon_s: float,
                safety: float = CHASE_SAFETY) -> ChaseResult:
    """Largest feasible progress ``α`` toward ``target_pose`` this tick.

    ``state0`` is the seed ``(pose, twist, accel)`` (the active plan sampled at
    now — already workspace-valid by construction); ``target_pose`` the (already
    margin-clamped) target. Returns a :class:`ChaseResult` — see the module
    docstring for the decomposition, the horizon scaling, and the empty-interval
    (A2) routing contract. The caller must build the candidate over
    ``result.horizon_s`` — the α interval is only valid at that T.

    Raises ``ValueError`` on non-finite inputs (defensive: the follower's inputs
    are already gated upstream; a NaN here would silently poison every interval
    comparison below, so fail loudly instead).
    """
    cur = np.asarray(state0[0], dtype=float)
    v0 = np.asarray(state0[1], dtype=float)
    a0 = np.asarray(state0[2], dtype=float)
    target = np.asarray(target_pose, dtype=float)
    if not (np.all(np.isfinite(cur)) and np.all(np.isfinite(v0))
            and np.all(np.isfinite(a0)) and np.all(np.isfinite(target))):
        raise ValueError("chase_alpha: non-finite seed state or target")

    delta = target - cur
    J = compute_jacobian(cur[:3], rotvec_to_rot_matrix(cur[3:6]), geom)
    ld = J @ delta                                    # leg-space delta (6,)
    v_pk = float(np.max(np.abs(J @ v0)))
    a_pk = float(np.max(np.abs(J @ a0)))
    dl_pk = float(np.max(np.abs(ld)))

    ls_vel = safety * float(limits.leg_vel_mmps)
    ls_acc = safety * float(limits.leg_acc_mmps2)
    ls_jerk = safety * float(limits.leg_jerk_mmps3)
    # Knot-step bound as a leg-rate limit: |Δu0| per 25 ms knot ≤ margin·max_step
    # ⇒ leg rate (rev/s) ≤ margin·max_step/knot_dt. The rate proxy for the true
    # 25 ms integral is conservative under the 0.8 margin + safety (the peak rate
    # over-estimates the mean rate across a knot).
    mm_to_rev = np.asarray(geom.mm_to_rev, dtype=float)
    ls_step = safety * (STEP_BOUND_MARGIN * float(limits.max_step_rev)
                        / float(limits.knot_dt_s))
    # For the horizon pick, fold the step bound into the velocity limit via the
    # tightest leg's mm→rev scale (the intersection below applies it per leg).
    vel_sel = min(ls_vel, ls_step / float(np.max(mm_to_rev)))

    T = _chase_horizon(v_pk, a_pk, dl_pk, vel_sel, ls_acc, ls_jerk,
                       float(horizon_s))

    # Base (decel-in-place) leg-space derivative samples, axes superposed under
    # the frozen J: W_n(t) = Σ_ax J[:, ax] · base_deriv_ax(t)  → (41, 6).
    n = len(_TS)
    W1 = np.zeros((n, 6))
    W2 = np.zeros((n, 6))
    W3 = np.zeros((n, 6))
    for ax in range(6):
        if v0[ax] != 0.0 or a0[ax] != 0.0:
            bv, ba, bj = _base_derivs(v0[ax], a0[ax], T, _TS)
            col = J[:, ax]
            W1 += np.outer(bv, col)
            W2 += np.outer(ba, col)
            W3 += np.outer(bj, col)
    # Delta-shape leg derivatives per unit α.
    U1 = np.outer(_S1 / T, ld)
    U2 = np.outer(_S2 / T**2, ld)
    U3 = np.outer(_S3 / T**3, ld)

    alpha_lo = 0.0
    alpha_hi = 1.0
    for W, U, L in ((W1, U1, ls_vel), (W2, U2, ls_acc), (W3, U3, ls_jerk),
                    (W1 * mm_to_rev, U1 * mm_to_rev, ls_step)):
        lo, hi = _interval_bounds(W.ravel(), U.ravel(), L)
        alpha_lo = max(alpha_lo, float(np.max(lo)))
        alpha_hi = min(alpha_hi, float(np.min(hi)))

    if alpha_lo > alpha_hi:
        # Empty interval: no α makes the candidate feasible at this T (base
        # infeasible and the delta term cannot unload it). A2: the follower
        # routes to build_graceful_stop; return the seed pose (zero progress).
        return ChaseResult(cur.copy(), 0.0, T, False)

    # Maximum progress within the feasible band. α_hi ≥ α_lo ≥ 0 and ≤ 1 by the
    # initial bounds; α = 0 (a legal decel-to-rest-in-place candidate) is the
    # natural collapse for an over-energetic seed.
    alpha = alpha_hi
    return ChaseResult(cur + alpha * delta, alpha, T, True)
