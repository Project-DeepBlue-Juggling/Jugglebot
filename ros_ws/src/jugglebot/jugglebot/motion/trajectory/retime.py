"""Retiming-invariant duration model — propose T, let the unchanged gate verify.

A shaped rest-to-rest ``build_move`` is a **one-parameter family** in ``u = 1/T``
on a FIXED normalised-time grid ``s = t/T ∈ [0, 1]``. This module exploits that
structure to pick the minimal feasible duration in ONE monotone root-find plus one
verify pass through :func:`feasibility.validate`, replacing the legacy 6-pass
stretch+bisection search (``planner._min_feasible_move`` / ``_refine_shaped_min``).

The gate stays the source of truth: this module only *proposes* a duration; the
caller builds the shaped plan at that duration and runs the unchanged
:func:`feasibility.validate`. A rejection falls back to the legacy loop, so no
unvalidated plan is ever returned (the K-contract single-enforcement-point property
is preserved — every returned plan passed ``validate``).

Why the family is a fixed-u-power polynomial per sample
-------------------------------------------------------
For a rest-to-rest quintic from ``p0`` to ``target`` over duration ``T``, the base
pose is ``p(s) = p0 + (target-p0)·h(s)`` with ``h(s) = 10s³-15s⁴+6s⁵`` — the SHAPE
is fixed on the s-grid, independent of ``T``. Differentiating in real time
(``d/dt = u·d/ds``):

  * base twist  ∝ u    (``dp/dt = (target-p0)·h'(s)·u``)          — fixed vector × u
  * base accel  ∝ u²   (``d²p/dt² = (target-p0)·h''(s)·u²``)      — fixed vector × u²
  * base FD jerk ∝ u³  (finite-difference of leg-acc on the fixed s-grid: the
    difference scales as the acc's u-power and the ``1/dt = u·(n-1)`` factor adds
    one more u)

The lean superposition (``shaping._ShapedPlan``) adds, in POSE, a windowed tilt +
lever-arm xy shift that is ``∝ base accel ∝ u²``. So the lean's real-time
derivatives pick up extra u's exactly as the base did:

  * lean pose  ∝ u²  ⇒  lean twist ∝ u³  ⇒  lean accel ∝ u⁴  (⇒ lean FD jerk ∝ u⁵)

Mapping to leg space through the Jacobian chain (``leg_vel = J·twist``,
``leg_acc = J·accel + J̇·twist``) and evaluating ``J`` / ``J̇`` at the REFERENCE
shaped pose (see the second-order note below) gives per-sample leg quantities that
are pure polynomials in ``r = u/u_ref = T_ref/T``:

  * leg vel  = av1·r  + av3·r³
  * leg acc  = aa2·r² + aa4·r⁴ + aa6·r⁶
  * leg jerk = aj3·r³ + aj5·r⁵ + aj7·r⁷

where (per sample, per leg):

  * av1 = J·base_twist,  av3 = J·lean_twist                            (u, u³)
  * aa2 = J·base_accel + J̇_base·base_twist                            (u²)
  * aa4 = J·lean_accel + J̇_base·lean_twist + J̇_lean·base_twist        (u⁴)
  * aa6 = J̇_lean·lean_twist                                           (u⁶, lean×lean)
  * aj{3,5,7} = (n-1)/T_ref · Δ_s(aa{2,4,6})                          (FD adds one r)

``J̇_base`` / ``J̇_lean`` are ``compute_jacobian_dot`` evaluated with the base and
lean twists separately; the central finite difference is linear in the twist (to
O(dt)), so ``J̇(base+lean) = J̇_base + J̇_lean`` and the r⁴/r⁶ cross terms follow.

The J-at-shaped-pose second-order error and the 1.02 inflation
-------------------------------------------------------------
Two approximations enter: (1) ``J`` is evaluated at the reference shaped pose (lean
frozen at ``T_ref``) rather than the true shaped pose at the answer ``T`` — the lean
pose perturbation is ≤5° tilt + a few mm, so ``J`` barely moves; (2) ``J̇`` is
linearised in the twist. Measured across the operating band the resulting leg
vel/acc/jerk match the unchanged gate to **<1.5 %** (empirically <0.3 % on the
battery moves — see the module's validation in ``tests/motion/test_retime.py``).
The proposed ``T_min`` is inflated by :data:`RETIME_SAFETY_INFLATE` (1.02) to more
than cover that residual before the verify pass, so the verify accepts essentially
always (0/42 rejects on the prototype corpus); a rejection still falls back cleanly.

The 5° tilt cap and the cap-free reference
------------------------------------------
The lean tilt is ``∝ u²`` and is hard-capped at ``LEAN_TILT_CAP_DEG`` (5°) inside
the shaper. Extracting the lean geometry at a ``T_ref`` where the cap is ACTIVE
would bake a distorted (capped) lean into the coefficients. So the reference is
built at a **cap-free** ``T_ref`` (:func:`_cap_free_reference_T`): a slow duration
where the added tilt is comfortably under the cap. The extracted lean is then the
true un-capped lean; at a shorter answer ``T`` the real gate can only cap tilt
DOWN, making actual leg peaks ≤ the model's — i.e. the model is conservative in the
(infeasible-anyway) cap region. In practice feasible durations sit far below the
cap threshold, so the cap never binds at the answer.

Why the per-sample model, not an aggregate power-law
----------------------------------------------------
The binding sample switches with ``T`` (the s-location of the worst leg jerk moves
as the lean re-weights), and the cap distorts an aggregate fit — a 2-point
aggregate power-law refit rejected 39/42 corpus cases. The per-sample polynomial
tracks the moving binding sample exactly.

Pure Python + numpy. No ROS2 / repo-root imports.
"""

from __future__ import annotations

import numpy as np

from jugglebot.motion.trajectory.feasibility import (
    _SHAPED_VALIDATE_SAMPLES,
    _VALIDATE_JERK_MARGIN,
    _batched_jacobian,
    _batched_jacobian_dot,
)
from jugglebot.motion.trajectory.plan import TrajectoryPlan
from jugglebot.motion.trajectory.segment import POSE_DIM, QuinticSegment
from jugglebot.motion.trajectory.shaping import (
    _base_states_batch,
    batched_shaped_states,
)

# Safety inflation applied to the model's raw minimal feasible duration before the
# verify pass. Covers the <1.5 % J-at-shaped-pose / J̇-linearisation residual (see
# the module docstring) with room, so the one verify pass accepts essentially always
# while the planned duration still lands ~+2 % over the true optimum — far tighter
# than the legacy bisection's ~+8 %.
RETIME_SAFETY_INFLATE = 1.02

# Fraction of the tilt cap the cap-free reference search stays under. A reference
# with added tilt below this fraction of LEAN_TILT_CAP_DEG has an un-capped lean.
_CAP_FREE_FRAC = 0.80

# Number of coarse samples for the (Jacobian-free) cap-free reference search.
_CAP_PROBE_SAMPLES = 64
_CAP_PROBE_ITERS = 8

# Bisection budget for the monotone T solve. 60 halvings resolve T to ~1e-16 of the
# bracket — deterministic and far finer than the 1 ms the model agrees with the gate.
_SOLVE_ITERS = 60
# Cap on the doubling used to bracket a feasible (slow) duration. A move that is not
# feasible even at min_move·2^_BRACKET_DOUBLINGS is pathological → return None so the
# caller falls back to the legacy loop rather than trust an unbracketed model.
_BRACKET_DOUBLINGS = 40


def _rest_segment(pose, target, T):
    return QuinticSegment(
        p0=np.asarray(pose, float), v0=np.zeros(POSE_DIM), a0=np.zeros(POSE_DIM),
        p1=np.asarray(target, float), v1=np.zeros(POSE_DIM), a1=np.zeros(POSE_DIM),
        duration=float(T))


def _cap_free_reference_T(pose, target, shaper, min_T):
    """Pick a slow ``T_ref`` at which the added lean tilt is below the cap.

    The lean tilt magnitude scales as ``u² = 1/T²``, so if a candidate ``T`` shows a
    peak added tilt of ``m`` the tilt at ``T·sqrt(m / (frac·cap))`` is ~``frac·cap``.
    Iterates from a moderate start until the peak added tilt is under
    ``_CAP_FREE_FRAC`` of the shaper cap (Jacobian-free — pose sampling only). A
    zero-lean move (no lateral accel, or gain 0) returns the start immediately.
    """
    cap = float(getattr(shaper, 'tilt_cap_rad', np.inf))
    T = max(float(min_T) * 6.0, 1.0)
    ts_frac = np.linspace(0.0, 1.0, _CAP_PROBE_SAMPLES)
    for _ in range(_CAP_PROBE_ITERS):
        seg = _rest_segment(pose, target, T)
        shaped = shaper.shape(TrajectoryPlan((seg,), np.asarray(target, float)))
        ts = ts_frac * T
        base_pose, _, _ = _base_states_batch(seg, ts)
        shp_pose, _, _ = batched_shaped_states(shaped, ts)
        added = shp_pose[:, 3:5] - base_pose[:, 3:5]
        peak_tilt = float(np.max(np.hypot(added[:, 0], added[:, 1])))
        if not np.isfinite(cap) or peak_tilt <= _CAP_FREE_FRAC * cap or peak_tilt == 0.0:
            return T
        # tilt ∝ 1/T² → grow T so the peak tilt drops to ~_CAP_FREE_FRAC·cap.
        T *= float(np.sqrt(peak_tilt / (_CAP_FREE_FRAC * cap)))
    return T


class RetimeModel:
    """Per-sample leg vel/acc/jerk u-power coefficients for one shaped rest move.

    Built once at a cap-free ``T_ref`` by sampling the reference shaped geometry and
    decomposing the base/lean Jacobian-chain contributions (see the module
    docstring). :meth:`peaks` and :meth:`min_feasible_T` then evaluate any ``T``
    analytically — no further ``validate`` passes.
    """

    def __init__(self, T_ref, av1, av3, aa2, aa4, aa6, aj3, aj5, aj7):
        self.T_ref = float(T_ref)
        self.av1, self.av3 = av1, av3
        self.aa2, self.aa4, self.aa6 = aa2, aa4, aa6
        self.aj3, self.aj5, self.aj7 = aj3, aj5, aj7

    def peaks(self, T):
        """Model peak (leg vel, leg acc, leg jerk) at duration ``T``.

        Jerk carries :data:`feasibility._VALIDATE_JERK_MARGIN` — the SAME inflation
        the gate applies — so the model's feasibility boundary tracks the gate's.
        """
        r = self.T_ref / float(T)
        r2, r3 = r * r, r ** 3
        vel = self.av1 * r + self.av3 * r3
        acc = self.aa2 * r2 + self.aa4 * r2 * r2 + self.aa6 * r2 ** 3
        jerk = self.aj3 * r3 + self.aj5 * r3 * r2 + self.aj7 * r3 * r2 * r2
        peak_vel = float(np.max(np.abs(vel)))
        peak_acc = float(np.max(np.abs(acc)))
        peak_jerk = float(np.max(np.abs(jerk))) * _VALIDATE_JERK_MARGIN
        return peak_vel, peak_acc, peak_jerk

    def _feasible(self, T, limits):
        pv, pa, pj = self.peaks(T)
        return (pv <= limits.leg_vel_mmps
                and pa <= limits.leg_acc_mmps2
                and pj <= limits.leg_jerk_mmps3)

    def min_feasible_T(self, limits, T_floor):
        """Smallest ``T ≥ T_floor`` the model deems feasible (monotone bisection).

        Faster moves (smaller ``T``) have strictly larger leg peaks, so feasibility
        flips once as ``T`` grows — a monotone predicate a bisection resolves
        exactly. Returns ``T_floor`` when the floor itself is already feasible, or
        ``None`` when no feasible duration can be bracketed within
        ``_BRACKET_DOUBLINGS`` (a pathological move → caller falls back).
        """
        T_floor = float(T_floor)
        if self._feasible(T_floor, limits):
            return T_floor
        # Bracket a feasible (slow) T above the infeasible floor.
        T_hi = max(T_floor, self.T_ref)
        doublings = 0
        while not self._feasible(T_hi, limits):
            T_hi *= 2.0
            doublings += 1
            if doublings > _BRACKET_DOUBLINGS:
                return None
        T_lo = T_floor                      # infeasible
        for _ in range(_SOLVE_ITERS):
            mid = 0.5 * (T_lo + T_hi)
            if self._feasible(mid, limits):
                T_hi = mid
            else:
                T_lo = mid
        return T_hi                         # the feasible side of the boundary


def build_model(pose, target, geom, shaper, *, T_ref=None,
                n_samples=_SHAPED_VALIDATE_SAMPLES, min_T=0.2):
    """Build a :class:`RetimeModel` for the shaped rest move ``pose → target``.

    Reuses the batched shaped sampler (``shaping.batched_shaped_states``) and the
    batched analytic Jacobian chain (``feasibility._batched_jacobian`` /
    ``_batched_jacobian_dot``) — the EXACT machinery the verify gate uses — so the
    model is built on the same shaped-pose evaluation it will be checked against.
    ``T_ref`` defaults to a cap-free reference (:func:`_cap_free_reference_T`).
    """
    pose = np.asarray(pose, float)
    target = np.asarray(target, float)
    if T_ref is None:
        T_ref = _cap_free_reference_T(pose, target, shaper, min_T)
    T_ref = float(T_ref)

    seg = _rest_segment(pose, target, T_ref)
    shaped = shaper.shape(TrajectoryPlan((seg,), target))
    n = int(n_samples)
    ts = T_ref * np.arange(n) / (n - 1)

    base_pose, base_tw, base_ac = _base_states_batch(seg, ts)
    shp_pose, shp_tw, shp_ac = batched_shaped_states(shaped, ts)
    lean_tw = shp_tw - base_tw
    lean_ac = shp_ac - base_ac

    # J / J̇ at the reference SHAPED pose (best approximation to the true shaped pose;
    # the lean perturbation of J is the <1.5 % second-order term the 1.02 covers).
    J = _batched_jacobian(shp_pose, geom)[0]              # (n,6,6)
    Jd_base = _batched_jacobian_dot(shp_pose, base_tw, geom)
    Jd_lean = _batched_jacobian_dot(shp_pose, lean_tw, geom)

    av1 = np.einsum('nij,nj->ni', J, base_tw)             # leg vel, u¹
    av3 = np.einsum('nij,nj->ni', J, lean_tw)             # leg vel, u³
    aa2 = (np.einsum('nij,nj->ni', J, base_ac)
           + np.einsum('nij,nj->ni', Jd_base, base_tw))   # leg acc, u²
    aa4 = (np.einsum('nij,nj->ni', J, lean_ac)
           + np.einsum('nij,nj->ni', Jd_base, lean_tw)
           + np.einsum('nij,nj->ni', Jd_lean, base_tw))   # leg acc, u⁴
    aa6 = np.einsum('nij,nj->ni', Jd_lean, lean_tw)       # leg acc, u⁶ (lean×lean)

    # Jerk = FD of leg acc on the fixed s-grid. Δ_s(aa_k) scales as the acc's r-power;
    # the 1/dt = u·(n-1) = r·(n-1)/T_ref factor adds one r → aj_k = (n-1)/T_ref·Δaa.
    fd = (n - 1) / T_ref
    aj3 = fd * np.diff(aa2, axis=0)                       # leg jerk, u³
    aj5 = fd * np.diff(aa4, axis=0)                       # leg jerk, u⁵
    aj7 = fd * np.diff(aa6, axis=0)                       # leg jerk, u⁷

    return RetimeModel(T_ref, av1, av3, aa2, aa4, aa6, aj3, aj5, aj7)


def propose_move_duration(pose, target, limits, geom, shaper, *,
                          inflate=RETIME_SAFETY_INFLATE):
    """Propose the minimal feasible shaped-move duration, or ``None`` to fall back.

    Builds the retiming model, solves the monotone min-feasible ``T``, applies the
    :data:`RETIME_SAFETY_INFLATE` safety inflation, and floors at
    ``limits.min_move_duration_s``. Returns ``None`` (caller runs the legacy loop)
    when the model cannot bracket a feasible duration or a numerical pathology is
    hit — the model is a *proposal*, never a source of truth, so any doubt defers to
    the unchanged gate. The returned ``T`` is NOT itself validated here; the caller
    runs the one mandatory verify pass through :func:`feasibility.validate`.
    """
    try:
        floor = float(limits.min_move_duration_s)
        model = build_model(pose, target, geom, shaper, min_T=floor)
        t_min = model.min_feasible_T(limits, floor)
        if t_min is None or not np.isfinite(t_min):
            return None
        return max(float(t_min) * float(inflate), floor)
    except (FloatingPointError, ValueError, np.linalg.LinAlgError):
        return None
