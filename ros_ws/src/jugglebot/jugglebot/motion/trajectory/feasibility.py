"""The single canonical feasibility gate for all platform motion.

Every plan constructor in ``planner.py`` calls :func:`validate` before returning,
so **nothing that moves the platform bypasses this gate** — that single-enforcement
-point property is what makes the loud-rejection guarantee hold (see the MVP plan's
"single most load-bearing convention").

Phase 2 scope (this file): the **full** gate. In order (first failure wins the
``code``):

1. **Geometry** per dense sample (80/segment default; shaped plans floored at 1600 —
   see ``_SHAPED_VALIDATE_SAMPLES``): non-finite reject (``UNREACHABLE``);
   leg extensions within the hard stroke margins (``WORKSPACE``); Jacobian
   condition number under the workspace hard bound (``UNREACHABLE`` — a
   near-singular pose is unrealisable no matter the timing).
2. **Leg kinematic peaks** via the ``ik_solver`` Jacobian chain: peak leg velocity
   (``LIMIT_VEL``), acceleration (``LIMIT_ACC``), and jerk (``LIMIT_JERK``, from
   the finite difference of the analytic leg acceleration — a second-order-accurate
   stand-in for the third difference of leg position).
3. **Knot-step bound** (``STEP_BOUND``): the actual wire ``u0`` sequence is the plan
   sampled at the 25 ms knot spacing and mapped ``ext × mm_to_rev``; the max
   per-knot ``|Δu0|`` must stay under ``max_step_rev`` with a 20 % margin so a
   move is rejected here, before motion, rather than mid-stream at the pump.

Timing (``TOO_FAST``) and the duration-stretch loop live in ``planner.build_move``,
which drives :func:`validate` iteratively; this function is a **pure predicate over
a fully-specified plan** — it never mutates durations. It always computes *all*
peaks (a full pass, no early return on a limit failure) so the planner can read the
worst-ratio off one report and stretch in a single step. Only the geometry checks
early-return (a non-finite / out-of-stroke / singular pose makes the Jacobian peaks
meaningless).

Pure Python + numpy. No ROS2 / repo-root imports.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from jugglebot.motion.ik_solver import (
    accel_to_leg_accels,
    compute_jacobian,
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
    twist_to_leg_velocities,
)
from jugglebot.motion.trajectory.shaping import _ShapedPlan, batched_shaped_states
from jugglebot.motion.workspace import (
    WorkspaceLimits,
    check_leg_extensions,
    compute_condition_number,
)


# Machine-readable feasibility codes. The full set is declared for downstream
# consumers to pin against a stable enum.
OK = 'OK'
UNREACHABLE = 'UNREACHABLE'
WORKSPACE = 'WORKSPACE'
LIMIT_VEL = 'LIMIT_VEL'
LIMIT_ACC = 'LIMIT_ACC'
LIMIT_JERK = 'LIMIT_JERK'
TOO_FAST = 'TOO_FAST'
STEP_BOUND = 'STEP_BOUND'
STALE_STATE = 'STALE_STATE'
WRONG_MODE = 'WRONG_MODE'

# The per-knot |Δu0| bound is enforced with a 20 % safety margin below the pump's
# hard step gate, so the gate rejects a step-heavy move BEFORE it reaches the pump
# (which would otherwise reject it mid-stream, one tick after motion started).
STEP_BOUND_MARGIN = 0.80

# A jerk finite difference needs at least this many samples per segment.
# NB (jerk endpoint bias): the forward-difference jerk is centred *between* samples,
# so it under-measures the true jerk peak at a segment endpoint. At the default 80
# samples/segment (below) this endpoint-bias COMPONENT is up to ~2.3 % (measured
# 2026-07-16); the TOTAL under-measurement vs the ∞-sample truth is ≤3.7 % —
# _VALIDATE_JERK_MARGIN (1.05) covers the total so the gate stays strictly
# conservative on jerk (the binding constraint).
_MIN_SAMPLES = 4

# Default dense-sampling resolution for the analytic gate on an UNSHAPED plan.
# Dropped 200 → 80 as the planning speedup (2026-07-16): the analytic vel/acc peaks
# are closed-form-exact at any sample count (compute_jacobian evaluates the true
# analytic leg vel/acc per sample), and the step-bound pass (pass 3) samples at the
# fixed knot spacing — neither depends on this number. ONLY the jerk finite
# difference does. For an unshaped rest-to-rest quintic the leg jerk is BOUNDED
# (j(0)=j(T)=60·d/T³ is finite), so the FD jerk peak CONVERGES: at 80 samples it
# under-measures the ∞-sample truth by ≤3.7 % (measured 2026-07-16), well inside the
# _VALIDATE_JERK_MARGIN below. Fewer samples ⇒ proportionally fewer compute_jacobian
# calls ⇒ the dominant validate() cost drops ~2.5× (unshaped build_move ~730→~100 ms
# on the Jetson; the rest of the win is the component-form cross in
# ik_solver.compute_jacobian).
_VALIDATE_SAMPLES = 80

# Dense-sampling resolution for a SHAPED plan — an ACCURACY mesh, not a speed floor.
# WHY shaped is different (and why the number is now 1600, not the unshaped 80 or the
# historical 200):
#
#   The binding shaped quantity is FD leg jerk, and it UNDER-CONVERGES with sample
#   count. A LeanShaper superposes a windowed tilt; the C2 lean plateau window
#   (shaping.py LEAN_WINDOW_EDGE_FRAC) concentrates high w″ curvature into the 15 %
#   edge ramps, and the finite-difference jerk peak resolves that curvature only as
#   the mesh gets dense. Coarse meshes measure a SMALLER jerk ⇒ the duration-stretch
#   loop stretches shaped moves LESS ⇒ the plan silently emits more true leg jerk than
#   the gate believes, on the exact (lean) path the operator reported as "sharp". The
#   mesh is therefore chosen for FIDELITY, not cost.
#
#   Two eras of the floor:
#   * 200 (pre-2026-07-17) was a *speed-limited* floor — the per-sample Python loop
#     (compute_jacobian + accel_to_leg_accels + condition SVD ×N) cost ~185 ms/pass on
#     the Jetson, so 200 was as dense as was affordable, and it still under-measured
#     the 6400-sample reference jerk by ~21-23 % on x+150.
#   * 1600 (2026-07-17, this constant) became affordable when Phase 1a vectorised the
#     shaped branch (_validate_shaped_batched): the whole grid runs in numpy at
#     ~40 ms/pass @1600 (vs ~7.5 ms @200 — the mesh, not the loop, is now the cost).
#     At 1600 the shaped FD jerk peak is within ~3 % of the 6400-sample reference
#     (measured -3.2 %; convergence: 400→-15 %, 800→-7 %, 1600→-3 %, 3200→-1 %), i.e.
#     the old 23 % under-measurement is cut ~7×. The residual ~3 % is the un-resolved
#     window-edge w″ curvature; _VALIDATE_JERK_MARGIN (1.05) covers it with room, so
#     the gate stays strictly conservative on jerk.
#
#   Clamp semantics (see validate): an EXPLICIT denser request (e.g. a 6400-sample
#   convergence reference in a test) is honoured; a COARSER request is clamped UP to
#   this floor, so a shaped plan is never gated below the 1600-sample fidelity.
#
#   Intended behavioural consequence of the 200→1600 bump: jerk-bound shaped moves
#   plan ~5-8 % LONGER (T ∝ jerk^(1/3) on the ~25 % more jerk the gate now sizes) —
#   the FIX for the silent under-stretch, not a regression. See logbook
#   2026-07-16-lean-planning-latency-and-boundary-step.md and the
#   shaped-planning-efficiency plan (Phase 1b).
_SHAPED_VALIDATE_SAMPLES = 1600

# Jerk inflation applied to the finite-difference peak before the limit comparison,
# mirroring _FOLLOW_JERK_MARGIN on the fast follower gate. At 80 samples/segment the
# UNSHAPED FD jerk peak under-measures the true peak by ≤3.7 % (measured 2026-07-16);
# a 5 % inflation more than covers that, so the analytic gate is provably conservative
# on unshaped jerk — over-conservatism only ever REJECTS a marginal plan (the planner
# then stretches its duration), never accepts an over-jerk one. (On a shaped plan the
# margin is applied too, on top of the dense ≥1600-sample accuracy floor above: the
# floor pulls the measured FD jerk to within ~3 % of truth, and this 5 % margin then
# covers that residual — the two compose to keep shaped jerk conservative.) Vel/acc/
# step comparisons are analytic/exact and get NO margin.
_VALIDATE_JERK_MARGIN = 1.05


@dataclass
class FeasibilityReport:
    """Outcome of a :func:`validate` pass over a plan.

    Carries **all** measured leg-space peaks (not just the failing one) so the
    planner's duration-stretch loop can compute the worst limit ratio from a
    single report and converge in one step.
    """
    ok: bool
    code: str
    reasons: list = field(default_factory=list)
    min_duration_s: float = 0.0
    # Measured peaks over the sampled path.
    peak_leg_vel_mmps: float = 0.0
    peak_leg_acc_mmps2: float = 0.0
    peak_leg_jerk_mmps3: float = 0.0
    peak_leg_ext_mm: float = 0.0
    peak_step_rev: float = 0.0


class TrajectoryInfeasible(Exception):
    """Raised by ``planner.py`` constructors when a plan fails :func:`validate`.

    Carries the machine ``code``, human ``reasons``, and (when the failure is a
    timing/limit one that a longer duration would fix) ``min_duration_s`` so the
    caller can surface an achievable duration to the operator — the plan's
    "loudly rejected, never silently dropped" requirement.
    """

    def __init__(self, code: str, reasons, min_duration_s: float = 0.0):
        self.code = code
        self.reasons = list(reasons)
        self.min_duration_s = float(min_duration_s)
        super().__init__(f"{code}: {'; '.join(self.reasons)}")


# Per-geometry cache of the workspace limits. ``WorkspaceLimits.from_geometry``
# runs an SVD (compute_condition_number at the reference pose), which is pure
# overhead to repeat on every validate() call — the geometry is fixed for a node's
# lifetime. Keyed by id(geom) with a strong ref to geom held alongside so the id
# cannot be reused while the entry is live (guards the id-recycling edge). This is
# the perf hoist called out in the Phase-2 audit; the SpaceMouse follower (Phase 3)
# will lean on validate() being cheap.
_WLIMITS_CACHE: dict = {}


def _workspace_limits(geom):
    key = id(geom)
    cached = _WLIMITS_CACHE.get(key)
    if cached is None or cached[0] is not geom:
        wl = WorkspaceLimits.from_geometry(geom)
        _WLIMITS_CACHE[key] = (geom, wl)
        return wl
    return cached[1]


def _pose_to_pos_rot(pose: np.ndarray):
    pos = np.asarray(pose[:3], dtype=float)
    rot = rotvec_to_rot_matrix(np.asarray(pose[3:6], dtype=float))
    return pos, rot


def _segment_grids(plan, samples_per_segment: int):
    """Per-segment uniform time grids ``(times, dt)`` for jerk finite differences.

    A uniform grid *within* each segment is what lets the jerk finite difference
    use a single ``dt``; jerk is only C0 across segment joins (the plan is C2, not
    C3), so a per-segment grid is correct — a cross-join difference would fabricate
    a spurious jerk spike at the seam.
    """
    grids = []
    if plan.segments:
        for start, seg in zip(plan._starts, plan.segments):
            n = max(_MIN_SAMPLES, int(samples_per_segment))
            times = start + seg.duration * np.arange(n) / (n - 1)
            grids.append((times, seg.duration / (n - 1)))
    else:
        # HoldPlan: a single fixed pose, no time axis.
        grids.append((np.array([0.0]), 0.0))
    return grids


def validate(plan, limits, geom, *, samples_per_segment: int = _VALIDATE_SAMPLES
             ) -> FeasibilityReport:
    """Validate ``plan`` against ``limits`` by dense sampling of the leg chain.

    Returns a :class:`FeasibilityReport`. On a geometry failure the report
    early-returns with the offending ``code`` and populated ``reasons``. Otherwise
    it always carries every measured peak, and ``code`` is the first limit/step
    check to fail (priority: vel → acc → jerk → step) or ``OK``.
    """
    wlimits = _workspace_limits(geom)
    mm_to_rev = np.asarray(geom.mm_to_rev, dtype=float)

    # Shaped plans' FD jerk peak under-converges with sample count (window-edge
    # curvature from the C2 lean window), so they are gated at the dense ACCURACY
    # floor _SHAPED_VALIDATE_SAMPLES (=1600, within ~3 % of the 6400-sample reference
    # jerk) rather than the leaner unshaped default — see that constant's rationale.
    # An explicit denser request (e.g. a 6400-sample convergence reference in a test)
    # is still honoured; a coarser request is clamped UP to the floor.
    if isinstance(plan, _ShapedPlan):
        samples_per_segment = max(int(samples_per_segment), _SHAPED_VALIDATE_SAMPLES)
        # Vectorised shaped gate (Phase 1a): the per-sample Python loop below
        # recomputes the analytic Jacobian chain + lean superposition at every one of
        # the ≥1600 shaped samples and dominated shaped build_move cost. The batched
        # twin measures the IDENTICAL leg-space quantities on the whole grid in numpy
        # (parity with this loop at equal mesh: ~1.4e-10 max rel err, from the shared
        # 1e-7 J-dot FD + batched Rodrigues — verified in tests/motion/
        # test_shaped_batch.py) — see _validate_shaped_batched. Only the
        # single-segment rest-to-rest build_move plan (every shaped plan produced
        # today) dispatches here; a multi-segment _ShapedPlan (not built today) falls
        # through to the scalar loop, which handles per-segment grids correctly.
        if len(plan.segments) == 1:
            return _validate_shaped_batched(
                plan, limits, geom, samples_per_segment, wlimits=wlimits,
                mm_to_rev=mm_to_rev)

    peak_vel = 0.0
    peak_acc = 0.0
    peak_jerk = 0.0
    peak_ext = 0.0

    # ── Passes 1 + 2: geometry + leg vel/acc/jerk, per segment ──
    for times, dt in _segment_grids(plan, samples_per_segment):
        acc_samples = []
        for t in times:
            pose, twist, accel = plan.state_at(float(t))
            # Reject non-finite input up front: a NaN/Inf pose sails through every
            # numeric comparison below (all comparisons against NaN are False), so
            # the gate must catch it explicitly. Reuse UNREACHABLE.
            if not np.all(np.isfinite(pose)):
                return FeasibilityReport(
                    ok=False, code=UNREACHABLE,
                    reasons=[f"pose contains non-finite values (NaN/Inf) "
                             f"at t={t:.3f}s"])
            pos, rot = _pose_to_pos_rot(pose)

            ext = pose_to_leg_lengths(pos, rot, geom)
            peak_ext = max(peak_ext, float(np.max(np.abs(ext))))
            valid, states = check_leg_extensions(ext, geom)
            if not valid:
                bad = [i for i, s in enumerate(states) if s != 0]
                return FeasibilityReport(
                    ok=False, code=WORKSPACE,
                    reasons=[f"leg {i} out of stroke ({ext[i]:.1f} mm) "
                             f"at t={t:.3f}s" for i in bad],
                    peak_leg_ext_mm=peak_ext)

            J = compute_jacobian(pos, rot, geom)
            cond = compute_condition_number(pos, rot, geom, J=J)
            if cond > wlimits.cond_hard:
                # Near-singular: unrealisable no matter the timing.
                return FeasibilityReport(
                    ok=False, code=UNREACHABLE,
                    reasons=[f"Jacobian condition {cond:.1f} > "
                             f"{wlimits.cond_hard:.1f} (near singularity) "
                             f"at t={t:.3f}s"],
                    peak_leg_ext_mm=peak_ext)

            leg_vel = twist_to_leg_velocities(twist, pos, rot, geom, J=J)
            peak_vel = max(peak_vel, float(np.max(np.abs(leg_vel))))
            leg_acc = accel_to_leg_accels(accel, twist, pos, rot, geom, J=J)
            peak_acc = max(peak_acc, float(np.max(np.abs(leg_acc))))
            acc_samples.append(leg_acc)

        # Jerk from the finite difference of the analytic leg acceleration. Inflate
        # by _VALIDATE_JERK_MARGIN so the gate is strictly conservative on jerk (the
        # FD peak under-measures by ≤3.7 % total at 80 samples) — same idiom as the
        # fast gate's _FOLLOW_JERK_MARGIN. The stored peak carries the inflation, so
        # the planner's duration-stretch loop also sizes off the conservative value.
        if dt > 0.0 and len(acc_samples) >= 2:
            arr = np.asarray(acc_samples)
            jerk = np.diff(arr, axis=0) / dt
            peak_jerk = max(peak_jerk,
                            float(np.max(np.abs(jerk))) * _VALIDATE_JERK_MARGIN)

    # ── Pass 3: knot-step bound (the actual wire u0 sequence) ──
    peak_step = 0.0
    if plan.total_duration > 0.0:
        kdt = float(limits.knot_dt_s)
        n_knots = int(np.floor(plan.total_duration / kdt)) + 2
        prev_rev = None
        for k in range(n_knots):
            tk = min(k * kdt, plan.total_duration)
            pose_k, _, _ = plan.state_at(tk)
            pos_k, rot_k = _pose_to_pos_rot(pose_k)
            motor_rev = pose_to_leg_lengths(pos_k, rot_k, geom) * mm_to_rev
            if prev_rev is not None:
                peak_step = max(
                    peak_step, float(np.max(np.abs(motor_rev - prev_rev))))
            prev_rev = motor_rev
            if tk >= plan.total_duration:
                break

    # ── Decide the code (priority order; first failure wins) ──
    step_bound = STEP_BOUND_MARGIN * float(limits.max_step_rev)
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

    return FeasibilityReport(
        ok=(code == OK), code=code, reasons=reasons,
        peak_leg_vel_mmps=peak_vel, peak_leg_acc_mmps2=peak_acc,
        peak_leg_jerk_mmps3=peak_jerk, peak_leg_ext_mm=peak_ext,
        peak_step_rev=peak_step)


# ═══════════════════════════════════════════════════════════════════════════
# Fast follower gate (Phase 3) — the SAME checks as validate(), vectorised
# ═══════════════════════════════════════════════════════════════════════════
#
# WHY a second gate function (and why it is still "one gate"):
# The SpaceMouse follower (follower.py) replans every 40 Hz emitter tick, so it
# runs inside the 25 ms emit budget. The analytic validate() above costs ~377 ms
# on this Jetson (measured 2026-07-07) — three orders of magnitude too slow — and
# it is the analytic Jacobian chain (compute_jacobian + accel_to_leg_accels'
# J̇ finite difference) that dominates, NOT the condition-number SVD (that is ~9 %).
# validate_follow() measures the IDENTICAL leg-space quantities a different, cheaper
# way: it vectorises the pose→leg-extension sampling over the whole segment in one
# numpy expression (no Python per-sample loop, no analytic Jacobian) and derives
# leg vel/acc/jerk by FINITE-DIFFERENCING the sampled extensions — the exact method
# validate()'s step-bound pass already trusts. Result: ~1.5 ms.
#
# It is NOT a bypass of the loud-rejection contract:
#   * it lives in this one feasibility module, returns the same FeasibilityReport,
#     and uses the same codes / limits / stroke logic;
#   * the follower reaches it only through planner.build_follow (same as every
#     other motion path goes through planner);
#   * its step-bound check is BIT-IDENTICAL to validate() (same knot sampling),
#     so the "every emitted frame is pump-accepted" invariant is preserved exactly;
#   * vel/acc finite differences match the analytic peaks to < 0.05 % (verified);
#   * the jerk (third difference) under-measures the true peak by ≤ 1.5 % at 300
#     samples — the SAME endpoint bias validate() itself already accepts (see the
#     _MIN_SAMPLES note) — and validate_follow inflates it by _FOLLOW_JERK_MARGIN so
#     the follower gate is *strictly conservative* on jerk (never anti-conservative
#     vs the session limit).
#
# Decimation the task authorised: the per-sample condition-number SVD is checked at
# only _FOLLOW_COND_SAMPLES points (defence-in-depth — the per-sample STROKE check
# runs on ALL samples and, in this geometry, always fires before ill-conditioning;
# see the validate() condition-check Discussion in the Phase-2 logbook).

# Follower-gate resolution. 300 samples/segment lands the finite-difference jerk
# bias at ≤ 1.5 % (== validate()'s accepted analytic endpoint bias) while keeping
# the whole gate at ~1.6 ms. The condition SVD is decimated to 12 points.
_FOLLOW_SAMPLES = 300
_FOLLOW_COND_SAMPLES = 12
# Extra jerk inflation on top of the ≤1.5 % finite-difference under-measurement, so
# the follower gate is provably conservative on the binding constraint. Over-
# conservatism only ever REJECTS a marginal plan (follower keeps its last valid
# plan) — never accepts an over-jerk one.
_FOLLOW_JERK_MARGIN = 1.05


def _batched_rotvec_to_R(rv: np.ndarray) -> np.ndarray:
    """(N,3) rotvecs → (N,3,3) rotation matrices — batched Rodrigues.

    Bit-matches the scalar ``ik_solver.rotvec_to_rot_matrix`` (verified to 1e-9);
    the small-angle rows fall back to identity to avoid the 0/0 in the axis
    normalisation.
    """
    theta = np.linalg.norm(rv, axis=1)                       # (N,)
    small = theta < 1e-12
    theta_safe = np.where(small, 1.0, theta)
    k = rv / theta_safe[:, None]                             # (N,3) unit axes
    kx, ky, kz = k[:, 0], k[:, 1], k[:, 2]
    n = rv.shape[0]
    K = np.zeros((n, 3, 3))
    K[:, 0, 1] = -kz; K[:, 0, 2] = ky
    K[:, 1, 0] = kz;  K[:, 1, 2] = -kx
    K[:, 2, 0] = -ky; K[:, 2, 1] = kx
    s = np.sin(theta)[:, None, None]
    c = (1.0 - np.cos(theta))[:, None, None]
    R = np.eye(3)[None] + s * K + c * (K @ K)
    R[small] = np.eye(3)
    return R


def _batched_leg_vectors(poses: np.ndarray, geom):
    """(N,6) poses → ``(leg_vecs (N,6,3), R (N,3,3))``.

    Vectorised twin of ``ik_solver.compute_leg_vectors``: matches it per-sample to
    machine precision (verified). ``R`` is returned so the decimated condition
    check can reuse it without a second Rodrigues.
    """
    pos = poses[:, :3]                                       # (N,3)
    R = _batched_rotvec_to_R(poses[:, 3:6])                  # (N,3,3)
    centre = pos + geom.init_height_vec                      # (N,3)
    plat_world = centre[:, None, :] + np.einsum(
        'nij,kj->nki', R, geom.plat_nodes)                   # (N,6,3)
    return plat_world - geom.base_nodes[None], R


def _batched_condition(leg_vecs: np.ndarray, R: np.ndarray, geom) -> np.ndarray:
    """Normalised Jacobian condition number per sample (batched).

    Replicates ``workspace.compute_condition_number`` EXACTLY (verified): the
    rotational columns are divided by ``plat_radius_mm`` before the SVD, so the
    result is the ~3–8 normalised condition (not the raw ~450) that
    ``WorkspaceLimits.cond_hard`` is expressed against.
    """
    unit = leg_vecs / np.linalg.norm(leg_vecs, axis=2, keepdims=True)   # (N,6,3)
    a_world = np.einsum('nij,kj->nki', R, geom.plat_nodes)              # (N,6,3)
    n = leg_vecs.shape[0]
    J = np.empty((n, 6, 6))
    J[:, :, :3] = unit
    J[:, :, 3:] = np.cross(a_world, unit)
    J[:, :, 3:] /= geom.plat_radius_mm
    return np.linalg.cond(J)                                             # (N,)


# ═══════════════════════════════════════════════════════════════════════════
# Batched analytic Jacobian chain — the shaped gate's vectorised J / J̇ (Phase 1a)
# ═══════════════════════════════════════════════════════════════════════════
#
# These vectorise the SAME analytic Jacobian chain validate() drives per-sample via
# ik_solver.compute_jacobian / compute_jacobian_dot — NOT the finite-difference
# reconstruction validate_follow uses. The distinction is load-bearing: a shaped
# plan carries the lean superposition in its twist/accel, so its leg vel/acc MUST be
# formed as J·twist / (J·accel + J̇·twist) on the shaped state (validate_follow's FD
# of the sampled *base* extensions would miss the lean entirely — the reason its
# _ShapedPlan TypeError guard exists). J̇ uses the SAME central FD (dt=1e-7) as
# ik_solver.compute_jacobian_dot, so the batched leg acc matches the scalar gate to
# ~1e-10 (verified). J is built in the component-cross form ik_solver.compute_jacobian
# ships (arithmetic-only, no np.cross dispatch), batched over samples.


def _batched_jacobian_from_posR(pos: np.ndarray, R: np.ndarray, geom) -> np.ndarray:
    """(N,3) positions + (N,3,3) rotations → analytic Jacobian ``J (N,6,6)``.

    Batched twin of ``ik_solver.compute_jacobian`` that takes a rotation MATRIX
    directly (not a rotvec), so ``_batched_jacobian_dot`` can build the fwd/bwd
    Jacobians from the FD-advanced ``(pos ± v·dt, dR·R)`` states exactly as the
    scalar ``compute_jacobian_dot`` does (which advances the rotation matrix, never
    round-tripping through a rotvec).
    """
    centre = pos + geom.init_height_vec                                    # (N,3)
    plat_world = centre[:, None, :] + np.einsum(
        'nij,kj->nki', R, geom.plat_nodes)                                 # (N,6,3)
    leg_vecs = plat_world - geom.base_nodes[None]
    l = leg_vecs / np.linalg.norm(leg_vecs, axis=2, keepdims=True)         # (N,6,3)
    a_world = np.einsum('nij,kj->nki', R, geom.plat_nodes)                 # (N,6,3)
    n = pos.shape[0]
    J = np.empty((n, 6, 6))
    J[:, :, :3] = l
    ax, ay, az = a_world[..., 0], a_world[..., 1], a_world[..., 2]
    lx, ly, lz = l[..., 0], l[..., 1], l[..., 2]
    J[:, :, 3] = ay * lz - az * ly
    J[:, :, 4] = az * lx - ax * lz
    J[:, :, 5] = ax * ly - ay * lx
    return J


def _batched_jacobian(poses: np.ndarray, geom):
    """(N,6) poses → ``(J (N,6,6), leg_vecs (N,6,3), R (N,3,3))``.

    Batched ``ik_solver.compute_jacobian`` reusing ``_batched_leg_vectors`` (so the
    leg vectors and ``R`` are shared with the stroke/condition checks). The rotational
    columns are the row-wise cross ``a_world × l`` in the same component form the
    scalar Jacobian ships.
    """
    leg_vecs, R = _batched_leg_vectors(poses, geom)                        # (N,6,3),(N,3,3)
    l = leg_vecs / np.linalg.norm(leg_vecs, axis=2, keepdims=True)
    a_world = np.einsum('nij,kj->nki', R, geom.plat_nodes)                 # (N,6,3)
    n = poses.shape[0]
    J = np.empty((n, 6, 6))
    J[:, :, :3] = l
    ax, ay, az = a_world[..., 0], a_world[..., 1], a_world[..., 2]
    lx, ly, lz = l[..., 0], l[..., 1], l[..., 2]
    J[:, :, 3] = ay * lz - az * ly
    J[:, :, 4] = az * lx - ax * lz
    J[:, :, 5] = ax * ly - ay * lx
    return J, leg_vecs, R


def _batched_jacobian_dot(poses: np.ndarray, twists: np.ndarray, geom,
                          dt: float = 1e-7) -> np.ndarray:
    """Central-FD Jacobian time-derivative per sample → ``J̇ (N,6,6)``.

    Identical construction to ``ik_solver.compute_jacobian_dot``: advance the pose by
    ``±twist·dt`` (translation linearly, rotation by ``rotvec_to_rot_matrix(ω·dt)·R``)
    and central-difference the analytic Jacobian, with the SAME ``dt=1e-7`` half-step,
    so the batched leg acceleration ``J·accel + J̇·twist`` matches the scalar gate.
    """
    pos = poses[:, :3]
    v = twists[:, :3]
    omega = twists[:, 3:]
    R = _batched_rotvec_to_R(poses[:, 3:6])                                # (N,3,3)
    R_fwd = np.einsum('nij,njk->nik', _batched_rotvec_to_R(omega * dt), R)
    R_bwd = np.einsum('nij,njk->nik', _batched_rotvec_to_R(-omega * dt), R)
    J_fwd = _batched_jacobian_from_posR(pos + v * dt, R_fwd, geom)
    J_bwd = _batched_jacobian_from_posR(pos - v * dt, R_bwd, geom)
    return (J_fwd - J_bwd) / (2.0 * dt)


def _validate_shaped_batched(plan, limits, geom, samples_per_segment, *,
                             wlimits, mm_to_rev) -> FeasibilityReport:
    """Vectorised twin of :func:`validate`'s per-sample loop for a single-segment
    ``_ShapedPlan``.

    Measures the IDENTICAL leg-space quantities the scalar loop does — geometry
    (finite / stroke / condition), analytic leg vel/acc (``J·twist`` /
    ``J·accel + J̇·twist``), FD jerk with :data:`_VALIDATE_JERK_MARGIN`, and the
    plan-wide knot-step bound — but batched over the whole ≥1600-sample grid in numpy.
    Reproduces the scalar contract exactly: the same ``FeasibilityReport`` fields,
    the same first-failure-wins ordering (geometry rejects in time order, with
    finite→stroke→condition priority within a sample; then vel→acc→jerk→step), and
    the same populated peaks on early geometry returns (only ``peak_leg_ext_mm``).

    The full condition SVD runs on all N samples (not decimated) for strict parity
    with the scalar gate. See the module block above :func:`_batched_jacobian_from_posR`
    for why leg vel/acc use the analytic chain rather than the follower gate's FD.
    """
    seg = plan.segments[0]
    n = max(_MIN_SAMPLES, int(samples_per_segment))
    T = seg.duration
    dt = T / (n - 1)
    ts = T * np.arange(n) / (n - 1)                                        # (n,)

    poses, twists, accels = batched_shaped_states(plan, ts)

    # ── Passes 1: geometry (finite / stroke / condition), first-failure-wins ──
    # A non-finite pose makes leg_vecs / condition NaN; substitute a benign pose on
    # those rows so the batched geometry is numerically well-defined — the finite
    # check has strictly higher priority, so those rows never read their stroke/cond.
    finite_rows = np.all(np.isfinite(poses), axis=1)                       # (n,)
    safe_poses = poses if finite_rows.all() else np.where(
        finite_rows[:, None], poses, 0.0)

    J, leg_vecs, R = _batched_jacobian(safe_poses, geom)
    ext = np.linalg.norm(leg_vecs, axis=2) - geom.init_leg_lengths_mm      # (n,6)
    stroke_bad = np.any((ext < wlimits.leg_hard_min_mm)
                        | (ext > wlimits.leg_hard_max_mm), axis=1)         # (n,)
    conds = _batched_condition(leg_vecs, R, geom)                          # (n,)
    cond_bad = conds > wlimits.cond_hard

    geom_fail = (~finite_rows) | stroke_bad | cond_bad
    if geom_fail.any():
        first = int(np.argmax(geom_fail))
        t_fail = float(ts[first])
        if not finite_rows[first]:
            return FeasibilityReport(
                ok=False, code=UNREACHABLE,
                reasons=[f"pose contains non-finite values (NaN/Inf) "
                         f"at t={t_fail:.3f}s"])
        # peak_ext accumulates over samples 0..first (inclusive), matching the scalar
        # loop's max at the point it early-returns.
        peak_ext = float(np.max(np.abs(ext[:first + 1])))
        if stroke_bad[first]:
            bad = [i for i in range(6)
                   if ext[first, i] < wlimits.leg_hard_min_mm
                   or ext[first, i] > wlimits.leg_hard_max_mm]
            return FeasibilityReport(
                ok=False, code=WORKSPACE,
                reasons=[f"leg {i} out of stroke ({ext[first, i]:.1f} mm) "
                         f"at t={t_fail:.3f}s" for i in bad],
                peak_leg_ext_mm=peak_ext)
        return FeasibilityReport(
            ok=False, code=UNREACHABLE,
            reasons=[f"Jacobian condition {conds[first]:.1f} > "
                     f"{wlimits.cond_hard:.1f} (near singularity) "
                     f"at t={t_fail:.3f}s"],
            peak_leg_ext_mm=peak_ext)

    peak_ext = float(np.max(np.abs(ext)))

    # ── Pass 2: analytic leg vel/acc + FD jerk (all peaks measured) ──
    leg_vel = np.einsum('nij,nj->ni', J, twists)                          # (n,6)
    J_dot = _batched_jacobian_dot(poses, twists, geom)
    leg_acc = (np.einsum('nij,nj->ni', J, accels)
               + np.einsum('nij,nj->ni', J_dot, twists))                  # (n,6)
    peak_vel = float(np.max(np.abs(leg_vel)))
    peak_acc = float(np.max(np.abs(leg_acc)))
    peak_jerk = 0.0
    if dt > 0.0 and n >= 2:
        jerk = np.diff(leg_acc, axis=0) / dt
        peak_jerk = float(np.max(np.abs(jerk))) * _VALIDATE_JERK_MARGIN

    # ── Pass 3: knot-step bound (the actual wire u0 sequence) ──
    peak_step = 0.0
    if plan.total_duration > 0.0:
        kdt = float(limits.knot_dt_s)
        n_knots = int(np.floor(plan.total_duration / kdt)) + 2
        kts = np.minimum(np.arange(n_knots) * kdt, plan.total_duration)
        kposes, _, _ = batched_shaped_states(plan, kts)
        kvecs, _ = _batched_leg_vectors(kposes, geom)
        rev = (np.linalg.norm(kvecs, axis=2)
               - geom.init_leg_lengths_mm) * mm_to_rev                    # (n_knots,6)
        if n_knots > 1:
            peak_step = float(np.max(np.abs(np.diff(rev, axis=0))))

    # ── Decide the code (priority order; first failure wins) ──
    step_bound = STEP_BOUND_MARGIN * float(limits.max_step_rev)
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

    return FeasibilityReport(
        ok=(code == OK), code=code, reasons=reasons,
        peak_leg_vel_mmps=peak_vel, peak_leg_acc_mmps2=peak_acc,
        peak_leg_jerk_mmps3=peak_jerk, peak_leg_ext_mm=peak_ext,
        peak_step_rev=peak_step)


def validate_follow(plan, limits, geom, *,
                    samples_per_segment: int = _FOLLOW_SAMPLES,
                    cond_samples: int = _FOLLOW_COND_SAMPLES
                    ) -> FeasibilityReport:
    """Vectorised finite-difference feasibility gate for the follower hot path.

    Same contract as :func:`validate` (a pure predicate returning a
    :class:`FeasibilityReport`, first-failure-wins ``code``, all peaks measured),
    but ~250× faster — see the module block above for why this is the same gate,
    not a bypass. Intended for per-tick replanning; the service path keeps using
    the analytic :func:`validate`.

    **Shaping-blind — shaped plans MUST NOT reach here.** The finite-difference
    method reconstructs the *base* quintic's leg peaks, not the analytic lean
    superposition a :class:`~jugglebot.motion.trajectory.shaping._ShapedPlan` carries
    (``eval_pose_batch`` samples the base segment, bypassing ``_ShapedPlan.state_at``),
    so a shaped plan would be gated on its UNSHAPED peaks — silently under-measuring
    the lean's leg motion. Lean shipped only on the service (``build_move``) path,
    which uses the analytic :func:`validate`; the follower (``build_follow``) never
    shapes. A shaped plan arriving here is a programming error — reject it LOUDLY
    (an explicit ``TypeError``, not a bare ``assert`` which ``python -O`` strips).
    """
    if isinstance(plan, _ShapedPlan):
        raise TypeError(
            "validate_follow cannot gate a _ShapedPlan: its finite-difference peaks "
            "measure the base quintic, not the lean superposition. Shaped plans are "
            "gated exclusively by the analytic validate() on the service path.")
    wlimits = _workspace_limits(geom)
    mm_to_rev = np.asarray(geom.mm_to_rev, dtype=float)

    peak_vel = 0.0
    peak_acc = 0.0
    peak_jerk = 0.0
    peak_ext = 0.0

    for seg in (plan.segments or ()):
        n = max(_MIN_SAMPLES, int(samples_per_segment))
        T = seg.duration
        dt = T / (n - 1)
        ts = T * np.arange(n) / (n - 1)
        poses = seg.eval_pose_batch(ts)                     # (n,6)

        if not np.all(np.isfinite(poses)):
            return FeasibilityReport(
                ok=False, code=UNREACHABLE,
                reasons=["pose contains non-finite values (NaN/Inf) on the "
                         "sampled follower segment"])

        leg_vecs, R = _batched_leg_vectors(poses, geom)
        ext = np.linalg.norm(leg_vecs, axis=2) - geom.init_leg_lengths_mm  # (n,6)
        peak_ext = max(peak_ext, float(np.max(np.abs(ext))))

        # Stroke check on EVERY sample (batched, cheap) — the real per-sample
        # backstop the decimated condition check leans on.
        hard_min = wlimits.leg_hard_min_mm
        hard_max = wlimits.leg_hard_max_mm
        oob = (ext < hard_min) | (ext > hard_max)
        if np.any(oob):
            si, li = np.argwhere(oob)[0]
            return FeasibilityReport(
                ok=False, code=WORKSPACE,
                reasons=[f"leg {li} out of stroke ({ext[si, li]:.1f} mm) on the "
                         f"follower segment"],
                peak_leg_ext_mm=peak_ext)

        # Decimated condition-number SVD (defence-in-depth).
        idx = np.linspace(0, n - 1, min(cond_samples, n)).astype(int)
        conds = _batched_condition(leg_vecs[idx], R[idx], geom)
        worst = float(np.max(conds))
        if worst > wlimits.cond_hard:
            return FeasibilityReport(
                ok=False, code=UNREACHABLE,
                reasons=[f"Jacobian condition {worst:.1f} > "
                         f"{wlimits.cond_hard:.1f} (near singularity) on the "
                         f"follower segment"],
                peak_leg_ext_mm=peak_ext)

        # Leg vel/acc/jerk by finite differences of the sampled extensions.
        vel = np.diff(ext, axis=0) / dt
        acc = np.diff(vel, axis=0) / dt
        jerk = np.diff(acc, axis=0) / dt
        peak_vel = max(peak_vel, float(np.max(np.abs(vel))))
        peak_acc = max(peak_acc, float(np.max(np.abs(acc))))
        peak_jerk = max(peak_jerk,
                        float(np.max(np.abs(jerk))) * _FOLLOW_JERK_MARGIN)

    # Degenerate (HoldPlan / zero-segment) branch: the per-segment loop above ran
    # no geometry checks, so mirror validate()'s degenerate branch and gate the
    # single held pose (finiteness + stroke + decimated condition). Without this a
    # zero-segment plan — e.g. an out-of-stroke build_graceful_stop at-rest HoldPlan
    # — would pass validate_follow with ZERO checks (the finding this closes).
    if not plan.segments:
        hold_pose = np.asarray(plan.state_at(0.0)[0], dtype=float)[None, :]  # (1,6)
        if not np.all(np.isfinite(hold_pose)):
            return FeasibilityReport(
                ok=False, code=UNREACHABLE,
                reasons=["pose contains non-finite values (NaN/Inf) on the held "
                         "follower pose"])
        leg_vecs, R = _batched_leg_vectors(hold_pose, geom)
        ext = np.linalg.norm(leg_vecs, axis=2) - geom.init_leg_lengths_mm  # (1,6)
        peak_ext = float(np.max(np.abs(ext)))
        oob = (ext < wlimits.leg_hard_min_mm) | (ext > wlimits.leg_hard_max_mm)
        if np.any(oob):
            li = int(np.argwhere(oob)[0][1])
            return FeasibilityReport(
                ok=False, code=WORKSPACE,
                reasons=[f"leg {li} out of stroke ({ext[0, li]:.1f} mm) on the "
                         f"held follower pose"],
                peak_leg_ext_mm=peak_ext)
        worst = float(np.max(_batched_condition(leg_vecs, R, geom)))
        if worst > wlimits.cond_hard:
            return FeasibilityReport(
                ok=False, code=UNREACHABLE,
                reasons=[f"Jacobian condition {worst:.1f} > "
                         f"{wlimits.cond_hard:.1f} (near singularity) on the held "
                         f"follower pose"],
                peak_leg_ext_mm=peak_ext)

    # Knot-step bound — sampled at the wire knot spacing, identical to validate().
    peak_step = 0.0
    if plan.total_duration > 0.0:
        kdt = float(limits.knot_dt_s)
        n_knots = int(np.floor(plan.total_duration / kdt)) + 2
        kts = np.minimum(np.arange(n_knots) * kdt, plan.total_duration)
        rev = np.stack([_pose_to_rev(plan.state_at(float(tk))[0], geom, mm_to_rev)
                        for tk in kts])
        if n_knots > 1:
            peak_step = float(np.max(np.abs(np.diff(rev, axis=0))))

    step_bound = STEP_BOUND_MARGIN * float(limits.max_step_rev)
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

    return FeasibilityReport(
        ok=(code == OK), code=code, reasons=reasons,
        peak_leg_vel_mmps=peak_vel, peak_leg_acc_mmps2=peak_acc,
        peak_leg_jerk_mmps3=peak_jerk, peak_leg_ext_mm=peak_ext,
        peak_step_rev=peak_step)


def _pose_to_rev(pose, geom, mm_to_rev) -> np.ndarray:
    """pose_6dof → motor_rev, the exact chain the emitter/pump use (scalar)."""
    pos, rot = _pose_to_pos_rot(pose)
    return pose_to_leg_lengths(pos, rot, geom) * mm_to_rev
