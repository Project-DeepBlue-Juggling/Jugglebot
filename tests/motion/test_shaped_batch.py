"""Batched shaped-gate parity tests (Phase 1a of shaped-planning-efficiency).

Phase 1a vectorised the SHAPED branch of ``feasibility.validate`` at the then-current
``_SHAPED_VALIDATE_SAMPLES`` (=200) mesh — a parity refactor with ZERO behaviour
change; Phase 1b then raised the mesh to 1600 (the accuracy fix — see section (c)).
Two Phase-1a things are pinned here:

  (a) **Sampler parity** — ``shaping.batched_shaped_states`` reproduces the scalar
      ``_ShapedPlan.state_at`` (pose/twist/accel) to machine precision over a dense
      grid × moves × gains × durations. The scalar ``state_at`` is the frozen
      reference; the batched sampler is a faster evaluation of the identical law.

  (b) **Gate parity** — the batched shaped ``validate`` reproduces an in-test scalar
      reference (which samples ``state_at`` per point and runs the same per-sample
      ``compute_jacobian`` / ``accel_to_leg_accels`` / FD-jerk / knot-step math) at
      n=200: every ``FeasibilityReport`` field within 1e-9 rel and identical codes,
      including one deliberately-failing plan per code (WORKSPACE, LIMIT_VEL,
      LIMIT_ACC, LIMIT_JERK, STEP_BOUND). The scalar reference lives in this test —
      it is NOT resurrected in production code (production dispatches to the batched
      path for every single-segment shaped plan).

The gate-parity tests in (b) run both sides at an EXPLICIT equal mesh (via
``_batched_at``), so they pin the Phase-1a identity independently of the Phase-1b
floor; the floor/clamp behaviour itself is pinned by the accuracy-mesh floor test in
``test_trajectory_feasibility.py`` and the conservativeness corpus in section (c).

**Timing sanity (not asserted — timing tests flake):** on this Jetson the batched
shaped ``validate`` runs ~7.5 ms/pass at n=200 (measured 2026-07-17), vs ~185 ms/pass
for the pre-Phase-1a scalar per-sample loop — a ~25x per-pass speedup at bit-parity.
"""

from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory.limits import TrajectoryLimits
from jugglebot.motion.trajectory.plan import TrajectoryPlan
from jugglebot.motion.trajectory.segment import QuinticSegment
from jugglebot.motion.trajectory.shaping import LeanShaper, batched_shaped_states
from jugglebot.motion.ik_solver import (
    accel_to_leg_accels,
    compute_jacobian,
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
    twist_to_leg_velocities,
)
from jugglebot.motion.workspace import (
    WorkspaceLimits,
    check_leg_extensions,
    compute_condition_number,
)


G = float(hw.GRAVITY_MMPS2)
NEUTRAL = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
D2R = np.pi / 180.0


@pytest.fixture
def geom():
    return StewartGeometry()


def _limits(*, vel=1e9, acc=1e9, jerk=1e9, step=0.3, dt=0.025, min_dur=0.2):
    """A TrajectoryLimits with chosen ceilings (constructed directly to bypass the
    session-limit clamping, so vel/acc/jerk can be made effectively off)."""
    return TrajectoryLimits(
        leg_vel_mmps=vel, leg_acc_mmps2=acc, leg_jerk_mmps3=jerk,
        leg_vel_ceiling_mmps=1e12, leg_acc_ceiling_mmps2=1e12,
        leg_jerk_ceiling_mmps3=1e12,
        hand_vel_limit_rps=1e9, hand_acc_limit_rps2=1e9,
        hand_vel_ceiling_rps=1e12, hand_acc_ceiling_rps2=1e12,
        knot_dt_s=dt, max_step_rev=step,
        min_move_duration_s=min_dur, min_timed_lead_s=0.25, max_timed_lead_s=60.0)


def _shaped(target, gain, dur, seed=NEUTRAL):
    seg = QuinticSegment(np.asarray(seed, float), np.zeros(6), np.zeros(6),
                         np.asarray(target, float), np.zeros(6), np.zeros(6), dur)
    base = TrajectoryPlan((seg,), final_pose=np.asarray(target, float))
    return LeanShaper(gain, g_mm_s2=G).shape(base)


MOVES = {
    'x+150': NEUTRAL + np.array([150., 0, 0, 0, 0, 0.]),
    'y+150': NEUTRAL + np.array([0, 150., 0, 0, 0, 0.]),
    'z+50':  NEUTRAL + np.array([0, 0, 50., 0, 0, 0.]),
    'rx+8':  NEUTRAL + np.array([0, 0, 0, 8 * D2R, 0, 0.]),
    'mixed': NEUTRAL + np.array([60., -40, 30, 0, 6 * D2R, 0.]),
    'diag':  NEUTRAL + np.array([100., 100., -30, -4 * D2R, 3 * D2R, 0.]),
}
GAINS = (0.3, 0.6, 1.0)
DURS = (0.5, 0.7, 1.0)


# ══════════════════════════════════════════════════════════════════════════
# (a) Sampler parity: batched_shaped_states vs _ShapedPlan.state_at
# ══════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize('name', list(MOVES))
def test_batched_sampler_matches_state_at(name):
    """batched_shaped_states reproduces the scalar state_at (pose/twist/accel) to a
    1e-11 grade over a dense grid × gains × durations — machine precision, the same
    law evaluated faster (measured worst-case pose 7e-15, twist 6e-14, accel 5e-13).
    """
    tgt = MOVES[name]
    worst = {'pose': 0.0, 'twist': 0.0, 'accel': 0.0}
    for gain in GAINS:
        for dur in DURS:
            plan = _shaped(tgt, gain, dur)
            ts = np.linspace(0.0, dur, 501)
            bp, btw, bac = batched_shaped_states(plan, ts)
            for k, t in enumerate(ts):
                p, tw, ac = plan.state_at(float(t))
                worst['pose'] = max(worst['pose'], float(np.max(np.abs(bp[k] - p))))
                worst['twist'] = max(worst['twist'], float(np.max(np.abs(btw[k] - tw))))
                worst['accel'] = max(worst['accel'], float(np.max(np.abs(bac[k] - ac))))
    assert worst['pose'] < 1e-11, worst
    assert worst['twist'] < 1e-10, worst
    assert worst['accel'] < 1e-9, worst


def test_batched_sampler_endpoints_and_window_vanish(geom):
    """At both segment ends the windowed lean vanishes (w(0)=w(1)=0 with C2-zero
    derivatives), so the batched shaped pose/twist/accel collapse onto the base
    quintic's boundary state — pinned against state_at at the exact endpoints."""
    plan = _shaped(MOVES['diag'], 0.6, 0.8)
    ts = np.array([0.0, plan.total_duration])
    bp, btw, bac = batched_shaped_states(plan, ts)
    for k, t in enumerate(ts):
        p, tw, ac = plan.state_at(float(t))
        assert np.allclose(bp[k], p, atol=1e-11)
        assert np.allclose(btw[k], tw, atol=1e-10)
        assert np.allclose(bac[k], ac, atol=1e-9)


# ══════════════════════════════════════════════════════════════════════════
# (b) Gate parity: batched shaped validate vs an in-test scalar reference
# ══════════════════════════════════════════════════════════════════════════
#
# EQUAL-MESH note (Phase 1b, 2026-07-17): the parity property proved here is
# "batched shaped gate == scalar per-sample math AT THE SAME MESH n" — a mesh-
# independent identity (it held at 200 and 800 in Phase 1a). Since Phase 1b raised
# _SHAPED_VALIDATE_SAMPLES to 1600, the PUBLIC ``validate(..., samples_per_segment=200)``
# now clamps a shaped request UP to 1600, so it can no longer be used to pin the
# batched math at an explicit n=200 against a 200-sample scalar reference (they would
# be at different meshes). We therefore call the batched gate FUNCTION directly at the
# explicit mesh via ``_batched_at`` — the parity identity is orthogonal to the clamp,
# and the clamp itself is covered by the floor test in test_trajectory_feasibility.py
# and the dispatch test below. Running the scalar reference at 1600 instead would be
# faithful but ~1.5 s/call (a pure-Python loop ×8 the samples) — untenable for a corpus.


def _batched_at(plan, limits, geom, n):
    """Run the batched shaped gate at an EXPLICIT mesh ``n`` (bypassing validate()'s
    ≥_SHAPED_VALIDATE_SAMPLES clamp) so an equal-mesh parity comparison against the
    n-sample scalar reference is exact. Reproduces validate()'s pre-dispatch setup."""
    wlimits = feas._workspace_limits(geom)
    mm_to_rev = np.asarray(geom.mm_to_rev, dtype=float)
    return feas._validate_shaped_batched(
        plan, limits, geom, n, wlimits=wlimits, mm_to_rev=mm_to_rev)


def _scalar_reference(plan, limits, geom, n):
    """The pre-Phase-1a scalar validate() loop for a single-segment plan — samples
    state_at per point and runs the identical per-sample analytic math. Returns the
    same fields validate() would; NOT used in production (lives here as the frozen
    parity reference)."""
    wl = WorkspaceLimits.from_geometry(geom)
    mm_to_rev = np.asarray(geom.mm_to_rev, dtype=float)
    seg = plan.segments[0]
    T = seg.duration
    times = T * np.arange(n) / (n - 1)
    dt = T / (n - 1)
    peak_vel = peak_acc = peak_jerk = peak_ext = 0.0
    acc_samples = []
    for t in times:
        pose, twist, accel = plan.state_at(float(t))
        if not np.all(np.isfinite(pose)):
            return dict(ok=False, code=feas.UNREACHABLE, peak_leg_ext_mm=0.0,
                        peak_leg_vel_mmps=0.0, peak_leg_acc_mmps2=0.0,
                        peak_leg_jerk_mmps3=0.0, peak_step_rev=0.0)
        pos = np.asarray(pose[:3], float)
        rot = rotvec_to_rot_matrix(np.asarray(pose[3:6], float))
        ext = pose_to_leg_lengths(pos, rot, geom)
        peak_ext = max(peak_ext, float(np.max(np.abs(ext))))
        valid, _ = check_leg_extensions(ext, geom)
        if not valid:
            return dict(ok=False, code=feas.WORKSPACE, peak_leg_ext_mm=peak_ext,
                        peak_leg_vel_mmps=0.0, peak_leg_acc_mmps2=0.0,
                        peak_leg_jerk_mmps3=0.0, peak_step_rev=0.0)
        J = compute_jacobian(pos, rot, geom)
        cond = compute_condition_number(pos, rot, geom, J=J)
        if cond > wl.cond_hard:
            return dict(ok=False, code=feas.UNREACHABLE, peak_leg_ext_mm=peak_ext,
                        peak_leg_vel_mmps=0.0, peak_leg_acc_mmps2=0.0,
                        peak_leg_jerk_mmps3=0.0, peak_step_rev=0.0)
        peak_vel = max(peak_vel, float(np.max(np.abs(
            twist_to_leg_velocities(twist, pos, rot, geom, J=J)))))
        la = accel_to_leg_accels(accel, twist, pos, rot, geom, J=J)
        peak_acc = max(peak_acc, float(np.max(np.abs(la))))
        acc_samples.append(la)
    jerk = np.diff(np.asarray(acc_samples), axis=0) / dt
    peak_jerk = float(np.max(np.abs(jerk))) * feas._VALIDATE_JERK_MARGIN
    peak_step = 0.0
    kdt = float(limits.knot_dt_s)
    n_knots = int(np.floor(plan.total_duration / kdt)) + 2
    prev = None
    for k in range(n_knots):
        tk = min(k * kdt, plan.total_duration)
        pk, _, _ = plan.state_at(tk)
        pos = np.asarray(pk[:3], float)
        rot = rotvec_to_rot_matrix(np.asarray(pk[3:6], float))
        mr = pose_to_leg_lengths(pos, rot, geom) * mm_to_rev
        if prev is not None:
            peak_step = max(peak_step, float(np.max(np.abs(mr - prev))))
        prev = mr
        if tk >= plan.total_duration:
            break
    step_bound = feas.STEP_BOUND_MARGIN * float(limits.max_step_rev)
    code = feas.OK
    if peak_vel > limits.leg_vel_mmps:
        code = feas.LIMIT_VEL
    elif peak_acc > limits.leg_acc_mmps2:
        code = feas.LIMIT_ACC
    elif peak_jerk > limits.leg_jerk_mmps3:
        code = feas.LIMIT_JERK
    elif peak_step > step_bound:
        code = feas.STEP_BOUND
    return dict(ok=(code == feas.OK), code=code, peak_leg_ext_mm=peak_ext,
                peak_leg_vel_mmps=peak_vel, peak_leg_acc_mmps2=peak_acc,
                peak_leg_jerk_mmps3=peak_jerk, peak_step_rev=peak_step)


_FIELDS = ('peak_leg_vel_mmps', 'peak_leg_acc_mmps2', 'peak_leg_jerk_mmps3',
           'peak_leg_ext_mm', 'peak_step_rev')


def _assert_report_parity(batched, ref, ctx):
    assert batched.code == ref['code'], f"{ctx}: code {batched.code} != {ref['code']}"
    assert batched.ok == ref['ok'], ctx
    for f in _FIELDS:
        a = getattr(batched, f)
        b = ref[f]
        rel = abs(a - b) / max(abs(b), 1e-9)
        assert rel < 1e-9, f"{ctx}: {f} {a!r} vs {b!r} (rel {rel:.2e})"


@pytest.mark.parametrize('name', list(MOVES))
@pytest.mark.parametrize('gain', GAINS)
def test_gate_parity_ok_path(name, gain, geom):
    """OK-path gate parity at n=200: every FeasibilityReport field within 1e-9 rel of
    the scalar reference, over the move × gain corpus (limits effectively off)."""
    lims = _limits(step=1e9)  # every limit off → genuinely OK plans
    for dur in DURS:
        plan = _shaped(MOVES[name], gain, dur)
        rb = _batched_at(plan, lims, geom, 200)
        rs = _scalar_reference(plan, lims, geom, 200)
        assert rb.code == feas.OK
        _assert_report_parity(rb, rs, f"{name} g{gain} d{dur}")


def test_gate_parity_at_denser_mesh(geom):
    """Parity is not mesh-special: it holds at an explicit denser request (n=800) too
    — the batched and scalar gates agree at whatever mesh they are handed."""
    lims = _limits()
    for name in ('x+150', 'mixed', 'diag'):
        for gain in GAINS:
            plan = _shaped(MOVES[name], gain, 0.7)
            rb = _batched_at(plan, lims, geom, 800)
            rs = _scalar_reference(plan, lims, geom, 800)
            _assert_report_parity(rb, rs, f"{name} g{gain} n800")


# One deliberately-failing plan per code — batched must reproduce the scalar
# reference's code AND fields exactly.

def test_gate_parity_reject_workspace(geom):
    # A large-z shaped move drives the base pose out of stroke → WORKSPACE.
    plan = _shaped(NEUTRAL + np.array([0, 0, 400., 0, 0, 0.]), 0.6, 1.0)
    lims = _limits()
    rb = _batched_at(plan, lims, geom, 200)
    rs = _scalar_reference(plan, lims, geom, 200)
    assert rb.code == feas.WORKSPACE
    _assert_report_parity(rb, rs, 'reject-workspace')


def test_gate_parity_reject_nonfinite_unreachable(geom):
    """NaN-injection pins the batched gate's subtlest new logic: the safe-pose
    substitution + argmax first-failure priority for non-finite samples. A NaN
    target endpoint must return UNREACHABLE with peak_leg_ext_mm == 0.0 (the
    scalar contract: peaks are meaningless on a non-finite pose), identical to
    the scalar reference."""
    plan = _shaped(NEUTRAL + np.array([np.nan, 0, 0, 0, 0, 0.]), 0.6, 1.0)
    lims = _limits()
    rb = _batched_at(plan, lims, geom, 200)
    rs = _scalar_reference(plan, lims, geom, 200)
    assert rb.code == feas.UNREACHABLE
    assert rb.peak_leg_ext_mm == 0.0
    _assert_report_parity(rb, rs, 'reject-nonfinite')


def test_gate_parity_reject_limit_vel(geom):
    plan = _shaped(NEUTRAL + np.array([120., 0, 0, 0, 0, 0.]), 0.6, 0.35)
    lims = _limits(vel=200.0)
    rb = _batched_at(plan, lims, geom, 200)
    rs = _scalar_reference(plan, lims, geom, 200)
    assert rb.code == feas.LIMIT_VEL
    _assert_report_parity(rb, rs, 'reject-vel')


def test_gate_parity_reject_limit_acc(geom):
    plan = _shaped(NEUTRAL + np.array([120., 0, 0, 0, 0, 0.]), 0.6, 0.5)
    lims = _limits(acc=800.0)
    rb = _batched_at(plan, lims, geom, 200)
    rs = _scalar_reference(plan, lims, geom, 200)
    assert rb.code == feas.LIMIT_ACC
    _assert_report_parity(rb, rs, 'reject-acc')


def test_gate_parity_reject_limit_jerk(geom):
    plan = _shaped(NEUTRAL + np.array([120., 0, 0, 0, 0, 0.]), 0.6, 0.8)
    lims = _limits(jerk=8000.0)
    rb = _batched_at(plan, lims, geom, 200)
    rs = _scalar_reference(plan, lims, geom, 200)
    assert rb.code == feas.LIMIT_JERK
    _assert_report_parity(rb, rs, 'reject-jerk')


def test_gate_parity_reject_step_bound(geom):
    plan = _shaped(NEUTRAL + np.array([120., 0, 0, 0, 0, 0.]), 0.6, 0.4)
    lims = _limits(step=0.05)
    rb = _batched_at(plan, lims, geom, 200)
    rs = _scalar_reference(plan, lims, geom, 200)
    assert rb.code == feas.STEP_BOUND
    _assert_report_parity(rb, rs, 'reject-step')


def test_shaped_dispatch_is_batched(geom):
    """A single-segment shaped plan routes through the batched shaped gate, not the
    scalar per-sample loop — asserted by patching the scalar-loop-only symbol
    (accel_to_leg_accels) and confirming validate never calls it for a shaped plan."""
    import jugglebot.motion.trajectory.feasibility as fmod
    calls = {'n': 0}
    orig = fmod.accel_to_leg_accels

    def spy(*a, **k):
        calls['n'] += 1
        return orig(*a, **k)

    fmod.accel_to_leg_accels = spy
    try:
        plan = _shaped(MOVES['x+150'], 0.6, 0.7)
        feas.validate(plan, _limits(), geom, samples_per_segment=200)
    finally:
        fmod.accel_to_leg_accels = orig
    assert calls['n'] == 0, "shaped validate still used the scalar per-sample loop"


# ══════════════════════════════════════════════════════════════════════════
# (c) Conservativeness corpus (Phase 1b): the 200 → 1600 mesh cutover is safe
# ══════════════════════════════════════════════════════════════════════════
#
# Phase 1b bumped _SHAPED_VALIDATE_SAMPLES 200 → 1600. This is a deliberate ACCURACY
# change: the shaped FD leg-jerk peak under-converges with mesh, so 1600 measures
# ~25 % MORE jerk than the old 200 floor, and jerk-bound shaped moves plan ~5-8 %
# LONGER (the fix for the silent under-stretch). The corpus below is the cutover gate.
#
# TWO properties, VERIFIED against ground truth on this Jetson (2026-07-17), NOT
# assumed from the plan draft — a couple of the draft's phrasings did not survive:
#
#   1. **Cutover safety = @1600 never accepts what the SHIPPED @200 rejects.** The
#      load-bearing differential is 1600-vs-the-old-200, because 200 is what shipped.
#      FD jerk is systematically biased LOW at coarse mesh (a wider FD stencil averages
#      out the peak), so jerk@1600 ≥ jerk@200 EVERYWHERE (0 violations over 5 seeds ×
#      90 moves here; 0/400 in the plan's full offline corpus, 2026-07-17). Since jerk
#      is the binding shaped constraint, @1600-accept ⟹ @200-accept: the new gate is
#      strictly stricter than the one it replaces. THAT is what makes the cutover safe.
#
#   2. **@6400 is a CONVERGENCE reference, NOT an acceptance oracle.** @1600 still
#      under-measures the near-truth @6400 by ~3 %, so at the exact jerk boundary @1600
#      CAN accept a marginal plan that @6400 rejects (measured ~1/90 on some seeds) —
#      that residual ~3 % is precisely what _VALIDATE_JERK_MARGIN (1.05) covers. So this
#      corpus asserts CONVERGENCE (|jerk@1600 − jerk@6400|/jerk@6400 ≤ 5 %), never
#      "@1600 never accepts what @6400 rejects" (empirically false, and asserting it
#      would be a cherry-picked-seed flake).
#
# NOTE (do NOT assert): the plan draft's "vel/acc@1600 ≥ vel/acc@200" is FALSE — vel/acc
# are analytic point-samples of a continuous curve, so a coarse grid can by luck land a
# sample nearer a sharp peak than a specific fine grid (measured ~5-10/90 tiny reversals,
# sub-%). Only FD JERK is systematically monotone in mesh. This corpus asserts jerk
# monotonicity only.

# Realistic session limit tiers (the shipped working point + a tighter and a looser
# neighbour) so acceptance is a genuine, non-vacuous verdict.
_CORPUS_TIERS = (
    _limits(vel=1000.0, acc=5000.0, jerk=30000.0),   # shipped working point (2026-07-17)
    _limits(vel=400.0, acc=2000.0, jerk=20000.0),    # tighter
    _limits(vel=1500.0, acc=6000.0, jerk=40000.0),   # looser
)


def test_shaped_1600_conservativeness_corpus(geom):
    """Seeded random shaped-move corpus (moves × gains × durations × realistic limit
    tiers) proving the 200 → 1600 mesh cutover is safe on this Jetson (2026-07-17).

    Asserts, over N=90 deterministic random moves:
      * jerk@1600 ≥ jerk@200 everywhere (strictly-more-conservative on the binding
        constraint — 0 violations; the plan's full offline N=400 corpus also 0);
      * @1600-accept ⟹ @200-accept everywhere (the new gate never accepts a plan the
        shipped 200-sample gate rejected — 0 violations; the actual cutover-safety
        property, holding across seeds {1,7,42,20260717,99});
      * jerk@1600 within 5 % of the near-truth 6400-sample reference (convergence;
        worst 3.36 % at this seed, ≤3.70 % across the 5 seeds probed).

    See the section header above for why @6400 is a convergence reference and NOT an
    acceptance oracle, and why vel/acc monotonicity is NOT asserted.
    """
    rng = np.random.default_rng(20260717)
    N = 90
    D2R_ = np.pi / 180.0
    jerk_reversals = 0        # jerk@1600 < jerk@200 (must be 0)
    accept_reversals = 0      # @1600 OK but @200 not (must be 0)
    worst_conv = 0.0          # worst |jerk1600 − jerk6400| / jerk6400
    checked_peaks = 0         # samples where both meshes passed geometry (peaks live)
    both_verdicts = {'ok': 0, 'reject': 0}
    for _ in range(N):
        tgt = NEUTRAL + np.array([
            rng.uniform(-140, 140), rng.uniform(-140, 140), rng.uniform(-40, 50),
            rng.uniform(-8, 8) * D2R_, rng.uniform(-8, 8) * D2R_, 0.0])
        gain = float(rng.choice([0.3, 0.6, 1.0]))
        dur = float(rng.uniform(0.3, 1.6))
        lims = _CORPUS_TIERS[int(rng.integers(len(_CORPUS_TIERS)))]
        plan = _shaped(tgt, gain, dur)
        # The OLD shipped mesh: validate(...,200) now CLAMPS UP to 1600, so read the
        # true 200-sample gate via the batched function directly. r1600 goes through
        # the PUBLIC validate (exercising the real clamp/dispatch); r6400 (>floor) is
        # honoured by validate as the near-truth convergence reference.
        r200 = _batched_at(plan, lims, geom, 200)
        r1600 = feas.validate(plan, lims, geom, samples_per_segment=1600)
        r6400 = feas.validate(plan, lims, geom, samples_per_segment=6400)

        both_verdicts['ok' if r1600.ok else 'reject'] += 1
        # Cutover safety: 1600 must never ACCEPT what the shipped 200 REJECTS.
        if r1600.ok and not r200.ok:
            accept_reversals += 1

        # Peak-level checks only where BOTH meshes cleared geometry (peaks meaningful).
        if r200.peak_leg_jerk_mmps3 > 0.0 and r1600.peak_leg_jerk_mmps3 > 0.0:
            checked_peaks += 1
            if r1600.peak_leg_jerk_mmps3 < r200.peak_leg_jerk_mmps3 - 1e-6:
                jerk_reversals += 1
            if r6400.peak_leg_jerk_mmps3 > 0.0:
                conv = (abs(r1600.peak_leg_jerk_mmps3 - r6400.peak_leg_jerk_mmps3)
                        / r6400.peak_leg_jerk_mmps3)
                worst_conv = max(worst_conv, conv)

    assert accept_reversals == 0, (
        f"@1600 accepted {accept_reversals} plan(s) the shipped @200 rejected — "
        f"the cutover is NOT strictly stricter")
    assert jerk_reversals == 0, (
        f"jerk@1600 fell below jerk@200 on {jerk_reversals} move(s) — the mesh bump is "
        f"not strictly-more-conservative on the binding constraint")
    assert worst_conv <= 0.05, (
        f"jerk@1600 diverged {worst_conv*100:.2f}% from the 6400-reference (>5%) — the "
        f"1600 mesh no longer resolves shaped jerk to near-truth")
    # The corpus actually exercised BOTH verdicts and measured live peaks (not vacuous).
    assert both_verdicts['ok'] > 0 and both_verdicts['reject'] > 0, both_verdicts
    assert checked_peaks > N // 2
