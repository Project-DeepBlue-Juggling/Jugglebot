"""Batched shaped-gate parity tests (Phase 1a of shaped-planning-efficiency).

Phase 1a vectorised the SHAPED branch of ``feasibility.validate`` at the current
``_SHAPED_VALIDATE_SAMPLES`` (=200) mesh — a bit-parity refactor with ZERO behaviour
change. Two things are pinned here:

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

The existing shaped tests in ``test_trajectory_feasibility.py`` and
``test_trajectory_shaping.py`` pin the behaviour at n=200 UNCHANGED — those, plus (a)
and (b) here, are the zero-behaviour-change proof.

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
        leg_jerk_ceiling_mmps3=1e12, knot_dt_s=dt, max_step_rev=step,
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
        rb = feas.validate(plan, lims, geom, samples_per_segment=200)
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
            rb = feas.validate(plan, lims, geom, samples_per_segment=800)
            rs = _scalar_reference(plan, lims, geom, 800)
            _assert_report_parity(rb, rs, f"{name} g{gain} n800")


# One deliberately-failing plan per code — batched must reproduce the scalar
# reference's code AND fields exactly.

def test_gate_parity_reject_workspace(geom):
    # A large-z shaped move drives the base pose out of stroke → WORKSPACE.
    plan = _shaped(NEUTRAL + np.array([0, 0, 400., 0, 0, 0.]), 0.6, 1.0)
    lims = _limits()
    rb = feas.validate(plan, lims, geom, samples_per_segment=200)
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
    rb = feas.validate(plan, lims, geom, samples_per_segment=200)
    rs = _scalar_reference(plan, lims, geom, 200)
    assert rb.code == feas.UNREACHABLE
    assert rb.peak_leg_ext_mm == 0.0
    _assert_report_parity(rb, rs, 'reject-nonfinite')


def test_gate_parity_reject_limit_vel(geom):
    plan = _shaped(NEUTRAL + np.array([120., 0, 0, 0, 0, 0.]), 0.6, 0.35)
    lims = _limits(vel=200.0)
    rb = feas.validate(plan, lims, geom, samples_per_segment=200)
    rs = _scalar_reference(plan, lims, geom, 200)
    assert rb.code == feas.LIMIT_VEL
    _assert_report_parity(rb, rs, 'reject-vel')


def test_gate_parity_reject_limit_acc(geom):
    plan = _shaped(NEUTRAL + np.array([120., 0, 0, 0, 0, 0.]), 0.6, 0.5)
    lims = _limits(acc=800.0)
    rb = feas.validate(plan, lims, geom, samples_per_segment=200)
    rs = _scalar_reference(plan, lims, geom, 200)
    assert rb.code == feas.LIMIT_ACC
    _assert_report_parity(rb, rs, 'reject-acc')


def test_gate_parity_reject_limit_jerk(geom):
    plan = _shaped(NEUTRAL + np.array([120., 0, 0, 0, 0, 0.]), 0.6, 0.8)
    lims = _limits(jerk=8000.0)
    rb = feas.validate(plan, lims, geom, samples_per_segment=200)
    rs = _scalar_reference(plan, lims, geom, 200)
    assert rb.code == feas.LIMIT_JERK
    _assert_report_parity(rb, rs, 'reject-jerk')


def test_gate_parity_reject_step_bound(geom):
    plan = _shaped(NEUTRAL + np.array([120., 0, 0, 0, 0, 0.]), 0.6, 0.4)
    lims = _limits(step=0.05)
    rb = feas.validate(plan, lims, geom, samples_per_segment=200)
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
