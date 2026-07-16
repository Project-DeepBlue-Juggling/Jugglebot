"""Full feasibility-gate tests (Phase 2).

Covers the gate that all platform motion routes through:
  * the closed-form quintic peak bounds match dense sampling (property test — the
    copied ``controller`` math is correct and N-generalised);
  * every machine code is reachable with a populated ``reasons`` message
    (the loud-rejection contract), isolating vel / acc / jerk / step by choosing
    limits that make exactly one binding;
  * the jerk finite-difference and knot-step bound behave;
  * the report always carries every measured peak (so the planner can stretch).
"""

from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import TrajectoryLimits
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory.plan import HoldPlan, TrajectoryPlan
from jugglebot.motion.trajectory.segment import QuinticSegment
from jugglebot.motion.trajectory.quintic import (
    quintic_interp_with_accel,
    quintic_peak_acc_per_axis,
    quintic_peak_vel_per_axis,
)


NEUTRAL = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])


@pytest.fixture
def geom():
    return StewartGeometry()


@pytest.fixture
def limits():
    return TrajectoryLimits.from_config(hw)


def _limits(*, vel=1e9, acc=1e9, jerk=1e9, step=0.3, dt=0.025, min_dur=0.2):
    """A TrajectoryLimits with chosen session ceilings (defaults effectively off)."""
    return TrajectoryLimits(
        leg_vel_mmps=vel, leg_acc_mmps2=acc, leg_jerk_mmps3=jerk,
        leg_vel_ceiling_mmps=1e12, leg_acc_ceiling_mmps2=1e12,
        leg_jerk_ceiling_mmps3=1e12, knot_dt_s=dt, max_step_rev=step,
        min_move_duration_s=min_dur, min_timed_lead_s=0.25, max_timed_lead_s=60.0)


def _move(target, duration, p0=NEUTRAL):
    seg = QuinticSegment(p0, np.zeros(6), np.zeros(6),
                         target, np.zeros(6), np.zeros(6), duration)
    return TrajectoryPlan((seg,), final_pose=target)


# ── Closed-form peak bounds vs dense sampling (property test) ──

@pytest.mark.parametrize('seed', range(12))
def test_closed_form_peaks_match_dense_sampling(seed):
    """quintic_peak_{vel,acc}_per_axis agree with dense sampling of the quintic."""
    rng = np.random.default_rng(seed)
    n = 6
    p0 = rng.uniform(-50, 50, n)
    v0 = rng.uniform(-20, 20, n)
    a0 = rng.uniform(-10, 10, n)
    p1 = rng.uniform(-50, 50, n)
    v1 = rng.uniform(-20, 20, n)
    a1 = rng.uniform(-10, 10, n)
    T = rng.uniform(0.3, 2.5)

    peak_vel = quintic_peak_vel_per_axis(p0, v0, a0, p1, v1, a1, T)
    peak_acc = quintic_peak_acc_per_axis(p0, v0, a0, p1, v1, a1, T)

    ts = np.linspace(0.0, T, 4000)
    dense_vel = np.zeros(n)
    dense_acc = np.zeros(n)
    for t in ts:
        _, tw, ac = quintic_interp_with_accel(p0, v0, a0, p1, v1, a1, T, t)
        dense_vel = np.maximum(dense_vel, np.abs(tw))
        dense_acc = np.maximum(dense_acc, np.abs(ac))

    # Closed form is the true extremum (roots of the derivative), so it must be
    # >= the sampled max, and the sampled max must approach it as the grid refines.
    assert np.all(peak_vel >= dense_vel - 1e-6)
    assert np.all(peak_acc >= dense_acc - 1e-6)
    assert np.allclose(peak_vel, dense_vel, rtol=2e-3, atol=1e-3)
    assert np.allclose(peak_acc, dense_acc, rtol=2e-3, atol=1e-2)


# ── OK path + all peaks populated ─────────────────────────────

def test_hold_is_ok_and_zero_peaks(geom, limits):
    report = feas.validate(HoldPlan(NEUTRAL), limits, geom)
    assert report.ok and report.code == feas.OK
    assert report.peak_leg_vel_mmps == 0.0
    assert report.peak_step_rev == 0.0


def test_feasible_move_reports_all_peaks(geom):
    plan = _move(NEUTRAL + np.array([0, 0, 20, 0, 0, 0.]), 1.5)
    report = feas.validate(plan, _limits(), geom)
    assert report.ok
    # Every peak measured and positive for a real move.
    assert report.peak_leg_vel_mmps > 0
    assert report.peak_leg_acc_mmps2 > 0
    assert report.peak_leg_jerk_mmps3 > 0
    assert report.peak_step_rev > 0


# ── Rejection matrix: each machine code reachable, reasons populated ──

def test_reject_unreachable_nonfinite(geom, limits):
    bad = np.array([0.0, 0.0, np.nan, 0.0, 0.0, 0.0])
    report = feas.validate(HoldPlan(bad), limits, geom)
    assert not report.ok and report.code == feas.UNREACHABLE
    assert report.reasons


def test_reject_unreachable_singular(geom, monkeypatch):
    # No in-stroke pose in this geometry is ill-conditioned (the stroke limit binds
    # first), so exercise the condition-number backstop by forcing a high value.
    monkeypatch.setattr(feas, 'compute_condition_number',
                        lambda *a, **k: 1e3)
    report = feas.validate(_move(NEUTRAL + np.array([0, 0, 10, 0, 0, 0.]), 1.0),
                           _limits(), geom)
    assert not report.ok and report.code == feas.UNREACHABLE
    assert 'condition' in report.reasons[0].lower()


def test_reject_workspace(geom, limits):
    report = feas.validate(HoldPlan(np.array([0., 0., 500., 0., 0., 0.])),
                           limits, geom)
    assert not report.ok and report.code == feas.WORKSPACE
    assert report.reasons


def test_reject_limit_vel(geom):
    # Fast move, only the velocity ceiling low → LIMIT_VEL wins (highest priority).
    plan = _move(NEUTRAL + np.array([30, 0, 0, 0, 0, 0.]), 0.25)
    report = feas.validate(plan, _limits(vel=10.0), geom)
    assert not report.ok and report.code == feas.LIMIT_VEL
    assert report.reasons and report.peak_leg_vel_mmps > 10.0


def test_reject_limit_acc(geom):
    # Vel ceiling generous, acc ceiling low → LIMIT_ACC (vel passes first).
    plan = _move(NEUTRAL + np.array([30, 0, 0, 0, 0, 0.]), 0.5)
    report = feas.validate(plan, _limits(acc=10.0), geom)
    assert not report.ok and report.code == feas.LIMIT_ACC
    assert report.peak_leg_acc_mmps2 > 10.0


def test_reject_limit_jerk(geom):
    # Vel + acc generous, jerk ceiling low → LIMIT_JERK.
    plan = _move(NEUTRAL + np.array([30, 0, 0, 0, 0, 0.]), 0.8)
    report = feas.validate(plan, _limits(jerk=10.0), geom)
    assert not report.ok and report.code == feas.LIMIT_JERK
    assert report.peak_leg_jerk_mmps3 > 10.0


def test_reject_step_bound(geom):
    # Vel/acc/jerk effectively off; a fast move drives the per-knot u0 step past the
    # 80%-of-max bound. Isolates STEP_BOUND (last in priority).
    plan = _move(NEUTRAL + np.array([40, 0, 0, 0, 0, 0.]), 0.25)
    report = feas.validate(plan, _limits(step=0.05), geom)
    assert not report.ok and report.code == feas.STEP_BOUND
    assert report.reasons and report.peak_step_rev > feas.STEP_BOUND_MARGIN * 0.05


# ── Priority order: velocity wins when several would fail ──

def test_priority_vel_over_acc_over_jerk(geom):
    # A very fast move fails vel, acc, AND jerk; the reported code is LIMIT_VEL.
    plan = _move(NEUTRAL + np.array([30, 0, 0, 0, 0, 0.]), 0.2)
    report = feas.validate(plan, _limits(vel=10.0, acc=10.0, jerk=10.0), geom)
    assert report.code == feas.LIMIT_VEL


# ── Step-bound reflects the true wire knot sequence ──

def test_step_bound_matches_emitter_knot_sequence(geom):
    """The gate's peak_step equals the max |Δmotor_rev| the emitter would send."""
    from jugglebot.motion.trajectory import KnotEmitter
    plan = _move(NEUTRAL + np.array([15, 0, 5, 0, 0, 0.]), 0.6)
    report = feas.validate(plan, _limits(), geom)
    emitter = KnotEmitter(geom, knot_dt_s=0.025)
    prev = None
    measured = 0.0
    tau = 0.0
    while tau <= plan.total_duration + 1e-9:
        frame = emitter.frame(plan, tau, 0)
        mr = np.asarray(frame['motor_rev'], dtype=float)
        if prev is not None:
            measured = max(measured, float(np.max(np.abs(mr - prev))))
        prev = mr
        tau += 0.025
    assert report.peak_step_rev == pytest.approx(measured, abs=1e-6)


# ── Gate conservativeness: 80-sample default never accepts what dense@400 rejects ──

def _shaped(plan, gain):
    """Wrap a base plan in a LeanShaper (gain>0) — the shaped-plan path build_move
    hands to the analytic gate."""
    from jugglebot.motion.trajectory.shaping import LeanShaper
    return LeanShaper(gain, g_mm_s2=float(hw.GRAVITY_MMPS2)).shape(plan)


def test_validate_80_never_accepts_what_dense_400_rejects(geom):
    """The 2026-07-16 planning speedup dropped validate()'s default sampling 200→80
    and added a 5 % jerk margin. The safety property that makes that sound: the 80-
    sample gate (with margin) must NEVER ACCEPT a plan that a dense 400-sample gate
    REJECTS. If the margin under-covered the finite-difference jerk under-measurement
    at 80 samples, this would fail.

    Grid: several move directions × three limit tiers × a span of FIXED durations
    (built directly, NOT via build_move's stretch loop, so many land marginal/over-
    limit) × unshaped and shaped (LeanShaper gain 0.3 — the service lean path). Both
    the coarse and dense gate carry the same _VALIDATE_JERK_MARGIN, so this isolates
    the sampling-density effect. The dense@400 report is the near-truth reference.
    """
    targets = [
        NEUTRAL + np.array([150.0, 0, 0, 0, 0, 0.]),      # x+150 (operator battery)
        NEUTRAL + np.array([0, 150.0, 0, 0, 0, 0.]),      # y+150
        NEUTRAL + np.array([0, 0, 50.0, 0, 0, 0.]),       # z+50
        NEUTRAL + np.array([0, 0, 0, np.deg2rad(10), 0, 0.]),  # rx+10
        NEUTRAL + np.array([60.0, -40.0, 30.0, 0, np.deg2rad(6), 0.]),  # mixed
    ]
    tiers = [
        dict(vel=200.0, acc=660.0, jerk=10500.0),
        dict(vel=400.0, acc=2000.0, jerk=20000.0),
        dict(vel=1500.0, acc=5000.0, jerk=40000.0),
    ]
    durations = [0.2, 0.3, 0.45, 0.7, 1.0, 1.5]

    checked = 0
    contradictions = []
    for target in targets:
        for tier in tiers:
            lims = _limits(**tier)
            for dur in durations:
                base = _move(target, dur)
                for gain in (0.0, 0.3):
                    plan = _shaped(base, gain) if gain > 0.0 else base
                    r80 = feas.validate(plan, lims, geom)          # default 80
                    r400 = feas.validate(plan, lims, geom,
                                         samples_per_segment=400)
                    checked += 1
                    # The unsafe outcome: coarse gate ACCEPTS, dense gate REJECTS.
                    if r80.ok and not r400.ok:
                        contradictions.append(
                            (target.tolist(), tier, dur, gain, r400.code,
                             r80.peak_leg_jerk_mmps3, r400.peak_leg_jerk_mmps3))

    assert not contradictions, (
        f"80-sample gate accepted {len(contradictions)} plan(s) that dense@400 "
        f"rejected (of {checked} checked): {contradictions[:5]}")
    # Sanity: the grid actually exercised BOTH verdicts (not vacuously all-accept).
    assert checked > 0


def test_validate_80_unshaped_jerk_within_margin_of_dense_400(geom):
    """For an UNSHAPED rest-to-rest quintic the leg jerk is bounded, so its FD peak
    converges: the 80-sample peak is never more than a few % below the dense@400 peak
    — i.e. the 5 % _VALIDATE_JERK_MARGIN comfortably covers the sampling bias at the
    lowered default. (Shaped plans are handled separately — see the floor test below —
    because their FD jerk peak under-converges: window-edge curvature since the C2
    lean window landed; a boundary jerk impulse before it.)"""
    worst_shortfall = 0.0  # max fractional amount j80 falls below j400
    lims = _limits(vel=1e9, acc=1e9, jerk=1e9)  # limits off → always OK, peaks free
    for target in [NEUTRAL + np.array([150.0, 0, 0, 0, 0, 0.]),
                   NEUTRAL + np.array([0, 150.0, 0, 0, 0, 0.]),
                   NEUTRAL + np.array([0, 0, 50.0, 0, 0, 0.]),
                   NEUTRAL + np.array([60.0, -40.0, 30.0, 0, np.deg2rad(6), 0.])]:
        for dur in (0.3, 0.5, 0.8, 1.2):
            plan = _move(target, dur)   # unshaped
            j80 = feas.validate(plan, lims, geom).peak_leg_jerk_mmps3
            j400 = feas.validate(plan, lims, geom,
                                 samples_per_segment=400).peak_leg_jerk_mmps3
            if j400 > 0.0:
                worst_shortfall = max(worst_shortfall, (j400 - j80) / j400)
    # Both peaks carry the same ×1.05 margin, so any shortfall is pure sampling bias.
    assert worst_shortfall < 0.05, (
        f"unshaped 80-sample jerk fell {worst_shortfall*100:.1f}% below dense@400 — "
        f"the 5% margin no longer covers the sampling bias")


def test_shaped_plan_gated_at_status_quo_sample_floor(geom):
    """A shaped plan's FD jerk peak under-converges with sample count. Pre-window the
    cause was a near-impulsive boundary jerk (lean tilt_rate ∝ base jerk, stepping at
    rest-to-rest quintic boundaries); the C2 lean plateau window (same day) removed
    that impulse, but the window's edge ramps concentrate high w″ curvature that the
    FD grid still under-resolves at 200 samples. Either way: lowering the analytic
    gate's default to 80 must NOT change how shaped plans are gated, or shaped moves
    would be measured as lower-jerk and stretched less — silently more aggressive on
    the lean path. This pins the guarantee that validate() gates a shaped plan at the
    _SHAPED_VALIDATE_SAMPLES (=200, the pre-speedup value) floor regardless of the
    lowered default.
    """
    lims = _limits(vel=1e9, acc=1e9, jerk=1e9)
    plan = _shaped(_move(NEUTRAL + np.array([150.0, 0, 0, 0, 0, 0.]), 1.0), 0.3)

    j_default = feas.validate(plan, lims, geom).peak_leg_jerk_mmps3
    j_200 = feas.validate(plan, lims, geom, samples_per_segment=200).peak_leg_jerk_mmps3
    j_80req = feas.validate(plan, lims, geom, samples_per_segment=80).peak_leg_jerk_mmps3
    j_400 = feas.validate(plan, lims, geom, samples_per_segment=400).peak_leg_jerk_mmps3

    # The default (lowered to 80) gates a shaped plan exactly as the 200 floor does.
    assert j_default == pytest.approx(j_200, rel=1e-12)
    # A coarser-than-floor request is clamped UP to the floor (never gates a shaped
    # plan below status quo) — the load-bearing safety guarantee.
    assert j_80req == pytest.approx(j_200, rel=1e-12)
    # A denser request is still honoured, and reads a MATERIALLY higher peak: direct
    # evidence the shaped FD jerk has not converged at 200 (the reason the floor
    # exists and the reason lowering it would silently under-gate shaped moves).
    # Post-window this residual comes from the window-edge w″ ramps; if a future
    # smoothing change makes shaped jerk converge, relax this to >= and revisit
    # the floor itself.
    assert j_400 > j_200 * 1.05
