"""Cup-contact contract (``plans/active/cup-contact-contract.md`` § 2/§ 4) —
tests 1, 2, 3 of the plan's test list, written FIRST and RED until the U2
production change lands (repo empirical-probe / TDD convention). Test 4 (the
sim xfail flip) is a later unit; test 5 lives in ``test_skills_executor.py``.

Sign convention (world z up, matches the plan's normative § 2): "seating
force" ``s = g + a_cup,z`` — ``g`` at rest, 0 in free fall, NEGATIVE when the
cup dives faster than g (no attitude can seat a ball).

* **C-CUP-1** — banking (``cup_realize.tilt_schedule``'s
  ``cfg.banking_enabled`` branch, ``tilt_geometry.tilt_to_receive(g − a_cup)``)
  is defined only at knots with ``s >= eps*g``, ``eps = 0.2``
  (``hw.JB_TRAJ_CUP_BANKING_SEATING_MIN_G``). Consequence: commanded tilt is
  amplitude-aware, tending to zero as the lateral residual tends to zero.
* **C-CUP-2** — over the contact window, ``a_cup,z >= -kappa*g``,
  ``kappa = 0.7`` (``hw.JB_TRAJ_CUP_CONTACT_ACC_FLOOR_G``): the cup never
  falls away from a held ball. Enforced by ``feasibility.validate_cycle``
  via the new ``CyclePlan.contact_knots`` field and the new
  ``feasibility.CUP_CONTACT_ACC`` refusal code (pinned interfaces, not yet
  implemented as of this file).
* **C-CUP-3** (not tested here — see test 2 below, which is the *symptom*
  the jerk-aware widen loop is meant to fix, not the widen loop itself)  —
  ``cup_realize._accel_bounded_schedule``'s widen loop should exit on the
  THIRD difference too, not just the second.

MEASURED 2026-09-18, before the contract (``scratchpad/probe_u1.py``, venv,
R3's single-site operating point: apex 0.9 m, ``sites.columns_sites(100.0)[0]``
self-toss, launch limits 300/5000/150000 mm(/s,/s^2,/s^3), hand acc 3500
rev/s^2 — CATCH with ``then_throw`` at the same site, landing offset ``dx``
applied to x only, y=0):

======  =============  ====================
dx_mm   peak_tilt_deg   peak_leg_jerk_mmps3
======  =============  ====================
  0.0           0.000                   0.0
  0.5           2.163              136608.4
  1.0           2.175              136703.6
  2.0           2.198              136893.1
  3.9           2.242              137249.9
  4.0           2.244              137268.5
  8.0           1.666               49762.2
 16.0           1.724               51675.8
 31.0           1.834               56171.0
======  =============  ====================

Today's failure shapes:

* **test 1** — peak commanded tilt jumps to ~2.2° at the SMALLEST nonzero
  offset (0.5 mm) and is non-monotone (drops from 2.244° at 4 mm to 1.666° at
  8 mm): the 12° raw clamp (``tilt_geometry.tilt_to_receive``'s
  ``max_tilt_deg`` saturation) survives through the rate/accel-bounded
  smoother regardless of how small the lateral residual is.
* **test 2** — the single reproduction point (dx=3.9 mm, 137250 mm/s^3) is
  itself UNDER the 150000 cap, but the grid is grossly non-monotone (137268
  at 4 mm down to 49762 at 8 mm — a discrete smoother-branch artefact), which
  is what the monotonicity assertion catches.
* **test 3** — ``CyclePlan`` accepts no ``contact_knots`` keyword yet and
  ``feasibility`` has no ``CUP_CONTACT_ACC`` code yet, so every sub-test
  fails either at construction (``TypeError``) or at the first assertion
  (interface gap, not a fixture mistake) — see the per-test docstrings and
  ``scratchpad/handoff_u1.md``.

Unmarked and parallel-safe: pure computation + a handful of real
``plan_segment`` solves (module-scope fixtures cache the grid — plan § 0's
"keep planning to a handful of solves"), no filesystem, no ports.
"""

from __future__ import annotations

import dataclasses
import re

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import ballistics_bc
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory.cycle_plan import CyclePlan
from jugglebot.motion.trajectory.limits import TrajectoryLimits
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as st

#: R3's single-site operating point (``tools/admissible_sweep.py``'s
#: ``SINGLE_SITE_LEG_JERK_MMPS3`` / ``tests/motion/test_skills_segments.py``'s
#: LEG_VEL/LEG_ACC/HAND_ACC — the leg jerk cap is R3's, not R2's 200000).
LEG_VEL = 300.0
LEG_ACC = 5000.0
LEG_JERK = 150000.0
HAND_ACC = 3500.0
APEX_M = 0.9
T_F = 2.0 * (2.0 * APEX_M / (ballistics_bc.GRAVITY_MMS2 / 1000.0)) ** 0.5
DWELL_S = 0.30
LAUNCH_S = 0.4               # tools/admissible_sweep.py SINGLE_SITE_LAUNCH_S

OFFSETS_MM = (0.0, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 31.0)
#: The exact 2026-09-16 reproduction point (the logbook entry / plan § 4 test 2).
JERK_REPRO_DX_MM = 3.9

DT = float(hw.JB_TRAJ_KNOT_DT_S)
#: kappa*g in mm/s^2 — the C-CUP-2 floor, from the generated constant (never
#: re-typed) times ballistics_bc's leg-space gravity constant.
_KAPPA_G_MM_S2 = float(hw.JB_TRAJ_CUP_CONTACT_ACC_FLOOR_G) * ballistics_bc.GRAVITY_MMS2
#: An IN-WORKSPACE level pose — same recipe as ``test_validate_cycle.py``'s
#: ``NEUTRAL`` — for the synthetic dives below.  A synthetic pose held at
#: z=750 mm is refused ``WORKSPACE`` by the geometry early-return before the
#: contact-floor ladder is ever reached (U3b diagnosis, 2026-09-18); this is
#: the in-workspace twin so those tests exercise ``CUP_CONTACT_ACC`` on
#: purpose, and ``OUT_OF_WORKSPACE_Z_MM`` below is kept for the ONE test that
#: wants the geometry refusal deliberately.
NEUTRAL = np.array([0.0, 0.0, float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM), 0.0, 0.0, 0.0])
OUT_OF_WORKSPACE_Z_MM = 750.0


def _rest_state(cup_mm):
    """Same construction as ``test_skills_segments.py`` / ``admissible_sweep.py``."""
    rcfg = cr.RealizeConfig()
    slider_mm = float(cup_mm[2]) - rcfg.cup_z_base_mm
    rev = ((slider_mm - rcfg.slider_rev_zero_mm) / 1000.0) * cr.HAND_REV_PER_M
    pose = np.array([cup_mm[0], cup_mm[1], rcfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, rcfg)


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=LEG_VEL, leg_acc_mmps2=LEG_ACC, leg_jerk_mmps3=LEG_JERK,
        hand_acc_rps2=HAND_ACC)


@pytest.fixture(scope='module')
def cfg():
    return sg.SegmentConfig()


@pytest.fixture(scope='module')
def site():
    """R3's single self-toss site (``sites.columns_sites``'s ``P1``)."""
    return st.columns_sites(100.0)[0]


@pytest.fixture(scope='module')
def catch_seed(limits, geom, site):
    """The post-release state a CATCH plans from: a real self-toss LAUNCH's
    release, same recipe as ``test_skills_segments.py``'s ``catch_seed`` /
    ``admissible_sweep._throw_cell``."""
    seed = _rest_state(site.rest_site_mm())
    goals = uc.CycleGoals(period_s=LAUNCH_S, throw_site_mm=site.throw_site_mm(),
                          throw_target_mm=site.throw_site_mm(), flight_s=T_F)
    plan_a, meta_a = uc.plan_launch(goals, seed, limits, geom)
    return uc.release_state_from_meta(meta_a, plan_a)


def _v_arrival():
    """A purely vertical self-toss's arrival velocity (target == site, no
    offset on the SEEDING throw — the same simplification
    ``test_skills_segments.py``'s ``catch_terminal`` fixture uses)."""
    return np.array([0.0, 0.0, -ballistics_bc.GRAVITY_MMS2 * (T_F / 2.0)])


def _catch_throw_segment(dx_mm, catch_seed, cfg, limits, geom, site):
    """The CATCH-with-``then_throw`` (STEADY window) R3's schedule actually
    dispatches, at lateral landing offset ``dx_mm`` (x only, y=0) — the same
    shape ``admissible_sweep._chained_catch_cell`` gates."""
    landing_mm = site.catch_site_mm() + np.array([dx_mm, 0.0, 0.0])
    then_throw = sg.ThrowAfterCatch(t_release_s=T_F + DWELL_S,
                                    site_mm=site.throw_site_mm(),
                                    target_mm=landing_mm, flight_s=T_F)
    terminal = sg.CatchTerminal(landing_mm=landing_mm,
                                landing_vel_mm_s=_v_arrival(), t_land_s=T_F,
                                rest_site_mm=site.rest_site_mm(),
                                then_throw=then_throw)
    return sg.plan_segment(sg.CATCH, catch_seed, terminal, cfg, limits, geom)


@pytest.fixture(scope='module')
def catch_throw_by_offset(catch_seed, cfg, limits, geom, site):
    """Every grid offset PLUS the exact reproduction point (3.9 mm), solved
    ONCE and cached — tests 1 and 2 both read this."""
    dxs = sorted(set(OFFSETS_MM) | {JERK_REPRO_DX_MM})
    return {dx: _catch_throw_segment(dx, catch_seed, cfg, limits, geom, site)
           for dx in dxs}


def _peak_tilt_deg(segment) -> float:
    """From-vertical tilt (2-norm of ``pose[:, 3:5]`` = (rx, ry)), peak over
    the whole plan, in degrees."""
    pose = segment.plan.pose
    return float(np.degrees(np.max(np.hypot(pose[:, 3], pose[:, 4]))))


# ═══════════════════════════════════════════════════════════════════════════
# Test 1 — amplitude invariance (C-CUP-1)
# ═══════════════════════════════════════════════════════════════════════════

def test_commanded_tilt_is_amplitude_aware_not_saturated(catch_throw_by_offset):
    """The peak commanded tilt must be monotone non-decreasing in the lateral
    offset and stay under 1 deg at 4 mm — i.e. it must tend to zero as the
    lateral residual tends to zero, not saturate to a fixed value for every
    nonzero offset. Today it does neither (module docstring table): every
    offset from 0.5-4 mm reads 2.16-2.24 deg, and the sequence is non-monotone
    past 4 mm (drops to 1.666 deg at 8 mm).
    """
    tilts = [_peak_tilt_deg(catch_throw_by_offset[dx]) for dx in OFFSETS_MM]
    for lo, hi in zip(tilts, tilts[1:]):
        assert hi >= lo - 1e-9, (
            f'peak commanded tilt not monotone non-decreasing across '
            f'{OFFSETS_MM}: {tilts}')
    tilt_4mm = _peak_tilt_deg(catch_throw_by_offset[4.0])
    assert tilt_4mm < 1.0, (
        f'peak commanded tilt at 4 mm is {tilt_4mm:.3f} deg, expected < 1 deg '
        '(measured 2026-09-18, before the contract: 2.244 deg)')


# ═══════════════════════════════════════════════════════════════════════════
# Test 2 — the 2026-09-16 jerk reproduction, and monotonicity
# ═══════════════════════════════════════════════════════════════════════════

def test_leg_jerk_reproduction_and_monotonicity(catch_throw_by_offset):
    """dx=3.9 mm: peak leg jerk (from ``validate_cycle``, via
    ``segment.meta.report``) must be under the session's 150000 mm/s^3 cap —
    it already is today (measured 137250) — AND peak leg jerk must be
    monotone non-decreasing in dx across the full grid, with a 2% relative
    tolerance for solver noise. Today's grid is grossly non-monotone (137268
    mm/s^3 at 4 mm down to 49762 at 8 mm, a ~64% drop — the widen loop's
    discrete-branch jump this assertion is built to catch), so this fails on
    the monotonicity half even though the single reproduction point passes.
    """
    jerk_3p9 = (catch_throw_by_offset[JERK_REPRO_DX_MM]
               .meta.report.peak_leg_jerk_mmps3)
    assert jerk_3p9 < LEG_JERK, (
        f'peak leg jerk at dx=3.9mm is {jerk_3p9:.0f} mm/s^3, cap {LEG_JERK:.0f}')

    jerks = [catch_throw_by_offset[dx].meta.report.peak_leg_jerk_mmps3
            for dx in OFFSETS_MM]
    tol = 1.02   # 2% relative tolerance for solver noise (plan § 4 test 2)
    for lo, hi in zip(jerks, jerks[1:]):
        assert hi >= lo / tol, (
            f'peak leg jerk not monotone non-decreasing (2% tol) across '
            f'{OFFSETS_MM}: {jerks}')


# ═══════════════════════════════════════════════════════════════════════════
# Test 3(a) — every real segment kind carries a contact window inside the floor
# ═══════════════════════════════════════════════════════════════════════════

@pytest.fixture(scope='module')
def throw_segment(limits, geom, cfg, site):
    seed = _rest_state(site.rest_site_mm())
    terminal = sg.ThrowTerminal(site_mm=site.throw_site_mm(),
                                target_mm=site.throw_site_mm(), flight_s=T_F,
                                t_release_s=LAUNCH_S)
    return sg.plan_segment(sg.THROW, seed, terminal, cfg, limits, geom)


@pytest.fixture(scope='module')
def standalone_catch_segment(catch_seed, cfg, limits, geom, site):
    terminal = sg.CatchTerminal(landing_mm=site.catch_site_mm(),
                                landing_vel_mm_s=_v_arrival(), t_land_s=T_F,
                                rest_site_mm=site.rest_site_mm())
    return sg.plan_segment(sg.CATCH, catch_seed, terminal, cfg, limits, geom)


@pytest.fixture(scope='module')
def catch_throw_segment(catch_throw_by_offset):
    return catch_throw_by_offset[0.0]


@pytest.fixture(scope='module')
def rest_segment(cfg, limits, geom, site):
    """A REST genuinely seeded AT REST (not from a catch/throw's post-release
    state — a cup that just released is decelerating out of its own take-off
    and cannot be holding anything, so a window there is refused at knot 0 by
    design). ``catch_seed`` is the seed FOR PLANNING a catch, i.e. a
    post-release state — using it here was the fixture bug: the whole point of
    this test is a REST that IS holding a ball, which is only true when it is
    seeded from rest (owner decision, U3 handoff § 'Needs an owner decision' 1).
    """
    seed = _rest_state(site.rest_site_mm())
    terminal = sg.RestTerminal(rest_site_mm=site.rest_site_mm(), t_rest_s=0.5)
    return sg.plan_segment(sg.REST, seed, terminal, cfg, limits, geom)


@pytest.mark.parametrize('fixture_name', [
    'throw_segment', 'standalone_catch_segment', 'catch_throw_segment',
    'rest_segment'])
def test_every_segment_carries_a_contact_window_within_the_dive_floor(
        fixture_name, request):
    """C-CUP-2: every segment kind's plan must carry ``contact_knots`` (the
    INCLUSIVE knot range a ball is/may be held over) and ``min(a_cup,z)``
    there must stay >= ``-kappa*g``.

    Measures a_cup,z from ``segment.meta.cup_plan.acc[:, 2]`` (m/s^2, times
    1000 for mm/s^2) — the SOURCE cup trajectory ``plan_segment``'s own QP
    solved for, preferred over reconstructing from ``plan.pose[:, 2]`` +
    ``plan.hand_rev`` (the realised, decomposed pair) because it is the exact
    planned quantity rather than an after-the-fact finite difference of a
    decomposition, and ``unified_cycle.CycleMeta`` already carries it
    (``cup_plan``, "present for a single window and for a spliced one").

    TODAY: ``CyclePlan`` accepts no ``contact_knots`` keyword at all, so
    every one of these plans has none — this fails at the first assertion
    (``getattr`` default ``None``) for every segment kind, not a fixture
    mistake.
    """
    segment = request.getfixturevalue(fixture_name)
    contact_knots = getattr(segment.plan, 'contact_knots', None)
    assert contact_knots is not None, (
        f'{fixture_name}: plan carries no contact_knots yet (CyclePlan has '
        'no such attribute) — the U2 CyclePlan/segments change has not landed')
    cup_plan = segment.meta.cup_plan
    assert cup_plan is not None, f'{fixture_name}: meta.cup_plan is None'
    a_cup_z_mmps2 = np.asarray(cup_plan.acc, dtype=float)[:, 2] * 1000.0
    # ``contact_knots`` is a tuple of ranges (2026-09-18, C-CUP-2's U3b change)
    # — none of these fixtures are chained plans, so each carries exactly one,
    # but the check reads it generically rather than assuming that shape.
    worst = np.inf
    for k0, k1 in contact_knots:
        window = a_cup_z_mmps2[k0:k1 + 1]
        assert window.size > 0, (
            f'{fixture_name}: empty contact_knots range {(k0, k1)} in '
            f'{contact_knots}')
        worst = min(worst, float(np.min(window)))
    assert worst >= -_KAPPA_G_MM_S2 - 1e-6, (
        f'{fixture_name}: min a_cup,z over contact_knots={contact_knots} is '
        f'{worst:.1f} mm/s^2, floor is {-_KAPPA_G_MM_S2:.1f} mm/s^2')


# ═══════════════════════════════════════════════════════════════════════════
# Test 3(b)/(c) — a synthetic CyclePlan drives validate_cycle directly
# ═══════════════════════════════════════════════════════════════════════════

def _generous_limits(dt=DT):
    """Leg ceilings effectively off, hand pair at the shipped default — same
    pattern as ``test_validate_cycle.py``'s ``_limits()``: constructed
    directly (not via ``from_config``/``with_session_limits``) so no ceiling
    clamp applies."""
    return TrajectoryLimits(
        leg_vel_mmps=1e9, leg_acc_mmps2=1e9, leg_jerk_mmps3=1e9,
        leg_vel_ceiling_mmps=1e12, leg_acc_ceiling_mmps2=1e12,
        leg_jerk_ceiling_mmps3=1e12,
        hand_vel_limit_rps=float(hw.JB_TRAJ_HAND_VEL_LIMIT_RPS),
        hand_acc_limit_rps2=float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2),
        hand_vel_ceiling_rps=1e12, hand_acc_ceiling_rps2=1e12,
        knot_dt_s=dt, max_step_rev=1e9, min_move_duration_s=0.2,
        min_timed_lead_s=0.25, max_timed_lead_s=60.0)


def _synthetic_diving_plan(contact_knots, *, a_dive_rps2=-280.0, n=4,
                           pose_row=None):
    """A level, stationary-platform ``CyclePlan`` whose hand channel dives at
    a CONSTANT acceleration.

    Exact-quadratic knot construction: a cubic Hermite spline matching a
    quadratic function's value+derivative at both span endpoints reproduces
    it exactly (the cubic term is identically zero), so every SUB-knot
    sample — not just the knots — reads the same acceleration. With the
    platform pose held bit-identically constant (pose_vel == 0 everywhere),
    leg vel/acc/jerk are exactly zero (``_hermite``'s acc/vel formulas
    collapse when p0==p1 and v0==v1==0), and the tilt is level (rx=ry=0), so
    ``cup_realize.decompose``'s ``drop_k = arm*(1 - a_z)`` term is IDENTICALLY
    zero regardless of the (irrelevant, at level) lever arm — i.e.
    ``cup_z_mm == slider_mm + const`` and ``a_cup,z(t) == d^2/dt^2[slider_mm(t)]``
    exactly, with nothing else contributing.

    ``a_dive_rps2 = -280`` rev/s^2 -> a_cup,z ~= -280 * 1000/HAND_REV_PER_M
    ~= -9118 mm/s^2, well past the -6864 mm/s^2 (-kappa*g, kappa=0.7) floor,
    while the hand stays within [0, ~10] rev stroke (starts at 2.0 rev, drops
    by <= 0.9 rev) and well under the hand vel/acc ceilings (peak |v| ~= 21
    rev/s, |a| = 280 rev/s^2 constant).
    """
    t = np.arange(n, dtype=float) * DT
    row = NEUTRAL if pose_row is None else pose_row
    pose = np.tile(np.asarray(row, dtype=float), (n, 1))
    pose_vel = np.zeros((n, 6))
    hand_rev0, hand_vel0 = 2.0, 0.0
    hand_rev = hand_rev0 + hand_vel0 * t + 0.5 * a_dive_rps2 * t ** 2
    hand_vel = hand_vel0 + a_dive_rps2 * t
    return CyclePlan(pose=pose, pose_vel=pose_vel, hand_rev=hand_rev,
                     hand_vel_rps=hand_vel, dt=DT, contact_knots=contact_knots)


def test_synthetic_dive_faster_than_kappa_g_is_refused_cup_contact_acc(geom):
    """A dive at ~-9118 mm/s^2 over ``contact_knots=(0, 3)`` (the whole
    plan) must refuse ``CUP_CONTACT_ACC``, and the reason must name the knot
    and the (negative) value.
    """
    plan = _synthetic_diving_plan(contact_knots=(0, 3))
    report = feas.validate_cycle(plan, _generous_limits(), geom)
    assert report.code == feas.CUP_CONTACT_ACC
    reason = ' '.join(report.reasons)
    assert re.search(r'knot\s*\d+', reason, re.IGNORECASE), (
        f'no knot index named in the refusal: {report.reasons!r}')
    assert re.search(r'-\d+(\.\d+)?', reason), (
        f'no negative magnitude named in the refusal: {report.reasons!r}')


def test_synthetic_dive_with_contact_knots_none_is_not_refused_on_that_code(geom):
    """The identical dive, un-gated (``contact_knots=None``): the check is
    vacuous, so this must NOT be refused ``CUP_CONTACT_ACC`` — and since
    nothing else about this plan violates a limit, the report should be OK
    outright.
    """
    plan = _synthetic_diving_plan(contact_knots=None)
    report = feas.validate_cycle(plan, _generous_limits(), geom)
    assert report.code != feas.CUP_CONTACT_ACC
    assert report.ok is True, (
        f'expected OK with contact_knots=None, got code={report.code!r} '
        f'reasons={report.reasons!r}')


def test_cup_contact_reason_survives_a_higher_priority_leg_refusal(geom):
    """Pinned-interface requirement: the refusal is reported WITH every other
    refusal, not instead of them. A plan that ALSO breaks a leg limit — here,
    a small x zig-zag (zero knot velocities either side of +-0.3 mm) against
    an artificially tiny 1.0 mm/s^3 session leg-jerk cap, so a leg code wins
    ``report.code`` virtually regardless of the exact motion profile — must
    still carry a CUP_CONTACT_ACC-shaped reason in ``report.reasons``.
    ``FeasibilityReport.reasons`` is a plain list (``field(default_factory=list)``),
    so nothing about today's shape prevents appending a second string to it
    once the check exists — the gap is only that the check does not exist yet.
    """
    n = 4
    t = np.arange(n, dtype=float) * DT
    pose = np.tile(NEUTRAL, (n, 1))
    pose[:, 0] = NEUTRAL[0] + np.array([0.0, 0.3, -0.3, 0.0])  # a small zig-zag, zero knot vel
    pose_vel = np.zeros((n, 6))
    a_dive_rps2 = -280.0
    hand_rev = 2.0 + 0.5 * a_dive_rps2 * t ** 2
    hand_vel = a_dive_rps2 * t
    plan = CyclePlan(pose=pose, pose_vel=pose_vel, hand_rev=hand_rev,
                     hand_vel_rps=hand_vel, dt=DT, contact_knots=(0, n - 1))
    limits = dataclasses.replace(_generous_limits(), leg_jerk_mmps3=1.0)
    report = feas.validate_cycle(plan, limits, geom)
    assert report.code != feas.OK, 'expected the zig-zag to trip a leg code'
    reason = ' '.join(report.reasons)
    assert re.search(r'knot\s*\d+', reason, re.IGNORECASE), (
        f'no CUP_CONTACT_ACC-shaped reason survived alongside '
        f'code={report.code!r}: {report.reasons!r}')


def test_cup_contact_reason_survives_a_workspace_early_return(geom):
    """The geometry early-return (U3b decision 3, "report every refusal at
    once" — the UH-3 rule): a pose OUT of workspace on purpose (kept at
    z=750 mm, the fixture pose the tests above moved away from) refuses
    ``WORKSPACE`` — but must still carry a CUP_CONTACT_ACC-shaped reason
    alongside it, because the early return fires before the ladder that would
    otherwise report it.
    """
    plan = _synthetic_diving_plan(
        contact_knots=(0, 3),
        pose_row=[0.0, 0.0, OUT_OF_WORKSPACE_Z_MM, 0.0, 0.0, 0.0])
    report = feas.validate_cycle(plan, _generous_limits(), geom)
    assert report.code == feas.WORKSPACE
    reason = ' '.join(report.reasons)
    assert re.search(r'knot\s*\d+', reason, re.IGNORECASE), (
        f'no CUP_CONTACT_ACC-shaped reason alongside code={report.code!r}: '
        f'{report.reasons!r}')
