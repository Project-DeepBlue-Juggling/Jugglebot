"""Whole-cycle 7-DoF gate — ``sim/cycle_gate.py``.

Plan: ``plans/active/unified-7dof-planner.md`` § 4 Phase 1, the phase gate.

Unmarked (per-commit), matching ``tests/sim/test_toss_gate.py``: the gate drives
the production planning chain, which is the hardware-safety surface Phase 2 ships,
so it is never a ``nightly`` demotion candidate.  The module-scoped fixture runs a
two-point subset with the advisory MuJoCo column and the flight sweep OFF, which
is the whole file's cost; the heavier columns get one-cell slices.

The load-bearing invariants asserted here:

* the gate's own verdict on the pinned grid, and that the grid is not vacuous;
* the plan's two required comparisons — zero-banking parity against
  ``realize_tilted`` at **exactly** 0.0, and banking beating level on the lateral
  specific-force impulse with MATCHED pins;
* the runway constraint really refuses a catch with no room under it (the Rung-3
  failure mode), and the no-slam band really fails on a plan that does slam;
* the two window regressions WP4 found: the catch is scored at the interpolated
  touch-down, not at ``cup.catch_k``, and the carry starts at touch-down;
* MuJoCo is advisory — the verdict is identical with the column on and off.

Reported but deliberately NOT asserted: ``v_match`` (a documented deferral — the
cup QP matches catch velocity softly by design), the MuJoCo make/drop counts, and
the shipped-session-limit verdict (an owner decision, see ``_SHIPPED_LIMIT_NOTE``).
"""

from __future__ import annotations

import json

import numpy as np
import pytest

pytest.importorskip('mujoco')

from jugglebot import hardware_config as hw                        # noqa: E402
from jugglebot.motion.trajectory import cup_cycle as cc            # noqa: E402
from jugglebot.motion.trajectory import cup_realize as cr          # noqa: E402
from jugglebot.motion.trajectory.hand_stroke import (              # noqa: E402
    LINEAR_GAIN_REV_PER_M,
)
from sim.cycle_gate import (                                       # noqa: E402
    CAPTURE_TOL_MM, CATCH_CUP_Z_M, CUP_Z_HI_M, CUP_Z_LO_M,
    CycleGate, CycleGateConfig, SEAT_CONE_DEG, THROW_CUP_Z_M,
    _pins, _point_id, cup_state_at, default_grid, first_carry_knot,
    lateral_impulse, max_plannable_flight_s, plan_cycle, run_gate,
    seat_angles, touchdown_time_s,
)

_SMOKE_POINTS = [
    dict(flight_s=0.60, throw_xy_mm=(0.0, 0.0), catch_xy_mm=(0.0, 0.0),
         banking=True, advisory=False),
    dict(flight_s=0.60, throw_xy_mm=(0.0, 0.0), catch_xy_mm=(80.0, 0.0),
         banking=True, advisory=False),
]


@pytest.fixture(scope='module')
def smoke_report():
    """The gate on a two-point subset — one same-site cycle, one re-pose cycle."""
    cfg = CycleGateConfig(points=_SMOKE_POINTS, trials_per_point=1, seed=0,
                          contact_diag=False, flight_sweep=False)
    return CycleGate(cfg).run()


@pytest.fixture(scope='module')
def demo_cycle():
    """One planned cup cycle, reused by the pure geometry tests."""
    return plan_cycle(_SMOKE_POINTS[0])


# ── the gate's verdict ───────────────────────────────────────────────────────

def test_the_smoke_subset_passes_and_is_not_vacuous(smoke_report):
    """A PASS that came from real accepted cycles, not from an empty band."""
    assert smoke_report['passed'] is True
    assert smoke_report['no_binding_points'] is False
    assert smoke_report['accepted'] == len(_SMOKE_POINTS)
    assert smoke_report['core_clean'] == len(_SMOKE_POINTS)
    assert smoke_report['rejected'] == []


def test_every_gating_band_is_reported_per_cycle(smoke_report):
    """``core_clean`` is the conjunction it claims to be, on every row.

    Without this a band could silently stop being read and the gate would keep
    passing — the failure mode the ``core_clean``/``clean`` split exists to make
    visible in ``toss_gate``.
    """
    for row in smoke_report['results']:
        assert row['core_clean'] == (
            row['validate_ok'] and row['capture_ok'] and row['seat_ok']
            and row['no_slam'] and row['runway_active'])
        assert row['validate_code'] == 'OK'


def test_the_default_grid_has_unique_ids_and_a_real_binding_band():
    grid = default_grid()
    ids = [_point_id(p) for p in grid]
    assert len(ids) == len(set(ids)), 'grid point ids collide'
    binding = [p for p in grid if not p['advisory']]
    advisory = [p for p in grid if p['advisory']]
    assert len(binding) >= 8 and advisory, 'grid must have both bands'
    # The re-pose column really does displace, and stays inside the envelope.
    disp = [float(np.hypot(p['catch_xy_mm'][0] - p['throw_xy_mm'][0],
                           p['catch_xy_mm'][1] - p['throw_xy_mm'][1]))
            for p in grid]
    assert max(disp) == pytest.approx(80.0, abs=1e-6)


def test_the_advisory_flight_really_is_over_the_hand_cap():
    """The advisory band is a physics statement, not a convenience exemption.

    If 0.9 s ever fits under the hand-acceleration cap it belongs in the binding
    band, and this test is what notices.
    """
    gate = CycleGate(CycleGateConfig(contact_diag=False, flight_sweep=False))
    res = gate.run_trial(0, dict(flight_s=0.90, throw_xy_mm=(0.0, 0.0),
                                 catch_xy_mm=(0.0, 0.0), banking=True,
                                 advisory=True))
    assert res.accepted
    assert res.peak_hand_acc_rps2 > float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)
    assert res.validate_code == 'HAND_LIMIT_ACC'


# ── the plan's two required comparisons ──────────────────────────────────────

def test_zero_banking_reproduces_realize_tilted_exactly(smoke_report):
    """Comparison 1, at ``== 0.0``.

    Not a tolerance: the port's whole claim is that ``decompose`` IS
    ``realize_tilted`` generalised to a knot series, and the two axis forms it
    could have used differ at 8e-17 — so any non-zero here means the chain picked
    up a second spelling of the geometry.
    """
    assert smoke_report['parity_exact'] is True
    assert smoke_report['parity_worst_abs'] == 0.0


def test_banking_beats_level_on_the_lateral_impulse(smoke_report):
    """Comparison 2, with MATCHED pins on both arms.

    The score is the lateral specific force the ball actually feels,
    ``∫|(g − a_cup) × cup_axis| dt``.  Matched pins are the point: the receive
    tilt is a boundary condition of the catch, so an unmatched comparison would
    be scoring the pin rather than the banking.
    """
    assert smoke_report['banking_beats_level'] is True
    assert smoke_report['banking_arms'] == len(_SMOKE_POINTS)
    for row in smoke_report['results']:
        assert row['lateral_impulse_banked'] < row['lateral_impulse_level']
        assert row['banking_gain_frac'] > 0.0


def test_the_banking_comparison_uses_the_same_pins_on_both_arms(demo_cycle):
    """Pin the matched-arm discipline itself.

    Scoring a banked arm carrying a receive pin against a LEVEL-pinned zero-banking
    arm reverses the verdict on the demo cycle (measured 2026-09-01: 0.52 vs 0.48
    with mismatched pins, where the matched pair is 0.26 vs 0.51).  The comparison
    is only meaningful pin-for-pin.
    """
    recv, throw = _pins(demo_cycle)
    banked = cr.tilt_schedule(demo_cycle, recv, throw,
                              cr.RealizeConfig(banking_enabled=True))
    level = cr.tilt_schedule(demo_cycle, recv, throw,
                             cr.RealizeConfig(banking_enabled=False))
    # Same pins, both arms — exactly.
    np.testing.assert_array_equal(banked[demo_cycle.catch_k], recv)
    np.testing.assert_array_equal(level[demo_cycle.catch_k], recv)
    assert lateral_impulse(demo_cycle, banked) < lateral_impulse(demo_cycle, level)


# ── the runway constraint, and the slam it exists to prevent ─────────────────

def _cycle_at_catch_height(catch_z_m, *, runway):
    """Plan the demo cycle with the catch moved down and the runway on/off."""
    import sim.cycle_gate as gate_mod
    cfg = cc.CupCycleConfig(z_min_m=CUP_Z_LO_M, z_max_m=CUP_Z_HI_M,
                            catch_runway_z_floor_m=CUP_Z_LO_M,
                            catch_runway_enabled=runway)
    saved = gate_mod.CATCH_CUP_Z_M
    gate_mod.CATCH_CUP_Z_M = catch_z_m
    try:
        return gate_mod.plan_cycle(_SMOKE_POINTS[0], cfg)
    finally:
        gate_mod.CATCH_CUP_Z_M = saved


def test_the_runway_refuses_a_catch_with_no_stroke_under_it():
    """The Rung-3 answer, asserted as a refusal rather than as an outcome.

    With the runway constraint ON, a catch 30 mm above the slider floor is refused
    by name.  With it OFF the same request plans happily and puts touch-down at
    ~1.0 rev — about 30 mm of travel in which to stop a 1.75 rev/s slider, which is
    precisely the "obtain the deceleration room by overshooting" trade the phase
    exists to remove.
    """
    with pytest.raises(cc.CupCycleInfeasible) as exc:
        _cycle_at_catch_height(0.72, runway=True)
    assert exc.value.reason == 'CATCH_RUNWAY'

    loose = _cycle_at_catch_height(0.72, runway=False)
    k = first_carry_knot(loose)
    rcfg = cr.RealizeConfig(banking_enabled=True)
    recv, throw = _pins(loose)
    realized = cr.decompose(loose, cr.tilt_schedule(loose, recv, throw, rcfg),
                            rcfg)
    assert float(realized.slider_rev[k]) < 1.5, (
        'the unconstrained program is supposed to take an unusable catch height; '
        'if it no longer does, this test proves nothing')


def test_no_slam_fails_on_a_plan_that_does_slam(demo_cycle):
    """Non-vacuity for the ceiling band.

    On the gate's own cup-z box the band passes with ~0.3 rev to spare, so it must
    be shown to FAIL on a plan that leaves the band — otherwise "slam-free" would
    be a property of the box rather than a check.  ``cup_cycle``'s DEFAULT box
    (0.45/1.10 m) is exactly such a plan: it asks the slider for travel it does not
    have.
    """
    prime = float(hw.JB_OP_HAND_CATCH_PRIME_REV)
    gate = CycleGate(CycleGateConfig(contact_diag=False, flight_sweep=False))
    res = gate.run_trial(0, _SMOKE_POINTS[0])
    assert res.no_slam is True
    assert res.pre_catch_hand_max_rev <= prime

    default_box = plan_cycle(_SMOKE_POINTS[0], cc.CupCycleConfig())
    rcfg = cr.RealizeConfig(banking_enabled=True)
    recv, throw = _pins(default_box)
    realized = cr.decompose(default_box,
                            cr.tilt_schedule(default_box, recv, throw, rcfg),
                            rcfg)
    k = first_carry_knot(default_box)
    # The stroke CLAMP in ``decompose`` hides the overshoot in ``slider_mm``, so
    # the assertion is on the DEMAND: the un-clamped slider the plan asks for.
    assert realized.slider_saturated[:k + 1].any(), 'the clamp never engaged'
    demanded_mm = (default_box.pos[:k + 1, 2] * 1000.0 - cr.CUP_Z_BASE_MM
                   - cr.SLIDER_REV_ZERO_MM)
    assert demanded_mm.max() / 1000.0 * LINEAR_GAIN_REV_PER_M > prime, (
        'the default cup-z box is supposed to demand more stroke than the prime '
        'ceiling; if it no longer does, this non-vacuity case is stale')


def test_runway_is_active_on_every_accepted_cycle(smoke_report):
    """And the margin is a MEASURED number, read back from the planner itself."""
    assert smoke_report['runway_active_everywhere'] is True
    assert smoke_report['slam_free_everywhere'] is True
    assert smoke_report['worst_runway_margin_m'] >= 0.0
    for row in smoke_report['results']:
        assert np.isfinite(row['runway_margin_m'])
    # Non-vacuity: the requirement is a real distance, not a rounding of zero.
    _floor, need = cc.catch_runway_requirement(
        np.array([0.10, -0.05, -2.50]),
        cc.CupCycleConfig(z_min_m=CUP_Z_LO_M, z_max_m=CUP_Z_HI_M,
                          catch_runway_z_floor_m=CUP_Z_LO_M))
    assert need > 0.02, 'the runway requirement collapsed to the margin alone'


# ── the two window regressions WP4 found ─────────────────────────────────────

def test_the_catch_is_scored_at_touchdown_not_at_the_catch_knot(demo_cycle):
    """``cup.catch_k`` is ``floor(t_catch/dt)`` — 20 ms of descent before the pin.

    Scoring at the knot read 33.7 mm of "capture error" on a cycle whose true
    error at touch-down is ~1e-11 mm.  Both numbers are asserted so the regression
    cannot come back as a smaller-but-still-wrong offset.
    """
    site = np.array([0.0, 0.0, CATCH_CUP_Z_M])
    p_td, _ = cup_state_at(demo_cycle, touchdown_time_s())
    assert np.linalg.norm(p_td - site) * 1000.0 < 1e-6
    knot_err = np.linalg.norm(demo_cycle.pos[demo_cycle.catch_k] - site) * 1000.0
    assert knot_err > CAPTURE_TOL_MM, (
        'the catch no longer falls between knots, so this regression case is '
        'stale — pick a CATCH_FRAC that does')


def test_the_carry_is_scored_from_touchdown_not_from_the_catch_knot(demo_cycle):
    """At the knot before touch-down the cup is still diving to meet the ball.

    Measured there: ``a_z = −13.5 m/s²``, i.e. the cup is falling away from where
    the ball will be, so apparent-up points the other way and the seat angle reads
    ~177° on a carry that is otherwise inside a few degrees.  The window must start
    when the ball is actually in the cup.
    """
    assert first_carry_knot(demo_cycle) == int(demo_cycle.catch_k) + 1
    recv, throw = _pins(demo_cycle)
    tilts = cr.tilt_schedule(demo_cycle, recv, throw,
                             cr.RealizeConfig(banking_enabled=True))
    worst, scored = seat_angles(demo_cycle, tilts)
    assert scored > 0 and worst <= SEAT_CONE_DEG

    g = np.array([0.0, 0.0, -float(hw.GRAVITY_MPS2)])
    pre = g - demo_cycle.acc[int(demo_cycle.catch_k)]
    assert pre[2] > 0.0, (
        'the pre-touch-down knot is supposed to sit in the dive that flips '
        'apparent-up; if it no longer does, this regression case is stale')


# ── deferrals and the advisory column ────────────────────────────────────────

def test_v_match_is_reported_and_deferred_not_gated(smoke_report):
    """The soft velocity match is a design choice, so it is measured, not gated.

    ``catch_slider_vel_ratio = 0.7`` puts the achieved match near 0.3 BY
    CONSTRUCTION; the deferral must travel with the artifact so a future reader
    does not mistake it for an unchecked band.
    """
    assert smoke_report['deferred_criteria'] == ['v_match']
    assert 'catch_slider_vel_ratio' in smoke_report['deferred_note']
    ratio = cc.CupCycleConfig().catch_slider_vel_ratio
    for row in smoke_report['results']:
        assert row['v_match'] == pytest.approx(1.0 - ratio, abs=0.05)
        assert row['core_clean'] is True        # …and it did not gate anything


def test_mujoco_is_advisory_and_cannot_change_the_verdict():
    """Same grid, column on and off — identical verdict and identical bands."""
    base = dict(points=_SMOKE_POINTS[:1], trials_per_point=1,
                flight_sweep=False)
    off = CycleGate(CycleGateConfig(contact_diag=False, **base)).run()
    on = CycleGate(CycleGateConfig(contact_diag=True, **base)).run()
    assert on['passed'] == off['passed'] is True
    assert on['contact_diagnostic']['gating'] is False
    assert 'ADVISORY ONLY' in on['contact_diagnostic']['note']
    for a, b in zip(on['results'], off['results']):
        for band in ('validate_ok', 'capture_ok', 'seat_ok', 'no_slam',
                     'core_clean'):
            assert a[band] == b[band]
    # The column really ran, so "advisory" is not a synonym for "skipped".
    assert on['contact_diagnostic']['ran'] is True
    assert off['contact_diagnostic']['ran'] is False


def test_the_shipped_limit_verdict_travels_with_the_report(smoke_report):
    """The gate runs catch-capable session limits; the shipped verdict is REPORTED.

    That disagreement is the Phase-1 owner decision, so it must be impossible to
    read the artifact without meeting it.
    """
    assert smoke_report['shipped_limit_verdicts']
    assert 'REPORTED, NOT GATED' in smoke_report['shipped_limit_note']
    assert smoke_report['thresholds']['session_leg_jerk_mmps3'] == 150000.0


# ── the headline sweep and the artifact ──────────────────────────────────────

def test_the_flight_headline_finds_a_ceiling_and_names_it():
    """The number the owner asked for, with the row above it over the cap."""
    head = max_plannable_flight_s(step_s=0.10, hi_s=1.00)
    assert np.isfinite(head['max_flight_s'])
    assert 0.4 <= head['max_flight_s'] <= 1.0
    assert head['runway_below_release_m'] == pytest.approx(
        THROW_CUP_Z_M - CUP_Z_LO_M)
    over = [r for r in head['rows']
            if r['planned'] and r['flight_s'] > head['max_flight_s']]
    assert over, 'the sweep never found the ceiling it reports'
    assert all(r['hand_acc_rps2'] > head['hand_acc_cap_rps2']
               or r['validate_code'] != 'OK' for r in over)


def test_run_gate_writes_the_report_where_it_says(tmp_path):
    path = tmp_path / 'cycle_gate.json'
    rep = run_gate(CycleGateConfig(points=_SMOKE_POINTS[:1], contact_diag=False,
                                   flight_sweep=False, report_path=str(path)))
    assert path.exists()
    assert rep['report_path'] == str(path)
    on_disk = json.loads(path.read_text())
    assert on_disk['gate'] == 'cycle'
    assert on_disk['passed'] is True
    # Every per-trial field must be JSON-clean (no None where a float belongs).
    for row in on_disk['results']:
        assert isinstance(row['throw_xy_mm'], list)
        assert row['reject_code'] is None
