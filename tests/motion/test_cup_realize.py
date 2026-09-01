"""Unified 7-DoF planner, Phase 1 — ``cup_realize`` tilt schedule + decomposition.

Plan: plans/active/unified-7dof-planner.md § 4 Phase 1, tests T-U3 and T-U4.

T-U3 covers :func:`cup_realize.tilt_schedule` — apparent-gravity alignment inside
the saturation band, the 12° cap, the exact receive/throw endpoint pins, the rate
limit — and the load-bearing one: **zero-banking decomposition reproduces
``sim/juggle_tilt.py::realize_tilted`` bit for bit**.  That parity is the Phase-1
acceptance bar, so it is asserted at ``max |Δ| == 0.0``, not at a tolerance; the
pattern is ported from ``tests/sim/test_juggle_tilt.py`` (drive both functions with
identical fixture inputs and compare the realisation outputs directly).

T-U4 covers the z pin: false ⇒ every emitted pose z is ``JB_OP_DEFAULT_ACTIVE_Z_MM``
EXACTLY (``==``, not ``approx`` — the pin is an identity, and it has two chains: the
production constant here and the sim's ``Z_ACTIVE_MM`` literal, which the parity
test cross-checks); true ⇒ z moves only where the slider saturates and never
further than the band.
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import hand_stroke
from jugglebot.motion.trajectory import tilt_geometry as tg


# ── fixture cup plans (the CupCyclePlan field contract, duck-typed) ───────────

@dataclasses.dataclass
class _CupPlan:
    """The ``sim/juggle_planner/juggle_planner.py:86`` field contract, by hand.

    Production code duck-types this shape; the tests build it with numpy so no
    ``sim`` import (and no CasADi) is needed to exercise the realisation.
    """
    pos: np.ndarray
    vel: np.ndarray
    acc: np.ndarray
    jerk: np.ndarray
    t: np.ndarray
    dt: float
    catch_k: int
    takeoff_vel: np.ndarray


def _make_cup_plan(n_knots: int = 21, dt: float = 0.025, catch_k: int = 8,
                   *, z_lo: float = 0.75, z_hi: float = 0.95,
                   xy_amp: float = 0.08, accel_scale: float = 1.0) -> _CupPlan:
    """A smooth analytic cup track — closed-form pos/vel/acc so the derivative
    relations the realisation relies on are exact, not finite-differenced."""
    t = np.arange(n_knots, dtype=float) * dt
    T = max(t[-1], dt)
    w = 2.0 * np.pi / T
    z_mid = 0.5 * (z_lo + z_hi)
    z_amp = 0.5 * (z_hi - z_lo)

    pos = np.empty((n_knots, 3))
    vel = np.empty((n_knots, 3))
    acc = np.empty((n_knots, 3))
    pos[:, 0] = xy_amp * np.sin(w * t)
    pos[:, 1] = 0.6 * xy_amp * np.sin(2.0 * w * t)
    pos[:, 2] = z_mid - z_amp * np.cos(w * t)
    vel[:, 0] = xy_amp * w * np.cos(w * t)
    vel[:, 1] = 0.6 * xy_amp * 2.0 * w * np.cos(2.0 * w * t)
    vel[:, 2] = z_amp * w * np.sin(w * t)
    acc[:, 0] = -xy_amp * w * w * np.sin(w * t) * accel_scale
    acc[:, 1] = -0.6 * xy_amp * (2.0 * w) ** 2 * np.sin(2.0 * w * t) * accel_scale
    acc[:, 2] = z_amp * w * w * np.cos(w * t) * accel_scale

    return _CupPlan(pos=pos, vel=vel, acc=acc,
                    jerk=np.zeros((n_knots - 1, 3)), t=t, dt=dt,
                    catch_k=catch_k, takeoff_vel=vel[-1].copy())


# ── T-U3: tilt_schedule ──────────────────────────────────────────────────────

def test_banking_tracks_apparent_gravity_inside_saturation():
    """Unsaturated, the cup axis lands anti-parallel to ``g − a_cup``.

    That IS the banking invariant: a ball in the cup feels the specific force
    ``g − a_cup``, so pointing the cup's up-axis against that field presses the
    ball straight down the axis with no lateral component in the cup frame.
    """
    # Small accelerations so nothing hits the 12° cap.  BOTH shaping bounds are
    # switched off so what is left is the banking objective alone: the rate limit
    # and the accel limit are comfort/feasibility bounds that deliberately move the
    # schedule off the objective, and this test is about the objective.  That the
    # shaped schedule still *tracks* the objective is asserted separately, by
    # test_the_accel_bound_shapes_the_bank_without_abandoning_it.
    plan = _make_cup_plan(n_knots=25, accel_scale=0.05)
    cfg = cr.RealizeConfig(tilt_rate_limit_rad_s=0.0, tilt_accel_limit_rad_s2=0.0)
    tilts = cr.tilt_schedule(plan, (0.0, 0.0), (0.0, 0.0), cfg)

    g = np.array([0.0, 0.0, -hw.GRAVITY_MPS2])
    # Skip the pinned knots (catch_k and the release knot carry the pins, not the
    # banking solution).
    for k in range(len(tilts)):
        if k == plan.catch_k or k == len(tilts) - 1:
            continue
        want = g - plan.acc[k]
        want = -want / np.linalg.norm(want)
        assert np.degrees(np.hypot(*tilts[k])) < cr.RealizeConfig().max_tilt_deg
        np.testing.assert_allclose(tg.cup_axis(*tilts[k]), want, atol=1e-9)


def test_banking_level_when_cup_is_not_accelerating():
    plan = _make_cup_plan(n_knots=9, accel_scale=0.0)
    tilts = cr.tilt_schedule(plan, (0.0, 0.0), (0.0, 0.0))
    np.testing.assert_array_equal(tilts, np.zeros_like(tilts))


def test_banking_never_exceeds_the_12_degree_cap():
    """A violent cup accel saturates rather than tipping past the usable ceiling.

    Asserted on BOTH schedules the module can produce.  Unshaped, the cap must be
    *attained* — otherwise the case is too gentle and the test proves nothing.
    Shaped by the accel bound, the cap must still *hold*: the smoother is a convex
    combination of points already inside the disc and the pin blend is another, so
    the ceiling survives by convexity and never by a re-clamp.
    """
    plan = _make_cup_plan(n_knots=31, accel_scale=60.0)
    recv = tg.tilt_to_receive(np.array([0.4, -0.2, -3.0]))
    throw = tg.tilt_to_throw(np.array([0.3, 0.1, 3.2]))

    raw = cr.tilt_schedule(plan, recv, throw,
                           cr.RealizeConfig(tilt_accel_limit_rad_s2=0.0))
    raw_mags = np.degrees(np.hypot(raw[:, 0], raw[:, 1]))
    assert raw_mags.max() <= tg.MAX_TILT_DEG + 1e-12
    # ...and it really did saturate somewhere, or the test proves nothing.
    assert raw_mags.max() == pytest.approx(tg.MAX_TILT_DEG, abs=1e-9)

    tilts = cr.tilt_schedule(plan, recv, throw)
    mags = np.degrees(np.hypot(tilts[:, 0], tilts[:, 1]))
    assert mags.max() <= tg.MAX_TILT_DEG + 1e-12


def test_endpoints_pinned_exactly_to_receive_and_throw():
    plan = _make_cup_plan(n_knots=25, catch_k=10, accel_scale=40.0)
    recv = np.array(tg.tilt_to_receive(np.array([0.5, -0.3, -2.8])))
    throw = np.array(tg.tilt_to_throw(np.array([-0.2, 0.4, 3.1])))
    tilts = cr.tilt_schedule(plan, recv, throw)
    # Exact equality: these are boundary conditions of the ball-frame physics.
    np.testing.assert_array_equal(tilts[plan.catch_k], recv)
    np.testing.assert_array_equal(tilts[-1], throw)


def test_rate_limit_respected_between_free_knots():
    """The bound is on the tilt VECTOR (a 2-norm), matching the 12° ceiling's own
    from-vertical-angle semantics — see the per-axis leak the cap test caught."""
    plan = _make_cup_plan(n_knots=25, catch_k=10, accel_scale=80.0)
    cfg = cr.RealizeConfig(tilt_rate_limit_rad_s=1.5)
    recv = np.array(tg.tilt_to_receive(np.array([0.9, -0.6, -2.5])))
    throw = np.array(tg.tilt_to_throw(np.array([-0.7, 0.5, 2.9])))
    tilts = cr.tilt_schedule(plan, recv, throw, cfg)

    step = cfg.tilt_rate_limit_rad_s * plan.dt
    d = np.linalg.norm(np.diff(tilts, axis=0), axis=1)
    assert d.max() <= step + 1e-12, (
        f"tilt step {d.max():.6f} rad exceeds the {step:.6f} rad/knot limit")
    # The unshaped banking solution really does slew faster than this, or the rate
    # limit was never exercised.  BOTH bounds are off in the reference: the accel
    # bound smooths the schedule enough on its own that leaving it on would make
    # this check pass or fail on the wrong mechanism.
    free = cr.tilt_schedule(plan, recv, throw,
                            cr.RealizeConfig(tilt_rate_limit_rad_s=0.0,
                                             tilt_accel_limit_rad_s2=0.0))
    assert np.linalg.norm(np.diff(free, axis=0), axis=1).max() > step


@pytest.mark.parametrize("rate", [0.5, 1.0, 2.0, 3.0, 6.0])
@pytest.mark.parametrize("accel_scale", [5.0, 40.0, 120.0])
def test_slew_sweeps_converge_across_the_operating_grid(rate, accel_scale):
    """The rate limit is imposed by a BOUNDED number of alternating sweeps, so a
    non-converging case would silently ship a schedule that violates it.  Pin the
    budget: over the operating grid every feasible case must converge."""
    plan = _make_cup_plan(n_knots=33, catch_k=13, accel_scale=accel_scale)
    cfg = cr.RealizeConfig(tilt_rate_limit_rad_s=rate)
    recv = np.array(tg.tilt_to_receive(np.array([0.6, -0.4, -2.7])))
    throw = np.array(tg.tilt_to_throw(np.array([-0.5, 0.3, 3.0])))
    tilts = cr.tilt_schedule(plan, recv, throw, cfg)
    step = rate * plan.dt
    d = np.linalg.norm(np.diff(tilts, axis=0), axis=1)
    assert d.max() <= step + 1e-12, (
        f"slew sweeps did not converge at rate={rate}: {d.max():.6f} > {step:.6f}")
    assert np.degrees(np.linalg.norm(tilts, axis=1)).max() <= tg.MAX_TILT_DEG + 1e-12


def _tilt_accel(tilts, dt):
    """Peak 2-norm of the tilt vector's second difference, as rad/s²."""
    d2 = tilts[:-2] - 2.0 * tilts[1:-1] + tilts[2:]
    return float(np.hypot(d2[:, 0], d2[:, 1]).max()) / (dt * dt)


@pytest.mark.parametrize("accel_scale", [5.0, 40.0, 120.0])
def test_banking_tilt_acceleration_respects_the_cap(accel_scale):
    """The bound WP3's measurement made necessary, asserted by finite difference.

    Rate limiting bounds tilt VELOCITY and leaves tilt ACCELERATION free: WP3
    measured the as-built schedule slewing at exactly its 3.0 rad/s cap while its
    second difference reached 240 rad/s², which ``decompose`` turned into
    104k–115k mm/s² of leg acceleration against a 5000 mm/s² session limit.  So the
    property that matters is measured the way the machine feels it — as a second
    difference of the emitted series, not as a promise about the algorithm.

    The grid is long-cycle on purpose: the smoothing widths are bounded by the knot
    count and by the gap between the two pins, so a short cycle genuinely cannot
    hold an arbitrary cap (see :func:`cup_realize._accel_bounded_schedule`).  57
    knots is the WP3 demo cycle's length.
    """
    plan = _make_cup_plan(n_knots=57, catch_k=30, accel_scale=accel_scale)
    cfg = cr.RealizeConfig()
    recv = np.array(tg.tilt_to_receive(np.array([0.10, -0.05, -2.5])))
    throw = np.zeros(2)
    tilts = cr.tilt_schedule(plan, recv, throw, cfg)
    assert _tilt_accel(tilts, plan.dt) <= cfg.tilt_accel_limit_rad_s2 + 1e-9

    # Non-vacuity: without the bound this very case blows straight past it.
    unbounded = cr.tilt_schedule(plan, recv, throw,
                                 cr.RealizeConfig(tilt_accel_limit_rad_s2=0.0))
    assert _tilt_accel(unbounded, plan.dt) > 10.0 * cfg.tilt_accel_limit_rad_s2


def test_the_accel_bound_keeps_the_cap_the_pins_and_the_rate_limit():
    """The three properties the accel bound is not allowed to buy its smoothness with."""
    plan = _make_cup_plan(n_knots=57, catch_k=30, accel_scale=80.0)
    cfg = cr.RealizeConfig(tilt_rate_limit_rad_s=1.5)
    recv = np.array(tg.tilt_to_receive(np.array([0.9, -0.6, -2.5])))
    throw = np.array(tg.tilt_to_throw(np.array([-0.7, 0.5, 2.9])))
    tilts = cr.tilt_schedule(plan, recv, throw, cfg)

    np.testing.assert_array_equal(tilts[plan.catch_k], recv)   # pins, exactly
    np.testing.assert_array_equal(tilts[-1], throw)
    assert np.degrees(np.linalg.norm(tilts, axis=1)).max() <= tg.MAX_TILT_DEG + 1e-12
    step = cfg.tilt_rate_limit_rad_s * plan.dt
    assert np.linalg.norm(np.diff(tilts, axis=0), axis=1).max() <= step + 1e-12
    assert _tilt_accel(tilts, plan.dt) <= cfg.tilt_accel_limit_rad_s2 + 1e-9


def test_the_accel_bound_shapes_the_bank_without_abandoning_it():
    """Smoothing must not quietly degenerate into "hold level".

    The banking objective is to keep the specific force ``g − a_cup`` along the cup
    axis, so the physical score is the lateral specific force the ball feels,
    ``∫|(g − a_cup) × cup_axis| dt``.  The shaped schedule must still beat a level
    platform on it — a smoother that merely flattened the bank would pass every
    cap/pin/rate assertion above and be worthless.
    """
    plan = _make_cup_plan(n_knots=57, catch_k=30, accel_scale=40.0)
    g = np.array([0.0, 0.0, -hw.GRAVITY_MPS2])

    def lateral(tilts):
        return float(sum(
            np.linalg.norm(np.cross(g - plan.acc[k], tg.cup_axis(*tilts[k])))
            for k in range(len(tilts)))) * plan.dt

    banked = cr.tilt_schedule(plan, np.zeros(2), np.zeros(2), cr.RealizeConfig())
    level = np.zeros((len(plan.acc), 2))
    assert lateral(banked) < lateral(level)


def test_the_default_tilt_accel_cap_is_derived_from_the_leg_budget():
    """The default is config-derived, not a literal — a change to the leg limit or
    the platform radius must ripple into it instead of silently disagreeing."""
    lever = ((cr.CUP_Z_BASE_MM + cr.SLIDER_REV_ZERO_MM
              + hw.JB_OP_HAND_CATCH_PRIME_REV / hand_stroke.LINEAR_GAIN_REV_PER_M
              * 1000.0)
             - tg.CUP_TILT_CENTER_Z_MM + hw.GEOM_PLAT_RADIUS_MM)
    assert cr.TILT_ACCEL_LEVER_MM == pytest.approx(lever)
    assert cr.TILT_ACCEL_LIMIT_DEFAULT_RAD_S2 == pytest.approx(
        cr.TILT_ACCEL_BUDGET_FRACTION * hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2 / lever)
    # Sanity: the derived number must be in the band the WP4 sweep found workable
    # (5-7 rad/s² validates on the demo cycle; 10 does not).
    assert 3.0 < cr.TILT_ACCEL_LIMIT_DEFAULT_RAD_S2 < 8.0


def test_zero_banking_is_bit_identical_whatever_the_accel_cap_says():
    """The accel bound is a banking-only shaper.

    Zero banking is the legacy two-constant realisation and its bit-for-bit
    agreement with ``realize_tilted`` is the Phase-1 acceptance bar, so no value of
    the accel cap — including one far too tight to be met — may move it.
    """
    plan = _make_cup_plan(n_knots=33, catch_k=14, accel_scale=25.0)
    recv = np.array([0.02, -0.01])
    throw = np.array([0.021, -0.0105])
    base = cr.tilt_schedule(plan, recv, throw, cr.RealizeConfig(
        banking_enabled=False, tilt_accel_limit_rad_s2=0.0))
    for cap in (0.05, 1.0, cr.TILT_ACCEL_LIMIT_DEFAULT_RAD_S2, 1e6):
        shaped = cr.tilt_schedule(plan, recv, throw, cr.RealizeConfig(
            banking_enabled=False, tilt_accel_limit_rad_s2=cap))
        assert np.max(np.abs(shaped - base)) == 0.0, f"cap={cap} moved zero banking"


def test_pin_outside_the_usable_ceiling_is_refused_loudly():
    plan = _make_cup_plan(n_knots=9, catch_k=4)
    with pytest.raises(ValueError, match=r"throw_tilt .* usable ceiling"):
        cr.tilt_schedule(plan, (0.0, 0.0), (np.radians(20.0), 0.0))


def test_rate_limit_pins_win_when_the_slew_cannot_reach_them():
    """The pins are physics; the rate limit is a comfort bound.  When they collide
    the pin survives and the step into it is the one that exceeds the limit."""
    plan = _make_cup_plan(n_knots=6, catch_k=3, accel_scale=0.0)
    cfg = cr.RealizeConfig(tilt_rate_limit_rad_s=0.05)     # 0.00125 rad per knot
    recv = np.array([0.0, 0.0])
    throw = np.array([0.15, -0.10])                        # unreachable in 2 knots
    tilts = cr.tilt_schedule(plan, recv, throw, cfg)
    np.testing.assert_array_equal(tilts[3], recv)
    np.testing.assert_array_equal(tilts[-1], throw)


def test_zero_banking_is_exactly_the_constant_tilt():
    plan = _make_cup_plan(n_knots=17, catch_k=7, accel_scale=25.0)
    tilt = np.array([0.031, -0.017])
    cfg = cr.RealizeConfig(banking_enabled=False)
    tilts = cr.tilt_schedule(plan, tilt, tilt, cfg)
    np.testing.assert_array_equal(tilts, np.tile(tilt, (len(plan.pos), 1)))


def test_zero_banking_holds_receive_through_the_catch_then_the_throw():
    plan = _make_cup_plan(n_knots=13, catch_k=6, accel_scale=25.0)
    recv = np.array([0.02, -0.01])
    throw = np.array([0.021, -0.0105])          # close enough to slew in one knot
    cfg = cr.RealizeConfig(banking_enabled=False, tilt_rate_limit_rad_s=3.0)
    tilts = cr.tilt_schedule(plan, recv, throw, cfg)
    for k in range(plan.catch_k + 1):
        np.testing.assert_array_equal(tilts[k], recv)
    np.testing.assert_array_equal(tilts[-1], throw)


# ── T-U3: the parity contract vs sim/juggle_tilt.py::realize_tilted ──────────

@pytest.mark.parametrize("tilt", [
    (0.0, 0.0),
    (0.05, 0.0),
    (0.0, -0.06),
    (0.031, -0.017),
    (np.radians(11.0) * 0.6, -np.radians(11.0) * 0.8),   # near the 12° cap
])
def test_zero_banking_decompose_matches_realize_tilted_bit_for_bit(tilt):
    """THE Phase-1 acceptance bar.  Same inputs, identical outputs to the last ulp.

    ``realize_tilted`` is the parity contract for the whole realisation stage: the
    lever-arm compensation, the vertical-drop slider correction and the z pin all
    came from it.  A tolerance-based check would let the two drift by an
    ever-growing epsilon; ``max |Δ| == 0.0`` is what actually pins the port.  This
    is why ``cup_realize`` reaches for ``tilt_geometry.cup_axis`` (matrix form,
    what ``realize_tilted`` uses) rather than ``shaping.cup_lateral_shift_mm``
    (closed-form Rodrigues) — the two differ at ~8e-17.
    """
    from sim.juggle_tilt import Z_ACTIVE_MM, realize_tilted

    # The z pin's two chains must agree before parity means anything.
    assert Z_ACTIVE_MM == hw.JB_OP_DEFAULT_ACTIVE_Z_MM

    plan = _make_cup_plan(n_knots=21, catch_k=8, z_lo=0.72, z_hi=0.98)
    tilt = np.asarray(tilt, dtype=float)
    cfg = cr.RealizeConfig(banking_enabled=False)
    tilts = cr.tilt_schedule(plan, tilt, tilt, cfg)
    np.testing.assert_array_equal(tilts, np.tile(tilt, (len(plan.pos), 1)))

    out = cr.decompose(plan, tilts, cfg)

    ref_pose = np.empty_like(out.pose)
    ref_slider = np.empty(len(plan.pos))
    for k in range(len(plan.pos)):
        p, s = realize_tilted(plan.pos[k, :2], float(plan.pos[k, 2]),
                              float(tilt[0]), float(tilt[1]))
        ref_pose[k] = p
        ref_slider[k] = s

    pose_delta = float(np.max(np.abs(out.pose - ref_pose)))
    slider_delta = float(np.max(np.abs(out.slider_mm - ref_slider)))
    assert pose_delta == 0.0, f"pose parity broken: max |Δ| = {pose_delta!r} mm"
    assert slider_delta == 0.0, f"slider parity broken: max |Δ| = {slider_delta!r} mm"


def test_parity_holds_where_the_slider_clamps():
    """The stroke clamp is part of ``realize_tilted``'s contract too — a cup track
    that runs off both ends of the stroke must clamp identically."""
    from sim.juggle_tilt import realize_tilted

    plan = _make_cup_plan(n_knots=25, catch_k=10, z_lo=0.55, z_hi=1.15)
    tilt = np.array([0.04, -0.03])
    cfg = cr.RealizeConfig(banking_enabled=False)
    tilts = cr.tilt_schedule(plan, tilt, tilt, cfg)
    out = cr.decompose(plan, tilts, cfg)

    ref = np.array([realize_tilted(plan.pos[k, :2], float(plan.pos[k, 2]),
                                   float(tilt[0]), float(tilt[1]))[1]
                    for k in range(len(plan.pos))])
    assert float(np.max(np.abs(out.slider_mm - ref))) == 0.0
    assert out.slider_saturated.any(), "fixture never saturated — test is vacuous"
    assert out.slider_mm.min() == 0.0
    assert out.slider_mm.max() == pytest.approx(cfg.slider_stroke_mm)


# ── T-U4: the z pin and the z-float band ─────────────────────────────────────

def test_z_pinned_exactly_when_float_disabled():
    plan = _make_cup_plan(n_knots=25, catch_k=10, z_lo=0.55, z_hi=1.15)
    cfg = cr.RealizeConfig(z_float_enabled=False)
    tilts = cr.tilt_schedule(plan, (0.02, 0.0), (0.0, 0.03), cfg)
    out = cr.decompose(plan, tilts, cfg)
    # Exact identity, every knot, INCLUDING the knots where the slider saturates.
    assert np.all(out.pose[:, 2] == hw.JB_OP_DEFAULT_ACTIVE_Z_MM)
    assert out.slider_saturated.any(), "fixture never saturated — test is vacuous"
    np.testing.assert_array_equal(out.z_excursion_mm, np.zeros(len(plan.pos)))
    np.testing.assert_array_equal(out.pose_vel[:, 2], np.zeros(len(plan.pos)))


def test_z_floats_only_where_the_slider_saturates_and_stays_in_band():
    plan = _make_cup_plan(n_knots=25, catch_k=10, z_lo=0.55, z_hi=1.15)
    cfg = cr.RealizeConfig(z_float_enabled=True, z_band_mm=15.0)
    tilts = cr.tilt_schedule(plan, (0.02, 0.0), (0.0, 0.03), cfg)
    out = cr.decompose(plan, tilts, cfg)

    z = out.pose[:, 2]
    excursion = z - cfg.active_z_mm
    assert np.all(np.abs(excursion) <= cfg.z_band_mm + 1e-12)
    np.testing.assert_allclose(excursion, out.z_excursion_mm, atol=0.0)

    # Reference: the same decomposition with the pin in place tells us exactly
    # which knots ran out of slider.  z may only move at those knots.
    pinned = cr.decompose(plan, tilts, cr.RealizeConfig(z_float_enabled=False))
    moved = excursion != 0.0
    assert np.all(pinned.slider_saturated[moved])
    assert np.all(z[~moved] == cfg.active_z_mm)

    # Above the stroke the platform RISES (buying back stroke); below zero it drops.
    hi = pinned.slider_mm >= cfg.slider_stroke_mm
    lo = pinned.slider_mm <= 0.0
    assert np.all(excursion[hi & moved] > 0.0)
    assert np.all(excursion[lo & moved] < 0.0)


def test_z_float_absorbs_the_shortfall_it_can_and_clamps_the_rest():
    """A 15 mm band against a ~46 mm overshoot: the slider still clamps, but the
    platform has taken the full band off it."""
    plan = _make_cup_plan(n_knots=9, catch_k=4, z_lo=1.00, z_hi=1.00)
    tilts = np.zeros((len(plan.pos), 2))
    band = 15.0
    pinned = cr.decompose(plan, tilts, cr.RealizeConfig(z_float_enabled=False))
    floated = cr.decompose(plan, tilts,
                           cr.RealizeConfig(z_float_enabled=True, z_band_mm=band))
    # cup_z 1.0 m ⇒ slider demand 340.4 mm, inside the 344.75 stroke: no clamp,
    # no excursion.  Push it higher to force the shortfall.
    assert not pinned.slider_saturated.any()
    np.testing.assert_array_equal(floated.z_excursion_mm,
                                  np.zeros(len(plan.pos)))

    tall = _make_cup_plan(n_knots=9, catch_k=4, z_lo=1.05, z_hi=1.05)
    pinned_tall = cr.decompose(tall, tilts, cr.RealizeConfig(z_float_enabled=False))
    floated_tall = cr.decompose(
        tall, tilts, cr.RealizeConfig(z_float_enabled=True, z_band_mm=band))
    shortfall = (1050.0 - cr.CUP_Z_BASE_MM) - cr.RealizeConfig().slider_stroke_mm
    assert shortfall > band                       # the band cannot cover it
    assert pinned_tall.slider_saturated.all()
    np.testing.assert_allclose(floated_tall.z_excursion_mm, band, atol=0.0)
    assert np.all(floated_tall.slider_mm == cr.RealizeConfig().slider_stroke_mm)


# ── decomposition bookkeeping ────────────────────────────────────────────────

def test_slider_rev_matches_hand_stroke_mm_to_rev():
    """``slider_rev`` is ``hand_stroke.mm_to_rev`` vectorised over the offset frame
    — one conversion authority, not a second copy of the gain."""
    plan = _make_cup_plan(n_knots=13, catch_k=5)
    tilts = np.zeros((len(plan.pos), 2))
    out = cr.decompose(plan, tilts)
    want = np.array([hand_stroke.mm_to_rev(mm - cr.SLIDER_REV_ZERO_MM)
                     for mm in out.slider_mm])
    np.testing.assert_allclose(out.slider_rev, want, rtol=0.0, atol=0.0)


def test_slider_frame_lands_the_sim_prime_on_the_production_prime_rev():
    """The 20 mm sim/firmware stroke-frame divergence is honoured, not papered over:
    the sim's prime slider (20 mm inset + 315 mm stroke) must land exactly on
    ``JB_OP_HAND_CATCH_PRIME_REV``.  A dropped offset would put it 0.63 rev high."""
    prime_mm = (hw.TEENSY_TRAJ_STROKE_MARGIN_M * 1000.0
                + hw.HAND_STROKE_TOP_REV / hand_stroke.LINEAR_GAIN_REV_PER_M * 1000.0)
    rev = hand_stroke.mm_to_rev(prime_mm - cr.SLIDER_REV_ZERO_MM)
    assert rev == pytest.approx(hw.JB_OP_HAND_CATCH_PRIME_REV, abs=1e-4)
    assert rev == pytest.approx(hw.HAND_STROKE_TOP_REV, abs=1e-9)


def test_level_untilted_velocities_are_the_analytic_cup_velocities():
    """With no tilt and z pinned, the platform tracks the cup's lateral velocity
    1:1 and the slider tracks its vertical velocity 1:1 — including at the LAST
    knot, where the takeoff velocity lives and a finite difference would be wrong."""
    plan = _make_cup_plan(n_knots=17, catch_k=7)
    tilts = np.zeros((len(plan.pos), 2))
    out = cr.decompose(plan, tilts)
    assert not out.slider_saturated.any()
    np.testing.assert_allclose(out.pose_vel[:, :2], plan.vel[:, :2] * 1000.0,
                               rtol=0.0, atol=1e-9)
    np.testing.assert_allclose(out.slider_vel_rev_s,
                               plan.vel[:, 2] * 1000.0 / 1000.0
                               * hand_stroke.LINEAR_GAIN_REV_PER_M,
                               rtol=0.0, atol=1e-12)
    np.testing.assert_array_equal(out.pose_vel[:, 5], np.zeros(len(plan.pos)))


def test_knot_velocities_converge_on_the_position_series_as_the_grid_refines():
    """The velocity path is a hybrid — analytic in the cup's own velocity, finite
    differenced only in the tilt-rate correction.  A sign error in ``shift_dot`` or
    ``drop_dot`` would not show on a level fixture, so check the emitted velocities
    against a central difference of the poses the SAME call produced, on a plan
    with a genuinely time-varying tilt.

    The check is tolerance-free by construction: halve the grid over the same
    trajectory and the residual must fall ~4×.  A residual that shrinks like dt² IS
    the central difference's truncation error; a modelling disagreement (wrong sign,
    missing term) would not shrink at all.
    """
    def residuals(n_knots, dt):
        plan = _make_cup_plan(n_knots=n_knots, dt=dt, catch_k=(n_knots - 1) // 2,
                              accel_scale=8.0, z_lo=0.78, z_hi=0.92)
        # No rate limit: the schedule is then the same continuous function sampled
        # on both grids, so the comparison isolates the velocity model.
        cfg = cr.RealizeConfig(tilt_rate_limit_rad_s=0.0)
        tilts = cr.tilt_schedule(plan, (0.02, -0.01), (0.0, 0.03), cfg)
        assert np.abs(np.diff(tilts, axis=0)).max() > 1e-4, "tilt is not varying"
        out = cr.decompose(plan, tilts, cfg)
        assert not out.slider_saturated.any()
        fd_pose = (out.pose[2:] - out.pose[:-2]) / (2.0 * dt)
        fd_slider = (out.slider_rev[2:] - out.slider_rev[:-2]) / (2.0 * dt)
        return (float(np.max(np.abs(out.pose_vel[1:-1] - fd_pose))),
                float(np.max(np.abs(out.slider_vel_rev_s[1:-1] - fd_slider))))

    coarse = residuals(41, 0.0125)
    fine = residuals(81, 0.00625)
    for name, c, f in zip(('pose', 'slider'), coarse, fine):
        assert f > 0.0
        assert c / f > 3.0, (f"{name} residual {c:.4g} → {f:.4g} did not fall like "
                             "dt² — the velocity model disagrees with the poses")


def test_slider_velocity_is_zero_where_the_stroke_clamps():
    plan = _make_cup_plan(n_knots=21, catch_k=8, z_lo=0.55, z_hi=1.15)
    tilts = np.zeros((len(plan.pos), 2))
    out = cr.decompose(plan, tilts)
    assert out.slider_saturated.any()
    np.testing.assert_array_equal(out.slider_vel_rev_s[out.slider_saturated], 0.0)


def test_decompose_shapes_and_clock():
    plan = _make_cup_plan(n_knots=19, catch_k=9)
    tilts = cr.tilt_schedule(plan, (0.01, 0.0), (0.0, 0.02))
    out = cr.decompose(plan, tilts)
    n = len(plan.pos)
    assert out.pose.shape == (n, 6)
    assert out.pose_vel.shape == (n, 6)
    assert out.slider_mm.shape == out.slider_rev.shape == (n,)
    assert out.slider_vel_rev_s.shape == (n,)
    assert out.dt == plan.dt and out.catch_k == plan.catch_k
    np.testing.assert_allclose(out.t, plan.t)
    np.testing.assert_array_equal(out.pose[:, 3:5], tilts)


def test_decompose_rejects_mismatched_tilt_series():
    plan = _make_cup_plan(n_knots=11, catch_k=5)
    with pytest.raises(ValueError, match=r"tilts must be"):
        cr.decompose(plan, np.zeros((7, 2)))
