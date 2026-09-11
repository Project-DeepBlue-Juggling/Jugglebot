"""Contract C-HAND-3 — the derived throw-admission envelope.

Spec: ``ros_ws/docs/hand_throw_envelope.md``.
Enforcement point: ``jugglebot/motion/trajectory/throw_envelope.py``.

**What these tests are for.** The envelope replaced two hand-picked literals
(``FLIGHT_TIME_MIN_S = 0.55`` / ``FLIGHT_TIME_MAX_S = 1.10``) that had no
enforcement at all — nothing failed if they drifted, because there was nothing
for them to drift *from*. The whole point of a derived envelope is that every
number in it is traceable, so the tests here are of three kinds:

1. **traceability** — no bound may rest on a literal that is not a config key or
   an algebraic consequence of one;
2. **direction** — each margin and each declared plant value must be
   conservative *in the direction its own use requires*, which for the two
   declared inertias is OPPOSITE directions;
3. **the refusal** — it must name the binding bound and carry the numbers,
   because a bare "too high" routes the operator to the wrong knob.

The numeric expectations are quoted at the shipped config. Where a number is a
*measurement* rather than a consequence (the coast ladder), the test says so and
pins the consequence, not the measurement.
"""

from __future__ import annotations

import math

import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.trajectory import throw_envelope as te


# ══════════════════════════════════════════════════════════════════════════
#  1. the stroke landmarks the whole contract hangs off
# ══════════════════════════════════════════════════════════════════════════

def test_stroke_landmarks_are_velocity_independent():
    """``x2`` and ``x3`` carry no ``v`` — algebraically, ``x1 = accelSt/(1+IR)``
    and ``v*t_vel = velHold``, so ``x2`` has no ``v``; ``x3 = totalStroke``
    identically.  Hence ``d_dec = x3 - x2`` is velocity-independent, which is
    exactly why a fractional tracking shortfall converts straight into
    end-stop travel at every speed.

    R1 (2026-09-11): both are read from the generated constants directly
    (``HAND_THROW_POS_M``, ``HAND_STROKE_TOP_REV``) rather than from
    ``hand_stroke.HandStrokeModel`` (deleted with the stroke engine); the
    values are otherwise unchanged in kind, only the rev conversion now uses
    the measured ``HAND_REV_PER_M`` in place of the retired fudge-factor gain.
    """
    assert te.RELEASE_POS_REV == pytest.approx(
        float(hw.HAND_THROW_POS_M) * float(hw.HAND_REV_PER_M), rel=1e-12)
    assert te.STROKE_TOP_REV == pytest.approx(hw.HAND_STROKE_TOP_REV, rel=1e-12)
    assert te.DECEL_ALLOWANCE_REV == pytest.approx(3.929, abs=5e-4)
    assert te.RELEASE_POS_REV == pytest.approx(5.7434, abs=5e-4)


def test_the_decel_allowance_is_the_stopping_distance_identity():
    """``v_rev^2 / (2*a_cmd(v)) == d_dec`` exactly, at every speed.

    This identity is why ``peak = x2 + v_rev^2/(2*a_ach)`` and
    ``over = d_dec*(a_cmd/a_ach - 1)`` are the SAME statement, and therefore why
    the coast ladder (measured as ``over``) and the achieved-deceleration
    numbers quoted in C-HAND-2 are two readings of one measurement rather than
    two independent claims. If it ever fails, the coast model and the authority
    model have stopped describing the same profile.

    R1: tolerance loosened to 1e-5 (was 1e-12) — HAND_THROW_POS_M is rounded
    to 6 dp in generate_config.py, and that rounding noise now compounds
    through the corrected HAND_REV_PER_M into a ~1e-5 relative residual.
    """
    for v in (0.5, 2.742, 3.969, 4.858, 6.0):
        v_rev = v * float(hw.HAND_REV_PER_M)
        assert (v_rev ** 2 / (2.0 * te.commanded_decel_rps2(v))
                == pytest.approx(te.DECEL_ALLOWANCE_REV, rel=1e-5)), v


def test_commanded_ramps_match_the_shipped_stroke_model():
    """The envelope reads ``a_cmd`` off its own closed-form
    ``_throw_accel_mps2`` (the algebra ``hand_stroke.HandStrokeModel`` used to
    wrap, kept host-side at R1), never a re-derived coefficient chosen by this
    test. Pinned against an INDEPENDENTLY recomputed coefficient anyway, so a
    change to the stroke constants or the measured gain shows up here as the
    envelope shift it is. R1: 119.99, not the pre-correction 123.55 — the
    ~3 % shift is exactly ``HAND_REV_PER_M``'s correction (30.706 vs 31.628
    rev/m)."""
    ir = float(hw.TEENSY_TRAJ_INERTIA_RATIO)
    accel_st = te._THROW_ACCEL_STROKE_M
    coeff = (ir + 1.0) / (2.0 * accel_st * ir) * float(hw.HAND_REV_PER_M)
    assert coeff == pytest.approx(119.99, abs=0.05)
    for v in (2.742, 3.969, 4.858):
        assert te.commanded_decel_rps2(v) == pytest.approx(coeff * v * v,
                                                           rel=1e-9)
        assert te.commanded_accel_rps2(v) == pytest.approx(
            hw.TEENSY_TRAJ_INERTIA_RATIO * te.commanded_decel_rps2(v), rel=1e-9)


# ══════════════════════════════════════════════════════════════════════════
#  2. traceability — every bound is made of config
# ══════════════════════════════════════════════════════════════════════════

def test_every_declared_input_comes_from_generated_config():
    """No bound may rest on a module literal.

    Every number here comes from ``hardware_config`` — R1 (2026-09-11) deletes
    the one exception (``MIN_EVENT_DELAY_S``, a local copy of
    ``catch_coordinator_node``'s floor) along with ``arm_window_s``, its only
    consumer.
    """
    assert te.HARD_STOP_REV == hw.GEOM_HAND_MOTOR_HARD_STOP_REVS
    assert te.END_STOP_MARGIN_REV == hw.HAND_ENV_END_STOP_MARGIN_REV
    assert te.KT_NM_PER_A == hw.HAND_ENV_HAND_TORQUE_CONSTANT_NM_PER_A
    assert te.CURRENT_LIMIT_A == hw.ODRIVE_HAND_CURR_LIMIT_A
    assert te.TORQUE_SOFT_LIMIT_NM == hw.HAND_ENV_HAND_TORQUE_SOFT_LIMIT_NM
    assert te.J_MEASURED_KGM2 == hw.HAND_ENV_MEASURED_REFLECTED_INERTIA_KGM2
    assert te.REGEN_POWER_W == pytest.approx(
        hw.HAND_ENV_REGEN_CURRENT_LIMIT_A * hw.HAND_ENV_DC_BUS_NOMINAL_V)
    assert te.GRAVITY_TORQUE_NM == pytest.approx(
        hw.HAND_ENV_GRAVITY_HOLD_CURRENT_A
        * hw.HAND_ENV_HAND_TORQUE_CONSTANT_NM_PER_A)
    assert te.PEAK_LIMIT_REV == pytest.approx(
        hw.GEOM_HAND_MOTOR_HARD_STOP_REVS - hw.HAND_ENV_END_STOP_MARGIN_REV)


def test_the_host_coast_margin_is_never_looser_than_the_firmware_clamp():
    """A RELATION, not an equality — and the difference matters.

    The two guard different things: the firmware's
    ``smooth_move_excursion_margin_rev`` bounds a COMPUTED smooth-move excursion
    the Teensy checks exactly, while this one bounds a STOCHASTIC ballistic
    overshoot nothing checks in flight. So the host margin must be at least as
    large, and may be larger.

    **Asserting equality would have been a trap.** Both the YAML and the
    contract say raising the host margin is the safe direction and needs no
    evidence; an equality pin makes that edit go RED, and the obvious way to
    green it — raising the firmware key too — changes
    ``Trajectory.h``'s ``SMOOTH_MOVE_POS_CEIL_REV`` and therefore needs a FLASH,
    silently tightening an unrelated guard. Their coincidence at 0.2 today is
    also not a design: `3760daa` moved the firmware key 0.5 -> 0.2 for the sole
    purpose of holding the runbook's 10.60 rev line fixed when the stop moved
    11.1 -> 10.8.
    """
    assert (hw.HAND_ENV_END_STOP_MARGIN_REV
            >= hw.TEENSY_TRAJ_SMOOTH_MOVE_EXCURSION_MARGIN_REV), (
        'the stochastic coast margin may never be tighter than the firmware\'s '
        'deterministic excursion clamp')
    assert te.PEAK_LIMIT_REV <= (hw.GEOM_HAND_MOTOR_HARD_STOP_REVS
                                 - hw.TEENSY_TRAJ_SMOOTH_MOVE_EXCURSION_MARGIN_REV)
    # FW 18 (2026-09-08): the stop moved 10.8 -> 10.701 rev (operator
    # re-measurement); the margin did not, so the ceiling moved with it,
    # 10.60 -> 10.501. Absolute pin, not the derivation restated as a
    # tautology (a bare A == A check against the same hw fields would not
    # catch a regression in either constant).
    assert te.PEAK_LIMIT_REV == pytest.approx(10.501, abs=1e-9)


def test_the_declared_inertias_bracket_the_measurement_in_opposite_directions():
    """The SAME physical quantity is declared twice, at two values, on purpose.

    * ``teensy_trajectory.throw_decel_reflected_inertia_kgm2`` (9.5e-6) sizes a
      FEEDFORWARD, where under-declaring means under-braking, so it must sit
      BELOW the measurement (C-HAND-2's one-sided-safety clause).
    * ``hand_throw_envelope.measured_reflected_inertia_kgm2`` (1.050e-5) sizes an
      AUTHORITY CEILING ``a_max = I*Kt/(J*2*pi)``, where over-declaring means
      less credited authority, so it must sit AT OR ABOVE the measurement.

    Each is conservative for its own use and neither is conservative for the
    other's, which is the whole reason they are two keys. A future edit that
    "tidies" them into one would silently make one of the two uses unsafe, and
    this test is what stops it.
    """
    assert hw.TEENSY_TRAJ_THROW_DECEL_REFLECTED_INERTIA_KGM2 < 1.0126e-5, (
        'the firmware feedforward inertia must stay under the decel-side '
        'torque-balance bound, or the feedforward alone can over-brake')
    assert hw.HAND_ENV_MEASURED_REFLECTED_INERTIA_KGM2 >= 1.0126e-5, (
        'the envelope inertia must stay at or above the decel-side bound, or '
        'the authority ceiling credits torque the axis cannot deliver')
    assert (hw.TEENSY_TRAJ_THROW_DECEL_REFLECTED_INERTIA_KGM2
            < hw.HAND_ENV_MEASURED_REFLECTED_INERTIA_KGM2)


# ══════════════════════════════════════════════════════════════════════════
#  3. the coast ladder
# ══════════════════════════════════════════════════════════════════════════

def test_the_coast_ladder_is_the_post_fix_measurement():
    """The ladder IS the measurement on the FLASHED plant, not a model of it.

    Per-speed MAXIMA from bags ``2026-08-20_21-51-39`` (n = 2 / 2 / 2) and
    ``2026-08-18_18-42-19`` (n = 12 more at 4.436), re-derivable with
    ``tools/probes/hand_stroke_timeline.py --bag <dir> --json``.

    It supersedes the 2026-07-27 pre-fix ladder (0.074 / 0.063 / 0.347 / 1.020
    at 2.742 / 3.440 / 3.969 / 4.858 m/s), which was captured with the legacy
    decel feedforward and the hand ODrive's −10.00 A ``torque_soft_min`` live.
    **Do not mix rungs from the two ladders** — the pin below is what stops it.
    """
    raw = {round(v, 3): c for v, c in hw.HAND_ENV_MEASURED_COAST_REV}
    assert raw == {3.142: 0.1342, 3.714: 0.2119, 4.436: 0.226}
    superseded = {2.742: 0.074, 3.44: 0.063, 3.969: 0.347, 4.858: 1.02}
    assert not (set(raw) & set(superseded)), 'pre-fix rung readmitted'
    assert max(raw.values()) < min(v for v in superseded.values() if v > 0.3), (
        'every post-fix rung must sit below the pre-fix rungs that set the old '
        'envelope — if not, the clamp fix has regressed')


def test_the_end_stop_margin_covers_the_one_sided_scatter():
    """The ladder is a CENTRAL estimate, so the margin has to pay for scatter.

    Each rung is a per-tier group MEAN (``hand_decel_authority.py``'s
    ``_SELF_CHECK_PRE_OVER``, labelled exactly that), so ``peak_rev`` is the
    peak of an average throw and about half the throws at any speed land above
    it. The one-sided budget the margin must cover, at the tier that anchors the
    top of the ladder:

    * **within-tier upper deviation 0.083 rev** — 1.103 measured max against the
      1.0204 mean over five throws (C-HAND-2 § *Why a feedforward correction*);
    * **aliasing 0.029 rev** — hand telemetry is a ~100 Hz sample of a 500 Hz
      stream, so a measured apex under-reports by up to
      ``0.5 * a_ach * (5 ms)^2`` = 0.029 rev at the top tier.

    Since 2026-08-20 the rungs are per-speed **MAXIMA**, so the within-tier term
    (0.083 rev under the old mean-based ladder) is gone and only aliasing is
    left: **0.171 rev = 5.4 mm** of residual headroom, up from 2.8 mm.
    """
    aliasing_rev = 0.5 * te.TOP_RUNG_ACHIEVED_DECEL_RPS2 * (1.0 / 100.0 / 2.0) ** 2
    assert aliasing_rev == pytest.approx(0.029, abs=2e-3)
    assert te.END_STOP_MARGIN_REV - aliasing_rev == pytest.approx(0.171, abs=3e-3)
    assert te.END_STOP_MARGIN_REV > aliasing_rev, (
        f'end_stop_margin_rev {te.END_STOP_MARGIN_REV} no longer covers even '
        f'the aliasing budget {aliasing_rev:.3f} rev')


def test_the_coast_model_is_monotone_and_continuous():
    """Coast never decreases with speed, and the model has no step.

    The measured ladder is NOT monotone (0.074 at 2.742 m/s vs 0.063 at
    3.440 m/s), which is scatter on a physically non-decreasing quantity; the
    model monotonises with a RUNNING MAXIMUM rather than sorting or fitting,
    because the conservative reading of scatter is "the worse of the two applies
    from here up". A step at the top-rung join would be a different failure: the
    extrapolation branch must MEET the ladder, not restart from it.
    """
    vs = [0.31 + 0.01 * i for i in range(660)]
    coasts = [te.coast_rev(v) for v in vs]
    for a, b in zip(coasts, coasts[1:]):
        assert b >= a - 1e-12
    top_v = hw.HAND_ENV_MEASURED_COAST_REV[-1][0]
    assert te.coast_rev(top_v - 1e-9) == pytest.approx(te.coast_rev(top_v),
                                                       abs=1e-6)
    assert te.coast_rev(top_v) == pytest.approx(
        hw.HAND_ENV_MEASURED_COAST_REV[-1][1], abs=1e-9)


def test_above_the_ladder_the_model_is_a_power_law_at_the_pessimistic_exponent():
    """``coast_top*(v/v_top)^p`` with p = 2.0, and why p = 2.0.

    Fitted exponents on this data (computed 2026-08-20): log-log OLS on
    per-speed maxima **1.50**, on means 1.31, on all 18 individual points 1.12;
    minimax best 1.4 / 1.2 / 0.8. **2.0 is above every one of them**, and it is
    also the ballistic exponent — what coast would do if the tracking shortfall
    were a fixed fraction of a ``v^2`` commanded decel.

    It is affordable precisely because it is not load-bearing: at p = 2.0
    END_STOP first binds at **7.468 m/s**, above every other bound, so the
    ceiling is unchanged. Raising p is the safe direction (p = 2.5 → 6.73 m/s,
    still non-binding).
    """
    top_v, top_coast = hw.HAND_ENV_MEASURED_COAST_REV[-1]
    assert te.COAST_EXPONENT == pytest.approx(2.0)
    for v in (5.0, 5.8, 6.5, 7.0):
        assert te.coast_rev(v) == pytest.approx(
            top_coast * (v / top_v) ** te.COAST_EXPONENT, rel=1e-12), v
    # The old "hold achieved decel constant" law was the AUTHORITY-SATURATED
    # shape. It fitted the pre-fix plant and is wrong for this one — it would
    # predict 3.3 rev of coast at the ceiling against the fitted family's 0.39.
    saturated = ((5.8155 * float(hw.HAND_REV_PER_M)) ** 2
                 / (2.0 * te.TOP_RUNG_ACHIEVED_DECEL_RPS2)
                 - te.DECEL_ALLOWANCE_REV)
    assert saturated > 8 * te.coast_rev(5.8155)


def test_the_end_stop_claim_survives_a_strict_upper_envelope():
    """Robustness of "END_STOP does not bind", against the harshest member of
    the family: a single ``C*v^2`` curve forced at or above EVERY measured
    point (C = 0.015362, set by the 3.714 m/s outlier rather than the top rung).

    That curve binds at 7.344 m/s (R1, 2026-09-11: STROKE_TOP_REV is now
    generated with the measured ``HAND_REV_PER_M``, which widened the margin
    over ``DECEL_AUTHORITY`` here). So the conclusion does not depend on the
    anchoring choice, which is what makes the extrapolation caveat tolerable.
    """
    pts = [(3.142, 0.1342), (3.142, 0.1315), (3.714, 0.1803), (3.714, 0.2119),
           (4.436, 0.226), (4.436, 0.1796)]
    C = max(c / v ** 2.0 for v, c in pts)
    assert C == pytest.approx(0.015362, rel=1e-3)
    strict_bound = ((te.PEAK_LIMIT_REV - te.STROKE_TOP_REV) / C) ** 0.5
    assert strict_bound == pytest.approx(7.344, abs=5e-3)
    assert strict_bound > te.MAX_RELEASE_SPEED_MPS


def test_a_malformed_ladder_refuses_at_import_rather_than_at_the_throw():
    """Fail closed, LOUDLY, and at node start.

    An empty or malformed ladder means the envelope has no coast model. Silently
    admitting everything is the one behaviour that must be impossible, and a
    refusal at the first throw would surface with a ball in the cup.
    """
    import importlib
    original = hw.HAND_ENV_MEASURED_COAST_REV
    try:
        for bad in ([], None, [[1.0]], [[float('nan'), 0.1]], [[-1.0, 0.1]],
                    [[3.0, -0.5]], [[3.0, 0.1], [2.0, 0.2]]):
            hw.HAND_ENV_MEASURED_COAST_REV = bad
            # Drive the IMPORT, not just the loader — the claim is that a bad
            # regenerate takes the node down at start, and only reload proves it.
            with pytest.raises(ValueError):
                importlib.reload(te)
    finally:
        hw.HAND_ENV_MEASURED_COAST_REV = original
        importlib.reload(te)
    # The reloaded module must be usable again, or every later test in this
    # worker inherits a half-initialised one.
    assert te.evaluate(0.80, te.vertical_release_speed_mps(0.80)).ok


def test_a_non_positive_margin_refuses_at_import():
    """The margins LOOSEN the envelope as they shrink, so zero/negative is not a
    degenerate case that refuses everything — it is one that admits more.
    That asymmetry is why they are validated rather than trusted."""
    import importlib
    for key in ('HAND_ENV_END_STOP_MARGIN_REV',):
        original = getattr(hw, key)
        try:
            for bad in (0.0, -0.1, float('nan')):
                setattr(hw, key, bad)
                with pytest.raises(ValueError):
                    importlib.reload(te)
        finally:
            setattr(hw, key, original)
            importlib.reload(te)
    assert te.evaluate(0.80, te.vertical_release_speed_mps(0.80)).ok


# ══════════════════════════════════════════════════════════════════════════
#  4. the bounds, individually
# ══════════════════════════════════════════════════════════════════════════

def test_the_torque_bounds_bind_and_end_stop_does_not():
    """**The headline, unchanged since 2026-08-20**: the TORQUE bounds bind
    before END_STOP, so the machine's throw height is limited by what the
    drive can brake with, not by a metal collision.

    R1 (2026-09-11): every edge shifted with the measured hand gain
    (``HAND_REV_PER_M`` 30.706 vs the retired fudge-factor gain's 31.628
    rev/m) and with ``STROKE_TOP_REV`` now generated from it — END_STOP now
    binds at 8.494 m/s (was 6.867), well clear of every torque bound, because
    the corrected gain widened ``DECEL_ALLOWANCE_REV`` (3.929 vs 4.046 rev).
    The ORDER — the contract's whole claim — is unchanged.

    The order is pinned because it is the contract's whole claim. If END_STOP
    ever re-enters the front, the plant has regressed toward its pre-fix
    behaviour and the ladder must be re-measured before anything else is done.
    """
    ir = float(hw.TEENSY_TRAJ_INERTIA_RATIO)
    coeff = (ir + 1.0) / (2.0 * te._THROW_ACCEL_STROKE_M * ir) * float(hw.HAND_REV_PER_M)
    v_end = ((te.PEAK_LIMIT_REV - te.STROKE_TOP_REV) / te._TOP_COAST) ** (
        1.0 / te.COAST_EXPONENT) * te._TOP_V
    v_decel = math.sqrt(te.DECEL_AUTHORITY_RPS2 / coeff)
    v_accel = math.sqrt(te.ACCEL_AUTHORITY_RPS2
                        / (coeff * hw.TEENSY_TRAJ_INERTIA_RATIO))
    v_regen = te.REGEN_POWER_W / te.BRAKING_TORQUE_LIMIT_NM / (
        2.0 * math.pi * float(hw.HAND_REV_PER_M))
    assert v_decel == pytest.approx(5.901, abs=5e-3)
    assert v_accel == pytest.approx(5.999, abs=5e-3)
    assert v_regen == pytest.approx(6.769, abs=5e-3)
    assert v_end == pytest.approx(8.494, abs=5e-3)
    v_ff = math.sqrt(te.DECEL_FF_HEADROOM_RPS2 / coeff)
    assert v_ff == pytest.approx(5.720, abs=5e-3)
    assert v_ff < v_decel < v_accel < v_regen < v_end, (
        f'the binding order moved: FF {v_ff:.3f} / DECEL {v_decel:.3f} / '
        f'ACCEL {v_accel:.3f} / REGEN {v_regen:.3f} / END_STOP {v_end:.3f} m/s '
        f'— re-read C-HAND-3 § The bounds before shipping this')
    assert te.MAX_RELEASE_SPEED_MPS == pytest.approx(v_ff, abs=5e-3)


def test_regen_fences_the_burst_not_the_steady_state():
    """The rail's 300 W is STEADY-STATE; the fence is the drive's 360 W burst.

    Braking is a burst — a 50–90 ms decel ramp, once per CYCLE — so fencing the
    instantaneous peak against a steady-state number would tighten the envelope
    on a duty the machine never runs. The owner confirmed 360 W bursts are
    within the rail (2026-08-20), so ``dc_max_negative_current = −8.0 A`` at
    45 V is a design point, not a misconfiguration.

    ⚠ The duty cycle MOVED on 2026-08-22. This docstring cited "a 3.5 s
    ``MIN_TOSS_THROW_DELAY_S`` cadence floor, ~2 % duty"; that floor is retired
    (census A1) and the ladder's operating point is a 0.985 s cycle period, so
    the duty is 5.1–9.2 % and the margin is ~9–16x rather than ~60x. The
    inequality this test pins is unchanged, but the headroom behind it is not,
    and the arithmetic is asserted below so a further cadence rung has to come
    back here.
    """
    decel_ramp_s, cycle_period_s = 0.090, 0.985
    duty = decel_ramp_s / cycle_period_s
    assert duty < 0.10
    assert te.REGEN_POWER_W * duty < te.REGEN_RAIL_STEADY_W
    assert te.REGEN_POWER_W == pytest.approx(8.0 * 45.0)
    assert te.REGEN_RAIL_STEADY_W == pytest.approx(300.0)
    assert te.REGEN_POWER_W > te.REGEN_RAIL_STEADY_W
    # The steady-state check the duty cycle actually has to clear, at the
    # ceiling: peak burst x duty, against the rail's continuous rating.
    peak_w = (te.BRAKING_TORQUE_LIMIT_NM * te.MAX_RELEASE_SPEED_MPS
              * float(hw.HAND_REV_PER_M) * 2.0 * math.pi)
    ir = float(hw.TEENSY_TRAJ_INERTIA_RATIO)
    t_dec_s = (2.0 * te._THROW_ACCEL_STROKE_M * ir
              / ((ir + 1.0) * te.MAX_RELEASE_SPEED_MPS))
    duty = t_dec_s / 3.5
    assert peak_w * duty < 0.1 * te.REGEN_RAIL_STEADY_W, (
        f'{peak_w:.0f} W x {duty:.3%} duty is no longer negligible against the '
        f'{te.REGEN_RAIL_STEADY_W:.0f} W steady-state rail')


def test_the_current_limit_binds_not_the_torque_soft_limit():
    """Which of the two torque fences is live — asserted, not assumed.

    ``torque_soft_min`` was ``-0.055133 Nm`` (exactly -10.00 A) until
    2026-08-18, i.e. 5x TIGHTER than the current limit, and that asymmetry is
    the suspected reason the 2026-07-27 braking looks truncated. If it ever
    regresses, this test says so before the envelope silently halves.
    """
    assert te.BRAKING_TORQUE_LIMIT_NM == pytest.approx(
        te.CURRENT_LIMIT_A * te.KT_NM_PER_A, rel=1e-12)
    assert te.CURRENT_LIMIT_A * te.KT_NM_PER_A < te.TORQUE_SOFT_LIMIT_NM


def test_the_ascent_never_binds_before_the_decel_at_the_top_of_the_band():
    """Proven, not assumed — and the crossing is stated.

    Ascent torque is ``IR * (J_ascent/J)`` of the decel torque at the
    same speed, plus a constant gravity term, so the two demands cross at
    some speed: BELOW that the ascent asks for more (but both are far inside
    the limit), ABOVE it the decel does. Since every bound that matters lives
    above the crossing, the decel side is always the one that binds at the top.

    R1 (2026-09-11): the ratio and crossing shifted with the measured
    ``HAND_REV_PER_M`` (``J_ASCENT_KGM2``'s ball term scales as
    ``1/gain**2``).
    """
    ratio = (hw.TEENSY_TRAJ_INERTIA_RATIO * te.J_ASCENT_KGM2 / te.J_MEASURED_KGM2)
    assert ratio == pytest.approx(0.929, abs=2e-3)
    ir = float(hw.TEENSY_TRAJ_INERTIA_RATIO)
    coeff = (ir + 1.0) / (2.0 * te._THROW_ACCEL_STROKE_M * ir) * float(hw.HAND_REV_PER_M)
    crossing = math.sqrt(te.ASCENT_GRAVITY_TORQUE_NM
                         / ((1.0 - ratio) * te.J_MEASURED_KGM2 * 2.0 * math.pi
                            * coeff))
    assert crossing == pytest.approx(4.435, abs=0.05)
    for v in (4.6, 5.0, 5.5, 5.8):
        decel_frac = te.commanded_decel_rps2(v) / te.DECEL_AUTHORITY_RPS2
        accel_frac = te.commanded_accel_rps2(v) / te.ACCEL_AUTHORITY_RPS2
        assert decel_frac > accel_frac, v


def test_the_ball_mass_comes_from_the_inertia_ratio_identity():
    """``INERTIA_RATIO = m_hand/(m_hand + m_ball)`` — the identity that makes
    ``throwD = -throwA/IR`` a constant-motor-torque design. So the ball mass is
    DERIVED, not a fifth declared number, and C-HAND-2's 0.0952 kg falls out."""
    assert te.BALL_MASS_KG == pytest.approx(0.0952, abs=5e-4)
    # R1: scales as 1/HAND_REV_PER_M**2, so the corrected (smaller) gain
    # raises this from the pre-correction 2.412e-6.
    assert te.J_ASCENT_KGM2 - te.J_MEASURED_KGM2 == pytest.approx(2.557e-6,
                                                                  rel=5e-3)


def test_the_flight_time_inverse_takes_the_physical_root():
    """``vz(T) = dz/T + g*T/2`` has a MINIMUM (at T = 37 ms), so every reachable
    speed has two roots and only the large one is a toss. The inverse takes it.

    The small root is not merely unphysical, it is absurd — 1.4 ms of flight for
    a 4.36 m/s release — so a naive ``(-b - sqrt(disc))`` would report the
    envelope ceiling as a sub-millisecond flight and every band reported off it
    would be nonsense.
    """
    for t in (0.50, 0.80, 0.887, 1.10):
        v = te.vertical_release_speed_mps(t)
        assert te.flight_time_for_vertical_speed_s(v) == pytest.approx(t,
                                                                      rel=1e-9)
    # Below the curve's minimum there is no real toss at all — nan, not a guess.
    v_min = 2.0 * math.sqrt(te.RELEASE_TO_CUP_MM * te.GRAVITY_MMS2 / 2.0) / 1000.0
    assert math.isnan(te.flight_time_for_vertical_speed_s(0.5 * v_min))


# ══════════════════════════════════════════════════════════════════════════
#  5. the envelope as shipped
# ══════════════════════════════════════════════════════════════════════════

def test_the_derived_band_and_what_it_replaced():
    """The shipped envelope, and the direction it moved.

    R1 (2026-09-11): the CEILING is torque-bound as before, now 1.165 s (was
    1.1485) with the corrected gain. The FLOOR is a DIFFERENT MECHANISM now:
    ARM_WINDOW (the reactive-catch timing bound that used to set it, closing
    at 0.4542 s) is deleted with the reactive catch it modelled — R1 leaves no
    replacement floor bound, so ``_flight_ok`` is the four SPEED bounds alone,
    and for this machine's near-zero ``RELEASE_TO_CUP_MM`` (6.7 mm — release
    and catch sit almost at the same hand height) those never rule out an
    arbitrarily short "flight": the floor now sits AT the search bracket's own
    ``_SEARCH_LO_S``. This is a KNOWN GAP, not a hardware-safety one — every
    machine-damage bound (END_STOP/DECEL_AUTHORITY/DECEL_FF_HEADROOM/
    ACCEL_AUTHORITY/REGEN/WIRE_BAND) still gates real speeds — but a dispatch
    quality floor is gone. Cross-unit: toss_ilc.py, sim/toss_gate.py and
    tools/probes/ilc_speed_band.py still read this Tier-8a band; whichever
    unit owns them this rung should confirm whether it is being retired
    alongside its legacy FSM callers or needs a new floor.
    """
    lo, hi = te.flight_time_band_s()
    assert lo == pytest.approx(te._SEARCH_LO_S, abs=1e-9)
    assert hi == pytest.approx(1.165, abs=1e-3)
    apex_lo, apex_hi = te.apex_height_band_m()
    assert apex_lo == pytest.approx(0.00306, abs=2e-4)
    assert apex_hi == pytest.approx(1.665, abs=3e-3)
    assert hi > 1.10


def test_the_clamp_was_the_coast_mechanism():
    """The within-session A/B, as an executable number.

    Bag ``2026-08-18_18-42-19`` changes the hand ODrive's braking clamp
    mid-session: braking ``iq_meas`` never passes −8.87 A through t = 0–100 s,
    then reaches −13.7 → −17.4 A from t = 100 s. Its first throw (t = 84.9 s,
    clamped) coasted **+0.763 rev**; every other throw in the same bag, at the
    IDENTICAL commanded 4.436 m/s, coasts 0.18–0.23. Same command, same
    firmware, **3.8×** the coast.

    That is why the pre-fix ladder — on which a 1.10 s toss modelled to 12.17
    rev against a 10.8 rev stop — does not describe this plant, and it is the
    evidence the 2026-08-18 draft of this contract did not have.
    """
    clamped_coast, unclamped_max = 0.7628, 0.226
    assert clamped_coast / unclamped_max == pytest.approx(3.38, abs=0.05)
    assert te.coast_rev(4.436) == pytest.approx(unclamped_max, abs=1e-3)
    # On the pre-fix ladder, 1.10 s modelled past metal. On this one it does not.
    v = te.vertical_release_speed_mps(1.10)
    assert v == pytest.approx(5.399, abs=2e-3)
    assert te.peak_rev(v) == pytest.approx(10.007, abs=5e-3)
    assert te.peak_rev(v) < te.PEAK_LIMIT_REV
    # R1 (2026-09-11): STROKE_TOP_REV is now generated with the measured
    # HAND_REV_PER_M instead of the retired fudge-factor gain, which widened
    # the modelled clearance to metal at this speed.
    assert ((te.HARD_STOP_REV - te.peak_rev(v)) / float(hw.HAND_REV_PER_M)
            * 1000.0) == pytest.approx(22.6, abs=0.5)


def test_the_shipped_working_point_is_now_admitted():
    """The 1.0 m working point the machine actually flies, and its real margin.

    The 2026-08-18 draft REFUSED this — the pre-fix ladder put its peak at
    10.660 rev, 4.4 mm from metal. Measured on the flashed plant it peaks at
    **10.185 rev, 16.3 mm from metal** (FW 18, 2026-09-08: was 19.4 mm before
    the metal moved 10.8 -> 10.701 rev), and it is admitted with room. n = 14
    at exactly this speed, so it is the best-characterised point in the model.
    """
    v = te.vertical_release_speed_mps(0.9032)
    assert v == pytest.approx(4.4360, abs=1e-3)
    assert te.evaluate(0.9032, v).ok
    assert te.peak_rev(v) == pytest.approx(9.898, abs=5e-3)
    assert ((te.HARD_STOP_REV - te.peak_rev(v)) / float(hw.HAND_REV_PER_M)
            * 1000.0) == pytest.approx(26.1, abs=0.3)


def test_the_default_toss_is_admitted():
    """The shipped default (``toss_flight_time_default_s``) must be inside the
    envelope, or every zero-height goal is refused and the machine is bricked.

    Deliberately an assertion about the CONFIG default, not about 0.80: if an
    operator retunes the default, this is where the two meet.
    """
    t = hw.JB_OP_TOSS_FLIGHT_TIME_DEFAULT_S
    verdict = te.evaluate(t, te.vertical_release_speed_mps(t))
    assert verdict.ok, (
        f'the shipped default toss ({t} s) is REFUSED by the envelope: '
        f'{verdict.message}')


def test_the_band_does_not_touch_its_own_search_bracket():
    """The CEILING must not reach the search bracket (a configuration error
    reported as a physical answer). The FLOOR, since R1, legitimately DOES
    reach ``_SEARCH_LO_S`` for this machine — see
    ``test_the_derived_band_and_what_it_replaced`` for why (ARM_WINDOW,
    the mechanism that used to set the floor, is deleted with no
    replacement)."""
    assert te._SEARCH_LO_S <= te.MIN_FLIGHT_TIME_S
    assert te.MAX_FLIGHT_TIME_S < te._SEARCH_HI_S


def test_reported_band_edges_agree_with_evaluate():
    """The reported edges are computed by a SECOND predicate (``_flight_ok``,
    message-free, so that ``evaluate`` can quote the edges without recursing).
    Two expressions of one rule is a drift risk, so they are pinned equal —
    including on which SIDE of the crossing each edge lands, because an edge a
    float epsilon into the refused half breaks every consumer that samples the
    band at its own edge (the sim sweep does exactly that).
    R1 (2026-09-11): the LOWER edge is no longer a real bound crossing (see
    ``test_the_derived_band_and_what_it_replaced`` — ARM_WINDOW, which used to
    set it, is deleted with no replacement, so ``lo`` now sits exactly at the
    search floor); only the ceiling side is checked for a genuine crossing.
    """
    lo, hi = te.flight_time_band_s()
    assert te.evaluate(lo, te.vertical_release_speed_mps(lo)).ok
    assert te.evaluate(hi, te.vertical_release_speed_mps(hi)).ok
    assert not te.evaluate(hi + 1e-4,
                           te.vertical_release_speed_mps(hi + 1e-4)).ok
    assert te.evaluate(0.80, te.MAX_RELEASE_SPEED_MPS).ok
    assert not te.evaluate(0.80, te.MAX_RELEASE_SPEED_MPS + 1e-4).ok


def test_the_speed_gate_is_independent_of_the_flight_time():
    """Tier 8b AIMS, so its release speed is NOT the vertical inverse of its
    flight time — the gate must bound the COMMANDED speed, not a speed
    re-derived from T. A T-only band would silently under-bound every displaced
    throw, which is the failure this signature exists to prevent."""
    t = 0.80                      # comfortably inside the band on its own
    assert te.evaluate(t, te.vertical_release_speed_mps(t)).ok
    faster = te.evaluate(t, 5.85)  # an 8b aim that releases far harder
    assert not faster.ok and faster.bound == 'DECEL_FF_HEADROOM'


# ══════════════════════════════════════════════════════════════════════════
#  6. the refusal
# ══════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize('flight_s, speed, bound', [
    (0.80, 8.6, 'END_STOP'),    # only reachable above the wire band; bounds are
                                #   ordered machine-damage-first so it still wins
    (0.80, 5.80, 'DECEL_FF_HEADROOM'),  # between the FF line (5.720) and the
                                        #   hard authority line (5.901)
    (0.80, 6.20, 'DECEL_AUTHORITY'),    # past both; the harder limit reports
    (float('nan'), 3.0, 'INPUT'),
    (-0.8, 3.0, 'INPUT'),
    (0.80, 0.0, 'INPUT'),
    (0.80, 0.2, 'WIRE_BAND'),   # under the bridge floor. 8.6 m/s would read
                                #   END_STOP, because bounds are ordered
                                #   machine-damage-first and 8.6 is past
                                #   metal long before the wire objects
])
def test_the_refusal_names_the_bound(flight_s, speed, bound):
    """Every refusal carries ``BOUND:numbers``. A bare "too high" sends the
    operator to the wrong knob, so the bound name is the load-bearing half of
    the message."""
    v = te.vertical_release_speed_mps(flight_s) if speed is None else speed
    verdict = te.evaluate(flight_s, v)
    assert not verdict.ok
    assert verdict.bound == bound
    assert verdict.message.startswith(bound + ':')


def test_the_refusal_carries_the_computed_envelope_not_just_a_complaint():
    """The bound an operator actually hits must quote BOTH the offending
    quantity and the envelope, so the next goal can be typed off one line."""
    hi = te.evaluate(0.80, 8.6)
    assert 'peak' in hi.detail and 'rev' in hi.detail
    assert 'hard stop' in hi.detail and 'margin' in hi.detail
    assert 'envelope allows' in hi.detail and 'apex' in hi.detail


def test_an_admitted_throw_returns_a_clean_verdict():
    ok = te.evaluate(0.80, te.vertical_release_speed_mps(0.80))
    assert ok.ok and ok.bound == '' and ok.message == ''
