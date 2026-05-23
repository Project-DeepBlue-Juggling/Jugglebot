"""Tests for jugglebot.can.throw_ballistics — predict + inverse + transforms."""
from __future__ import annotations

import math

import pytest

from jugglebot.can.throw_ballistics import (
    ThrowSolution,
    _wrap_pi,
    global_to_bb_local,
    predict_throw,
    solve_throw_local,
    yaw_solve_thetas,
)


# ─────────────────────────────────────────────────────────────────────
# _wrap_pi
# ─────────────────────────────────────────────────────────────────────

def test_wrap_pi_within_range_is_identity():
    for a in (-1.0, 0.0, 1.0, math.pi / 2):
        assert _wrap_pi(a) == pytest.approx(a)


def test_wrap_pi_above_pi_wraps_negative():
    assert _wrap_pi(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)


def test_wrap_pi_minus_pi_normalised_to_plus_pi():
    assert _wrap_pi(-math.pi) == pytest.approx(math.pi)


# ─────────────────────────────────────────────────────────────────────
# yaw_solve_thetas
# ─────────────────────────────────────────────────────────────────────

def test_yaw_solve_with_zero_offset_is_atan2():
    # When s=0, both solutions degenerate to atan2(y, x) (plus the pi-shift)
    t1, t2, chosen = yaw_solve_thetas(1000.0, 0.0, 0.0)
    assert t1 == pytest.approx(0.0)
    assert abs(chosen) <= abs(t2)


def test_yaw_solve_target_inside_s_circle_returns_nan():
    # Target inside the s-offset circle has no real solution
    t1, t2, chosen = yaw_solve_thetas(50.0, 0.0, 200.0)
    assert math.isnan(t1) and math.isnan(t2) and math.isnan(chosen)


def test_yaw_solve_origin_returns_nan():
    t1, t2, chosen = yaw_solve_thetas(0.0, 0.0, -100.0)
    assert math.isnan(t1) and math.isnan(t2) and math.isnan(chosen)


def test_yaw_solve_picks_smaller_magnitude():
    # Target with one obvious near solution and a far back-half solution
    t1, t2, chosen = yaw_solve_thetas(2000.0, 50.0, -105.65)
    assert chosen == t1 if abs(t1) <= abs(t2) else t2


# ─────────────────────────────────────────────────────────────────────
# global_to_bb_local
# ─────────────────────────────────────────────────────────────────────

def test_global_to_bb_local_identity_at_origin_zero_offset():
    x, y, z = global_to_bb_local(100.0, 200.0, 300.0, (0.0, 0.0, 0.0), 0.0)
    assert (x, y, z) == pytest.approx((100.0, 200.0, 300.0))


def test_global_to_bb_local_pure_translation():
    x, y, z = global_to_bb_local(150.0, 250.0, 350.0,
                                 bb_position_mm=(50.0, 50.0, 50.0),
                                 yaw_offset_rad=0.0)
    assert (x, y, z) == pytest.approx((100.0, 200.0, 300.0))


def test_global_to_bb_local_pure_rotation_90deg():
    # yaw_offset = +90° rotates BB-local x-axis to world +y axis.
    # A world point at (+y) sits along BB-local +x after the inverse rotation.
    x, y, z = global_to_bb_local(0.0, 1000.0, 0.0,
                                 bb_position_mm=(0.0, 0.0, 0.0),
                                 yaw_offset_rad=math.pi / 2)
    assert x == pytest.approx(1000.0)
    assert y == pytest.approx(0.0, abs=1e-9)


# ─────────────────────────────────────────────────────────────────────
# solve_throw_local — feasibility + roundtrip
# ─────────────────────────────────────────────────────────────────────

def _platform_target_in_bb_local():
    """A target in BB local frame that's typical of the catching cone:

    ~1.2 m in front of BB (x), small lateral offset (y), at z=0 in BB local
    frame (BB origin is roughly co-planar with the catch height in practice).
    """
    return 1200.0, 50.0, 0.0


def test_solve_throw_local_returns_throwsolution_for_typical_target():
    x, y, z = _platform_target_in_bb_local()
    sol = solve_throw_local(x, y, z)
    assert isinstance(sol, ThrowSolution)
    assert 0.0 < sol.speed_mps <= 5.0          # within BB max
    assert math.radians(12.0) <= sol.pitch_rad <= math.radians(85.0)
    assert sol.tof_s > 0
    assert sol.peak_height_mm >= 0


def test_solve_throw_local_lands_at_target_via_release_point_projectile():
    """The inverse solver's output, propagated through release-point
    projectile motion (matching the solver's own BB serial-chain model),
    must land at the requested target.

    NOTE: ``predict_throw`` uses a *simpler* point-launch model that
    ignores BB serial-chain offsets (s/d/l).  Comparing solver output to
    ``predict_throw`` shows a systematic ~100 mm landing offset / tens-of-ms
    ToF discrepancy — that's a real model mismatch the production pipeline
    absorbs via ``BB_OP_LANDING_TIME_OFFSET_MS`` (-120 ms).  This test
    validates the solver's internal kinematics; the model-mismatch test
    follows separately.
    """
    import jugglebot.hardware_config as hw

    x, y, z = _platform_target_in_bb_local()
    sol = solve_throw_local(x, y, z)

    s = hw.BB_GEOM_YAW_S_OFFSET_MM
    d = hw.BB_GEOM_PITCH_D_OFFSET_MM
    l = hw.BB_GEOM_RELEASE_L_POSITION_MM
    g_mmps2 = hw.GRAVITY_MPS2 * 1000.0

    # Release point in BB local frame.  Z includes the pitch-axis vertical
    # offset from the yaw axis (BB_GEOM_PITCH_Z_OFFSET_MM) plus l*sin(pitch).
    cos_p = math.cos(sol.pitch_rad)
    sin_p = math.sin(sol.pitch_rad)
    r_rel = math.hypot(s, l * cos_p - d)
    alpha = sol.yaw_rad + math.atan2(s, l * cos_p - d)
    x_rel = r_rel * math.cos(alpha)
    y_rel = r_rel * math.sin(alpha)
    z_rel = hw.BB_GEOM_PITCH_Z_OFFSET_MM + l * sin_p

    # Throw velocity (along yaw + pitch direction)
    v_mmps = sol.speed_mps * 1000.0
    vx = v_mmps * cos_p * math.cos(sol.yaw_rad)
    vy = v_mmps * cos_p * math.sin(sol.yaw_rad)
    vz = v_mmps * sin_p

    # Propagate ToF
    t = sol.tof_s
    lx = x_rel + vx * t
    ly = y_rel + vy * t
    lz = z_rel + vz * t - 0.5 * g_mmps2 * t * t

    # Solver's own model should land within mm of the target
    assert lx == pytest.approx(x, abs=5.0)
    assert ly == pytest.approx(y, abs=5.0)
    assert lz == pytest.approx(z, abs=5.0)


def test_solve_throw_local_vs_predict_throw_has_serial_chain_bias():
    """Document the bias between the two ballistic models.

    Solver: full BB serial chain (release point = yaw_axis + serial offsets).
    Predictor: point launch from ``bb_position_mm``.

    For typical throws the landing-position discrepancy is on the order of
    100 mm in x and the ToF discrepancy is tens of ms — both are
    constant-ish biases that the production pipeline compensates via
    ``BB_OP_LANDING_TIME_OFFSET_MS``.  If a future refactor unifies the two
    models, the magnitude should drop and this test will fail loudly —
    flagging the obsoleted offset for removal.
    """
    x, y, z = _platform_target_in_bb_local()
    sol = solve_throw_local(x, y, z)

    pred = predict_throw(
        yaw_rad=sol.yaw_rad,
        pitch_rad=sol.pitch_rad,
        speed_mps=sol.speed_mps,
        bb_position_mm=(0.0, 0.0, 0.0),
        yaw_offset_rad=0.0,
        catch_height_mm=z,
    )
    assert pred is not None
    # x-error of the predictor relative to the target (solver lands at x).
    x_err_mm = abs(pred.landing_position[0] - x)
    assert 50.0 < x_err_mm < 200.0, (
        f"Predictor-vs-target x-bias = {x_err_mm:.1f} mm — outside the "
        "expected serial-chain offset range. If this dropped near zero, "
        "predict_throw may have started accounting for serial offsets, "
        "in which case the production landing_time_offset compensation is "
        "obsolete and worth re-tuning.")


def test_solve_throw_local_far_target_raises_speed_constraint():
    """A target far enough that no pitch satisfies the speed cap.

    100 m in front at z=0 is well outside the 5 m/s BB envelope — every
    pitch in the search grid will need v > v_max.
    """
    with pytest.raises(ValueError, match="No feasible trajectory"):
        solve_throw_local(100_000.0, 0.0, 0.0)


def test_solve_throw_local_target_on_s_circle_above_raises():
    """Target on the s-offset circle has zero horizontal range A.  With z>0
    it lies directly above the release locus — projectile motion can't reach
    it (the special-case branch in the solver)."""
    s_test = 100.0   # positive s so chosen yaw lands at 0° (within [0°, 185°])
    with pytest.raises(ValueError, match="directly above"):
        # Target at (0, s_test, +z) sits on the s-circle; yaw_solve_thetas
        # picks yaw=0 (chosen has smaller magnitude), A_sq=0 → A=0 → branch.
        solve_throw_local(0.0, s_test, 500.0,
                          yaw_s_offset_mm=s_test)


def test_solve_throw_local_yaw_out_of_range_raises():
    # Target behind BB (negative x) → yaw solution ~180°, outside default
    # [0°, 185°] range only when negative.  Use a target with yaw < yaw_min.
    # With yaw_min=0°, any negative-y at positive x picks a slightly negative
    # yaw — set y very negative to push outside.
    with pytest.raises(ValueError, match="out of BB range"):
        solve_throw_local(100.0, -1500.0, 0.0)


def test_solve_throw_local_peak_height_below_limit():
    """The solver must respect the max-height constraint."""
    x, y, z = _platform_target_in_bb_local()
    sol = solve_throw_local(x, y, z)
    assert sol.peak_height_mm <= 500.0      # default BB_OP_MAX_THROW_HEIGHT_M


def test_solve_throw_local_pitch_grid_finer_step_does_not_break_anything():
    """A finer pitch step should still produce a valid solution (smaller
    landing error in the roundtrip, but here just smoke-test no crash)."""
    x, y, z = _platform_target_in_bb_local()
    sol = solve_throw_local(x, y, z, pitch_step_deg=0.1)
    assert sol.speed_mps > 0
    assert sol.tof_s > 0
