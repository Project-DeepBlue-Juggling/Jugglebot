"""Tests for jugglebot.can.throw_ballistics — predict + inverse + transforms."""
from __future__ import annotations

import math

import pytest

from jugglebot.can.throw_ballistics import (
    ThrowSolution,
    _wrap_pi,
    bb_release_state,
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

    NOTE: ``predict_throw`` now uses this SAME release-point model
    (``bb_release_state``), so it round-trips with the solver (see
    ``test_predict_throw_round_trips_with_solver``).  This test validates the
    solver's internal kinematics directly by manual propagation.
    """
    import jugglebot.hardware_config as hw

    x, y, z = _platform_target_in_bb_local()
    sol = solve_throw_local(x, y, z)

    s = hw.BB_GEOM_YAW_S_OFFSET_MM
    d = hw.BB_GEOM_PITCH_D_OFFSET_MM
    l = hw.BB_GEOM_RELEASE_L_POSITION_MM
    g_mmps2 = hw.GRAVITY_MPS2 * 1000.0

    # Release point in BB local frame (pitch-axis-z origin, per the
    # solver's frame convention — see docstring of solve_throw_local).
    # bb_calibration.py absorbs pitch_z_offset into bb_mocap_position.z,
    # so the solver's input z is already in the pitch-axis frame.
    cos_p = math.cos(sol.pitch_rad)
    sin_p = math.sin(sol.pitch_rad)
    r_rel = math.hypot(s, l * cos_p - d)
    alpha = sol.yaw_rad + math.atan2(s, l * cos_p - d)
    x_rel = r_rel * math.cos(alpha)
    y_rel = r_rel * math.sin(alpha)
    z_rel = l * sin_p

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


def test_predict_throw_round_trips_with_solver():
    """Forward/inverse consistency: ``predict_throw(solve_throw_local(target))``
    must land back at the target and agree on ToF.

    Both now share the release-point geometry (``bb_release_state``), so the old
    ~100 mm / tens-of-ms point-launch bias is gone.  If this regresses, the two
    models have drifted apart again — and the original reason for that bias means
    any ``BB_OP_LANDING_TIME_OFFSET_MS`` band-aid tuned to it must be re-checked.
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
    assert pred.landing_position[0] == pytest.approx(x, abs=1.0)
    assert pred.landing_position[1] == pytest.approx(y, abs=1.0)
    assert pred.tof_s == pytest.approx(sol.tof_s, abs=1e-3)


def test_bb_release_state_offsets_from_origin_match_geometry():
    """``bb_release_state`` places the release point at the serial-chain offset
    from the BB origin: along-throw l·cosφ − d, lateral s, up l·sinφ."""
    import jugglebot.hardware_config as hw
    s = hw.BB_GEOM_YAW_S_OFFSET_MM
    d = hw.BB_GEOM_PITCH_D_OFFSET_MM
    l = hw.BB_GEOM_RELEASE_L_POSITION_MM
    pitch = math.radians(60.0)
    # Throw straight along +x (yaw=0, no offset): along-throw → x, lateral → +y.
    rel, vel = bb_release_state(0.0, pitch, 3.0, (0.0, 0.0, 0.0), 0.0)
    assert rel[0] == pytest.approx(l * math.cos(pitch) - d, abs=1e-6)
    assert rel[1] == pytest.approx(s, abs=1e-6)
    assert rel[2] == pytest.approx(l * math.sin(pitch), abs=1e-6)
    # Velocity is along (cosφ, 0, sinφ)·speed.
    assert vel[0] == pytest.approx(3000.0 * math.cos(pitch), abs=1e-6)
    assert vel[2] == pytest.approx(3000.0 * math.sin(pitch), abs=1e-6)


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


# ─────────────────────────────────────────────────────────────────────
# Closed-form pitch vs the retired 0.5° sweep (tests/_bb_pitch_grid_oracle.py)
# ─────────────────────────────────────────────────────────────────────
#
# Recipe probed 2026-09-18 before these were written (210 000 samples over
# seven regimes, zero disagreements, zero targets lost outside the near field):
# the continuous optimum sits within ONE grid step above the sweep's argmin,
# never has a larger horizontal velocity, and lands on the target exactly.
# The regimes below are the three constraints that can bind, plus the one
# place the apex-cap solve is delicate.

_REGIMES = {
    # name: (pitch_max_deg, max_speed_mps, max_height_mm, z range, min binding counts)
    'default-apex-bound': (85.0, 5.0, 500.0, (-1500.0, 600.0), {'h': 500}),
    'speed-bound':        (85.0, 3.3, 500.0, (-1500.0, 600.0), {'v': 300, 'h': 100}),
    'pitch-max-bound':    (60.0, 5.0, 2000.0, (-1500.0, 600.0), {'pmax': 500, 'v': 50}),
    # Targets within millimetres of the apex cap (cap 500 mm above a release
    # point ~100-150 mm up): the moving release point opens a WINDOW of
    # feasible pitches with two cap roots, and the solver must take the upper.
    'apex-knife-edge':    (85.0, 5.0, 500.0, (540.0, 660.0), {'h': 300}),
}


@pytest.mark.parametrize('regime', sorted(_REGIMES))
def test_closed_form_pitch_matches_retired_grid_sweep(regime):
    import random

    import jugglebot.hardware_config as hw
    from tests._bb_pitch_grid_oracle import grid_pitch_solve

    pitch_max_deg, v_max_mps, h_max_mm, z_range, min_binding = _REGIMES[regime]
    s = hw.BB_GEOM_YAW_S_OFFSET_MM
    l = hw.BB_GEOM_RELEASE_L_POSITION_MM
    d = hw.BB_GEOM_PITCH_D_OFFSET_MM
    g = hw.GRAVITY_MPS2 * 1000.0
    p_min = math.radians(hw.BB_GEOM_PITCH_MIN_DEG)
    p_max = math.radians(pitch_max_deg)
    # The sweep's true step: int() truncation can stretch it past 0.5°.
    n_steps = max(int((p_max - p_min) / math.radians(0.5)), 1) + 1
    step = (p_max - p_min) / (n_steps - 1)

    rng = random.Random(20260918)
    binding = {'pmax': 0, 'h': 0, 'v': 0}
    refused_near = rising_refused = 0
    for _ in range(2000):
        A = rng.uniform(250.0, 3600.0)
        z = rng.uniform(*z_range)
        yaw = rng.uniform(0.2, 1.2)
        x = A * math.cos(yaw) - s * math.sin(yaw)
        y = A * math.sin(yaw) + s * math.cos(yaw)

        old = grid_pitch_solve(A, z, l, d, p_min, p_max,
                               v_max_mps * 1000.0, h_max_mm, g)
        try:
            sol = solve_throw_local(x, y, z, pitch_max_deg=pitch_max_deg,
                                    max_speed_mps=v_max_mps,
                                    max_height_mm=h_max_mm)
        except ValueError as e:
            if 'too close' in str(e):
                refused_near += 1
                continue
            # The only other refusals allowed: one the sweep agreed with, or a
            # sweep "solution" that reaches the target still RISING (a
            # fly-through, which the closed form now refuses).
            if old is not None:
                o_pitch, o_v, _, o_h_vel = old
                o_tof = (A - l * math.cos(o_pitch) + d) / o_h_vel
                assert o_v * math.sin(o_pitch) - g * o_tof >= 0, (
                    f'lost a target the sweep solved: A={A} z={z}: {e}')
                rising_refused += 1
            continue

        # Limits hold exactly — no tolerance.
        assert p_min <= sol.pitch_rad <= p_max
        assert sol.speed_mps <= v_max_mps
        assert sol.peak_height_mm <= h_max_mm

        # Lands on the target through the independent forward model.
        pred = predict_throw(sol.yaw_rad, sol.pitch_rad, sol.speed_mps,
                             (0.0, 0.0, 0.0), 0.0, catch_height_mm=z)
        assert pred is not None
        assert pred.landing_position[0] == pytest.approx(x, abs=1e-6)
        assert pred.landing_position[1] == pytest.approx(y, abs=1e-6)
        assert pred.tof_s == pytest.approx(sol.tof_s, abs=1e-9)

        if old is None:
            continue    # feasible window narrower than a grid step: a gain
        old_pitch, _, _, old_h_vel = old
        assert -1e-9 <= sol.pitch_rad - old_pitch < step + 1e-9, (A, z)
        assert sol.speed_mps * 1000.0 * math.cos(sol.pitch_rad) <= old_h_vel + 1e-6

        if sol.pitch_rad == p_max:
            binding['pmax'] += 1
        elif sol.peak_height_mm > h_max_mm - 1e-6:
            binding['h'] += 1
        else:
            binding['v'] += 1

    for key, floor in min_binding.items():
        assert binding[key] >= floor, (regime, binding)
    assert refused_near < 200, refused_near     # the guard is not eating the domain
    if regime == 'default-apex-bound':
        assert rising_refused == 0      # unreachable inside the real envelope


def test_solve_throw_local_apex_sits_on_the_cap_for_typical_target():
    """No grid quantisation left: the typical throw's apex IS the cap (the
    sweep left it anywhere in ~470–500 mm depending on where the grid fell)."""
    x, y, z = _platform_target_in_bb_local()
    sol = solve_throw_local(x, y, z)
    assert sol.peak_height_mm <= 500.0
    assert sol.peak_height_mm == pytest.approx(500.0, abs=1e-6)


@pytest.mark.parametrize('x, y, z', [
    (200.0, 150.0, -1000.0),    # A ≈ 227 mm, deep drop: sweep chose a 12° "drop"
    (130.0, 140.0, 30.0),       # A ≈ 159 mm: optimum at an INTERIOR pitch (probed 2026-09-18)
    (0.0, 105.65, -500.0),      # on the s-circle, directly below: A = 0
])
def test_solve_throw_local_refuses_near_field_targets(x, y, z):
    """Near the yaw axis the release point's travel dominates the range and the
    steepest throw is not the softest landing — refused by design (owner,
    2026-09-18), never silently solved with the wrong rule."""
    with pytest.raises(ValueError, match='too close'):
        solve_throw_local(x, y, z, yaw_s_offset_mm=105.65)
