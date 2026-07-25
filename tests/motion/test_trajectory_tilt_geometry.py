"""Cup tilt geometry tests — the bb-reference geometry regression.

Pins the ported ``juggle_tilt`` catch math (Phase 6) against the Jugglebot-bb
reference values (1.66 mm/deg lever arm at the Rung-0 operating cup height, ≤12°
tilt clamp) and the collinear-catch invariant (cup axis anti-parallel to the
arrival velocity), plus the Phase-4 throw mirror (``tilt_to_throw``, single-ball-
toss Tier 8b: cup axis PARALLEL to the take-off, exact mirror identity), so a
future edit to ``tilt_geometry`` / ``shaping`` that drifts the shared cup
geometry fails loudly.
"""

from __future__ import annotations

import numpy as np
import pytest

from jugglebot.motion.trajectory import tilt_geometry as tg


# ── Collinear-catch invariant: cup axis anti-parallel to the arrival velocity ──

@pytest.mark.parametrize('seed', range(8))
def test_cup_axis_antiparallel_to_arrival_velocity(seed):
    """For a descending arrival within the tilt clamp, cup_axis(tilt_to_receive(v))
    is anti-parallel to v (the ball drops straight down the cup axis)."""
    rng = np.random.default_rng(seed)
    # A descending arrival with a modest lateral component (stays under the 12° clamp).
    lateral = rng.uniform(-400.0, 400.0, 2)
    v = np.array([lateral[0], lateral[1], -rng.uniform(2500.0, 4000.0)])
    rx, ry = tg.tilt_to_receive(v)
    axis = tg.cup_axis(rx, ry)
    want = -v / np.linalg.norm(v)
    # Anti-parallel: cup up-axis points back along the incoming (descending) ball.
    assert np.dot(axis, want) > 0.999


def test_straight_down_arrival_is_level():
    """A straight-down arrival needs no tilt (level cup)."""
    rx, ry = tg.tilt_to_receive(np.array([0.0, 0.0, -3000.0]))
    assert rx == 0.0 and ry == 0.0


def test_zero_velocity_is_level():
    rx, ry = tg.tilt_to_receive(np.zeros(3))
    assert (rx, ry) == (0.0, 0.0)


def test_velocity_unit_agnostic():
    """tilt_to_receive is direction-only — mm/s and m/s give the same tilt."""
    v_mms = np.array([300.0, -200.0, -3200.0])
    a = tg.tilt_to_receive(v_mms)
    b = tg.tilt_to_receive(v_mms / 1000.0)
    assert np.allclose(a, b)


# ── The 12° clamp ──

def test_tilt_clamped_to_max():
    """A fast, shallow arrival saturates the from-vertical tilt at MAX_TILT_DEG."""
    # Nearly horizontal arrival → would want ~80° tilt; must clamp to 12°.
    v = np.array([3000.0, 0.0, -300.0])
    assert tg.tilt_to_receive_deg(v) == pytest.approx(tg.MAX_TILT_DEG, abs=1e-6)
    assert tg.MAX_TILT_DEG == 12.0


def test_custom_max_tilt_respected():
    v = np.array([3000.0, 0.0, -300.0])
    assert tg.tilt_to_receive_deg(v, max_tilt_deg=5.0) == pytest.approx(5.0, abs=1e-6)


# ── Throw mirror (Phase 4 port): cup axis parallel to the take-off ──

def test_throw_straight_up_takeoff_is_level():
    """A straight-up take-off needs no tilt (level cup) — mirrors the
    straight-down receive case exactly."""
    rx, ry = tg.tilt_to_throw(np.array([0.0, 0.0, 3000.0]))
    assert (rx, ry) == (0.0, 0.0)


@pytest.mark.parametrize('seed', range(8))
def test_throw_cup_axis_parallel_to_takeoff_velocity(seed):
    """For an ascending take-off within the tilt clamp, cup_axis(tilt_to_throw(v))
    is PARALLEL to v (the ball detaches up the tilted cup axis) — the mirror of
    the receive test's anti-parallel invariant."""
    rng = np.random.default_rng(seed)
    # An ascending take-off with a modest lateral component (under the 12° clamp).
    lateral = rng.uniform(-400.0, 400.0, 2)
    v = np.array([lateral[0], lateral[1], rng.uniform(2500.0, 4000.0)])
    rx, ry = tg.tilt_to_throw(v)
    axis = tg.cup_axis(rx, ry)
    assert np.dot(axis, v / np.linalg.norm(v)) > 0.999


@pytest.mark.parametrize('v', [
    (300.0, -150.0, 2600.0),
    (0.0, 250.0, 3900.0),
    (-120.0, -80.0, 3000.0),
    (3000.0, 0.0, 300.0),      # would-saturate case: mirror holds through the clamp
])
def test_throw_is_exact_mirror_of_receive(v):
    """tilt_to_throw(v) == tilt_to_receive(-v), exact ``==`` same-module — the
    port-faithfulness pin (the sim body was exactly this one-line mirror)."""
    v = np.array(v)
    assert tg.tilt_to_throw(v) == tg.tilt_to_receive(-v)


def test_throw_direction_signs():
    """Rung-0 sign convention: +vx take-off ⇒ cup leans toward +x ⇒ ry > 0,
    rx == 0; +vy take-off ⇒ rx < 0, ry == 0."""
    rx, ry = tg.tilt_to_throw(np.array([300.0, 0.0, 2500.0]))
    assert ry > 0.0 and rx == 0.0
    rx2, ry2 = tg.tilt_to_throw(np.array([0.0, 300.0, 2500.0]))
    assert rx2 < 0.0 and ry2 == 0.0


def test_throw_tilt_clamps_and_honours_custom_max_positionally():
    """A 45° take-off saturates at MAX_TILT_DEG; a custom ceiling is honoured
    POSITIONALLY (sim/juggle_throw.py:252 and sim/juggle_selfcatch.py:466 pass
    it positionally through the re-export — the signature must not change)."""
    v = np.array([2500.0, 0.0, 2500.0])
    rx, ry = tg.tilt_to_throw(v)
    assert np.degrees(np.hypot(rx, ry)) == pytest.approx(tg.MAX_TILT_DEG, abs=1e-6)
    rx5, ry5 = tg.tilt_to_throw(v, 5.0)
    assert np.degrees(np.hypot(rx5, ry5)) == pytest.approx(5.0, abs=1e-6)


def test_throw_velocity_unit_agnostic():
    """tilt_to_throw is direction-only — mm/s and m/s give the same tilt."""
    v = np.array([300.0, -200.0, 3200.0])
    assert np.allclose(tg.tilt_to_throw(v), tg.tilt_to_throw(v / 1000.0))


# ── bb-reference lever-arm regression (the shared cup geometry, single source) ──

def test_lever_arm_reference_values():
    """The re-exported cup lever arm matches the Jugglebot-bb Rung-0 reference."""
    assert tg.CUP_TILT_CENTER_Z_MM == 744.3
    assert tg.LEVER_ARM_MM_PER_DEG == 1.66
    # Derivation cross-check: at the Rung-0 operating cup height the arm is ~95.1 mm
    # above the tilt centre, and arm·sin(1°) reproduces the reported 1.66 mm/deg.
    arm = tg.cup_lever_arm_mm(tg.LEAN_CUP_Z_MM)
    assert arm == pytest.approx(95.1, abs=0.1)
    assert arm * np.sin(np.radians(1.0)) == pytest.approx(tg.LEVER_ARM_MM_PER_DEG,
                                                          abs=0.005)


def test_lever_arm_scales_with_cup_height():
    """The arm is cup_z − centre, so it flips sign below the tilt centre."""
    assert tg.cup_lever_arm_mm(tg.CUP_TILT_CENTER_Z_MM + 50.0) == pytest.approx(50.0)
    assert tg.cup_lever_arm_mm(tg.CUP_TILT_CENTER_Z_MM - 50.0) == pytest.approx(-50.0)
