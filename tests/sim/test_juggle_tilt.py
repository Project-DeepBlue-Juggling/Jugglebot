"""Unit tests for the tilt-aimed cup geometry (``sim.juggle_tilt``).

Plan: plans/active/bb-online-juggle-tilt-rearchitecture.md Phase 1 / Rung 1.
Covers the collinear catch tilt (:func:`tilt_to_receive`), the cup-axis / lever-
arm helpers, and the lever-arm-compensated realisation (:func:`realize_tilted`)
that lands the cup OPENING on the commanded target. The realisation accuracy is
checked against the actual MuJoCo ``hand_opening`` site (the load-bearing Rung 0
lever-arm compensation).
"""
import numpy as np
import pytest

from sim.juggle_tilt import (
    MAX_TILT_DEG, Z_ACTIVE_MM, CUP_Z_BASE_MM, SLIDER_STROKE_MM,
    tilt_to_receive, tilt_to_throw, cup_axis, cup_lever_arm_mm, cup_lateral_shift_mm,
    realize_tilted,
)


# ---- tilt_to_receive: collinear catch -----------------------------------
def test_tilt_straight_down_arrival_is_level():
    rx, ry = tilt_to_receive(np.array([0.0, 0.0, -3.0]))
    assert (rx, ry) == (0.0, 0.0)


def test_tilt_zero_speed_is_level():
    assert tilt_to_receive(np.array([0.0, 0.0, 0.0])) == (0.0, 0.0)


def test_tilt_aligns_cup_axis_antiparallel_to_arrival():
    # Kai collinear catch: cup up-axis should be anti-parallel to the arrival
    # velocity (ball drops straight down the cup axis, zero lateral cup-frame vel).
    v = np.array([0.25, 0.0, -2.5])               # ~5.7 deg from vertical
    rx, ry = tilt_to_receive(v)
    a = cup_axis(rx, ry)
    np.testing.assert_allclose(a, -v / np.linalg.norm(v), atol=1e-6)


def test_tilt_direction_signs():
    # +vx arrival (descending) -> cup leans toward -x => ry < 0, rx ~ 0.
    rx, ry = tilt_to_receive(np.array([0.3, 0.0, -2.5]))
    assert ry < 0.0 and abs(rx) < 1e-9
    # +vy arrival -> rx > 0, ry ~ 0.
    rx2, ry2 = tilt_to_receive(np.array([0.0, 0.3, -2.5]))
    assert rx2 > 0.0 and abs(ry2) < 1e-9


def test_tilt_clamped_to_max():
    # A steep (45 deg) arrival saturates at MAX_TILT_DEG.
    rx, ry = tilt_to_receive(np.array([2.5, 0.0, -2.5]))
    mag = np.degrees(np.hypot(rx, ry))
    assert mag == pytest.approx(MAX_TILT_DEG, abs=1e-6)


# ---- tilt_to_throw: tilt-aimed throw (Rung 2a) --------------------------
def test_throw_straight_up_takeoff_is_level():
    rx, ry = tilt_to_throw(np.array([0.0, 0.0, 3.0]))
    assert (rx, ry) == (0.0, 0.0)


def test_throw_aligns_cup_axis_parallel_to_takeoff():
    # The ball detaches up the cup axis, so the axis points ALONG the (ascending)
    # take-off velocity — parallel, not anti-parallel (the catch's case).
    v = np.array([0.25, 0.0, 2.5])                 # ~5.7 deg from vertical
    rx, ry = tilt_to_throw(v)
    a = cup_axis(rx, ry)
    np.testing.assert_allclose(a, v / np.linalg.norm(v), atol=1e-6)


def test_throw_is_mirror_of_receive():
    # tilt_to_throw(v) == tilt_to_receive(-v): the throw aligns the axis along
    # the take-off; the catch aligns it against the (negated) arrival.
    v = np.array([0.3, -0.15, 2.6])
    assert tilt_to_throw(v) == tilt_to_receive(-v)


def test_throw_direction_signs():
    # +vx take-off (ascending) -> cup leans toward +x => ry > 0, rx ~ 0.
    rx, ry = tilt_to_throw(np.array([0.3, 0.0, 2.5]))
    assert ry > 0.0 and abs(rx) < 1e-9
    # +vy take-off -> rx < 0, ry ~ 0.
    rx2, ry2 = tilt_to_throw(np.array([0.0, 0.3, 2.5]))
    assert rx2 < 0.0 and abs(ry2) < 1e-9


def test_throw_tilt_clamped_to_max():
    rx, ry = tilt_to_throw(np.array([2.5, 0.0, 2.5]))   # 45 deg take-off
    assert np.degrees(np.hypot(rx, ry)) == pytest.approx(MAX_TILT_DEG, abs=1e-6)


def test_throw_is_production_reexport_single_source():
    """sim.juggle_tilt.tilt_to_throw IS the production function (single-ball-
    toss Phase 4 port: the def moved to motion/trajectory/tilt_geometry and a
    re-export replaced it here — one source of truth since the 2026-07-24
    merge). test_throw_is_mirror_of_receive above therefore cross-checks the
    PRODUCTION throw against the SIM receive: the cross-copy drift guard."""
    from jugglebot.motion.trajectory import tilt_geometry as tg
    from sim import juggle_tilt
    assert juggle_tilt.tilt_to_throw is tg.tilt_to_throw


def test_lever_arm_units_trap_sim_metres_vs_production_mm():
    """The lever helpers are deliberately NOT unified across the boundary
    (Phase-4 scope limit): the sim copies take cup_z in METRES, the production
    shaping/tilt_geometry ones take MILLIMETRES. A blind re-export would
    silently rescale every sim caller ×1000 — pin the cross-boundary parity at
    the correct units AND that a same-number mix-up is loudly wrong."""
    from jugglebot.motion.trajectory import tilt_geometry as tg
    rx, ry = 0.03, -0.02
    np.testing.assert_allclose(cup_lateral_shift_mm(rx, ry, 0.84),
                               tg.cup_lateral_shift_mm(rx, ry, cup_z_mm=840.0),
                               atol=1e-12)
    assert cup_lever_arm_mm(0.84) == pytest.approx(tg.cup_lever_arm_mm(840.0))
    # The trap: feeding the same NUMBER to both is a ×1000 error, not a drift.
    assert abs(cup_lever_arm_mm(840.0) - tg.cup_lever_arm_mm(840.0)) > 1e5


# ---- cup-axis / lever-arm helpers ---------------------------------------
def test_cup_axis_is_unit_and_level_is_z():
    np.testing.assert_allclose(cup_axis(0.0, 0.0), [0.0, 0.0, 1.0])
    a = cup_axis(0.05, -0.03)
    assert np.linalg.norm(a) == pytest.approx(1.0)


def test_lever_arm_sign_flips_through_tilt_centre():
    # Cup above the tilt centre -> positive arm; below -> negative.
    assert cup_lever_arm_mm(0.84) > 0.0       # 840 mm > 744.3 mm centre
    assert cup_lever_arm_mm(0.70) < 0.0       # 700 mm < centre


def test_lateral_shift_sign_matches_rung0():
    # Rung 0 sign check (arm > 0): +ry swings the opening +x.
    shift = cup_lateral_shift_mm(0.0, np.radians(5.0), 0.84)
    assert shift[0] > 0.0 and abs(shift[1]) < 1e-6


# ---- realize_tilted: level special case ---------------------------------
def test_realize_level_reduces_to_simple_slider():
    pose, slider = realize_tilted(np.array([0.05, -0.02]), 0.80, 0.0, 0.0)
    np.testing.assert_allclose(pose[:2], [50.0, -20.0])     # centroid = cup xy
    assert pose[2] == Z_ACTIVE_MM
    np.testing.assert_allclose(pose[3:], [0.0, 0.0, 0.0])
    assert slider == pytest.approx(800.0 - CUP_Z_BASE_MM)


def test_juggle_online_realize_level_unchanged_and_delegates_to_tilted():
    """``juggle_online.realize`` (Rung 2a unified it onto ``realize_tilted``) is
    byte-for-byte the old level form at zero tilt, and matches ``realize_tilted``
    under tilt — the single realisation for the level runner and the tilt throw."""
    from sim.juggle_online import realize, Z_ACTIVE_MM as ZA, CUP_Z_BASE_MM as CB
    cup = np.array([0.05, -0.02, 0.80])
    pose, slider = realize(cup)                          # level (default)
    np.testing.assert_allclose(pose, [50.0, -20.0, ZA, 0.0, 0.0, 0.0])
    assert slider == pytest.approx(800.0 - CB)
    # under tilt it delegates to realize_tilted exactly
    rx, ry = tilt_to_throw(np.array([0.2, 0.0, 2.7]))
    p1, s1 = realize(cup, rx, ry)
    p2, s2 = realize_tilted(cup[:2], float(cup[2]), rx, ry)
    np.testing.assert_array_equal(p1, p2)
    assert s1 == s2


def test_realize_slider_clamped_to_stroke():
    _, slider_hi = realize_tilted(np.zeros(2), 1.20, 0.0, 0.0)   # too high
    _, slider_lo = realize_tilted(np.zeros(2), 0.40, 0.0, 0.0)   # too low
    assert slider_hi == pytest.approx(SLIDER_STROKE_MM)
    assert slider_lo == pytest.approx(0.0)


# ---- realize_tilted: lever-arm compensation vs the real MuJoCo cup -------
@pytest.mark.parametrize("cup_xy", [(0.0, 0.0), (0.10, 0.0), (0.0, 0.12),
                                    (0.10, -0.10)])
@pytest.mark.parametrize("cup_z", [0.90, 0.74])
def test_realize_tilted_lands_opening_on_target(cup_xy, cup_z):
    """The Rung-0 lever-arm compensation must put the cup OPENING exactly on the
    commanded target, across the workspace and the slider range, under tilt."""
    import mujoco
    from sim.plant.mujoco_plant import MuJoCoPlant

    plant = MuJoCoPlant(control_dt=0.025, contact_carry=True)
    sid = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')
    rx, ry = tilt_to_receive(np.array([0.25, -0.1, -2.5]))      # a representative tilt
    pose, slider = realize_tilted(np.array(cup_xy), cup_z, rx, ry)
    plant.reset()
    for _ in range(120):                       # let the position actuators settle
        plant.command(plant.pose_to_extensions(pose))
        plant.command_hand(slider)
        plant.step(0.025)
    opening = plant.data.site_xpos[sid].copy()
    target = np.array([cup_xy[0], cup_xy[1], cup_z])
    np.testing.assert_allclose(opening, target, atol=1.0e-3)    # within 1 mm
