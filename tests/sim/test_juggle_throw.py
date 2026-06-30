"""Rung 2a — single-ball tilt-aimed open-loop throw (the throw primitive).

Phase 2 of plans/active/bb-online-juggle-tilt-rearchitecture.md. Drives the real
contact-physics throw (``sim.juggle_throw.run_single_throw``): carry the seated
ball up the TILTED cup axis on a ``plan_cup_cycle`` trajectory, detach along the
tilted axis, fly free, measure the landing. These tests pin the OPEN-LOOP
landing accuracy on the **reliable box** (the column toss + small lateral targets
that Rung 2b's self-catch needs) and the tilt-aim relation; the larger-lateral
directional separation asymmetry is a characterised limitation (see the probe
``tools/probes/juggle_throw_accuracy.py`` and the logbook entry).

The throw is DETERMINISTIC on the pinned MuJoCo (no RNG in the throw path — the
§3 noise only perturbs the post-throw landing OBSERVATION via the seeded
estimator), so the true landing error is seed-independent and the thresholds
below carry comfortable margin over the measured values (2026-06-30).
"""
import numpy as np
import pytest

from sim.juggle_throw import run_single_throw, SingleThrowConfig
from sim.juggle_noise import NoiseConfig


@pytest.fixture(scope="module")
def column():
    """Column toss (target = throw xy, tilt ~ 0) — the Rung-2b self-catch case."""
    return run_single_throw(SingleThrowConfig(target_xy_m=(0.0, 0.0)))


@pytest.fixture(scope="module")
def lateral_x():
    """100 mm +x lateral throw (the favorable, well-separating direction)."""
    return run_single_throw(SingleThrowConfig(target_xy_m=(0.10, 0.0)))


def test_column_toss_separates_and_lands_accurately(column):
    """The column toss separates cleanly and lands within ~15 mm of the target —
    deep inside the catch's ~60-80 mm reliable reach (Rung 1)."""
    assert column.separated
    assert column.error_mm < 15.0          # measured 8.3 mm
    assert column.tilt_deg < 0.5           # essentially level
    assert column.reach_mm < 15.0          # the reach the self-catch would face


def test_lateral_throw_separates_and_lands_within_catch_reach(lateral_x):
    """A 100 mm lateral tilt-aimed throw separates and lands within the catch's
    reliable reach (the gate framing: throw scatter == the catch's reach)."""
    assert lateral_x.separated
    assert lateral_x.error_mm < 35.0       # measured 21.5 mm
    assert lateral_x.error_mm < 80.0       # << the catch's ~60-80 mm reach


def test_tilt_aim_lateral_takeoff_is_slider_speed_times_sin_tilt(lateral_x):
    """Tilt-aim under real contact: the lateral take-off equals the take-off
    speed projected through the tilt (the lateral comes from the slider, not from
    band-limited platform translation), and the cup actually reaches that speed."""
    expected_lateral = lateral_x.takeoff_speed_mps * np.sin(np.radians(lateral_x.tilt_deg))
    assert lateral_x.lateral_takeoff_mps == pytest.approx(expected_lateral, abs=1e-6)
    # the cup is driven to the take-off speed (within slider tracking margin)
    assert lateral_x.cup_speed_at_release_mps == pytest.approx(
        lateral_x.takeoff_speed_mps, rel=0.05)
    assert lateral_x.tilt_deg > 1.0        # genuinely tilt-aimed


def test_favorable_small_lateral_targets_land_within_reach():
    """The reliable box (small lateral, favorable directions) lands within the
    catch's reliable reach — accurate enough to close the Rung-2b loop on."""
    for target, bound in [((0.05, 0.0), 25.0),     # measured 14.0
                          ((-0.07, 0.07), 25.0),   # measured 15.0
                          ((0.0, 0.05), 28.0)]:     # measured 18.3
        r = run_single_throw(SingleThrowConfig(target_xy_m=target))
        assert r.separated, f"{target} did not separate"
        assert r.error_mm < bound, f"{target} error {r.error_mm:.1f} >= {bound}"


def test_open_loop_error_is_deterministic_noise_adds_small_scatter():
    """The TRUE landing error is seed-independent (open-loop, no noise in the
    throw); the §3 tracking noise adds only a few mm of scatter to the landing
    the catch OBSERVES (the estimator averages it down)."""
    true_errs, obs_errs = [], []
    for s in range(3):
        r = run_single_throw(SingleThrowConfig(target_xy_m=(0.05, 0.0), seed=s))
        true_errs.append(r.error_mm)
        obs_errs.append(r.observed_error_mm)
    # true error identical across seeds (deterministic throw)
    assert max(true_errs) - min(true_errs) < 1e-6
    # noisy observed landing stays close to the true error, small spread
    assert max(obs_errs) - min(obs_errs) < 12.0
    assert max(obs_errs) < true_errs[0] + 12.0


def test_noise_off_matches_default_true_landing():
    """Disabling the §3 noise leaves the TRUE landing unchanged (the noise only
    touches the observation path, never the throw)."""
    noisy = run_single_throw(SingleThrowConfig(target_xy_m=(0.05, 0.0), seed=1))
    clean = run_single_throw(SingleThrowConfig(
        target_xy_m=(0.05, 0.0),
        noise=NoiseConfig(bb_throw_noise_frac=0.0, tracking_noise_mm=0.0)))
    assert noisy.landing_xy_m == pytest.approx(clean.landing_xy_m, abs=1e-9)
