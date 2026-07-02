"""Integration tests for the Rung-1 single catch (``sim.juggle_catch``).

Plan: plans/active/bb-online-juggle-tilt-rearchitecture.md Phase 1 / Rung 1 —
catch one BB-thrown ball cleanly (seated AND held, small characterised in-cup
offset) by translate-to-reach (position) + tilt-to-receive (orientation), robust
to the §3 noise (2% BB throw + 0.5 mm tracking, on by default).

These drive the full MuJoCo contact-carry catch, so they are slower than the
noise/tilt unit tests — kept to a lean seed/placement set. The catch is a
soft-contact seat on a knife-edge, so the robustness assertion is a *majority*
clean-rate (deterministic given the pinned MuJoCo build), not 100%; every clean
catch is asserted to seat within a small characterised offset. Headline
characterisation (logbook 2026-06-30): default placement 17/20 seeds clean under
the §3 noise, mean in-cup offset 3.3 mm, max 7.9 mm.
"""
import numpy as np
import pytest

from sim.juggle_catch import run_single_catch, SingleCatchConfig, SEAT_RADIUS_MM
from sim.juggle_noise import NoiseConfig


def test_catch_noise_off_is_clean_and_centred():
    """The deterministic baseline: with noise disabled the default BB throw is
    caught, held, and seated within a few mm of the cup centre."""
    r = run_single_catch(SingleCatchConfig(noise=NoiseConfig(0.0, 0.0), seed=0))
    assert r.caught and r.held_at_end and r.clean
    assert r.in_cup_offset_mm < 12.0
    # tilt-to-receive is applied: the BB arrives at a modest angle, so the catch
    # tilt is non-zero and within the ≤12° usable band.
    assert 0.0 < r.catch_tilt_deg <= 12.0


def test_catch_robust_under_section3_noise():
    """Under the §3 noise (2% BB + 0.5 mm tracking, default-on), the default
    placement catches cleanly for a strong majority of seeds, each seated within
    the characterised offset bound."""
    results = [run_single_catch(SingleCatchConfig(seed=s)) for s in range(8)]
    n_clean = sum(int(r.clean) for r in results)
    assert n_clean >= 6, f"only {n_clean}/8 clean under §3 noise"
    clean_offsets = [r.in_cup_offset_mm for r in results if r.clean]
    assert max(clean_offsets) < 15.0
    assert np.mean(clean_offsets) < SEAT_RADIUS_MM / 3.0    # well within the seat


@pytest.mark.parametrize("landing_xy", [(0.06, 0.0), (-0.06, 0.0),
                                        (0.0, 0.06), (0.0, -0.06)])
def test_translate_to_reach_catches_offset_landings(landing_xy):
    """Translate-to-reach: a BB throw landing off the nominal home (0,0) is caught
    cleanly by translating the platform centroid to the observed landing — the
    cup actually reaches (reach_mm > 0) and still seats within the offset bound."""
    r = run_single_catch(SingleCatchConfig(landing_xy_m=landing_xy, seed=0))
    assert r.clean, f"reach catch at {landing_xy} not clean (off={r.in_cup_offset_mm:.0f})"
    assert r.in_cup_offset_mm < 20.0
    assert r.reach_mm > 20.0          # the platform translated to reach the landing


def test_reach_clamped_to_workspace():
    """A landing pushed beyond the reach bound by noise is clamped, not flung to a
    leg-saturating reach — the result stays inside the workspace box."""
    from sim.juggle_catch import WORKSPACE_XY_M
    r = run_single_catch(SingleCatchConfig(landing_xy_m=(0.14, 0.0), seed=0))
    lx, ly = r.landing_xy_m
    assert abs(lx) <= WORKSPACE_XY_M + 1e-6 and abs(ly) <= WORKSPACE_XY_M + 1e-6


def test_fast_real_ball_butler_throw_is_seated():
    """Fast-catch FIDELITY (2026-07-02): a REAL Ball Butler throw — the fast, flat
    arrival the hardware actually delivers (~4.9 m/s vz, ~15° from vertical at
    catch_z, from the demo BB placement) — is caught, held, and seated tightly by
    the phase-matched descent seat + the firmed catch contact.

    This is the primitive the old (soft 50 ms contact + hold-high-then-drop-late)
    seat could NOT do: the fast ball punched straight through the still-descending
    cup (0/N held). The fix makes the cup CO-MOVING at the arrival phase and firms
    the contact enough to arrest the impact inside the seat window.

    The §3 TRACKING noise (0.5 mm observation, ON) is the noise the estimator must
    average to time the phase-matched descent — the crux this test exercises. The
    §3 BB-THROW noise (2%) is OFF here on purpose: for the real BB's long
    (~1.3 m) throw it scatters the LANDING well beyond the platform's reach box on
    a minority of seeds — a REACH limitation (characterised separately, as for the
    synthetic throw), NOT a seating failure. Turning it off isolates the variable
    under test — the SEATING of a fast arrival — which is what "fast-catch
    fidelity" means. Deterministic given the pinned MuJoCo + the seeded noise.
    """
    from sim.juggle_noise import NoiseConfig
    results = [run_single_catch(SingleCatchConfig(
        use_real_bb=True, noise=NoiseConfig(bb_throw_noise_frac=0.0,
                                            tracking_noise_mm=0.5), seed=s))
        for s in range(6)]
    for s, r in enumerate(results):
        assert r.caught and r.held_at_end and r.clean, (
            f"seed {s}: fast BB not cleanly seated "
            f"(caught={r.caught} held={r.held_at_end} off={r.in_cup_offset_mm:.0f})")
        assert r.in_cup_offset_mm < 20.0, (
            f"seed {s}: seated {r.in_cup_offset_mm:.1f} mm off centre (> 20 mm)")
        # The arrival is genuinely the FAST, FLAT BB throw — it comes in past the
        # 12° usable tilt band, so the receive tilt clamps near MAX_TILT_DEG. This
        # distinguishes the test from the gentle synthetic lob (~5.7°) and guards
        # against a BB-placement change that would quietly make it a slow throw.
        assert r.catch_tilt_deg >= 10.0, (
            f"seed {s}: catch tilt {r.catch_tilt_deg:.1f}° — not the fast/flat BB arrival")
