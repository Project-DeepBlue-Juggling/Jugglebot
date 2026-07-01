"""Rung 2b — single-ball tilt-aimed self-catch loop (the MAKE-OR-BREAK gate).

Phase 3 of plans/active/bb-online-juggle-tilt-rearchitecture.md. Drives the real
contact-physics self-catch loop (``sim.juggle_selfcatch.run_self_catch``): compose
the Rung-2a tilt-aimed throw with the Rung-1 catch into a single-ball
toss->catch->toss loop, re-planned each cycle under the §3 tracking noise.

These tests pin BOTH make-or-break rungs, both a BREAK:

COLUMN (``oscillate=False``, the original 2026-07-01 gate):
* the primitives DO compose for a single cycle — the first throw separates and is
  caught + held, landing within the catch's reliable reach; and
* the headline capability — a self-catch that **sustains >= 10 cycles with loop
  gain < 1** — is **not achieved**: the pure column self-catch diverges (the
  ``xfail(strict=True)`` below), for the column-specific reasons characterised in
  ``logbook/2026-07-01-rung2b-selfcatch-column-divergence.md`` (a caught, centred,
  spinless ball does not cleanly detach on a column, and the reach amplifies each
  cycle — loop gain > 1). Tilt is ~0 for a column, so the tilt mechanism never
  engages; the column is a degenerate case where tilt cannot help.

OSCILLATION (``oscillate=True``, the operator's OPTION-1 re-plan — the HONEST tilt
test on a NON-degenerate geometry):
* the primitives compose the first LATERAL cycle WITH tilt engaged (~1.4 deg); but
* the oscillation ALSO does not sustain (``xfail(strict=True)`` headline) — it
  diverges within 1-4 cycles with tilt engaged (max sustained 4 of >= 10, at x-20,
  across the committed 20-70 mm sweep). The in-cup SEAT offset stays small
  (tilt keeps the ball centred) but the LANDING amplifies (loop gain > 1). Root
  cause: the tilt-aimed throw's landing is chaotically/deterministically sensitive
  to the throw-ORIGIN pose (a contact-detach knife-edge). Tilt fixes the band-limit
  but NOT this pose-chaos. See
  ``logbook/2026-07-01-rung2b-oscillation-tilt-engaged-diverges.md``.

The loop is DETERMINISTIC per seed (the §3 noise is the only RNG and the runner
threads ``seed``), so the thresholds below carry margin over the measured
(2026-07-01) values and the divergence is seed-reproducible.
"""
import numpy as np
import pytest

from sim.juggle_selfcatch import (
    run_self_catch, SelfCatchConfig, CATCH_REACH_MM,
)
from sim.juggle_throw import run_single_throw, SingleThrowConfig
from sim.juggle_noise import NoiseConfig


@pytest.fixture(scope="module")
def faithful_seed0():
    """The faithful composition (Rung-2a plan_cup_cycle recover, stationary
    column) — the primary make-or-break configuration."""
    return run_self_catch(SelfCatchConfig(seed=0, n_cycles=12))


def test_first_cycle_composes(faithful_seed0):
    """The primitives compose for ONE cycle: the first (seeded, centred) column
    throw separates, is caught + held, and lands within the catch's ~60-80 mm
    reliable reach (the Rung-2a column result, ~8 mm, carried into the loop)."""
    c0 = faithful_seed0.cycles[0]
    assert c0.separated
    assert c0.caught and c0.held_at_end
    assert c0.reach_mm < 25.0                      # measured ~8 mm
    assert c0.reach_mm < CATCH_REACH_MM            # well inside the catch reach
    assert c0.in_off_end_mm < 20.0                 # seated near-centred


@pytest.mark.xfail(strict=True, reason=(
    "Rung-2b MAKE-OR-BREAK: the pure column self-catch does NOT sustain "
    "(loop gain > 1). A caught, centred, spinless ball does not cleanly detach "
    "on a column, and the reach amplifies each cycle past the catch's reliable "
    "reach -> drop. Tilt is ~0 for a column, so the tilt mechanism never engages. "
    "Pending operator re-plan -- see logbook/2026-07-01-rung2b-selfcatch-"
    "column-divergence.md. Flip to XPASS if a future approach sustains >= 10."))
def test_column_selfcatch_sustains_ten_cycles(faithful_seed0):
    """HEADLINE capability (the gate): the single-ball self-catch sustains >= 10
    cycles with loop gain < 1. Currently xfails (diverges within 1-3 cycles)."""
    assert faithful_seed0.sustained >= 10


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4, 5])
def test_column_selfcatch_does_not_sustain(seed):
    """The characterised BREAK, across seeds: the column self-catch composes the
    first cycle but does NOT sustain (it diverges within 1-3 cycles). The failure
    mode varies by seed -- some diverge via the reach amplifying past the catch's
    reliable reach, some via the column catch-seat knife-edge at a modest reach --
    but no seed sustains anywhere near 10 cycles."""
    r = run_self_catch(SelfCatchConfig(seed=seed, n_cycles=12))
    assert r.sustained >= 1                        # the first cycle always composes
    assert r.sustained < 5                         # ...but the loop does not sustain
    assert r.diverged


def test_reach_amplifies_loop_gain_gt_one():
    """The loop-gain signature (seed 3, a clear reach-amplification case): the
    catch reach AMPLIFIES from the tight first throw (~7 mm) past the catch's
    ~60-80 mm reliable reach within a couple of cycles -- loop gain > 1, the same
    class as the pre-tilt divergence (logbook 2026-06-27)."""
    r = run_self_catch(SelfCatchConfig(seed=3, n_cycles=12))
    reaches = [c.reach_mm for c in r.cycles]
    assert reaches[0] < 20.0                       # tight first throw (~7 mm)
    assert max(reaches) > 3.0 * reaches[0]         # amplifies several-fold
    assert max(reaches) > CATCH_REACH_MM           # past the reliable reach -> drop


def test_selfcatch_is_deterministic_per_seed():
    """The loop is seed-reproducible (the §3 tracking noise is the only RNG): the
    same seed gives the same sustained count and the same first-cycle reach."""
    a = run_self_catch(SelfCatchConfig(seed=2, n_cycles=8))
    b = run_self_catch(SelfCatchConfig(seed=2, n_cycles=8))
    assert a.sustained == b.sustained
    assert a.cycles[0].reach_mm == pytest.approx(b.cycles[0].reach_mm, abs=1e-9)
    assert a.cycles[0].landing_xy_mm == pytest.approx(b.cycles[0].landing_xy_mm, abs=1e-9)


def test_drift_variant_also_fails_to_sustain():
    """The in-place (drift) variant — throw a column from the caught xy, NO
    reposition-to-origin — isolates the reposition's amplification, and STILL does
    not sustain: the column catch-seat is itself a knife-edge (the ball escapes the
    seat), so even without the reposition amplification the loop breaks at cycle 0.
    Confirms the divergence is column-intrinsic, not only a reposition artefact."""
    r = run_self_catch(SelfCatchConfig(seed=0, n_cycles=12, stationary=False))
    assert r.sustained < 10


# ============================================================================
# OSCILLATION (oscillate=True) — the operator's OPTION-1 re-plan: the HONEST tilt
# test on a NON-degenerate geometry. Shuttle a single ball A<->B, every throw
# lateral, tilt ENGAGED. RESULT: STILL a BREAK (diverges within 1-4 cycles).
# ============================================================================

@pytest.fixture(scope="module")
def osc_seed0():
    """The default oscillation (x-axis, A=(-20,0) <-> B=(20,0), 40 mm, tilt ~1.4
    deg) — the primary make-or-break configuration for the tilt hypothesis."""
    return run_self_catch(SelfCatchConfig(seed=0, n_cycles=12, oscillate=True))


def test_oscillation_engages_tilt(osc_seed0):
    """The whole point of the re-plan: a LATERAL A<->B throw commands NON-ZERO
    tilt, so the tilt mechanism actually engages (contrast the degenerate column,
    whose throw commands ~0 tilt). This is what makes the oscillation an HONEST
    test of the tilt hypothesis, not a degenerate one."""
    assert osc_seed0.cycles[0].tilt_deg > 1.0            # measured ~1.42 deg
    # contrast: the column commands ~0 tilt (tilt mechanism inactive)
    column = run_self_catch(SelfCatchConfig(seed=0, n_cycles=2))
    assert column.cycles[0].tilt_deg < 0.01              # measured 0.0


def test_oscillation_first_cycle_composes(osc_seed0):
    """The primitives compose the first LATERAL cycle WITH tilt engaged: the
    seeded A->B throw separates, is caught + held, and lands tight on B (~3.7 mm),
    seated near-centred. So the loop STARTS cleanly — the divergence below is not a
    failure to get going, it is a genuine loop-gain > 1."""
    c0 = osc_seed0.cycles[0]
    assert c0.separated
    assert c0.caught and c0.held_at_end
    assert c0.tilt_deg > 1.0
    assert c0.landing_err_mm < 15.0                      # measured ~3.7 mm
    assert c0.in_off_end_mm < 10.0                       # seated near-centred


@pytest.mark.xfail(strict=True, reason=(
    "Rung-2b MAKE-OR-BREAK (OPTION-1 re-plan): the two-point A<->B oscillation does "
    "NOT sustain either, WITH tilt engaged (~1.4 deg). It diverges within 1-4 "
    "cycles (loop gain > 1) at every separation/axis. The in-cup seat offset stays "
    "small (tilt keeps the ball centred) but the LANDING amplifies: the tilt-aimed "
    "throw is chaotically/deterministically sensitive to the throw-ORIGIN pose (a "
    "contact-detach knife-edge). Tilt fixes the band-limit but NOT this pose-chaos. "
    "GENUINE tilt-hypothesis failure -- pending operator go/no-go, see logbook/"
    "2026-07-01-rung2b-oscillation-tilt-engaged-diverges.md. Flip to XPASS if a "
    "future approach sustains >= 10 with tilt engaged."))
def test_oscillation_sustains_ten_cycles(osc_seed0):
    """HEADLINE capability (the gate): the two-point oscillation sustains >= 10
    cycles with loop gain < 1, WITH tilt engaged. Currently xfails (diverges within
    1-3 cycles despite tilt being active)."""
    assert osc_seed0.sustained >= 10


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4, 5])
def test_oscillation_does_not_sustain(seed):
    """The characterised oscillation BREAK, across seeds: with tilt engaged the
    A<->B loop does NOT sustain (it diverges within 1-3 cycles). No seed sustains
    anywhere near 10 -- so tilt engaging is not sufficient to close the loop."""
    r = run_self_catch(SelfCatchConfig(seed=seed, n_cycles=8, oscillate=True))
    assert r.cycles[0].tilt_deg > 1.0                   # tilt IS engaged
    assert r.sustained < 5                              # ...yet the loop diverges
    assert r.diverged


def test_oscillation_landing_amplifies_loop_gain_gt_one():
    """The loop-gain signature WITH tilt engaged (seed 1, a clear amplification
    case): the LANDING error amplifies from a tight first throw (~3.7 mm) past the
    catch's reliable reach within a couple of cycles (measured 3.7 -> 89 -> 728 mm)
    -- loop gain > 1. CRUCIALLY the in-cup SEAT offset stays small on the caught
    cycles (tilt keeps the ball centred): the amplified quantity is the LANDING
    (the throw's pose-sensitivity), not the seat offset -- the same distinction the
    column BREAK drew, now with tilt ACTIVE."""
    r = run_self_catch(SelfCatchConfig(seed=1, n_cycles=12, oscillate=True))
    errs = [c.landing_err_mm for c in r.cycles]
    assert errs[0] < 10.0                               # tight first throw (~3.7)
    assert max(errs) > CATCH_REACH_MM                   # amplifies past the reach
    assert max(errs) > 5.0 * errs[0]                    # several-fold amplification
    held_offs = [c.in_off_end_mm for c in r.cycles if c.caught and c.held_at_end]
    assert max(held_offs) < 5.0                         # seat offset stays small


def test_oscillation_throw_is_pose_sensitive():
    """The ROOT cause (deterministic, noise off): the tilt-aimed throw's landing is
    highly sensitive to the throw-ORIGIN position -- a 10 mm shift of the throw
    point swings the landing ~40 mm (dLanding/dOrigin ~4). This pose-chaos is the
    contact-detach knife-edge that gives the oscillation loop gain > 1; tilt does
    not address it. (Uses the Rung-2a throw harness directly.)"""
    off = NoiseConfig(bb_throw_noise_frac=0.0, tracking_noise_mm=0.0)
    l0 = np.array(run_single_throw(SingleThrowConfig(
        seed=0, throw_xy_m=(0.0, 0.0), target_xy_m=(0.0, 0.0), noise=off)).landing_xy_m)
    l1 = np.array(run_single_throw(SingleThrowConfig(
        seed=0, throw_xy_m=(-0.010, 0.0), target_xy_m=(0.0, 0.0), noise=off)).landing_xy_m)
    shift_mm = float(np.linalg.norm(l1 - l0) * 1000.0)
    assert shift_mm > 20.0                              # measured ~40.8 mm


def test_oscillation_is_deterministic_per_seed():
    """The oscillation loop is seed-reproducible (the §3 tracking noise is the only
    RNG): the same seed gives the same sustained count and the same first-cycle
    landing."""
    a = run_self_catch(SelfCatchConfig(seed=2, n_cycles=6, oscillate=True))
    b = run_self_catch(SelfCatchConfig(seed=2, n_cycles=6, oscillate=True))
    assert a.sustained == b.sustained
    assert a.cycles[0].landing_xy_mm == pytest.approx(b.cycles[0].landing_xy_mm, abs=1e-9)
    assert a.cycles[0].landing_err_mm == pytest.approx(b.cycles[0].landing_err_mm, abs=1e-9)
