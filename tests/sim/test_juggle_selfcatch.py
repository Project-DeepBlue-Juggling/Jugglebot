"""Rung 2b — single-ball tilt-aimed self-catch loop (the MAKE-OR-BREAK gate).

Phase 3 of plans/active/bb-online-juggle-tilt-rearchitecture.md. Drives the real
contact-physics self-catch loop (``sim.juggle_selfcatch.run_self_catch``): compose
the Rung-2a tilt-aimed throw with the Rung-1 catch into a single-ball
toss->catch->toss loop, re-planned each cycle under the §3 tracking noise.

These tests pin two things:

* the primitives DO compose for a single cycle — the first throw separates and is
  caught + held, landing within the catch's reliable reach (the Rung-2a column
  result carries into the loop); and
* the headline capability — a self-catch that **sustains >= 10 cycles with loop
  gain < 1** — is **not achieved**: the pure column self-catch diverges (the
  ``xfail(strict=True)`` below), for the column-specific reasons characterised in
  ``logbook/2026-07-01-rung2b-selfcatch-column-divergence.md`` (a caught, centred,
  spinless ball does not cleanly detach on a column, and the reach amplifies each
  cycle — loop gain > 1). Tilt is ~0 for a column, so the tilt mechanism never
  engages; the column is a degenerate case where tilt cannot help.

The loop is DETERMINISTIC per seed (the §3 noise is the only RNG and the runner
threads ``seed``), so the thresholds below carry margin over the measured
(2026-07-01) values and the divergence is seed-reproducible.
"""
import numpy as np
import pytest

from sim.juggle_selfcatch import (
    run_self_catch, SelfCatchConfig, CATCH_REACH_MM,
)


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
