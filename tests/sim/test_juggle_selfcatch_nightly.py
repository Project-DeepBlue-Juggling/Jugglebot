"""Rung 2b self-catch — the BREAK characterizations + the full 6-seed sweeps.

**Nightly tier** (`pytestmark = nightly`; runs at 04:00 via `tools/nightly_suite.sh`
and on `./run_tests.sh --full`). Split out of ``test_juggle_selfcatch.py`` on
2026-08-01 — that file was the xdist critical path at 230.8 s; moving this half
out took the per-commit cost to 61.0 s. Nothing was deleted: every assertion
below still runs, once a night instead of once a commit. (This file measures
190.5 s standalone, slightly more than the 169.9 s it removed, because the two
files now build their module fixtures independently and the seed-0 kinematic MAKE
runs in both. Nightly wall-clock is not a constraint; gate wall-clock is.)

**What lives here vs. in the gate file.** The boundary is drawn on CONTENT, not on
runtime: this is research *characterization* — records of behaviours the sim does
NOT have (the two BREAKs) and the seed-robustness sweeps that generalise them.
The MAKE acceptance capability (``test_oscillation_kinematic_release_sustains``,
seed 0) stays per-commit in ``test_juggle_selfcatch.py``, because that is the
capability the rung claims. A regression that breaks the MAKE is caught in the
gate within minutes; a regression that accidentally FIXES a documented BREAK is
caught overnight, and "the sim got better" is not a reason to block a commit.

The two characterised BREAKs:

COLUMN (``oscillate=False``, the original 2026-07-01 gate) — the pure column
self-catch does not sustain: a caught, centred, spinless ball does not cleanly
detach on a column and the reach amplifies each cycle (loop gain > 1). Tilt is ~0
for a column, so the tilt mechanism never engages; the column is a degenerate case
where tilt cannot help. See
``logbook/2026-07-01-rung2b-selfcatch-column-divergence.md``.

OSCILLATION + ``detach`` (the operator's OPTION-1 re-plan, the HONEST tilt test on
a NON-degenerate geometry) — diverges within 1-4 cycles even with tilt engaged
(~1.4 deg). The in-cup SEAT offset stays small (tilt keeps the ball centred) but
the LANDING amplifies. Root cause: the tilt-aimed throw's landing is chaotically
sensitive to the throw-ORIGIN pose (a contact-detach knife-edge). Tilt fixes the
band-limit but NOT this pose-chaos. See
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
from sim.juggle_catch import SEAT_RADIUS_MM


pytestmark = pytest.mark.nightly


def _kin_cfg(seed, n_cycles=12, **kw):
    """The Rung-2b MAKE config: kinematic release + the tuned carry dip (0.10).
    Mirrors the same helper in ``test_juggle_selfcatch.py`` — the two files are
    deliberately independent so either can be run alone."""
    return SelfCatchConfig(seed=seed, n_cycles=n_cycles, oscillate=True,
                           release="kinematic", dip_m=0.10, **kw)


# ============================================================================
# COLUMN (oscillate=False) — the original BREAK.
# ============================================================================

@pytest.fixture(scope="module")
def faithful_seed0():
    """The faithful composition (Rung-2a plan_cup_cycle recover, stationary
    column) — the primary make-or-break configuration."""
    return run_self_catch(SelfCatchConfig(seed=0, n_cycles=12))


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


#: The seed whose column loop breaks via REACH AMPLIFICATION rather than via the
#: catch-seat knife edge. Mode selection is a property of the catch CONTACT, not
#: of the seed: it was seed 0 under the 2026-07-02 firmed contact and became
#: seed 3 on 2026-08-21 when the ball radius was corrected 35.0 -> 37.0 mm
#: (owner caliper). Measured over seeds 0-5 that day: 3 amplifies (6.92 ->
#: 105.41 -> 211.66 mm), 0/1/2/4/5 break at cycle 1 with a sub-0.3 mm reach.
#: Every seed still diverges either way.
_AMPLIFY_SEED = 3


def test_reach_amplifies_loop_gain_gt_one():
    """The loop-gain signature (a clear reach-amplification case): the
    catch reach AMPLIFIES from the tight first throw (~7.7 mm) past the catch's
    ~60-80 mm reliable reach within a couple of cycles (measured 7.7 -> 106.7 mm)
    -- loop gain > 1, the same class as the pre-tilt divergence (logbook 2026-06-27).

    Which seed shows the reach-amplification mode (vs the catch-seat-knife-edge
    mode) depends on the catch contact: under the 2026-07-02 firmed catch contact
    seed 0 amplified (7.7 -> 106.7 mm) while seeds 1-5 broke at cycle 1 via the
    knife-edge (tiny cycle-1 reach). Every seed still DIVERGES either way
    (``test_column_selfcatch_does_not_sustain``); this test pins the amplification
    mode specifically. See logbook/2026-07-02-fast-catch-fidelity.md.

    **THE SEED MOVED 2026-08-21, and the docstring above predicted exactly why.**
    Correcting the ball radius (``physics.juggling_ball_radius_mm`` 35.0 -> 37.0,
    owner caliper) is a change to the CATCH CONTACT, and mode selection is
    contact-dependent — so seed 0 flipped to the knife-edge (cycle-1 reach
    **0.09 mm**, the tiny reach this docstring names) and **seed 3** now carries
    the amplification mode (6.92 -> 105.41 -> 211.66 mm, measured 2026-08-21 over
    seeds 0-5). Nothing about the BREAK changed: all six seeds still diverge,
    sustained <= 2 of 12. The test follows the mode rather than the seed number,
    which is what it was always for."""
    r = run_self_catch(SelfCatchConfig(seed=_AMPLIFY_SEED, n_cycles=12))
    reaches = [c.reach_mm for c in r.cycles]
    assert reaches[0] < 20.0                       # tight first throw (~6.9 mm)
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
# OSCILLATION + detach (oscillate=True, release="detach") — the tilt-engaged BREAK.
# ============================================================================

@pytest.fixture(scope="module")
def osc_seed0():
    """The default oscillation (x-axis, A=(-20,0) <-> B=(20,0), 40 mm, tilt ~1.4
    deg) — the primary make-or-break configuration for the tilt hypothesis."""
    return run_self_catch(SelfCatchConfig(seed=0, n_cycles=12, oscillate=True))


def test_oscillation_detach_does_not_sustain(osc_seed0):
    """HEADLINE (the BREAK half): with the shipped ``detach`` throw the two-point
    oscillation does NOT sustain even WITH tilt engaged (~1.4 deg) -- it diverges
    within 1-4 cycles (loop gain > 1). Tilt fixes the band-limit but NOT the
    contact-detach's chaotic sensitivity to the throw-ORIGIN pose (see
    ``test_oscillation_throw_is_pose_sensitive``). The MAKE half is
    ``test_oscillation_kinematic_release_sustains`` in the gate file."""
    assert osc_seed0.cycles[0].tilt_deg > 1.0           # tilt IS engaged
    assert osc_seed0.sustained < 5                      # ...yet detach diverges


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4, 5])
def test_oscillation_does_not_sustain(seed):
    """The characterised oscillation BREAK, across seeds: with the ``detach`` throw
    the A<->B loop does NOT sustain (it diverges within 1-3 cycles). No seed
    sustains anywhere near 10 -- tilt engaging is not sufficient with the emergent
    contact throw; the kinematic release is what closes it (below)."""
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
    # The FIRST-CYCLE bound is the cup seat radius, not a pinned "measured"
    # value: correcting the ball radius (35.0 -> 37.0 mm, owner caliper,
    # 2026-08-21) moved this cycle's landing error 3.7 -> 28.2 mm, and sweeping
    # the radius 33-38 mm that day gave 13.2 / 23.2 / 3.7 / 10.6 / 28.2 / 39.9 --
    # non-monotonic, so the 3.7 was one draw of a chaotic quantity. What this
    # test is ABOUT is the amplification below, and that is untouched: measured
    # 2026-08-21 the oscillation still runs 28.2 -> ~470 mm on every seed 0-5,
    # a 16-17x amplification past the 80 mm reliable reach (453.9-492.6 mm
    # across seeds 0-5; the factor is a range, not one number).
    assert errs[0] < SEAT_RADIUS_MM                     # measured 28.2 mm
    assert max(errs) > CATCH_REACH_MM                   # amplifies past the reach
    assert max(errs) > 5.0 * errs[0]                    # several-fold amplification
    held_offs = [c.in_off_end_mm for c in r.cycles if c.caught and c.held_at_end]
    assert max(held_offs) < 5.0                         # seat offset stays small


def test_oscillation_throw_is_pose_sensitive():
    """The ROOT cause (deterministic, noise off): the tilt-aimed throw's landing is
    highly sensitive to the throw-ORIGIN position -- a small shift of the throw
    point swings the landing several times as far (dLanding/dOrigin up to ~4).
    This pose-chaos is the contact-detach knife-edge that gives the oscillation
    loop gain > 1; tilt does not address it. (Uses the Rung-2a throw harness
    directly.)

    **Probed over FOUR origin shifts since 2026-08-21, not one.** The old form
    probed a single −10 mm shift and asserted ``> 20 mm`` on a "measured
    ~40.8 mm". Correcting the ball radius (35.0 -> 37.0 mm, owner caliper) made
    that one probe read **4.29 mm** — and the map it samples is precisely the
    chaotic one this test exists to characterise, so pinning one direction of it
    was pinning a coin flip. Measured that day at ±10 / ±20 mm the landing
    shifts are **4.29 / 43.55 / 34.09 / 39.26 mm**, i.e. per-mm gains of
    **0.43 / 4.35 / 1.70 / 1.96**.

    The two assertions below say what "chaotic" actually means and are far more
    robust than the old single probe: SOME direction swings the landing well past
    its own origin shift, and the gain is strongly NON-UNIFORM across directions
    (a smooth, well-behaved throw would have all four gains alike)."""
    off = NoiseConfig(bb_throw_noise_frac=0.0, tracking_noise_mm=0.0)

    def _landing(dx_m):
        return np.array(run_single_throw(SingleThrowConfig(
            seed=0, throw_xy_m=(dx_m, 0.0), target_xy_m=(0.0, 0.0),
            noise=off)).landing_xy_m)

    l0 = _landing(0.0)
    gains = []
    for dx_mm in (-10.0, 10.0, -20.0, 20.0):
        shift_mm = float(np.linalg.norm(_landing(dx_mm / 1000.0) - l0) * 1000.0)
        gains.append(shift_mm / abs(dx_mm))
    assert max(gains) > 3.0, (
        'no probed direction amplifies the origin shift — the throw is no '
        'longer pose-chaotic, which would be a FIX to the documented BREAK: '
        'gains {}'.format(['%.2f' % g for g in gains]))
    assert max(gains) > 3.0 * min(gains), (
        'the pose sensitivity is uniform across directions, i.e. smooth rather '
        'than knife-edge: gains {}'.format(['%.2f' % g for g in gains]))


def test_oscillation_is_deterministic_per_seed():
    """The oscillation loop is seed-reproducible (the §3 tracking noise is the only
    RNG): the same seed gives the same sustained count and the same first-cycle
    landing."""
    a = run_self_catch(SelfCatchConfig(seed=2, n_cycles=6, oscillate=True))
    b = run_self_catch(SelfCatchConfig(seed=2, n_cycles=6, oscillate=True))
    assert a.sustained == b.sustained
    assert a.cycles[0].landing_xy_mm == pytest.approx(b.cycles[0].landing_xy_mm, abs=1e-9)
    assert a.cycles[0].landing_err_mm == pytest.approx(b.cycles[0].landing_err_mm, abs=1e-9)


def test_detach_mode_is_byte_identical_default():
    """The ``release`` field defaults to ``detach``: the shipped oscillation is
    unchanged by the Rung-2b additions (still the characterised BREAK). Guards the
    'tilt=0 / Rung-1 / Rung-2a / detach paths stay byte-identical' invariant."""
    default = run_self_catch(SelfCatchConfig(seed=0, n_cycles=6, oscillate=True))
    explicit = run_self_catch(SelfCatchConfig(seed=0, n_cycles=6, oscillate=True,
                                              release="detach"))
    assert default.sustained == explicit.sustained
    assert default.cycles[0].landing_xy_mm == pytest.approx(
        explicit.cycles[0].landing_xy_mm, abs=1e-12)


# ============================================================================
# KINEMATIC RELEASE — the FULL seed sweep + the head-to-head. Seed 0 of the MAKE
# stays per-commit in test_juggle_selfcatch.py; this is the robustness half.
# ============================================================================

@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4, 5])
def test_oscillation_kinematic_release_sustains_all_seeds(seed):
    """The MAKE's seed-robustness sweep (the gate runs seed 0 of this as
    ``test_oscillation_kinematic_release_sustains``): with the KINEMATIC release
    the default A<->B oscillation (x-40, tilt ~1.4 deg) sustains the full 12 cycles
    on EVERY seed -- loop gain < 1, the divergence killed. Measured 12/12 on all 6
    seeds (2026-07-01)."""
    r = run_self_catch(_kin_cfg(seed))
    assert r.cycles[0].tilt_deg > 1.0                   # tilt IS engaged
    assert r.sustained >= 10                            # measured 12/12
    assert not r.diverged


def test_kinematic_beats_detach_head_to_head():
    """Head-to-head, same geometry + seed: the kinematic release SUSTAINS where the
    shipped detach throw DIVERGES -- the single-variable demonstration that the
    knife-edge (not tilt, not the catch) was the binding failure. Nightly because
    half of it is the detach BREAK."""
    seed = 4
    detach = run_self_catch(SelfCatchConfig(seed=seed, n_cycles=12, oscillate=True))
    kin = run_self_catch(_kin_cfg(seed))
    assert detach.sustained < 5                         # detach diverges
    assert kin.sustained >= 10                          # kinematic sustains
    assert kin.sustained > 2 * detach.sustained
