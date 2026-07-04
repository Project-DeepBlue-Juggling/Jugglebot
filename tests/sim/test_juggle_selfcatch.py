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


def test_reach_amplifies_loop_gain_gt_one(faithful_seed0):
    """The loop-gain signature (seed 0, a clear reach-amplification case): the
    catch reach AMPLIFIES from the tight first throw (~7.7 mm) past the catch's
    ~60-80 mm reliable reach within a couple of cycles (measured 7.7 -> 106.7 mm)
    -- loop gain > 1, the same class as the pre-tilt divergence (logbook 2026-06-27).

    Which seed shows the reach-amplification mode (vs the catch-seat-knife-edge
    mode) depends on the catch contact: under the 2026-07-02 firmed catch contact
    seed 0 amplifies (7.7 -> 106.7 mm) while seeds 1-5 break at cycle 1 via the
    knife-edge (tiny cycle-1 reach). Every seed still DIVERGES either way
    (``test_column_selfcatch_does_not_sustain``); this test pins the amplification
    mode specifically. See logbook/2026-07-02-fast-catch-fidelity.md."""
    reaches = [c.reach_mm for c in faithful_seed0.cycles]
    assert reaches[0] < 20.0                       # tight first throw (~7.7 mm)
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


def test_oscillation_detach_does_not_sustain(osc_seed0):
    """HEADLINE (the BREAK half): with the shipped ``detach`` throw the two-point
    oscillation does NOT sustain even WITH tilt engaged (~1.4 deg) -- it diverges
    within 1-4 cycles (loop gain > 1). Tilt fixes the band-limit but NOT the
    contact-detach's chaotic sensitivity to the throw-ORIGIN pose (see
    ``test_oscillation_throw_is_pose_sensitive``). The MAKE half is
    ``test_oscillation_kinematic_release_sustains`` below."""
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


# ============================================================================
# KINEMATIC RELEASE (release="kinematic") — the Rung-2b MAKE. Cut the ball free at
# the planned take-off velocity (Ball.ballistic_release), breaking hand contact so
# no residual push re-corrupts it (open-loop knife-edge gain 2.7 -> 0.01). Paired
# with the co-moving robust catch + the shorter carry dip (0.10). RESULT: the A<->B
# oscillation SUSTAINS >= 10 cycles with loop gain < 1, the REAL contact-carry seat
# offset rejected -- see logbook/2026-07-01-rung2b-kinematic-release.md.
# ============================================================================

def _kin_cfg(seed, n_cycles=12, **kw):
    """The Rung-2b MAKE config: kinematic release + the tuned carry dip (0.10)."""
    return SelfCatchConfig(seed=seed, n_cycles=n_cycles, oscillate=True,
                           release="kinematic", dip_m=0.10, **kw)


@pytest.fixture(scope="module")
def kin_seed0():
    return run_self_catch(_kin_cfg(0))


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4, 5])
def test_oscillation_kinematic_release_sustains(seed):
    """HEADLINE capability (the gate MAKE): with the KINEMATIC release the default
    A<->B oscillation (x-40, tilt ~1.4 deg) sustains the full 12 cycles on every
    seed -- loop gain < 1, the divergence killed. This is the make-or-break MAKE
    (contrast ``test_oscillation_does_not_sustain``, the detach BREAK, same
    geometry). Measured 12/12 on all 6 seeds (2026-07-01)."""
    r = run_self_catch(_kin_cfg(seed))
    assert r.cycles[0].tilt_deg > 1.0                   # tilt IS engaged
    assert r.sustained >= 10                            # measured 12/12
    assert not r.diverged


def test_kinematic_release_loop_gain_below_one(kin_seed0):
    """The loop-gain signature (the MAKE): the per-cycle in-cup seat offset is FLAT
    (does not amplify) -- the opposite of the detach BREAK's 3.7 -> 89 -> 728 mm
    landing blow-up. Every cycle is caught + held with a small, bounded offset."""
    assert kin_seed0.sustained >= 10
    offs = [c.in_off_end_mm for c in kin_seed0.cycles]
    assert max(offs) < 3.0                              # measured ~0.5-0.8 mm, flat
    assert max(offs) <= 3.0 * offs[0] + 1.0             # bounded (no amplification)
    errs = [c.landing_err_mm for c in kin_seed0.cycles]
    assert max(errs) < 15.0                             # landing flat too (~3-6 mm)


def test_kinematic_release_rejects_real_seat_offset(kin_seed0):
    """The REAL disturbance is rejected: the ball physically SEATS (a contact-carry
    seat, not a kinematic snap) at a real, non-zero in-cup depth every caught cycle,
    yet the loop holds -- the seat offset is a genuine physical disturbance the loop
    rejects, not a zeroed-out artefact."""
    caught = [c for c in kin_seed0.cycles if c.caught]
    assert len(caught) >= 10
    # every caught cycle registered a physical seat (a real settle, not a teleport)
    assert all(c.seat_offset_mm > 0.0 for c in caught)


def test_kinematic_beats_detach_head_to_head():
    """Head-to-head, same geometry + seed: the kinematic release SUSTAINS where the
    shipped detach throw DIVERGES -- the single-variable demonstration that the
    knife-edge (not tilt, not the catch) was the binding failure."""
    seed = 4
    detach = run_self_catch(SelfCatchConfig(seed=seed, n_cycles=12, oscillate=True))
    kin = run_self_catch(_kin_cfg(seed))
    assert detach.sustained < 5                         # detach diverges
    assert kin.sustained >= 10                          # kinematic sustains
    assert kin.sustained > 2 * detach.sustained


def test_kinematic_release_is_deterministic_per_seed():
    """The kinematic MAKE is seed-reproducible (the §3 tracking noise is the only
    RNG): the same seed gives the same sustained count and first-cycle landing."""
    a = run_self_catch(_kin_cfg(3, n_cycles=6))
    b = run_self_catch(_kin_cfg(3, n_cycles=6))
    assert a.sustained == b.sustained
    assert a.cycles[0].landing_xy_mm == pytest.approx(b.cycles[0].landing_xy_mm, abs=1e-9)


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
# MOTION QUALITY — the co-design catch (continuous velocity-matched sub-tick
# command). The kinematic MAKE cycle must be SMOOTH, not the old ceiling-slam:
# ZERO slider clamps, the ball received on a DESCENDING cup, and a cup path
# materially shorter than the ~1026 mm/cycle jump-and-settle. See
# logbook/2026-07-03-catch-control-formulation-design-basis.md.
# ============================================================================

def test_kinematic_catch_motion_is_smooth():
    """The design-basis fix, gated: the continuous velocity-matched catch (+ the
    low-velocity-release carry it enables) makes the self-catch MAKE cycle SMOOTH.

    Instruments the cup (``hand_opening`` site) + slider vs the true ball each
    control tick and asserts the motion-quality signals the 2026-07-03 review
    called for and the constant-per-tick jump-and-settle catch FAILED:

      * ZERO slider-clamp ticks (was 93 ceiling ticks — the ceiling overshoot the
        old catch needed to build downward runway into a stiff position actuator);
      * cup moving DOWN at every ball contact (was +9..+11 mm/s UP — the ball fell
        onto a parked, upward-drifting cup);
      * cup vertical path per cycle materially shorter than the old ~1026 mm.

    A regression to the constant-per-tick command (or the v_takeoff carry) trips
    at least one of these even while the MAKE still passes — outcome metrics alone
    let a full-stroke slam score perfectly (the review's core lesson).
    """
    import mujoco
    from sim.juggle_selfcatch import (
        SelfCatchRunner, SelfCatchConfig, SLIDER_STROKE_MM)

    n_cycles = 3
    runner = SelfCatchRunner(SelfCatchConfig(
        oscillate=True, n_cycles=n_cycles, seed=0, release="kinematic", dip_m=0.10))
    plant = runner.plant
    sid = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')
    jid = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_JOINT, 'hand_slide')
    sadr = plant.model.jnt_qposadr[jid]
    log = []
    orig_step = plant.step

    def spy(*a, **k):
        orig_step(*a, **k)
        vel6 = np.zeros(6)
        mujoco.mj_objectVelocity(plant.model, plant.data,
                                 mujoco.mjtObj.mjOBJ_SITE, sid, vel6, 0)
        log.append((float(plant.data.qpos[sadr]) * 1000.0,          # slider mm
                    float(plant.data.site_xpos[sid][2]) * 1000.0,    # cup z mm
                    float(vel6[5]) * 1000.0,                         # cup vz mm/s
                    bool(plant.get_ball_state(0).held)))

    plant.step = spy
    res = runner.run()
    assert res.sustained >= n_cycles                       # all cycles make

    sliders = np.array([x[0] for x in log])
    cz = np.array([x[1] for x in log])
    cvz = [x[2] for x in log]
    held = [x[3] for x in log]

    ceil = int(np.sum(sliders >= SLIDER_STROKE_MM - 1.0))
    floor = int(np.sum(sliders <= 1.0))
    assert ceil == 0 and floor == 0, f"slider clamp hits: ceil={ceil} floor={floor}"

    path_per_cycle = float(np.sum(np.abs(np.diff(cz)))) / n_cycles
    assert path_per_cycle < 550.0, (                       # measured ~322 vs old ~1026
        f"cup path {path_per_cycle:.0f} mm/cycle not materially reduced (~halved)")

    contacts = [cvz[max(0, i - 2)] for i in range(1, len(log))
                if held[i] and not held[i - 1]]
    assert contacts, "no ball-contact transition observed"
    assert all(v < 0.0 for v in contacts), (               # measured ~-669 vs old +10
        f"cup not moving DOWN at contact (received on a rising/parked cup): {contacts}")
