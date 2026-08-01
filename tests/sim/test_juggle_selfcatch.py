"""Rung 2b — single-ball tilt-aimed self-catch loop: the GATE SMOKE.

Phase 3 of plans/active/bb-online-juggle-tilt-rearchitecture.md. Drives the real
contact-physics self-catch loop (``sim.juggle_selfcatch.run_self_catch``): compose
the Rung-2a tilt-aimed throw with the Rung-1 catch into a single-ball
toss->catch->toss loop, re-planned each cycle under the §3 tracking noise.

**This file is the per-commit half** (2026-08-01 split; the file was the xdist
critical path at 230.8 s -> ~61 s here). It keeps exactly what a commit must not
be allowed to break:

* the primitives COMPOSE (column + lateral first cycle, tilt engaged);
* the MAKE acceptance capability — ``test_oscillation_kinematic_release_sustains``
  at seed 0: with the kinematic release the A<->B oscillation sustains >= 10
  cycles with loop gain < 1;
* the loop-gain + real-seat-offset mechanism behind that MAKE;
* the MOTION QUALITY of the MAKE cycle (zero slider clamps, descending cup).

The BREAK characterizations (the column divergence, the detach-oscillation
divergence, their loop-gain signatures and pose-chaos root cause) and the full
6-seed robustness sweeps live in ``test_juggle_selfcatch_nightly.py`` — same
assertions, run every night by ``tools/nightly_suite.sh`` and by
``./run_tests.sh --full``, not on every commit. A regression that breaks the MAKE
reds the gate in minutes; a regression that accidentally FIXES a documented BREAK
surfaces overnight and is not a reason to block a commit.

KINEMATIC RELEASE (``release="kinematic"``) is the Rung-2b MAKE: cut the ball free
at the planned take-off velocity (``Ball.ballistic_release``), breaking hand
contact so no residual push re-corrupts it (open-loop knife-edge gain 2.7 -> 0.01).
Paired with the co-moving robust catch + the shorter carry dip (0.10). See
``logbook/2026-07-01-rung2b-kinematic-release.md``.

The loop is DETERMINISTIC per seed (the §3 noise is the only RNG and the runner
threads ``seed``), so the thresholds below carry margin over the measured
(2026-07-01) values.
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


# ============================================================================
# OSCILLATION (oscillate=True) — the operator's OPTION-1 re-plan: the HONEST tilt
# test on a NON-degenerate geometry. Shuttle a single ball A<->B, every throw
# lateral, tilt ENGAGED. The BREAK half (the shipped `detach` release diverges)
# is characterised in test_juggle_selfcatch_nightly.py.
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
    seated near-centred. So the loop STARTS cleanly — the divergence characterised
    in the nightly file is not a failure to get going, it is a genuine loop-gain
    > 1."""
    c0 = osc_seed0.cycles[0]
    assert c0.separated
    assert c0.caught and c0.held_at_end
    assert c0.tilt_deg > 1.0
    assert c0.landing_err_mm < 15.0                      # measured ~3.7 mm
    assert c0.in_off_end_mm < 10.0                       # seated near-centred


# ============================================================================
# KINEMATIC RELEASE (release="kinematic") — the Rung-2b MAKE.
# ============================================================================

def _kin_cfg(seed, n_cycles=12, **kw):
    """The Rung-2b MAKE config: kinematic release + the tuned carry dip (0.10)."""
    return SelfCatchConfig(seed=seed, n_cycles=n_cycles, oscillate=True,
                           release="kinematic", dip_m=0.10, **kw)


@pytest.fixture(scope="module")
def kin_seed0():
    return run_self_catch(_kin_cfg(0))


@pytest.mark.parametrize("seed", [0])
def test_oscillation_kinematic_release_sustains(seed):
    """HEADLINE capability (the gate MAKE): with the KINEMATIC release the default
    A<->B oscillation (x-40, tilt ~1.4 deg) sustains the full 12 cycles -- loop
    gain < 1, the divergence killed. This is the make-or-break MAKE (contrast
    ``test_oscillation_does_not_sustain`` in the nightly file, the detach BREAK on
    the same geometry). Measured 12/12 on all 6 seeds (2026-07-01).

    Kept parametrized (over seed 0 alone) rather than de-parametrized, so the
    node id stays ``...::test_oscillation_kinematic_release_sustains[0]`` — the
    id every logbook entry and plan since 2026-07-01 refers to. The full [0..5]
    sweep runs nightly as
    ``test_oscillation_kinematic_release_sustains_all_seeds``: seed 0 is the
    capability claim, the other five are its robustness evidence, and only the
    claim needs to gate a commit."""
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


def test_kinematic_release_is_deterministic_per_seed():
    """The kinematic MAKE is seed-reproducible (the §3 tracking noise is the only
    RNG): the same seed gives the same sustained count and first-cycle landing.
    Stays per-commit because it is what makes the single-seed MAKE above a
    trustworthy gate — a non-deterministic loop would turn seed 0 into a coin
    flip."""
    a = run_self_catch(_kin_cfg(3, n_cycles=6))
    b = run_self_catch(_kin_cfg(3, n_cycles=6))
    assert a.sustained == b.sustained
    assert a.cycles[0].landing_xy_mm == pytest.approx(b.cycles[0].landing_xy_mm, abs=1e-9)


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
