"""End-to-end sim integration tests for the BB-led two-ball juggle demo.

Validates the Phase 3 §5 integration test cases — T-I3 (full juggle reaches
30+ catches), T-I4 (scheduling accuracy), T-I5 (priming transient meshes
into steady state), T-I6 (abort mid-pattern yields a smooth exit). Each
test runs the standalone runner in :mod:`sim.juggle_demo` against MuJoCo
in headless mode.

These tests are deliberately slow — a 30 s sim run takes ~12 s wall
time on the Jetson and exercises the full Phase 2 + 3a + 3b + 3c stack
(optimiser, multi-ball plant, master timeline, exit transient,
trajectory player, hand sequences, BallButlerSim). The IPOPT solve at
the runner's startup is shared across tests via a module-scoped
fixture; per-test re-solves are not needed.

Plan reference: ``plans/active/bb-led-two-ball-juggle-demo.md`` §5
integration tests.
"""
import pytest

from sim.juggle_demo import (
    JuggleDemoConfig, _JuggleDemoRunner, _optimise_cache, run,
)


# --------------------------------------------------------------------------
# T-I3: sustained catches in MuJoCo — the plan §3 Phase 3 exit criterion.
#
# 2026-06-27 state (throw-aim diagnosis + closed-loop catch). The throw-aim
# problem was DIAGNOSED to root and the 2026-06-26 "angular tracking" headline
# was REFUTED: the same-instant cup-velocity decomposition shows ω×r tracks
# fine and contributes <0.04 m/s, while the dominant miss is a PLATFORM BAND
# LIMIT — the heavy connect-constraint platform low-pass-filters every fast
# maneuver (~30 mm+ tracking error at the throw AND the catch), so the emergent
# contact throw lands ~100-140 mm from the planned catch. The old demo only hit
# 30 catches because the kinematic velocity-override BYPASSED the platform.
# Mitigations landed: optimiser throw-knot twist penalty (throw error 249→137
# mm/s), CALIB 1.08→1.0 (vertical), and a CLOSED-LOOP CATCH that observes the
# in-flight ball and reaches the platform xy to meet it. Result: the demo now
# catches BOTH balls (the short test below PASSES, un-xfailed). The full
# ≥30-catch pattern is still blocked by a band-limit cascade (loose re-seat
# after the fast BB catch → wild re-throw; the reach is itself band-limited;
# multi-ball reach/collision) — so T-I3 stays xfail. See logbook
# 2026-06-27-throw-aim-band-limit-and-closed-loop-catch.
# --------------------------------------------------------------------------
@pytest.mark.xfail(strict=True, reason=(
    "Throw-aim is a platform BAND LIMIT (not angular tracking): the emergent "
    "contact throw lands ~100-140 mm from the planned catch and the full "
    "≥30-catch pattern is blocked by a band-limit cascade (loose re-seat → "
    "wild re-throw, band-limited reach, multi-ball collision). Closed-loop "
    "catch gets to 2 catches. See logbook "
    "2026-06-27-throw-aim-band-limit-and-closed-loop-catch."))
def test_full_sim_juggle_reaches_target_catches():
    """T-I3: the demo sustains the pattern past the 30-catch threshold.

    Runs the default JuggleDemoConfig (real contact mechanics; seat-based
    capture; closed-loop catch; leg-jerk² + leg-vel-equalise optimiser with
    throw-knot twist penalty, sep=200) for 30 s and asserts >= 30 catches.
    Currently xfails at 2 catches pending the band-limit cascade fix (see
    module header).
    """
    stats = run(JuggleDemoConfig(duration_s=30.0, n_catches=32, seed=0))
    assert stats.n_captures >= 30, (
        f"only {stats.n_captures} catches in 30 s — Phase 3 exit "
        f"criterion is >= 30. Stats: {stats}")
    assert stats.n_drops == 0, (
        f"unexpected drops in the 30 s run: {stats.drops}")


# --------------------------------------------------------------------------
# Smoke test — a short run lands the BB-primed first catch + at least one
# Jugglebot throw. Fast (~3 s wall time); catches regressions in the
# wiring before the slow T-I3 test runs.
#
# 2026-06-27: un-xfailed. The closed-loop catch (observe the in-flight ball,
# reach the platform xy to meet it) now seats BOTH balls — the BB-primed ball 0
# AND the first emergent Jugglebot throw (ball 1) — so this passes. The full
# ≥30-catch pattern (T-I3 above) is still blocked by the band-limit cascade.
# --------------------------------------------------------------------------
def test_short_run_catches_the_bb_primed_ball_and_a_throw():
    """A 5 s run picks up the BB-primed catch and at least one downstream catch.

    Both balls are caught: the BB-primed ball 0 and the first emergent hand
    throw (ball 1, caught via the closed-loop catch reach). See module header.
    """
    stats = run(JuggleDemoConfig(duration_s=5.0, n_catches=8, seed=0))
    assert stats.n_captures >= 2, (
        f"5 s run captured only {stats.n_captures} balls — expected "
        f"the BB-primed catch + at least one Jugglebot throw. Stats: {stats}")
    # Both balls should have been touched (the BB-primed ball-0 catch
    # and the pre-held ball-1 thrown by Jugglebot at t=t0).
    captured_balls = {ball for _t, ball in stats.captures}
    assert captured_balls == {0, 1}, (
        f"expected both balls to be captured in 5 s, got balls {captured_balls}")


# --------------------------------------------------------------------------
# T-I6: abort mid-pattern produces no further catches and the timeline's
# future events are dropped.
# --------------------------------------------------------------------------
def test_abort_mid_pattern_stops_future_throws():
    """T-I6: abort at sim_time 3 s; no Jugglebot throw events fire after.

    The catch count after abort is bounded by what had already been
    caught by sim_time = 3 s; the test just confirms the run completes
    cleanly with ``aborted=True`` and no drops.
    """
    stats = run(JuggleDemoConfig(duration_s=5.0, abort_at_s=3.0, seed=0))
    assert stats.aborted is True
    assert stats.abort_time_s is not None
    assert stats.abort_time_s == pytest.approx(3.0, abs=0.1)
    assert stats.n_drops == 0
    # T-I6's exit criterion is a *smooth exit to stow*, not a hard
    # capture-count ceiling: any throw whose prelude already began
    # before the abort instant still releases its ball, and that ball
    # lands at the catch position roughly one flight-time later — the
    # hand may happen to be at the right z (still sitting where the
    # cancelled catch sequence's final state placed it) and catch the
    # ball. The abort path drops the *future* schedule (no new throws
    # past abort, no unstarted catches), so post-abort captures are
    # bounded by the throws whose stroke had begun by ``abort_at_s``.
    # On 2026-05-23 a 3 s abort produces at most 2 such captures —
    # leave generous headroom.
    captures_after_abort = [t for t, _ in stats.captures if t > 3.1]
    assert len(captures_after_abort) <= 4, (
        f"too many catches well past the abort instant: {captures_after_abort}")


# --------------------------------------------------------------------------
# Determinism — same seed → same capture stream (cheap regression catch).
# --------------------------------------------------------------------------
def test_runner_is_deterministic_with_fixed_seed():
    """Two runs with the same config produce the same captures and drops.

    Scatter is disabled (``bb_scatter_mm=0``): :class:`BallButlerSim`'s
    scatter uses an unseeded ``np.random.default_rng()`` and is not
    deterministic without changes to that module. Without scatter the
    runner is deterministic by construction (MuJoCo + the deterministic
    optimiser + the deterministic event schedule), and this test
    catches regressions that introduce inadvertent non-determinism.
    """
    cfg = JuggleDemoConfig(duration_s=5.0, n_catches=8, seed=0,
                           bb_scatter_mm=0.0)
    a = run(cfg)
    # Clear the runner's optimiser cache so the second run does a fresh
    # IPOPT solve — without this the second run gets a cached result
    # and we only test downstream determinism (MuJoCo + master timeline
    # + hand sequences), not optimiser determinism.
    _optimise_cache.clear()
    b = run(cfg)
    assert a.captures == b.captures
    assert a.drops == b.drops
    assert a.events_dispatched == b.events_dispatched


# --------------------------------------------------------------------------
# White-box: confirm the runner schedules events in time order.
# --------------------------------------------------------------------------
def test_runner_schedules_events_in_time_order():
    """All ``events_dispatched`` come from the master timeline in t_wall order."""
    cfg = JuggleDemoConfig(duration_s=3.0, n_catches=4, seed=0)
    runner = _JuggleDemoRunner(cfg)
    try:
        runner.run()
    finally:
        runner.close()
    # The timeline has 1 BB + 4 throws + 4 catches = 9 events. In a 3 s
    # run we should dispatch at least the first few; check ordering of
    # all_events (not just dispatched) as a structural invariant.
    ts = [e.t_wall for e in runner.timeline.all_events()]
    assert ts == sorted(ts), "timeline events are not in t_wall order"
