"""W8 — adversarial sim sequences that exercise the 14 failure-mode scenarios.

Each scenario reproduces the geometry of one failure mode from the W1–W11
investigation (see ``controller/REFERENCE_LAYER_CONTRACT.md`` and
``logbook/2026-04-19-bundle-a-*.md``) against a live ``MuJoCoPlant`` through
the real ``run_mpc_loop``.

Pass criterion for every scenario:

* ``max(consecutive_non_success_run) ≤ 3`` — the MPC must not stall for
  more than 3 ticks in a row.  W1's make_feasible_events + W5's warm-start
  invalidation + W6's improved cold-start + W7's fallback hardening should
  together keep the solver converging through every adversarial transition.
* ``cold_hold`` status only permitted on the first solve (JIT warmup).
* Per-scenario tracking ceilings (tuned per scenario — see each test).

These tests are the structural-regression floor for the W1–W11 cycle: if
any future change breaks the K1–K6 contract or the fallback hardening, at
least one adversarial test fails.
"""

from __future__ import annotations

import numpy as np
import pytest

from plant.mujoco_plant import MuJoCoPlant
from controller.mpc import MPCController
from controller.params import MPCParams
from controller.target import (
    ReferenceEvent, TargetCommand, StaticTargetSource, TargetSource,
    make_feasible_events, pose_6dof_from_state,
)
from controller.plant import PlantState
from controller.telemetry import TelemetryLogger
from controller.runner import run_mpc_loop

CONTROL_DT = 0.025
V_MAX_SIM = 1000.0   # generous so sim solves easily when K1–K6 is in the loop
TAU = 0.04


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def plant():
    return MuJoCoPlant()


@pytest.fixture
def mpc(plant):
    params = MPCParams(
        max_cpu_time=2.0,       # sim — let IPOPT converge
        max_iter=500,
        max_leg_vel_mmps=V_MAX_SIM,
        prime_solver=False,
    )
    return MPCController.from_plant(params, plant)


# ---------------------------------------------------------------------------
# Helpers for scenario execution
# ---------------------------------------------------------------------------

class _ScriptedTargetSource:
    """Emits a schedule of (t_switch, TargetCommand) pairs through
    make_feasible_events.  Unlike StaticTargetSource, accepts arbitrary
    ref_events pre-built by the test."""

    def __init__(self, schedule, v_max_mmps, tau_s):
        self._schedule = schedule  # [(t_switch, target_pose, target_twist_or_None, arrival_or_None)]
        self._idx = 0
        self._cached_events = None
        self._cached_key = None
        self._v_max = v_max_mmps
        self._tau_s = tau_s
        self._prev_warm_valid = True

    def update(self, sim_time, state):
        # Advance schedule
        while (self._idx + 1 < len(self._schedule)
               and sim_time >= self._schedule[self._idx + 1][0]):
            self._idx += 1
        _, pose, twist, arrival = self._schedule[self._idx]
        key = (self._idx,)
        if key != self._cached_key:
            cur = pose_6dof_from_state(state)
            cur_tw = np.asarray(state.platform_twist, dtype=float)
            t_arr = sim_time + 0.6 if arrival is None else arrival
            proposal = [
                ReferenceEvent(time=sim_time, pose=cur, twist=cur_tw,
                               accel=np.zeros(6)),
                ReferenceEvent(
                    time=t_arr, pose=np.asarray(pose, dtype=float),
                    twist=(np.asarray(twist, dtype=float)
                           if twist is not None else np.zeros(6)),
                    accel=np.zeros(6),
                ),
            ]
            events, _reason = make_feasible_events(
                cur, cur_tw, proposal, sim_time,
                v_max_mmps=self._v_max, tau_s=self._tau_s,
            )
            self._cached_events = events
            self._cached_key = key
            warm_valid = False  # new target → caller hint to cold-start
        else:
            warm_valid = True
        return TargetCommand(
            target_pose=np.asarray(pose, dtype=float),
            arrival_time=arrival,
            target_twist=(np.asarray(twist, dtype=float)
                          if twist is not None else None),
            ref_events=self._cached_events,
            boost_vel_weights=True,
            warm_start_valid=warm_valid,
        )


def _run_and_measure(plant, mpc, source, duration):
    logger = TelemetryLogger()
    run_mpc_loop(plant, mpc, source, duration=duration,
                 logger=logger, control_dt=CONTROL_DT)
    records = logger.records
    statuses = [r.solve_status for r in records]
    success = [s in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')
               for s in statuses]
    # Max consecutive non-success run
    max_run = 0
    run = 0
    for s in success:
        if s:
            run = 0
        else:
            run += 1
            if run > max_run:
                max_run = run
    return records, max_run


def _assert_clean_run(records, max_consec_fallback,
                      *, max_consec_limit=3,
                      pos_tail_rms_limit_mm=5.0):
    # cold_hold only allowed on first solve
    for i, r in enumerate(records):
        if 'cold_hold' in r.solve_status and i > 0:
            raise AssertionError(
                f"cold_hold at sample {i}: {r.solve_status}"
            )
    assert max_consec_fallback <= max_consec_limit, (
        f"max consecutive non-success = {max_consec_fallback} "
        f"> limit {max_consec_limit}"
    )
    tail = records[-int(0.3 / CONTROL_DT):]
    pos_errs = []
    for r in tail:
        ref = np.array([r.ref_pose_x, r.ref_pose_y, r.ref_pose_z])
        act = np.array([r.actual_pose_x, r.actual_pose_y, r.actual_pose_z])
        pos_errs.append(np.linalg.norm(ref - act))
    if pos_errs:
        rms = float(np.sqrt(np.mean(np.square(pos_errs))))
        assert rms < pos_tail_rms_limit_mm, (
            f"tail position RMS {rms:.2f} mm > limit {pos_tail_rms_limit_mm}"
        )


def _spoof_platform_twist(plant, twist_6dof):
    """Override the MuJoCoPlant's next get_state() twist.  Used to seed a
    residual-velocity scenario without integrating sim physics to build it
    up — the geometric boundary condition is what matters for the W1 K2/K3
    enforcement, not the physics that produced it."""
    original = plant.get_state

    def _spoofed():
        s = original()
        return PlantState(
            leg_extensions_mm=s.leg_extensions_mm,
            leg_velocities_mmps=s.leg_velocities_mmps,
            platform_pos_mm=s.platform_pos_mm,
            platform_rot=s.platform_rot,
            platform_twist=np.array(twist_6dof, dtype=float),
            time=s.time,
            hand_pos_mm=s.hand_pos_mm,
            hand_vel_mmps=s.hand_vel_mmps,
            data_age_s=s.data_age_s,
        )
    plant.get_state = _spoofed
    return original


# ---------------------------------------------------------------------------
# Scenario 1 — Move 5: plant ahead, ref accelerating same direction
# ---------------------------------------------------------------------------

class TestScenario01_Move5(object):
    def test_off_active_upward_move_no_saturation(self, plant, mpc):
        """Plant at (8.57, 34.0, -0.7) off-Active moving up, target (0,0,50).

        This is the Move-5 geometry from 2026-04-18.  Pre-W1 the ref could
        step away from the plant; post-W1 the K1 anchor keeps ref[0] at
        plant pose and the quintic peak-v is within β·v_max.
        """
        plant.reset(pose_6dof=np.array([8.57, 34.0, -0.7, 0, 0, 0]))
        plant.step(0.025)
        source = StaticTargetSource(
            [(0.0, np.array([0.0, 0.0, 50.0, 0, 0, 0]))],
            v_max_mmps=V_MAX_SIM, tau_s=TAU,
        )
        records, max_run = _run_and_measure(plant, mpc, source, duration=1.5)
        _assert_clean_run(records, max_run, pos_tail_rms_limit_mm=3.0)


# ---------------------------------------------------------------------------
# Scenario 2 — 223902: target reversal with residual upward velocity
# ---------------------------------------------------------------------------

class TestScenario02_DirectionReversal(object):
    def test_mid_ramp_reversal_no_cascade(self, plant, mpc):
        """--sequence style: ramp to z=50, then reverse to z=0 mid-ramp.

        W1 (stretch), W5 (warm-start reset on reversal), and W7 (fallback
        plant-tracking on shift) together prevent the 147-fallback cascade
        observed in the 223902 hardware session.

        Core invariant: max_run == 0 (zero saturation, zero fallback) in
        sim with generous CPU budget.  Tracking RMS floor is lenient
        because the reversal intrinsically leaves residual error while
        the plant catches up to the new ref.
        """
        plant.reset(pose_6dof=np.zeros(6))
        plant.step(0.025)
        pose_up = np.array([0.0, 0.0, 50.0, 0, 0, 0])
        pose_down = np.array([0.0, 0.0, 0.0, 0, 0, 0])
        source = StaticTargetSource(
            [(0.0, pose_up), (1.2, pose_down)],
            v_max_mmps=V_MAX_SIM, tau_s=TAU,
        )
        records, max_run = _run_and_measure(plant, mpc, source, duration=3.0)
        # The PRIMARY regression we're defending against: cascade of
        # fallbacks.  Zero fallbacks is the bar for this scenario in sim.
        assert max_run == 0, (
            f"Direction-reversal cascade regressed: {max_run} consecutive "
            f"non-success"
        )
        # Tail RMS limit is generous — the reversal produces a natural
        # 5–8 mm settling transient that decays over ~0.5s.
        _assert_clean_run(records, max_run, pos_tail_rms_limit_mm=10.0)


# ---------------------------------------------------------------------------
# Scenario 3 — quintic peak-v exceeds v_max (the deep cause)
# ---------------------------------------------------------------------------

class TestScenario03_HighBoundaryVelocity(object):
    def test_boundary_velocity_caps_via_K6(self, plant, mpc):
        """A caller-supplied target_twist > v_max gets K6-clamped to ±v_max.
        Verified at the function level — here we just run a sim through it
        without K2/K3 saturation."""
        plant.reset()
        plant.step(0.025)
        # Sequence that would require peak |v| > v_max_sim without clamping
        source = StaticTargetSource(
            [(0.0, np.array([30.0, 0.0, 40.0, 0, 0, 0]))],
            v_max_mmps=V_MAX_SIM, tau_s=TAU,
        )
        records, max_run = _run_and_measure(plant, mpc, source, duration=1.5)
        _assert_clean_run(records, max_run, pos_tail_rms_limit_mm=3.0)


# ---------------------------------------------------------------------------
# Scenario 4 — peak accel would exceed actuator bandwidth
# ---------------------------------------------------------------------------

class TestScenario04_HighAccelNeed(object):
    def test_short_arrival_stretches_to_feasibility(self, plant, mpc):
        """Short arrival_time forces peak accel high — stretch kicks in."""
        plant.reset()
        plant.step(0.025)
        from controller.target import WaypointTargetSource
        # 80 mm move with 0.2s arrival → would need ~a_max*3 without stretch
        source = WaypointTargetSource(
            [(np.array([0.0, 0.0, 80.0, 0, 0, 0]), 0.2)],
            v_max_mmps=V_MAX_SIM, tau_s=TAU,
        )
        records, max_run = _run_and_measure(plant, mpc, source, duration=2.0)
        _assert_clean_run(records, max_run, pos_tail_rms_limit_mm=3.0)


# ---------------------------------------------------------------------------
# Scenario 5 — ref step-discontinuity (Bundle A territory, regression floor)
# ---------------------------------------------------------------------------

class TestScenario05_StepDiscontinuity(object):
    def test_ref_anchored_at_plant_not_target(self, plant, mpc):
        """K1 must anchor ref[0] at plant pose even when plant is off-Active."""
        plant.reset(pose_6dof=np.array([0, 0, 30, 0, 0, 0]))
        plant.step(0.025)
        source = StaticTargetSource(
            [(0.0, np.array([0.0, 0.0, 50.0, 0, 0, 0]))],
            v_max_mmps=V_MAX_SIM, tau_s=TAU,
        )
        records, max_run = _run_and_measure(plant, mpc, source, duration=1.5)
        _assert_clean_run(records, max_run, pos_tail_rms_limit_mm=3.0)
        # First record's ref should be near the plant's starting pose
        first = records[0]
        assert abs(first.ref_pose_z - 30.0) < 5.0, (
            f"K1 anchor broken: first ref_z={first.ref_pose_z}, want ~30"
        )


# ---------------------------------------------------------------------------
# Scenario 6 — warm-start structural incompatibility (W5 territory)
# ---------------------------------------------------------------------------

class TestScenario06_WarmStartShift(object):
    def test_large_target_shift_mid_run(self, plant, mpc):
        """50 mm target shift triggers W5 warm-start invalidation.
        The cold-start (W6) should absorb the transition within 3 ticks."""
        plant.reset()
        plant.step(0.025)
        source = StaticTargetSource(
            [(0.0, np.array([0.0, 0.0, 20.0, 0, 0, 0])),
             (0.6, np.array([0.0, 0.0, 70.0, 0, 0, 0]))],
            v_max_mmps=V_MAX_SIM, tau_s=TAU,
        )
        records, max_run = _run_and_measure(plant, mpc, source, duration=2.5)
        _assert_clean_run(records, max_run, pos_tail_rms_limit_mm=4.0)


# ---------------------------------------------------------------------------
# Scenario 7 — rapid successive target changes
# ---------------------------------------------------------------------------

class TestScenario07_RapidSuccession(object):
    def test_five_targets_in_two_seconds(self, plant, mpc):
        plant.reset()
        plant.step(0.025)
        targets = [
            (0.0, np.array([0, 0, 20, 0, 0, 0])),
            (0.4, np.array([0, 0, 50, 0, 0, 0])),
            (0.8, np.array([20, 0, 50, 0, 0, 0])),
            (1.2, np.array([20, 20, 30, 0, 0, 0])),
            (1.6, np.array([0, 0, 40, 0, 0, 0])),
        ]
        source = StaticTargetSource(
            targets, v_max_mmps=V_MAX_SIM, tau_s=TAU,
        )
        records, max_run = _run_and_measure(plant, mpc, source, duration=3.0)
        _assert_clean_run(records, max_run, pos_tail_rms_limit_mm=5.0)


# ---------------------------------------------------------------------------
# Scenario 8 — inconsistent twist across boundary (raises ValueError)
# ---------------------------------------------------------------------------

class TestScenario08_InconsistentTwist(object):
    def test_k5_inconsistent_twist_raises(self):
        """make_feasible_events must reject two events at same time with
        different twist — K5 invariant."""
        t_now = 0.0
        proposal = [
            ReferenceEvent(time=0.5, pose=np.array([0, 0, 50, 0, 0, 0]),
                           twist=np.array([0, 0, 30, 0, 0, 0]),
                           accel=np.zeros(6)),
            ReferenceEvent(time=0.5, pose=np.array([0, 0, 50, 0, 0, 0]),
                           twist=np.array([0, 0, -30, 0, 0, 0]),
                           accel=np.zeros(6)),
        ]
        with pytest.raises(ValueError, match="K5"):
            make_feasible_events(np.zeros(6), np.zeros(6), proposal, t_now,
                                 v_max_mmps=V_MAX_SIM, tau_s=TAU)


# ---------------------------------------------------------------------------
# Scenario 9 — stale plant snapshot (K1 overwrites)
# ---------------------------------------------------------------------------

class TestScenario09_StaleSnapshot(object):
    def test_live_state_used_not_proposal(self, plant, mpc):
        """Even if a proposal supplies a stale p0, K1 anchors at LIVE state."""
        plant.reset(pose_6dof=np.array([0, 0, 40, 0, 0, 0]))
        plant.step(0.025)
        # Build a stale proposal manually
        stale_p0 = np.array([0, 0, 0, 0, 0, 0])
        proposal = [
            ReferenceEvent(time=0.0, pose=stale_p0, twist=np.zeros(6),
                           accel=np.zeros(6)),
            ReferenceEvent(time=0.6, pose=np.array([0, 0, 70, 0, 0, 0]),
                           twist=np.zeros(6), accel=np.zeros(6)),
        ]
        state = plant.get_state()
        live_pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
        events, _reason = make_feasible_events(
            live_pose, state.platform_twist, proposal, 0.0,
            v_max_mmps=V_MAX_SIM, tau_s=TAU,
        )
        np.testing.assert_allclose(events[0].pose, live_pose, atol=1e-6)


# ---------------------------------------------------------------------------
# Scenario 10 — arrival_time in the past
# ---------------------------------------------------------------------------

class TestScenario10_PastArrival(object):
    def test_past_event_dropped_quintic_anchored_at_t_now(self):
        proposal = [
            ReferenceEvent(time=0.1, pose=np.array([0, 0, 20, 0, 0, 0]),
                           twist=np.zeros(6), accel=np.zeros(6)),
            ReferenceEvent(time=0.8, pose=np.array([0, 0, 30, 0, 0, 0]),
                           twist=np.zeros(6), accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            np.zeros(6), np.zeros(6), proposal, t_now=0.5,
            v_max_mmps=V_MAX_SIM, tau_s=TAU,
        )
        assert reason is None
        assert events[0].time == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# Scenario 11 — caller target_twist exceeds v_max
# ---------------------------------------------------------------------------

class TestScenario11_OversizedTargetTwist(object):
    def test_k6_clamps_terminal_twist(self):
        proposal = [
            ReferenceEvent(time=0.0, pose=np.zeros(6),
                           twist=np.zeros(6), accel=np.zeros(6)),
            ReferenceEvent(time=0.5, pose=np.array([0, 0, 50, 0, 0, 0]),
                           twist=np.array([2000, 0, 0, 0, 0, 0]),
                           accel=np.zeros(6)),
        ]
        events, _ = make_feasible_events(
            np.zeros(6), np.zeros(6), proposal, 0.0,
            v_max_mmps=V_MAX_SIM, tau_s=TAU,
        )
        # K6 clamp = β·v_max = 0.85 · 1000 = 850
        assert abs(events[-1].twist[0]) <= 0.85 * V_MAX_SIM + 1e-6


# ---------------------------------------------------------------------------
# Scenario 12 — FK-noise spike on start twist (Bundle A + K6)
# ---------------------------------------------------------------------------

class TestScenario12_FKNoiseSpike(object):
    def test_k6_clamps_start_twist_spike(self):
        fk_spike = np.array([2000, 0, 0, 0, 0, 0], dtype=float)
        proposal = [
            ReferenceEvent(time=0.0, pose=np.zeros(6), twist=fk_spike,
                           accel=np.zeros(6)),
            ReferenceEvent(time=0.6, pose=np.array([20, 0, 0, 0, 0, 0]),
                           twist=np.zeros(6), accel=np.zeros(6)),
        ]
        events, _ = make_feasible_events(
            np.zeros(6), fk_spike, proposal, 0.0,
            v_max_mmps=V_MAX_SIM, tau_s=TAU,
        )
        assert abs(events[0].twist[0]) <= 0.85 * V_MAX_SIM + 1e-6


# ---------------------------------------------------------------------------
# Scenario 13 — walk-forward on old plan after target flip (W7)
# ---------------------------------------------------------------------------

class TestScenario13_WalkForwardOldDir(object):
    def test_ref_mid_run_survives_cpu_pressure(self, plant, mpc):
        """Moderately-tight CPU budget + target reversal.

        The 223902 hardware failure was 147 consecutive timeouts with zero
        recovery: walk-forward kept marching an old upward-bound plan while
        the target flipped downward.  W7's plant-tracking swap-in (on large
        ref shift) should prevent any cascade longer than ~25 ticks in sim.

        The budget (18 ms) is chosen so that converged cold-start solves
        CAN happen (so W7's success-snapshot state gets armed) but the
        direction reversal creates enough solver pressure to exercise the
        fallback path.

        **Load-sensitivity note** (observed during Plan 2 Phase 6 work,
        run 2026-05-11): the 18 ms budget is tight enough that this
        test can flake on a busy Jetson — concurrent test invocations,
        hypothesis ci-deep runs, or system build pressure push enough
        solves into ``Maximum_CpuTime_Exceeded`` that the post-flip
        fallback distribution shifts and the ``ratio >= 0.5`` assertion
        fails.  The test passes reliably when run in isolation
        (``pytest tests/sim/test_mpc_adversarial_sequences.py::TestScenario13_WalkForwardOldDir::test_ref_mid_run_survives_cpu_pressure``)
        on an idle machine.  A failure on a loaded system does NOT
        indicate a regression in W7's plant-tracking swap-in — verify
        by re-running in isolation before treating as a real failure.
        """
        tight_params = MPCParams(
            max_cpu_time=0.018, max_iter=30,
            max_leg_vel_mmps=V_MAX_SIM, prime_solver=False,
        )
        tight_mpc = MPCController.from_plant(tight_params, plant)
        plant.reset()
        plant.step(0.025)
        source = StaticTargetSource(
            [(0.0, np.array([0, 0, 40, 0, 0, 0])),
             (0.6, np.array([0, 0, 0, 0, 0, 0]))],
            v_max_mmps=V_MAX_SIM, tau_s=TAU,
        )
        records, max_run = _run_and_measure(
            plant, tight_mpc, source, duration=2.5,
        )
        # The DIRECT test for W7 correctness: when the MPC is in a fallback
        # streak AND the reference has shifted structurally, the fallback
        # mode should switch from walk-forward (`fallback(…)`) to plant-
        # tracking (`hold_extrap(…)`).  This is what prevents the 223902
        # cascade where walk-forward marched an old upward plan into a
        # downward-flipped target.
        #
        # Inspect fallback statuses AFTER the target flip (which happens at
        # t=0.6s, sample ~24).
        flip_sample = int(0.6 / CONTROL_DT)
        post_flip_fallbacks = [
            r.solve_status for r in records[flip_sample:]
            if 'fallback' in r.solve_status or 'hold_extrap' in r.solve_status
        ]
        if post_flip_fallbacks:
            hold_extrap_count = sum(
                1 for s in post_flip_fallbacks if 'hold_extrap' in s
            )
            # Once W7 activates (large-shift detected), every subsequent
            # failure in the streak should use hold_extrap.  Require at
            # least 50% of post-flip fallbacks be hold_extrap as a firm
            # proof that W7's shift-detection is firing.
            ratio = hold_extrap_count / len(post_flip_fallbacks)
            assert ratio >= 0.5, (
                f"W7 large-shift guard not engaging: "
                f"{hold_extrap_count}/{len(post_flip_fallbacks)} "
                f"= {ratio:.0%} hold_extrap post-flip (want ≥50%)"
            )
        # Also assert the MPC eventually recovered — not permanently stuck.
        success_count = sum(
            1 for r in records
            if r.solve_status in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')
        )
        assert success_count >= 3, (
            f"Expected ≥3 successful solves; got {success_count} — "
            f"MPC is permanently stuck"
        )


# ---------------------------------------------------------------------------
# Scenario 14 — catch retarget: plant velocity opposite to target_twist
# ---------------------------------------------------------------------------

class TestScenario14_OpposingCatchRetarget(object):
    def test_catch_path_rejects_opposing_retarget(self):
        """no_stretch=True with plant falling + target rising = reject."""
        plant_twist = np.array([0, 0, -500, 0, 0, 0], dtype=float)
        proposal = [
            ReferenceEvent(time=0.0, pose=np.array([0, 0, 50, 0, 0, 0]),
                           twist=plant_twist, accel=np.zeros(6)),
            ReferenceEvent(time=0.1, pose=np.array([0, 0, 80, 0, 0, 0]),
                           twist=np.array([0, 0, 600, 0, 0, 0]),
                           accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            np.array([0, 0, 50, 0, 0, 0]), plant_twist, proposal, 0.0,
            v_max_mmps=V_MAX_SIM, tau_s=TAU, no_stretch=True,
        )
        assert reason is not None, (
            "Catch path must reject an opposing retarget — don't silently stretch"
        )

    def test_non_catch_path_stretches_instead(self):
        """Same geometry with no_stretch=False → stretch to feasibility."""
        plant_twist = np.array([0, 0, -500, 0, 0, 0], dtype=float)
        proposal = [
            ReferenceEvent(time=0.0, pose=np.array([0, 0, 50, 0, 0, 0]),
                           twist=plant_twist, accel=np.zeros(6)),
            ReferenceEvent(time=0.1, pose=np.array([0, 0, 80, 0, 0, 0]),
                           twist=np.array([0, 0, 600, 0, 0, 0]),
                           accel=np.zeros(6)),
        ]
        events, reason = make_feasible_events(
            np.array([0, 0, 50, 0, 0, 0]), plant_twist, proposal, 0.0,
            v_max_mmps=V_MAX_SIM, tau_s=TAU, no_stretch=False,
            max_stretch_ratio=10.0,
        )
        assert reason is None, (
            "Non-catch path should stretch rather than reject"
        )
        assert events[-1].time > 0.1 + 1e-6, "Stretch did not happen"
