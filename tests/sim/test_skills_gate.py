"""Skill-stack sim gate -- ``sim/skills_gate.py`` (plan § 4 R2, skill-stack).

Replaces ``tests/sim/test_cycle_gate.py`` and ``tests/sim/test_unified_gate.py``
(both deleted the same rung, 2026-09-12, Unit E): those covered the SUPERSEDED
FSM/unified-cycle sim gates; this covers the current one -- ``SkillExecutor``
driving the real emitter/pump/wire/mirror chain (``sim/stream_chain.py``)
against the MuJoCo plant.

**Marker policy: UNMARKED (per-commit)**, matching the two deleted gates' own
policy: every hop this file drives is on the hardware-safety surface
(``teensy_link`` wire bytes, the ``SetpointPump`` gates, the firmware
interpolation ladder, ``motion/``'s planner and the skill-stack executor) --
none of which is a ``nightly`` demotion candidate.  Cost is contained the same
way: the module-scoped smoke fixture runs a SHORT columns schedule
(``n_throws=4``, one seed), and the full 20-throw x 5-seed sweep runs manually
via ``python sim/skills_gate.py``.
"""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip('mujoco')

from jugglebot.motion.geometry import StewartGeometry            # noqa: E402
from jugglebot.motion.skills import schedule as sk                # noqa: E402
from jugglebot.motion.skills import sites                         # noqa: E402
from jugglebot.motion.skills.segments import (                    # noqa: E402
    THROW, SegmentConfig, ThrowTerminal, plan_segment,
)
from jugglebot.motion.trajectory import ballistics_bc as bal      # noqa: E402

import sim.skills_gate as sg                                      # noqa: E402
from sim import stream_chain                                      # noqa: E402
from sim.juggle_noise import BallisticEstimator                   # noqa: E402
from sim.skills_gate import (                                     # noqa: E402
    MIRROR_TOL_HAND_REV, MIRROR_TOL_LEG_REV, SkillsGate,
    SkillsGateConfig, SelfTossGateConfig, _WANT_FLAGS,
)
import teensy_interp as ti                                        # noqa: E402


@pytest.fixture(scope='module')
def gate():
    return SkillsGate(SkillsGateConfig(seeds=(0,), n_throws=4))


@pytest.fixture(scope='module')
def smoke_report(gate):
    """ONE short columns run shared by the assertions (MuJoCo is the slow
    part): four throws = 1 pre-existing catch + 3 matching catches."""
    return gate.run()


@pytest.fixture(scope='module')
def throw_plan(gate):
    """A standalone THROW segment plan, off the gate's own limits/geom --
    the hand-lane tests' fixture, ported to build against ``segments.
    plan_segment`` + ``stream_chain`` rather than a whole-gate plan.
    """
    site0, site1 = sites.columns_sites(gate.cfg.separation_mm)
    seed = gate._rest_state(site0)
    t_f = sk.flight_s(gate.cfg.apex_m)
    terminal = ThrowTerminal(site_mm=site0.throw_site_mm(),
                             target_mm=site1.catch_site_mm(),
                             flight_s=t_f, t_release_s=0.4)
    seg = plan_segment(THROW, seed, terminal, SegmentConfig(), gate.limits,
                       gate.geom)
    return seg.plan


# ── the gate's verdict ───────────────────────────────────────────────────────

def test_the_smoke_run_is_not_vacuous_and_never_drops(smoke_report):
    """The gate's own verdict on the smoke schedule: every scheduled catch made,
    zero drops, no refusal -- and not vacuous (four catches happened).

    Two findings shaped this assertion, both recorded in
    ``logbook/2026-09-12-skill-stack-r2-skills-schedule-stream.md``:

    * Unit E's original build (a THROW spliced one knot after a rest-terminal
      CATCH) ended every run ``LIMIT_JERK`` at install 2 -- resolved by the
      CATCH-with-throw segment (Unit G), which this run exercises;
    * with the carried ball physically released (Unit H2), the model's DEFAULT
      2 % throw-velocity scatter (the Ball Butler's, a documented placeholder)
      drifts a landing ~155 mm and the catch is refused before motion --
      resolved by the owner's decision (2026-09-12) that R2 certifies the
      software chain with an EXACT release and the 0.5 mm tracking noise, and
      that the machine's own scatter is measured at R3 (the gate's module
      docstring carries the separation-vs-scatter table).

    MEASURED (seed 0, ``n_throws=4``, 2026-09-12, deterministic): 4/4 catches,
    0 drops, ``attempt_ended`` False.
    """
    assert len(smoke_report['trials']) == 1
    t = smoke_report['trials'][0]
    assert t['scheduled_catches'] == 4
    assert t['makes'] == t['scheduled_catches'] == 4
    assert t['drops'] == 0
    assert t['attempt_ended'] is False, t['end_code']
    assert smoke_report['passed'] is True


def test_every_emitted_frame_is_pump_accepted(smoke_report):
    """``pump_rejects == 0`` AND ``accepted == emitted`` -- the non-vacuous
    form: a gate that only counted rejects would pass on a stream that
    emitted nothing."""
    t = smoke_report['trials'][0]
    assert t['pump_rejects'] == 0
    assert t['pump_frames_emitted'] > 0
    assert t['pump_frames_accepted'] == t['pump_frames_emitted']
    assert t['pump_clean'] is True


def test_every_accepted_frame_carries_the_hand_and_v1_flags(smoke_report):
    """The flag discipline: a frame that quietly dropped ``HAS_HAND`` would
    leave the firmware's hand lane free-running on its decay ladder."""
    t = smoke_report['trials'][0]
    assert t['flags_seen'] == [hex(_WANT_FLAGS)]
    assert t['flags_ok'] is True


def test_the_firmware_mirror_reconstructs_the_plan(smoke_report):
    """The 500 Hz ladder rebuilds what the planner asked for, on both
    channels, within the bounds carried over from ``unified_gate``."""
    t = smoke_report['trials'][0]
    assert t['mirror_hand_worst_rev'] <= MIRROR_TOL_HAND_REV
    assert t['mirror_leg_worst_rev'] <= MIRROR_TOL_LEG_REV
    assert t['mirror_ok'] is True


def test_the_plan_wall_time_is_reported(smoke_report):
    """The sim-side twin of the < 50 ms Jetson install gate: min/p50/max over
    every accepted install in the run."""
    pw = smoke_report['plan_wall_ms']
    assert pw['n'] > 0
    assert 0.0 <= pw['min'] <= pw['p50'] <= pw['max']


# ── the hand lane (ported from ``tests/sim/test_unified_gate.py``) ─────────

def test_the_hand_lane_decays_on_the_falling_edge(throw_plan, gate):
    """Cut the stream mid-stroke: the lane winds down, it does not hold.

    The NORMATIVE rule (canbridge_config.h's FW 17 block): when ``HAS_HAND``
    falls while the hand is MOVING, the lane must run its Hermite segment out,
    Taylor-extrapolate from the segment ENDPOINT and decay the velocity to
    zero -- never hold Mode 1's ``s = 1`` endpoint, which would keep
    commanding it with ``vel_ff = v1`` from up to 200 rev/s.

    Cut deliberately at the knot of PEAK hand speed, so a lane that held
    would hold the worst possible feedforward.
    """
    d = stream_chain.hand_decay_probe(throw_plan, gate.geom)
    assert d['ok'] is True
    assert abs(d['v_at_cut_rps']) > 10.0, (
        'the cut landed on a stationary hand (%.3f rev/s) -- the probe would '
        'observe nothing' % d['v_at_cut_rps'])
    assert d['age_left_mode1_s'] is not None, (
        'the hand lane never left Mode 1 -- it is holding the segment '
        'endpoint and its up-to-200 rev/s feedforward, which is the exact '
        'failure the falling-edge rule forbids')
    assert ti.SEGMENT_T_S < d['age_left_mode1_s'] <= (ti.SEGMENT_T_S
                                                      + d['sampling_slack_s']
                                                      + 1e-9), (
        'left Mode 1 at %.4f s, not at the %.3f s segment boundary'
        % (d['age_left_mode1_s'], ti.SEGMENT_T_S))
    assert d['monotone_after_mode1'] is True
    assert d['age_velocity_zero_s'] is not None
    assert d['age_velocity_zero_s'] <= (d['decay_deadline_s']
                                        + d['sampling_slack_s'] + 1e-9)
    assert d['final_vel_rps'] == 0.0
    assert abs(d['travel_after_cut_rev']) > d['max_lead_hand_rev'], (
        'the raw wind-down (%.4f rev) stayed inside the %.1f rev clamp band, '
        'so the clamp is not what bounded it and the next assertion is '
        'vacuous' % (d['travel_after_cut_rev'], d['max_lead_hand_rev']))
    assert d['lead_clamp_ticks'] > 0, (
        'the hand lead clamp never engaged during the wind-down')
    assert abs(d['clamped_travel_after_cut_rev']) <= d['max_lead_hand_rev'] \
        + 1e-6


def test_the_hand_lane_is_inert_until_a_has_hand_frame_latches():
    """``tick_hand`` transmits NOTHING before the first HAS_HAND frame.

    The firmware's ``s_hand_active`` gate.  A mirror that emitted a 0.0 rev
    command from an unlatched lane would command the hand to the retract
    stop.
    """
    mirror = stream_chain.make_mirror(StewartGeometry())
    assert mirror.tick_hand(0.0, 5.0) is None
    mirror.latch_hand(5.0, 0.0, 0.0, u1=5.1, u2=5.2, v1=4.0)
    out = mirror.tick_hand(0.0, 5.0)
    assert out is not None and out[0] == pytest.approx(5.0)


def test_the_hand_lane_skips_an_axis_that_has_never_reported_an_encoder():
    """``fb_rev is None`` => no command, counted -- the firmware's
    unseen-skip.  0.0 rev is a real, reachable, WRONG hand position, so an
    axis that has never reported must not be commanded at all.
    """
    mirror = stream_chain.make_mirror(StewartGeometry())
    mirror.latch_hand(5.0, 0.0, 0.0, u1=5.1, u2=5.2, v1=4.0)
    assert mirror.tick_hand(0.0, None) is None
    assert mirror.hand_unseen_skips == 1
    assert mirror.hand_raw_pos == pytest.approx(5.0)


def test_tracker_landing_time_is_anchored_to_the_last_sample_not_now():
    """R3-j (2026-09-13): ``_make_tracker``'s ``t_land_abs_s`` must be
    ``last_obs_t + t_rem``, never ``plant.data.time + t_rem``.

    ``ballistics_bc.arrival_state_at_z``'s ``t_rem`` is time-to-touchdown
    from the fit's OWN reference instant -- ``BallisticEstimator.estimate()``
    evaluates at ``Δt = 0`` at its LATEST sample time, not "now". Anchoring to
    ``plant.data.time`` instead means a frozen fit (sampling stopped, e.g. the
    ball just caught) drifts its reported landing later by exactly the ticks
    elapsed since the last sample -- measured on a self-toss LAUNCH throw
    whose catch closed ~25 ms before the fit's own predicted crossing: the
    drift accumulated over the ~94 ms to ``executor.CAUGHT_WINDOW_S``
    finalisation reported a flight 96 ms too long (0.9548 s against a true
    ~0.859 s, ``python sim/skills_gate.py --learn --policy B --seeds 0``).
    This test freezes the estimator (no new samples) and asserts the
    predicted landing instant does not move when "now" advances alone.
    """
    g = np.asarray(bal.G_VEC_MMS2, dtype=float)
    est = BallisticEstimator(g)
    t_ref = 5.0
    z0, vz0 = 1200.0, -3000.0          # mm, mm/s -- descending toward CATCH_CUP_Z_MM
    for dt in (-0.010, -0.005, 0.0):   # three clean samples ending AT t_ref
        t = t_ref + dt
        z = z0 + vz0 * dt + 0.5 * float(g[2]) * dt ** 2
        est.add(t, np.array([0.0, 0.0, z]))
    ball_state = {0: dict(estimator=est, last_obs_t=t_ref, airborne=True)}

    class _Data:
        time = t_ref

    class _FakePlant:
        data = _Data()

    plant = _FakePlant()
    tracker = sg._make_tracker(plant, ball_state)

    landing_now = tracker(0)
    assert landing_now is not None

    # "Now" advances 0.5 s with NO new sample added (sampling has stopped,
    # e.g. the ball was just caught) -- the frozen fit's predicted landing
    # instant must not move with it.
    plant.data.time = t_ref + 0.5
    landing_later = tracker(0)
    assert landing_later.t_land_abs_s == pytest.approx(
        landing_now.t_land_abs_s, abs=1e-9)


def test_tracker_returns_none_while_the_ball_is_not_airborne():
    """R3-n (2026-09-13, ``/tmp/probe_handoff_capture_v3.py`` /
    ``/tmp/probe_handoff_flip_v3.py``): once a ball is caught, sampling stops
    but the estimator still holds >= 3 samples from the flight that just
    ended, so an un-gated tracker keeps handing out that FROZEN landing —
    aimed at a flight already over. ``_make_tracker`` must return ``None``
    while ``bstate['airborne']`` is False, matching the robot tracker (whose
    per-ball correlation the executor gates at ``_valid_tracked_landing``)."""
    g = np.asarray(bal.G_VEC_MMS2, dtype=float)
    est = BallisticEstimator(g)
    t_ref = 5.0
    z0, vz0 = 1200.0, -3000.0
    for dt in (-0.010, -0.005, 0.0):
        t = t_ref + dt
        z = z0 + vz0 * dt + 0.5 * float(g[2]) * dt ** 2
        est.add(t, np.array([0.0, 0.0, z]))
    ball_state = {0: dict(estimator=est, last_obs_t=t_ref, airborne=True)}

    class _Data:
        time = t_ref

    class _FakePlant:
        data = _Data()

    plant = _FakePlant()
    tracker = sg._make_tracker(plant, ball_state)

    assert tracker(0) is not None       # airborne, 3 samples -- has a landing

    ball_state[0]['airborne'] = False   # caught -- sampling stops
    assert tracker(0) is None


# ── R3: the self-toss learner run (plan § 4 R3 "Sim validation") ───────────
#
# A SMALL smoke run only -- one seed, a handful of throws -- exercising the
# real learner + Memory + AdmissibleBox + observer/observations chain
# end-to-end without paying for the gate's own 25-throw x 5-seed sweep (that
# one runs manually via ``python sim/skills_gate.py --learn``, same split as
# the columns gate above).  MEASURED (2026-09-13, this test, seed 0, 5
# throws): every attempt in this schedule produces exactly ONE throw before
# ending ``NO_LANDING`` (the single-ball self-toss's catch-with-throw
# dispatches ``HANDOFF_LEAD_S`` = 0.2 s BEFORE its own ball's release, so the
# tracker never has 3 samples yet -- a real gap in ``compile_self_toss`` for
# a schedule with no second ball to buy the slack columns has; not this
# unit's file to fix, see the logbook entry) -- so ``max_attempts`` is sized
# for one throw per attempt, not for a multi-throw chain landing in one.

def _small_self_toss_run():
    cfg = SelfTossGateConfig(target_throws=5, band_entry_throws=5,
                             max_attempts=15)
    return cfg, SkillsGate(cfg).run_self_toss_seed(0, policy='A')


def test_a_small_self_toss_learner_run_enters_the_flight_band():
    """From a cold memory, a handful of self-toss throws land the FLIGHT error
    inside the R3 band (20 ms) well within the 5-throw entry criterion, with
    no drops -- the same learner, memory, admissible-box and
    observer/observations wiring the full 25-throw x 5-seed gate uses,
    exercised cheaply."""
    cfg, res = _small_self_toss_run()
    assert res['n_throws_collected'] == 5
    assert res['drops'] == 0
    assert res['throws_to_band_flight'] is not None
    assert res['throws_to_band_flight'] <= cfg.band_entry_throws
    assert res['throws'][-1]['err_flight_ms'] <= cfg.flight_band_s * 1000.0
    assert all(t['caught'] for t in res['throws'])


@pytest.mark.xfail(strict=True, reason=(
    'known defect, plans/active/two-ball-skill-stack.md R3 item (k): the '
    'dense apex-scoped box (2026-09-14) admits no y landing correction at '
    '0.9 m because the chained catch fails its 90 % margin for small offsets '
    'near 0.77/0.81 s flights; the sim aim error is +y, so xy cannot enter '
    'the band until that planner margin ring is fixed and the box re-swept'))
def test_a_small_self_toss_learner_run_enters_the_xy_band():
    """The xy half of the R3 band (20 mm) within the 5-throw entry criterion.
    Strict xfail: it turns into a failure the day xy authority returns, so
    the marker cannot outlive the fix."""
    cfg, res = _small_self_toss_run()
    assert res['throws_to_band_xy'] is not None
    assert res['throws_to_band_xy'] <= cfg.band_entry_throws
    assert res['throws'][-1]['err_xy_mm'] <= cfg.xy_band_mm
