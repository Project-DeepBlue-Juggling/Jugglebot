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

import pytest

pytest.importorskip('mujoco')

from jugglebot.motion.geometry import StewartGeometry            # noqa: E402
from jugglebot.motion.skills import schedule as sk                # noqa: E402
from jugglebot.motion.skills import sites                         # noqa: E402
from jugglebot.motion.skills.segments import (                    # noqa: E402
    THROW, SegmentConfig, ThrowTerminal, plan_segment,
)

from sim import stream_chain                                      # noqa: E402
from sim.skills_gate import (                                     # noqa: E402
    MIRROR_TOL_HAND_REV, MIRROR_TOL_LEG_REV, SkillsGate,
    SkillsGateConfig, _WANT_FLAGS,
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
