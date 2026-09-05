"""Unified-mode sim gate — ``sim/unified_gate.py`` (T-I2).

Plan: ``plans/active/unified-7dof-planner.md`` § 4 Phase 4 and § 5 T-I2.

**Marker policy: UNMARKED (per-commit)**, matching ``tests/sim/test_toss_gate.py``
and ``tests/sim/test_cycle_gate.py``.  The demotion rule is on CONTENT, not
runtime, and every hop this file drives is on the hardware-safety surface —
``teensy_link`` wire bytes, the ``SetpointPump`` gates, the firmware
interpolation ladder, ``motion/``'s planner — none of which is a ``nightly``
demotion candidate.  Cost is contained the way ``toss_gate``'s smoke contains
it instead: the module-scoped fixture runs a TWO-point subset (one co-located
toss, one one-cycle ring), and the full 27-point grid runs manually via
``python sim/unified_gate.py``.  The heavier questions (the non-vacuity of the
reconstruction band, the falling-edge ladder, the flag discipline) are answered
by PURE replays with no plant at all.

The load-bearing claims asserted here:

* the gate's own verdict on the subset, and that the subset is not vacuous;
* every emitted frame is pump-accepted (``accepted == emitted``, the
  non-vacuous form) with zero rejects;
* every accepted frame carries ``HAS_U1|HAS_U2|HAS_HAND|HAS_V1`` — the flag
  discipline that stands in for ``toss_gate``'s ``catch_armed`` band;
* the firmware mirror reconstructs the plan within the two bounds, AND that
  band is non-vacuous: masking ``HAS_V1`` off the wire moves the hand
  reconstruction more than three orders past its bound (measured: four), and
  moves the LEG reconstruction past its own bound on the displaced ring;
* the hand lane's falling-edge decay: cut mid-stroke, the lane leaves Mode 1,
  winds down monotonically and reaches EXACTLY zero velocity inside the
  firmware's own deadline — and the raw wind-down travel is bounded only by
  ``MAX_LEAD_HAND_REV``;
* the ball is caught and held on every planned catch;
* the two-pose ring's release instants are exactly periodic.

Reported but deliberately NOT asserted, mirroring the two sibling gates: the
MuJoCo contact detail (``capture_dist_mm``, makes/drops), the executed-cup
tracking error, and the stream phase error — all three are characterisations of
the plant or the link, not contracts.
"""

from __future__ import annotations

import dataclasses
import json
import math

import numpy as np
import pytest

pytest.importorskip('mujoco')

from teensy_link.protocol import Setpoint                        # noqa: E402
from teensy_link.setpoint_pump import (                          # noqa: E402
    FLAG_HAS_HAND, FLAG_HAS_U1, FLAG_HAS_U2, FLAG_HAS_V1,
)

from jugglebot.motion.geometry import StewartGeometry           # noqa: E402
from jugglebot.motion.trajectory import KnotEmitter              # noqa: E402

from sim.gate_common import (                                     # noqa: E402
    HOLD_TILT_DEG, HOLD_TRAVEL_MM, SEPARATION_MS,
)

from sim.unified_gate import (                                   # noqa: E402
    BEAT_TOL_S, CYCLE_PERIOD_S, LAUNCH_PERIOD_S, MIRROR_TOL_HAND_REV,
    MIRROR_TOL_LEG_REV, PREPOSITION_TOL_MM, THROW_CUP_Z_MM, UnifiedGate,
    UnifiedGateConfig, _latch, _make_mirror, _make_pump, _pass_threshold,
    _point_id, beat_grid, default_grid, hand_decay_probe, run_gate,
    single_toss_grid,
)
import teensy_interp as ti                                       # noqa: E402

_WANT_FLAGS = FLAG_HAS_U1 | FLAG_HAS_U2 | FLAG_HAS_HAND | FLAG_HAS_V1

#: One co-located single toss at the shipped default flight, and a ONE-cycle
#: ring (the shortest thing that still contains a LAUNCH, a STEADY and a
#: LANDING, i.e. all three window kinds and two chained ``extend`` seams).
_SMOKE_POINTS = [
    dict(kind='toss', xy_mm=(0.0, 0.0), z_offset_mm=0.0, flight_s=0.60,
         catch_dxy_mm=(0.0, 0.0), advisory=False),
    dict(kind='ring', displacement_mm=60.0, flight_s=0.60, steady_cycles=1,
         advisory=False),
]


@pytest.fixture(scope='module')
def gate():
    return UnifiedGate(UnifiedGateConfig(points=_SMOKE_POINTS,
                                         decay_probe=False))


@pytest.fixture(scope='module')
def smoke_report():
    """ONE small-N run shared by the assertions (MuJoCo is the slow part)."""
    cfg = UnifiedGateConfig(points=_SMOKE_POINTS, seed=0)
    return UnifiedGate(cfg).run()


@pytest.fixture(scope='module')
def smoke_plan(gate):
    """The single-toss plan, reused by the pure (no-plant) replays."""
    return gate.plan_single_toss(_SMOKE_POINTS[0])


@pytest.fixture(scope='module')
def smoke_ring_plan(gate):
    """The one-cycle ring plan — the point where the PLATFORM actually moves.

    The co-located toss barely displaces the platform, so its leg lanes are
    nearly linear over a knot and the ``HAS_V1`` fallback reproduces them almost
    exactly; the ring is where a leg-lane regression has room to show.
    """
    return gate.plan_ring(_SMOKE_POINTS[1])


# ── the gate's verdict ───────────────────────────────────────────────────────

def test_the_smoke_subset_passes_and_is_not_vacuous(smoke_report):
    """A PASS that came from real accepted cycles, not from an empty band."""
    assert smoke_report['passed'] is True
    assert smoke_report['single_toss']['ran'] is True
    assert smoke_report['constant_beat']['ran'] is True
    assert smoke_report['accepted'] == len(_SMOKE_POINTS)
    assert smoke_report['core_clean'] == len(_SMOKE_POINTS)
    assert smoke_report['rejected'] == []
    assert smoke_report['invariants']['all_ok'] is True


def test_every_emitted_frame_is_pump_accepted(smoke_report):
    """``pump_rejects == 0`` AND ``accepted == emitted`` — the non-vacuous form.

    A gate that counted only rejects would pass just as happily on a stream
    that emitted nothing; ``toss_gate`` records that lesson and this is the same
    invariant, on a stream that additionally carries a hand lane through the
    pump's independent per-channel gate.
    """
    for t in smoke_report['results']:
        assert t['pump_rejects'] == 0, (t['point_id'], t['pump_rejects'])
        assert t['pump_frames_emitted'] > 0
        assert t['pump_frames_accepted'] == t['pump_frames_emitted']
        assert t['pump_clean'] is True


def test_every_accepted_frame_carries_the_hand_and_v1_flags(smoke_report):
    """The flag discipline that replaces ``toss_gate``'s ``catch_armed`` band.

    A frame that quietly dropped ``HAS_HAND`` would leave the firmware's hand
    lane free-running on its decay ladder while the legs kept streaming — the
    exact failure the NORMATIVE falling-edge rule exists to bound, and one no
    other band here would see.  Every frame off a ``CyclePlan`` must carry the
    full set.
    """
    for t in smoke_report['results']:
        assert t['flags_seen'] == [hex(_WANT_FLAGS)], (t['point_id'],
                                                       t['flags_seen'])
        assert t['flags_ok'] is True


def test_the_firmware_mirror_reconstructs_the_plan(smoke_report):
    """The 500 Hz ladder rebuilds what the planner asked for, on both channels.

    The two bounds have different provenance and are asserted separately: the
    hand is EXACTLY reconstructible (the same Hermite from the same knots, so
    only the wire's float32 remains) while the legs are not (a rev-space cubic
    through IK'd knots is not the IK of a pose-space cubic in between).  See
    the two constants' docstrings for the measured numbers.
    """
    for t in smoke_report['results']:
        assert t['mirror_hand_worst_rev'] <= MIRROR_TOL_HAND_REV, (
            t['point_id'], t['mirror_hand_worst_rev'])
        assert t['mirror_leg_worst_rev'] <= MIRROR_TOL_LEG_REV, (
            t['point_id'], t['mirror_leg_worst_rev'])
        assert t['mirror_ok'] is True
    # The hand bound really is at the float32 floor, not merely satisfied.  The
    # bar is derived from the band rather than restated: MIRROR_TOL_HAND_REV's
    # own analysis puts the arithmetic floor at ONE float32 half-ulp of the
    # ~10 rev operating point (4.77e-7 rev) and sets the band at ~8x that, so
    # half the band is 4x the floor — the tightest bar the analysis supports.
    # A literal here (it was 1e-6) drifts against the constant it is judging:
    # the two disagreed until 2026-09-04, the constant claiming a ~2e-6 floor
    # the test's own bar sat below.  MEASURED worst over the full 27-point grid
    # (`python sim/unified_gate.py --no-viewer`, run 2026-09-05, with the
    # production settle site): 4.766e-07 rev.
    assert smoke_report['worst_mirror_hand_rev'] < MIRROR_TOL_HAND_REV / 2.0


def test_the_mirror_band_is_non_vacuous(smoke_plan, smoke_ring_plan, gate):
    """Break the ``HAS_V1`` rule on the wire and BOTH bands must react.

    Empirical-probe rule: a tolerance nothing can violate is not a band.  This
    replays the SAME plans through the SAME emitter and pump, then masks
    ``HAS_V1`` out of the decoded frame before latching — which is exactly what
    a pre-v6 board, or a producer that forgot ``vel_next_mm_s``, would present.
    The firmware then falls back to the ``(u2−u1)/T`` forward difference, whose
    endpoint velocity is not the plan's.

    MEASURED (re-measured 2026-09-05 against the PRODUCTION settle site, which
    moved the toss's hand row and left the ring's three untouched):

    ==================  ==============  ==============  ==============
    plan / lane         honest          HAS_V1 masked   band
    ==================  ==============  ==============  ==============
    toss, hand          4.697e-07 rev   6.713e-02 rev   4e-6
    toss, leg           9.891e-08 rev   9.891e-08 rev   5e-4
    ring, hand          4.693e-07 rev   6.719e-02 rev   4e-6
    ring, leg           9.131e-07 rev   5.836e-04 rev   5e-4
    ==================  ==============  ==============  ==============

    **The leg lane is asserted on the RING, and the co-located toss row is why.**
    Masking ``HAS_V1`` moves the co-located toss's leg reconstruction by
    *nothing at all* — bit-identical, because with the platform nearly stationary
    the leg extension is nearly linear across a 25 ms knot and the
    ``(u2−u1)/T`` fallback IS the transmitted v1 to within the mirror's own
    resolution.  On the 60 mm ring, where the platform strokes, the same fault
    puts the leg band 1.17× outside its bound.  So the leg band does catch a v1
    regression, but only where there is platform motion to catch it in — which is
    a real property of the chain, not slack in the number, and it is stated here
    rather than left for a future reader to rediscover on a co-located rung.

    Pure: no plant, no MuJoCo — it is a statement about the ladder.
    """
    plan, _meta = smoke_plan
    ring, _ring_meta = smoke_ring_plan
    honest = _replay_mirror(plan, gate.geom, gate.mm_to_rev, mask_v1=False)
    broken = _replay_mirror(plan, gate.geom, gate.mm_to_rev, mask_v1=True)
    assert honest['hand'] <= MIRROR_TOL_HAND_REV
    assert honest['leg'] <= MIRROR_TOL_LEG_REV
    assert broken['hand'] > MIRROR_TOL_HAND_REV * 1000.0, (
        'masking HAS_V1 barely moved the hand reconstruction (%.3e vs %.3e) — '
        'the band would not catch a v1 regression' % (broken['hand'],
                                                      honest['hand']))
    assert broken['flags'] == (FLAG_HAS_U1 | FLAG_HAS_U2 | FLAG_HAS_HAND,)

    r_honest = _replay_mirror(ring, gate.geom, gate.mm_to_rev, mask_v1=False)
    r_broken = _replay_mirror(ring, gate.geom, gate.mm_to_rev, mask_v1=True)
    assert r_honest['leg'] <= MIRROR_TOL_LEG_REV
    assert r_honest['hand'] <= MIRROR_TOL_HAND_REV
    assert r_broken['leg'] > MIRROR_TOL_LEG_REV, (
        'masking HAS_V1 left the ring leg reconstruction inside its band '
        '(%.3e vs bound %.3e) — the leg band would not catch a v1 regression '
        'anywhere' % (r_broken['leg'], MIRROR_TOL_LEG_REV))
    assert r_broken['hand'] > MIRROR_TOL_HAND_REV * 1000.0


def _replay_mirror(plan, geom, mm_to_rev, *, mask_v1: bool) -> dict:
    """Plan → emitter → pump → wire → mirror at 500 Hz, no plant.

    Scores the mirror against the plan at the MIRROR'S OWN trajectory phase (the
    ladder question), exactly as ``UnifiedGate._stream`` does.
    """
    from jugglebot.motion.ik_solver import (pose_to_leg_lengths,
                                            rotvec_to_rot_matrix)
    emitter = KnotEmitter(geom)
    pump = _make_pump()
    mirror = _make_mirror(geom)
    dt = float(plan.dt)
    n_frames = int(math.floor(float(plan.total_duration) / dt)) + 1
    tick = 0.002
    worst_leg = worst_hand = 0.0
    flags = set()
    fb = [0.0] * 6
    t = 0.0
    seq = 0
    latch_tau = 0.0
    next_t = 0.0
    while t <= float(plan.total_duration) + 1e-12:
        if seq < n_frames and t >= next_t - 1e-9:
            sp, _ = pump.build(emitter.frame(plan, seq * dt, seq),
                               t_origin_us=int(seq * 25000))
            assert sp is not None
            rx = Setpoint.unpack(sp.pack())
            if mask_v1:
                rx = dataclasses.replace(rx, flags=rx.flags & ~FLAG_HAS_V1)
            flags.add(int(rx.flags))
            _latch(mirror, rx, t)
            latch_tau = seq * dt
            seq += 1
            next_t += dt
        mirror.tick(t, fb)
        mirror.tick_hand(t, float(mirror.hand_base_pos))
        tau = latch_tau + (t - mirror.base_timestamp)
        if 0.0 <= tau <= plan.total_duration:
            pose_p, _, _ = plan.state_at(tau)
            rot = rotvec_to_rot_matrix(np.asarray(pose_p[3:6], dtype=float))
            ext = pose_to_leg_lengths(np.asarray(pose_p[:3], dtype=float), rot,
                                      geom)
            worst_leg = max(worst_leg, float(np.max(np.abs(
                np.asarray(mirror.raw_pos) - ext * mm_to_rev))))
            hr, _ = plan.hand_at(tau)
            worst_hand = max(worst_hand,
                             abs(float(mirror.hand_raw_pos) - float(hr)))
        t += tick
    return {'leg': worst_leg, 'hand': worst_hand,
            'flags': tuple(sorted(flags))}


# ── the hand lane ────────────────────────────────────────────────────────────

def test_the_hand_lane_decays_on_the_falling_edge(smoke_plan, gate):
    """Cut the stream mid-stroke: the lane winds down, it does not hold.

    The NORMATIVE rule (canbridge_config.h's FW 17 block): when ``HAS_HAND``
    falls while the hand is MOVING, the lane must run its Hermite segment out,
    Taylor-extrapolate from the segment ENDPOINT and decay the velocity to
    zero — never hold Mode 1's ``s = 1`` endpoint, which would keep commanding
    it with ``vel_ff = v1`` from up to 200 rev/s.

    Cut deliberately at the knot of PEAK hand speed, so a lane that held would
    hold the worst possible feedforward.

    MEASURED (2026-09-04, ``python sim/unified_gate.py --no-viewer``): cut at
    74.8 rev/s on the co-located toss (91.5 on the ring), leaves Mode 1 at
    **0.026 s**, velocity exactly 0 at **0.136 s** against the 0.135 s deadline
    plus one 2 ms sample, raw wind-down travel **6.36 rev** against the 2.0 rev
    the clamp allows, **188 clamped ticks**.
    """
    plan, _meta = smoke_plan
    d = hand_decay_probe(plan, gate.geom)
    assert d['ok'] is True
    assert abs(d['v_at_cut_rps']) > 10.0, (
        'the cut landed on a stationary hand (%.3f rev/s) — the probe would '
        'observe nothing' % d['v_at_cut_rps'])
    # It leaves Mode 1 (it does not sit on the endpoint forever), and it does so
    # AT the firmware's segment boundary.  The instant is read off the lane's own
    # rung (``teensy_interp.hand_mode``), not from the age: recording "the first
    # age past SEGMENT_T_S" would be recording the probe's own loop condition,
    # which cannot fail even on a lane that never left Mode 1.  PROVENANCE:
    # SEGMENT_T_S = 0.025 s (canbridge_config.h SEG_T, pinned by
    # tests/firmware/test_hermite_xref.py), sampled on the 500 Hz tick grid, so
    # the earliest observable transition is one TICK_S after it.
    assert d['age_left_mode1_s'] is not None, (
        'the hand lane never left Mode 1 — it is holding the segment endpoint '
        'and its up-to-200 rev/s feedforward, which is the exact failure the '
        'falling-edge rule forbids')
    assert ti.SEGMENT_T_S < d['age_left_mode1_s'] <= (ti.SEGMENT_T_S
                                                      + d['sampling_slack_s']
                                                      + 1e-9), (
        'left Mode 1 at %.4f s, not at the %.3f s segment boundary'
        % (d['age_left_mode1_s'], ti.SEGMENT_T_S))
    # ... winds down monotonically ...
    assert d['monotone_after_mode1'] is True
    # ... and reaches EXACTLY zero inside the firmware's own deadline.
    assert d['age_velocity_zero_s'] is not None
    assert d['age_velocity_zero_s'] <= (d['decay_deadline_s']
                                        + d['sampling_slack_s'] + 1e-9)
    assert d['final_vel_rps'] == 0.0
    # The lead clamp is what bounds the wind-down, and by a wide margin: the
    # raw ladder coasts several rev from a fast cut.  Three assertions, because
    # travel alone proves nothing about the clamp — the band is 2.0 rev, so a
    # 1.0 rev wind-down would satisfy a "> 1.0" bar without the clamp ever
    # engaging.  The raw travel must exceed the BAND, the clamped travel must
    # not, and the clamp must have counted ticks.
    assert abs(d['travel_after_cut_rev']) > d['max_lead_hand_rev'], (
        'the raw wind-down (%.4f rev) stayed inside the %.1f rev clamp band, so '
        'the clamp is not what bounded it and the second assertion below is '
        'vacuous' % (d['travel_after_cut_rev'], d['max_lead_hand_rev']))
    assert d['lead_clamp_ticks'] > 0, (
        'the hand lead clamp never engaged during the wind-down')
    assert abs(d['clamped_travel_after_cut_rev']) <= d['max_lead_hand_rev'] \
        + 1e-9


def test_the_hand_lane_is_inert_until_a_has_hand_frame_latches():
    """``tick_hand`` transmits NOTHING before the first HAS_HAND frame.

    The firmware's ``s_hand_active`` gate.  A mirror that emitted a 0.0 rev
    command from an unlatched lane would command the hand to the retract stop.
    """
    mirror = _make_mirror(StewartGeometry())
    assert mirror.tick_hand(0.0, 5.0) is None
    mirror.latch_hand(5.0, 0.0, 0.0, u1=5.1, u2=5.2, v1=4.0)
    out = mirror.tick_hand(0.0, 5.0)
    assert out is not None and out[0] == pytest.approx(5.0)


def test_the_hand_lane_skips_an_axis_that_has_never_reported_an_encoder():
    """``fb_rev is None`` ⇒ no command, counted — the firmware's unseen-skip.

    0.0 rev is a real, reachable, WRONG hand position, so an axis that has
    never reported must not be commanded at all.  This is the bench7
    unseen-skip lesson, kept because it is the most likely way a sitting
    measures six lanes while believing it measured seven.
    """
    mirror = _make_mirror(StewartGeometry())
    mirror.latch_hand(5.0, 0.0, 0.0, u1=5.1, u2=5.2, v1=4.0)
    assert mirror.tick_hand(0.0, None) is None
    assert mirror.hand_unseen_skips == 1
    # The ladder still RAN — only the transmit was skipped.
    assert mirror.hand_raw_pos == pytest.approx(5.0)


# ── the ball, the beat, the grid ─────────────────────────────────────────────

def test_the_ball_is_caught_and_held(smoke_report):
    """Every planned catch is made, the ball is still held at the end.

    ``toss_gate``'s capture model, unchanged: a ``contact_carry=False`` plant,
    contact triggering the make and a kinematic hold carrying it.  The contact
    DETAIL (``capture_dist_mm``) is reported and not asserted, per the
    2026-08-29 owner resolution.
    """
    for t in smoke_report['results']:
        assert t['n_catches'] >= 1
        assert t['makes'] >= t['n_catches'], (t['point_id'], t['makes'],
                                              t['n_catches'])
        assert t['caught'] is True
        assert t['held_at_end'] is True
        assert t['separation_ms'] <= SEPARATION_MS
        assert np.isfinite(t['capture_dist_mm'])


def test_the_hold_is_quiescent_and_the_preposition_lands(smoke_report):
    """``gate_common``'s quiescence thresholds over the post-plan settle.

    The numbers are IMPORTED, not restated: they are the same thresholds
    ``sim/unified_gate.py`` scores ``quiescent`` / ``prepositioned`` with, and a
    second copy of 5.0 here would let this test keep passing after the gate's own
    band moved — i.e. the test would stop testing the gate.
    """
    for t in smoke_report['results']:
        assert t['hold_travel_mm'] < HOLD_TRAVEL_MM
        assert t['hold_tilt_deg'] < HOLD_TILT_DEG
        assert t['preposition_err_mm'] <= PREPOSITION_TOL_MM
        assert t['quiescent'] is True and t['prepositioned'] is True


def test_the_constant_beat_is_exact(smoke_report):
    """The ring's release instants are periodic to float summation error.

    Asserted on the RELEASE instants rather than on the window lengths, because
    the beat is what an observer sees and a bookkeeping error in ``extend``'s
    re-basing would move it without moving any window.
    """
    cb = smoke_report['constant_beat']
    assert cb['passed'] is True
    assert cb['beat_exact'] is True
    assert cb['worst_beat_dev_s'] <= BEAT_TOL_S
    assert cb['period_s'] == CYCLE_PERIOD_S
    ring = [r for r in smoke_report['results'] if r['kind'] == 'ring'][0]
    # LAUNCH + STEADY×1 + LANDING ⇒ two releases and two catches.
    assert ring['n_releases'] == 2 and ring['n_catches'] == 2
    assert ring['duration_s'] == pytest.approx(
        LAUNCH_PERIOD_S + 2 * CYCLE_PERIOD_S)


def test_a_longer_ring_holds_the_beat_over_four_cycles(gate):
    """The shipped ring: five releases, all exactly one period apart.

    Planning only (no plant): the beat is a property of the chained plan, and
    the plant column already ran on the one-cycle ring above.
    """
    plan, meta = gate.plan_ring(beat_grid()[0])
    rel = sorted(float(m.t_s) for m in meta.releases)
    assert len(rel) == 5
    dev = float(np.max(np.abs(np.diff(rel) - CYCLE_PERIOD_S)))
    assert dev <= BEAT_TOL_S, 'beat drifted by %.3e s' % dev
    # The catches ride the same beat, one flight time behind each release.
    cat = sorted(float(m.t_s) for m in meta.catches)
    assert len(cat) == 5
    assert float(np.max(np.abs(np.diff(cat) - CYCLE_PERIOD_S))) <= BEAT_TOL_S


def test_the_grid_maps_the_legacy_tiers(gate):
    """The single-toss grid is ``toss_gate``'s binding bands, tier for tier."""
    pts = single_toss_grid()
    ids = [_point_id(p) for p in pts]
    assert len(ids) == len(set(ids)), 'duplicate point ids in the grid'
    # 5 xy x 2 speed-band flights + 5 xy at 0.80 + 4 z-sweep + the 8-dir ring.
    assert len(pts) == 10 + 5 + 4 + 8
    # Nothing past the planner's measured 0.80 s ceiling: running a rung the
    # machine cannot fly would only re-measure a known refusal.
    assert max(p['flight_s'] for p in pts) <= 0.80 + 1e-12
    # Exactly one advisory point, and it is the +30 mm / 0.80 s one.
    adv = [p for p in pts if p['advisory']]
    assert len(adv) == 1
    assert adv[0]['z_offset_mm'] == 30.0 and adv[0]['flight_s'] == 0.80
    # The displaced ring really is displaced, and inside the reach envelope.
    ring = [p for p in pts if any(p['catch_dxy_mm'])]
    assert len(ring) == 8
    for p in ring:
        assert float(np.hypot(*p['catch_dxy_mm'])) == pytest.approx(50.0)
    assert default_grid() == pts + beat_grid()


def test_the_legacy_height_tier_maps_onto_the_cup_not_the_platform(gate):
    """A legacy platform-z offset becomes a CUP-z offset, exactly.

    The unified planner pins the platform's z, so the tier can only be reached
    with the slider — and the two are interchangeable because
    ``cup_z = CUP_Z_BASE_MM + slider + (pose_z − active_z)`` is linear in both
    with the same coefficient.  Asserted on the plan the gate actually builds:
    the throw site's cup z moves by the offset and the emitted pose z does not
    move at all.
    """
    base = dict(kind='toss', xy_mm=(0.0, 0.0), flight_s=0.60,
                catch_dxy_mm=(0.0, 0.0), advisory=False)
    plan0, meta0 = gate.plan_single_toss(dict(base, z_offset_mm=0.0))
    plan1, meta1 = gate.plan_single_toss(dict(base, z_offset_mm=-30.0))
    assert float(meta0.releases[0].site_mm[2]) == pytest.approx(THROW_CUP_Z_MM)
    assert float(meta1.releases[0].site_mm[2]) == pytest.approx(
        THROW_CUP_Z_MM - 30.0)
    z_pin = float(plan0.pose[0][2])
    assert np.allclose(np.asarray(plan0.pose)[:, 2], z_pin, atol=0.0)
    assert np.allclose(np.asarray(plan1.pose)[:, 2], z_pin, atol=0.0)
    # The slider carries the whole 30 mm instead.
    d_rev = float(plan0.hand_rev[0] - plan1.hand_rev[0])
    from jugglebot.motion.trajectory.hand_stroke import LINEAR_GAIN_REV_PER_M
    assert d_rev / LINEAR_GAIN_REV_PER_M * 1000.0 == pytest.approx(30.0,
                                                                   abs=1e-6)


def test_pass_threshold_agrees_with_both_sibling_gates():
    """The 90 % band is the SAME band all three gates use.

    ``unified_gate`` imports ``cycle_gate``'s function rather than restating
    the expression, and this pins that ``toss_gate``'s independent copy still
    agrees with it across the range either grid can produce — a divergence
    would mean two gates quietly held the fleet to different standards.
    """
    from sim.cycle_gate import _pass_threshold as cycle_threshold
    from sim.toss_gate import _pass_threshold as toss_threshold
    assert _pass_threshold is cycle_threshold
    for n in range(0, 40):
        assert _pass_threshold(n) == toss_threshold(n) == -(-9 * n // 10), n
    # The two numbers the shipped grids actually land on.
    assert _pass_threshold(10) == 9 and _pass_threshold(26) == 24


# ── the report ───────────────────────────────────────────────────────────────

def test_report_well_formed_and_written(tmp_path):
    """The JSON report round-trips and carries the notes that make it readable.

    Shape only, so the plant column and the decay probe are OFF: this asserts
    which keys exist and that the not-run sets report NOT RUN, none of which
    needs ~2 s of MuJoCo or a 0.4 s ladder replay.  The chain columns have their
    own tests above, on the shared ``smoke_report`` fixture.
    """
    out = tmp_path / 'unified_gate.json'
    rep = run_gate(UnifiedGateConfig(points=[_SMOKE_POINTS[0]],
                                     plant_column=False,
                                     decay_probe=False,
                                     report_path=str(out)))
    assert out.exists()
    disk = json.loads(out.read_text())
    assert disk['gate'] == 'unified'
    assert disk['passed'] is True
    for key in ('single_toss', 'constant_beat', 'invariants', 'thresholds',
                'contact_note', 'advisory_note', 'dropped_bands_note',
                'leg_clamp_note', 'hand_decay', 'results'):
        assert key in disk, key
    # The leg lane's mirror is motor_guard's clamp, NOT FW 17's, and the report
    # has to say so where a reader of the clamp counters will see it.
    assert 'motor_guard' in disk['leg_clamp_note']
    assert 'total_leg_lead_clamp_ticks' in disk
    assert 'total_hand_lead_clamp_ticks' in disk
    # The streaming deadline is reported per trial.  Both shipped sets end at
    # rest, so it is inf — a finite value would mean a release-terminal plan was
    # streamed off its own cliff.
    assert disk['results'][0]['latest_supersede_s'] == float('inf')
    # The dropped bands are named IN the report, so a reader comparing this
    # gate with toss_gate finds the explanation without reading the source.
    assert 'catch_armed' in disk['dropped_bands_note']
    assert 'never gated' in disk['contact_note'].lower()
    assert disk['thresholds']['mirror_tol_hand_rev'] == MIRROR_TOL_HAND_REV
    assert disk['thresholds']['mirror_tol_leg_rev'] == MIRROR_TOL_LEG_REV
    assert rep['report_path'] == str(out)
    # The constant-beat set was NOT run here — it must report NOT RUN, never a
    # vacuous pass, and the overall verdict still stands on the set that ran.
    assert disk['constant_beat']['ran'] is False
    assert disk['constant_beat']['passed'] is None
