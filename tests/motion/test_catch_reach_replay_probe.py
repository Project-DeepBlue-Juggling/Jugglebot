"""The catch-reach replay harness must keep telling the PRODUCTION planner's story.

``tools/probes/catch_reach_replay.py`` is the instrument that reproduced the
2026-07-25 catch-reach excursion offline (``plans/active/catch-reach-degenerate-
overshoot.md`` Phase 0) and is the command a future investigation will re-run on
a fresh capture. Its whole value is that it explains the excursion *in closed
form* — a psi/phi decomposition of the reach, a settle-scale factor, and an
amplification ratio — rather than merely plotting it.

A closed form is only worth having while it still equals what the robot runs.
Every constant it depends on lives in production code that is explicitly slated
to change (``planner._CATCH_TILT_THROUGH_RATE_RADPS``,
``_CATCH_TILT_OVERSHOOT_FRAC``, ``build_catch``'s ``tilt_decay_s``,
``hw.JB_TRAJ_CATCH_SETTLE_HOLD_S`` and ``catch_coordinator_node._PRETILT_EARLY_S``),
and the through-seat frame defect this plan exists to fix WILL move at least one
of them. So the failure mode this file exists to prevent is concrete: the
planner's arrival-twist model changes, the probe keeps printing the old
arithmetic with confident 6-decimal precision, and the next investigation
attributes a NEW excursion to a mechanism that no longer exists.

Those five are guarded by ``self_check`` case 7, which compares each mirrored
value against its production source DIRECTLY. That case exists because the
2026-07-26 review mutation-tested the original self-check and found it caught
only two of the five: ``build_replay`` passed the probe's own ``tilt_decay_s``
into every production call, so moving ``build_catch``'s default 0.15 -> 0.25 left
it passing 7/7, and nothing compared ``_PRETILT_EARLY_S`` at all. A downstream
symptom is not a guard.

Pure Python — the probe's ``mcap_ros2`` import is lazy (inside ``read_events`` /
``echo_epoch``), so everything here runs with no bag reader and no ROS.
"""

from __future__ import annotations

import importlib.util
import os

import numpy as np
import pytest

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_PROBE = os.path.join(_REPO, 'tools', 'probes', 'catch_reach_replay.py')


def _load_probe():
    spec = importlib.util.spec_from_file_location('catch_reach_replay', _PROBE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def probe():
    return _load_probe()


def test_self_check_passes(probe):
    """The probe's own eight-case validation against the production planner."""
    assert probe.self_check() == 0


def test_self_check_catches_a_moved_production_constant(probe, monkeypatch):
    """Case 7 must FAIL when a mirrored constant drifts — mutation-verified here.

    Without this, "the self-check passed" means only "the probe agrees with
    itself". Phase 2 is expected to move ``tilt_decay_s``; if the self-check
    stays green through that, the operator scores a post-fix capture with a model
    that builds the wrong decay and reads a correct fix as NOT-REPRODUCED.
    """
    from jugglebot.motion.trajectory import planner

    assert probe.self_check() == 0, 'baseline must be green before mutating'
    kwd = dict(planner.build_catch.__kwdefaults__)
    kwd['tilt_decay_s'] = 0.25
    monkeypatch.setattr(planner.build_catch, '__kwdefaults__', kwd)
    assert probe.self_check() == 1, (
        'moving build_catch(tilt_decay_s=) must break the self-check — the '
        'probe mirrors 0.15 and would otherwise keep scoring old physics')


def test_closed_form_equals_the_production_quintic(probe):
    """``p0 + (p1-p0)*psi + v1*T*phi`` IS ``QuinticSegment``, not an approximation.

    The whole "the excursion is the reach ACQUIRING its specified arrival rate"
    argument rests on this identity. If a future segment implementation stops
    being that quintic, the argument silently becomes a story about a curve the
    robot no longer runs.
    """
    from jugglebot.motion.trajectory.segment import QuinticSegment

    p0 = np.array([1.0, -2.0, 168.0, 0.004, -0.002, 0.001])
    p1 = np.array([-3.0, 5.0, 172.0, -0.0136, -0.0012, 0.0])
    v1 = np.array([0.0, 0.0, 0.0, -0.0697, -0.0062, 0.0])
    T = 2.5
    seg = QuinticSegment(p0=p0, v0=np.zeros(6), a0=np.zeros(6),
                         p1=p1, v1=v1, a1=np.zeros(6), duration=T)
    for s in (0.0, 0.1, 1.0 / 3.0, 2.0 / 3.0, 0.9, 1.0):
        got = seg.eval(s * T)[0]
        want = probe.rest_seeded_quintic(p0, p1, v1, T, s)
        assert got == pytest.approx(want, abs=1e-9), f's={s}'


def test_arrival_rate_basis_drives_the_wrong_way_first(probe):
    """``phi`` is NEGATIVE where it is largest — that is the sign reversal.

    A negative specified arrival rate therefore produces a POSITIVE excursion
    first. This is the single fact that turns "+2.32 deg from a -0.78 deg
    target" from an anomaly into a boundary condition.
    """
    s = np.linspace(0.0, 1.0, 20001)
    v = probe.phi(s)
    assert v.min() == pytest.approx(-16.0 / 81.0, abs=1e-9)
    assert s[int(np.argmin(v))] == pytest.approx(2.0 / 3.0, abs=1e-4)
    assert probe.phi(0.0) == pytest.approx(0.0)
    assert probe.phi(1.0) == pytest.approx(0.0, abs=1e-12)
    # psi is the rest-to-rest partner: monotone 0 -> 1, 0.79012 at s = 2/3.
    assert probe.psi(2.0 / 3.0) == pytest.approx(0.790123, abs=1e-6)
    assert np.all(np.diff(probe.psi(s)) >= -1e-12)


def test_amplification_is_direction_only_and_matches_both_measured_reaches(probe):
    """``(16/81)*rate*T/|tilt|`` — the number that explains the reload contrast.

    Both reaches came out of the same ``build_catch`` on the same session. The
    ratio is what separates them, and it depends on the tilt MAGNITUDE only
    through the denominator — which is why shrinking the requested tilt (the
    direction any levelling-frame fix pushes it) makes the amplification worse,
    not better. That is the argument against "just special-case the degenerate
    target"; pin it.
    """
    # 2026-07-25 toss pre-tilt: the levelling correction itself, ~3.71 s lead.
    tilt = probe.levelling.correct_pose(
        np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]),
        probe.levelling.correction_from_offset(*probe.REFERENCE_OFFSET))[3:5]
    toss = probe.excursion_ratio(tilt[0], tilt[1], probe.REFERENCE_LEAD_S)
    assert toss == pytest.approx(3.756, abs=5e-3)
    # 2026-07-25 reload pre-tilt: 11.08 deg receive tilt, ~2.37 s lead.
    reload_tilt = np.radians(np.array([1.774062, -10.636334]))
    rel = probe.excursion_ratio(reload_tilt[0], reload_tilt[1], 2.371)
    assert rel == pytest.approx(0.174, abs=5e-3)
    assert toss / rel > 20.0, 'the contrast is the headline — do not let it drift'
    # Direction-independent: scaling the tilt vector scales the ratio inversely,
    # rotating it leaves the ratio alone.
    assert probe.excursion_ratio(2 * tilt[0], 2 * tilt[1],
                                 probe.REFERENCE_LEAD_S) == pytest.approx(toss / 2)
    assert probe.excursion_ratio(-tilt[1], tilt[0],
                                 probe.REFERENCE_LEAD_S) == pytest.approx(toss)


def test_settle_scale_is_the_1_385x_the_plan_could_not_place(probe):
    """The settle matches neither a single nor a double correction — by design.

    ``1 + 0.5*rate*decay/|tilt|``. Pinning the arithmetic is what stops a future
    reader re-opening "the levelling correction is applied 1.385 times somewhere".
    """
    tilt = probe.levelling.correct_pose(
        np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]),
        probe.levelling.correction_from_offset(*probe.REFERENCE_OFFSET))[3:5]
    scale = probe.settle_scale(tilt[0], tilt[1])
    assert scale == pytest.approx(1.384732, abs=1e-5)
    assert np.degrees(tilt[0]) * scale == pytest.approx(probe.REFERENCE_SETTLE_DEG,
                                                        abs=5e-4)
    # Neither 1x nor 2x: the two readings the plan weighed and rejected.
    assert abs(scale - 1.0) > 0.3 and abs(scale - 2.0) > 0.3
    # A level catch (no tilt) disables the through-seat entirely.
    assert probe.settle_scale(0.0, 0.0) == pytest.approx(1.0)


def test_the_acc_jerk_spike_belongs_to_the_through_seat_decay(probe):
    """Segment-level attribution of feature 3, through the production gate.

    The 0.15 s decay is an order of magnitude above the multi-second reach in
    acceleration and two in jerk. Any future change that moves the spike back
    into the reach (or removes the decay) must break this, because the runbook's
    reading of a mid-flight acc/jerk step depends on the attribution.
    """
    from jugglebot.motion.geometry import StewartGeometry
    from jugglebot.motion.trajectory.limits import TrajectoryLimits
    import jugglebot.hardware_config as hw

    geom = StewartGeometry()
    limits = TrajectoryLimits.from_config(hw)
    target = probe.levelling.correct_pose(
        np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]),
        probe.levelling.correction_from_offset(*probe.REFERENCE_OFFSET))
    plan, report = probe.build_replay(
        np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]), target,
        probe.REFERENCE_LEAD_S, limits, geom)
    segs = probe.segment_peaks(plan, limits, geom)
    assert [round(s['duration_s'], 3) for s in segs] == [
        round(probe.REFERENCE_LEAD_S, 3), probe.TILT_DECAY_S,
        float(hw.JB_TRAJ_CATCH_SETTLE_HOLD_S)]
    assert segs[1]['peak_leg_acc_mmps2'] > 5.0 * segs[0]['peak_leg_acc_mmps2']
    assert segs[1]['peak_leg_jerk_mmps3'] > 20.0 * segs[0]['peak_leg_jerk_mmps3']
    # The plan-level report IS the decay's peaks — which is exactly why the
    # published peak_leg_* is lead-INVARIANT across catch installs and looked
    # "stale" to the original investigation.
    assert report.peak_leg_acc_mmps2 == pytest.approx(
        segs[1]['peak_leg_acc_mmps2'], rel=1e-6)
    assert report.peak_leg_jerk_mmps3 == pytest.approx(
        segs[1]['peak_leg_jerk_mmps3'], rel=1e-6)


def test_catch_plan_predicted_peaks_are_lead_invariant_only_for_a_small_reach(probe):
    """The refutation of "peak_leg_* is stale or cached" — WITH its qualifier.

    The field read identically before and after the install because a catch
    plan's predicted peaks are the through-seat decay's peaks whenever the reach
    is small enough for that fixed 0.15 s shape to dominate — not because the
    field was never written.

    The qualifier is load-bearing and the first draft of this test lost it by
    sweeping the lead at a near-degenerate displacement only. A reload-sized
    reach is NOT lead-invariant (its reach term scales as displacement/lead and
    sits above the decay floor), and the reference bag shows it: ``move_seq`` 4
    and 15, the two reload catch installs, published ``23.6`` and ``23.8`` mm/s
    where every toss catch published ``14.2``. Without this half, a future reader
    who sees the predicted vel move between two consecutive catch installs would
    conclude a non-catch plan came in between — reviving the very inference error
    this whole investigation existed to kill.
    """
    from jugglebot.motion.geometry import StewartGeometry
    from jugglebot.motion.trajectory.limits import TrajectoryLimits
    import jugglebot.hardware_config as hw

    geom = StewartGeometry()
    limits = TrajectoryLimits.from_config(hw)
    target = probe.levelling.correct_pose(
        np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]),
        probe.levelling.correction_from_offset(*probe.REFERENCE_OFFSET))
    seed = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    leads = (0.8, 2.0, 3.707, 8.0)

    # (a) near-degenerate reach: invariant over a ten-fold lead range.
    peaks = []
    for lead in leads:
        _plan, rep = probe.build_replay(seed, target, lead, limits, geom)
        peaks.append((rep.peak_leg_vel_mmps, rep.peak_leg_acc_mmps2,
                      rep.peak_leg_jerk_mmps3))
    for got in peaks[1:]:
        assert got == pytest.approx(peaks[0], rel=2e-3)
    assert peaks[0] == pytest.approx(probe.REFERENCE_PRED_PEAKS, rel=0.06)

    # (b) reload-sized reach: the vel peak DOES move with the lead.
    reload_target = np.array([11.877, 2.867, 171.163,
                              np.radians(1.774062), np.radians(-10.636334), 0.0])
    reload_seed = np.array([0.0, 0.0, 170.0, target[3], target[4], 0.0])
    vels = []
    for lead in leads:
        _plan, rep = probe.build_replay(reload_seed, reload_target, lead,
                                        limits, geom)
        vels.append(rep.peak_leg_vel_mmps)
    assert vels[0] > 5.0 * vels[-1], (
        'a reload-sized catch reach is NOT lead-invariant — the reach term '
        'dominates the decay floor at a short lead')
    assert vels == sorted(vels, reverse=True)
    # It converges DOWN onto the decay floor, which is what makes (a) true.
    assert vels[-1] == pytest.approx(14.0, abs=0.5)


def test_install_census_needs_every_signal_to_agree(probe):
    """A single-install verdict must not rest on ``move_seq`` alone.

    A follower install deliberately does not bump ``move_seq``, and the emitter
    step backstop / escalation hold / freeze-in-place assign ``_active_plan``
    directly, bypassing ``_install`` and bumping nothing at all — so "one value
    seen" is necessary and not sufficient. The census therefore also requires a
    single ``plan_kind`` (a ``HoldPlan`` reads ``'hold'``, which is what exposes
    the bypasses), no accepted ``target_feedback``, no newly-raised rejection and
    a still plan-end.
    """
    class _KV:
        def __init__(self, key, value):
            self.key, self.value = key, value

    class _Diag:
        def __init__(self, seq):
            self.values = [_KV('move_seq', str(seq)),
                           _KV('peak_leg_vel_mmps', '14.2'),
                           _KV('peak_leg_acc_mmps2', '142.4'),
                           _KV('peak_leg_jerk_mmps3', '3950')]

    class _Status:
        def __init__(self, rem, rejection='', kind='move'):
            self.plan_kind = kind
            self.plan_time_remaining_s = rem
            self.last_rejection = rejection

    def series(seqs, rejection='', end=10.0, kind='move', t_start=1.0):
        ev = []
        for i, seq in enumerate(seqs):
            t = t_start + 0.2 * i
            ev.append((t, probe.DIAG, _Diag(seq)))
            ev.append((t, probe.STATUS, _Status(end - t, rejection, kind)))
        return ev

    clean = probe.install_census(series([48] * 20), 0.0, 20.0)
    assert clean['single_install'] is True
    assert clean['plan_end_drift_ms'] < 1e-6

    # A re-install: move_seq moves AND the plan-end jumps.
    reinstalled = probe.install_census(
        series([48] * 10) + series([49] * 10, end=14.0), 0.0, 20.0)
    assert reinstalled['single_install'] is False
    assert reinstalled['move_seqs'] == [48, 49]
    assert reinstalled['plan_end_drift_ms'] > 1000.0

    # The backstop shape: move_seq NEVER moves, but the plan_kind flips to
    # 'hold' and a fresh rejection is raised at the same instant.
    backstop = probe.install_census(
        series([48] * 10)
        + series([48] * 10, t_start=3.0, kind='hold',
                 rejection='emitter step 0.4 rev > 0.3 bound — froze to hold'),
        0.0, 20.0)
    assert backstop['move_seqs'] == [48]
    assert backstop['single_install'] is False, (
        'the emitter backstop installs a HoldPlan behind move_seq — plan_kind '
        'and a newly-RAISED rejection are what expose it')
    assert backstop['plan_kinds'] == ['hold', 'move']
    assert backstop['new_rejections']


def test_a_rejection_latched_before_the_window_is_not_an_install(probe):
    """``last_rejection`` is a sticky lifetime latch — do not gate on it.

    ``trajectory_node`` assigns ``_last_rejection = ''`` in exactly one place,
    ``__init__`` (verified 2026-07-26: 28 assignment sites, one of them empty),
    and republishes it on every 5 Hz status forever after. The first draft of
    the census treated any non-empty value as an in-window install, which meant
    ONE early rejection anywhere in a session — a ``WORKSPACE`` reach-envelope
    reject, a ``STALE_STATE`` retry, a deliberate pre-``level`` refusal — would
    force ``SINGLE INSTALL: False`` and therefore ``NOT-REPRODUCED``, exit 1, on
    every healthy reach after it. The operator runbook reads a non-reproduction
    as a finding, so that would route a correct fix back for rework and burn a
    powered sitting: an instrument failing a working system, which is the exact
    trap the tolerance rule elsewhere in this probe exists to avoid.

    It never fired on the reference bag (0 non-empty ``last_rejection`` across
    all 1453 status samples), which is why it survived the first pass.
    """
    class _KV:
        def __init__(self, key, value):
            self.key, self.value = key, value

    class _Diag:
        def __init__(self, seq):
            self.values = [_KV('move_seq', str(seq))]

    class _Status:
        def __init__(self, rem, rejection=''):
            self.plan_kind = 'move'
            self.plan_time_remaining_s = rem
            self.last_rejection = rejection

    latched = 'catch target rejected: WORKSPACE'
    ev = []
    # Two status samples BEFORE the window already carry the latched string.
    for i in range(2):
        ev.append((0.2 + 0.2 * i, probe.STATUS, _Status(9.0, latched)))
    # ...and it is still latched throughout the (clean) window.
    for i in range(20):
        t = 1.0 + 0.2 * i
        ev.append((t, probe.DIAG, _Diag(48)))
        ev.append((t, probe.STATUS, _Status(10.0 - t, latched)))

    census = probe.install_census(ev, 0.9, 20.0)
    assert census['move_seqs'] == [48]
    assert census['rejections'] == [latched], 'still reported, as an advisory'
    assert census['rejection_latched_before'] == latched
    assert census['new_rejections'] == [], 'nothing was RAISED inside the window'
    assert census['single_install'] is True, (
        'a rejection latched before the window is session history, not an '
        'install inside it — gating on it fails a healthy capture')
