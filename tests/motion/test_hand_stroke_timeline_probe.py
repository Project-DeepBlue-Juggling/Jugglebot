"""``hand_stroke_timeline.py`` — the truncation criterion, both sides of it.

``tools/probes/hand_stroke_timeline.py`` is the instrument
``tests/hardware/session_anomaly_fixes.md`` § Section HAND names to turn a toss
capture into a PASS/ABORT verdict, and rows 1, 2, ``H2.2`` and ``H4.6`` all read
its ``trunc`` / ``seeds`` output.  Those rows exist to catch the 2026-07-25
clobber class: a command that lands *inside* the throw stroke, clears the packed
queue and re-preludes from the live encoder, so the ball departs on the position
loop's reaction to a frozen setpoint.

Until 2026-08-18 the criterion could not tell that apart from **Phase 1's arm
gate working**.  The gate withholds the catch arm until the stroke completes and
then dispatches on the next balls tick, so a correct gate lands a from-rest
prelude just past the stroke end, seeded at a live position that has sagged
0.06-0.17 rev under ``x3`` — which satisfies both halves of the old predicate
(``vel_ff_cmd`` ~ 0, ``pos_cmd`` short of ``x3``) whenever it fell inside a 50 ms
wall-clock margin.  The 2026-07-27 validation sitting straddled that margin
(arms at 36.7-127.9 ms past the modelled stroke end): 6 tosses reported a
truncation across its six bags and 3 across its three traces, and for the one
session recorded BOTH ways the same physical tosses read 4 (bag) against 3
(trace).  Every one was adjudicated PASS by hand at the bench.

The criterion is now the stroke-completion event itself — the scan stops at the
first sample whose ``pos_cmd`` is no longer short of ``x3`` — so the two cases
differ in *kind* rather than in delay.  This file pins both sides:

* the gated arm's prelude, at two holds either side of the old 50 ms wall,
  reads **not truncated** — and is still *reported*, because a criterion that
  hides the event it stopped scoring is a blind spot, not a fix;
* a command that lands **inside** the stroke, late on the decel ramp where any
  narrowed margin would have lost it, still reads **truncated** with a seed;
* the real 2026-07-25 pre-fix capture, from the committed fixture, still reads
  its truncation at the recorded position;
* the probe's own two-sided ``--gate`` — the runbook's MANDATORY instrument
  self-check, whose ``GATE FAIL`` ABORTs the whole HAND analysis — passes.

THE SAME DEFECT, TWICE (2026-08-20)
-----------------------------------
The OTHER half of the predicate had the same shape.  ``trunc`` also has to
localise *where* the command froze, and it did that with an absolute
``_COLLAPSE_VEL_REV_S = 10`` rev/s — a fixed velocity judging a ramp whose
velocity scales with the throw.  At the slow end of the band the ramp fell under
it while still short of ``x3``, so the probe reported a truncation on a perfectly
clean stroke; whether it did depended on where a ~100 Hz sample landed inside a
~2 ms window, i.e. on the RECORDING.  At the C-HAND-3 admission floor (2.440 m/s)
that was 1.94 ms — **20 of 100 sampling phases** — 13 at 2.550, 6 at 2.6971, 2 at
2.800, 0 at 2.845 and above.

"Collapse" is now ``_collapse_floor_rps(model)``: the modelled stroke's own
commanded velocity at ``x3 - _X3_SHORT_REV``.  Constant deceleration makes
commanded velocity monotone in commanded position, so an intact stroke short of
``x3`` is never under it — the self-trigger window is EMPTY at every speed rather
than narrow.  This file pins that with a sampling-phase sweep, and pins the other
direction (a real freeze at the band floor still fires) so raising the floor
cannot blunt the detector.  These four tests replaced an ``xfail(strict=True)``
that had held the defect while the owner decided; the marker is removed, not
flipped.

Note the tests import the C-HAND-3 band floor rather than quoting it.  The floor
moved DOWN from 2.6971 to 2.440 m/s between this defect being characterised and
being fixed, tripling the exposure — a literal would have gone stale in the
safe-looking direction.

That last one is not decoration.  The gate was **RED on the shipped tree** from
the 2026-08-18 hard-stop correction (11.1 -> 10.8 rev) until this file landed,
because ``_GATE_EXPECT``'s ``headroom_to_limit_rev`` row was a literal derived
from the old anchor and nothing in the suite ran the gate.  An operator would
have discovered it at the bench, mid-sitting, and correctly refused to score any
HAND row.

Empirical basis (``CLAUDE.md``'s empirical-probe rule).  The discriminator was
prototyped in ``/tmp/probe_trunc_criterion.py`` against the whole evidence base —
the six 2026-07-27 session bags plus six ``toss_trace_*.jsonl`` recordings, 71
analysable throws — run 2026-08-18.  Measured, relative to the instant
``pos_cmd`` first reached ``x3``: every gated-arm prelude lands **+30.6 to
+128.3 ms** (positive by construction, since the gate is what put it there),
while the two real pre-fix truncations land **-208.1 and -299.0 ms** (there the
command only reaches ``x3`` later, as part of the *replacement* quintic).  Sign,
not size, separates them.  Post-fix, all 69 clean throws read ``trunc = -`` and
``seeds = 0``, and both real truncations still read ``ok``.

Pure Python — the probe's ``mcap_ros2`` import is lazy (inside ``load_bag``), so
everything here runs with no bag reader and no ROS.
"""

from __future__ import annotations

import importlib.util
import os
import sys

import pytest

import jugglebot.hardware_config as hw

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_PROBE = os.path.join(_REPO, 'tools', 'probes', 'hand_stroke_timeline.py')

#: two holds that BRACKET the retired 50 ms wall.  ``arm_hold_s`` is measured
#: from the MODELLED stroke end, so the comparable sitting figure is its earliest
#: arm at +36.7 ms past that end — 0.028 s is deliberately tighter than anything
#: the capture produced, and 0.062 s deliberately looser than the wall.  (The
#: realized ``stroke_end_hold_ms`` at 0.028 is 40 ms, because the command enters
#: the 0.05 rev band ~7 ms before the modelled end; the two are different
#: quantities and this file is careful not to conflate them.)  The point of
#: running both is that the criterion may not depend on the delay AT ALL.
_HOLDS_S = (0.028, 0.062)


def _load_probe():
    spec = importlib.util.spec_from_file_location('hand_stroke_timeline', _PROBE)
    mod = importlib.util.module_from_spec(spec)
    # registered before exec: the module defines @dataclass types, and
    # dataclasses resolves annotations through sys.modules[cls.__module__]
    sys.modules['hand_stroke_timeline'] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def probe():
    return _load_probe()


def _arm_prelude_capture(probe, hold_s):
    """The POST-FIX shape with the gated arm's own prelude ``hold_s`` past x3."""
    session, t_desc = probe._synth_fixed_session(0.0, tof_s=0.80, v_mps=3.9308,
                                                 arm_hold_s=hold_s)
    tl = probe.analyse_throw(session, session.announcements[0])
    return tl, t_desc


def _late_truncation_capture(probe, short_rev=0.30, v_mps=3.9308,
                             tof_s=0.80):
    """A genuine clobber ``short_rev`` before x3 — late on the decel ramp.

    Built by freezing ``pos_cmd`` on the otherwise-clean arm-prelude capture, so
    the ONLY difference from the case above is that the command never reaches
    ``x3``.  ``short_rev`` is bounded below by ``_X3_SHORT_REV`` = 0.05 rev: a
    freeze inside that band is not a truncation by definition, and no criterion
    can see it.  0.30 rev is six times the band and still inside the last 10 %
    of the stroke — the ramp is 0.05 rev short of ``x3`` only for its last
    7.2 ms at 3.93 m/s, which is what makes the true-positive edge a FIXED
    quantity while the arm's arrival is not.
    """
    session, _ = probe._synth_fixed_session(0.0, tof_s=max(tof_s, 0.204 * v_mps),
                                            v_mps=v_mps, arm_hold_s=0.028)
    model = probe.StrokeModel(v_mps)
    freeze = model.x3_rev - short_rev
    t_freeze = None
    for k, w in enumerate(session.samples):
        if t_freeze is None:
            if (w.pos_cmd >= freeze
                    and w.vel_ff_cmd > probe._collapse_floor_rps(model)):
                t_freeze = w.t
            continue
        session.samples[k] = probe.HandSample(
            w.t, w.pos_meas, w.vel_meas, min(w.pos_cmd, freeze),
            0.0 if w.pos_cmd >= freeze else w.vel_ff_cmd)
    assert t_freeze is not None, 'the synthetic never produced a freezable ramp'
    tl = probe.analyse_throw(session, session.announcements[0])
    return tl, t_freeze, freeze


# ── the fix: the gated arm working is not a truncation ────────────────────

@pytest.mark.parametrize('hold_s', _HOLDS_S)
def test_the_gated_arm_prelude_is_not_a_truncation(probe, hold_s):
    """Phase 1's gate doing its job must not read as the defect it prevents.

    Both holds bracket the old ``_TRUNC_SCAN_MARGIN_S = 0.050`` wall.  Before the
    fix the +28 ms case reported ``trunc`` with one seed and the +62 ms case did
    not — the same physical event, scored opposite ways by arrival delay alone.
    """
    tl, _ = _arm_prelude_capture(probe, hold_s)
    assert tl.status == 'not-truncated'
    assert tl.trunc is None
    assert tl.n_seeds == 0


@pytest.mark.parametrize('hold_s', _HOLDS_S)
def test_the_arm_prelude_is_reported_rather_than_hidden(probe, hold_s):
    """Narrowing the criterion must not make the event invisible.

    ``post_stroke_cmd`` carries the instant, the commanded position and its
    distance from the live ``pos_meas`` — the ``makeSmoothMove`` re-seed
    fingerprint — so an operator scoring rows 4 and 7 still sees the command
    that used to be mis-scored as a truncation, and ``H4.6``'s "a seed at or
    above ``x3`` with ``trunc = -``" case has somewhere to be read off.
    """
    tl, _ = _arm_prelude_capture(probe, hold_s)
    t_arm = probe._SYNTH_T_REL + probe.StrokeModel(3.9308).t_dec + hold_s
    assert tl.stroke_end_reached is not None
    assert tl.stroke_end_reached <= t_arm
    assert tl.post_stroke_cmd is not None
    assert tl.post_stroke_cmd == pytest.approx(t_arm, abs=0.020)
    # seeded AT the live encoder position: 0.000x rev is the fingerprint
    assert tl.post_stroke_cmd_vs_meas_rev <= 0.010
    assert tl.stroke_end_hold_ms > 0.0


@pytest.mark.parametrize('hold_s', _HOLDS_S)
def test_the_arm_prelude_leaves_the_other_gated_rows_clean(probe, hold_s):
    """The rows that DO own the post-stroke region must still read healthy.

    Otherwise the fix would only have moved the false ABORT from row 1 to row 4.
    """
    tl, t_desc = _arm_prelude_capture(probe, hold_s)
    assert tl.dip_below_x3_rev <= probe._DIP_BELOW_X3_BAND_REV
    # the prelude is not mistaken for the armed catch descent, which bounds the
    # dip/peak window and the truncation scan alike
    assert tl.catch_desc == pytest.approx(t_desc, abs=0.020)


# ── the other side: a real clobber still fires ────────────────────────────

def test_a_command_inside_the_stroke_is_still_a_truncation(probe):
    """The blind spot a narrowed margin would have bought.

    This freeze is 0.30 rev short of ``x3``, i.e. inside the last 10 % of the
    stroke and comfortably inside any margin wide enough to clear the arm.  It
    fires because the bound is the commanded profile reaching ``x3``, which this
    capture never does — not because it happened to arrive early enough.
    """
    tl, t_freeze, freeze = _late_truncation_capture(probe)
    assert tl.status == 'ok'
    assert tl.trunc == pytest.approx(t_freeze, abs=0.020)
    assert tl.trunc_pos_rev == pytest.approx(freeze, abs=0.010)
    assert tl.n_seeds >= 1


def test_the_committed_fixture_still_reads_the_pre_fix_truncation(probe):
    """Real data, not a synthetic: the 2026-07-25 capture the plan was written from.

    Numbers are the plan's Context table (``trunc`` at 7.700 rev, one seed),
    which ``--gate`` also asserts; pinned here so a change to the criterion has
    to survive the recorded defect as well as the synthetic ones.
    """
    session = probe.load_trace(probe._GATE_FIXTURE)
    throws = [probe.analyse_throw(session, a) for a in session.announcements]
    throws = [t for t in throws if t.status == 'ok']
    assert len(throws) == 1
    tl = throws[0]
    assert tl.trunc is not None
    assert tl.trunc_pos_rev == pytest.approx(7.700, abs=0.010)
    assert tl.n_seeds == 1
    # the command reaches x3 only LATER, and as part of the replacement quintic
    assert tl.stroke_end_reached > tl.trunc


# ── the instrument self-check the operator runs at the bench ──────────────

def test_the_operator_gate_self_check_passes(probe, capsys):
    """``hand_stroke_timeline.py --gate``, which a HAND sitting cannot proceed without.

    Runs against the COMMITTED fixture, so a fresh clone gets the same verdict as
    this Jetson.  Both branches must report ``GATE PASS``: the Context-table
    branch against the recorded pre-fix capture, and the fixed-shape branch
    against the synthetics.  The evidence base does carry the fixed shape (69 of
    71 analysable throws), but those recordings live under gitignored ``temp/``
    and on a bag drive, so a fresh clone cannot run them — and none of them
    carries the ``deep-brake``, ``late-trunc`` or arm-hold variants the gate
    pins.
    """
    rc = probe.run_gate(probe._GATE_FIXTURE)
    out = capsys.readouterr().out
    assert rc == 0, out
    assert out.count('GATE PASS') == 2, out
    assert 'GATE FAIL' not in out


# ── the collapse floor: relative to the profile, not an absolute rev/s ────

def _self_trigger_window_s(probe, v, floor_rps):
    """[lo, hi] of the decel ramp where BOTH truncation predicates hold.

    Walked over the closed form at 0.01 ms steps.  Empty (``None``) means the
    ramp can never report itself, which is the property under test.
    """
    m = probe.StrokeModel(v)
    lo = hi = None
    t = 0.0
    while t <= m.t_dec + 1e-9:
        pos = m.pos_rev(t)
        vel = (m.pos_rev(t + 1e-6) - m.pos_rev(t - 1e-6)) / 2e-6
        if pos < m.x3_rev - probe._X3_SHORT_REV and vel < floor_rps:
            lo = t if lo is None else lo
            hi = t
        t += 1e-5
    return (lo, hi)


#: the C-HAND-3 admission floor, and the two slower/faster speeds the defect was
#: characterised at.  Imported rather than hardcoded where possible: the envelope
#: moved DOWN (2.6971 -> 2.4400 m/s) between this defect being found and fixed,
#: and the exposure is worst at the floor, so a literal would have gone stale in
#: the safe-looking direction.
def _band_floor_speed():
    from jugglebot.motion.trajectory import throw_envelope as te
    return te.vertical_release_speed_mps(te.min_flight_time_s())


def test_the_decel_ramp_can_never_report_itself_as_a_truncation(probe):
    """The 2026-08-20 contract change, at the speed where it bit hardest.

    "Collapse" is no longer an absolute 10 rev/s.  It is
    ``_collapse_floor_rps(model)`` — the modelled stroke's OWN commanded velocity
    at ``x3 - _X3_SHORT_REV``.  Because the firmware's decel segment is constant
    deceleration, commanded velocity falls monotonically with commanded position,
    so every sample of an intact stroke that is still short of ``x3`` is at or
    above that floor **by construction**.  The window in which the ramp satisfies
    both predicates is therefore empty at every speed — not merely narrow.

    Under the old absolute 10 rev/s it was not.  At the C-HAND-3 admission floor
    the ramp's velocity at the band edge is 8.58 rev/s, so the window was
    **1.94 ms wide** — about 1 toss in 5 at a 10 ms telemetry period.
    """
    v = _band_floor_speed()
    m = probe.StrokeModel(v)
    floor = probe._collapse_floor_rps(m)

    # The defect, still measurable against the retired constant — but only while
    # the shipped band floor is slow enough for 10 rev/s to sit inside the ramp's
    # range.  Guarded rather than asserted unconditionally, so a future envelope
    # that lifts the floor above 2.845 m/s self-skips instead of reddening.
    if floor < 10.0:
        lo_old, hi_old = _self_trigger_window_s(probe, v, 10.0)
        assert lo_old is not None and (hi_old - lo_old) > 1.0e-3, (
            'the absolute-10-rev/s window at the band floor should be ~1.94 ms; '
            'got %r' % (None if lo_old is None else hi_old - lo_old,))

    # ...and gone under the profile-relative floor, at every admissible speed
    for v_test in (v, 2.6971, 2.8000, 3.4400, 4.3568):
        m_t = probe.StrokeModel(v_test)
        lo, hi = _self_trigger_window_s(
            probe, v_test, probe._collapse_floor_rps(m_t))
        assert lo is None, (
            'the ramp satisfies both truncation predicates for %.3f ms at '
            '%.4f m/s' % (1000.0 * (hi - lo), v_test))
    # The invariant that actually matters, and it is derived rather than quoted:
    # the floor is a fixed FRACTION of the commanded peak (0.1112, velocity-
    # independent), so it can never reach the velocity-hold plateau that the scan
    # range also contains.  Today's value at the admission floor is 8.576 rev/s;
    # that number lives in the docstring, not in an assertion that a band move
    # would falsify.
    assert floor < v * probe.LINEAR_GAIN_REV_PER_M
    assert floor / (v * probe.LINEAR_GAIN_REV_PER_M) == pytest.approx(
        0.111171, abs=1e-5)


def test_the_collapse_floor_scales_with_the_commanded_throw(probe):
    """It is the profile's own velocity, so it must move with the profile.

    A floor that did not scale is the defect this replaced.  Closed form
    ``sqrt(-2*throwD*delta)``, cross-checked against a bisection on ``pos_rev``
    so the algebra cannot drift from the model it claims to describe.
    """
    seen = []
    for v in (2.4400, 2.6971, 3.4400, 3.9308, 4.3568, 7.0):
        m = probe.StrokeModel(v)
        got = probe._collapse_floor_rps(m)
        # independent: bisect for the time at which pos_cmd == x3 - band, then
        # differentiate the model there
        lo, hi = 0.0, m.t_dec
        for _ in range(200):
            mid = 0.5 * (lo + hi)
            if m.pos_rev(mid) < m.x3_rev - probe._X3_SHORT_REV:
                lo = mid
            else:
                hi = mid
        want = (m.pos_rev(lo + 1e-7) - m.pos_rev(lo - 1e-7)) / 2e-7
        assert got == pytest.approx(want, abs=1e-3), v
        seen.append(got)
    assert seen == sorted(seen), 'the floor must rise with the throw'
    assert seen[0] < 10.0 < seen[-1], (
        'the retired absolute 10 rev/s sat INSIDE the admissible range, which '
        'is why it fired at the bottom and was slack at the top: %r' % (seen,))


@pytest.mark.parametrize('v_mps', [2.4400, 2.6971, 2.8000, 3.9308])
def test_no_sampling_phase_makes_a_clean_capture_report_a_truncation(probe,
                                                                     v_mps):
    """The end-to-end version of the above, through ``analyse_throw``.

    Whether the old defect fired depended on where a ~100 Hz sample landed inside
    a ~2 ms window — a free parameter of the RECORDING, not of the robot, which
    is what made it an artefact.  So sweep the sampling phase across a whole
    telemetry period rather than testing one grid.

    WHAT THIS SWEEP CAN AND CANNOT SEE.  ``_synth_fixed_session`` derives
    ``vel_ff_cmd`` as a finite difference of its own ``pos_cmd``, so velocity and
    position are exactly consistent with ``StrokeModel``.  Under that
    construction "0 of 40 fire" pins the DETECTOR'S WIRING — the ``i_peak`` /
    ``i_end`` range and the per-throw floor, and mutation G (a floor that is
    derived but does not scale) is caught only here — but it cannot see
    model-vs-recording disagreement, which is the one way the exact-complement
    property could break.  That half is evidenced on real telemetry instead: the
    minimum in-scan ``vel_ff_cmd - floor`` over all 69 clean throws in the
    evidence base is +0.4288 rev/s (489 samples), and at that tightest sample the
    measured commanded velocity matches the model to 0.002 rev/s.
    """
    m = probe.StrokeModel(v_mps)
    fired = []
    for k in range(40):
        session, _ = probe._synth_fixed_session(
            m.x3_rev + 0.02, tof_s=max(0.50, 0.204 * v_mps), v_mps=v_mps,
            phase_s=k * (0.010 / 40))
        tl = probe.analyse_throw(session, session.announcements[0])
        if tl.trunc is not None:
            fired.append((k, tl.trunc_pos_rev))
    assert not fired, (
        '%d of 40 sampling phases reported a truncation on a clean %.4f m/s '
        'capture; first at %.4f rev' % (len(fired), v_mps, fired[0][1])
        if fired else '')


def test_a_freeze_at_the_band_floor_still_fires(probe):
    """The other direction: raising the floor must not blunt the detector.

    The floor at 2.4400 m/s is 8.58 rev/s, well above the ~0 rev/s a frozen
    setpoint reads — the two real 2026-07-25 truncations freeze at 0.010 rev/s
    against a 13.82 rev/s floor, a 1382x margin.
    """
    v = _band_floor_speed()
    tl, t_freeze, freeze = _late_truncation_capture(probe, short_rev=0.30,
                                                    v_mps=v)
    assert tl.status == 'ok'
    assert tl.trunc == pytest.approx(t_freeze, abs=0.020)
    assert tl.trunc_pos_rev == pytest.approx(freeze, abs=0.010)


def test_the_gate_reference_is_anchored_on_the_shipped_hard_stop(probe):
    """``headroom_to_limit_rev`` must be derived, not transcribed.

    It was a literal 0.93 rev (= 11.1 - 10.174) and the 2026-08-18 correction to
    the measured 10.8 rev hard stop left it behind, so ``--gate`` was RED on the
    shipped tree with nothing in the suite to say so.  Pinning it to the same
    generated constant the probe computes against means the next correction to
    the hard stop cannot silently invalidate the operator's self-check.

    This asserts the EXPECTATION is derived, not that the probe computes it the
    same way — that is ``A == A`` against the current source and it guards only
    against someone re-hardcoding a literal.  What pins the COMPUTED value is
    ``test_the_operator_gate_self_check_passes``: the gate reproduces
    ``headroom_to_limit_rev = 0.626`` rev against the real capture.
    """
    want = dict((name, exp) for name, exp, _tol, _u in probe._GATE_EXPECT)
    assert want['headroom_to_limit_rev'] == pytest.approx(
        hw.GEOM_HAND_MOTOR_HARD_STOP_REVS - want['peak_pos_rev'], abs=1e-12)


# ── the coast-then-sag ordering the gated row is blind to (2026-08-23) ─────
#
# `dip_below_x3` searches for the bottom only AFTER the maximum of a window
# that runs to the catch descent, and that maximum can belong to a LATER
# COMMANDED MOVE: the gated arm's prelude climbs back past x3 and overshoots it
# by a measured 0.046-0.222 rev.  When the throw's own coast tops out lower
# than that, the search starts ~200 ms too late and the sag it exists to catch
# is already behind it.  This file's `_synth_fixed_session` docstring has
# carried that as a known blind spot since 2026-07-28 (instrument-defect item
# 10); the 2026-08-23 HAND-7 ladder made it load-bearing — the gated row read
# 0.000 rev on 15 of 15 throws whose coasts finished 0.119-0.481 rev under x3.
#
# The band is an operator ABORT threshold, so the fix does NOT re-arm it here.
# `coast_below_x3_rev` is REPORTED beside the gated row and the printer marks
# the disagreement, so the instrument can no longer pass quietly.

@pytest.mark.parametrize('hold_s', _HOLDS_S)
def test_the_coast_row_sees_the_sag_the_gated_row_misses(probe, hold_s):
    """The blind spot, asserted as a defect rather than described in prose."""
    tl, _ = _arm_prelude_capture(probe, hold_s)
    # the gated row passes: the arm's climb supplied the maximum it searched
    # down from, so the sag is outside its search
    assert tl.dip_below_x3_rev <= probe._DIP_BELOW_X3_BAND_REV
    # the coast row sees it — the built sag is 0.156 rev, plus the synthetic's
    # own 8 ms measurement lag
    assert tl.coast_below_x3_rev == pytest.approx(0.156, abs=0.020)
    assert tl.coast_below_x3_rev > probe._DIP_BELOW_X3_BAND_REV
    assert probe._coast_disagrees(tl) is True


@pytest.mark.parametrize('hold_s', _HOLDS_S)
def test_the_coast_row_is_bounded_at_the_stroke_end_not_the_release(probe,
                                                                   hold_s):
    """The ascent starts at x2 = 5.91 rev and would supply any lower bound.

    A window opening at the release reports a "dip" of ~4 rev on every healthy
    throw, which is not a blind spot but a different wrong answer.  The coast
    opens at the commanded stroke end, so its minimum is a post-stroke quantity.
    """
    tl, _ = _arm_prelude_capture(probe, hold_s)
    model = probe.StrokeModel(3.9308)
    assert tl.coast_min_rev > model.x2_rev
    assert tl.coast_min_rev == pytest.approx(model.x3_rev - 0.156, abs=0.020)
    assert tl.coast_peak_rev <= model.x3_rev + 0.010


def test_a_truncated_stroke_reports_no_coast_at_all(probe):
    """There is no latched stroke top to bound a coast against.

    On a truncation `pos_cmd` never reaches x3, so the row stays blank rather
    than inventing a coast out of the replacement quintic — and the blind-spot
    marker cannot fire on a capture whose gated row is already doing its job.
    """
    tl, _t_freeze, _freeze = _late_truncation_capture(probe)
    assert tl.status == 'ok'
    assert tl.trunc is not None
    assert tl.coast_below_x3_rev is None
    assert tl.coast_peak_rev is None
    assert probe._coast_disagrees(tl) is False
