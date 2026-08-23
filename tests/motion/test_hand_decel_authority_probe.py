"""``hand_decel_authority.py`` — the window the inertia identification lives in.

``tools/probes/hand_decel_authority.py`` is the instrument contract **C-HAND-2**
(``ros_ws/docs/hand_decel_feedforward.md``) names as *"the empirical probe behind
every number here"*, and § CHECK HAND-7 of ``tests/hardware/session_anomaly_fixes.md``
routes rows **H7.2** (peak), **H7.3** (flatness) and **H7.5** (decel current) to
it as the sole authority.  Until this file existed nothing in the suite executed
it: the probe's own two-sided ``--self-check`` was reachable only from a runbook
step an operator types by hand, so a regression in it surfaced on a powered
sitting or not at all.

WHAT THESE TESTS PIN, AND WHY EACH ONE COST SOMETHING
-----------------------------------------------------
Two defects in the same class were found on 2026-08-23, both of them silent, and
both of them upstream of every empirical number in C-HAND-2:

1. **The window.**  ``peak`` was the maximum ``pos_meas`` over the ~400 ms after
   the commanded stroke end, and ``over_x3`` / ``eta`` / the decel-side inertia
   bound were all derived from it.  C-HAND-1's gated catch arm dispatches its
   prelude 37-183 ms after the stroke completes and climbs back past ``x3``,
   overshooting it by a measured 0.046-0.222 rev — so on any throw whose own
   coast tops out lower, the reported ``peak`` describes **the arm's move, ~200
   ms after the throw ended**.  Measured contamination: 10 of the 17 tosses of
   the 2026-07-27 sitting *every C-HAND-2 number is drawn from* (including both
   low tiers on every toss, where the reported ``over_x3`` is ``+0.074`` /
   ``+0.063`` rev while the coasts actually finished ``-0.051`` / ``-0.092`` rev
   — i.e. BELOW ``x3``), and **15 of 15** on the first HAND-7 ladder flown on the
   unclamped drive.

2. **The torque.**  The per-tier bound multiplied ``tor_legacy`` —
   ``accelToTorque``, the pre-C-HAND-2 feedforward — long after Platform FW 2 put
   ``throwDecelToTorque`` aboard, understating the shipped braking torque by
   1.289x and the bound with it.  The capture carries the answer directly:
   ``buildSegment`` stops one 500 Hz sample short of ``t3``, so the last
   commanded frame latches the FULL decel feedforward across the coast.

The fix is one idea applied at both points — *identify the deceleration from the
throw's own coast, and read the torque that braked it off the wire* — and the
tests below assert the defect first: each one fails against the pre-2026-08-23
behaviour, which was verified by mutating the window back and watching
``--self-check`` go RED (0.1458 rev of recovered-vs-built error).

``peak`` deliberately KEEPS the wide window.  The end-stop question is "what is
the largest excursion of any cause", and on a healthy capture the arm's own
overshoot is routinely the answer; narrowing that column would trade a
measurement bug for a safety hole.  One test pins that separation.
"""

from __future__ import annotations

import importlib.util
import os
import sys

import pytest

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_PROBE = os.path.join(_REPO, 'tools', 'probes', 'hand_decel_authority.py')


def _load_probe():
    spec = importlib.util.spec_from_file_location('hand_decel_authority', _PROBE)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['hand_decel_authority'] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def probe():
    return _load_probe()


def _score(probe, tmp_path_factory, overs, name):
    """Write a synthetic capture and score it through the shipped ``analyse``.

    Built with the arm prelude in it, which is the whole point: without that
    event in the capture the window defect passes its own gate.
    """
    import json
    model = probe.Model()
    hand, ann = probe._synth_session(model, overs)
    path = tmp_path_factory.mktemp('hda') / (name + '.jsonl')
    with path.open('w') as fh:
        for s in hand:
            fh.write(json.dumps(dict(
                t=s[0], t_ros=s[0], topic='hand_telemetry', n=0,
                d=dict(pos_meas=s[1], vel_meas=s[2], pos_cmd=s[3],
                       vel_ff_cmd=s[4], tor_ff_cmd=s[5], iq_meas=s[6]))) + '\n')
        for a in ann:
            fh.write(json.dumps(dict(t=a['throw_time'], t_ros=a['throw_time'],
                                     topic='throw_announcements', n=0,
                                     d=a)) + '\n')
    _m, rows, _stale = probe.analyse(path)
    return model, rows


@pytest.fixture(scope='module')
def scored(probe, tmp_path_factory):
    """The POST-FIX synthetic: a velocity-independent 0.426 rev coast."""
    return _score(probe, tmp_path_factory, probe._SELF_CHECK_POST_OVER, 'post')


@pytest.fixture(scope='module')
def scored_pre(probe, tmp_path_factory):
    """The PRE-FIX synthetic: the 2026-07-27 per-tier group means.

    This is the shape where the window defect BITES — the two low tiers coast
    0.074 and 0.063 rev past x3, both smaller than the arm prelude's 0.222 rev
    overshoot, so an unbounded peak search reports the arm on those two and the
    throw on the other two.
    """
    return _score(probe, tmp_path_factory, probe._SELF_CHECK_PRE_OVER, 'pre')


def test_the_operator_self_check_passes(probe, capsys):
    """§ CHECK HAND-7 row **H7.0b / INST-6**, in the suite rather than by hand.

    A ``BAD`` line here is what an operator would otherwise discover at the
    bench, after powering the robot.
    """
    assert probe._self_check() == 0
    assert 'SELF-CHECK: PASS' in capsys.readouterr().out


def test_over_x3_measures_the_coast_and_not_the_arm_prelude(probe, scored):
    """Defect 1, asserted directly.

    The synthetic builds a velocity-INDEPENDENT 0.426 rev coast at every tier
    and an arm prelude that overshoots ``x3`` by 0.222 rev.  With the
    pre-2026-08-23 unbounded window the recovered ``over_x3`` picks up whichever
    of the two is larger; with the coast bound it is always the built one.
    """
    _model, rows = scored
    for r, built in zip(rows, probe._SELF_CHECK_POST_OVER):
        assert r['over_x3'] == pytest.approx(built, abs=0.02)
        assert r['coast_peak'] < r['peak'] or r['peak_is_coast']


def test_a_coast_smaller_than_the_arm_still_reports_the_coast(probe, scored_pre):
    """Defect 1 where it actually bit: the two LOW tiers of the founding capture.

    Both are smaller than the arm prelude's overshoot, so the pre-2026-08-23
    window reported the arm's 0.222 rev on them and the throw's own on the two
    upper tiers — a per-tier flatness statistic computed across two different
    physical events.  ``peak_is_coast`` is the fingerprint: on exactly those
    tiers the safety column and the identification column now disagree, and the
    report marks them with a ``*``.
    """
    _model, rows = scored_pre
    for r, built in zip(rows, probe._SELF_CHECK_PRE_OVER):
        assert r['over_x3'] == pytest.approx(built, abs=0.02)
        assert r['peak_is_coast'] is (built > probe._SELF_CHECK_ARM_OVER)
    contaminated = [r for r in rows if not r['peak_is_coast']]
    assert len(contaminated) == 2, (
        'the synthetic must keep contaminating SOME tiers and not others — a '
        'fixture that contaminates all of them or none of them would pass '
        'against a probe that simply always reports one of the two events')


def test_the_end_stop_column_still_sees_the_later_move(probe, scored):
    """The half of the fix that is a NON-change, and has to stay one.

    ``peak`` guards the 10.8 rev hard stop.  Bounding it to the coast would hide
    exactly the excursions that approach the stop on a healthy capture, so the
    coast bound applies to the identification columns only.
    """
    model, rows = scored
    for r in rows:
        # every post-fix tier's own coast (0.426 rev) is larger than the arm's
        # 0.222 rev overshoot, so `peak` is the coast here...
        assert r['peak'] >= r['coast_peak']
        # ...and the column that would have to see a bigger arm is the wide one
        assert r['peak'] >= model.x3_rev + probe._SELF_CHECK_ARM_OVER - 0.25


def test_the_wire_feedforward_is_read_off_the_capture(probe, scored):
    """Defect 2.  ``tau_wire`` must be the CORRECTED feedforward, quantised.

    Legacy and corrected are never confusable on a real capture — the
    2026-08-23 ladder reads 0.050 / 0.090 / 0.110 / 0.150 / 0.170 N.m against a
    legacy model of 0.042 / 0.068 / 0.088 / 0.113 / 0.135 — so a probe that
    re-derives instead of reading has no excuse for getting the generation
    wrong.
    """
    model, rows = scored
    for r in rows:
        m = model.at(r['v_cmd'])
        corrected = round(m['tor_corrected'] * 100.0) / 100.0
        legacy = round(m['tor_legacy'] * 100.0) / 100.0
        assert r['tau_wire'] == pytest.approx(corrected, abs=5e-4)
        assert abs(r['tau_wire'] - legacy) > 5e-4, (
            'legacy and corrected must stay distinguishable at every tier, or '
            'this assertion proves nothing')


def test_the_coast_carries_the_over_brake_side(probe, scored):
    """``under_x3`` exists because ``peak`` cannot express a shortfall.

    A hand that stops SHORT of the stroke top is the over-brake failure
    C-HAND-2's one-sided-safety clause exists to prevent, and no maximum over
    any window can report it.  The post-fix synthetic overshoots, so the column
    reads zero here; the assertion is that the column is present and signed the
    right way, which is what the 2026-08-23 capture then exercised for real
    (0.113-0.468 rev).
    """
    model, rows = scored
    for r in rows:
        assert r['under_x3'] >= 0.0
        assert r['under_x3'] == pytest.approx(
            max(0.0, model.x3_rev - r['coast_min']), abs=1e-4)


def test_eta_above_one_means_the_hand_stopped_short(probe, scored):
    """The sign convention a reader has to be able to trust.

    ``eta = d_dec / (d_dec + over_x3)``, so an undershoot (negative
    ``over_x3``) puts eta ABOVE 1.  On the 2026-08-23 ladder every tier read
    1.014-1.078, which is the reading that inverted the package's expected
    conclusion — it is worth pinning that the arithmetic says what the docstring
    claims.
    """
    _model, rows = scored
    for r in rows:
        if r['over_x3'] > 0:
            assert r['eta'] < 1.0
        elif r['over_x3'] < 0:
            assert r['eta'] > 1.0
