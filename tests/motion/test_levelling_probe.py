"""The levelling verdict instrument must read BOTH shapes, not just the broken one.

``tools/probes/levelling_tilt_bag_check.py`` is the command
``tests/hardware/session_anomaly_fixes.md`` § CHECK LVL-3 names to turn a capture
into a PASS/ABORT verdict on contract C-LEVEL-1. An instrument validated only
against the *pre-fix* bag (the only real capture that existed when it was written)
would score a working fix as a failure at the bench — which is worse than having
no instrument, because it routes correct work back for rework and burns a powered
sitting.

So this file pins the two-sided acceptance criterion, using series synthesised
through the **production** ``planner.build_catch`` rather than hand-drawn traces:

* post-fix shape  -> ``PASS``
* pre-fix shape   -> ``FAIL``  (the 2026-07-25 defect: park at plan-frame rx = 0)
* post-fix shape contaminated by a long pre-``go_home`` ACTIVATE hold -> ``FAIL``
  **with the ambiguity note**, because that hold is commanded at plan-frame
  ``rx ~ 0`` and is *correctly* uncorrected (C-LEVEL-1's second half: a seed is
  never corrected), so it can win the "longest plateau" vote on a healthy session.

Pure Python — the probe's ``mcap_ros2`` import is lazy (inside ``reconstruct``),
so the scoring half runs with no bag reader and no ROS.
"""

from __future__ import annotations

import importlib.util
import os

import pytest

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_PROBE = os.path.join(_REPO, 'tools', 'probes', 'levelling_tilt_bag_check.py')


def _load_probe():
    spec = importlib.util.spec_from_file_location('levelling_tilt_bag_check',
                                                  _PROBE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def probe():
    return _load_probe()


def test_probe_accepts_the_post_fix_shape(probe):
    """The half a pre-fix-only validation cannot cover."""
    times, poses = probe.synth_series(probe.SELF_CHECK_OFFSET,
                                      corrected_park=True)
    row = probe.score_series(times, poses, offset=probe.SELF_CHECK_OFFSET)
    assert row['verdict'] == 'PASS', row
    assert row['park_rx_deg'] == pytest.approx(-0.77878414, abs=1e-3)
    assert not row['park_ambiguous']


def test_probe_flags_the_pre_fix_shape(probe):
    """The 2026-07-25 defect, in the one number the gate reads."""
    times, poses = probe.synth_series(probe.SELF_CHECK_OFFSET,
                                      corrected_park=False)
    row = probe.score_series(times, poses, offset=probe.SELF_CHECK_OFFSET)
    assert row['verdict'] == 'FAIL', row
    assert row['park_rx_deg'] == pytest.approx(0.0, abs=1e-3)
    # No ambiguity note: nothing in a pre-fix session parks at the corrected pose.
    assert not row['park_ambiguous']


def test_probe_says_so_when_the_activate_hold_won_the_park_vote(probe):
    """A healthy session can still FAIL if the operator dawdles after ACTIVATE.

    Without the note the runbook's ABORT bullet ("park ~ 0 means the pre-fix
    frame — the installed code is stale or the correction never loaded") sends the
    operator hunting a build/QoS fault that does not exist.
    """
    times, poses = probe.synth_series(probe.SELF_CHECK_OFFSET,
                                      corrected_park=True,
                                      activate_hold_s=30.0)
    row = probe.score_series(times, poses, offset=probe.SELF_CHECK_OFFSET)
    assert row['verdict'] == 'FAIL', row
    assert row['park_ambiguous'] is True
    assert '--t0' in row['park_ambiguous_note']


def test_arrival_twist_model_recovers_the_synthesised_catch_lead(probe):
    """``peak_above_park`` is exactly ``(16/81)*rate*|tdir_x|*lead``.

    That is why it is REPORTED and never gated: a threshold on it is really a
    threshold on the catch lead. Inverting it instead gives a cross-check — on the
    post-fix shape the implied lead must recover the lead the reach was built with.
    """
    for lead in (1.2, 2.0, 3.70):
        times, poses = probe.synth_series(probe.SELF_CHECK_OFFSET,
                                          corrected_park=True, lead_s=lead)
        row = probe.score_series(times, poses, offset=probe.SELF_CHECK_OFFSET)
        assert row['implied_lead_s'] == pytest.approx(lead, rel=2e-3), (
            f"lead {lead}: implied {row['implied_lead_s']}")
    assert probe.ARRIVAL_TWIST_BASIS_PEAK == pytest.approx(16.0 / 81.0)


def test_settle_selection_follows_the_correction_sign(probe):
    """A hard-coded ``min()`` would return the PARK for a positive-rx correction.

    The machine's current mount tilts rx negative, so ``min()`` happened to be the
    settle; it is not the settle in general, and a spurious ``settle_err`` is what
    the runbook tells the operator to read as model *confirmation*.
    """
    flipped = (-probe.SELF_CHECK_OFFSET[0], -probe.SELF_CHECK_OFFSET[1])
    times, poses = probe.synth_series(flipped, corrected_park=True)
    row = probe.score_series(times, poses, offset=flipped)
    assert row['verdict'] == 'PASS', row
    assert row['park_rx_deg'] == pytest.approx(+0.77878414, abs=1e-3)
    # The settle is FURTHER from level than the park, in the correction's direction.
    assert row['settle_rx_deg'] > row['park_rx_deg']
    assert row['settle_err_rx_deg'] == pytest.approx(0.0, abs=5e-3)


def test_pre_fix_reference_constants_are_self_consistent(probe):
    """The +3.099 and the +2.3204 in the artefacts are the SAME peak, two frames.

    They were quoted as one quantity ("the pre-fix baseline") in the probe, the
    runbook and the plan, with an ABORT threshold attached to whichever one the
    reader happened to pick up. Pin the arithmetic that relates them.
    """
    assert probe.PRE_FIX_PHYSICAL_PEAK_DEG == pytest.approx(3.0992, abs=1e-4)
    assert (probe.PRE_FIX_PHYSICAL_PEAK_DEG
            - (probe.PRE_FIX_PEAK_DEG - probe.PRE_FIX_PARK_DEG)
            == pytest.approx(0.77878414, abs=1e-6))
    # PRE_FIX_LEAD_S is the catch lead that reproduces the reference bag's peak
    # through the production planner. Note the linear arrival-twist model does NOT
    # apply to the pre-fix shape: there p0 != p1, so the reach also carries the
    # frame term -0.7788*h(s), which is exactly why the pre-fix capture's implied
    # lead reads short (2.94 s against a true 3.70 s).
    times, poses = probe.synth_series(probe.SELF_CHECK_OFFSET,
                                      corrected_park=False,
                                      lead_s=probe.PRE_FIX_LEAD_S)
    row = probe.score_series(times, poses, offset=probe.SELF_CHECK_OFFSET)
    assert row['peak_above_park_deg'] == pytest.approx(probe.PRE_FIX_PEAK_DEG,
                                                       abs=5e-3)
    assert row['implied_lead_s'] < 0.85 * probe.PRE_FIX_LEAD_S
