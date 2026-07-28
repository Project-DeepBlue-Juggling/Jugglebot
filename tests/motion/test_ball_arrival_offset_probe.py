"""CI guard for ``tools/probes/ball_arrival_offset.py``.

The probe is the SCORING INSTRUMENT for the pre-registered seat-rate A/B
(``tests/hardware/session_anomaly_fixes.md`` SECTION SEAT-EXP): it turns a
capture into each ball's measured arrival offset, which is what "compare
bounce-outs at MATCHED arrival offsets" requires.

That makes it exactly the class of thing the 2026-07-27 run close-out was burned
by — a runbook citing an analysis command whose behaviour nobody had checked. Two
of its five defects were instruments (a criterion that fired on correct
behaviour; a gate with no probe in the repo that could read it). So the probe's
own acceptance runs in CI, and the one bias that would invalidate the experiment
is mutation-verified rather than asserted.

Bag-dependent behaviour is NOT tested here (no bag in CI). The bag validation is
recorded in the probe's docstring with measured numbers against
``logbook/2026-07-28-anomaly-fixes-validation-sitting.md``.
"""

from __future__ import annotations

import importlib.util
import math
import os

import pytest

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_PROBE = os.path.join(_REPO, 'tools', 'probes', 'ball_arrival_offset.py')


@pytest.fixture(scope='module')
def probe():
    spec = importlib.util.spec_from_file_location('ball_arrival_offset', _PROBE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_probe_file_exists():
    """SECTION SEAT-EXP cites this path; a missing file is a dead runbook row."""
    assert os.path.isfile(_PROBE)


def test_self_check_passes(probe, capsys):
    """Instrument acceptance, in CI rather than at the bench."""
    rc = probe.self_check()
    capsys.readouterr()
    assert rc == 0


def test_post_contact_exclusion_is_load_bearing(probe):
    """MUTATION: without the descending-branch cut, the fit moves — a lot.

    This is the bias that would invalidate the whole experiment, and it is worth
    proving rather than trusting. A bounce-out's post-contact excursion is large
    (measured up to ~70 mm of ``x`` travel on ball 6 of the 2026-07-27 sitting)
    and points in a *different direction on different balls*. If those samples
    entered the fit, each track's measured "arrival offset" would be perturbed by
    an amount that CORRELATES WITH ITS OUTCOME — so the A/B would compare
    bounce-outs against catches at offsets that were themselves shifted by
    whether the ball bounced. That is not a noisy instrument, it is a
    confounded one.

    Here: take a clean descent, graft a violent post-contact excursion on below
    the plane, and confirm (a) the real fit is untouched and (b) a fit that
    naively used every sample in the band would move by more than a millimetre.
    """
    rows = probe._synthetic(-1200.0, 300.0, 200.0, -80.0, 5000.0, 1600.0)
    clean = probe.fit_plane_crossing(rows, 1000.0, probe.BAND_MM)
    assert clean is not None

    # Post-contact samples that sit back INSIDE the fit band in z (a ball
    # deflected upward off the rim does exactly this — ball 11 reappeared
    # 345 mm above its free-fall prediction).
    t_last = rows[-1][0]
    bounced = list(rows) + [(t_last + i * 0.005, 400.0, -400.0, 1000.0 + 3.0 * i)
                            for i in range(1, 40)]
    with_cut = probe.fit_plane_crossing(bounced, 1000.0, probe.BAND_MM)
    assert with_cut is not None
    assert with_cut[0] == pytest.approx(clean[0], abs=1e-9)

    # The mutation: no descending-branch cut, i.e. fit everything in the band.
    band = [r for r in bounced if 1000.0 <= r[3] <= 1000.0 + probe.BAND_MM]
    n = len(band)
    z_mean = sum(r[3] for r in band) / n
    szz = sum((r[3] - z_mean) ** 2 for r in band)
    x_mean = sum(r[1] for r in band) / n
    slope = sum((r[3] - z_mean) * (r[1] - x_mean) for r in band) / szz
    naive_x = x_mean + slope * (1000.0 - z_mean)
    assert abs(naive_x - clean[0]) > 1.0, (
        'the mutation must actually move the answer, or this test proves '
        f'nothing (cut {clean[0]:.3f} vs naive {naive_x:.3f})')


def test_a_sparse_track_is_reported_not_hidden(probe):
    """``n`` must survive to the caller so SPARSE can be flagged.

    Ball 11 of the reference sitting was lost by mocap at ``z = 1111`` and never
    re-acquired on the way in, so its offset is an extrapolation from three
    samples — and it is one of the three bounce-outs the whole experiment is
    about. An instrument that returned that number without saying how thin it was
    would let the decision rule weight it exactly like a 13-sample track.
    """
    rows = [r for r in probe._synthetic(-1200.0, 300.0, 200.0, -80.0, 5000.0,
                                        1600.0) if r[3] > 1150.0]
    got = probe.fit_plane_crossing(rows, 1000.0, probe.BAND_MM)
    assert got is not None
    _x, _y, n, z_lo, z_hi = got
    assert n == len([r for r in rows if 1000.0 <= r[3] <= 1300.0])
    assert z_lo > 1150.0 and z_hi <= 1300.0


def test_stats_refuse_to_invent_a_zero_sd(probe):
    """A one-sample block must not read as perfectly repeatable.

    The pre-registered decision rule divides a block-mean gap by the pooled
    within-block sd. A block that lost all but one track to occlusion would, with
    a naive ``sd = 0``, make ANY mean gap infinitely significant — turning the
    instrument's own data loss into a decisive experimental result.
    """
    assert probe.stats([])[0] == 0
    assert probe.stats([-12.0]) == (1, -12.0, None)
    n, mean, sd = probe.stats([-10.0, -30.0])
    assert (n, mean) == (2, -20.0)
    assert sd == pytest.approx(math.sqrt(200.0))


def test_group_spec_parses_and_rejects_garbage(probe):
    assert probe.parse_group('A=7-18') == ('A', 7, 18)
    with pytest.raises(SystemExit):
        probe.parse_group('A=seven-eighteen')
