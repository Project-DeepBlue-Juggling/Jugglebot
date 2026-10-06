"""BB calibration's consensus axis gate (2026-10-06).

The axis-deviation gate used to require EVERY fitted marker's circle centre
within 3 mm of the shared axis, so one bad marker refused the whole
calibration. On 2026-10-05/06 that was 11 of 14 hardware attempts, every one
the same marker (QTM 1), whose parked readings crept ~3 mm relative to the
rest of the constellation while the other six agreed within 2.3 mm.

Now the largest subset of at least ``MIN_AGREEING_MARKERS`` that agrees within
``MAX_AXIS_DEVIATION_MM`` defines the axis; a marker left out is an outcast —
reported, excluded from the axis AND the plane height — and the calibration
stands. Each outcast test below would have raised under the old rule.

What stays closed: three or more outcasts, an outcast with too few markers to
out-vote it, and an outcast yaw anchor (whose parked angle IS the yaw offset).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from jugglebot.bb_calibration import (
    BB_MARKER_COUNT,
    BB_YAW_ANCHOR_INDEX,
    MAX_AXIS_DEVIATION_MM,
    MIN_AGREEING_MARKERS,
    run_calibration,
)


BB_POS = np.array([-975.0, -389.0, 1718.0])
PITCH_Z_OFFSET_MM = 17.5
RADII = (136.0, 135.0, 117.0, 117.0, 99.0, 118.0, 76.0)
Z_OFFSETS = (-26.0, -26.0, 0.0, 0.0, 0.0, 0.0, 0.0)
PHASES = tuple(math.radians(d) for d in (30.0, 0.0, 60.0, 130.0, 200.0, 260.0, 320.0))
AXIS = np.array([0.0, 0.0, 1.0])
U = np.array([1.0, 0.0, 0.0])
V = np.array([0.0, 1.0, 0.0])
#: An outcast's offset: 5 mm sideways, well past MAX_AXIS_DEVIATION_MM.
SHIFT_MM = 5.0


def _dataset(shifted=(), shift_z_mm=0.0, present=range(BB_MARKER_COUNT)):
    """120° sweep then a hold; markers in *shifted* read SHIFT_MM sideways
    (and *shift_z_mm* up) throughout — a rigidly offset circle."""
    data = {i: [] for i in range(BB_MARKER_COUNT)}
    for i in present:
        centre = BB_POS + Z_OFFSETS[i] * AXIS
        if i in shifted:
            # Each outcast off in its own direction, so two cannot agree.
            d = math.radians(97.0 * i)
            centre = centre + SHIFT_MM * (math.cos(d) * U + math.sin(d) * V)
            centre = centre + shift_z_mm * AXIS
        ang = PHASES[i] + np.linspace(0.0, math.radians(120.0), 200)
        sweep = [centre + RADII[i] * (math.cos(a) * U + math.sin(a) * V) for a in ang]
        data[i] = sweep + [sweep[-1]] * 250   # > the 200-sample yaw-offset window
    return data


def _yaw():
    return list(np.linspace(0.0, 120.0, 40)) + [120.0] * 20


def _run(data):
    return run_calibration(data, _yaw(), pitch_z_offset_mm=PITCH_Z_OFFSET_MM)


def test_gate_constants():
    """3.0 mm is the pre-consensus value, kept: the healthy markers spread to
    2.3 mm on hardware. 5 of 7 lets up to two be out-voted."""
    assert MAX_AXIS_DEVIATION_MM == 3.0
    assert MIN_AGREEING_MARKERS == 5


def test_clean_constellation_has_no_outcast():
    res = _run(_dataset())
    assert all(m.status == 'ok' for m in res.marker_metrics.values())
    assert res.bb_position_mm[:2] == pytest.approx(BB_POS[:2], abs=0.01)


def test_one_outcast_is_excluded_and_named():
    """The 2026-10-06 case: QTM 1 off, the other six agree. The old gate
    raised; now QTM 1 is the outcast at its true distance from the consensus
    axis, and the position is the truth, not pulled toward it."""
    res = _run(_dataset(shifted=(0,)))
    m = res.marker_metrics[0]
    assert m.status == 'outcast'
    assert m.distance_from_axis_mm == pytest.approx(SHIFT_MM, abs=0.01)
    assert '6 markers agree' in m.reason
    assert [i for i, q in res.marker_metrics.items() if q.status == 'ok'] == [1, 2, 3, 4, 5, 6]
    assert res.bb_position_mm[:2] == pytest.approx(BB_POS[:2], abs=0.01)


def test_two_outcasts_leave_exactly_the_minimum():
    res = _run(_dataset(shifted=(0, 1)))
    assert {i for i, q in res.marker_metrics.items() if q.status == 'outcast'} == {0, 1}
    assert res.bb_position_mm[:2] == pytest.approx(BB_POS[:2], abs=0.01)


def test_three_outcasts_refuse_with_per_marker_breakdown():
    """Below MIN_AGREEING_MARKERS the gate refuses, and the message carries
    every marker's deviation — the breakdown the failures never logged."""
    with pytest.raises(ValueError) as exc:
        _run(_dataset(shifted=(0, 1, 6)))
    msg = str(exc.value)
    assert 'deviate' in msg
    for k in range(1, BB_MARKER_COUNT + 1):
        assert f'Marker {k} ' in msg


def test_no_vote_without_a_majority():
    """With only five markers fitted there is nobody to out-vote one: the
    pre-consensus rule applies and a single offset marker refuses."""
    with pytest.raises(ValueError, match='deviate'):
        _run(_dataset(shifted=(2,), present=(0, 2, 3, 4, 5)))


def test_outcast_yaw_anchor_refuses():
    """The yaw offset IS the anchor's parked angle; an anchor the others
    out-voted is biased by the same order, so fail closed."""
    with pytest.raises(ValueError) as exc:
        _run(_dataset(shifted=(BB_YAW_ANCHOR_INDEX,)))
    # The operator line must say WHY an outcast fails here when one elsewhere
    # does not: the yaw offset has no other source.
    assert str(exc.value).startswith(
        'Yaw-anchor Marker 4 is the outcast, so the yaw offset cannot be '
        'estimated reliably — it is read from this marker alone: circle centre ')


def test_outcast_plane_marker_is_left_out_of_the_plane_height():
    """A co-planar outcast reading 30 mm high would lift the plane by 6 mm if
    its Z still counted."""
    res = _run(_dataset(shifted=(2,), shift_z_mm=30.0))
    assert res.marker_metrics[2].status == 'outcast'
    assert res.bb_position_mm[2] == pytest.approx(BB_POS[2] + PITCH_Z_OFFSET_MM, abs=0.01)
