"""tools/trapped_ball_finder.py on synthetic marker data (no bag needed).

Thresholds were fixed from the three 2026-11-13 stalls (see the tool's header): the
Hand's own marker rides the axis at z = -47 + 32.567 mm/rev x rev, and the trapped
marker of 13:15:22.69 sat at about (16, 22, -26) mm in the Platform frame at Hand 2.6 rev.
"""
from __future__ import annotations

import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "tools"))

import trapped_ball_finder as tbf  # noqa: E402


def test_hand_marker_is_not_a_ball():
    for rev in (0.0, 2.6, 9.4):
        z = tbf.hand_marker_local_z(rev)
        assert tbf.region_of((0.0, -3.0, z), rev) == "HAND_MARKER"


def test_event3_marker_is_under_hand():
    assert tbf.region_of((16.0, 22.0, -26.0), 2.6) == "UNDER_HAND"


def test_ball_seated_in_cup_is_not_a_candidate():
    assert tbf.region_of((8.0, 23.0, -25.0), 0.0) == "IN_CUP"
    assert tbf.region_of((8.0, 23.0, -25.0 + 32.567 * 5), 5.0) == "IN_CUP"
    # same (x, y) but the Cup is 80 mm above: that is the event-3 trapped marker
    assert tbf.region_of((15.0, 23.0, -20.0), 2.6) == "UNDER_HAND"


def test_regions():
    assert tbf.region_of((150.0, 0.0, 10.0), 5.0) == "ON_PLATFORM"
    assert tbf.region_of((0.0, 0.0, 400.0), 5.0) is None      # airborne
    assert tbf.region_of((400.0, 0.0, 0.0), 5.0) is None      # beyond the Platform
    assert tbf.region_of((20.0, 20.0, -600.0), 5.0) is None   # on the floor


def test_to_platform_frame_identity_and_yaw():
    pose = (10.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    out = tbf.to_platform_frame(np.array([[10.0, 20.0, tbf.INITIAL_HEIGHT_MM + 5.0]]), pose)
    assert np.allclose(out, [[0, 0, 5.0]])
    s = np.sqrt(0.5)  # +90 deg about z: platform x axis = world y
    out = tbf.to_platform_frame(np.array([[0.0, 30.0, tbf.INITIAL_HEIGHT_MM]]), (0, 0, 0, 0, 0, s, s))
    assert np.allclose(out, [[30.0, 0.0, 0.0]], atol=1e-9)


def _frames(ball_from, ball_to, rev=2.6, dt=0.02, hand_marker=True):
    out = []
    n = int((ball_to + 6.0) / dt)  # 6 s of Ball-free frames: keeps it under the fixture fraction
    for i in range(n):
        t = i * dt
        pts = []
        if hand_marker:
            pts.append((0.0, -3.0, tbf.hand_marker_local_z(rev)))
        if ball_from <= t < ball_to:
            pts.append((16.0 + 0.3 * np.sin(i), 22.0, -26.0))
        out.append((t, pts, rev))
    return out


def test_dwell_found_and_hand_marker_ignored():
    d = tbf.find_dwells(_frames(1.0, 2.0))
    assert len(d) == 1
    assert d[0]["region"] == "UNDER_HAND"
    assert 0.9 < d[0]["dur_s"] <= 1.0
    assert np.allclose(d[0]["local_mean"][:2], [16, 22], atol=1.0)


def test_short_pass_is_not_a_dwell():
    assert tbf.find_dwells(_frames(1.0, 1.1)) == []


def test_moving_marker_is_not_a_dwell():
    frames = [(0.02 * i, [(-100 + 5.0 * i * 0.02 * 50, 0.0, 10.0)], 5.0) for i in range(100)]
    assert tbf.find_dwells(frames) == []


def test_fixture_marker_filtered():
    frames = [(0.02 * i, [(80.0, 0.0, 10.0)], 5.0) for i in range(200)]  # always there
    assert tbf.find_dwells(frames) == []


def _hand(stall):
    t = np.arange(0, 3, 0.01)
    pos = np.full_like(t, 9.0)
    cmd = pos.copy()
    vel = np.zeros_like(t)
    iq = np.zeros_like(t)
    if stall:
        m = (t > 1.0) & (t < 1.2)
        pos[m] = 2.6
        cmd[m] = 0.6
        iq[m] = -70.0
    return np.stack([t, cmd, pos, vel, iq], axis=1)


def test_stall_detected_and_coincides():
    st = tbf.find_stalls(_hand(True))
    assert len(st) == 1 and abs(st[0]["peak_iq_a"] - 70.0) < 1e-9
    d = tbf.find_dwells(_frames(1.05, 2.0))
    tbf.coincide(st, d)
    assert st[0]["trapped"] == [0]


def test_stall_without_marker_and_no_stall_without_clamp():
    st = tbf.find_stalls(_hand(True))
    tbf.coincide(st, [])
    assert st[0]["trapped"] == []
    assert tbf.find_stalls(_hand(False)) == []


def test_trace_ball_picks_closest_recent_sample():
    balls = [(0.0, 7, 0.0, 0.0, 500.0, 1), (4.5, 8, 100.0, 0.0, 50.0, 1), (4.9, 8, 60.0, 0.0, 30.0, 3), (-9.0, 3, 60.0, 0.0, 30.0, 1)]
    r = tbf.trace_ball((60.0, 0.0, 30.0), 5.0, balls)
    assert r["ball_id"] == 8 and r["dist_mm"] < 1e-9
    assert tbf.trace_ball((0, 0, 0), 100.0, balls) is None


def test_short_marker_at_a_stall_counts_below_the_listing_threshold():
    """Event 1 of 2026-11-13: the marker sat only ~0.13 s under the Hand."""
    frames = _frames(1.02, 1.15)
    assert tbf.find_dwells(frames) == []                       # not listed as a dwell
    d = tbf.find_dwells(frames, min_s=tbf.STALL_DWELL_MIN_S)
    st = tbf.find_stalls(_hand(True))
    tbf.coincide(st, d)
    assert len(st[0]["markers"]) == 1


def test_marker_on_the_platform_top_does_not_explain_a_stall():
    frames = [(0.02 * i, [(-55.0, 56.0, 3.0)], 9.0) for i in range(100)]  # at Hand 9 rev, above the Hand marker? no: below it
    d = tbf.find_dwells(frames + [(2.0 + 0.02 * i, [], 9.0) for i in range(300)], min_s=tbf.STALL_DWELL_MIN_S)
    for x in d:
        x["region"] = "ON_PLATFORM"
    st = tbf.find_stalls(_hand(True))
    tbf.coincide(st, d)
    assert st[0]["trapped"] == []
