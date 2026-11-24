"""Shared geometry + the ONLINE dwell detector (FEAT-50), pure Python, synthetic frames."""
from __future__ import annotations

import numpy as np

from jugglebot.tracking import trapped_ball as tb

UNDER = (16.0, 22.0, -26.0)   # event 3's trapped marker, Hand at 2.6 rev
DT = 0.02


def _run(det, pts_fn, t_end, rev=2.6, t_start=0.0):
    ev = []
    n = int(round((t_end - t_start) / DT))
    for i in range(n):
        t = t_start + i * DT
        ev += [(t, e) for e in det.update(t, pts_fn(t, i), rev)]
    return ev


def test_region_cases():
    assert tb.region_of((0.0, -3.0, tb.hand_marker_local_z(2.6)), 2.6) == "HAND_MARKER"
    assert tb.region_of(UNDER, 2.6) == "UNDER_HAND"
    assert tb.region_of((8.0, 23.0, -25.0), 0.0) == "IN_CUP"
    assert tb.region_of((-55.0, 57.0, 3.0), 0.0) == "ON_PLATFORM"
    assert tb.region_of((0.0, 0.0, 400.0), 5.0) is None


def test_fires_after_min_s_and_not_before():
    det = tb.OnlineDwellDetector()
    ev = _run(det, lambda t, i: [(UNDER[0] + 0.3 * np.sin(i), UNDER[1], UNDER[2])], 1.0)
    assert ev, "dwell never reported"
    t_first, e = ev[0]
    assert t_first >= tb.DWELL_MIN_S - 1e-9
    assert t_first < tb.DWELL_MIN_S + 2 * DT
    assert e["hand_rev"] == 2.6
    assert np.allclose(e["local_mean"][:2], [16, 22], atol=1.0)
    # nothing before the threshold
    det2 = tb.OnlineDwellDetector()
    assert _run(det2, lambda t, i: [UNDER], 0.28) == []


def test_repeats_at_one_hertz_while_it_persists():
    det = tb.OnlineDwellDetector()
    ev = _run(det, lambda t, i: [UNDER], 3.5)
    times = [t for t, _e in ev]
    assert len(times) == 4, times
    assert all(abs((b - a) - tb.WARN_REPEAT_S) < 2 * DT for a, b in zip(times, times[1:]))


def test_no_warn_for_cup_platform_or_fixture():
    cup = (8.0, 23.0, -25.0 + tb.HAND_MM_PER_REV * 2.6)
    assert _run(tb.OnlineDwellDetector(), lambda t, i: [cup], 2.0) == []
    assert _run(tb.OnlineDwellDetector(), lambda t, i: [(-55.0, 57.0, 3.0)], 2.0, rev=0.0) == []
    # caveat pinned: with the Hand RAISED the same rim-pocket Ball is geometrically UNDER_HAND
    assert tb.region_of((-55.0, 57.0, 3.0), 2.6) == "UNDER_HAND"
    # a static UNDER_HAND marker present from the start is a fixture once 10 s of
    # history exist: no WARN after that point (it may warn while history builds)
    det = tb.OnlineDwellDetector()
    ev = _run(det, lambda t, i: [UNDER, (0.0, 0.0, 600.0)], 15.0)
    assert all(t < tb.FIXTURE_MIN_S + 1.0 for t, _e in ev)
    assert _run(det, lambda t, i: [UNDER], 20.0, t_start=15.0) == []


def test_short_pass_and_moving_marker_are_not_dwells():
    assert _run(tb.OnlineDwellDetector(), lambda t, i: [UNDER] if t < 0.2 else [], 2.0) == []
    assert _run(tb.OnlineDwellDetector(), lambda t, i: [(-60.0 + 100 * t, 0.0, -100.0)], 1.0) == []


def test_occlusion_gap_shorter_than_gap_s_keeps_the_dwell():
    det = tb.OnlineDwellDetector()
    seen = lambda t, i: [] if 0.20 <= t < 0.35 else [UNDER]   # 0.15 s gap < 0.25 s
    ev = _run(det, seen, 0.6)
    assert ev and ev[0][1]["dur_s"] >= tb.DWELL_MIN_S
    # a gap longer than gap_s restarts the clock
    det = tb.OnlineDwellDetector()
    seen = lambda t, i: [] if 0.20 <= t < 0.60 else [UNDER]
    assert _run(det, seen, 0.85) == []


def test_missing_hand_rev_does_nothing():
    det = tb.OnlineDwellDetector()
    for i in range(100):
        assert det.update(i * DT, [UNDER], None) == []
        assert det.update(i * DT, [UNDER], float("nan")) == []


# ---------------------------------------------------------------------------
# 2026-11-25 audit findings (FEAT-50 held branch)
# ---------------------------------------------------------------------------
def test_jitter_straddling_a_voxel_boundary_is_still_a_fixture():
    # x = 20 is a voxel edge; +-1.5 mm jitter splits the marker ~50/50 over two
    # voxels. Undilated, neither reaches the 90 % bar and it WARNs every second.
    det = tb.OnlineDwellDetector()
    jit = lambda t, i: [(20.0 + (1.5 if i % 2 else -1.5), UNDER[1], UNDER[2]), (0.0, 0.0, 600.0)]
    ev = _run(det, jit, 30.0)
    assert all(t < tb.FIXTURE_MIN_S + 1.0 for t, _e in ev), [t for t, _e in ev]


def test_prune_runs_on_a_timer_not_every_frame():
    class CountingCounter(tb.Counter):
        sweeps = 0

        def items(self):
            CountingCounter.sweeps += 1
            return super().items()

    det = tb.OnlineDwellDetector()
    det._vox = CountingCounter()
    # a marker sweeping through fresh voxels every frame: 60 s of fed frames
    _run(det, lambda t, i: [(-90.0 + 3.0 * (i % 60), 0.0, -100.0 - (i // 60) * 10.0 % 100)], 60.0)
    assert CountingCounter.sweeps <= 60.0 / tb.PRUNE_PERIOD_S + 2, CountingCounter.sweeps
    # and dead voxels are still bounded: nothing older than window + one prune period lingers
    assert len(det._vox) < 27 * (tb.FIXTURE_WINDOW_S + tb.PRUNE_PERIOD_S) / DT


def _alternating_rev_cluster(frac_under_every):
    """Rim-pocket marker, region flips with the Hand: UNDER_HAND when rev 2.6, ON_PLATFORM at rev 0."""
    pt = (-55.0, 57.0, 3.0)
    det = tb.OnlineDwellDetector()
    ev = []
    for i in range(int(2.0 / DT)):
        t = i * DT
        rev = 2.6 if (i % frac_under_every == 0) else 0.0
        ev += det.update(t, [pt], rev)
    return ev


def test_online_labels_a_cluster_by_majority_region_like_the_batch_tool():
    # 1 UNDER_HAND frame in 5: batch labels the cluster ON_PLATFORM, so online must not warn.
    assert _alternating_rev_cluster(5) == []
    # 4 UNDER_HAND frames in 5 (every frame except i % 5 == 0 flips): majority UNDER_HAND -> warns
    pt = (-55.0, 57.0, 3.0)
    det = tb.OnlineDwellDetector()
    ev = []
    for i in range(int(2.0 / DT)):
        ev += det.update(i * DT, [pt], 0.0 if i % 5 == 0 else 2.6)
    assert ev and ev[0]["region"] == "UNDER_HAND"


def test_online_and_batch_share_the_clustering_code_and_agree():
    import os
    import sys
    repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    sys.path.insert(0, os.path.join(repo, "tools"))
    import trapped_ball_finder as tbf
    assert tbf.cluster_add is tb.cluster_add and tbf.cluster_region is tb.cluster_region
    pt = (-55.0, 57.0, 3.0)
    for every in (2, 5):
        frames = [(i * DT, [np.array(pt)], 2.6 if i % every == 0 else 0.0) for i in range(100)]
        batch = tbf.find_dwells(frames, fixture_frac=1.01)   # fixture rules differ by design (whole bag vs rolling)
        det = tb.OnlineDwellDetector()
        online = [e for t, p, r in frames for e in det.update(t, p, r)]
        batch_under = [d for d in batch if d["region"] == "UNDER_HAND"]
        assert bool(online) == bool(batch_under), (every, batch, online)
