"""GapTracker / EpisodeLatch (pure Python): the bridge forwarding-gap monitor."""
from __future__ import annotations

from teensy_link.gap_monitor import (
    EMPTY_STATS, SCHED_COVER_MS, EpisodeLatch, GapTracker)


def test_cover_constant_is_two_knots_plus_grace():
    assert SCHED_COVER_MS == 70.0


def test_empty_and_single_record_have_no_gap():
    t = GapTracker()
    assert t.snapshot(0.0) == EMPTY_STATS
    t.record(1.0)
    assert t.snapshot(1.0) == EMPTY_STATS


def test_max_and_p99_of_the_window():
    t = GapTracker(window_s=1.0)
    now = 10.0
    t.record(now)
    for k in range(98):                       # 98 gaps of 25 ms
        now += 0.025
        t.record(now)
    now += 0.109                              # the 2026-11-01 stall
    t.record(now)
    s = t.snapshot(now)
    assert 30 <= s.n <= 40                    # ~1 s of 25 ms gaps + the stall
    assert abs(s.max_ms - 109.0) < 1e-6
    assert s.p99_ms >= 25.0 - 1e-6


def test_window_expires_old_gaps():
    t = GapTracker(window_s=1.0)
    t.record(0.0)
    t.record(0.109)
    assert t.snapshot(0.2).max_ms > 100.0
    assert t.snapshot(2.0) == EMPTY_STATS


def test_idle_restart_is_not_a_gap():
    t = GapTracker(idle_s=1.0)
    t.record(0.0)
    t.record(30.0)                            # between goals
    assert t.snapshot(30.0) == EMPTY_STATS


def test_episode_latch_fires_once_per_episode():
    e = EpisodeLatch(SCHED_COVER_MS)
    assert not e.update(25.0)
    assert e.update(109.0)
    assert not e.update(120.0)
    assert not e.update(70.0)                 # at the limit = clear
    assert e.update(71.0)                     # new episode
