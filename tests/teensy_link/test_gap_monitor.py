"""GapTracker / EpisodeLatch (pure Python): the bridge forwarding-gap monitor."""
from __future__ import annotations

from teensy_link.gap_monitor import (
    EMPTY_STATS, HAND_C1_COVER_MS, HAND_C1_STOP_GRACE_MS, SCHED_COVER_MS, EpisodeLatch, GapTracker)


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


def test_hand_c1_cover_mirrors_the_header_grace_and_the_twin():
    """HAND_C1_COVER_MS = 50 ms cover + HAND_C1_STOP_GRACE_US; pinned to the
    firmware header and the Python twin so a grace change cannot go stale here."""
    import os
    import re
    import sys
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    hdr = open(os.path.join(repo, "ros_ws", "src", "jugglebot",
                            "Teensy_code_canbridge", "canbridge_config.h")).read()
    m = re.search(r"HAND_C1_STOP_GRACE_US\s*=\s*(\d+)u", hdr)
    assert m
    assert HAND_C1_STOP_GRACE_MS * 1e3 == int(m.group(1))
    sys.path.insert(0, os.path.join(repo, "tools", "probes",
                                    "teensy_link_profiling", "hermite_xref"))
    import teensy_interp as ti
    assert HAND_C1_COVER_MS * 1e3 == ti.HAND_C1_DECIDE_US


# ── DownlinkLossTracker (JB-174, FW 40) ─────────────────────────────────────

def test_downlink_loss_is_zero_through_in_flight_frames_and_unequal_baselines():
    """The host and firmware counters start at different times and a sample may
    catch a frame or two in flight; neither is loss."""
    from teensy_link.gap_monitor import DownlinkLossTracker
    t = DownlinkLossTracker(window=4)
    # firmware has counted 1000 from an earlier node; this node starts at 0.
    assert t.update(0, 1000) == 0
    assert t.update(4, 1003) == 0      # one in flight
    assert t.update(8, 1008) == 0
    assert t.update(12, 1010) == 0     # two in flight
    assert t.update(16, 1016) == 0
    assert t.lost == 0


def test_downlink_loss_counts_a_setpoint_the_firmware_never_saw():
    from teensy_link.gap_monitor import DownlinkLossTracker
    t = DownlinkLossTracker(window=3)
    t.update(0, 0)
    t.update(4, 4)
    # One setpoint is lost: the firmware is permanently one behind.
    assert t.update(8, 7) == 0         # the window still holds an older clean sample
    assert t.update(12, 11) == 0
    assert t.update(16, 15) == 1       # every sample in the window is one short
    assert t.update(20, 18) == 1       # a second, with one more in flight
    t.update(24, 22)
    assert t.update(28, 26) == 2


def test_downlink_loss_first_sample_caught_in_flight_never_reads_negative():
    from teensy_link.gap_monitor import DownlinkLossTracker
    t = DownlinkLossTracker(window=3)
    assert t.update(5, 3) == 0         # two in flight at the very first sample
    assert t.update(9, 9) == 0         # the true constant shows up later
    assert t.update(13, 13) == 0
    assert t.update(17, 17) == 0
    assert t.lost == 0


def test_downlink_loss_rebaselines_when_either_counter_goes_backwards():
    from teensy_link.gap_monitor import DownlinkLossTracker
    t = DownlinkLossTracker(window=2)
    t.update(0, 0)
    t.update(4, 3)
    assert t.update(8, 7) == 1
    assert t.update(12, 2) == 0        # bridge rebooted: its counter restarted
    assert t.update(16, 6) == 0
    assert t.update(1, 10) == 0        # node restarted: its counter restarted
