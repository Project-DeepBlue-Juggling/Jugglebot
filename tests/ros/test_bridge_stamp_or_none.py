"""teensy_link.bridge_stamp.bridge_stamp_or_none (BUG-134): the single rule."""

from __future__ import annotations

from teensy_link.bridge_stamp import MAX_SKEW_US, bridge_stamp_or_none

NOW = 1_700_000_000_000_000


def test_synced_and_close_returns_the_tick_time():
    assert bridge_stamp_or_none(NOW - 12_000, True, NOW) == NOW - 12_000


def test_not_synced_is_none():
    assert bridge_stamp_or_none(NOW - 12_000, False, NOW) is None


def test_missing_or_zero_is_none():
    assert bridge_stamp_or_none(None, True, NOW) is None
    assert bridge_stamp_or_none(0, True, NOW) is None


def test_uptime_epoch_is_none():
    assert bridge_stamp_or_none(7_000_000, True, NOW) is None


def test_skew_boundary_is_inclusive_both_directions():
    assert MAX_SKEW_US == 1_000_000
    assert bridge_stamp_or_none(NOW - 1_000_000, True, NOW) == NOW - 1_000_000
    assert bridge_stamp_or_none(NOW + 1_000_000, True, NOW) == NOW + 1_000_000
    assert bridge_stamp_or_none(NOW - 1_000_001, True, NOW) is None
    assert bridge_stamp_or_none(NOW + 1_000_001, True, NOW) is None


def test_custom_max_skew():
    assert bridge_stamp_or_none(NOW - 500, True, NOW, max_skew_us=500) == NOW - 500
    assert bridge_stamp_or_none(NOW - 501, True, NOW, max_skew_us=500) is None
