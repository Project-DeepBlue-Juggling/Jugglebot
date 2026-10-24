"""LegClampDiag (0x94) codec pins — can-bridge FW 31, plan leg-lead-clamp-age-aware P1.

48 B = u32[6] lead_clamp_ticks + u32[6] stale_anchor_ticks, cumulative since
boot, additive (no PROTOCOL_VERSION bump).
"""

from __future__ import annotations

import struct

import pytest

from teensy_link import LegClampDiag, MsgType
from teensy_link.protocol import LEG_CLAMP_DIAG_SIZE


def test_msg_type_and_size_are_frozen():
    assert int(MsgType.LEG_CLAMP_DIAG) == 148
    assert LEG_CLAMP_DIAG_SIZE == 48


def test_round_trip_with_distinct_per_leg_values():
    """Distinct values everywhere: a leg transposition or a clamp/stale swap fails."""
    d = LegClampDiag(lead_clamp_ticks=(1, 20, 300, 4000, 50000, 600000),
                     stale_anchor_ticks=(7, 80, 900, 10000, 110000, 4294967295))
    raw = d.pack()
    assert len(raw) == LEG_CLAMP_DIAG_SIZE
    back = LegClampDiag.unpack(raw)
    assert tuple(back.lead_clamp_ticks) == d.lead_clamp_ticks
    assert tuple(back.stale_anchor_ticks) == d.stale_anchor_ticks


def test_short_payload_is_refused():
    """A 47 B payload raises struct.error (NOT ValueError) - which is why the
    node's RX callback must catch broadly."""
    good = LegClampDiag(lead_clamp_ticks=(1, 2, 3, 4, 5, 6),
                        stale_anchor_ticks=(6, 5, 4, 3, 2, 1)).pack()
    with pytest.raises(struct.error):
        LegClampDiag.unpack(good[:LEG_CLAMP_DIAG_SIZE - 1])
    assert LegClampDiag.unpack(good[:LEG_CLAMP_DIAG_SIZE]) is not None
