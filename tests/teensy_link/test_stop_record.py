"""FEAT-46: the firmware STOP record's decode and classification.

The can-bridge (FW 42, PROTOCOL_VERSION 13) uplinks one STOP record per lane
group in HeartbeatT2J; ``teensy_link.stop_record`` decodes both slots and says
which of the three candidates ended the cover: a stamp gap, a mapping step, or
a due frame left in the queue. Each case below is built the way the firmware
fills the record (``leg_interp.cpp`` sr_capture / sr_note_frame), so a wrong
field index, sign or verdict rule fails here.
"""

from __future__ import annotations

from teensy_link import protocol as p
from teensy_link.stop_record import (
    MAPPING_STEP,
    NONE_OF_THESE,
    QUEUED_NOT_PROMOTED,
    STAMP_GAP,
    StopRecordLog,
    classify_stop_record,
    stop_records,
)

C2 = -70_000      # playing t_start - entry for a C2 stop
_NEXT = int(p.StopRecordFlags.NEXT_VALID)
_TB = int(p.StopRecordFlags.TB_VALID)
_C1 = int(p.StopRecordFlags.HAND_C1)


def _hb(group=0, seq=7, ring=(), nxt=None, depth=0, head=2 ** 31 - 1,
        play=C2, flags=0, tb=None, overwritten=0, entry_wall=1_700_000_000_000_000):
    """A HeartbeatT2J whose slot ``group`` carries one record. ``ring`` and
    ``nxt`` are (origin_rel, start_rel, arrive_rel) tuples, newest first."""
    hb = p.HeartbeatT2J()
    two = lambda v: tuple(v if g == group else 0 for g in (0, 1))  # noqa: E731
    ro, rs, ra = [0] * 8, [0] * 8, [0] * 8
    for j, (o, s, a) in enumerate(ring):
        ro[group * 4 + j], rs[group * 4 + j], ra[group * 4 + j] = o, s, a
    if nxt is not None:
        flags |= _NEXT
    if tb is not None:
        flags |= _TB
    hb.sr_seq = two(seq)
    hb.sr_overwritten = two(overwritten)
    hb.sr_entry_wall_us = two(entry_wall)
    hb.sr_play_start_rel_us = two(play)
    hb.sr_q_head_rel_us = two(head)
    hb.sr_ring_origin_rel_us = tuple(ro)
    hb.sr_ring_start_rel_us = tuple(rs)
    hb.sr_ring_arrive_rel_us = tuple(ra)
    hb.sr_next_origin_rel_us = two(nxt[0] if nxt else 0)
    hb.sr_next_start_rel_us = two(nxt[1] if nxt else 0)
    hb.sr_next_arrive_rel_us = two(nxt[2] if nxt else 0)
    hb.sr_tb_rel_us = two(tb[0] if tb else 0)
    hb.sr_tb_delta_us = two(tb[1] if tb else 0)
    hb.sr_flags = two(flags)
    hb.sr_ring_n = two(len(ring))
    hb.sr_q_depth = two(depth)
    # Through the wire: the decode must survive pack/unpack at 380 B.
    blob = hb.pack()
    assert len(blob) == p.HEARTBEAT_T2J_SIZE == 380
    return p.HeartbeatT2J.unpack(blob)


def _steady(play=C2):
    """The playing frame and the three before it, 25 ms apart, ~20 ms early."""
    return tuple((play - 25_000 * j, play - 25_000 * j, play - 25_000 * j - 20_000)
                 for j in range(4))


def _one(hb, group=0):
    recs = stop_records(hb)
    assert recs[1 - group] is None
    return recs[group]


def test_an_empty_heartbeat_has_no_records():
    assert stop_records(p.HeartbeatT2J()) == (None, None)


def test_decode_reads_the_hand_slot_from_the_second_half_of_the_ring():
    ring = _steady()
    rec = _one(_hb(group=1, ring=ring, flags=_C1, play=-64_000, seq=9), group=1)
    assert rec.group_name == 'Hand' and rec.hand_c1 and rec.seq == 9
    assert [(f.origin_rel_us, f.start_rel_us, f.arrive_rel_us) for f in rec.ring] == list(ring)
    assert rec.q_head_rel_us is None and rec.next is None and rec.tb_rel_us is None


def test_stamp_gap_successor_stamped_150_ms_after_the_playing_frame():
    # The next stamp is 6 knots on: on time (arrived 60 ms before the STOP),
    # but its play time is 80 ms after the STOP.
    ring = ((80_000, 80_000, -60_000),) + _steady()[:3]
    v = classify_stop_record(_one(_hb(ring=ring, depth=1, head=80_000)))
    assert v.verdicts == (STAMP_GAP,)
    assert '+150.0 ms after the playing one' in v.text
    assert 'stamp gap' in v.text and 'queue depth 1, head t_start +80.0 ms' in v.text


def test_mapping_step_contiguous_stamps_mapped_30_ms_late():
    # Successor stamped one knot on (contiguous), but the offset moved by
    # -30 ms before it was queued, so it maps 30 ms later than its stamp says.
    nxt = (-45_000, -15_000, 5_000)
    v = classify_stop_record(_one(_hb(ring=_steady(), nxt=nxt)))
    assert v.verdicts == (MAPPING_STEP,)
    assert 'map shift +30.0 ms' in v.text


def test_mapping_step_from_a_time_base_change_inside_200_ms():
    nxt = (-45_000, -45_000, 2_000)
    v = classify_stop_record(_one(_hb(ring=_steady(), nxt=nxt, tb=(-150_000, -25_000))))
    assert v.verdicts == (MAPPING_STEP,)
    assert '(inside 200 ms)' in v.text


def test_an_old_time_base_change_is_not_a_candidate():
    nxt = (-45_000, -45_000, 2_000)
    v = classify_stop_record(_one(_hb(ring=_steady(), nxt=nxt, tb=(-5_000_000, -25_000))))
    assert v.verdicts == ()
    assert v.primary == NONE_OF_THESE
    assert 'last time-base change -25.0 ms at -5000.0 ms;' in v.text


def test_queued_not_promoted_a_due_newer_frame_sat_in_the_queue():
    # Knot +1 arrived 40 ms before the STOP and was due 45 ms before it.
    ring = ((-45_000, -45_000, -40_000),) + _steady()[:3]
    v = classify_stop_record(_one(_hb(ring=ring, depth=1, head=-45_000)))
    assert v.verdicts == (QUEUED_NOT_PROMOTED,)


def test_a_late_frame_is_none_of_these_with_its_arrival_printed():
    # Contiguous stamp, no mapping change, arrived 30 ms after the STOP.
    nxt = (-45_000, -45_000, 30_000)
    v = classify_stop_record(_one(_hb(ring=_steady(), nxt=nxt)))
    assert v.verdicts == ()
    assert 'none of these' in v.text and 'arrived +30.0 ms' in v.text


def test_an_expired_late_frame_is_not_queued_not_promoted():
    # Arrived before the STOP but 55 ms after its own stamp: promotion drops it
    # as expired (counted), so it is not a promotion fault.
    ring = ((-60_000, -60_000, -5_000),) + _steady()[:3]
    assert QUEUED_NOT_PROMOTED not in classify_stop_record(_one(_hb(ring=ring))).verdicts


def test_record_log_waits_for_the_next_frame_or_one_second():
    log = StopRecordLog()
    pending = _hb(seq=3, ring=_steady())
    assert log.update(pending, 10.0) == []           # no next frame yet
    assert log.update(pending, 10.5) == []
    lines = log.update(pending, 11.0)                # 1 s after it first appeared
    assert len(lines) == 1 and lines[0].startswith('STOP record #3 legs (C2)')
    assert 'next frame not yet queued' in lines[0]
    assert log.update(pending, 12.0) == []           # once per record
    filled = _hb(seq=4, ring=_steady(), nxt=(-45_000, -45_000, 2_000))
    lines = log.update(filled, 12.1)                 # next part filled: at once
    assert len(lines) == 1 and '#4' in lines[0]
