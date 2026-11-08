"""The firmware STOP record (FEAT-46, can-bridge FW 42, PROTOCOL_VERSION 13).

Every scheduler STOP on the can-bridge (the 70 ms cover + grace running out, or
the Hand C1 stop at 64 ms) writes a record for its lane group (legs or Hand),
and ``HeartbeatT2J`` re-sends the latest record per group at 10 Hz. This module
decodes the two slots and says why the cover ended.

The cover runs from each frame's ``t_start``: its stamp ``t_origin_us`` mapped
onto the bridge's monotonic clock through the wall offset when the frame was
queued. A STOP on a clean host stream therefore has three candidates, and the
record separates them:

* **stamp gap**: the next stamp after the playing frame is more than one knot
  (25 ms) later, so the frames arrived on time but their play times left a
  hole (a Splice, a re-plan);
* **mapping step**: the wall offset moved between the playing frame and its
  successor (``start_rel - origin_rel`` differs), or the bridge logged an
  offset change > 1 ms in the 200 ms before the STOP;
* **queued, not promoted**: a frame newer than the playing one was in the
  queue and due before the STOP, yet the STOP fired.

None of these is "a late frame": that reads as ``none of these`` with the
successor's arrival time printed. Pure Python, no ROS imports.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from teensy_link import protocol as p

RING = 4
KNOT_US = 25_000
#: Successive stamps more than one knot apart (1 ms of slack for rounding).
STAMP_GAP_US = KNOT_US + 1_000
#: A mapped-t_start shift larger than this between two frames is a mapping step.
MAP_STEP_US = 1_000
#: A time-base change this close before the STOP is reported as a candidate.
TB_WINDOW_US = 200_000
#: The firmware's promotion drops a frame this late as expired (2T cover).
COVER_US = 2 * KNOT_US
#: q_head_rel_us when nothing was queued for the group.
NO_QUEUE = 2 ** 31 - 1

_HAND_C1 = int(p.StopRecordFlags.HAND_C1)
_NEXT_VALID = int(p.StopRecordFlags.NEXT_VALID)
_TB_VALID = int(p.StopRecordFlags.TB_VALID)

GROUP_NAMES = ('legs', 'Hand')

STAMP_GAP = 'stamp gap'
MAPPING_STEP = 'mapping step'
QUEUED_NOT_PROMOTED = 'queued, not promoted'
NONE_OF_THESE = 'none of these'


@dataclass(frozen=True)
class FrameNote:
    """One queued frame, relative to the STOP (us)."""

    origin_rel_us: int   # t_origin_us - the STOP entry on the wall clock
    start_rel_us: int    # mapped t_start - the STOP entry (bridge monotonic)
    arrive_rel_us: int   # queueing time - the STOP entry (bridge monotonic)

    @property
    def map_shift_us(self) -> int:
        """Wall-offset change between this frame's queueing and the STOP."""
        return self.start_rel_us - self.origin_rel_us


@dataclass(frozen=True)
class StopRecord:
    group: int
    seq: int
    overwritten: int
    entry_wall_us: int
    hand_c1: bool
    play_start_rel_us: int
    q_depth: int
    q_head_rel_us: Optional[int]
    ring: Tuple[FrameNote, ...]          # newest queued first
    next: Optional[FrameNote]
    tb_rel_us: Optional[int]
    tb_delta_us: Optional[int]

    @property
    def group_name(self) -> str:
        return GROUP_NAMES[self.group]


@dataclass(frozen=True)
class StopVerdict:
    verdicts: Tuple[str, ...]            # empty = none of these
    text: str

    @property
    def primary(self) -> str:
        return self.verdicts[0] if self.verdicts else NONE_OF_THESE


def stop_records(hb) -> Tuple[Optional[StopRecord], Optional[StopRecord]]:
    """The (legs, Hand) records carried by one ``HeartbeatT2J``; None = no
    STOP of that group since the bridge booted."""
    out = []
    for g in (0, 1):
        seq = int(hb.sr_seq[g])
        if seq == 0:
            out.append(None)
            continue
        flags = int(hb.sr_flags[g])
        n = max(0, min(RING, int(hb.sr_ring_n[g])))
        ring = tuple(
            FrameNote(int(hb.sr_ring_origin_rel_us[g * RING + j]),
                      int(hb.sr_ring_start_rel_us[g * RING + j]),
                      int(hb.sr_ring_arrive_rel_us[g * RING + j]))
            for j in range(n))
        nxt = (FrameNote(int(hb.sr_next_origin_rel_us[g]),
                         int(hb.sr_next_start_rel_us[g]),
                         int(hb.sr_next_arrive_rel_us[g]))
               if flags & _NEXT_VALID else None)
        depth = int(hb.sr_q_depth[g])
        head = int(hb.sr_q_head_rel_us[g])
        tb_ok = bool(flags & _TB_VALID)
        out.append(StopRecord(
            group=g, seq=seq, overwritten=int(hb.sr_overwritten[g]),
            entry_wall_us=int(hb.sr_entry_wall_us[g]),
            hand_c1=bool(flags & _HAND_C1),
            play_start_rel_us=int(hb.sr_play_start_rel_us[g]),
            q_depth=depth,
            q_head_rel_us=None if (depth == 0 or head == NO_QUEUE) else head,
            ring=ring, next=nxt,
            tb_rel_us=int(hb.sr_tb_rel_us[g]) if tb_ok else None,
            tb_delta_us=int(hb.sr_tb_delta_us[g]) if tb_ok else None))
    return out[0], out[1]


def _ms(us: int) -> str:
    return '%+.1f' % (us / 1000.0)


def classify_stop_record(rec: StopRecord) -> StopVerdict:
    """Say why the cover ended, with the numbers that decide it."""
    frames = list(rec.ring) + ([rec.next] if rec.next is not None else [])
    play = next((f for f in rec.ring
                 if f.start_rel_us == rec.play_start_rel_us), None)
    if play is not None:
        play_origin, play_shift, est = play.origin_rel_us, play.map_shift_us, ''
    else:
        # The playing frame fell out of the ring: assume the oldest entry's
        # mapping held for it too.
        play_shift = rec.ring[-1].map_shift_us if rec.ring else 0
        play_origin, est = rec.play_start_rel_us - play_shift, ' (est)'
    newer = [f for f in frames if f.origin_rel_us > play_origin]
    succ = min(newer, key=lambda f: f.origin_rel_us) if newer else None

    verdicts = []
    parts = ['playing t_start %s ms' % _ms(rec.play_start_rel_us)]
    if succ is not None:
        gap = succ.origin_rel_us - play_origin
        step = succ.map_shift_us - play_shift
        parts.append(
            'successor stamp %s ms after the playing one%s, t_start %s ms, '
            'arrived %s ms, map shift %s ms'
            % (_ms(gap), est, _ms(succ.start_rel_us),
               _ms(succ.arrive_rel_us), _ms(step)))
        if gap > STAMP_GAP_US:
            verdicts.append(STAMP_GAP)
        if abs(step) > MAP_STEP_US:
            verdicts.append(MAPPING_STEP)
    else:
        parts.append('no successor frame seen')
    if rec.tb_rel_us is not None:
        recent = (-TB_WINDOW_US <= rec.tb_rel_us <= 0
                  and abs(rec.tb_delta_us) > MAP_STEP_US)
        parts.append('last time-base change %s ms at %s ms%s'
                     % (_ms(rec.tb_delta_us), _ms(rec.tb_rel_us),
                        ' (inside 200 ms)' if recent else ''))
        if recent and MAPPING_STEP not in verdicts:
            verdicts.append(MAPPING_STEP)
    else:
        parts.append('no time-base change > 1 ms since boot')
    # A frame newer than the playing one, queued and due before the STOP, and
    # not so late that promotion would drop it as expired, should have taken
    # over: the STOP should not have fired.
    stuck = [f for f in frames
             if f.start_rel_us > rec.play_start_rel_us
             and f.start_rel_us <= 0 and f.arrive_rel_us <= 0
             and f.arrive_rel_us - f.start_rel_us < COVER_US]
    if stuck or (rec.q_head_rel_us is not None and rec.q_head_rel_us <= 0):
        verdicts.append(QUEUED_NOT_PROMOTED)
    parts.append('queue depth %d%s' % (
        rec.q_depth, '' if rec.q_head_rel_us is None
        else ', head t_start %s ms' % _ms(rec.q_head_rel_us)))
    parts.append('ring (origin/t_start/arrival ms) ' + (' '.join(
        '%s/%s/%s' % (_ms(f.origin_rel_us), _ms(f.start_rel_us),
                      _ms(f.arrive_rel_us)) for f in rec.ring) or 'empty'))
    if rec.next is None:
        parts.append('next frame not yet queued')
    verdict_txt = ' + '.join(verdicts) if verdicts else NONE_OF_THESE
    text = ('STOP record #%d %s (%s): %s; %s; overwritten %d'
            % (rec.seq, rec.group_name, 'Hand C1' if rec.hand_c1 else 'C2',
               verdict_txt, '; '.join(parts), rec.overwritten))
    return StopVerdict(tuple(verdicts), text)


class StopRecordLog:
    """One line per new record: when its next-frame part is filled, or
    ``wait_s`` after the record first appears, whichever is first."""

    def __init__(self, wait_s: float = 1.0) -> None:
        self._wait_s = float(wait_s)
        self._logged = [0, 0]
        self._first_seen = [None, None]   # (seq, t) of a record waiting for its next frame

    def update(self, hb, now: float):
        """Lines due at ``now`` (monotonic seconds) for this heartbeat."""
        lines = []
        for rec in stop_records(hb):
            if rec is None or rec.seq == self._logged[rec.group]:
                continue
            g = rec.group
            pend = self._first_seen[g]
            if pend is None or pend[0] != rec.seq:
                pend = self._first_seen[g] = (rec.seq, now)
            if rec.next is not None or now - pend[1] >= self._wait_s:
                lines.append(classify_stop_record(rec).text)
                self._logged[g] = rec.seq
                self._first_seen[g] = None
        return lines
