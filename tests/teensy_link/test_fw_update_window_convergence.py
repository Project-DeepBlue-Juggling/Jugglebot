"""Platform firmware-over-CAN DATA phase against a faithful FW 10 receiver and a
realistic relay, in virtual time (2026-11-19, logbook
2026-11-19-platform-fw-update-rewind-derail).

Three --verify-only runs of the FW 11 image (can-bridge FW 44 and FW 45) ended
the same way: an isolated lost frame, then two BAD_SEQ rewinds per 16-frame
window until the end, then ``VERIFY — BAD_SEQ``. The cause is host-side: the
board's replies reach the host late (the can-bridge uplinks 0x6F1 from an
8-slot ring, 4 frames per 10 ms tick), so a NAK burst's tail lands after the
host has started the next window, and the window-reply filter took a straggler
naming the window's FIRST frame as that window's verdict. VERIFY then read a
DATA-opcode straggler as its own answer (the receiver's VERIFY handler never
answers BAD_SEQ).

The models here are the parts of the chain that decide that:

* ``Fw10Receiver`` is ``FwUpdate::handleCommand`` of
  ``git show 1814578d:ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino``
  (the FW 10 receiver on the board): DATA strictly in order, ACK when
  ``seq % 16 == 15`` or the image completes, BAD_SEQ naming the expected seq for
  every out-of-order frame, a 4 KB sector flush that blacks the board out (frames
  arriving then are lost) and sends that frame's reply only after the flush,
  VERIFY = flush the tail, length check (BAD_STATE), CRC-32 (BAD_CRC),
  identity (BAD_IDENTITY), OK; COMMIT only from VERIFIED.
* ``Relay`` is the can-bridge: one DATA RPC per 1 ms tick, the 0x6F0 frame on
  the wire before the ack, and the reply uplink of ``can_buses.cpp``
  ``platform_ring_push`` (8 slots, 7 usable, drop newest) drained by
  ``telemetry.cpp`` ``platform_uplink_step`` (``PLATFORM_FWD_BUDGET`` 4) every
  10 ms.
* ``SimWaiter`` has ``PlatformFrameWaiter``'s semantics (a latest-reply slot for
  ``wait``, an arrival-order queue for ``take``, ``clear`` drops both).

A link policy injects loss, duplication or reordering of chosen DATA frames
between the relay and the board. The REAL tool code (``_run_fw_update_session``
and everything under it) runs against all of it with ``tlb.time`` replaced by
the virtual clock, so a 40 s transfer takes well under a second.
"""

from __future__ import annotations

import binascii
import heapq
import itertools
import struct
import types
from collections import deque

import pytest

from teensy_link import RpcMethod, rpc_args

OP_BEGIN, OP_DATA, OP_VERIFY, OP_COMMIT = 0x01, 0x02, 0x03, 0x04
ST_OK = rpc_args.PLATFORM_FW_STATUS_OK
ST_BAD_STATE = rpc_args.PLATFORM_FW_STATUS_BAD_STATE
ST_BAD_SEQ = rpc_args.PLATFORM_FW_STATUS_BAD_SEQ
ST_TOO_BIG = rpc_args.PLATFORM_FW_STATUS_TOO_BIG
ST_BAD_CRC = rpc_args.PLATFORM_FW_STATUS_BAD_CRC
ST_BAD_IDENTITY = rpc_args.PLATFORM_FW_STATUS_BAD_IDENTITY
SECTOR = 4096
REPLY_ID = 0x6F1

# Relay / board timing (seconds).
RPC_TICK = 0.001          # the bridge services one synchronous RPC per 1 kHz tick
WIRE = 0.0004             # relay TX -> board RX for a DATA frame
BOARD_TO_RING = 0.0003    # board reply TX -> bridge relay ring
UPLINK_TICK = 0.010       # task_telem drains the ring at 100 Hz ...
UPLINK_BUDGET = 4         # ... PLATFORM_FWD_BUDGET frames per tick
RING_USABLE = 7           # PLATFORM_RING_CAP 8, head/tail SPSC: one slot always empty
UDP = 0.0003
FLUSH_S = 0.052           # erase 45 ms typical + 16 page writes + read-back


class Clock:
    """Discrete-event virtual clock; ``monotonic``/``sleep`` stand in for ``time``."""

    def __init__(self):
        self.t = 1000.0
        self._q = []
        self._n = itertools.count()

    def at(self, t, fn):
        heapq.heappush(self._q, (t, next(self._n), fn))

    def after(self, dt, fn):
        self.at(self.t + dt, fn)

    def run_until(self, t, pred=None):
        """Run events up to ``t``; stop early (clock at that event) once ``pred()``."""
        while True:
            if pred is not None and pred():
                return True
            if not self._q or self._q[0][0] > t:
                self.t = max(self.t, t)
                return pred() if pred is not None else True
            te, _, fn = heapq.heappop(self._q)
            self.t = max(self.t, te)
            fn()

    def monotonic(self):
        return self.t

    def sleep(self, s):
        self.run_until(self.t + max(0.0, s))


class SimWaiter:
    """PlatformFrameWaiter semantics over the virtual clock."""

    def __init__(self, clock):
        self.clock = clock
        self.slot = {}
        self.queue = deque()

    def deliver(self, can_id, data):
        self.slot[can_id] = data
        self.queue.append(data)

    def clear(self, can_id):
        self.slot.pop(can_id, None)
        self.queue.clear()

    def wait(self, can_id, expected_dlc, timeout):
        self.clock.run_until(self.clock.t + timeout, pred=lambda: can_id in self.slot)
        return self.slot.get(can_id)

    def take(self, can_id, expected_dlc, timeout):
        self.clock.run_until(self.clock.t + timeout, pred=lambda: bool(self.queue))
        return self.queue.popleft() if self.queue else None


class Fw10Receiver:
    """FW 10 ``FwUpdate::handleCommand``, frame for frame (see the module docstring)."""

    IDENTITY = b"jugglebot-platform"

    def __init__(self, clock, send_reply, flush_s=FLUSH_S):
        self.clock, self.send_reply, self.flush_s = clock, send_reply, flush_s
        self.phase = "IDLE"
        self.image_len = 0
        self.staged = bytearray()
        self.flushed = 0
        self.expect = 0
        self.last_seq = 0
        self.busy_until = 0.0
        self.lost_in_flush = 0
        self.committed = False
        self.verified_crc = None
        self.replies = []            # (t, op, status, seq) — what the board said, in order

    def _reply(self, op, status, seq, detail):
        self.replies.append((self.clock.t, op, status, seq))
        self.send_reply(struct.pack('<BBHI', op, status, seq & 0xFFFF, detail & 0xFFFFFFFF))

    def rx(self, frame):
        if self.clock.t < self.busy_until:
            self.lost_in_flush += 1          # IRQs masked by the flash primitives: FIFO overrun
            return
        op = frame[0]
        if op == OP_BEGIN:
            (n,) = struct.unpack('<I', frame[1:5])
            self.image_len, self.staged, self.flushed = n, bytearray(), 0
            self.expect = self.last_seq = 0
            self.phase = "RECEIVING"
            self._reply(op, ST_OK, 0, 0)
        elif op == OP_DATA:
            if self.phase != "RECEIVING":
                self._reply(op, ST_BAD_STATE, self.expect, len(self.staged))
                return
            seq = struct.unpack('<H', frame[1:3])[0]
            payload = frame[3:]
            if seq != (self.expect & 0xFFFF):
                self._reply(op, ST_BAD_SEQ, self.expect, len(self.staged))
                return
            if len(self.staged) + len(payload) > self.image_len:
                self._reply(op, ST_TOO_BIG, seq, len(self.staged))
                return
            before = len(self.staged) // SECTOR
            self.staged += payload
            self.last_seq, self.expect = seq, self.expect + 1
            complete = len(self.staged) == self.image_len
            ack = (seq % 16) == 15 or complete
            if len(self.staged) // SECTOR > before:
                # flushBuffer() runs inside this handler: the reply waits for it.
                self.flushed = (len(self.staged) // SECTOR) * SECTOR
                self.busy_until = self.clock.t + self.flush_s
                if ack:
                    staged = len(self.staged)
                    self.clock.at(self.busy_until, lambda: self._reply(op, ST_OK, seq, staged))
                return
            if ack:
                self._reply(op, ST_OK, seq, len(self.staged))
        elif op == OP_VERIFY:
            if self.phase == "IDLE" or len(frame) < 5:
                self._reply(op, ST_BAD_STATE, self.last_seq, 0)
                return
            if len(self.staged) != self.image_len:
                self.phase = "RECEIVING"
                self._reply(op, ST_BAD_STATE, self.expect, len(self.staged))
                return
            (want,) = struct.unpack('<I', frame[1:5])
            got = binascii.crc32(bytes(self.staged)) & 0xFFFFFFFF
            if got != want:
                self.phase = "RECEIVING"
                self._reply(op, ST_BAD_CRC, self.last_seq, got)
                return
            if self.IDENTITY not in self.staged:
                self.phase = "RECEIVING"
                self._reply(op, ST_BAD_IDENTITY, self.last_seq, got)
                return
            self.phase, self.verified_crc = "VERIFIED", got
            self._reply(op, ST_OK, self.last_seq, got)
        elif op == OP_COMMIT:
            if self.phase != "VERIFIED":
                self._reply(op, ST_BAD_STATE, self.last_seq, 0)
                return
            self.committed = True
            self._reply(op, ST_OK, self.last_seq, 0)


class Relay:
    """The can-bridge between the host's RPCs and the board, plus the link policy."""

    def __init__(self, clock, waiter, policy=None, reply_policy=None):
        self.clock, self.waiter = clock, waiter
        self.policy = policy or (lambda seq, frame: [WIRE])
        # Board-side TX faults on the reply (FlexCAN_T4's duplicate-TX path, a
        # same-id mailbox reorder): extra delays per reply, [0.0] = as sent.
        self.reply_policy = reply_policy or (lambda data: [0.0])
        self.board = Fw10Receiver(clock, self._from_board)
        self.ring = deque()
        self.ring_drops = 0
        self.data_sent = 0
        self._schedule_uplink(self.clock.t + UPLINK_TICK * 0.37)

    # board -> host
    def _from_board(self, data):
        for d in self.reply_policy(data):
            self.clock.after(BOARD_TO_RING + d, lambda: self._ring_push(data))

    def _ring_push(self, data):
        if len(self.ring) >= RING_USABLE:
            self.ring_drops += 1                   # ring full -> drop newest
            return
        self.ring.append(data)

    def _schedule_uplink(self, t):
        def step():
            for _ in range(UPLINK_BUDGET):
                if not self.ring:
                    break
                data = self.ring.popleft()
                self.clock.after(UDP, lambda d=data: self.waiter.deliver(REPLY_ID, d))
            self._schedule_uplink(t + UPLINK_TICK)
        self.clock.at(t, step)

    # host -> board
    def call(self, method, args=b"", **kw):
        method = int(method)
        if method == int(RpcMethod.PLATFORM_FW_BEGIN):
            frame = bytes([OP_BEGIN]) + args[:4]
            delays = [WIRE]
        elif method == int(RpcMethod.PLATFORM_FW_DATA):
            seq, n = struct.unpack('<HB', args[:3])
            frame = bytes([OP_DATA]) + args[:2] + args[3:3 + n]
            self.data_sent += 1
            delays = self.policy(seq, frame)
        elif method == int(RpcMethod.PLATFORM_FW_VERIFY):
            frame = bytes([OP_VERIFY]) + args[:4]
            delays = [WIRE]
        elif method == int(RpcMethod.PLATFORM_FW_COMMIT):
            frame = bytes([OP_COMMIT])
            delays = [WIRE]
        else:
            raise AssertionError(f"unexpected RPC {method}")
        for d in delays:
            self.clock.after(d, lambda f=frame: self.board.rx(f))
        self.clock.sleep(RPC_TICK)                  # the bridge's sync ack, one tick later
        return b""


# ── link policies ────────────────────────────────────────────────────────────

def lose(*seqs):
    """Drop the FIRST transmission of each named seq (a frame lost on the wire)."""
    pending = set(seqs)

    def policy(seq, frame):
        if seq in pending:
            pending.discard(seq)
            return []
        return [WIRE]
    return policy


def duplicate(*seqs):
    """Deliver the first transmission of each named seq twice, back to back."""
    pending = set(seqs)

    def policy(seq, frame):
        if seq in pending:
            pending.discard(seq)
            return [WIRE, WIRE + 0.0001]
        return [WIRE]
    return policy


def overtake(*seqs):
    """The first transmission of each named seq arrives AFTER the next frame (the
    same-id mailbox reorder on the relay's CAN TX)."""
    pending = set(seqs)

    def policy(seq, frame):
        if seq in pending:
            pending.discard(seq)
            return [WIRE + 1.5 * RPC_TICK]
        return [WIRE]
    return policy


def combine(*policies):
    def policy(seq, frame):
        out = None
        for pol in policies:
            d = pol(seq, frame)
            if d != [WIRE]:
                out = d
        return out if out is not None else [WIRE]
    return policy


# ── the session runner ───────────────────────────────────────────────────────

def _image(n_bytes):
    body = bytearray((i * 131 + 7) & 0xFF for i in range(n_bytes))
    body[100:118] = Fw10Receiver.IDENTITY
    return bytes(body)


def every_nak_twice(data):
    """Each BAD_SEQ reply reaches the bus twice (the board's duplicate-TX defect)."""
    return [0.0, 0.0002] if data[1] == ST_BAD_SEQ else [0.0]


def nak_overtaken(nth):
    """The ``nth`` BAD_SEQ reply leaves the board 3 ms late, behind its successors."""
    count = [0]

    def policy(data):
        if data[1] == ST_BAD_SEQ:
            count[0] += 1
            if count[0] == nth:
                return [0.003]
        return [0.0]
    return policy


def run_session(monkeypatch, n_bytes, policy=None, commit=False, sector_pause_s=None,
                reply_policy=None):
    import dataclasses
    import tools.teensy_link_bridge as tlb
    clock = Clock()
    monkeypatch.setattr(tlb, "time", types.SimpleNamespace(monotonic=clock.monotonic,
                                                           sleep=clock.sleep))
    waiter = SimWaiter(clock)
    relay = Relay(clock, waiter, policy, reply_policy)
    target = tlb._FW_TARGETS["platform"]
    if sector_pause_s is not None:
        target = dataclasses.replace(target, sector_pause_s=sector_pause_s)
    image = _image(n_bytes)
    crc = binascii.crc32(image) & 0xFFFFFFFF
    t0 = clock.t
    rc = tlb._run_fw_update_session(relay, waiter, REPLY_ID, image, crc,
                                    commit=commit, target=target)
    return types.SimpleNamespace(rc=rc, relay=relay, board=relay.board, image=image, crc=crc,
                                 frames=(n_bytes + 4) // 5, elapsed=clock.t - t0)


def _assert_converged(s, faults, *, slack_per_fault=16):
    """VERIFY OK on a board that holds exactly the image, and at most one window
    resent per injected fault (before the fix: two windows per window, to the end)."""
    assert s.rc == 0, f"session failed; board replies tail: {s.board.replies[-6:]}"
    assert bytes(s.board.staged) == s.image
    assert s.board.phase == "VERIFIED" and s.board.verified_crc == s.crc
    assert s.relay.data_sent <= s.frames + slack_per_fault * max(1, faults), (
        f"{s.relay.data_sent} DATA frames for a {s.frames}-frame image and {faults} fault(s)")


# ── scenarios ────────────────────────────────────────────────────────────────

IMG = 3 * SECTOR + 1700          # 3 sector flushes, 2798 frames, a partial last window
FRAMES = (IMG + 4) // 5


def test_clean_transfer_needs_no_resend(monkeypatch):
    s = run_session(monkeypatch, IMG)
    _assert_converged(s, 0)
    assert s.relay.data_sent == FRAMES


@pytest.mark.parametrize("phase", range(16))
def test_one_lost_frame_at_every_phase_of_a_window(monkeypatch, phase):
    """A frame lost at window position ``phase`` (window [1600, 1616) has no
    sector flush) costs one rewind, never a derail."""
    s = run_session(monkeypatch, IMG, lose(1600 + phase))
    _assert_converged(s, 1)


@pytest.mark.parametrize("seq", [812, 816, 818, 819, 820, 822, 1632, 1637, 1638, 1639, 1640])
def test_loss_around_a_sector_pause(monkeypatch, seq):
    """Frames 819 and 1638 fill sectors 1 and 2 (the host pauses 0.5 s after each).
    A loss just before, at, or after the pause: the NAK burst's tail arrives
    during the pause, before the window's own reply."""
    s = run_session(monkeypatch, IMG, lose(seq))
    _assert_converged(s, 1)


@pytest.mark.parametrize("seq", [1600, 1607, 1615, 2455])
def test_duplicated_frame(monkeypatch, seq):
    s = run_session(monkeypatch, IMG, duplicate(seq))
    _assert_converged(s, 1)


@pytest.mark.parametrize("seq", [1600, 1607, 1614, 1615])
def test_overtaken_frame(monkeypatch, seq):
    s = run_session(monkeypatch, IMG, overtake(seq))
    _assert_converged(s, 1)


@pytest.mark.parametrize("seq", [1603, 1612, 1637])
def test_duplicated_naks(monkeypatch, seq):
    """Every NAK of the burst arrives twice: more stragglers, same verdict."""
    s = run_session(monkeypatch, IMG, lose(seq), reply_policy=every_nak_twice)
    _assert_converged(s, 1)


@pytest.mark.parametrize("nth", [1, 2, 5])
def test_reordered_naks(monkeypatch, nth):
    """One NAK of a burst overtaken by its successors on the way back."""
    s = run_session(monkeypatch, IMG, lose(1603), reply_policy=nak_overtaken(nth))
    _assert_converged(s, 1)


def test_naks_straggling_into_verify_are_not_its_verdict(monkeypatch):
    """A loss in the image's last window: its NAK burst is still draining from
    the relay ring when VERIFY is sent. VERIFY must report the board's own
    VERIFY answer (op 0x03), not a DATA straggler (the receiver's VERIFY handler
    has no BAD_SEQ path at all)."""
    last_window_start = (FRAMES - 1) // 16 * 16
    s = run_session(monkeypatch, IMG, lose(last_window_start + 1))
    _assert_converged(s, 1)


def test_many_isolated_losses_converge(monkeypatch):
    """One loss roughly every 1.5 s of transfer, at arbitrary window phases."""
    seqs = [205, 445, 1283, 1534, 2455, 2601, 2700]
    s = run_session(monkeypatch, IMG, lose(*seqs))
    _assert_converged(s, len(seqs))


def test_the_verify3_pattern_on_the_full_image(monkeypatch):
    """verify3.log, 2026-11-19 (bridge FW 45, the 88064-byte FW 11 image): the
    isolated rewinds it logged outside the doubled runs, injected as losses.
    Before the fix this session derails at 2455 (two rewinds per window from
    there on) and ends ``VERIFY — BAD_SEQ``."""
    seqs = [445, 1534, 2455, 9128, 13941, 16071, 17340, 17561]
    s = run_session(monkeypatch, 88064, lose(*seqs))
    _assert_converged(s, len(seqs))
    assert s.elapsed < 40.0          # 21 sector pauses = 10.5 s of the 0.5 s profile


def test_the_fw44_reorder_pattern_on_the_full_image(monkeypatch):
    """verify_fw44.log, 2026-11-19 (bridge FW 44): its 'N, N+1, window end'
    rewind triples (3901/3902/3904, 9461/9462/9472, ...) are the signature of a
    frame overtaken by its successor, and its 'N, window end' pairs that of a
    duplicate; injected as such, with losses at its plain onsets."""
    policy = combine(overtake(3901, 5076, 6977, 9461, 10994, 14410),
                     duplicate(3429, 16070), lose(205, 1283, 9874, 13163))
    s = run_session(monkeypatch, 88064, policy)
    _assert_converged(s, 12)


def test_commit_only_after_the_board_verified(monkeypatch):
    """COMMIT is unchanged: sent once, after VERIFY OK from the board's VERIFY
    handler, and the board accepts it only from VERIFIED."""
    s = run_session(monkeypatch, IMG, lose(1601, 1637, 1640), commit=True)
    assert s.rc == 0 and s.board.committed
    ops = [op for _, op, st, _ in s.board.replies if op in (OP_VERIFY, OP_COMMIT)]
    assert ops == [OP_VERIFY, OP_COMMIT]


def test_a_corrupted_image_never_commits(monkeypatch):
    """A payload flipped in transit is caught by the board's CRC: BAD_CRC, no COMMIT."""
    s_policy_hits = {"done": False}

    def policy(seq, frame):
        if seq == 900 and not s_policy_hits["done"]:
            s_policy_hits["done"] = True
            bad = bytearray(frame)
            bad[3] ^= 0x01
            relay_ref[0].clock.after(WIRE, lambda: relay_ref[0].board.rx(bytes(bad)))
            return []
        return [WIRE]

    relay_ref = [None]
    import tools.teensy_link_bridge as tlb
    clock = Clock()
    monkeypatch.setattr(tlb, "time", types.SimpleNamespace(monotonic=clock.monotonic,
                                                           sleep=clock.sleep))
    waiter = SimWaiter(clock)
    relay = Relay(clock, waiter, policy)
    relay_ref[0] = relay
    image = _image(IMG)
    crc = binascii.crc32(image) & 0xFFFFFFFF
    rc = tlb._run_fw_update_session(relay, waiter, REPLY_ID, image, crc, commit=True,
                                    target=tlb._FW_TARGETS["platform"])
    assert rc == 1 and not relay.board.committed
    assert relay.board.phase == "RECEIVING"
    assert [st for _, op, st, _ in relay.board.replies if op == OP_VERIFY] == [ST_BAD_CRC]
