"""A fake Platform Teensy + can-bridge for the IMU upload tests, built from
ros_ws/docs/platform_imu_protocol.md (not from the uploader). Time is a number the test advances.

Not collected by pytest (no ``test_`` prefix); imported by name from the test files in this directory.
"""

from __future__ import annotations

import heapq
import struct
import zlib

import numpy as np

import protocol_config as pc
from teensy_link import platform_imu as pi

OP, ST = pc.PlatformImuOp, pc.PlatformImuStatus


def build_image(*, n=2000, n_anchors=20, cap_id=0x4A21, boot_id=0x11223344, first_index=500, odr_us=1010.0,
                wall0_us=1_700_000_000_000_000, local0=4_000_000_000, disc_at=None, seed=1):
    """A frozen-capture image: {seq: 6-byte word}, the header bytes, and the truth to compare against.

    The chip clock runs ``odr_us`` us per sample (1 % slow rate error by default); anchors sit every
    ``n // n_anchors`` samples with positive drain-jitter. ``disc_at`` (anchor index) inserts a
    closing + DISCONTINUITY pair there and a sample gap of 40 us-ticks."""
    rng = np.random.default_rng(seed)
    samples = rng.integers(-30000, 30000, size=(n, 6), dtype=np.int16)
    words = {}
    for k in range(n):
        words[2 * k] = samples[k, :3].astype("<i2").tobytes()
        words[2 * k + 1] = samples[k, 3:].astype("<i2").tobytes()
    step = n // n_anchors
    anchors = []
    t_true = lambda k: local0 + odr_us * k        # noqa: E731  local_us of sample k (true)
    for a in range(n_anchors):
        k_last = (a + 1) * step - 1
        jitter = float(rng.integers(0, 40))
        local = int(t_true(k_last) + jitter) & 0xFFFFFFFF
        flags = 0
        anchors.append([first_index + k_last + 1, local, 0, flags])
    if disc_at is not None:
        # insert a discontinuity: samples after the closing anchor are time-shifted by 50 ms
        for a in range(disc_at, len(anchors)):
            anchors[a][1] = (anchors[a][1] + 50_000) & 0xFFFFFFFF
        anchors[disc_at][3] |= 0x01
    for a in anchors:
        # wall = local + a constant offset (+ tiny drift): the synced clock
        a[2] = wall0_us + (a[1] if a[1] >= local0 % (1 << 32) else a[1] + (1 << 32)) - local0 + 0
    anc_bytes = b"".join(struct.pack("<IIQBx", *a) for a in anchors)
    for i in range(0, len(anc_bytes), 6):
        words[pc.PLATFORM_IMU_ANCHOR_SEQ_BASE + i // 6] = anc_bytes[i:i + 6]
    sample_bytes = b"".join(words[s] for s in range(2 * n))
    payload_crc = zlib.crc32(sample_bytes, zlib.crc32(anc_bytes)) & 0xFFFFFFFF
    hdr = bytearray(66)
    struct.pack_into("<BBHIIIHHQIHHHBBBBhhHHHHII", hdr, 0, 1, 1, cap_id, boot_id, n, first_index, n_anchors, 0x01,
                     wall0_us + 5, local0 & 0xFFFFFFFF, 1000, 2048, 328, 49, 6, 38, 4, -321, 4321, 3, 5, 700, 11,
                     payload_crc, 0)
    struct.pack_into("<I", hdr, 62, zlib.crc32(bytes(hdr[:62])) & 0xFFFFFFFF)
    for i in range(11):
        words[pc.PLATFORM_IMU_HEADER_SEQ_BASE + i] = bytes(hdr[6 * i:6 * i + 6])
    return dict(words=words, header=bytes(hdr), samples=samples, anchors=anchors, n=n, cap_id=cap_id,
                boot_id=boot_id, first_index=first_index, payload_crc=payload_crc, odr_us=odr_us, local0=local0,
                wall0_us=wall0_us, anc_bytes=anc_bytes)


class FakePlatform:
    """Drives an UploadMachine with fake time. Options (all default off):

    drop_once      set of seqs dropped the FIRST time they are sent (hole, refilled by a re-read)
    drop_block_n   drop every frame of the first N READ blocks (a lost uplink burst)
    dup            set of seqs sent twice
    late           {seq: extra_delay_s}: frame delivered after the block's last frame (reordering)
    corrupt        {seq: xor_byte}: a payload byte flipped on every send; corrupt_once = first read only
    refuse_freeze  status code to answer FREEZE with
    already_frozen FREEZE answers ALREADY_FROZEN with this image's id
    reboot_at_s    sim time at which the board "reboots": STATUS boot changes, READs get ERR_NOT_FROZEN
    armed          bridge Armed: gaps clamped to 63
    lose_replies   number of leading IMU_REPLY frames the bridge loses
    read_status    status code to answer every READ with (a refusal; the board stays frozen)
    resume_ignored RESUME is neither applied nor answered (a lost command: the ring stays frozen)
    freeze_reply_lost  FREEZE is applied but its reply never arrives (the id stays unknown to the host)
    """

    def __init__(self, machine, image, **opt):
        self.m, self.img = machine, image
        self.opt = opt
        self.t = 0.0
        self.q = []
        self.n_ev = 0
        self.reads = []                 # (first, count, gap requested, gap used)
        self.ctrls = []
        self.sent_once = set()
        self.block_gen = 0
        self.frozen = False
        self.rebooted = False
        self.resumed = False
        self.reads_seen = 0
        self.lose_replies = opt.get("lose_replies", 0)
        self.corrupt_active = True
        machine._is_armed = lambda: bool(opt.get("armed", False))

    def _at(self, dt, fn):
        self.n_ev += 1
        heapq.heappush(self.q, (self.t + dt, self.n_ev, fn))

    def _reply(self, op, status, cid, detail):
        if self.lose_replies > 0:
            self.lose_replies -= 1
            return
        data = struct.pack("<BBHI", op, status, cid, detail)
        self._at(0.002, lambda: self.m.on_reply(pi.decode_imu_reply(data), self.t))

    # -- the bridge RPC surface the machine calls --
    def send_ctrl(self, op, reason, cid):
        self.ctrls.append((op, reason, cid))
        img = self.img
        if op == OP.FREEZE:
            if self.opt.get("freeze_reply_lost"):
                self.frozen = True
                return True, "ok"
            if self.opt.get("refuse_freeze") is not None:
                self._reply(op, self.opt["refuse_freeze"], 0, 0)
            elif self.opt.get("already_frozen") and not self.frozen:
                self.frozen = True
                self._reply(op, ST.ALREADY_FROZEN, img["cap_id"], 0)
            elif self.frozen:
                self._reply(op, ST.ALREADY_FROZEN, img["cap_id"], 0)
            else:
                self.frozen = True
                self._reply(op, ST.OK, img["cap_id"], img["n"])
        elif op == OP.RESUME:
            if self.opt.get("resume_ignored"):
                return True, "ok"
            if self.rebooted or cid != img["cap_id"] or not self.frozen:
                self._reply(op, ST.ERR_NOT_FROZEN, 0, 0)
            else:
                self.frozen = False
                self.resumed = True
                self._reply(op, ST.OK, cid, 0)
        elif op == OP.ABORT:
            self.block_gen += 1
            self._reply(op, ST.OK, img["cap_id"], 0)
        return True, "ok"

    def send_read(self, cid, first, count, gap):
        used = max(gap, 63) if self.opt.get("armed") else gap
        used = max(used, 40)
        if self.opt.get("corrupt_once") and getattr(self.m, "stats", {}).get("crc_rereads", 0) >= 1:
            self.corrupt_active = False       # the corruption hit the first pass only
        self.reads.append((first, count, gap, used))
        self.block_gen += 1
        gen = self.block_gen
        img = self.img
        self.reads_seen += 1
        if self.rebooted or not self.frozen:
            self._reply(OP.READ, ST.ERR_NOT_FROZEN, 0, 0)
            return True, "ok"
        if cid != img["cap_id"]:
            self._reply(OP.READ, ST.ERR_WRONG_CAPTURE, img["cap_id"], 0)
            return True, "ok"
        if self.opt.get("read_status") is not None:
            self._reply(OP.READ, self.opt["read_status"], img["cap_id"], 0)
            return True, "ok"
        self._reply(OP.READ, ST.OK, cid, first | (count << 16))
        drop_block = self.reads_seen <= self.opt.get("drop_block_n", 0)
        t_last = 0.0
        for i in range(count):
            seq = first + i
            dt = 0.004 + i * used * 10e-6
            t_last = dt
            if drop_block:
                continue
            if seq in self.opt.get("drop_once", ()) and seq not in self.sent_once:
                self.sent_once.add(seq)
                continue
            word = img["words"][seq]
            cor = self.opt.get("corrupt", {})
            if seq in cor and self.corrupt_active:
                word = bytes([word[0] ^ cor[seq]]) + word[1:]
            delay = dt + self.opt.get("late", {}).get(seq, 0.0)
            self._deliver(gen, delay, seq, word)
            if seq in self.opt.get("dup", ()):
                self._deliver(gen, delay + 0.001, seq, word)
        return True, "ok"

    def _deliver(self, gen, dt, seq, word):
        rec = struct.pack("<H", seq) + word

        def go():
            if gen == self.block_gen or True:   # late tails of a replaced block still arrive (spec section 3)
                self.m.on_records(rec, self.t)
        self._at(dt, go)

    def status(self, state=pc.PlatformImuState.FROZEN, boot=None):
        b = (self.img["boot_id"] & 0xFF) if boot is None else boot
        data = struct.pack("<BBbBBBBB", state, 0x03, 30, 0, 0, 20, 30, b)
        self.m.on_status(pi.decode_imu_status(data), self.t)

    def run(self, until_done=True, limit_s=120.0, dt=0.001, status_period=1.0):
        next_status = 1.0
        while self.t < limit_s:
            self.t = round(self.t + dt, 6)
            while self.q and self.q[0][0] <= self.t:
                heapq.heappop(self.q)[2]()
            reboot = self.opt.get("reboot_at_s")
            if reboot is not None and self.t >= reboot and not self.rebooted:
                self.rebooted = True
            if self.t >= next_status:
                next_status += status_period
                if self.rebooted:
                    self.status(state=pc.PlatformImuState.PROBING, boot=(self.img["boot_id"] + 1) & 0xFF)
                elif self.frozen:
                    self.status()
                else:
                    self.status(state=pc.PlatformImuState.RUNNING)
            self.m.tick(self.t)
            if until_done and self.m.phase == pi.DONE:
                break
        return self.t
