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
                wall0_us=1_700_000_000_000_000, local0=4_000_000_000, disc_at=None, seed=1, fw_version=11,
                odr_hz=1000):
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
                     wall0_us + 5, local0 & 0xFFFFFFFF, odr_hz, 2048, 328, 49, 6, 38, 4, -321, 4321, 3, 5, 700, fw_version,
                     payload_crc, 0)
    struct.pack_into("<I", hdr, 62, zlib.crc32(bytes(hdr[:62])) & 0xFFFFFFFF)
    for i in range(11):
        words[pc.PLATFORM_IMU_HEADER_SEQ_BASE + i] = bytes(hdr[6 * i:6 * i + 6])
    table = {}
    if fw_version >= 14:
        # FW 14 chunk-CRC table (spec section 3): entry c < SAMPLE_CHUNKS covers sample words [512c, 512c + 512),
        # the next ANCHOR_CHUNKS the anchor words, each ending early at its region's end; READY set.
        cw = pc.PLATFORM_IMU_CHUNK_CRC_WORDS
        for c in range(pi.CHUNK_TABLE_WORDS):
            data = sample_bytes if c < pi.SAMPLE_CHUNKS else anc_bytes
            start = (c if c < pi.SAMPLE_CHUNKS else c - pi.SAMPLE_CHUNKS) * cw * 6
            chunk = data[start:start + cw * 6]
            table[c] = (zlib.crc32(chunk) & 0xFFFFFFFF, len(chunk) // 6)
            words[pi.CHUNK_BASE + c] = struct.pack("<IH", table[c][0], table[c][1] | pi.CHUNK_READY)
    return dict(table=table, words=words, header=bytes(hdr), samples=samples, anchors=anchors, n=n, cap_id=cap_id,
                boot_id=boot_id, first_index=first_index, payload_crc=payload_crc, odr_us=odr_us, local0=local0,
                wall0_us=wall0_us, anc_bytes=anc_bytes)


class FakePlatform:
    """Drives an UploadMachine with fake time. Options (all default off):

    drop_once      set of seqs dropped the FIRST time they are sent (hole, refilled by a re-read)
    drop_always    set of seqs never delivered (a word the Platform repeatedly fails to send)
    loss_p         every frame sent is omitted independently with this probability (the Armed omission
                   model of the 2026-11-20 debrief); ``loss_seed`` fixes the draw
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
    corrupt_first  {seq: xor_byte}: the FIRST delivery of that seq carries a flipped byte (with drop_once on the
                   same seq: the refill copy is the corrupted one), later deliveries are clean
    aux_status     {region base: status}: FW 14 READs starting in that region (DIAG_BASE, CHUNK_BASE) are
                   refused with this status (a board that does not have it)
    chunk_not_ready_reads  the first N READs of the chunk table answer every entry not READY (lazy board)

    FW 14 regions (image built with fw_version >= 14): the chunk table is in the image; the diag words are a
    snapshot taken when a READ of DIAG_BASE is accepted (data_placed = frames sent so far, data_tx_done =
    frames sent and not omitted by the loss model).

    board          12 or 13: model the Platform's ONE sender instead of the per-READ legacy schedule. A READ
                   reaches the board ``cmd_lat`` after it is sent; the board sends one frame per gap (a fresh
                   block's first frame no sooner than one gap after the last frame sent) and each frame reaches
                   the host ``rx_lat`` later. A READ that arrives while a block is active REPLACES it (the rest
                   of that block is never sent), except on board 13 when its first_seq is the block's end: then
                   it EXTENDS the block and is answered READ_EXTENDED (spec section 4, READ chaining).
    cmd_lat, rx_lat  board-model latencies (default 4 ms and 10 ms: about the measured 16 ms READ turnaround)
    lose_read_replies  set of READ indices (0-based, in send order) whose IMU_REPLY the bridge loses
    lose_reads     set of READ indices that never reach the board (no reply, no frames)
    reply_lat      extra seconds a READ's IMU_REPLY takes to reach the host (default 0): above the block's frame
                   latency the reply arrives AFTER the block's first frames
    cmd_jitter     extra seconds, uniform in [0, cmd_jitter] (its own generator, seeded from ``loss_seed``), added to each READ's
                   ``cmd_lat``; READs still reach the board in send order (the bridge's CAN TX is one FIFO)
    block_delay    seconds of pacing delay each fresh block's frames run late by, beyond the schedule: models the
                   delay the real board's 2-gap catch-up does not recover (loop stalls, tx_deferred under bus load)
                   plus the bridge's CAN TX queue wait. A READ arriving before the delayed block's last frame
                   replaces it (counted in ``replaced``)
    batched        deliver frames like the real bridge: collected and handed to on_records in batches of up to
                   24 frames every 10 ms (frames reach the host no sooner than their rx time)
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
        self.loss_rng = np.random.default_rng(opt.get("loss_seed", 0))
        self._jit_rng = np.random.default_rng(opt.get("loss_seed", 0) + 1)
        self.omitted = 0
        machine._is_armed = lambda: bool(opt.get("armed", False))
        self.board = opt.get("board")
        self.bk = None                  # board model: the active block {next, end, gap, gen}
        self.last_tx = -1.0
        self.replaced = 0               # board model: READs that cut an active block (FW 12 behaviour)
        self.extended = 0
        self._cmd_arr = -1.0             # board model: when the last READ reaches the board (FIFO order)
        self.frames_sent = 0
        self._batch = []
        self._batch_next = 0.0
        self.corrupted = set()
        self.chunk_reads = 0
        self.diag_reads = 0

    def _aux(self, first):
        """FW 14 auxiliary READ: None to serve it, else the refusal status. Takes the diag snapshot."""
        for base, n in ((pi.DIAG_BASE, pi.DIAG_WORDS), (pi.CHUNK_BASE, pi.CHUNK_TABLE_WORDS)):
            if base <= first < base + n:
                st = self.opt.get("aux_status", {}).get(base)
                if st is not None:
                    return st
                if not self.img["table"]:
                    return ST.ERR_OUT_OF_RANGE                     # an FW 13 image has neither region
                if base == pi.DIAG_BASE:
                    self.diag_reads += 1
                    blob = struct.pack("<BBIIHHHHHBBHHI", 1, 1, self.frames_sent, self.frames_sent - self.omitted,
                                       0, 0, 900, 0, 0, 0, 0, 0, 0, 0)
                    for i in range(pi.DIAG_WORDS):
                        self.img["words"][base + i] = blob[6 * i:6 * i + 6]
                else:
                    self.chunk_reads += 1
                    ready = self.chunk_reads > self.opt.get("chunk_not_ready_reads", 0)
                    for c, (crc, n) in self.img["table"].items():
                        self.img["words"][base + c] = struct.pack(
                            "<IH", crc if ready else 0, n | (pi.CHUNK_READY if ready else 0))
                return None
        return None

    def _word(self, seq):
        word = self.img["words"][seq]
        cf = self.opt.get("corrupt_first", {})
        if seq in cf and seq not in self.corrupted:
            self.corrupted.add(seq)
            word = bytes([word[0] ^ cf[seq]]) + word[1:]
        return word

    def _at(self, dt, fn):
        self._at_abs(self.t + dt, fn)

    def _at_abs(self, t_abs, fn):
        # The board model schedules on absolute due times, so its frame pace is not quantised to the 1 ms tick.
        self.n_ev += 1
        heapq.heappush(self.q, (t_abs, self.n_ev, fn))

    def _reply(self, op, status, cid, detail):
        if self.lose_replies > 0:
            self.lose_replies -= 1
            return
        data = struct.pack("<BBHI", op, status, cid, detail)
        self._at(0.002 + self.opt.get("reply_lat", 0.0), lambda: self.m.on_reply(pi.decode_imu_reply(data), self.t))

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
                self.bk = None
                self._reply(op, ST.OK, cid, 0)
        elif op == OP.ABORT:
            self.block_gen += 1
            self.bk = None
            self._reply(op, ST.OK, img["cap_id"], 0)
        elif op == OP.SET_ODR:
            # Relayed from can-bridge FW 47 (byte 1 = odr_khz). Platform FW 13 rules: a value other than
            # 1 or 2 is ERR_BAD_ARGS; FROZEN or UPLOADING (this model's `frozen`) is ERR_FROZEN, nothing
            # changes and the ring survives; otherwise OK with the selected rate in Hz as the detail.
            if reason not in (1, 2):
                self._reply(op, ST.ERR_BAD_ARGS, 0, 0)
            elif self.frozen:
                self._reply(op, ST.ERR_FROZEN, img["cap_id"], 0)
            else:
                self._reply(op, ST.OK, 0, 1000 * reason)
        return True, "ok"

    def send_read(self, cid, first, count, gap):
        if self.board:
            return self._board_send_read(cid, first, count, gap)
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
        aux = self._aux(first)
        if aux is not None:
            self._reply(OP.READ, aux, img["cap_id"], first)
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
            if seq in self.opt.get("drop_always", ()):
                continue
            if self.opt.get("loss_p") and self.loss_rng.random() < self.opt["loss_p"]:
                self.omitted += 1
                continue
            word = self._word(seq)
            cor = self.opt.get("corrupt", {})
            if seq in cor and self.corrupt_active:
                word = bytes([word[0] ^ cor[seq]]) + word[1:]
            delay = dt + self.opt.get("late", {}).get(seq, 0.0)
            self._deliver(gen, delay, seq, word)
            if seq in self.opt.get("dup", ()):
                self._deliver(gen, delay + 0.001, seq, word)
        return True, "ok"

    # -- the board model (opt board=12|13) --
    def _board_send_read(self, cid, first, count, gap):
        used = max(max(gap, 63) if self.opt.get("armed") else gap, 40)
        idx = len(self.reads)
        self.reads.append((first, count, gap, used))
        if idx in self.opt.get("lose_reads", ()):
            return True, "ok"
        lat = self.opt.get("cmd_lat", 0.004)
        if self.opt.get("cmd_jitter"):
            lat += self._jit_rng.uniform(0.0, self.opt["cmd_jitter"])
        t_arr = max(self.t + lat, self._cmd_arr + 1e-6)     # FIFO: never overtakes the READ before it
        self._cmd_arr = t_arr
        self._at_abs(t_arr, lambda: self._board_read(idx, cid, first, count, used))
        return True, "ok"

    def _board_reply(self, idx, op, status, cid, detail):
        if idx in self.opt.get("lose_read_replies", ()):
            return
        self._reply(op, status, cid, detail)

    def _board_read(self, idx, cid, first, count, used):
        img = self.img
        if self.rebooted or not self.frozen:
            self._board_reply(idx, OP.READ, ST.ERR_NOT_FROZEN, 0, 0)
            return
        if cid != img["cap_id"]:
            self._board_reply(idx, OP.READ, ST.ERR_WRONG_CAPTURE, img["cap_id"], 0)
            return
        if self.opt.get("read_status") is not None:
            self._board_reply(idx, OP.READ, self.opt["read_status"], img["cap_id"], 0)
            return
        aux = self._aux(first)
        if aux is not None:
            self._board_reply(idx, OP.READ, aux, img["cap_id"], first)
            return
        detail = first | (count << 16)
        bk = self.bk
        if bk is not None and self.board >= 13 and first == bk["end"]:
            bk["end"] += count
            bk["gap"] = used
            self.extended += 1
            self._board_reply(idx, OP.READ, ST.READ_EXTENDED, cid, detail)
            return
        if bk is not None:
            self.replaced += 1
        self.block_gen += 1
        self.bk = {"next": first, "end": first + count, "gap": used, "gen": self.block_gen}
        self._board_reply(idx, OP.READ, ST.OK, cid, detail)
        t0 = max(self.t, self.last_tx + used * 10e-6) + self.opt.get("block_delay", 0.0)
        self._at_abs(t0, lambda g=self.block_gen, d=t0: self._board_tx(g, d))

    def _board_tx(self, gen, due):
        bk = self.bk
        if bk is None or bk["gen"] != gen or self.rebooted or not self.frozen:
            return
        seq = bk["next"]
        bk["next"] += 1
        self.last_tx = due
        self.frames_sent += 1
        self._emit(seq, due + self.opt.get("rx_lat", 0.010))
        if bk["next"] >= bk["end"]:
            self.bk = None
        else:
            nxt = due + bk["gap"] * 10e-6
            self._at_abs(nxt, lambda: self._board_tx(gen, nxt))

    def _emit(self, seq, t_abs):
        if seq in self.opt.get("drop_once", ()) and seq not in self.sent_once:
            self.sent_once.add(seq)
            return
        if seq in self.opt.get("drop_always", ()):
            return
        if self.opt.get("loss_p") and self.loss_rng.random() < self.opt["loss_p"]:
            self.omitted += 1
            return
        word = self._word(seq)
        cor = self.opt.get("corrupt", {})
        if seq in cor:
            word = bytes([word[0] ^ cor[seq]]) + word[1:]
        rec = struct.pack("<H", seq) + word
        t_abs += self.opt.get("late", {}).get(seq, 0.0)
        if self.opt.get("batched"):
            self._at_abs(t_abs, lambda: self._batch_in(rec))
        else:
            self._at_abs(t_abs, lambda: self.m.on_records(rec, self.t))

    def _batch_in(self, rec):
        self._batch.append(rec)
        if len(self._batch) >= 24:
            self._flush_batch()
        elif len(self._batch) == 1:
            self._batch_next = self.t + 0.010
            self._at_abs(self._batch_next, self._flush_batch)

    def _flush_batch(self):
        if self._batch:
            recs, self._batch = b"".join(self._batch), []
            self.m.on_records(recs, self.t)

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
