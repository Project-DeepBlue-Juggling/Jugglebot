#!/usr/bin/env python3
"""Losslessly recompress rosbag2 MCAP files in place: uncompressed chunks -> zstd chunks.

Foxy's ``ros2 bag record -s mcap`` writes MCAP files whose ~1 MiB Chunk records are
uncompressed. Chunk-level zstd-3 shrinks them ~3-4x. The python ``mcap`` Writer is far too
slow (~1 MB/s) to rewrite tens of GB, so this tool transcodes at the record level:

* Every record outside a Chunk is copied byte-for-byte, and the records INSIDE each Chunk
  (Schema/Channel/Message) are copied byte-for-byte too: only the Chunk's ``records`` field
  is compressed, its ``compression`` becomes ``"zstd"`` and its ``uncompressed_crc`` is
  computed (Foxy's libmcap 0.7.0 writes 0 = "not computed"; a non-zero input CRC is checked
  against the payload first, and a mismatch refuses the file).
* Offsets that move because Chunks shrink are rewritten: ChunkIndex (chunk_start_offset,
  chunk_length, message_index_offsets, compression, compressed_size), AttachmentIndex.offset,
  MetadataIndex.offset, SummaryOffset (group_start, group_length) and the Footer
  (summary_start, summary_offset_start, summary_crc). Every remapped offset must land
  exactly on a record boundary seen in the input, or the file is refused. MessageIndex
  offsets point inside the UNCOMPRESSED chunk and are unchanged.
* DataEnd.data_section_crc and Footer.summary_crc are always computed for the output
  (both are optional in the spec; computing them gives the verifier an end-to-end check
  of every byte written).

Nothing is replaced until the output is verified through the installed ``mcap`` library,
independently of the transcoder's own parsing (see ``verify``). Default mode is a dry run.

``--recover`` instead rebuilds crashed recordings (``probe()`` = ``unindexed``: the recorder
was killed before DataEnd/summary/Footer) into complete, indexed, zstd-chunked files; see
``recover`` and ``verify_recovered``. With ``--replace`` the original is kept beside it as
``<name>.mcap.unindexed-orig`` unless ``--discard-original``.

Usage:
    python tools/bag_compress.py ~/Desktop/rosbags                      # dry run + estimates
    nice -n 15 python tools/bag_compress.py ~/Desktop/rosbags --replace --report temp/bagz.json
    python tools/bag_compress.py ~/Desktop/rosbags --recover            # characterise crashed bags
    nice -n 15 python tools/bag_compress.py ~/Desktop/rosbags --recover --replace --write-metadata
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import io
import json
import os
import resource
import shutil
import struct
import sys
import time
import zlib
from concurrent.futures import ProcessPoolExecutor
from typing import Dict, List, Optional, Tuple

import zstandard

MAGIC = b"\x89MCAP0\r\n"
PARTIAL_SUFFIX = ".zst-partial"
GIB = 1 << 30
FREE_MARGIN = 2 * GIB
STALE_PARTIAL_S = 3600
MAX_RECORD = 1 << 30
DEFAULT_LOCK = "/tmp/jugglebot-bag-compress.lock"

OP_HEADER, OP_FOOTER, OP_SCHEMA, OP_CHANNEL, OP_MESSAGE = 0x01, 0x02, 0x03, 0x04, 0x05
OP_CHUNK, OP_MSG_INDEX, OP_CHUNK_INDEX, OP_ATTACHMENT, OP_ATT_INDEX = 0x06, 0x07, 0x08, 0x09, 0x0A
OP_STATISTICS, OP_METADATA, OP_META_INDEX, OP_SUMMARY_OFFSET, OP_DATA_END = 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
DATA_OPS = {OP_SCHEMA, OP_CHANNEL, OP_MESSAGE, OP_CHUNK, OP_MSG_INDEX, OP_ATTACHMENT, OP_METADATA}
SUMMARY_OPS = {OP_SCHEMA, OP_CHANNEL, OP_STATISTICS, OP_CHUNK_INDEX, OP_ATT_INDEX, OP_META_INDEX}
FOOTER_LEN = 1 + 8 + 20


class Refused(Exception):
    """The input is outside what the transcoder handles; the original is left untouched."""


class VerifyError(Exception):
    """The transcoded output does not match the original; it must never replace it."""


# --------------------------------------------------------------------------- probe

def _str(b: bytes, o: int) -> Tuple[bytes, int]:
    n = struct.unpack_from("<I", b, o)[0]
    if o + 4 + n > len(b):
        raise Refused("string overruns record")
    return b[o + 4:o + 4 + n], o + 4 + n


def _read_record(f) -> Tuple[int, bytes]:
    h = f.read(9)
    if len(h) != 9:
        raise Refused("truncated record header at %d" % (f.tell() - len(h)))
    op, ln = h[0], struct.unpack("<Q", h[1:])[0]
    if ln > MAX_RECORD:
        raise Refused("record 0x%02x of %d bytes exceeds the limit" % (op, ln))
    body = f.read(ln)
    if len(body) != ln:
        raise Refused("truncated record 0x%02x" % op)
    return op, body


def probe(path: str) -> Dict:
    """Classify a file from its footer + summary only (cheap, read-only).

    category: 'compress' | 'already' | 'unindexed' | 'refuse' ; plus reason and chunk info.
    """
    out = {"path": path, "size": os.path.getsize(path)}
    try:
        with open(path, "rb") as f:
            if f.read(8) != MAGIC:
                return dict(out, category="unindexed", reason="bad leading magic")
            if out["size"] < 8 + FOOTER_LEN + 8:
                return dict(out, category="unindexed", reason="too short")
            f.seek(-(FOOTER_LEN + 8), io.SEEK_END)
            tail = f.read(FOOTER_LEN + 8)
            if tail[-8:] != MAGIC or tail[0] != OP_FOOTER or struct.unpack_from("<Q", tail, 1)[0] != 20:
                return dict(out, category="unindexed", reason="no footer/trailing magic (crashed recording?)")
            summary_start, _, _ = struct.unpack_from("<QQI", tail, 9)
            if summary_start == 0:
                return dict(out, category="unindexed", reason="footer has no summary section")
            f.seek(summary_start)
            comps: Dict[str, int] = {}
            n_att = n_meta = 0
            chunk_bytes = 0
            while True:
                op, body = _read_record(f)
                if op == OP_FOOTER:
                    break
                if op == OP_CHUNK_INDEX:
                    off = 36 + struct.unpack_from("<I", body, 32)[0] + 8
                    c, off = _str(body, off)
                    comps[c.decode()] = comps.get(c.decode(), 0) + 1
                    chunk_bytes += struct.unpack_from("<Q", body, off + 8)[0]
                elif op == OP_ATT_INDEX:
                    n_att += 1
                elif op == OP_META_INDEX:
                    n_meta += 1
    except (Refused, struct.error, OSError) as e:
        return dict(out, category="unindexed", reason="unreadable summary: %s" % e)
    out.update(compressions=comps, attachments=n_att, metadata=n_meta, chunk_bytes=chunk_bytes)
    if not comps:
        return dict(out, category="already", reason="no chunk indexes in summary")
    if set(comps) == {"zstd"}:
        return dict(out, category="already", reason="already zstd")
    if set(comps) != {""}:
        return dict(out, category="refuse", reason="chunk compression %s" % sorted(comps))
    return dict(out, category="compress", reason="")


# --------------------------------------------------------------------------- transcode

class _Out:
    """Output writer tracking position and a running CRC over a chosen span."""

    def __init__(self, f):
        self.f, self.pos, self.crc = f, 0, 0

    def write(self, b: bytes) -> None:
        self.f.write(b)
        self.pos += len(b)
        self.crc = zlib.crc32(b, self.crc)

    def record(self, op: int, body: bytes) -> None:
        self.write(bytes([op]) + struct.pack("<Q", len(body)) + body)


def transcode(src: str, dst: str, level: int = 3, compress=None) -> Dict:
    """Write ``dst`` = ``src`` with every Chunk zstd-compressed. Raises Refused."""
    if compress is None:
        cctx = zstandard.ZstdCompressor(level=level)
        compress = cctx.compress
    newpos: Dict[int, int] = {}          # data-section record start: old -> new
    chunk_new: Dict[int, Tuple[int, int, int, bytes]] = {}  # old start -> (new start, new len, csize, check)
    with open(src, "rb", buffering=1 << 20) as f, open(dst, "wb", buffering=1 << 20) as fo:
        o = _Out(fo)
        if f.read(8) != MAGIC:
            raise Refused("bad magic")
        o.write(MAGIC)
        op, body = _read_record(f)
        if op != OP_HEADER:
            raise Refused("first record is 0x%02x, not Header" % op)
        o.record(op, body)
        while True:
            old = f.tell()
            op, body = _read_record(f)
            newpos[old] = o.pos
            if op == OP_DATA_END:
                break
            if op not in DATA_OPS:
                raise Refused("unexpected opcode 0x%02x in data section" % op)
            if op != OP_CHUNK:
                o.record(op, body)
                continue
            ms, me, usz, crc = struct.unpack_from("<QQQI", body, 0)
            comp, p = _str(body, 28)
            if comp != b"":
                raise Refused("chunk compression %r" % comp)
            rl = struct.unpack_from("<Q", body, p)[0]
            recs = body[p + 8:]
            if len(recs) != rl or rl != usz:
                raise Refused("chunk at %d: records length %d / uncompressed_size %d / payload %d"
                              % (old, rl, usz, len(recs)))
            real_crc = zlib.crc32(recs)
            if crc != 0 and crc != real_crc:
                raise Refused("chunk at %d fails its own uncompressed_crc (input corrupt)" % old)
            cb = compress(recs)
            nb = struct.pack("<QQQI", ms, me, usz, real_crc) + struct.pack("<I", 4) + b"zstd" \
                + struct.pack("<Q", len(cb)) + cb
            start = o.pos
            o.record(OP_CHUNK, nb)
            chunk_new[old] = (start, o.pos - start, len(cb), struct.pack("<QQQQ", ms, me, usz, 9 + len(body)))
        if len(body) < 4:
            raise Refused("short DataEnd")
        o.crc_data = o.crc  # covers magic .. last byte before DataEnd (spec + StreamReader)
        o.record(OP_DATA_END, struct.pack("<I", o.crc_data) + body[4:])
        old_summary_start = f.tell()
        new_summary_start = o.pos

        # --- summary section
        tail_size = os.fstat(f.fileno()).st_size
        f.seek(tail_size - FOOTER_LEN - 8)
        fop, fbody = _read_record(f)
        if fop != OP_FOOTER or f.read(8) != MAGIC:
            raise Refused("no footer / trailing magic")
        ss, so, _ = struct.unpack("<QQI", fbody[:20])
        if ss != old_summary_start:
            raise Refused("summary_start %d != end of DataEnd %d" % (ss, old_summary_start))
        f.seek(ss)
        o.crc = 0
        sumpos: Dict[int, int] = {}
        while True:
            old = f.tell()
            op, body = _read_record(f)
            sumpos[old] = o.pos
            if op == OP_FOOTER:
                if old != tail_size - FOOTER_LEN - 8:
                    raise Refused("footer not at end of file")
                break
            if op == OP_CHUNK_INDEX:
                body = _rewrite_chunk_index(body, newpos, chunk_new)
            elif op in (OP_ATT_INDEX, OP_META_INDEX):
                (off,) = struct.unpack_from("<Q", body, 0)
                if off not in newpos:
                    raise Refused("index offset %d is not a data-section record" % off)
                body = struct.pack("<Q", newpos[off]) + body[8:]
            elif op == OP_SUMMARY_OFFSET:
                gop, gs, gl = struct.unpack_from("<BQQ", body, 0)
                if gs not in sumpos or (gs + gl) not in sumpos and (gs + gl) != old:
                    raise Refused("summary offset group not on record boundaries")
                ns = sumpos[gs]
                ne = sumpos[gs + gl] if (gs + gl) in sumpos else o.pos
                body = struct.pack("<BQQ", gop, ns, ne - ns) + body[17:]
            elif op not in SUMMARY_OPS:
                raise Refused("unexpected opcode 0x%02x in summary" % op)
            o.record(op, body)
        if so != 0 and so not in sumpos:
            raise Refused("summary_offset_start %d not on a record boundary" % so)
        nso = sumpos[so] if so else 0
        prefix = bytes([OP_FOOTER]) + struct.pack("<QQQ", 20, new_summary_start, nso)
        summary_crc = zlib.crc32(prefix, o.crc)
        o.write(prefix + struct.pack("<I", summary_crc))
        o.write(MAGIC)
        fo.flush()
        os.fsync(fo.fileno())
    return {"chunks": len(chunk_new), "out_size": os.path.getsize(dst)}


def _rewrite_chunk_index(body: bytes, newpos, chunk_new) -> bytes:
    ms, me, cso, cl = struct.unpack_from("<QQQQ", body, 0)
    mlen = struct.unpack_from("<I", body, 32)[0]
    if mlen % 10:
        raise Refused("bad message_index_offsets length")
    pairs = [struct.unpack_from("<HQ", body, 36 + 10 * i) for i in range(mlen // 10)]
    p = 36 + mlen
    mil = struct.unpack_from("<Q", body, p)[0]
    comp, p = _str(body, p + 8)
    csz, usz = struct.unpack_from("<QQ", body, p)
    if p + 16 != len(body) or comp != b"":
        raise Refused("unexpected ChunkIndex layout/compression")
    if cso not in chunk_new:
        raise Refused("ChunkIndex points at %d, not a chunk" % cso)
    ns, nl, ncsz, check = chunk_new[cso]
    if check != struct.pack("<QQQQ", ms, me, usz, cl) or csz != usz:
        raise Refused("ChunkIndex at %d disagrees with its chunk" % cso)
    try:
        npairs = b"".join(struct.pack("<HQ", ch, newpos[off]) for ch, off in pairs)
    except KeyError:
        raise Refused("message_index_offsets not on record boundaries")
    return (struct.pack("<QQQQI", ms, me, ns, nl, mlen) + npairs + struct.pack("<Q", mil)
            + struct.pack("<I", 4) + b"zstd" + struct.pack("<QQ", ncsz, usz))


# --------------------------------------------------------------------------- recover
# A recorder SIGKILLed mid-write leaves Magic, Header, Chunk, MessageIndex... and then a cut
# at an arbitrary byte (Foxy's files end on 4 KiB boundaries): no DataEnd, summary or
# Footer, so seeking readers (the GUI) cannot open it. recover() rebuilds a complete,
# indexed, zstd-chunked file from every complete record. It never invents data: chunk
# payloads are copied byte for byte, MessageIndex records are regenerated from those
# payloads (and must equal the input's where the input has them), and the summary is
# computed from the messages actually written.

ORIG_SUFFIX = ".unindexed-orig"
OP_NAMES = {OP_HEADER: "Header", OP_FOOTER: "Footer", OP_SCHEMA: "Schema", OP_CHANNEL: "Channel",
            OP_MESSAGE: "Message", OP_CHUNK: "Chunk", OP_MSG_INDEX: "MessageIndex",
            OP_CHUNK_INDEX: "ChunkIndex", OP_ATTACHMENT: "Attachment", OP_ATT_INDEX: "AttachmentIndex",
            OP_STATISTICS: "Statistics", OP_METADATA: "Metadata", OP_META_INDEX: "MetadataIndex",
            OP_SUMMARY_OFFSET: "SummaryOffset", OP_DATA_END: "DataEnd"}
MSG_FIXED = 2 + 4 + 8 + 8


class NothingToRecover(Refused):
    """The input holds no complete message; there is nothing worth rewriting."""


def _check_schema(body: bytes) -> int:
    sid = struct.unpack_from("<H", body, 0)[0]
    _, p = _str(body, 2)
    _, p = _str(body, p)
    _, p = _str(body, p)
    if p != len(body) or sid == 0:
        raise Refused("malformed Schema record")
    return sid


def _check_channel(body: bytes, schemas) -> int:
    cid, sid = struct.unpack_from("<HH", body, 0)
    _, p = _str(body, 4)
    _, p = _str(body, p)
    mlen = struct.unpack_from("<I", body, p)[0]
    end = p + 4 + mlen
    p += 4
    while p < end:
        _, p = _str(body, p)
        _, p = _str(body, p)
    if p != end or end != len(body):
        raise Refused("malformed Channel record")
    if sid != 0 and sid not in schemas:
        raise Refused("Channel %d names unknown schema %d" % (cid, sid))
    return cid


def _walk_chunk(recs: bytes, schemas: Dict, chans: Dict, strict: bool, t_lo=0, t_hi=None):
    """Walk an uncompressed chunk payload. ``schemas``/``chans`` (id -> record body) are
    updated in place. Returns (msgs, used): msgs = [(channel_id, log_time, offset)], used =
    bytes of complete, accepted records. strict (a complete chunk): anything malformed
    raises Refused. Otherwise (salvage from a cut chunk): stop at the first record that is
    incomplete or fails a check."""
    msgs = []
    o = 0
    n = len(recs)
    while o < n:
        try:
            if o + 9 > n:
                raise Refused("record header cut at %d" % o)
            op, ln = recs[o], struct.unpack_from("<Q", recs, o + 1)[0]
            if o + 9 + ln > n:
                raise Refused("record 0x%02x cut at %d" % (op, o))
            body = recs[o + 9:o + 9 + ln]
            if op == OP_MESSAGE:
                if ln < MSG_FIXED:
                    raise Refused("short Message at %d" % o)
                ch, _, lt = struct.unpack_from("<HIQ", body, 0)
                if ch not in chans:
                    raise Refused("Message on undeclared channel %d" % ch)
                if lt < t_lo or (t_hi is not None and lt > t_hi):
                    raise Refused("Message log_time outside its chunk's range")
                msgs.append((ch, lt, o))
            elif op == OP_SCHEMA:
                sid = _check_schema(body)
                if schemas.get(sid, body) != body:
                    raise Refused("Schema %d redefined differently" % sid)
                schemas[sid] = body
            elif op == OP_CHANNEL:
                cid = _check_channel(body, schemas)
                if chans.get(cid, body) != body:
                    raise Refused("Channel %d redefined differently" % cid)
                chans[cid] = body
            else:
                raise Refused("opcode 0x%02x inside a chunk" % op)
        except (Refused, struct.error) as e:
            if strict:
                raise Refused("chunk payload: %s" % e)
            break
        o += 9 + ln
    return msgs, o


def _mi_bodies(msgs) -> Dict[int, bytes]:
    """MessageIndex bodies per channel, in first-message order (as mcap's Writer emits)."""
    per: Dict[int, List[Tuple[int, int]]] = {}
    for ch, lt, off in msgs:
        per.setdefault(ch, []).append((lt, off))
    return {ch: struct.pack("<HI", ch, 16 * len(v)) + b"".join(struct.pack("<QQ", *e) for e in v)
            for ch, v in per.items()}


def recover(src: str, dst: Optional[str] = None, level: int = 3, compress=None) -> Dict:
    """Characterise a crashed recording and, when ``dst`` is given, write the recovered file.

    Raises Refused for anything outside the crashed-Foxy shape (records other than Header,
    Schema, Channel, Chunk, MessageIndex before the cut; a compressed or CRC-failing chunk;
    an input MessageIndex that disagrees with its chunk) and NothingToRecover when no
    complete message exists."""
    from mcap.data_stream import RecordBuilder
    from mcap.records import ChunkIndex, Footer, Statistics, SummaryOffset
    from mcap.opcode import Opcode

    if dst is not None and compress is None:
        compress = zstandard.ZstdCompressor(level=level).compress
    info: Dict = {"size": os.path.getsize(src), "chunks": 0, "messages": 0, "mi_full": 0,
                  "mi_partial": 0, "mi_none": 0, "top_schema_channel": 0, "chunk_time_fixed": 0,
                  "data_end": False, "end": "", "tail_bytes": 0, "salvaged_messages": 0,
                  "salvaged_bytes": 0, "salvage": None}
    schemas: Dict[int, bytes] = {}
    chans: Dict[int, bytes] = {}
    counts: Dict[int, int] = {}
    tmin = tmax = None
    cis: List = []
    fo = open(dst, "wb", buffering=1 << 20) if dst else None
    o = _Out(fo) if fo else None

    def put_chunk(recs: bytes, msgs, in_crc: int, hdr_ms: int, hdr_me: int, in_mis, where: int):
        nonlocal tmin, tmax
        regen = _mi_bodies(msgs)
        seen = {}
        for b in in_mis:
            ch = struct.unpack_from("<H", b, 0)[0]
            if ch in seen or regen.get(ch) != b:
                raise Refused("input MessageIndex for channel %d after chunk at %d disagrees "
                              "with the chunk" % (ch, where))
            seen[ch] = b
        key = "mi_full" if len(seen) == len(regen) else ("mi_partial" if seen else "mi_none")
        info[key] += 1
        crc = zlib.crc32(recs)
        if in_crc not in (0, crc):
            raise Refused("chunk at %d fails its own uncompressed_crc" % where)
        if msgs:
            ms, me = min(m[1] for m in msgs), max(m[1] for m in msgs)
            info["chunk_time_fixed"] += (ms, me) != (hdr_ms, hdr_me)
            tmin = ms if tmin is None else min(tmin, ms)
            tmax = me if tmax is None else max(tmax, me)
        else:
            ms, me = hdr_ms, hdr_me
        for ch, _, _ in msgs:
            counts[ch] = counts.get(ch, 0) + 1
        info["chunks"] += 1
        info["messages"] += len(msgs)
        if o is None:
            return
        cb = compress(recs)
        start = o.pos
        o.record(OP_CHUNK, struct.pack("<QQQI", ms, me, len(recs), crc) + struct.pack("<I", 4)
                 + b"zstd" + struct.pack("<Q", len(cb)) + cb)
        clen = o.pos - start
        mi_start = o.pos
        offs = {}
        for ch, b in regen.items():
            offs[ch] = o.pos
            o.record(OP_MSG_INDEX, b)
        cis.append(ChunkIndex(message_start_time=ms, message_end_time=me, chunk_start_offset=start,
                              chunk_length=clen, message_index_offsets=offs,
                              message_index_length=o.pos - mi_start, compression="zstd",
                              compressed_size=len(cb), uncompressed_size=len(recs)))

    try:
        with open(src, "rb", buffering=1 << 20) as f:
            if f.read(8) != MAGIC:
                raise Refused("bad leading magic")
            h = f.read(9)
            hb = f.read(struct.unpack("<Q", h[1:])[0]) if len(h) == 9 and h[0] == OP_HEADER else b""
            if len(h) < 9 or h[0] != OP_HEADER or len(hb) != struct.unpack("<Q", h[1:])[0]:
                info["end"] = "inside the Header"
                raise NothingToRecover("the file ends inside its Header")
            if o:
                o.write(MAGIC)
                o.record(OP_HEADER, hb)
            pending = None
            while True:
                pos = f.tell()
                h = f.read(9)
                if not h:
                    info["end"] = "clean record boundary"
                    break
                if len(h) < 9:
                    info["end"] = "mid record header (%d of 9 bytes)" % len(h)
                    break
                op, ln = h[0], struct.unpack("<Q", h[1:])[0]
                if op not in OP_NAMES or ln > MAX_RECORD:
                    info["end"] = "unreadable record (opcode 0x%02x, length %d)" % (op, ln)
                    break
                body = f.read(ln)
                if len(body) < ln:
                    info["end"] = "mid %s (%d of %d body bytes)" % (OP_NAMES[op], len(body), ln)
                    if op == OP_CHUNK:
                        if pending:
                            put_chunk(*pending)
                            pending = None
                        _salvage(body, pos, schemas, chans, info, put_chunk)
                    break
                if op == OP_MSG_INDEX:
                    if pending is None:
                        raise Refused("MessageIndex at %d follows no chunk" % pos)
                    pending[5].append(body)
                    continue
                if pending:
                    put_chunk(*pending)
                    pending = None
                if op == OP_CHUNK:
                    ms, me, usz, crc = struct.unpack_from("<QQQI", body, 0)
                    comp, p = _str(body, 28)
                    if comp != b"":
                        raise Refused("chunk at %d is compressed (%r)" % (pos, comp))
                    rl = struct.unpack_from("<Q", body, p)[0]
                    recs = body[p + 8:]
                    if len(recs) != rl or rl != usz:
                        raise Refused("chunk at %d: inconsistent lengths" % pos)
                    msgs, _ = _walk_chunk(recs, schemas, chans, strict=True)
                    pending = [recs, msgs, crc, ms, me, [], pos]
                elif op == OP_SCHEMA:
                    sid = _check_schema(body)
                    if schemas.setdefault(sid, body) != body:
                        raise Refused("Schema %d redefined differently" % sid)
                    info["top_schema_channel"] += 1
                    if o:
                        o.record(op, body)
                elif op == OP_CHANNEL:
                    cid = _check_channel(body, schemas)
                    if chans.setdefault(cid, body) != body:
                        raise Refused("Channel %d redefined differently" % cid)
                    info["top_schema_channel"] += 1
                    if o:
                        o.record(op, body)
                elif op == OP_DATA_END:
                    info["data_end"] = True
                    info["end"] = "after DataEnd (summary/footer missing)"
                    break
                else:
                    raise Refused("%s record at %d is not handled in a crashed recording"
                                  % (OP_NAMES[op], pos))
            if pending:
                put_chunk(*pending)
            info["tail_bytes"] = info["size"] - pos
        info.update(schemas=len(schemas), channels=len(chans), message_start_time=tmin,
                    message_end_time=tmax, channel_message_counts=dict(counts))
        if not info["messages"]:
            raise NothingToRecover("no complete message (%s)" % info["end"])
        if o is None:
            return info

        o.record(OP_DATA_END, struct.pack("<I", o.crc))   # crc covers magic .. before DataEnd
        ss = o.pos
        sb = RecordBuilder()
        offsets = []

        def group(opc, write):
            g0 = sb.count
            write()
            offsets.append(SummaryOffset(group_opcode=opc, group_start=ss + g0, group_length=sb.count - g0))

        group(Opcode.SCHEMA, lambda: [sb.write(bytes([OP_SCHEMA]) + struct.pack("<Q", len(b)) + b)
                                      for b in schemas.values()])
        group(Opcode.CHANNEL, lambda: [sb.write(bytes([OP_CHANNEL]) + struct.pack("<Q", len(b)) + b)
                                       for b in chans.values()])
        stats = Statistics(message_count=info["messages"], schema_count=len(schemas),
                           channel_count=len(chans), attachment_count=0, metadata_count=0,
                           chunk_count=len(cis), message_start_time=tmin, message_end_time=tmax,
                           channel_message_counts=dict(counts))
        group(Opcode.STATISTICS, lambda: stats.write(sb))
        group(Opcode.CHUNK_INDEX, lambda: [ci.write(sb) for ci in cis])
        group(Opcode.ATTACHMENT_INDEX, lambda: None)
        group(Opcode.METADATA_INDEX, lambda: None)
        so = ss + sb.count
        for x in offsets:
            x.write(sb)
        summary = sb.end()
        crc = zlib.crc32(struct.pack("<BQQQ", OP_FOOTER, 20, ss, so), zlib.crc32(summary))
        o.write(summary)
        Footer(summary_start=ss, summary_offset_start=so, summary_crc=crc).write(sb)
        o.write(sb.end() + MAGIC)
        fo.flush()
        os.fsync(fo.fileno())
        info["out_size"] = o.pos
        return info
    finally:
        if fo:
            fo.close()


def _salvage(body: bytes, pos: int, schemas, chans, info, put_chunk) -> None:
    """The file ends inside a Chunk. Its payload is uncompressed and the chunk header was
    written whole before it, so every inner record fully present in the cut payload is a
    record the recorder wrote. Salvage them into a final chunk, after checking each one
    (opcode, exact field layout, declared channel/schema, log_time inside the header's
    [start, end]); stop at the first that fails. Nothing is salvaged unless at least one
    message is; schemas/channels are only committed with it."""
    s = {"chunk_offset": pos, "present": len(body)}
    info["salvage"] = s
    try:
        ms, me, usz, crc = struct.unpack_from("<QQQI", body, 0)
        comp, p = _str(body, 28)
        rl = struct.unpack_from("<Q", body, p)[0]
    except (Refused, struct.error):
        s["why"] = "chunk header cut"
        return
    payload = body[p + 8:]
    s.update(payload_offset=pos + 9 + p + 8, payload_present=len(payload), payload_total=rl)
    if comp != b"" or rl != usz or len(payload) >= rl:
        s["why"] = "chunk header implausible"
        return
    sc, cc = dict(schemas), dict(chans)
    msgs, used = _walk_chunk(payload, sc, cc, strict=False, t_lo=ms, t_hi=me)
    s["used"] = used
    if not msgs:
        s["why"] = "no complete message in the cut payload"
        return
    recs = payload[:used]
    schemas.update(sc)
    chans.update(cc)
    put_chunk(recs, msgs, 0, min(m[1] for m in msgs), max(m[1] for m in msgs), [], pos)
    info["salvaged_messages"] = len(msgs)
    info["salvaged_bytes"] = used
    s["why"] = ""


def characterise_line(info: Dict) -> str:
    sv = info.get("salvage") or {}
    tail = ("cut chunk: %d of %d payload bytes, salvaged %d msgs / %d B, dropped %d B"
            % (sv.get("payload_present", sv.get("present", 0)), sv.get("payload_total", 0),
               info["salvaged_messages"], info["salvaged_bytes"],
               sv.get("payload_present", 0) - sv.get("used", 0))
            + (" (%s)" % sv["why"] if sv.get("why") else "")) if sv else "no cut chunk"
    return ("%d msgs in %d chunks; ends %s; MI full/partial/none %d/%d/%d; %s; schemas %d channels %d"
            " (top-level %d)%s" % (info["messages"], info["chunks"], info["end"], info["mi_full"],
                                   info["mi_partial"], info["mi_none"], tail, info.get("schemas", 0),
                                   info.get("channels", 0), info["top_schema_channel"],
                                   "; chunk times fixed %d" % info["chunk_time_fixed"]
                                   if info["chunk_time_fixed"] else ""))


# --------------------------------------------------------------------------- verify
# Everything below reads files through the installed ``mcap`` library (or, for the summary
# CRC, a ten-line reader of the footer), never through transcode()'s parser.

def content_digest(path: str, truncated_ok: bool = False, split_at: Optional[int] = None) -> Dict:
    """SHA-256 over schemas, channels and messages in file order, via mcap's StreamReader
    with CRC validation (chunk CRCs and, when non-zero, the data-section CRC). Streaming:
    memory is bounded by one chunk.

    truncated_ok: a file cut mid-record (a crashed recording) ends the stream at the cut
    instead of raising; any other library error (a CRC failure) still raises.
    split_at: also return ``prefix_sha256``, the digest after the first ``split_at``
    content records, and ``tail``, the records after them (the salvaged ones)."""
    from mcap.exceptions import EndOfFile
    from mcap.records import Channel, DataEnd, Message, Schema
    from mcap.stream_reader import StreamReader

    h = hashlib.sha256()
    counts: Dict[int, int] = {}
    n = n_rec = 0
    prefix = None
    tail: List = []
    extra: Dict = {}
    defs: Dict = {}
    with open(path, "rb") as f:
        try:
            for r in StreamReader(f, validate_crcs=True).records:
                if isinstance(r, DataEnd):
                    break
                if not isinstance(r, (Message, Schema, Channel)):
                    continue
                if split_at is not None:
                    if n_rec == split_at:
                        prefix = h.hexdigest()
                    if n_rec >= split_at:
                        tail.append(r)
                n_rec += 1
                if isinstance(r, Message):
                    h.update(struct.pack("<BHIQQQ", 5, r.channel_id, r.sequence, r.log_time,
                                         r.publish_time, len(r.data)))
                    h.update(r.data)
                    counts[r.channel_id] = counts.get(r.channel_id, 0) + 1
                    n += 1
                elif isinstance(r, Schema):
                    h.update(repr((3, r.id, r.name, r.encoding, len(r.data))).encode())
                    h.update(r.data)
                    defs[("s", r.id)] = r
                elif isinstance(r, Channel):
                    defs[("c", r.id)] = r
                    h.update(repr((4, r.id, r.schema_id, r.topic, r.message_encoding,
                                   sorted(r.metadata.items()))).encode())
        except (EndOfFile, struct.error) as e:
            if not truncated_ok:
                raise
            extra["truncated"] = "%s: %s" % (type(e).__name__, e)
    if split_at is not None:
        extra.update(prefix_sha256=prefix if prefix is not None else (
            h.hexdigest() if n_rec == split_at else None), tail=tail, defs=defs)
    return dict({"sha256": h.hexdigest(), "messages": n, "counts": counts, "records": n_rec}, **extra)


def summary_crc_ok(path: str) -> bool:
    with open(path, "rb") as f:
        f.seek(-(FOOTER_LEN + 8), io.SEEK_END)
        foot_pos = f.tell()
        foot = f.read(FOOTER_LEN)
        ss, so, crc = struct.unpack_from("<QQI", foot, 9)
        f.seek(ss)
        data = f.read(foot_pos - ss)
    return crc == 0 or zlib.crc32(foot[:25], zlib.crc32(data)) == crc


def _ci_key(ci):
    return (ci.message_start_time, ci.message_end_time, ci.uncompressed_size,
            sorted(ci.message_index_offsets), ci.message_index_length)


def _msg_key(m):
    return (m.channel_id, m.sequence, m.log_time, m.publish_time, bytes(m.data))


def _check_chunks(fb, chunk_indexes) -> int:
    """Every ChunkIndex -> a CRC-valid zstd Chunk of its length; every MessageIndex entry ->
    the message it names. Returns the number of index entries checked."""
    from mcap.data_stream import ReadDataStream
    from mcap.records import Chunk, MessageIndex
    from mcap.stream_reader import get_chunk_data_stream

    n_idx = 0
    for ci in chunk_indexes:
        if ci.compression != "zstd":
            raise VerifyError("chunk at %d is %r" % (ci.chunk_start_offset, ci.compression))
        fb.seek(ci.chunk_start_offset)
        h = fb.read(9)
        if h[0] != 0x06 or struct.unpack("<Q", h[1:])[0] != ci.chunk_length - 9:
            raise VerifyError("ChunkIndex -> %d is not a Chunk of its length" % ci.chunk_start_offset)
        chunk = Chunk.read(ReadDataStream(fb))
        if (chunk.compression != "zstd" or len(chunk.data) != ci.compressed_size
                or chunk.uncompressed_size != ci.uncompressed_size or chunk.uncompressed_crc == 0
                or chunk.message_start_time != ci.message_start_time
                or chunk.message_end_time != ci.message_end_time):
            raise VerifyError("chunk at %d disagrees with its index" % ci.chunk_start_offset)
        stream, _ = get_chunk_data_stream(chunk, validate_crc=True)
        data = stream.read(chunk.uncompressed_size)
        for ch, off in ci.message_index_offsets.items():
            fb.seek(off)
            if fb.read(9)[0] != 0x07:
                raise VerifyError("message_index_offsets -> %d is not a MessageIndex" % off)
            mi = MessageIndex.read(ReadDataStream(fb))
            if mi.channel_id != ch:
                raise VerifyError("MessageIndex channel mismatch at %d" % off)
            for lt, moff in mi.records:
                if data[moff] != 0x05 or struct.unpack_from("<HIQ", data, moff + 9)[::2] != (ch, lt):
                    raise VerifyError("MessageIndex entry does not hit its message")
                n_idx += 1
    return n_idx


def indexed_check(orig: str, new: str) -> Dict:
    """The seeking path: summaries, every ChunkIndex -> Chunk (CRC), every MessageIndex
    entry -> a Message at that offset, first/last messages via iter_messages, attachments
    and metadata via their indexes."""
    from mcap.reader import make_reader

    with open(orig, "rb") as fa, open(new, "rb") as fb:
        ra, rb = make_reader(fa, validate_crcs=True), make_reader(fb, validate_crcs=True)
        sa, sb = ra.get_summary(), rb.get_summary()
        if sa is None or sb is None:
            raise VerifyError("missing summary")
        if sa.statistics != sb.statistics:
            raise VerifyError("statistics differ: %r vs %r" % (sa.statistics, sb.statistics))
        if sa.schemas != sb.schemas or sa.channels != sb.channels:
            raise VerifyError("summary schemas/channels differ")
        if [_ci_key(c) for c in sa.chunk_indexes] != [_ci_key(c) for c in sb.chunk_indexes]:
            raise VerifyError("chunk index lists differ")
        for a, b in ((sa.attachment_indexes, sb.attachment_indexes),
                     (sa.metadata_indexes, sb.metadata_indexes)):
            if [(x.name, x.length) for x in a] != [(x.name, x.length) for x in b]:
                raise VerifyError("attachment/metadata indexes differ")
        if list(ra.iter_attachments()) != list(rb.iter_attachments()):
            raise VerifyError("attachments differ")
        if list(ra.iter_metadata()) != list(rb.iter_metadata()):
            raise VerifyError("metadata differ")
        n_idx = _check_chunks(fb, sb.chunk_indexes)
        if sb.chunk_indexes:
            last = max(sb.chunk_indexes, key=lambda c: c.message_end_time)
            for kw in ({}, {"start_time": last.message_end_time}):
                ma = next(ra.iter_messages(**kw), None)
                mb = next(rb.iter_messages(**kw), None)
                if (ma is None) != (mb is None) or (ma and _msg_key(ma[2]) != _msg_key(mb[2])):
                    raise VerifyError("indexed read of %r differs" % (kw,))
        return {"chunks": len(sb.chunk_indexes), "indexed_messages": n_idx}


def verify(orig: str, new: str) -> Dict:
    """Raise VerifyError unless ``new`` carries exactly ``orig``'s content and indexes.
    Any exception from the reading library (CRC error, decode error) is a failed verify."""
    try:
        return _verify(orig, new)
    except VerifyError:
        raise
    except Exception as e:  # noqa: BLE001
        raise VerifyError("%s: %s" % (type(e).__name__, e))


def _verify(orig: str, new: str) -> Dict:
    t = time.time()
    if not summary_crc_ok(new):
        raise VerifyError("summary_crc mismatch in output")
    da, db = content_digest(orig), content_digest(new)
    if da != db:
        raise VerifyError("content digest differs (%d vs %d messages)" % (da["messages"], db["messages"]))
    info = indexed_check(orig, new)
    from mcap.reader import make_reader
    with open(new, "rb") as f:
        st = make_reader(f).get_summary().statistics
    if st is not None and (st.message_count != db["messages"] or
                           {k: v for k, v in st.channel_message_counts.items() if v} != db["counts"]):
        raise VerifyError("statistics disagree with the streamed message counts")
    info.update(messages=db["messages"], sha256=db["sha256"], verify_s=time.time() - t)
    return info


def verify_recovered(orig: str, new: str, info: Dict) -> Dict:
    """Raise VerifyError unless ``new`` is a complete, self-consistent, indexed MCAP that
    carries exactly the original's library-readable content, plus (only) the records
    recover() salvaged from a cut chunk, which must equal the original's bytes."""
    try:
        return _verify_recovered(orig, new, info)
    except VerifyError:
        raise
    except Exception as e:  # noqa: BLE001
        raise VerifyError("%s: %s" % (type(e).__name__, e))


def _verify_recovered(orig: str, new: str, info: Dict) -> Dict:
    from mcap.data_stream import RecordBuilder
    from mcap.reader import make_reader
    from mcap.records import Channel, Message, Schema

    t = time.time()
    if not summary_crc_ok(new):
        raise VerifyError("summary_crc mismatch in output")
    # (a) content: the original as far as the library can stream it == the output's prefix
    da = content_digest(orig, truncated_ok=True)
    db = content_digest(new, split_at=da["records"])
    if "truncated" in db:
        raise VerifyError("output is itself truncated")
    if db["prefix_sha256"] != da["sha256"]:
        raise VerifyError("content digest differs from the original's readable prefix")
    tail = db.pop("tail")
    defs = db.pop("defs")
    sv = info.get("salvage") or {}
    if tail:
        if not info["salvaged_messages"] or "payload_offset" not in sv:
            raise VerifyError("output has %d records the original's stream lacks" % len(tail))
        rb = RecordBuilder()
        for r in tail:
            if not isinstance(r, (Message, Schema, Channel)):
                raise VerifyError("unexpected salvaged record %r" % type(r))
            r.write(rb)
        raw = rb.end()
        with open(orig, "rb") as f:
            f.seek(sv["chunk_offset"])
            hdr = f.read(9)
            f.seek(sv["payload_offset"])
            src = f.read(len(raw) + 1)
        if hdr[0] != OP_CHUNK or sv["chunk_offset"] + 9 + struct.unpack("<Q", hdr[1:])[0] \
                <= os.path.getsize(orig):
            raise VerifyError("salvage source is not a cut Chunk in the original")
        if src[:len(raw)] != raw:
            raise VerifyError("salvaged records differ from the original's bytes")
        if sum(isinstance(r, Message) for r in tail) != info["salvaged_messages"]:
            raise VerifyError("salvaged message count mismatch")
    elif info.get("salvaged_messages"):
        raise VerifyError("salvaged messages missing from the output")
    # (b) the output against itself: summary, statistics, every index, the seeking read
    with open(new, "rb") as f:
        r = make_reader(f, validate_crcs=True)
        s = r.get_summary()
        if s is None or s.statistics is None:
            raise VerifyError("no summary/statistics")
        st = s.statistics
        if (st.message_count != db["messages"]
                or {k: v for k, v in st.channel_message_counts.items() if v} != db["counts"]
                or st.chunk_count != len(s.chunk_indexes) or st.schema_count != len(s.schemas)
                or st.channel_count != len(s.channels)):
            raise VerifyError("statistics disagree with the streamed content")
        if (s.schemas != {k[1]: v for k, v in defs.items() if k[0] == "s"}
                or s.channels != {k[1]: v for k, v in defs.items() if k[0] == "c"}):
            raise VerifyError("summary schemas/channels differ from the data section's")
        n_idx = _check_chunks(f, s.chunk_indexes)
        if n_idx != db["messages"]:
            raise VerifyError("%d message index entries for %d messages" % (n_idx, db["messages"]))
        n = 0
        prev = None
        lo = hi = None
        for _, ch, m in r.iter_messages(log_time_order=True):
            if prev is not None and m.log_time < prev:
                raise VerifyError("iter_messages(log_time_order=True) out of order")
            if ch.id not in s.channels:
                raise VerifyError("message on a channel missing from the summary")
            prev = m.log_time
            lo = m.log_time if lo is None else min(lo, m.log_time)
            hi = m.log_time if hi is None else max(hi, m.log_time)
            n += 1
        if n != db["messages"]:
            raise VerifyError("seeking read returned %d of %d messages" % (n, db["messages"]))
        if (st.message_start_time, st.message_end_time) != (lo, hi):
            raise VerifyError("statistics time range disagrees with the messages")
    # (c) the tool's own classifier now calls it finished
    if probe(new)["category"] != "already":
        raise VerifyError("probe() does not classify the output as indexed")
    return {"messages": db["messages"], "orig_readable_messages": da["messages"],
            "sha256": db["sha256"], "chunks": len(s.chunk_indexes), "verify_s": time.time() - t}


# --------------------------------------------------------------------------- per-file job

def _fsync_dir(d: str) -> None:
    fd = os.open(d, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _rss_mb() -> float:
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


def estimate_ratio(path: str, level: int = 3, samples: int = 16) -> Optional[float]:
    """Compress a few chunks spread through the file (dry run only)."""
    from mcap.reader import make_reader
    from mcap.records import Chunk
    from mcap.data_stream import ReadDataStream
    with open(path, "rb") as f:
        s = make_reader(f).get_summary()
        cis = s.chunk_indexes if s else []
        if not cis:
            return None
        pick = [cis[int(i * (len(cis) - 1) / max(1, samples - 1))] for i in range(min(samples, len(cis)))]
        cctx = zstandard.ZstdCompressor(level=level)
        raw = comp = 0
        for ci in pick:
            f.seek(ci.chunk_start_offset + 9)
            c = Chunk.read(ReadDataStream(f))
            raw += len(c.data)
            comp += len(cctx.compress(c.data))
    return raw / comp if comp else None


def process_file(path: str, opts: Dict, compress=None, corrupt_hook=None) -> Dict:
    """One file end to end. Never raises; returns a result dict with 'action'."""
    res = {"path": path, "before": 0, "after": None}
    try:
        st = os.stat(path)
        res["before"] = st.st_size
        p = probe(path)
        res["reason"] = p.get("reason", "")
        if opts.get("recover"):
            return _process_recover(path, opts, st, p, res, compress, corrupt_hook)
        if p["category"] == "already":
            return dict(res, action="skip-done")
        if p["category"] == "unindexed":
            return dict(res, action="skip-unindexed")
        if p["category"] == "refuse":
            return dict(res, action="refused")
        age_h = (time.time() - st.st_mtime) / 3600.0
        if age_h < opts["min_age_hours"]:
            return dict(res, action="skip-young", reason="age %.1f h" % age_h)
        if not opts["replace"]:
            r = estimate_ratio(path, opts["level"])
            est = res["before"] / r if r else None
            return dict(res, action="would-compress", after=est, est_ratio=r)
        if st.st_nlink != 1 or st.st_uid != os.getuid():
            return dict(res, action="refused", reason="hard-linked or owned by another user")
        free = shutil.disk_usage(os.path.dirname(os.path.abspath(path))).free
        if free < st.st_size + FREE_MARGIN:
            return dict(res, action="refused", reason="free space %.1f GB too low" % (free / GIB))
        partial = path + PARTIAL_SUFFIX
        t0 = time.time()
        try:
            tc = transcode(path, partial, opts["level"], compress=compress)
            t1 = time.time()
            if corrupt_hook:
                corrupt_hook(partial)
            v = verify(path, partial)
            st2 = os.stat(path)
            if (st2.st_size, st2.st_mtime_ns) != (st.st_size, st.st_mtime_ns):
                raise VerifyError("original changed during the run")
            os.chmod(partial, st.st_mode & 0o7777)
            os.utime(partial, ns=(st.st_atime_ns, st.st_mtime_ns))
            fd = os.open(partial, os.O_RDONLY)
            os.fsync(fd)
            os.close(fd)
            os.replace(partial, path)
            _fsync_dir(os.path.dirname(os.path.abspath(path)))
        except Refused as e:
            _unlink(partial)
            return dict(res, action="refused", reason=str(e))
        except VerifyError as e:
            _unlink(partial)
            return dict(res, action="FAILED-verify", reason=str(e))
        except BaseException:
            _unlink(partial)
            raise
        after = os.path.getsize(path)
        return dict(res, action="compressed", after=after, transcode_s=t1 - t0,
                    verify_s=v["verify_s"], messages=v["messages"], chunks=tc["chunks"],
                    content_sha256=v["sha256"],
                    peak_rss_mb=_rss_mb())
    except Exception as e:  # noqa: BLE001 - a per-file failure must not stop the run
        return dict(res, action="FAILED-error", reason="%s: %s" % (type(e).__name__, e))


def _process_recover(path: str, opts: Dict, st, p: Dict, res: Dict, compress, corrupt_hook) -> Dict:
    """--recover: rebuild an `unindexed` (crashed) recording; skip everything else."""
    meta = os.path.join(os.path.dirname(os.path.abspath(path)), "metadata.yaml")
    if p["category"] != "unindexed":
        if (p["category"] == "already" and opts.get("write_metadata") and not os.path.exists(meta)
                and len([n for n in os.listdir(os.path.dirname(meta)) if n.endswith(".mcap")]) == 1):
            if not opts["replace"]:
                return dict(res, action="would-write-meta")
            write_metadata(path)
            return dict(res, action="wrote-meta")
        return dict(res, action="skip-indexed" if p["category"] == "already" else "skip-not-crashed")
    age_h = (time.time() - st.st_mtime) / 3600.0
    if age_h < opts["min_age_hours"]:
        return dict(res, action="skip-young", reason="age %.1f h" % age_h)
    try:
        info = recover(path) if not opts["replace"] else None
    except NothingToRecover as e:
        return dict(res, action="skip-empty", reason=str(e))
    except Refused as e:
        return dict(res, action="refused", reason=str(e))
    if info is not None:
        return dict(res, action="would-recover", reason=characterise_line(info),
                    messages=info["messages"], salvaged=info["salvaged_messages"])
    kept = path + ORIG_SUFFIX
    if st.st_nlink != 1 or st.st_uid != os.getuid():
        return dict(res, action="refused", reason="hard-linked or owned by another user")
    if os.path.lexists(kept):
        return dict(res, action="refused", reason="%s already exists" % kept)
    free = shutil.disk_usage(os.path.dirname(os.path.abspath(path))).free
    if free < st.st_size + FREE_MARGIN:
        return dict(res, action="refused", reason="free space %.1f GB too low" % (free / GIB))
    partial = path + PARTIAL_SUFFIX
    linked = replaced = False
    t0 = time.time()
    try:
        info = recover(path, partial, opts["level"], compress=compress)
        t1 = time.time()
        if corrupt_hook:
            corrupt_hook(partial)
        v = verify_recovered(path, partial, info)
        st2 = os.stat(path)
        if (st2.st_size, st2.st_mtime_ns) != (st.st_size, st.st_mtime_ns):
            raise VerifyError("original changed during the run")
        os.chmod(partial, st.st_mode & 0o7777)
        os.utime(partial, ns=(st.st_atime_ns, st.st_mtime_ns))
        fd = os.open(partial, os.O_RDONLY)
        os.fsync(fd)
        os.close(fd)
        if not opts.get("discard_original"):
            os.link(path, kept)          # atomic; fails if it exists. path never goes missing
            linked = True
        os.replace(partial, path)
        replaced = True
        _fsync_dir(os.path.dirname(os.path.abspath(path)))
    except NothingToRecover as e:
        _unlink(partial)
        return dict(res, action="skip-empty", reason=str(e))
    except Refused as e:
        _unlink(partial)
        return dict(res, action="refused", reason=str(e))
    except VerifyError as e:
        _unlink(partial)
        if linked:
            _unlink(kept)
        return dict(res, action="FAILED-verify", reason=str(e))
    except BaseException:
        _unlink(partial)
        if linked and not replaced:
            _unlink(kept)
        raise
    out = dict(res, action="recovered", after=os.path.getsize(path), transcode_s=t1 - t0,
               verify_s=v["verify_s"], messages=v["messages"], salvaged=info["salvaged_messages"],
               chunks=v["chunks"], content_sha256=v["sha256"], kept=kept if linked else None,
               reason=characterise_line(info), peak_rss_mb=_rss_mb())
    if opts.get("write_metadata") and not os.path.exists(meta):
        write_metadata(path)
        out["metadata_yaml"] = meta
    return out


def metadata_yaml(path: str) -> str:
    """A rosbag2 version-4 metadata.yaml (the shape Foxy's recorder writes on a clean close)
    derived from an indexed MCAP's summary: every channel, its schema name as the type, its
    message_encoding, the ``offered_qos_profiles`` the recorder stored in the channel
    metadata, and the per-channel counts and time span from the Statistics record."""
    from mcap.reader import make_reader
    with open(path, "rb") as f:
        s = make_reader(f).get_summary()
    st = s.statistics
    lines = ["rosbag2_bagfile_information:", "  version: 4", "  storage_identifier: mcap",
             "  relative_file_paths:", "    - %s" % os.path.basename(path),
             "  duration:", "    nanoseconds: %d" % (st.message_end_time - st.message_start_time),
             "  starting_time:", "    nanoseconds_since_epoch: %d" % st.message_start_time,
             "  message_count: %d" % st.message_count, "  topics_with_message_count:"]
    for cid, ch in s.channels.items():
        sch = s.schemas.get(ch.schema_id)
        lines += ["    - topic_metadata:", "        name: %s" % ch.topic,
                  "        type: %s" % (sch.name if sch else ""),
                  "        serialization_format: %s" % ch.message_encoding,
                  "        offered_qos_profiles: %s" % json.dumps(ch.metadata.get("offered_qos_profiles", "")),
                  "      message_count: %d" % st.channel_message_counts.get(cid, 0)]
    lines += ['  compression_format: ""', '  compression_mode: ""']
    return "\n".join(lines)


def write_metadata(path: str) -> str:
    """Write metadata.yaml beside ``path`` (never over an existing one), atomically."""
    meta = os.path.join(os.path.dirname(os.path.abspath(path)), "metadata.yaml")
    tmp = meta + ".tmp"
    with open(tmp, "w") as f:
        f.write(metadata_yaml(path))
        f.flush()
        os.fsync(f.fileno())
    try:
        os.link(tmp, meta)
    finally:
        os.unlink(tmp)
    _fsync_dir(os.path.dirname(meta))
    return meta


def _unlink(p: str) -> None:
    try:
        os.unlink(p)
    except FileNotFoundError:
        pass


# --------------------------------------------------------------------------- CLI

def find_mcaps(roots: List[str]) -> List[str]:
    out = []
    for r in roots:
        if os.path.isfile(r):
            out.append(os.path.abspath(r))
            continue
        for d, _, files in os.walk(r):
            out += [os.path.join(d, n) for n in files if n.endswith(".mcap")]
    return sorted(set(out))


def stale_partials(roots: List[str], now: Optional[float] = None) -> List[str]:
    now = time.time() if now is None else now
    out = []
    for r in roots:
        d0 = os.path.dirname(r) if os.path.isfile(r) else r
        for d, _, files in os.walk(d0):
            for n in files:
                p = os.path.join(d, n)
                if n.endswith(PARTIAL_SUFFIX) and now - os.path.getmtime(p) > STALE_PARTIAL_S:
                    out.append(p)
    return out


def live_recorder() -> Optional[str]:
    """A running ``ros2 bag record`` (argv contains ``bag`` then ``record`` as separate
    arguments, so a shell command line merely mentioning it does not count)."""
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            with open("/proc/%s/cmdline" % pid, "rb") as f:
                argv = f.read().split(b"\0")
        except OSError:
            continue
        if any(argv[i] == b"bag" and argv[i + 1] == b"record" for i in range(len(argv) - 1)):
            return "%s %s" % (pid, b" ".join(argv).decode(errors="replace")[:120])
    return None


def _fmt(r: Dict) -> str:
    b = r["before"] / 1e6
    a = r.get("after")
    ratio = (r["before"] / a) if a else None
    secs = r.get("transcode_s", 0) + r.get("verify_s", 0)
    return "%-16s %9.1f -> %9s MB  %6s  %6.1fs  %s%s" % (
        r["action"], b, "%.1f" % (a / 1e6) if a else "-", "%.2fx" % ratio if ratio else "-",
        secs, r["path"], ("  [%s]" % r["reason"]) if r.get("reason") else "")


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--level", type=int, default=3)
    ap.add_argument("--min-age-hours", type=float, default=24.0)
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--dry-run", action="store_true", help="the default; listed for explicitness")
    ap.add_argument("--replace", action="store_true", help="transcode, verify and replace in place")
    ap.add_argument("--allow-live", action="store_true")
    ap.add_argument("--recover", action="store_true",
                    help="rebuild crashed (unindexed) recordings instead; skips everything else")
    ap.add_argument("--discard-original", action="store_true",
                    help="--recover --replace: do not keep <name>.mcap%s" % ORIG_SUFFIX)
    ap.add_argument("--write-metadata", action="store_true",
                    help="--recover: also write a rosbag2 metadata.yaml derived from the recovered summary")
    ap.add_argument("--report")
    ap.add_argument("--lock-file", default=DEFAULT_LOCK)
    a = ap.parse_args(argv)
    if a.dry_run and a.replace:
        ap.error("--dry-run and --replace are exclusive")
    if (a.discard_original or a.write_metadata) and not a.recover:
        ap.error("--discard-original and --write-metadata need --recover")
    live = live_recorder()
    if live and not a.allow_live:
        print("REFUSING: a recorder is running (%s); use --allow-live to override" % live)
        return 2
    lockf = open(a.lock_file, "w")
    try:
        fcntl.flock(lockf, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("REFUSING: another bag_compress run holds %s" % a.lock_file)
        return 2
    for p in stale_partials(a.paths):
        print("%s stale partial %s" % ("removing" if a.replace else "would remove", p))
        if a.replace:
            _unlink(p)
    files = find_mcaps(a.paths)
    opts = {"level": a.level, "min_age_hours": a.min_age_hours, "replace": a.replace,
            "recover": a.recover, "discard_original": a.discard_original,
            "write_metadata": a.write_metadata}
    t0 = time.time()
    results = []
    ex = ProcessPoolExecutor(a.workers) if a.workers > 1 and len(files) > 1 else None
    it = ex.map(process_file, files, [opts] * len(files)) if ex else (process_file(p, opts) for p in files)
    try:
        for r in it:
            results.append(r)
            print(_fmt(r), flush=True)
    finally:
        if ex:
            ex.shutdown()
    by: Dict[str, int] = {}
    for r in results:
        by[r["action"]] = by.get(r["action"], 0) + 1
    done = [r for r in results if r["action"] in ("compressed", "would-compress", "recovered")
            and r.get("after")]
    b = sum(r["before"] for r in done)
    af = sum(r["after"] for r in done)
    print("TOTAL %d files %s | %s: %.2f GB -> %.2f GB (%.2fx) in %.0fs%s" % (
        len(results), json.dumps(by, sort_keys=True), "replaced" if a.replace else "estimate",
        b / 1e9, af / 1e9, b / af if af else 0, time.time() - t0,
        "" if a.replace else "  [DRY RUN: nothing written]"))
    failed = [r for r in results if r["action"].startswith("FAILED")]
    if a.report:
        with open(a.report, "w") as f:
            json.dump({"results": results, "by_action": by, "replace": a.replace,
                       "wall_s": time.time() - t0}, f, indent=1)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
