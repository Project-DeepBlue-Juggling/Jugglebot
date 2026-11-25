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

Usage:
    python tools/bag_compress.py ~/Desktop/rosbags                      # dry run + estimates
    nice -n 15 python tools/bag_compress.py ~/Desktop/rosbags --replace --report temp/bagz.json
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


# --------------------------------------------------------------------------- verify
# Everything below reads files through the installed ``mcap`` library (or, for the summary
# CRC, a ten-line reader of the footer), never through transcode()'s parser.

def content_digest(path: str) -> Dict:
    """SHA-256 over schemas, channels and messages in file order, via mcap's StreamReader
    with CRC validation (chunk CRCs and, when non-zero, the data-section CRC). Streaming:
    memory is bounded by one chunk."""
    from mcap.records import Channel, DataEnd, Message, Schema
    from mcap.stream_reader import StreamReader

    h = hashlib.sha256()
    counts: Dict[int, int] = {}
    n = 0
    with open(path, "rb") as f:
        for r in StreamReader(f, validate_crcs=True).records:
            if isinstance(r, DataEnd):
                break
            if isinstance(r, Message):
                h.update(struct.pack("<BHIQQQ", 5, r.channel_id, r.sequence, r.log_time,
                                     r.publish_time, len(r.data)))
                h.update(r.data)
                counts[r.channel_id] = counts.get(r.channel_id, 0) + 1
                n += 1
            elif isinstance(r, Schema):
                h.update(repr((3, r.id, r.name, r.encoding, len(r.data))).encode())
                h.update(r.data)
            elif isinstance(r, Channel):
                h.update(repr((4, r.id, r.schema_id, r.topic, r.message_encoding,
                               sorted(r.metadata.items()))).encode())
    return {"sha256": h.hexdigest(), "messages": n, "counts": counts}


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


def indexed_check(orig: str, new: str) -> Dict:
    """The seeking path: summaries, every ChunkIndex -> Chunk (CRC), every MessageIndex
    entry -> a Message at that offset, first/last messages via iter_messages, attachments
    and metadata via their indexes."""
    from mcap.reader import make_reader
    from mcap.data_stream import ReadDataStream
    from mcap.records import Chunk, MessageIndex
    from mcap.stream_reader import get_chunk_data_stream

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
        n_idx = 0
        for ci in sb.chunk_indexes:
            if ci.compression != "zstd":
                raise VerifyError("chunk at %d is %r" % (ci.chunk_start_offset, ci.compression))
            fb.seek(ci.chunk_start_offset)
            h = fb.read(9)
            if h[0] != 0x06 or struct.unpack("<Q", h[1:])[0] != ci.chunk_length - 9:
                raise VerifyError("ChunkIndex -> %d is not a Chunk of its length" % ci.chunk_start_offset)
            chunk = Chunk.read(ReadDataStream(fb))
            if (chunk.compression != "zstd" or len(chunk.data) != ci.compressed_size
                    or chunk.uncompressed_size != ci.uncompressed_size or chunk.uncompressed_crc == 0
                    or chunk.message_start_time != ci.message_start_time):
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
    ap.add_argument("--report")
    ap.add_argument("--lock-file", default=DEFAULT_LOCK)
    a = ap.parse_args(argv)
    if a.dry_run and a.replace:
        ap.error("--dry-run and --replace are exclusive")
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
    opts = {"level": a.level, "min_age_hours": a.min_age_hours, "replace": a.replace}
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
    done = [r for r in results if r["action"] in ("compressed", "would-compress") and r.get("after")]
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
