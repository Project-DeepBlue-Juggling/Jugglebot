"""tools/bag_compress.py: lossless in-place zstd recompression of rosbag2 MCAP files.

Content equality is the contract (schemas, channels and every message, in file order),
plus statistics, the indexed (seeking) read path and the replace-only-after-verify rule.
"""
from __future__ import annotations

import os
import sys
import time

import pytest
from mcap.reader import NonSeekingReader, make_reader
from mcap.writer import CompressionType, Writer

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import bag_compress as bc  # noqa: E402

OLD = time.time() - 3 * 86400


def make_bag(path, compression=CompressionType.NONE, crcs=True, data_crcs=False, extras=False):
    """Multi-chunk, multi-channel bag: an empty channel, sparse sequence numbers,
    publish_time != log_time, compressible payloads of varying size."""
    with open(path, "wb") as f:
        w = Writer(f, chunk_size=2048, compression=compression, enable_crcs=crcs,
                   enable_data_crcs=data_crcs)
        w.start(profile="ros2", library="test")
        s1 = w.register_schema("std_msgs/msg/Float64", "ros2msg", b"float64 data")
        s2 = w.register_schema("a/msg/B", "ros2msg", b"int32 x\nstring y")
        chans = [w.register_channel("/a", "cdr", s1, {"offered_qos_profiles": "q"}),
                 w.register_channel("/b", "cdr", s2),
                 w.register_channel("/never", "cdr", s2)]
        for i in range(900):
            ch = chans[i % 2]
            data = (b"payload-%05d-" % i) * (1 + i % 7)
            w.add_message(ch, log_time=1_000_000 + i * 997, publish_time=999_000 + i * 991,
                          data=data, sequence=3 * i + (i % 5))
            if extras and i == 450:
                w.add_attachment(create_time=5, log_time=6, name="cal.json",
                                 media_type="application/json", data=b'{"k": 1}' * 50)
                w.add_metadata("rig", {"hand": "robot", "fw": "52"})
        w.finish()
    os.utime(path, (OLD, OLD))
    return path


def run(paths, tmp_path, *extra):
    return bc.main([str(p) for p in paths] + ["--workers", "1", "--allow-live",
                                              "--lock-file", str(tmp_path / "lock")] + list(extra))


def replace(paths, tmp_path):
    return run(paths, tmp_path, "--replace", "--min-age-hours", "0")


def read_all(path):
    with open(path, "rb") as f:
        r = make_reader(f, validate_crcs=True)
        msgs = [(m.channel_id, m.sequence, m.log_time, m.publish_time, m.data)
                for _, _, m in r.iter_messages(log_time_order=False)]
        return r.get_summary(), msgs, list(r.iter_attachments()), list(r.iter_metadata())


@pytest.mark.parametrize("crcs,data_crcs,extras", [(True, False, False), (False, False, False),
                                                   (True, True, True)])
def test_round_trip_content_stats_and_indexes(tmp_path, crcs, data_crcs, extras):
    p = make_bag(tmp_path / "b_0.mcap", crcs=crcs, data_crcs=data_crcs, extras=extras)
    st0 = os.stat(p)
    digest0 = bc.content_digest(str(p))
    s0, m0, a0, md0 = read_all(p)
    assert len(s0.chunk_indexes) > 20
    assert replace([p], tmp_path) == 0
    st1 = os.stat(p)
    assert st1.st_size < st0.st_size
    assert (st1.st_mtime_ns, st1.st_mode) == (st0.st_mtime_ns, st0.st_mode)
    assert bc.content_digest(str(p)) == digest0
    s1, m1, a1, md1 = read_all(p)
    assert s1.statistics == s0.statistics and s1.channels == s0.channels
    assert {c.compression for c in s1.chunk_indexes} == {"zstd"}
    assert (m1, a1, md1) == (m0, a0, md0)
    assert len(a1) == extras and len(md1) == extras
    with open(p, "rb") as f:  # unindexed path, CRC-validated (incl. the data-section CRC)
        assert len(list(NonSeekingReader(f, validate_crcs=True).iter_messages())) == 900
    assert bc.summary_crc_ok(str(p))
    assert not [n for n in os.listdir(tmp_path) if n.endswith(bc.PARTIAL_SUFFIX)]


def test_second_run_is_a_noop(tmp_path):
    p = make_bag(tmp_path / "b_0.mcap")
    assert replace([tmp_path], tmp_path) == 0
    before = p.read_bytes()
    assert bc.process_file(str(p), {"level": 3, "min_age_hours": 0, "replace": True})["action"] == "skip-done"
    assert replace([tmp_path], tmp_path) == 0
    assert p.read_bytes() == before


def _flip_middle(path):
    with open(path, "r+b") as f:
        f.seek(os.path.getsize(path) // 2)
        b = f.read(1)
        f.seek(-1, 1)
        f.write(bytes([b[0] ^ 0xFF]))


@pytest.mark.parametrize("how", ["flip", "compressor"])
def test_corrupt_transcode_fails_and_leaves_original(tmp_path, monkeypatch, how):
    p = make_bag(tmp_path / "b_0.mcap", crcs=False)
    orig = p.read_bytes()
    if how == "flip":
        real = bc.transcode

        def bad(src, dst, level=3, compress=None):
            out = real(src, dst, level, compress)
            _flip_middle(dst)
            return out
        monkeypatch.setattr(bc, "transcode", bad)
    else:
        import zstandard
        cz = zstandard.ZstdCompressor()
        monkeypatch.setattr(bc.zstandard, "ZstdCompressor", lambda level: type(
            "C", (), {"compress": staticmethod(lambda b: cz.compress(b[:-1] + b"\x00"))}))
    assert replace([p], tmp_path) == 1
    assert p.read_bytes() == orig
    assert os.listdir(tmp_path) == ["b_0.mcap"] or set(os.listdir(tmp_path)) == {"b_0.mcap", "lock"}


def test_truncated_and_lz4_and_young_and_dry_run_change_nothing(tmp_path, monkeypatch):
    t = make_bag(tmp_path / "trunc_0.mcap")
    with open(t, "r+b") as f:
        f.truncate(os.path.getsize(t) * 6 // 10)
    os.utime(t, (OLD, OLD))
    lz = make_bag(tmp_path / "lz_0.mcap", compression=CompressionType.LZ4)
    young = make_bag(tmp_path / "young_0.mcap")
    os.utime(young, None)
    plain = make_bag(tmp_path / "plain_0.mcap")
    snap = {n: (tmp_path / n).read_bytes() for n in os.listdir(tmp_path)}
    mt = {n: os.stat(tmp_path / n).st_mtime_ns for n in snap}
    opts = {"level": 3, "min_age_hours": 24, "replace": True}
    assert bc.process_file(str(t), opts)["action"] == "skip-unindexed"
    assert bc.process_file(str(lz), opts)["action"] == "refused"
    assert bc.process_file(str(young), opts)["action"] == "skip-young"
    dry = bc.process_file(str(plain), dict(opts, replace=False))
    assert dry["action"] == "would-compress" and dry["est_ratio"] > 1
    assert run([tmp_path], tmp_path) == 0                  # default = dry run
    assert run([tmp_path], tmp_path, "--dry-run", "--min-age-hours", "0") == 0
    assert run([young], tmp_path, "--replace") == 0         # 24 h default keeps it
    monkeypatch.setattr(bc, "live_recorder", lambda: "42 ros2 bag record -s mcap")
    assert bc.main([str(plain), "--replace", "--min-age-hours", "0", "--workers", "1",
                    "--lock-file", str(tmp_path / "lock")]) == 2
    for n, b in snap.items():
        assert (tmp_path / n).read_bytes() == b and os.stat(tmp_path / n).st_mtime_ns == mt[n]


def test_stale_partials_cleaned_only_when_old(tmp_path):
    old = tmp_path / "x_0.mcap.zst-partial"
    new = tmp_path / "y_0.mcap.zst-partial"
    old.write_bytes(b"junk")
    new.write_bytes(b"junk")
    os.utime(old, (OLD, OLD))
    assert run([tmp_path], tmp_path) == 0          # dry run lists, keeps
    assert old.exists()
    assert replace([tmp_path], tmp_path) == 0
    assert not old.exists() and new.exists()


# --------------------------------------------------------------------------- --recover
# A Foxy recorder killed mid-write leaves Magic, Header, Chunk, MessageIndex... cut at an
# arbitrary byte: no DataEnd, summary or Footer. The python Writer lays chunks out the same
# way (chunk, then its MessageIndex records), so truncating a Writer file reproduces it.

import struct  # noqa: E402

import yaml  # noqa: E402


def top_records(path):
    """[(opcode, start, end)] for the top-level records of a complete file."""
    out = []
    with open(path, "rb") as f:
        data = f.read()
    o = 8
    while o < len(data) - 8:
        op, ln = data[o], struct.unpack_from("<Q", data, o + 1)[0]
        out.append((op, o, o + 9 + ln))
        o += 9 + ln
    return out


def file_order_msgs(path):
    with open(path, "rb") as f:
        return [(m.channel_id, m.sequence, m.log_time, m.publish_time, m.data)
                for _, _, m in make_reader(f, validate_crcs=True).iter_messages(log_time_order=False)]


def payload_msgs_before(path, chunk_start, cut):
    """Messages of the (uncompressed) chunk at chunk_start whose records end before cut."""
    with open(path, "rb") as f:
        data = f.read()
    p = chunk_start + 9 + 8 * 3 + 4 + 4 + 8            # header fields, "" compression, records len
    end = chunk_start + 9 + struct.unpack_from("<Q", data, chunk_start + 1)[0]
    n = 0
    while p < end:
        op, ln = data[p], struct.unpack_from("<Q", data, p + 1)[0]
        if p + 9 + ln > cut:
            break
        n += op == 0x05
        p += 9 + ln
    return n


def msgs_in_first_chunks(full, k):
    from mcap.records import Chunk, Message
    from mcap.stream_reader import StreamReader, breakup_chunk
    n = seen = 0
    with open(full, "rb") as f:
        for rec in StreamReader(f, emit_chunks=True).records:
            if isinstance(rec, Chunk):
                if seen == k:
                    break
                n += sum(isinstance(r, Message) for r in breakup_chunk(rec))
                seen += 1
    return n


def cut_at(full, dst, nbytes):
    with open(full, "rb") as f:
        dst.write_bytes(f.read(nbytes))
    os.utime(dst, (OLD, OLD))
    return dst


def recover_run(paths, tmp_path, *extra):
    return run(paths, tmp_path, "--recover", "--replace", "--min-age-hours", "0", *extra)


def _chunks(recs):
    return [(s, e) for op, s, e in recs if op == 0x06]


def _case_cut(full, case):
    """(cut offset, number of complete chunks, salvaged messages expected)."""
    recs = top_records(full)
    ch = _chunks(recs)
    k = 5
    after_mi = next(s for op, s, e in recs if op == 0x06 and s > ch[k - 1][0])  # start of chunk k+1
    if case == "after_mis":
        return after_mi, k, 0
    if case == "chunk_then_cut":                       # between chunk k and its MessageIndexes
        return ch[k - 1][1], k, 0
    if case == "mid_mi":                               # the real-world shape (all 35 files)
        first_mi_end = next(e for op, s, e in recs if op == 0x07 and s == ch[k - 1][1])
        return first_mi_end + 9 + 5, k, 0
    if case == "mid_chunk":
        s, e = ch[k]
        cut = s + (e - s) * 2 // 3
        return cut, k, payload_msgs_before(full, s, cut)
    if case == "no_summary":                           # all chunks, cut at DataEnd
        de = next(s for op, s, e in recs if op == 0x0F)
        return de, len(ch), 0
    if case == "after_data_end":                       # DataEnd present, summary/footer missing
        de = next(e for op, s, e in recs if op == 0x0F)
        return de, len(ch), 0
    if case == "first_chunk_cut":                      # zero complete chunks, salvage only
        s, e = ch[0]
        cut = s + (e - s) // 2
        return cut, 0, payload_msgs_before(full, s, cut)
    raise AssertionError(case)


@pytest.mark.parametrize("crcs", [False, True])
@pytest.mark.parametrize("case", ["after_mis", "chunk_then_cut", "mid_mi", "mid_chunk", "no_summary",
                                  "after_data_end", "first_chunk_cut"])
def test_recover_truncations(tmp_path, case, crcs):
    full = make_bag(tmp_path / "full.mcap", crcs=crcs)
    allm = file_order_msgs(full)
    cut, k, salv = _case_cut(full, case)
    work = tmp_path / "rec"
    work.mkdir()
    p = cut_at(full, work / "r_0.mcap", cut)
    orig = p.read_bytes()
    st0 = os.stat(p)
    assert bc.probe(str(p))["category"] == "unindexed"
    info = bc.recover(str(p))                             # dry characterisation
    assert p.read_bytes() == orig
    n_complete = msgs_in_first_chunks(full, k)
    assert info["chunks"] == k + (1 if salv else 0)
    assert info["salvaged_messages"] == salv
    assert info["messages"] == n_complete + salv
    if case in ("mid_chunk", "first_chunk_cut"):
        assert salv > 0 and info["salvage"]["why"] == ""
    if case == "chunk_then_cut":
        assert info["mi_none"] == 1
    if case == "mid_mi":
        assert info["mi_partial"] == 1 and info["end"].startswith("mid MessageIndex")

    assert recover_run([work], tmp_path) == 0
    kept = work / ("r_0.mcap" + bc.ORIG_SUFFIX)
    assert kept.read_bytes() == orig and os.stat(kept).st_mtime_ns == st0.st_mtime_ns
    st1 = os.stat(p)
    assert (st1.st_mtime_ns, st1.st_mode) == (st0.st_mtime_ns, st0.st_mode)
    assert bc.probe(str(p))["category"] == "already" and bc.summary_crc_ok(str(p))
    got = file_order_msgs(p)
    assert got == allm[:n_complete + salv]               # exactly the pre-cut prefix, no more
    with open(p, "rb") as f:
        r = make_reader(f, validate_crcs=True)
        s = r.get_summary()
        assert s.statistics.message_count == len(got)
        assert s.statistics.chunk_count == len(s.chunk_indexes) == k + (1 if salv else 0)
        assert sorted(m.log_time for _, _, m in r.iter_messages(log_time_order=True)) == \
            sorted(m[2] for m in got)
    with open(p, "rb") as f:                              # streaming path incl. data-section CRC
        assert len(list(NonSeekingReader(f, validate_crcs=True).iter_messages())) == len(got)
    assert bc.verify_recovered(str(kept), str(p), info)["messages"] == len(got)
    assert sorted(os.listdir(work)) == ["r_0.mcap", "r_0.mcap" + bc.ORIG_SUFFIX]
    snap = p.read_bytes()                                 # idempotent
    assert recover_run([work], tmp_path) == 0
    assert p.read_bytes() == snap and kept.read_bytes() == orig


@pytest.mark.parametrize("where", ["header", "first_chunk_header"])
def test_recover_nothing_to_recover_leaves_file(tmp_path, where):
    full = make_bag(tmp_path / "full.mcap")
    first = _chunks(top_records(full))[0][0]
    work = tmp_path / "rec"
    work.mkdir()
    p = cut_at(full, work / "r_0.mcap", 8 + 5 if where == "header" else first + 9 + 20)
    orig, mt = p.read_bytes(), os.stat(p).st_mtime_ns
    with pytest.raises(bc.NothingToRecover):
        bc.recover(str(p))
    assert recover_run([work], tmp_path) == 0
    r = bc.process_file(str(p), {"level": 3, "min_age_hours": 0, "replace": True, "recover": True})
    assert r["action"] == "skip-empty"
    assert os.listdir(work) == ["r_0.mcap"]
    assert p.read_bytes() == orig and os.stat(p).st_mtime_ns == mt


def test_recover_discard_original_and_scope(tmp_path):
    """--discard-original keeps no copy; --recover touches only unindexed files; a normal
    (compress) run still skips unindexed files; the dry run writes nothing."""
    full = make_bag(tmp_path / "full.mcap")
    cut = _case_cut(full, "mid_mi")[0]
    d = tmp_path / "bags"
    d.mkdir()
    crashed = cut_at(full, d / "c_0.mcap", cut)
    plain = make_bag(d / "p_0.mcap")                           # indexed, uncompressed
    zst = make_bag(d / "z_0.mcap", compression=CompressionType.ZSTD)
    young = cut_at(full, d / "y_0.mcap", cut)
    os.utime(young, None)
    snap = {n: ((d / n).read_bytes(), os.stat(d / n).st_mtime_ns) for n in os.listdir(d)}

    def unchanged(names):
        for n in names:
            assert ((d / n).read_bytes(), os.stat(d / n).st_mtime_ns) == snap[n]

    opts = {"level": 3, "min_age_hours": 0, "replace": True}
    assert bc.process_file(str(crashed), opts)["action"] == "skip-unindexed"   # normal run
    assert run([d], tmp_path, "--recover") == 0                               # dry run
    unchanged(snap)
    ro = dict(opts, recover=True)
    assert bc.process_file(str(plain), ro)["action"] == "skip-not-crashed"
    assert bc.process_file(str(zst), ro)["action"] == "skip-indexed"
    assert bc.process_file(str(young), dict(ro, min_age_hours=24))["action"] == "skip-young"
    assert recover_run([crashed], tmp_path, "--discard-original") == 0
    assert bc.probe(str(crashed))["category"] == "already"
    assert sorted(os.listdir(d)) == ["c_0.mcap", "p_0.mcap", "y_0.mcap", "z_0.mcap"]
    unchanged(["p_0.mcap", "y_0.mcap", "z_0.mcap"])
    with pytest.raises(SystemExit):
        bc.main([str(d), "--write-metadata"])                                  # needs --recover


@pytest.mark.parametrize("how", ["flip", "drop_message"])
def test_recover_verify_failure_leaves_directory(tmp_path, monkeypatch, how):
    full = make_bag(tmp_path / "full.mcap", crcs=False)
    work = tmp_path / "rec"
    work.mkdir()
    p = cut_at(full, work / "r_0.mcap", _case_cut(full, "mid_chunk")[0])
    orig, mt = p.read_bytes(), os.stat(p).st_mtime_ns
    real = bc.recover

    def bad(src, dst=None, level=3, compress=None):
        if dst is None or how == "flip":
            info = real(src, dst, level, compress)
            if dst is not None:
                _flip_middle(dst)
            return info
        # a self-consistent output that silently loses the last salvaged message
        info = real(src, dst, level, compress)
        info["salvaged_messages"] += 1
        return info
    monkeypatch.setattr(bc, "recover", bad)
    assert recover_run([work], tmp_path) == 1
    assert os.listdir(work) == ["r_0.mcap"]
    assert p.read_bytes() == orig and os.stat(p).st_mtime_ns == mt


def test_verify_recovered_rejects_a_different_original(tmp_path):
    a = make_bag(tmp_path / "a.mcap")
    cut = _case_cut(a, "after_mis")[0]
    work = tmp_path / "rec"
    work.mkdir()
    p = cut_at(a, work / "r_0.mcap", cut)
    other = cut_at(a, tmp_path / "other.mcap", _case_cut(a, "no_summary")[0])
    info = bc.recover(str(p))
    assert recover_run([work], tmp_path) == 0
    bc.verify_recovered(str(work / ("r_0.mcap" + bc.ORIG_SUFFIX)), str(p), info)
    with pytest.raises(bc.VerifyError):
        bc.verify_recovered(str(other), str(p), info)


def test_recover_write_metadata_matches_foxy_shape_and_gui_reads_it(tmp_path):
    full = make_bag(tmp_path / "full.mcap")
    d = tmp_path / "2026-11-25_08-07-35"
    d.mkdir()
    p = cut_at(full, d / "2026-11-25_08-07-35_0.mcap", _case_cut(full, "mid_chunk")[0])
    assert recover_run([d], tmp_path, "--write-metadata") == 0
    meta = yaml.safe_load((d / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    assert set(meta) == {"version", "storage_identifier", "relative_file_paths", "duration",
                         "starting_time", "message_count", "topics_with_message_count",
                         "compression_format", "compression_mode"}
    with open(p, "rb") as f:
        s = make_reader(f).get_summary()
    st = s.statistics
    assert meta["version"] == 4 and meta["storage_identifier"] == "mcap"
    assert meta["relative_file_paths"] == [p.name]
    assert meta["message_count"] == st.message_count
    assert meta["starting_time"]["nanoseconds_since_epoch"] == st.message_start_time
    assert meta["duration"]["nanoseconds"] == st.message_end_time - st.message_start_time
    topics = {t["topic_metadata"]["name"]: t for t in meta["topics_with_message_count"]}
    assert set(topics) == {"/a", "/b", "/never"}
    assert topics["/a"]["topic_metadata"] == {"name": "/a", "type": "std_msgs/msg/Float64",
                                              "serialization_format": "cdr",
                                              "offered_qos_profiles": "q"}
    assert topics["/never"]["message_count"] == 0
    assert sum(t["message_count"] for t in topics.values()) == st.message_count
    sys.path.insert(0, os.path.join(ROOT, "ros_ws", "gui"))
    try:
        from replay import recordings
        info = recordings.find_recording(str(tmp_path), d.name)
    finally:
        sys.path.remove(os.path.join(ROOT, "ros_ws", "gui"))
    assert info["indexed"] and info["closed"]
    assert info["message_count"] == st.message_count
    assert info["start_ns"] == st.message_start_time
    before = (d / "metadata.yaml").read_bytes()               # never overwritten
    assert recover_run([d], tmp_path, "--write-metadata") == 0
    assert (d / "metadata.yaml").read_bytes() == before
