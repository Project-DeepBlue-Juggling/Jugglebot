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
