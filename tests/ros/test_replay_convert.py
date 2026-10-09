"""The replay converter (ros_ws/gui/replay/convert.py) against the cache
contract in ros_ws/gui/replay/schema.py, on synthetic rosbag2-shaped MCAPs
from tests/ros/_replay_fixture.py.

Pins: chunk count / contiguity, exact round-trip of flattened columns,
skipped-topic census, overview bands/ticks/presence, the unindexed (killed
recording) path with its two-chunk reorder buffer, dropped_late, the CLI exit
codes, and chunk bounds.
"""
from __future__ import annotations

import gzip
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import msgpack
import pytest

from tests.ros._replay_fixture import write_bag

REPO = Path(__file__).resolve().parents[2]
GUI = REPO / "ros_ws" / "gui"
if str(GUI) not in sys.path:
    sys.path.insert(0, str(GUI))

from replay import schema  # noqa: E402
from replay.convert import convert  # noqa: E402

DURATION = 35.0


def _chunks(out_dir: Path, n: int):
    recs = []
    for i in range(n):
        raw = (out_dir / schema.chunk_name(i)).read_bytes()
        recs.append(msgpack.unpackb(gzip.decompress(raw), raw=False))
    return recs


def _rows(recs, topic):
    """All (t, {col: value}) rows of ``topic`` across chunks, in chunk order."""
    out = []
    for r in recs:
        tp = r["topics"].get(topic)
        if tp is None:
            continue
        for k in range(tp["n"]):
            out.append((tp["t"][k], {c: v[k] for c, v in tp["cols"].items()}))
    return out


@pytest.fixture(scope="module")
def indexed(tmp_path_factory):
    d = tmp_path_factory.mktemp("indexed")
    info = write_bag(d / "bag_0.mcap", DURATION, seed=1)
    seen = []
    m = convert(str(d / "bag_0.mcap"), str(d / "cache"), on_manifest=seen.append)
    return info, m, d / "cache", seen


@pytest.fixture(scope="module")
def unindexed(tmp_path_factory):
    d = tmp_path_factory.mktemp("unindexed")
    info = write_bag(d / "bag_0.mcap", DURATION, seed=2, unindexed=True, chunk_size=16 * 1024)
    seen = []
    m = convert(str(d / "bag_0.mcap"), str(d / "cache"), on_manifest=seen.append)
    return info, m, d / "cache", seen


# 1. chunk count, contiguity, status, range -------------------------------

def test_chunk_count_and_range(indexed):
    info, m, out, seen = indexed
    n = math.ceil(DURATION / schema.CHUNK_S)
    assert m["status"] == schema.STATUS_COMPLETE, m["error"]
    assert m["error"] is None
    assert m["chunks_done"] == m["chunks_total"] == len(m["chunks"]) == n
    assert [c["i"] for c in m["chunks"]] == list(range(n))
    for i in range(n):
        assert (out / schema.chunk_name(i)).is_file()
    assert not list(out.glob("*.tmp"))
    assert m["t0"] == pytest.approx(info["t0_ns"] / 1e9, abs=1e-6)
    assert m["t1"] == pytest.approx(info["t1_ns"] / 1e9, abs=1e-6)
    assert m["source"]["indexed"] is True
    # Indexed bags know chunks_total from the summary before the first chunk.
    assert seen[0]["status"] == schema.STATUS_CONVERTING
    assert seen[0]["chunks_total"] == n
    assert m["converter"].startswith("rosbags ")
    on_disk = json.loads((out / schema.MANIFEST).read_text())
    assert on_disk == m


# 2. round-trip -----------------------------------------------------------

def test_round_trip_robot_state_and_mocap(indexed):
    info, m, out, _ = indexed
    recs = _chunks(out, len(m["chunks"]))

    rs = _rows(recs, "/robot_state")
    want = info["topics"]["/robot_state"]
    assert len(rs) == len(want)
    for (t, row), (t_ns, plain) in zip(rs, want):
        assert t == pytest.approx(t_ns / 1e9, abs=1e-6)
        assert row["is_homed"] is plain["is_homed"]
        assert row["has_undervoltage"] is plain["has_undervoltage"]
        assert row["timestamp.sec"] == plain["timestamp"]["sec"]
        assert row["timestamp.nanosec"] == plain["timestamp"]["nanosec"]
        assert row["pose_offset_quat.w"] == plain["pose_offset_quat"]["w"]
        assert row["pose_offset_rad"] == plain["pose_offset_rad"]
        assert row["error"] == plain["error"]
        assert row["motor_states"] == plain["motor_states"]

    mc = _rows(recs, "/mocap_data")
    want = info["topics"]["/mocap_data"]
    assert len(mc) == len(want)
    for (t, row), (t_ns, plain) in zip(mc, want):
        assert t == pytest.approx(t_ns / 1e9, abs=1e-6)
        assert row["markers"] == plain["markers"]
        assert row["aligned"] is True
        assert row["stamp.sec"] == plain["stamp"]["sec"]


def test_column_order_and_shape(indexed):
    _, m, out, _ = indexed
    recs = _chunks(out, len(m["chunks"]))
    tp = recs[0]["topics"]["/robot_state"]
    assert tp["type"] == "jugglebot_interfaces/msg/RobotState"
    assert list(tp["cols"])[:4] == ["timestamp.sec", "timestamp.nanosec", "motor_states", "error"]
    assert "pose_offset_quat.x" in tp["cols"]
    hand = recs[0]["topics"]["/hand_telemetry"]["cols"]
    assert "ball_held_stamp.nanosec" in hand


def test_diagnostic_status_columns_are_fields_only(indexed):
    _, m, out, _ = indexed
    seen = 0
    for rec in _chunks(out, len(m["chunks"])):
        for topic, tp in rec["topics"].items():
            if tp["type"] == "diagnostic_msgs/msg/DiagnosticStatus":
                seen += 1
                assert list(tp["cols"]) == ["level", "name", "message", "hardware_id", "values"], topic
                for c in ("OK", "WARN", "ERROR", "STALE"):
                    assert c not in tp["cols"], (topic, c)
    assert seen > 0


def test_chunk_sorting_and_census(indexed):
    _, m, out, _ = indexed
    recs = _chunks(out, len(m["chunks"]))
    for rec, entry in zip(recs, m["chunks"]):
        assert rec["format"] == schema.FORMAT_VERSION
        assert rec["i"] == entry["i"]
        assert entry["topics"] == {tp: v["n"] for tp, v in rec["topics"].items()}
        assert entry["n"] == sum(entry["topics"].values())
        for tp, v in rec["topics"].items():
            assert v["n"] >= 1
            assert v["t"] == sorted(v["t"])
            assert all(rec["t0"] <= t < rec["t1"] for t in v["t"])
            assert all(len(col) == v["n"] for col in v["cols"].values())
    totals = {}
    for entry in m["chunks"]:
        for tp, n in entry["topics"].items():
            totals[tp] = totals.get(tp, 0) + n
    assert totals == {tp: v["count"] for tp, v in m["topics"].items()}


# 3. skipped topic --------------------------------------------------------

def test_skipped_topic(indexed):
    info, m, out, _ = indexed
    assert m["skipped_topics"] == {
        "/bb/axis_estimates": {"type": "std_msgs/msg/Float64MultiArray",
                               "count": len(info["topics"]["/bb/axis_estimates"])}}
    assert "/bb/axis_estimates" not in m["topics"]
    for rec in _chunks(out, len(m["chunks"])):
        assert "/bb/axis_estimates" not in rec["topics"]


# 4. overview -------------------------------------------------------------

def test_overview(indexed):
    info, m, out, _ = indexed
    ov = json.loads((out / schema.OVERVIEW).read_text())
    t0 = info["t0_ns"] / 1e9
    assert ov["t0"] == m["t0"] and ov["t1"] == m["t1"]
    (band,) = ov["bands"]
    assert band["topic"] == "/orchestrator_state"
    segs = band["segments"]
    assert [s[2] for s in segs] == ["IDLE", "LEVELLING", "ACTIVE"]
    period = 0.1
    assert segs[0][1] - t0 == pytest.approx(4.0, abs=period)
    assert segs[1][0] == segs[0][1]
    assert segs[1][1] - t0 == pytest.approx(9.0, abs=period)
    assert segs[2][1] == m["t1"]

    by_kind = {}
    for tk in ov["ticks"]:
        by_kind.setdefault(tk["kind"], []).append(tk)
        assert tk["kind"] in schema.OVERVIEW_TICK_KINDS
    assert [tk["t"] for tk in ov["ticks"]] == sorted(tk["t"] for tk in ov["ticks"])
    rs_period = 0.01
    for kind, at in (("homed", 5.0), ("levelled", 8.0), ("fault", 20.0), ("fault_cleared", 22.0)):
        assert len(by_kind[kind]) == 1, kind
        assert by_kind[kind][0]["t"] - t0 == pytest.approx(at, abs=rs_period), kind
    assert len(by_kind["catch_event"]) == 2
    assert len(by_kind["skill_attempt"]) == 3
    # bb_calibration ticks come from /bb/calibration_attempt (every sweep,
    # success and failure); the one /bb/calibration_result adds none.
    assert [tk["label"] for tk in by_kind["bb_calibration"]] == ["ok", "failed", "ok"]
    assert [tk["t"] - t0 for tk in by_kind["bb_calibration"]] == pytest.approx([7.5, 15.2, 30.1], abs=0.01)
    # label = .message, or .name when the message is empty (attempt 1).
    assert [tk["label"] for tk in by_kind["skill_attempt"]] == [
        "attempt 0 CAUGHT", "skill_1", "attempt 2 CAUGHT"]

    # /cone/catch_event lands at 13.42 s and 25.18 s -> chunks 1 and 2 only.
    assert ov["presence"]["/cone/catch_event"] == [[t0 + 10.0, t0 + 30.0]]
    assert ov["presence"]["/robot_state"] == [[t0, m["t1"]]]
    assert "/bb/axis_estimates" not in ov["presence"]


# 5. unindexed (killed recording) ----------------------------------------

def test_unindexed(unindexed):
    info, m, out, seen = unindexed
    assert m["status"] == schema.STATUS_COMPLETE, m["error"]
    assert m["source"]["indexed"] is False
    assert seen[0]["chunks_total"] is None
    assert m["chunks_total"] == len(m["chunks"]) == m["chunks_done"]
    assert m["chunks_total"] >= 3
    assert m["dropped_late"] == 0
    assert m["t1"] == pytest.approx(info["t1_ns"] / 1e9, abs=1e-6)
    recs = _chunks(out, len(m["chunks"]))
    for tp, want in info["topics"].items():
        if tp == "/bb/axis_estimates":
            continue
        got = _rows(recs, tp)
        assert [round(t, 6) for t, _ in got] == [round(t_ns / 1e9, 6) for t_ns, _ in want], tp
    # The message written 0.3 s late in file order lands in its own chunk.
    t_ns, topic = info["reordered"]
    t = t_ns / 1e9
    i = int((t - m["t0"]) // schema.CHUNK_S)
    assert any(abs(x - t) < 1e-6 for x in recs[i]["topics"][topic]["t"])
    assert recs[i]["topics"][topic]["t"] == sorted(recs[i]["topics"][topic]["t"])


# 6. late message --------------------------------------------------------

def test_late_message_dropped(tmp_path):
    info = write_bag(tmp_path / "bag_0.mcap", DURATION, seed=3, late_message=True,
                     chunk_size=16 * 1024)
    m = convert(str(tmp_path / "bag_0.mcap"), str(tmp_path / "cache"))
    assert m["status"] == schema.STATUS_COMPLETE, m["error"]
    assert m["dropped_late"] == 1
    t_ns, topic = info["late"]
    for rec in _chunks(tmp_path / "cache", len(m["chunks"])):
        tp = rec["topics"].get(topic)
        if tp:
            assert all(abs(x - t_ns / 1e9) > 1e-6 for x in tp["t"])
    n_written = len(info["topics"][topic]) - 1
    assert m["topics"][topic]["count"] == n_written


# 7. CLI -----------------------------------------------------------------

def test_cli_complete_and_failed(tmp_path):
    write_bag(tmp_path / "bag_0.mcap", 12.0, seed=4)
    env = dict(os.environ)
    r = subprocess.run([sys.executable, "-m", "replay.convert", "--bag",
                        str(tmp_path / "bag_0.mcap"), "--out", str(tmp_path / "ok")],
                       cwd=str(GUI), capture_output=True, text=True, env=env)
    assert r.returncode == 0, r.stderr
    assert r.stdout == ""
    m = json.loads((tmp_path / "ok" / schema.MANIFEST).read_text())
    assert m["status"] == schema.STATUS_COMPLETE
    assert m["chunks_done"] == 2

    r = subprocess.run([sys.executable, "-m", "replay.convert", "--bag",
                        str(tmp_path / "missing.mcap"), "--out", str(tmp_path / "bad")],
                       cwd=str(GUI), capture_output=True, text=True, env=env)
    assert r.returncode == 1
    m = json.loads((tmp_path / "bad" / schema.MANIFEST).read_text())
    assert m["status"] == schema.STATUS_FAILED
    assert m["error"]


def test_cli_bulk_skips_complete(tmp_path):
    root = tmp_path / "rosbags"
    for name in ("2026-10-01_10-00-00", "2026-10-02_10-00-00"):
        (root / name).mkdir(parents=True)
        write_bag(root / name / (name + "_0.mcap"), 11.0, seed=5)
    (root / "not-a-recording").mkdir()
    cache = tmp_path / "cache"
    args = [sys.executable, "-m", "replay.convert", "--bulk", str(root),
            "--cache-root", str(cache), "--newest", "1"]
    r = subprocess.run(args, cwd=str(GUI), capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert sorted(p.name for p in cache.iterdir()) == ["2026-10-02_10-00-00"]
    m = json.loads((cache / "2026-10-02_10-00-00" / schema.MANIFEST).read_text())
    assert m["recording"] == "2026-10-02_10-00-00" and m["status"] == schema.STATUS_COMPLETE
    r = subprocess.run(args, cwd=str(GUI), capture_output=True, text=True)
    assert r.returncode == 0 and "already complete" in r.stderr
    # An old-format complete cache is NOT skipped: it is reconverted.
    mp = cache / "2026-10-02_10-00-00" / schema.MANIFEST
    m = json.loads(mp.read_text())
    m["format"] = schema.FORMAT_VERSION - 1
    mp.write_text(json.dumps(m))
    r = subprocess.run(args, cwd=str(GUI), capture_output=True, text=True)
    assert r.returncode == 0 and "converting 2026-10-02_10-00-00" in r.stderr
    assert json.loads(mp.read_text())["format"] == schema.FORMAT_VERSION


# 8. chunk bounds --------------------------------------------------------

def test_chunk_bounds(indexed, unindexed):
    for _, m, out, _ in (indexed, unindexed):
        recs = _chunks(out, len(m["chunks"]))
        for i, (entry, rec) in enumerate(zip(m["chunks"], recs)):
            lo, hi = schema.chunk_bounds(i, m["t0"])
            assert entry["t0"] == pytest.approx(lo) and entry["t1"] == pytest.approx(hi)
            assert rec["t0"] == entry["t0"] and rec["t1"] == entry["t1"]


# 9. field enumeration vs the .msg definitions -------------------------------

def _msg_field_names(text: str):
    """Field names of one .msg body: 'type name' lines, constants (with '=') excluded."""
    out = []
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or "=" in line:
            continue
        out.append(line.split()[1])
    return out


def test_field_names_match_msg_definitions():
    """``_field_names`` drops dataclass fields that carry a default (IDL
    constants).  A real field that ever gains a default would be silently
    dropped from the chunks; pin the enumeration to the .msg definitions."""
    from replay.convert import _field_names
    from tests.ros._replay_fixture import MSG_DIR, SEP, _full_name, make_typestore

    ts = make_typestore()
    top = ["jugglebot_interfaces/msg/RobotState", "jugglebot_interfaces/msg/HandTelemetryMessage",
           "jugglebot_interfaces/msg/MocapDataMulti", "jugglebot_interfaces/msg/CatchEvent",
           "std_msgs/msg/String", "diagnostic_msgs/msg/DiagnosticStatus"]
    names = set(top)
    for t in top:  # nested dependency closure
        gen, _ = ts.generate_msgdef(t, ros_version=2)
        for blk in gen.split(SEP)[1:]:
            names.add(_full_name(blk.strip().splitlines()[0].split("MSG:", 1)[1].strip()))
    checked = 0
    for name in sorted(names):
        if name.startswith("jugglebot_interfaces/msg/"):
            f = MSG_DIR / (name.rsplit("/", 1)[1] + ".msg")
            expected = _msg_field_names(f.read_text())
        else:
            gen, _ = ts.generate_msgdef(name, ros_version=2)
            expected = _msg_field_names(gen.split(SEP)[0])
        assert _field_names(ts.types[name]) == expected, name
        checked += 1
    assert checked >= len(top)
