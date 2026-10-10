"""Replay API tests: the listing route (design 06 § 1), real gui_server on an ephemeral port."""
from __future__ import annotations

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from _replay_srv import Srv, make_bag  # noqa: E402
from replay import recordings, schema  # noqa: E402

A, B, C = "2026-10-01_10-00-00", "2026-10-02_10-00-00", "2026-10-03_10-00-00"

META = """rosbag2_bagfile_information:
  version: 5
  storage_identifier: mcap
  duration:
    nanoseconds: 12500000000
  starting_time:
    nanoseconds_since_epoch: 1700000000000000000
  message_count: 4242
  topics_with_message_count:
    - topic_metadata:
        name: /robot_state
        type: jugglebot/msg/RobotState
        serialization_format: cdr
        offered_qos_profiles: ""
      message_count: 1200
    - topic_metadata:
        name: /not/allowlisted
        type: std_msgs/msg/String
        serialization_format: cdr
        offered_qos_profiles: ""
      message_count: 77
    - topic_metadata:
        name: /orchestrator_state
        type: jugglebot/msg/OrchestratorState
        serialization_format: cdr
        offered_qos_profiles: ""
      message_count: 0
  compression_format: ""
"""

ROW_KEYS = {"id", "size_bytes", "mtime", "closed", "indexed", "in_progress", "duration_s",
            "message_count", "start_ns", "topics", "overview"}


@pytest.fixture
def srv(tmp_path):
    s = Srv(tmp_path)
    yield s
    s.close()


def test_listing_shape_order_exclusions(srv):
    make_bag(srv.root, A, metadata=META)
    make_bag(srv.root, B)
    make_bag(srv.root, C, kind="killed", age_s=1.0)
    make_bag(srv.root, "not-a-recording")
    d = os.path.join(srv.root, "2026-10-04_10-00-00")
    make_bag(srv.root, "2026-10-04_10-00-00")
    open(os.path.join(d, "second.mcap"), "wb").close()
    code, h, body = srv.req("/api/replay/recordings")
    assert code == 200
    assert h["Content-Type"] == "application/json; charset=utf-8"
    assert h["Cache-Control"] == "no-cache"
    assert h["Access-Control-Allow-Origin"] == "*"
    j = json.loads(body)
    assert [r["id"] for r in j["recordings"]] == [C, B, A]  # newest first
    assert set(j) == {"recordings", "overview_available"}
    assert j["overview_available"] is False  # no venv worker in this fixture
    for r in j["recordings"]:
        assert set(r) == ROW_KEYS  # no path, no cache
    by = {r["id"]: r for r in j["recordings"]}
    assert by[A]["closed"] and by[A]["indexed"] and not by[A]["in_progress"]
    assert by[A]["duration_s"] == pytest.approx(12.5)
    assert by[A]["message_count"] == 4242
    assert by[A]["start_ns"] == 1700000000000000000
    assert by[A]["overview"] == "none"
    assert by[B]["duration_s"] is None and by[B]["topics"] is None
    assert not by[C]["closed"] and by[C]["in_progress"] and not by[C]["indexed"]


def test_topics_allowlisted_nonzero_only(srv):
    make_bag(srv.root, A, metadata=META)
    _c, j = srv.jreq("/api/replay/recordings")
    assert j["recordings"][0]["topics"] == {
        "/robot_state": {"type": "jugglebot/msg/RobotState", "count": 1200}}
    assert "/robot_state" in schema.ALLOWLIST


def test_killed_recording_is_not_indexed_nor_in_progress_when_old(srv):
    make_bag(srv.root, A, kind="killed", age_s=100.0)
    make_bag(srv.root, B, kind="nosummary")
    _c, j = srv.jreq("/api/replay/recordings")
    by = {r["id"]: r for r in j["recordings"]}
    assert not by[A]["closed"] and not by[A]["indexed"] and not by[A]["in_progress"]
    assert by[B]["closed"] and not by[B]["indexed"]


def test_footer_read_unit(tmp_path):
    p = make_bag(str(tmp_path), A)
    assert recordings.read_footer(p) == (True, True)
    p2 = make_bag(str(tmp_path), B, kind="nosummary")
    assert recordings.read_footer(p2) == (True, False)
    p3 = make_bag(str(tmp_path), C, kind="killed")
    assert recordings.read_footer(p3) == (False, False)
    tiny = tmp_path / "tiny.mcap"
    tiny.write_bytes(b"\x89MCAP0\r\n")  # magic only, no footer record
    assert recordings.read_footer(str(tiny)) == (True, False)
    assert recordings.read_footer(str(tmp_path / "missing")) == (False, False)


def test_metadata_parse_is_memoised(tmp_path, monkeypatch):
    make_bag(str(tmp_path), A, metadata=META)
    recordings.list_recordings(str(tmp_path))
    calls = []
    real = recordings.parse_metadata_text
    monkeypatch.setattr(recordings, "parse_metadata_text",
                        lambda t: calls.append(1) or real(t))
    recordings.list_recordings(str(tmp_path))
    assert calls == []  # unchanged (path, size, mtime) -> no re-parse


def test_overview_route_without_worker_and_unknown(srv):
    make_bag(srv.root, A)
    code, j = srv.jreq("/api/replay/recordings/{}/overview".format(A))
    assert code == 503 and j == {"status": "unavailable", "reason": "worker_unavailable"}
    assert srv.req("/api/replay/recordings/{}/overview".format("2030-01-01_00-00-00"))[0] == 404


def test_bad_id(srv):
    for p in ("/api/replay/recordings/..%2Fx/overview", "/api/replay/recordings/x/overview"):
        assert srv.req(p)[0] == 400


def test_static_still_served(srv):
    code, h, body = srv.req("/index.html")
    assert code == 200 and b"<html" in body.lower()
    assert h["Access-Control-Allow-Origin"] == "*"
    assert h["Cache-Control"] == "no-cache"
    code, h, _b = srv.req("/js/main.js")
    assert code == 200 and h["Content-Type"] == "application/javascript"
    assert srv.req("/api/other")[0] == 404


REAL_ROOT = os.path.expanduser("~/Desktop/rosbags")


@pytest.mark.skipif(not os.path.isdir(REAL_ROOT), reason="no ~/Desktop/rosbags on this box")
def test_footer_parse_and_listing_against_real_bags():
    """U3a verified the footer parse on fakes only: check it against REAL
    recordings, with the mcap library's own summary as the independent oracle."""
    from mcap.reader import SeekingReader
    rows = recordings.list_recordings(REAL_ROOT)
    assert rows, "real rosbags dir lists nothing"
    closed = [r for r in rows if r["closed"]]
    assert closed
    step = max(1, len(closed) // 25)
    checked = 0
    for r in closed[::step]:
        info = recordings.find_recording(REAL_ROOT, r["id"])
        with open(info["path"], "rb") as fh:
            try:
                has_summary = SeekingReader(fh).get_summary() is not None
            except Exception:  # noqa: BLE001 - a footer the library cannot read is not indexed
                has_summary = False
        assert info["indexed"] == has_summary, r["id"]
        checked += 1
    assert checked >= 1
