"""Recording rename (POST /api/replay/recordings/<id>/rename): the folder, the .mcap and metadata.yaml's
file reference all move to '<timestamp>_<name>', so the bag still lists, opens and plays under the new id."""
from __future__ import annotations

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from _replay_srv import Srv, make_bag  # noqa: E402
from replay import recordings  # noqa: E402

A = "2026-10-01_10-00-00"
META = """rosbag2_bagfile_information:
  version: 5
  storage_identifier: mcap
  relative_file_paths:
    - {rid}_0.mcap
  duration:
    nanoseconds: 12500000000
  files:
    - path: {rid}_0.mcap
      starting_time:
        nanoseconds_since_epoch: 1700000000000000000
"""


@pytest.fixture
def srv(tmp_path):
    s = Srv(tmp_path)
    yield s
    s.close()


def rename(srv, rid, name):
    return srv.jreq("/api/replay/recordings/{}/rename".format(rid), method="POST",
                    headers={"Content-Type": "application/json"},
                    data=json.dumps({"name": name}).encode())


def test_rename_moves_folder_mcap_and_metadata(srv):
    make_bag(srv.root, A, metadata=META.format(rid=A))
    code, j = rename(srv, A, "three ball / test #2")
    new = A + "_three-ball-test-2"
    assert code == 200 and j == {"id": new, "old_id": A}
    assert sorted(os.listdir(srv.root)) == [new]
    d = os.path.join(srv.root, new)
    assert sorted(os.listdir(d)) == [new + "_0.mcap", "metadata.yaml"]
    meta = open(os.path.join(d, "metadata.yaml")).read()
    assert meta == META.format(rid=new)                         # both rosbag2 references follow the file
    _c, listing = srv.jreq("/api/replay/recordings")
    assert [r["id"] for r in listing["recordings"]] == [new]
    assert listing["recordings"][0]["indexed"] is True
    assert srv.req("/api/replay/recordings/{}/file".format(new), method="HEAD")[0] == 200


def test_rename_again_and_back_to_bare_timestamp(srv):
    make_bag(srv.root, A, metadata=META.format(rid=A))
    assert rename(srv, A, "first")[1]["id"] == A + "_first"
    assert rename(srv, A + "_first", "second")[1]["id"] == A + "_second"   # replaces, never stacks, the name
    code, j = rename(srv, A + "_second", "   ")
    assert code == 200 and j["id"] == A
    assert sorted(os.listdir(os.path.join(srv.root, A))) == [A + "_0.mcap", "metadata.yaml"]


def test_rename_refusals(srv):
    make_bag(srv.root, A)
    make_bag(srv.root, A + "_taken")
    live = "2026-10-02_10-00-00"
    make_bag(srv.root, live, kind="killed", age_s=1.0)
    assert rename(srv, A, "taken")[0] == 409                    # never overwrite another recording
    assert rename(srv, live, "x")[1]["reason"] == "recording_in_progress"
    assert rename(srv, "2030-01-01_00-00-00", "x")[0] == 404
    assert srv.req("/api/replay/recordings/x/rename", method="POST", data=b"{}")[0] == 400
    assert srv.req("/api/replay/recordings/{}/rename".format(A), method="POST", data=b"nope")[0] == 400
    assert srv.req("/api/replay/recordings/{}/rename".format(A), method="POST",
                   data=json.dumps({"name": 5}).encode())[0] == 400
    assert os.path.isdir(os.path.join(srv.root, A))            # every refusal left the recording alone


def test_rename_without_metadata_and_foreign_mcap_name(srv, tmp_path):
    d = os.path.join(srv.root, A)
    p = make_bag(srv.root, A)
    os.rename(p, os.path.join(d, "custom.mcap"))               # a .mcap not named after the folder keeps its name
    code, j = rename(srv, A, "x")
    assert code == 200 and os.listdir(os.path.join(srv.root, j["id"])) == ["custom.mcap"]


def test_rename_carries_the_cached_overview(srv):
    make_bag(srv.root, A)
    os.makedirs(srv.backend.overview_dir, exist_ok=True)
    with open(os.path.join(srv.backend.overview_dir, A + ".json"), "w") as f:
        f.write("{}")
    new = rename(srv, A, "x")[1]["id"]
    assert os.listdir(srv.backend.overview_dir) == [new + ".json"]


def test_rename_rolls_back_when_the_folder_move_fails(tmp_path, monkeypatch):
    root = str(tmp_path)
    make_bag(root, A, metadata=META.format(rid=A))
    real = os.rename

    def flaky(src, dst):
        if os.path.isdir(src):
            raise OSError("boom")
        real(src, dst)
    monkeypatch.setattr(recordings.os, "rename", flaky)
    with pytest.raises(OSError):
        recordings.rename_recording(root, A, "x")
    d = os.path.join(root, A)
    assert sorted(os.listdir(d)) == [A + "_0.mcap", "metadata.yaml"]
    assert open(os.path.join(d, "metadata.yaml")).read() == META.format(rid=A)


@pytest.mark.parametrize("rid,ok", [
    (A, True), (A + "_x", True), (A + "_a.b-c_d", True), (A + "_", False), (A + "_.hidden", False),
    (A + "_a/b", False), (A + "_a b", False), (A + "_" + "a" * 80, True), (A + "_" + "a" * 81, False),
    ("x", False), (A + "\n", False),
])
def test_valid_id_accepts_named_recordings(rid, ok):
    assert recordings.valid_id(rid) is ok


@pytest.mark.parametrize("text,slug", [
    ("three ball test", "three-ball-test"), ("  a//b  ", "a-b"), ("ünïcode", "n-code"), ("...", ""),
    ("a__b", "a_b"), ("x" * 200, "x" * 80),
])
def test_slugify(text, slug):
    assert recordings.slugify(text) == slug
