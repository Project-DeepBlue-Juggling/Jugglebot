"""``GET /api/records`` (gui_server.py, JB-35): the records ledger as the GUI
panel reads it -- ``records.to_json`` of the ``--records-path`` file, no-cache,
the empty ledger when the file is missing, a 500 (never a guess) when it is
malformed. Same ephemeral-port harness as the replay route tests."""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from _replay_srv import Srv, gui_server  # noqa: E402  (path shim + server)
from jugglebot import records as rec  # noqa: E402


class _RecordsSrv(Srv):
    def __init__(self, tmp_path, records_path):
        os.makedirs(str(tmp_path / "bags"))
        args = argparse.Namespace(
            port=0, host="127.0.0.1", rosbags_dir=str(tmp_path / "bags"),
            overview_dir=str(tmp_path / "overview"), worker_python="/nonexistent",
            records_path=str(records_path))
        self.server, self.backend = gui_server.make_server(args)
        self.port = self.server.server_address[1]
        self.t = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.t.start()


@pytest.fixture
def ledger(tmp_path):
    return tmp_path / "records.md"


def _get(tmp_path, path):
    srv = _RecordsSrv(tmp_path, path)
    try:
        return srv.req("/api/records")
    finally:
        srv.close()


def test_serves_the_ledger_as_json_without_caching(tmp_path, ledger):
    model = rec.rebuild([rec.Attempt("2026-10-21 10:00", "columns", 12, 11, "STOPPED")])
    rec.write_atomic(str(ledger), rec.render(model))
    code, headers, body = _get(tmp_path, ledger)
    assert code == 200
    assert headers["Cache-Control"] == "no-cache"
    assert headers["Content-Type"] == "application/json"
    assert json.loads(body.decode()) == rec.to_json(model)


def test_a_missing_ledger_is_the_empty_one(tmp_path, ledger):
    code, _h, body = _get(tmp_path, ledger)
    assert code == 200
    assert json.loads(body.decode()) == rec.to_json(rec.empty())


def test_a_malformed_ledger_is_a_500_not_a_guess(tmp_path, ledger):
    ledger.write_text("# not a ledger\n")
    code, _h, body = _get(tmp_path, ledger)
    assert code == 500
    assert "malformed" in json.loads(body.decode())["error"]


def test_the_default_path_is_the_repos_docs_records_md():
    default = gui_server.build_parser().get_default("records_path")
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    assert os.path.abspath(default) == os.path.join(repo, "docs", "records.md")


def test_the_server_loads_the_module_by_path_not_the_ros_package():
    """No ``jugglebot`` package import happens: the server stays ROS-free."""
    assert gui_server.records.__name__ == "jugglebot_records"
    assert os.path.samefile(gui_server.records.__file__, rec.__file__)
