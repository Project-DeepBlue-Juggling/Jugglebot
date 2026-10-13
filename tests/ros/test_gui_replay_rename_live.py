# -*- coding: utf-8 -*-
"""Renaming the recording that is OPEN in replay (owner ask 2026-10-13).

``tests/ros/js/replay_rename_live_harness.js`` runs the real ``sources.js`` (``retarget``), ``mcap-worker.js``
(the out-of-band ``retarget`` op), ``mcap-decode.js`` (``httpReadable.setUrl``) and the trackbar's rename control
under node; the server side (old ids keep answering, overview pass state follows the rename) is
``test_replay_rename.py``.
"""
from __future__ import annotations

import glob
import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
GUI = REPO / "ros_ws" / "gui"
HARNESS = REPO / "tests" / "ros" / "js" / "replay_rename_live_harness.js"
OLD = "2026-10-12_23-24-02"
NEW = OLD + "_three-ball-PB"


def _find_node():
    found = shutil.which("node") or shutil.which("nodejs")
    if found:
        return found
    for pat in (os.path.expanduser("~/.nvm/versions/node/*/bin/node"), "/usr/local/bin/node", "/usr/bin/node"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[-1]
    return None


NODE = _find_node()
pytestmark = pytest.mark.skipif(NODE is None, reason="node not installed")


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    sb = tmp_path_factory.mktemp("renamelive")
    shutil.copytree(GUI / "js" / "replay", sb / "js" / "replay")   # mcap-decode.js imports ../../lib/
    shutil.copytree(GUI / "lib", sb / "lib")
    shutil.copy(HARNESS, sb / "js" / "harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "js" / "harness.js")], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout.decode().strip().splitlines()[-1])


def test_source_retarget_moves_id_worker_and_overview_poll(out):
    s = out["source"]
    assert s["id"] == NEW
    assert s["open_url"] == "/api/replay/recordings/{}/file".format(OLD)
    assert s["retarget"] == ["/api/replay/recordings/{}/file".format(NEW)]
    assert s["polls"] == ["/api/replay/recordings/{}/overview".format(OLD), "/api/replay/recordings/{}/overview".format(NEW)]
    assert s["changes"] == 1 and s["retarget_count_after_noop"] == 1


def test_worker_retarget_is_out_of_band_and_silent(out):
    w = out["worker"]
    assert w["url"] == "/api/replay/recordings/B/file"
    assert "retarget" not in w["replies"]     # no reply: the source does not wait on it


def test_readable_follows_the_new_url_with_the_same_etag(out):
    calls = out["readable"]
    assert [c["url"] for c in calls] == ["/old/file", "/old/file", "/new/file"]
    assert calls[-1]["ifMatch"] == '"10-5"'   # a rename keeps size + mtime, so the ETag still matches


def test_trackbar_rename_beside_exit(out):
    t = out["tb"]
    i = t["ctl"].index("rp-exit")
    assert t["ctl"][i + 1] == "rp-name-view" and t["ctl"][i + 2] == "rp-name-edit"
    assert t["name0"] == "unnamed" and "unnamed" in t["name0_cls"]
    assert t["editing"] == {"view_hidden": True, "input_value": ""}
    a = t["after"]
    assert a["name"] == "three-ball-PB" and "unnamed" not in a["cls"] and a["retargeted"] == NEW and a["edit_hidden"] is True
    assert a["post"] == {"url": "/api/replay/recordings/{}/rename".format(OLD), "body": {"name": "three ball PB"}}
    assert t["prefill"] == "three-ball-PB"
    assert t["fail"] is None and "already exists" in t["fail_toast"] and t["fail_still_editing"] is True
    assert t["esc_closed"] is True
