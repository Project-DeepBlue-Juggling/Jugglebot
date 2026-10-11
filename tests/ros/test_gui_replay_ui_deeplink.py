# -*- coding: utf-8 -*-
"""Replay Phase 6 unit A: deep links (``?recording=<id>&t=<t>``).

``tests/ros/js/replay_ui_deeplink_harness.js`` runs the real ``replay/ui/deeplink.js`` under node (TZ pinned) with a
fake ros / mode / toaster / document and prints JSON this file asserts on.
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
UI = REPO / "ros_ws" / "gui" / "js" / "replay" / "ui"
HARNESS = REPO / "tests" / "ros" / "js" / "replay_ui_deeplink_harness.js"
ID = "2026-10-09_14-00-00"


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
    sb = tmp_path_factory.mktemp("uideeplink")
    (sb / "replay" / "ui").mkdir(parents=True)
    shutil.copy(UI / "deeplink.js", sb / "replay" / "ui" / "deeplink.js")
    shutil.copy(HARNESS, sb / "replay_ui_deeplink_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    env = dict(os.environ, TZ="Australia/Melbourne")
    proc = subprocess.run([NODE, str(sb / "replay_ui_deeplink_harness.js")], env=env,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout.decode().strip().splitlines()[-1])


def test_parse_forms(out):
    p = out["parse"]
    assert p["offset"] == {"recording": ID, "t": {"kind": "offset", "sec": 83.4}}
    assert p["int"]["t"] == {"kind": "offset", "sec": 5}
    assert p["clock"]["t"] == {"kind": "clock", "sod": 14 * 3600 + 3 * 60 + 20}
    assert p["clock_frac"]["t"]["sod"] == 14 * 3600 + 3 * 60 + 20.5
    assert p["clock_encoded"]["t"] == p["clock"]["t"]
    assert p["none"]["t"] == {"kind": "none"}
    for k in ("junk", "bad_clock", "neg"):
        assert p[k]["t"]["kind"] == "invalid", k
    for k in ("no_rec", "empty", "other_params"):
        assert p[k] is None, k


def test_resolve_clamps_both_ends_and_wall_clock(out):
    r = out["resolve"]
    assert r["offset"] == {"sec": 83.4, "clamped": False, "invalid": False}
    assert r["clamp_hi"] == {"sec": 600, "clamped": True, "invalid": False}
    assert r["clock"]["sec"] == pytest.approx(200.5) and not r["clock"]["clamped"]
    assert r["clock_before"] == {"sec": 0, "clamped": True, "invalid": False}
    assert r["clock_after"]["sec"] == 600 and r["clock_after"]["clamped"]
    assert r["none"]["sec"] == 0 and r["invalid"]["invalid"] is True


def test_wall_clock_across_midnight(out):
    a, b, c = out["midnight"]
    assert a == pytest.approx(60) and b == pytest.approx(360)
    assert c["clamped"] is True and c["sec"] in (0, 600)


def test_build_round_trips(out):
    assert out["build"]["url"] == "http://jetson:8081/index.html?recording=" + ID + "&t=83.4"
    assert out["build"]["back"] == {"recording": ID, "t": {"kind": "offset", "sec": 83.4}}


def test_build_rounds_down_so_an_end_of_recording_link_is_not_clamped(out):
    assert out["build"]["end_t"] == "83.4"
    assert out["build"]["end_resolved"]["clamped"] is False
    assert out["build"]["exact"] == ["0.3", "0.7", "1.1", "2.9"]


def test_copy_text_paths(out):
    f = out["copy_fallback"]
    assert f["ok"] is True and f["log"]["cmd"] == "copy" and f["log"]["value"] == "U1" and f["log"]["removed"] == 1
    assert out["copy_clip"] == {"ok": True, "written": "U2", "appended": 0}
    assert out["copy_clip_rejects"] == {"ok": True, "cmd": "copy"}      # a denied clipboard falls back to execCommand
    assert out["copy_fails"] is False


def test_honoured_once_paused_at_t(out):
    assert out["initial_no_action"] == {"enters": 0, "outcome": "pending"}   # the cold-load 'disconnected' default is not settled
    h = out["honour"]
    assert h["mid"] == 0 and h["enters"] == [{"kind": "recording", "id": ID}]
    assert len(h["seeks"]) == 1 and h["seeks"][0] == pytest.approx(out["T0R"] + 83.4)
    assert h["toasts"] == []
    assert out["honour_once"] == {"enters": 1, "seeks": 1}                   # exiting replay does not re-trigger


def test_refused_while_connected(out):
    r = out["refuse"]
    assert r["enters"] == 0 and r["outcome"] == "refused_connected" and len(r["toasts"]) == 1
    assert "connected to the robot" in r["toasts"][0]
    assert out["refuse_at_register"]["enters"] == 0 and len(out["refuse_at_register"]["toasts"]) == 1


def test_not_found_notice_and_no_open(out):
    n = out["not_found"]
    assert n["enters"] == 0 and n["toasts"] == ["Recording nope not found"]


def test_clamp_invalid_and_start(out):
    assert len(out["clamp"]["seeks"]) == 1 and "clamped" in out["clamp"]["toasts"][0]
    assert out["invalid"]["seeks"] == [] and "could not read" in out["invalid"]["toasts"][0]
    assert out["no_t"] == {"seeks": [], "toasts": []}
    assert len(out["clock_t"]["seeks"]) == 1 and out["clock_t"]["toasts"] == []


def test_open_refusal_and_inert_without_link(out):
    assert out["open_refused"] == {"toasts": ["REFUSED in_progress"], "seeks": 0}
    assert out["no_link"] == {"enters": 0, "outcome": "none"}
