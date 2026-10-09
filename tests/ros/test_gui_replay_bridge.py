# -*- coding: utf-8 -*-
"""ros-bridge.js replay additions (Phase 2 unit U3, replay design § 3).

``tests/ros/js/replay_bridge_harness.js`` loads the REAL ``ros-bridge.js`` and
``clock.js`` (verbatim copies) under node with a minimal ROSLIB / window stub:

- ``dispatchLocal`` reaches every callback subscribed to the topic (leading
  '/' normalised), skips unknown topics, survives a throwing callback and does
  not touch the connection state;
- ``publish`` / ``callService`` are refused while ``clock.isReplay()`` (the
  transport fence) and work again after exit;
- the before-connected hook runs before every state listener, exactly once per
  'connected' edge.
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
JS = REPO / "ros_ws" / "gui" / "js"
HARNESS = REPO / "tests" / "ros" / "js" / "replay_bridge_harness.js"


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
    sb = tmp_path_factory.mktemp("bridge")
    for name in ("ros-bridge.js", "clock.js"):
        shutil.copy(JS / name, sb / name)
    (sb / "replay").mkdir()
    for name in ("session.js", "sources.js", "chunk.js"):   # ros-bridge.js imports the session-buffer tap (U6)
        shutil.copy(JS / "replay" / name, sb / "replay" / name)
    shutil.copy(HARNESS, sb / "replay_bridge_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "replay_bridge_harness.js")],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout)


def test_dispatch_local_reaches_subscribers(out):
    d = out["dispatch"]
    assert d == {"slash": 2, "bare": 1, "unknown": 0, "thrower": 1, "state_after": "disconnected"}
    assert out["got"] == [["robot_state", 1], ["robot_state#2", 1], ["bb/heartbeat", 2]]


def test_before_connected_hook_runs_first_once_per_connect(out):
    c = out["connect"]
    assert c["hookCalls"] == 2
    assert c["order"] == [
        "listener:disconnected", "listener:connecting",
        "hook", "listener:connected",
        "listener:disconnected",
        "hook", "listener:connected",
    ]


def test_transport_fence(out):
    f = out["fence"]
    assert f["published"] == ["live"]
    assert f["replayErr"] and "Replay" in f["replayErr"]
    assert f["servicesDuringReplay"] == 0
    assert f["liveRes"] == {"success": True} and f["servicesAfter"] == 1
