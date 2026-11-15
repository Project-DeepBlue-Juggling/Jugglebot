# -*- coding: utf-8 -*-
"""BUG-104: orchestrator command buttons carry an in-flight guard (the real commands.js under node, fake DOM).

A double click inside one state-update latency publishes once; the latch releases when the orchestrator state
changes, or when the cooldown timer fires, and NOT on a same-state re-publish (orchestrator_state streams
continuously).  Skips when node is absent, like the other GUI harness tests.
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
HARNESS = REPO / "tests" / "ros" / "js" / "command_inflight_harness.js"


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
    sb = tmp_path_factory.mktemp("inflight")
    (sb / "replay").mkdir()
    for name in ("clock.js", "event-store.js", "ros-bridge.js", "commands.js", "hold-to-confirm.js"):
        shutil.copy(JS / name, sb / name)
    for f in (JS / "replay").glob("*.js"):
        shutil.copy(f, sb / "replay" / f.name)
    (sb / "panels.js").write_text(
        "export let currentOrchestratorState = 'IDLE';\n"
        "export function __setState(s) { currentOrchestratorState = s; }\n")
    shutil.copy(HARNESS, sb / "command_inflight_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "command_inflight_harness.js")],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout.decode().strip().splitlines()[-1])


def test_control_idle_enables_level(out):
    assert out["idle_disabled"] == ["cmd-deactivate"]


def test_double_click_publishes_once_across_all_buttons(out):
    assert out["double_click_published"] == ["level"]
    assert out["latched_disabled"] == ["cmd-home", "cmd-level", "cmd-activate", "cmd-deactivate", "cmd-clear"]


def test_same_state_republish_does_not_release(out):
    assert out["same_state_release"] is False
    assert out["after_same_state_published"] == 1


def test_state_change_releases(out):
    # released into LEVELLING: normal state-driven enables (only clear-errors lit)
    assert out["after_change_disabled"] == ["cmd-home", "cmd-level", "cmd-activate", "cmd-deactivate"]
    assert out["after_release_published"] == ["level", "level"]


def test_cooldown_expiry_releases_without_state_change(out):
    assert out["after_cooldown_published"] == ["level", "level", "home"]
