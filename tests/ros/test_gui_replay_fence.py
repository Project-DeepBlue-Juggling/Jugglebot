# -*- coding: utf-8 -*-
"""Command fence (replay design § 3): the real commands.js under node with a fake DOM.

After replay entry with NO orchestrator_state dispatched, resetForSeek's trailing updateCommandStates() must leave
every cmd-* button disabled, and a click handler invoked under the fence must emit no COMMAND event.
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
HARNESS = REPO / "tests" / "ros" / "js" / "replay_fence_harness.js"


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
    sb = tmp_path_factory.mktemp("fence")
    (sb / "replay").mkdir()
    for name in ("clock.js", "event-store.js", "ros-bridge.js", "commands.js", "hold-to-confirm.js"):
        shutil.copy(JS / name, sb / name)
    for f in (JS / "replay").glob("*.js"):
        shutil.copy(f, sb / "replay" / f.name)
    (sb / "panels.js").write_text("export let currentOrchestratorState = 'IDLE';\n")   # stub: the live state is IDLE
    shutil.copy(HARNESS, sb / "replay_fence_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "replay_fence_harness.js")],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout.decode().strip().splitlines()[-1])


def test_unfenced_idle_enables_some_buttons(out):
    assert out["live_disabled"] == ["cmd-deactivate"]      # control: IDLE lights home/level/activate/clear


def test_fenced_update_disables_every_command_button(out):
    assert out["fenced_disabled"] == ["cmd-home", "cmd-level", "cmd-activate", "cmd-deactivate", "cmd-clear"]


def test_fenced_dispatch_emits_no_command_event(out):
    assert out["fenced_click_events"] == 0
    assert out["unfenced_click_events"] == 1               # control: the same click unfenced does log
