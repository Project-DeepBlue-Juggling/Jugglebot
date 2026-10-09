# -*- coding: utf-8 -*-
"""The replay dispatch policy (ros_ws/gui/js/replay/policy.js) covers every
converted topic, and its live gates are the throttles main.js subscribes with.

Phase 2 unit U3 (replay design §§ 4, 7). The table is read by importing the
real module under node (verbatim copy), not by regex, so the test sees exactly
what the engine sees. Pins:

- the policy's topic set == ``schema.SUBSCRIBED + PLANNED`` (both directions:
  a new converted topic without a class, or a stale row, fails here);
- every row's class is one of the five design classes;
- ``throttleMs`` == the ``throttleRate`` of that topic's ``ros.subscribe(...)``
  in main.js ``subscribeAll()`` (PLANNED topics, not subscribed yet, are 0);
- the on-change exceptions are exactly orchestrator_state + control_mode_topic;
- ``W_PREROLL_SEC`` and ``SPEED_LADDER`` carry the design values.
"""
from __future__ import annotations

import glob
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
GUI = REPO / "ros_ws" / "gui"
if str(GUI) not in sys.path:
    sys.path.insert(0, str(GUI))

from replay import schema  # noqa: E402

CLASSES = {"state-render", "state-edge", "history-ring", "event", "columns-only"}


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
def policy(tmp_path_factory):
    sb = tmp_path_factory.mktemp("policy")
    shutil.copy(GUI / "js" / "replay" / "policy.js", sb / "policy.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    (sb / "dump.js").write_text(
        "import * as P from './policy.js';\n"
        "process.stdout.write(JSON.stringify({table: P.TOPIC_POLICY, w: P.W_PREROLL_SEC,"
        " ladder: P.SPEED_LADDER, gate8: P.gateFor('/robot_state', 8),"
        " gateRev: P.gateFor('/robot_state', -4), ev: P.gateFor('/skills/attempt', 8),"
        " snap: [P.snapSpeed(3), P.snapSpeed(-0.3), P.snapSpeed(100)]}));\n")
    proc = subprocess.run([NODE, str(sb / "dump.js")], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout)


def _main_throttles():
    src = (GUI / "js" / "main.js").read_text(encoding="utf-8")
    body = re.search(r"function subscribeAll\(\) \{(.*?)\n\}", src, re.S).group(1)
    hits = re.findall(r"ros\.subscribe\('([^']+)',\s*'[^']+',\s*\w+,\s*(\d+)\)", body)
    assert hits, "no ros.subscribe(...) calls parsed from subscribeAll()"
    return {"/" + name: int(ms) for name, ms in hits}


def test_policy_covers_exactly_the_converted_topics(policy):
    assert set(policy["table"]) == set(schema.SUBSCRIBED) | set(schema.PLANNED)


def test_every_row_has_a_design_class(policy):
    for topic, row in policy["table"].items():
        assert row["cls"] in CLASSES, topic


def test_throttles_match_main_js(policy):
    live = _main_throttles()
    assert set(live) == set(schema.SUBSCRIBED)
    for topic, row in policy["table"].items():
        assert row["throttleMs"] == live.get(topic, 0), topic


def test_on_change_exceptions(policy):
    on_change = {t for t, r in policy["table"].items() if r.get("onChange")}
    assert on_change == {"/orchestrator_state", "/control_mode_topic"}


def test_constants_and_helpers(policy):
    assert policy["w"] == 30
    assert policy["ladder"] == [0.25, 0.5, 1, 2, 4, 8]
    assert abs(policy["gate8"] - 0.4) < 1e-12 and abs(policy["gateRev"] - 0.2) < 1e-12
    assert policy["ev"] == 0
    assert policy["snap"] == [4, -0.25, 8]
