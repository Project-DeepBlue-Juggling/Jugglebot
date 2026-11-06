# -*- coding: utf-8 -*-
"""GUI Juggle panel nomination block (``ros_ws/gui/js/juggle-panel.js``, P3b unit I3).

``tests/ros/js/juggle_nomination_harness.js`` runs the real panel under node with a
fake DOM, a virtual clock and a scriptable transport (the real ``clock.js``,
``event-store.js`` and ``hold-to-confirm.js`` copied verbatim; a fake
``ros-bridge.js``) and prints JSON this file asserts on.

Pinned: the ``skills/nominate`` request string is byte-identical to what Start
sends plus the param-backed field (Start writes that parameter first; the
nomination never writes one); a refusal
renders every line untruncated under an err summary; the debounce coalesces
edits; a reply to an superseded request is dropped; a missing or silent service
renders a neutral line and never blocks Start or raises an error status; a
dispatch cancels a pending nomination.
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
HARNESS = REPO / "tests" / "ros" / "js" / "juggle_nomination_harness.js"

FAKE_ROS = """\
export function getConnectionState() { return 'connected'; }
export function callService(name, type, req) { return globalThis.__fakeRos.callService(name, type, req); }
export function withTimeout(promise, ms, label) {
    return new Promise((resolve, reject) => {
        const t = setTimeout(() => reject(new Error(`${label} timed out after ${ms / 1000} s`)), ms);
        promise.then((v) => { clearTimeout(t); resolve(v); }, (e) => { clearTimeout(t); reject(e); });
    });
}
"""


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
needs_node = pytest.mark.skipif(NODE is None, reason="node not installed")


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    if NODE is None:
        pytest.skip("node not installed")
    sb = tmp_path_factory.mktemp("jugglenom")
    for name in ("juggle-panel.js", "clock.js", "event-store.js", "hold-to-confirm.js"):
        shutil.copy(JS / name, sb / name)
    (sb / "ros-bridge.js").write_text(FAKE_ROS)
    shutil.copy(HARNESS, sb / "juggle_nomination_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "juggle_nomination_harness.js")],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout.decode().strip().splitlines()[-1])


@needs_node
def test_block_is_hidden_until_something_is_asked(out):
    assert out["hidden_before_connect"] is True


@needs_node
def test_checking_state_shows_at_once_and_the_call_waits_for_the_debounce(out):
    assert out["checking_after_select"]["summary"] == "checking nomination…"
    assert "checking" in out["checking_after_select"]["cls"]
    assert out["calls_before_debounce"] == 0


@needs_node
def test_nomination_writes_no_parameter(out):
    assert out["highlow_calls"] == ["skills/nominate"]
    assert out["turned_calls"] == ["skills/nominate"]
    # Start keeps its own write-then-dispatch path.
    assert out["start_calls"] == ["/skill_node/set_parameters", "jugglebot/juggle_request"]


@needs_node
def test_param_backed_patterns_carry_their_field_otherwise_byte_identical_to_starts(out):
    start = out["start_data"]["data"]
    assert out["nominate_data"] == [start + ",apex_m_ball2=0.9"]
    assert out["nominate_data"] == ["columns_highlow,apex_m=1,separation_mm=100,dwell_s=0.27,apex_m_ball2=0.9"]
    assert out["turned_nominate_data"] == [out["turned_start_data"] + ",columns_angle_deg=45"]
    assert out["turned_nominate_data"] == ["columns_turned,apex_m=0.95,separation_mm=125,dwell_s=0.27,columns_angle_deg=45"]
    # A blank required param field is a field error (Start refuses it too): nothing is asked.
    assert out["blank_angle_data"] == []


@needs_node
def test_nomination_unavailable_reply_is_neutral_not_red(out):
    v = out["unavailable_reply_view"]
    assert v["summary"] == "nomination unavailable: an Attempt is running"
    assert "unavailable" in v["cls"].split() and "err" not in v["cls"].split()


@needs_node
def test_typed_overrides_ride_the_same_string(out):
    assert out["typed_nominate_data"] == [out["typed_start_data"]]
    assert "apex_m=0.9" in out["typed_start_data"]


@needs_node
def test_refusal_renders_every_line_untruncated(out):
    v = out["refused_view"]
    assert v["hidden"] is False
    assert v["summary"].startswith("nomination REFUSED: columns_highlow")
    assert "nearest viable: separation 100 mm" in v["summary"]
    assert "err" in v["cls"].split()
    assert len(v["lines"]) == 3
    assert v["lines"][0].endswith("margin -0.04 (vel)")
    assert v["lines"][2].endswith("no leg or knot time is recorded for a margin refusal")


@needs_node
def test_ok_reply_is_coloured_ok_with_no_lines(out):
    v = out["ok_view"]
    assert v["summary"].startswith("nomination OK: columns")
    assert "ok" in v["cls"].split() and v["lines"] == []


@needs_node
def test_rapid_edits_make_one_call_with_the_last_value(out):
    assert len(out["debounce_calls"]) == 1
    assert "apex_m=0.93" in out["debounce_calls"][0]


@needs_node
def test_a_stale_reply_is_dropped(out):
    assert out["stale_after_old_reply"]["summary"] == "checking nomination…"
    assert out["stale_after_old_reply"]["lines"] == []
    assert out["stale_after_new_reply"]["summary"].startswith("nomination OK")


@needs_node
def test_missing_service_is_neutral_and_start_stays_usable(out):
    v = out["missing_view"]
    assert v["summary"].startswith("nomination unavailable")
    assert "unavailable" in v["cls"].split() and "err" not in v["cls"].split()
    assert out["missing_start_disabled"] is False
    assert "err" not in out["missing_status"]["cls"].split()
    assert out["missing_start_calls"] == ["jugglebot/juggle_request"]


@needs_node
def test_silent_service_times_out_to_the_neutral_line(out):
    assert out["pending_view"]["summary"] == "checking nomination…"
    assert out["timeout_view"]["summary"].startswith("nomination unavailable")
    assert "timed out" in out["timeout_view"]["summary"]
    assert out["timeout_start_disabled"] is False


@needs_node
def test_dispatch_cancels_a_pending_nomination(out):
    assert out["dispatch_cancels"] == {"nominate_calls": 0, "set_params": 1}   # Start's own write only


@needs_node
def test_disconnect_hides_the_block(out):
    assert out["disconnected_hidden"] is True
