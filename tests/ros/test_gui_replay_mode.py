# -*- coding: utf-8 -*-
"""Replay mode state machine (ros_ws/gui/js/replay/mode.js), design § 3 / § 8, Phase 2 unit U6.

``tests/ros/js/replay_mode_harness.js`` runs the real mode.js with the real engine, cache, session buffer,
event store, clock, fence and ros-bridge under node (ROSLIB and the chart / link / main collaborators faked
to LOG), feeds 30 s through the ros-bridge tap, then prints JSON this file asserts on.

Properties:
1. entry is refused while connected; a 409 refusal and an empty session buffer leave the GUI untouched;
2. entry order == design § 8 (events snapshot, clock, fence, charts, links, resetForSeek);
3. exit order is the reverse and ends with the live disconnect blanking;
4. a connected edge mid-replay exits BEFORE main's connection listener observes it; live events restored;
5. the ros-bridge tap records every delivered message with clock.now(), and is skipped in replay;
6. "replay last session" opens paused at t1 - span, and plays to t1 then pauses (frozen snapshot is complete);
7. replay events dedup on (t, type, label) across a reverse-then-forward pass.
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
HARNESS = REPO / "tests" / "ros" / "js" / "replay_mode_harness.js"


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
    sb = tmp_path_factory.mktemp("mode")
    (sb / "replay").mkdir()
    for name in ("clock.js", "event-store.js", "ros-bridge.js"):
        shutil.copy(JS / name, sb / name)
    for name in ("chunk.js", "sources.js", "session.js", "policy.js", "engine.js", "cache.js", "fence.js", "mode.js"):
        shutil.copy(JS / "replay" / name, sb / "replay" / name)
    shutil.copy(HARNESS, sb / "replay_mode_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "replay_mode_harness.js")],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout.decode().strip().splitlines()[-1])


def test_entry_refused_while_connected(out):
    r = out["refused_connected"]
    assert r["reason"] == "connected" and r["order_len"] == 0 and r["state"] == "LIVE"
    assert out["lobby"] == "LOBBY"


def test_refusal_and_empty_session_touch_nothing(out):
    assert out["refusal_409"] == {"reason": "ros_running", "order_len": 0, "state": "LOBBY", "isReplay": False}
    assert out["no_session"] == {"reason": "no session buffer", "state": "LOBBY"}


def test_entry_order_is_design_section_8(out):
    assert out["entry"]["order_prefix"] == [
        "events.snapshot", "clock.enter", "fence.true", "charts.createStore", "charts.enter",
        "link.can:true", "link.udp:true", "link.hw:true",
    ]
    # resetForSeek follows the links
    assert "resetForSeek" in out["entry"]["after_entry_sample"][:1]
    assert out["entry"]["fenced"] and out["entry"]["isReplay"]


def test_exit_order_is_reverse_and_ends_with_blanking(out):
    o = out["exit_order"]
    assert o == ["trafficReset", "charts.exit", "fence.false", "clock.exit", "events.restore",
                 "link.can:false", "link.udp:false", "link.hw:false", "blank"]
    assert o[-1] == "blank"
    assert o.index("charts.exit") < o.index("fence.false") < o.index("clock.exit") < o.index("events.restore")
    a = out["after_exit"]
    assert a["state"] == "LOBBY" and not a["isReplay"] and not a["fenced"] and a["events_restored"]


def test_connected_edge_exits_before_main_listener(out):
    c = out["connected_edge"]
    L = c["listener"]
    assert L is not None and L["state"] == "connected"
    assert L["last"] == "blank"                 # the whole exit (ending in the blanking) ran first
    assert not L["isReplay"] and not L["fenced"]
    assert L["mode"] == "LIVE"
    assert c["events_restored"]
    assert c["order"][-1] == "blank" and "charts.exit" in c["order"]


def test_session_buffer_tap(out):
    t = out["tap"]
    assert t["n_robot_state_recorded"] == 60
    assert t["last_robot_state"] == {"t": t["expected_last_t"], "v": 59}
    assert t["last_orch"]["t"] == t["expected_last_t"]
    s = out["tap_replay_skip"]
    assert s["before"] == s["after"]


def test_replay_last_session_opens_paused_at_t1_minus_span(out):
    e = out["entry"]
    assert e["state"]["mode"] == "REPLAY" and e["state"]["sub"] == "paused"
    assert e["state"]["playhead"] == pytest.approx(e["expect_playhead"], abs=1e-6)
    f = out["forward"]
    assert f["mode"] == "paused" and f["playhead"] == pytest.approx(f["t1"], abs=0.2)


def test_event_dedup_across_reverse_then_forward(out):
    r = out["reverse_forward"]
    assert r["n_events"] == r["unique"] == out["forward"]["n_events"] > 0
    assert r["same_as_first"]
    assert out["dedup_direct"]["added"] == 1


def test_throwing_entry_step_rolls_back_fully(out):
    t = out["entry_throw"]
    assert t["error"] == "boom-enter"
    assert not t["isReplay"] and not t["fenced"] and t["state"] == "LOBBY" and not t["active"]
    assert t["events_restored"]
    # the entry got as far as charts.enter, then the full exit ran (latches saved at entry were restored)
    assert "latches.get" in t["order"] and "latches.restore:1" in t["order"]
    assert t["order"].index("charts.enter") < t["order"].index("fence.false") < t["order"].index("clock.exit")


def test_throwing_exit_step_still_undoes_fence_and_clock(out):
    t = out["exit_throw"]
    assert t["was_replay"] is True
    assert not t["isReplay"] and not t["fenced"] and not t["active"]
    assert t["logged"]
    assert t["order"].index("charts.exit") < t["order"].index("fence.false") < t["order"].index("clock.exit")
    assert t["order"][-1] == "blank"


def test_reconnect_loop_edges_never_reach_listeners_during_replay(out):
    """Flicker class (owner, 2026-10-10): with rosbridge down, the reconnect loop's connecting<->disconnected
    edges reached main's listener while OPENING/REPLAY and blanked the panels every ~2 s. The ros-bridge
    suppressor (installed by mode.js) holds every non-'connected' edge back from ALL listeners in that phase."""
    f = out["flicker_class"]
    d = f["during"]
    assert d["listener_states"] == [] and d["blanks"] == 0 and d["mode"] == "REPLAY"
    assert d["conn"] == "disconnected"           # the bridge still tracks the truth
    c = f["connected"]                            # the 'connected' pre-notify exit path is unchanged
    assert c["listener_states"] == ["connected"] and c["blanks"] == 1 and c["mode"] == "LIVE"
    assert f["exit_blanks"] == 1                  # exit runs the blanking itself, once
    assert f["after_exit_states"] == ["connecting", "disconnected"] and f["mode_after"] == "LOBBY"
