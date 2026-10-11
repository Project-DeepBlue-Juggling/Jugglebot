# -*- coding: utf-8 -*-
"""Replay mode state machine (ros_ws/gui/js/replay/mode.js), design § 3 / § 8, Phase 2 unit U6.

``tests/ros/js/replay_mode_harness.js`` runs the real mode.js with the real engine, cache, McapSource (scripted fake worker),
event store, clock, fence and ros-bridge under node (ROSLIB and the chart / link / main collaborators faked
to LOG), feeds 30 s to the live handlers and serves the same 30 s as a three-slot recording, then prints JSON this file asserts on.

Properties:
1. entry is refused while connected; a 409 refusal leaves the GUI untouched;
2. entry order == design § 8 (events snapshot, clock, fence, charts, links, resetForSeek);
3. exit order is the reverse and ends with the live disconnect blanking;
4. a connected edge mid-replay exits BEFORE main's connection listener observes it; live events restored;
5. (retired 2026-10-10: the live session-buffer tap was deleted; see test_gui_replay_memory_contract.py);
6. a recording opens paused at t0, and plays to t1 then pauses;
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
    for name in ("clock.js", "event-store.js", "ros-bridge.js", "trail-feed.js", "marker-palette.js"):
        shutil.copy(JS / name, sb / name)
    for name in ("chunk.js", "sources.js", "slot.js", "policy.js", "engine.js", "cache.js", "fence.js", "mode.js", "trail-window.js"):
        shutil.copy(JS / "replay" / name, sb / "replay" / name)
    shutil.copy(HARNESS.parent / "replay_test_support.js", sb / "replay_test_support.js")
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


def test_refusal_touches_nothing(out):
    assert out["refusal_409"] == {"reason": "ros_running", "order_len": 0, "state": "LOBBY", "isReplay": False}


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


def test_recording_opens_paused_at_t0_and_plays_to_t1(out):
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


def test_recording_entry_holds_in_opening_until_slot_0_is_resident(out):
    """Phase 4 design § 4: REPLAY is not entered (and no entry step runs) before the opening slot is resident;
    once it is, the entry completes paused. Root cause guarded: entering with an unloaded source made the
    first seek wait inside REPLAY with the fence up and the live panels already blanked."""
    h = out["hold_slot0"]
    assert h["held"]["state"] == "OPENING"
    assert h["held"]["order"] == [] and h["held"]["isReplay"] is False
    assert h["held"]["loads"] == [[h["first_load_index"], False]]
    assert h["after"]["state"] == "REPLAY" and h["after"]["sub"] == "paused"
    assert h["after"]["snapshot_at"] is True and h["after"]["isReplay"] is True


def test_recording_entry_slot_0_load_failure_leaves_lobby_untouched(out):
    """A slot-0 load that rejects aborts the entry with the load's reason: still LOBBY, no entry step ran,
    the clock never went to replay, and the half-opened source was closed."""
    f = out["hold_slot0_fail"]
    assert f["reason"] == "decode" and f["state"] == "LOBBY"
    assert f["order"] == [] and f["isReplay"] is False
    assert f["closed"] >= 1


def test_digester_lives_exactly_as_long_as_the_replay(out):
    d = out["digester"]
    assert d["during"]["n"] == 1 and d["during"]["disposed"] == 0
    assert d["during"]["keys"] == ["cache", "engine", "source", "store"] and d["during"]["store_is_fake"]
    assert d["n_total"] == 2 and d["after"] == [1, 1]          # disposed exactly once per replay
    assert d["dispose_before_charts_exit"]


def test_resident_changes_rebuild_the_chart_store_once_per_frame(out):
    c = out["resident_coalesce"]
    assert c["before_frame"] == 0          # slot arrivals alone never rebuild
    assert c["after_frame"] == 1           # the next frame rebuilds exactly once
    assert c["pending_exit_clean"] is True


def test_trails_dependency_creates_window_forwards_hooks_and_disposes(out):
    t = out["trails"]
    assert t["afterEntry"][0] == "R"                                  # live trails cleared before the first playhead
    assert any(x.startswith("S") and x != "Snull" for x in t["afterEntry"])  # the entry seek's onPlayhead reached the window
    assert t["preRoll"] >= 5 and t["preRollBig"] >= 60                # the seek's ensure covers the tail
    assert t["afterSeek"][0] == "R" and t["afterSeek"][-1].startswith("S")  # seek -> forced rebuild at the new playhead
    assert t["afterExit"] == ["R", "Snull"]                           # exit: reset + render time back to clock.now()
    assert t["unsubTail"] == 1 and t["tailCbCleared"]
    assert t["nullFeedOk"]
