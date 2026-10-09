# -*- coding: utf-8 -*-
"""Replay engine (ros_ws/gui/js/replay/engine.js) properties over a synthetic feed.

Phase 2 unit U3. ``tests/ros/js/replay_engine_harness.js`` builds a 60 s feed in
the in-browser chunk shape with the real ``chunk.js`` (robot_state 98 Hz with a
fault edge, orchestrator_state 10 Hz with transitions incl. a 0.3 s ERROR,
mocap 180 Hz, hand_telemetry 100 Hz stopping at 40 s, three skills/attempt
events), drives the real ``engine.js`` / ``policy.js`` / ``clock.js`` against a
recording ``dispatch`` spy and prints JSON; this file asserts on it. The
sandbox holds VERBATIM copies of the shipped modules (the fk-golden pattern).

Properties (replay design §§ 4, 6, 7):
1. per-topic dispatch count over 10 s of playhead never exceeds the live
   (rosbridge-throttle) count, and equals the throttle-gated count at 1x;
2. dispatched records are in non-decreasing t within every tick;
3. at 8x every event and every orchestrator transition is dispatched;
4. seek(p) leaves the state topics where a play-through to p + pause leaves
   them (1x and 4x), and its event dispatches are exactly the events in (p-30, p];
5. reverse dispatches only muted state records; pause re-seeks to the seek state;
6. buffering at a missing chunk and at the frontier, resuming when data appears;
7. a handler-armed 1 s virtual watchdog fires 1 s of playhead after the last
   dispatched record at 1x and 4 s at 4x, and seek clears pending timers.
"""
from __future__ import annotations

import glob
import json
import math
import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
JS = REPO / "ros_ws" / "gui" / "js"
HARNESS = REPO / "tests" / "ros" / "js" / "replay_engine_harness.js"
LADDER = [0.25, 0.5, 1, 2, 4, 8]
# live gates (s): rosbridge throttle from main.js subscribeAll(); orchestrator is
# unthrottled 10 Hz, so above 1x its gate is the recorded mean period (0.1 s).
LIVE_GATE = {"/robot_state": 0.05, "/mocap_data": 0.05, "/hand_telemetry": 0.1, "/orchestrator_state": 0.1}


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
    sb = tmp_path_factory.mktemp("engine")
    (sb / "replay").mkdir()
    shutil.copy(JS / "clock.js", sb / "clock.js")
    for name in ("chunk.js", "policy.js", "engine.js", "cache.js"):
        shutil.copy(JS / "replay" / name, sb / "replay" / name)
    shutil.copy(HARNESS, sb / "replay_engine_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "replay_engine_harness.js")],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout)


@pytest.mark.parametrize("speed", LADDER)
def test_rate_never_exceeds_live(out, speed):
    r = out["rates"][str(speed)]
    assert r["mode"] == "playing" and abs(r["playhead"] - 15) < 1e-6
    assert r["muted"] == 0
    for topic, g in LIVE_GATE.items():
        n = r["counts"][topic]
        if speed <= 1:
            bound = r["exact1x"].get(topic, n)  # below 1x: never more than the 1x set
        else:
            bound = math.floor(10 / (speed * g) + 1e-9) + 1
        if topic == "/orchestrator_state":
            bound += r["orchTransitions"]  # on-change exception (design § 7)
        assert n <= bound, (topic, speed, n, bound)
    assert r["counts"]["/skills/attempt"] == 1  # the event at 12.3 s, never gated


def test_one_x_equals_throttle_exactly(out):
    r = out["rates"]["1"]
    for topic, n in r["exact1x"].items():
        assert r["counts"][topic] == n, topic
    assert out["rates"]["0.25"]["counts"] == r["counts"]  # gates use max(1, |speed|)


def test_merged_time_order_within_every_tick(out):
    assert out["order_violations"] == 0


def test_events_and_transitions_survive_8x(out):
    f = out["full8"]
    assert f["events"] == f["events_recorded"]
    assert f["orch_dispatched"] == f["orch_recorded"]
    assert "ERROR" in f["orch_dispatched"]  # 0.3 s state, shorter than the 0.8 s gate
    assert f["end_mode"] == "paused" and abs(f["end_playhead"] - 60) < 1e-6


def test_seek_equals_play_and_pause(out):
    for e in out["seek_equiv"]:
        assert e["seekState"] == e["played"]["1"] == e["played"]["4"], e["p"]
        assert [x["t"] for x in e["seekEvents"]] == e["expectEvents"], e["p"]
        assert not any(x["muted"] for x in e["seekEvents"])
        assert e["earlyUnmuted"] == 0  # baseline (t <= p - 30) is muted
        assert e["resets"] == 1 and e["mode"] == "paused"


def test_reverse_mutes_and_pause_reseeks(out):
    r = out["reverse"]
    assert r["dispatched"] > 0 and r["fault_records_seen"] > 0
    assert r["event_dispatches"] == 0 and r["unmuted"] == 0
    assert abs(r["playhead"] - 30) < 1e-6 and r["reseeked"] and r["mode"] == "paused"
    assert r["state_after_pause"] == r["state_fresh_seek"]
    assert r["events_after_pause"] == r["events_fresh_seek"]


def test_buffering_at_gap_and_frontier(out):
    b = out["buffering"]
    assert b["stuck"]["mode"] == "buffering" and abs(b["stuck"]["playhead"] - 30) < 1e-6
    assert b["stuck"]["max_t"] < 30  # nothing dispatched through the missing chunk
    assert b["still"] == {"mode": "buffering", "playhead": b["stuck"]["playhead"]}  # holds while missing
    assert b["resumed"]["mode"] == "playing" and b["resumed"]["playhead"] > 30
    assert b["bufEvents"] == [True, False]
    assert b["chunksEntered"] == [2, 3]
    assert b["atFrontier"]["mode"] == "buffering" and abs(b["atFrontier"]["playhead"] - 20) < 1e-6
    assert b["extended"]["mode"] == "playing" and b["extended"]["playhead"] > 20


@pytest.mark.parametrize("speed", [1, 4])
def test_watchdog_scales_with_speed(out, speed):
    w = out["watchdog"][str(speed)]
    assert len(w["fires"]) == 1
    # fires |speed| s of playhead after the last dispatched record; one record
    # spacing (<= 50 ms gated dispatch) of quantisation
    assert 0 <= w["fires"][0] - (w["last_hand"] + speed) < 0.06


def test_seek_clears_timers(out):
    c = out["watchdog"]["cleared"]
    assert c["pendingBefore"] >= 1 and not c["probeFired"] and c["firesAfter"] == 0


def test_scrub_and_step(out):
    s = out["scrub"]
    assert s["all_muted"] and s["n"] == 4
    assert s["topics"] == ["/hand_telemetry", "/mocap_data", "/orchestrator_state", "/robot_state"]
    assert 30 < s["step_fwd"] < 30 + 1 / 180 + 1e-6  # one mocap record (180 Hz, the fastest)
    assert abs(s["step_back"] - 30) < 1e-6 and s["mode"] == "paused"


def test_throwing_timer_callback_does_not_stop_ticks(out):
    t = out["throw_timer"]
    assert not t["threw"] and t["logged"]
    assert t["advanced"] > 0


def test_throwing_seek_does_not_wedge_tick(out):
    t = out["throw_seek"]
    assert t["error"] == "boom-peek"
    assert t["advanced"] > 0


def test_scrub_supersedes_pending_seek(out):
    s = out["scrub_supersedes"]
    assert not s["seek_sync_section_ran"]
    assert s["final_playhead"] == pytest.approx(s["scrub_playhead"], abs=1e-9)
    assert s["seek_returned"] == pytest.approx(s["scrub_playhead"], abs=1e-9)
