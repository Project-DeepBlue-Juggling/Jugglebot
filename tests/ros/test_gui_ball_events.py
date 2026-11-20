"""Throw / Catch chart markers (ros_ws/gui/js/ball-events.js, frontend half).

The REAL module runs under node (``tests/ros/js/ball_events_harness.js``): the /balls message -> event mapping, the
source choice (Hand Throws only, planned release time; Catch at the fit's landing-plane time), live header.stamp
clock mapping, and replay idempotence. Source scans pin the wiring a harness cannot see: the chart-only types stay out
of the Event Log list, /balls is dispatched to ``onBalls`` in replay (state-edge) while the trails stay live-only.
"""
from __future__ import annotations

import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

from tests.ros._node import find_node as _find_node  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
JS = ROOT / "ros_ws" / "gui" / "js"
HARNESS = Path(__file__).resolve().parent / "js" / "ball_events_harness.js"
NODE = _find_node()
T0 = 1795175860


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    if NODE is None:
        pytest.skip("node not installed")
    sb = tmp_path_factory.mktemp("ball_events")
    shutil.copy(JS / "ball-events.js", sb / "ball-events.js")
    shutil.copy(HARNESS, sb / "harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "harness.js")], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout)


def test_hand_throw_is_one_planned_marker_at_the_announced_release(out):
    (ev,) = out["throw"]
    assert ev["type"] == "throw"
    assert ev["t"] == pytest.approx(T0 - 0.5, abs=1e-6)
    assert re.match(r"^Throw 12 \u00b7 release \d\d:\d\d:\d\d\.\d\d \(planned\)$", ev["label"])
    assert out["throwAgain"] == []


def test_no_throw_marker_for_pending_ball_butler_or_unannounced(out):
    assert out["pending"] == [] and out["bb"] == [] and out["noTime"] == []


def test_catch_sits_at_the_fit_landing_time_not_at_the_caught_status(out):
    (ev,) = out["catchFit"]
    assert ev["type"] == "catch" and ev["t"] == pytest.approx(T0 + 0.4, abs=1e-6)
    assert re.match(r"^Catch 12 \u00b7 \d\d:\d\d:\d\d\.\d\d$", ev["label"])
    (pred,) = out["catchPred"]
    assert pred["label"].endswith("(predicted)")


def test_dropped_and_foreign_destination_have_no_catch(out):
    assert out["dropped"] == [] and out["foreign"] == []


def test_live_times_follow_header_stamp_not_the_clock_skew(out):
    (ev,) = out["live"]
    # browser 100.02 s ahead, message stamped 0 s, release 0.5 s before the stamp: 0.5 s before "now" minus latency
    assert ev["t"] == pytest.approx(T0 + 100.02 - 0.5, abs=1e-6)


def test_muted_baseline_never_marks_done_and_backward_seek_re_emits(out):
    assert out["muted1"] == 1      # computed (the store drops it while muted)
    assert out["muted2"] == 1      # first unmuted pass still emits
    assert out["muted3"] == 0      # then once
    assert out["afterSeek"] == 1   # a backward seek un-happens it
    assert out["clock"] is True


def test_wiring_chart_only_types_and_replay_dispatch():
    store = (JS / "event-store.js").read_text()
    assert "THROW:       'throw'" in store and "CATCH:       'catch'" in store
    assert re.search(r"CHART_ONLY_TYPES = Object\.freeze\(new Set\(\['throw', 'catch'\]\)\)", store)
    hist = (JS / "command-history.js").read_text()
    assert "CHART_ONLY_TYPES.has(ev.type)" in hist
    policy = (JS / "replay" / "policy.js").read_text()
    assert re.search(r"'/balls':\s*\{ cls: E, throttleMs: 20 \}", policy)
    main = (JS / "main.js").read_text()
    body = main[main.index("function onBalls"):]
    body = body[:body.index("function onRigidBodyPoses")]
    assert "ballEvents.map(" in body and "eventsAreMuted()" in body
    assert body.index("ballEvents.map(") < body.index("if (clock.isReplay()) return;")
