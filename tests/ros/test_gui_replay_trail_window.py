# -*- coding: utf-8 -*-
"""Replay trail feeder (ros_ws/gui/js/replay/trail-window.js) over real typed chunk columns, with a logging fake feed.

``replay_trail_window_harness.js`` builds three 10 s slots (200 Hz /mocap_data with 3 markers incl. one unlabelled,
/balls at 200 Hz while a ball exists) through the shipped ``chunk.js`` column builder and drives the real window.
The sandbox holds VERBATIM copies of the shipped modules.
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
HARNESS = REPO / "tests" / "ros" / "js" / "replay_trail_window_harness.js"


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
    sb = tmp_path_factory.mktemp("trail_window")
    (sb / "replay").mkdir()
    for name in ("chunk.js", "trail-window.js"):
        shutil.copy(JS / "replay" / name, sb / "replay" / name)
    for name in ("trail-feed.js", "marker-palette.js", "clock.js"):   # trail-window imports BALL_STALE_SEC
        shutil.copy(JS / name, sb / name)
    shutil.copy(HARNESS, sb / "replay_trail_window_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "replay_trail_window_harness.js")],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout)


def test_typed_leaf_names_are_the_ones_the_window_reads(out):
    assert set(out["kinds"]["markers"]["csr"]["leaves"]) >= {"position.x", "position.y", "position.z", "label"}
    assert set(out["ballKinds"]["balls"]["csr"]["leaves"]) >= {"id", "status", "position.x", "position.y", "position.z"}


def test_seek_rebuild_equals_forward_append(out):
    e = out["seek_eq"]
    assert e["equal"] and e["nA"] > 1000 and e["nA"] == e["nB"]
    assert e["hasBall"] and e["hasTwoIds"] and e["unlabelled"] and e["sorted"]
    assert e["endsWith"][0] == "T" and e["endsWith"][1].startswith("S")


def test_reverse_step_rebuilds_the_shorter_window(out):
    r = out["reverse"]
    assert r["startsWithReset"] and r["equalsFresh"] and r["maxT"] and r["minT"] and r["rebuilds"] == 2


def test_forward_step_within_tail_appends_only_the_new_records(out):
    a = out["append"]
    assert a["noReset"] and a["min"] and a["max"] and a["n"] > 0 and a["stats"]["appends"] == 1 and a["stats"]["rebuilds"] == 1


def test_jump_past_the_tail_rebuilds(out):
    j = out["jump"]
    assert j["startsWithReset"] and j["min"] and j["rebuilds"] == 2


def test_tail_zero_still_feeds_the_ball_staleness_span(out):
    # audit 2026-10-11: tail 0 turns the trails off, not the ball spheres, so the window keeps feeding
    # max(tail, BALL_STALE_SEC) = 0.15 s: the first playhead and the seek rebuild, the 0.1 s step appends
    z = out["tail0"]
    assert z["recT"], "tail 0 must still feed records for the spheres"
    assert z["resets"] == 2 and len(z["renderTimes"]) == 3


def test_missing_chunk_marks_incomplete_and_residency_rebuilds(out):
    c = out["incomplete"]
    assert c["inc1"] is True and c["inc2"] is False and c["onlyBefore"]
    assert c["afterOther"] == 1  # a 'frontier' event does not rebuild
    assert c["startsWithReset"] and c["rebuiltEqualsFull"] and c["nAfter"] > 0
    assert c["noRebuildWhenComplete"] == 2  # a complete window ignores further residency events


def test_dispose_resets_feed_clears_render_time_and_unsubscribes(out):
    d = out["dispose"]
    assert d["log"] == ["R", "Snull"] and d["listeners"] == 0


def test_reset_for_seek_forces_a_rebuild_for_a_small_step(out):
    f = out["forced"]
    assert f["startsWithReset"] and f["min"]


def test_tail_change_rebuilds_at_the_playhead_and_unsubscribes(out):
    t = out["tailChange"]
    assert t["startsWithReset"] and t["min"] and t["unsub"] == 1


def test_untyped_topic_is_skipped(out):
    u = out["untyped"]
    assert u["ballCalls"] == 0 and u["mocapCalls"] > 0


def test_five_second_rebuild_timing_is_recorded(out):
    # Timing is reported in the logbook, never asserted: this runs in the loaded parallel gate.
    t = out["timing"]
    assert t["records"] > 1000
