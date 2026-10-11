# -*- coding: utf-8 -*-
"""trail-feed.js (Phase 5 keying and policy D1-D3) against a recording fake layer.

``tests/ros/js/trail_feed_harness.js`` drives the real ``trail-feed.js`` /
``marker-palette.js`` / ``clock.js`` copied into a sandbox; no Three needed.
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
HARNESS = REPO / "tests" / "ros" / "js" / "trail_feed_harness.js"


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
    sb = tmp_path_factory.mktemp("trail_feed")
    for name in ("clock.js", "marker-palette.js", "trail-feed.js"):
        shutil.copy(JS / name, sb / name)
    shutil.copy(HARNESS, sb / "trail_feed_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "trail_feed_harness.js")],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout)


def test_label_keying_and_colours(out):
    assert out["label_keys"] == ["Platform1", "Base2"]
    assert out["colour_fn"] == [0x3b82f6, 0xef4444, 0xd1d5db, 0xf97316, 0xec4899]


def test_nn_keying_continuity_and_new_identity(out):
    k = out["nn_keys"]
    assert k[0] == k[1] and k[2] != k[0] and k[3] == k[2]
    assert all(x < 0 for x in k)
    assert out["nn_memory_keys"][0] != out["nn_memory_keys"][1]
    a, b, c, d = out["nn_two_keys"]
    assert a != b and c == b and d == a


def test_d3_suppresses_near_ball_only(out):
    # near-ball unlabelled suppressed; far unlabelled + the labelled one near the ball still trail
    assert out["d3_keys"][0] == -1 and out["d3_keys"][1] == "Platform1" and len(out["d3_keys"]) == 2
    assert out["d3_far_key_first_slot"] is True  # the suppressed marker consumed no NN slot
    assert out["d3_after_stale"] == 3  # stale ball -> the same unlabelled marker trails again


def test_d1_stale_end_at_last_message_time(out):
    assert out["balls_n_fresh"] == 1 and out["balls_id0"] == 3
    assert out["balls_same_object"] is True
    assert out["ends_before"] == 0
    assert out["d1_ends"] == [["end", 1e9 + 3, 2.0]]
    assert out["balls_n_stale"] == 0
    assert out["d1_ends_after_second_tick"] == 1
    # live render ticks first, then renders at now
    assert out["render_live"] == [["end", 1e9, 5], ["render", 5.5, 1000]]
    # replay render uses the render time and does not tick (no end)
    assert out["render_replay"] == [["render", 9, 1000]]
    assert out["render_tail0"] == ["render", 9, 0]


def test_d1_gap_ends_previous_track_at_its_last_message(out):
    # beginBalls applies D1 first: no tick ran between the two messages, as in a replay rebuild
    assert out["gap_end"] == [["end", 1e9 + 5, 1.0]]
    assert out["gap_balls"] == [1, 6]


def test_absence_end(out):
    assert out["absence"] == ["begin:balls", "endMsg:balls:1", "begin:balls", "endMsg:balls:1.005"]
    assert out["absence_n"] == 1 and out["absence_ids"] == [0]


def test_reset_clears_everything(out):
    assert out["reset_called"] == 1 and out["reset_balls_n"] == 0
    assert out["reset_nn_key"] == -1
    assert out["reset_no_end"] == 0
