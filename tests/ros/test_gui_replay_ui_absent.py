# -*- coding: utf-8 -*-
"""Replay UI unit 3: "not in this recording" (charting decision 18). The harness runs the real replay/ui/absent.js."""
from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
UI = REPO / "ros_ws" / "gui" / "js" / "replay" / "ui"
HARNESS = REPO / "tests" / "ros" / "js" / "replay_ui_absent_harness.js"
NODE = shutil.which("node") or shutil.which("nodejs")
pytestmark = pytest.mark.skipif(NODE is None, reason="node not installed")


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    sb = tmp_path_factory.mktemp("uiabsent")
    (sb / "replay" / "ui").mkdir(parents=True)
    shutil.copy(UI / "absent.js", sb / "replay" / "ui" / "absent.js")
    shutil.copy(HARNESS, sb / "h.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    p = subprocess.run([NODE, str(sb / "h.js")], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert p.returncode == 0, p.stderr.decode()
    return json.loads(p.stdout.decode().strip().splitlines()[-1])


def test_complete_recording_dims_only_regions_whose_topics_are_all_missing(out):
    assert out["complete"] == ["#panel-bb", "#panel-catching-cone", "#panel-motion"]


def test_dimmed_region_gets_tooltip_and_flags_panel_untouched(out):
    assert out["bbTitle"] == out["title"] == "not in this recording"
    assert out["flagsTitle"] == ""


def test_exit_clears_classes_and_restores_titles(out):
    assert out["afterExit"] == []
    assert out["bbTitleAfter"] == "orig bb"


def test_converting_recording_is_not_judged(out):
    assert out["converting"] == []


def test_completion_reevaluates_via_source_onchange(out):
    assert out["completedLater"] == 6   # only /robot_state present: tracking is fed by it; the other six regions dim


def test_reopen_with_every_topic_present_clears_old_marks(out):
    assert out["allPresent"] == []


def test_session_source_uses_topic_set(out):
    assert "#panel-bb" in out["session"] and "#panel-flags" not in out["session"]


def test_zero_count_topic_is_absent_and_unknown_is_null(out):
    assert out["zeroCount"] == ["/balls"]
    assert out["nullWhenUnknown"] == [None, None]
    assert out["pureNull"] == []


def test_session_source_exposes_topic_set():
    src = (REPO / "ros_ws" / "gui" / "js" / "replay" / "sources.js").read_text()
    assert "topicSet()" in src


def test_table_topics_match_what_feeds_each_region(out):
    """Audit W4: tracking <- robot_state/echo/hand_telemetry (not mocap_data); can <- profile/link_status (not
    udp_diag); BB and cone also fed by their result topics. A region is dimmed only when none of its topics exist."""
    # robot_state present -> tracking lit; no profile/link_status -> can dimmed; bb/cone lit by result topics alone
    assert "#panel-can" in out["w4_partial"]
    assert "#panel-tracking" not in out["w4_partial"]
    assert "#panel-bb" not in out["w4_partial"] and "#panel-catching-cone" not in out["w4_partial"]
    # mocap_data / udp_diag no longer light tracking / can
    assert "#panel-tracking" in out["w4_wrong_topics"] and "#panel-can" in out["w4_wrong_topics"]
