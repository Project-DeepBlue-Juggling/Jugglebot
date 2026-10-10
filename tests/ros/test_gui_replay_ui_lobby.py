# -*- coding: utf-8 -*-
"""Replay UI unit 1 (Phase 3): lobby, picker, header status, toast.

``tests/ros/js/replay_ui_lobby_harness.js`` runs the real ``replay/ui/{dom,format,toast,picker,lobby}.js`` under
node with a minimal fake DOM, a fake ``fetch`` (listing + manifest + status), a fake ros connection state and a
fake mode whose ``enterReplay`` resolves/rejects on demand, then prints JSON this file asserts on.
"""
from __future__ import annotations

import glob
import json
import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
UI = REPO / "ros_ws" / "gui" / "js" / "replay" / "ui"
HARNESS = REPO / "tests" / "ros" / "js" / "replay_ui_lobby_harness.js"


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
    sb = tmp_path_factory.mktemp("uilobby")
    (sb / "replay" / "ui").mkdir(parents=True)
    for name in ("dom.js", "format.js", "toast.js", "picker.js", "lobby.js"):
        shutil.copy(UI / name, sb / "replay" / "ui" / name)
    shutil.copy(HARNESS, sb / "replay_ui_lobby_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "replay_ui_lobby_harness.js")],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout.decode().strip().splitlines()[-1])


def test_lobby_follows_connection_state(out):
    d = out["disconnected"]
    assert d["lobby_hidden"] is False and d["banner_hidden"] is False and d["cls_lobby"] is True
    assert d["banner"].startswith("Not connected to the robot. Live commands are unavailable")
    assert d["dock_hidden"] is True
    for k in ("connected",):
        c = out[k]
        assert c["lobby_hidden"] is True and c["banner_hidden"] is True and c["cls_lobby"] is False
    assert out["connecting"]["lobby_hidden"] is False      # not 'connected' -> still the lobby
    assert out["live_button_untouched"] is True            # hiding is CSS-only, the live buttons are not torn down
    assert out["probe_fetches"] >= 1                       # the lobby probes the backend once on show


def test_lobby_does_not_flash_on_cold_load(out):
    """Audit N2: the initial 'disconnected' default is not a failed connect (no lobby, no banner, no probe);
    nothing shows while the first attempt is merely 'connecting'; only a disconnected EDGE shows the lobby."""
    for k in ("initial", "first_connecting"):
        d = out[k]
        assert d["lobby_hidden"] is True and d["banner_hidden"] is True and d["cls_lobby"] is False, k
    assert out["initial"]["probe_fetches"] == 0


def test_empty_bag_sentinel_start_ns_sorts_by_mtime_and_renders_a_sane_date(out):
    """Audit N4: an empty bag reports start_ns = INT64_MAX; it must sort by mtime - duration (between r_noindex
    and r_stale) and show a real date, not a year-2262 row at the top."""
    import re
    assert re.match(r"^2026-\d\d-\d\d", out["empty_row_when"]), out["empty_row_when"]


def test_picker_double_open_probe_and_reopen(out):
    """Audit N3: double-open registers one keydown handler; a probe during the list load does not orphan it."""
    d = out["double_open"]
    assert d["first"] == d["second"] == 1 and d["after_close"] == 0
    assert out["probe_during_load"]["rows"] >= 5 and out["probe_during_load"]["loading"] is False


def test_listing_is_the_only_fetch(out):
    """Phase 4: the listing row carries topics/duration/indexed, so the picker never fetches /manifest or /status
    per row (replaces the enrichment tests: close_aborts, reopen_manifest_fetches)."""
    assert out["non_listing_fetches"] == 0


def test_no_session_button_or_row(out):
    """The live session buffer ("Replay last session") was deleted 2026-10-10: no lobby button, no picker row."""
    assert out["session_btn_absent"] is True
    assert "session" not in out["rows_keys"]


def test_list_newest_first_and_row_states(out):
    assert out["rows_keys"] == ["r_live", "r_complete", "r_rich", "r_noindex", "r_empty", "r_old"]
    s = out["states"]
    assert re.fullmatch(r"\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}", s["r_complete"]["when"])
    assert s["r_complete"]["cell"] == "ready" and s["r_complete"]["dur"] == "01:40"
    assert s["r_rich"]["cell"] == "ready" and s["r_old"]["cell"] == "ready"      # no cached/converting/stale states exist
    assert s["r_noindex"]["cell"] == "no index" and s["r_noindex"]["dur"] == "?"
    assert s["r_noindex"]["refused"] is True                                      # dimmed: a killed recording cannot be opened
    assert s["r_live"]["cell"] == "recording in progress" and s["r_live"]["refused"] is True
    assert s["r_complete"]["refused"] is False
    assert out["banner_hidden_ok"] is True


def test_missing_key_topics_are_struck_chips(out):
    s = out["states"]
    assert s["r_complete"]["chips"] == [["robot_state", False], ["balls", True], ["cone catch", True], ["bb state", True]]
    assert s["r_rich"]["chips"] == [["robot_state", False], ["balls", False], ["cone catch", False], ["bb state", False]]
    assert s["r_complete"]["topics"] == "2 topics"          # balls has count 0 -> not a recorded topic
    assert s["r_noindex"]["topics"] == "—"             # topics: null (no metadata.yaml counts) -> unknown, no struck chips
    assert all(not m for _, m in s["r_noindex"]["chips"])


def test_filter_narrows(out):
    assert out["filter_stale"] == ["r_old"]
    # topic filter matches the listing's topics: rows with topics null drop out
    assert out["filter_topic"] == ["r_live", "r_complete", "r_rich", "r_old"]
    assert out["filter_none"] == []


def test_choose_calls_enter_replay_and_closes(out):
    assert out["choose_live"] is False and out["calls_after_live"] == 0
    assert out["choose_noindex"] is False and out["calls_after_noindex"] == 0      # no-index rows are not selectable
    assert out["opening_cell"] == "opening…"
    assert out["calls"] == [{"kind": "recording", "id": "r_complete"}]
    assert out["chose"] is True and out["open_after_success"] is False


@pytest.mark.parametrize("reason,fragment", [
    ("recording_in_progress", "still being written"),
    ("in_progress", "still being written"),
    ("no_index", "no index"),
    ("compressed", "compressed chunks"),
    ("changed", "changed on disk"),
    ("http", "could not be read"),
    ("connected", "Connected to the robot"),
])
def test_refusal_message_keeps_picker_open(out, reason, fragment):
    r = out["refusals"][reason]
    assert r["ok"] is False and r["open"] is True and r["msg_visible"] is True
    assert fragment in r["msg"]


def test_close_gestures(out):
    assert out["esc_closed"] is True and out["backdrop_closed"] is True and out["inner_click_keeps"] is True


def test_unreachable_listing_banner_and_retry(out):
    b = out["backend_down"]
    assert b["banner_visible"] and b["has_retry"] and "unreachable" in b["text"] and b["cls"].endswith("err")
    assert b["open_disabled"] is True and b["backend_banner_hidden"] is False
    assert out["backend_back"] == {"banner_hidden": True, "open_disabled": False}


def test_missing_overview_worker_is_only_a_note(out):
    """Phase 4: opening never needs the venv worker; overview_available=false is an amber note, not a blocker."""
    n = out["overview_note"]
    assert n["banner_visible"] and "overview unavailable" in n["text"].lower() and n["cls"].endswith("warn")
    assert n["has_retry"] is False and n["open_disabled"] is False and n["rows_selectable"] >= 3
    assert n["lobby_note_hidden"] is False and n["lobby_cls"].endswith("warn") and "overview unavailable" in n["lobby_note"].lower()


def test_header_says_opening_until_slot_zero_is_resident(out):
    assert out["header_opening"] == "REPLAY  opening…"


def test_header_status_and_toast(out):
    h = out["header_replay"]
    assert re.fullmatch(r"REPLAY  \d{4}-\d{2}-\d{2}", h["text"]) and h["dot"] == "status-dot replay"
    assert h["cls_active"] and h["dock_hidden"] is False and h["lobby_hidden"] is True
    a = out["header_after"]
    assert a["text"] == "Disconnected" and a["dot"] == "status-dot disconnected" and a["lobby_hidden"] is False
    assert out["toasts"] == ["Replay ended: rosbridge connected"] and out["toasts_after_timer"] == 0
