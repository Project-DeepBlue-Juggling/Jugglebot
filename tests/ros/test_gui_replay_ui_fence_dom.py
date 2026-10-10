# -*- coding: utf-8 -*-
"""Replay UI unit 3: the DOM face of the command fence (charting decision 10). Runs the real replay/ui/fence-dom.js."""
from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
UI = REPO / "ros_ws" / "gui" / "js" / "replay" / "ui"
HARNESS = REPO / "tests" / "ros" / "js" / "replay_ui_fence_dom_harness.js"
NODE = shutil.which("node") or shutil.which("nodejs")
pytestmark = pytest.mark.skipif(NODE is None, reason="node not installed")


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    sb = tmp_path_factory.mktemp("uifence")
    (sb / "replay" / "ui").mkdir(parents=True)
    shutil.copy(UI / "fence-dom.js", sb / "replay" / "ui" / "fence-dom.js")
    shutil.copy(HARNESS, sb / "h.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    p = subprocess.run([NODE, str(sb / "h.js")], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert p.returncode == 0, p.stderr.decode()
    return json.loads(p.stdout.decode().strip().splitlines()[-1])


def test_table_covers_the_six_surfaces_of_decision_10(out):
    assert sorted(out["surfaces"]) == sorted(
        ["commandOverlay", "jogPanel", "jugglePanel", "minimapSequencer", "ballButler", "holdToConfirm"])


def test_every_affordance_disabled_with_tooltip_on_entry(out):
    assert out["onAllDisabled"] and out["onAllTitled"]
    assert all(out["perSurface"].values())


def test_replay_ui_own_controls_untouched(out):
    assert out["ownUntouched"]


def test_live_reenable_is_refenced_by_the_observer(out):
    assert out["reFenced"] is True


def test_observer_watches_only_the_surface_roots_not_the_body(out):
    """Audit N1: one observe() per surface root (not document.body)."""
    assert out["observed_roots"] == out["expected_roots"] and out["observed"] == len(out["expected_roots"])


def test_chart_signal_toggle_pills_stay_enabled(out):
    """Audit W2: .signal-toggle.hold-fillable (chart visibility pills) is not a command affordance."""
    assert out["pill_enabled"] is True


def test_exit_restores_prior_disabled_state_and_titles(out):
    assert out["afterEnabled"] == 1 and out["throwStillDisabled"] is True
    assert out["homeTitle"] == "Home"
    assert out["disconnected"] == 1 and out["fencedFlag"] is False


def test_selectors_reference_real_ids_in_the_gui_sources():
    gui = REPO / "ros_ws" / "gui"
    text = (gui / "index.html").read_text() + "".join(p.read_text() for p in (gui / "js").glob("*.js"))
    for ident in ("panel-jog", "panel-speed-limits", "minimap-juggle", "minimap-action", "bb-calibrate-btn",
                  "bb-throw-btn", "bb-aim-release", "bb-throw-content", "command-overlay", "hold-fillable", "cmd-btn"):
        assert ident in text, ident
