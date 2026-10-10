# -*- coding: utf-8 -*-
"""Replay chart store (Phase 2 unit U4): chunk-derived columns == live onTelemetryData.

A synthetic bag is converted with the production ``replay.convert.convert``; node loads
VERBATIM copies of telemetry-charts.js / chart-store.js / chunk.js (+ clock, event-store,
geometry-config, stub 3D models, fake DOM/uPlot) via ``tests/ros/js/replay_chart_harness.js``.
The harness feeds the same records through the real live path (``onTelemetryData``, with the
main.js legs/hand latches) and through the replay store and compares every series.
"""
from __future__ import annotations

import glob
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from tests.ros._replay_chunks import export_chunks
from tests.ros._replay_fixture import write_bag

REPO = Path(__file__).resolve().parents[2]
GUI = REPO / "ros_ws" / "gui"
if str(GUI) not in sys.path:
    sys.path.insert(0, str(GUI))

from replay.convert import convert  # noqa: E402

HARNESS = REPO / "tests" / "ros" / "js" / "replay_chart_harness.js"


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
    d = tmp_path_factory.mktemp("chart")
    write_bag(d / "bag_0.mcap", 35.0, seed=5)
    cache = d / "cache"
    m = convert(str(d / "bag_0.mcap"), str(cache))
    assert m["status"] == "complete" and len(m["chunks"]) == 4

    sb = d / "sandbox"
    (sb / "replay").mkdir(parents=True)
    for name in ("telemetry-charts.js", "clock.js", "event-store.js", "geometry-config.js"):
        shutil.copy(GUI / "js" / name, sb / name)
    for name in ("chunk.js", "chart-store.js"):
        shutil.copy(GUI / "js" / "replay" / name, sb / "replay" / name)
    (sb / "stewart-model.js").write_text("export function setStewartHighlight() {}\n")
    (sb / "ball-butler-model.js").write_text("export function setBallButlerHighlight() {}\n")
    chunks = d / "chunks"
    assert export_chunks(cache, chunks) == 4
    shutil.copy(HARNESS.parent / "replay_test_support.js", sb / "replay_test_support.js")
    shutil.copy(HARNESS, sb / "replay_chart_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')

    proc = subprocess.run([NODE, str(sb / "replay_chart_harness.js"), str(chunks)],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=180)
    if proc.returncode != 0:
        pytest.fail("node harness failed in %s\n%s" % (sb, proc.stderr.decode("utf-8", "replace")))
    return json.loads(proc.stdout.decode("utf-8"))


def test_node_check_module():
    for rel in ("js/replay/chart-store.js", "js/telemetry-charts.js"):
        r = subprocess.run([NODE, "--check", str(GUI / rel)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        assert r.returncode == 0, r.stderr.decode()


def test_replay_columns_equal_live_on_telemetry_data(out):
    assert out["n_charts"] == 9
    assert out["live_lengths"] == out["replay_lengths"]
    assert out["live_lengths"][0] == 3500 and out["live_lengths"][8] == 0
    assert out["equality"]["cmp"] > 50000
    assert out["equality"]["bad"] == 0


def test_join_rules(out):
    j = out["join"]
    # leg echo stale (> 1 s since the last echo) -> NaN, and only inside the gap
    assert j["nanInGap"] > 20 and j["finiteInGap"] == 0
    assert j["nanOutside"] == 0
    # hand is latest-before with no staleness: never NaN once a hand sample exists
    assert j["handNan"] == 0
    # the fixture bag has 7 motors: BB axes (7, 8) are never fed, like the live push loop
    assert j["axis7Samples"] == 0 or j["bbNan"] == j["axis7Samples"]


def test_store_swap_and_restore_are_exact(out):
    assert out["swap_equality"]["bad"] == 0
    assert out["inert"] is True
    assert out["exit_equality"]["bad"] == 0
    assert out["live_resumes"] is True


def test_buffer_is_immutable_across_rebuilds(out):
    im = out["immut"]
    assert all(im.values()), im
    assert out["open_chunk"]["second"] > out["open_chunk"]["first"] == 5


def test_playhead_moves_only_the_scale_and_span_is_clamped(out):
    assert out["window"]["span"] == 600 and out["window"]["cap"] == 600
    p = out["playhead"]
    assert p["centred"] and p["only_scale"] and p["span"] == pytest.approx(10.0)
    assert out["wheel_span"] == pytest.approx(600.0)
    assert out["wheel_centre_offset"] == pytest.approx(0.0, abs=1e-6)
    assert out["select"]["span"] == pytest.approx(8.0) and out["select"]["centred"]
    assert out["select"]["seek"] == pytest.approx(34.0, abs=1e-6)
    assert all(out["invalidate"].values())


def test_unit_toggle_during_replay_converts_parked_live_store(out):
    ut = out["unit_toggle"]
    assert ut["converted"] is True  # the toggle really changes pos/vel columns
    assert ut["vs_direct"] == pytest.approx(0.0, abs=1e-6)


def test_playhead_line_hook_installs_draws_from_value_and_removes(out):
    pl = out["playhead_line"]
    assert pl["before"] and not any(pl["before"])             # live charts carry no playhead hook
    assert all(pl["installed"]) and len(pl["installed"]) == 9  # installed on enter
    # panned window [10, 30], playhead 25 -> 3/4 of the plot box (left 100, width 400), NOT the centre
    assert len(pl["drawn"]) == 1 and pl["drawn"][0]["x"] == pytest.approx(400.0)
    assert pl["drawn"][0]["color"] == "#abcdef"                # the --replay-playhead var, trimmed
    assert pl["drawn"][0]["w"] >= 1
    assert pl["outside"] == 0                                  # playhead outside the window: no line
    assert all(pl["removed"])                                  # removed on exit
    assert pl["x_panned"] == pytest.approx(400.0) and pl["x_centre"] == pytest.approx(300.0)
    assert pl["x_left"] == pytest.approx(100.0)
    assert pl["x_out"] is None and pl["x_bad"] is None and pl["x_degenerate"] is None


def test_chart_dom_contracts_for_feedback_unit_a():
    """CSS contracts: banners follow the expanded minimap; the window select has a fixed width;
    the trackbar playhead and the chart playhead share one variable."""
    rc = (GUI / "css" / "replay.css").read_text()
    cc = (GUI / "css" / "charts.css").read_text()
    assert "#viewer-pane.minimap-expanded .replay-banner" in rc and "--minimap-width" in rc
    assert ":root { --replay-playhead:" in rc
    assert "background: var(--replay-playhead)" in rc
    sel = cc[cc.index("#chart-window-select {"):]
    sel = sel[:sel.index("}")]
    assert "width: 84px" in sel and "box-sizing: border-box" in sel


def test_two_tier_composed_data_has_gaps_for_missing_digests(out):
    t = out["tiers"]
    assert out["tiers_none"]["same_ts"]                       # no digests: exactly the zero-copy full tier
    assert t["mono"] and t["cols_len_ok"]                     # monotonic x, every column the same length
    assert t["n_before"] > 0 and t["n_after"] > 0             # digest bins on both sides of the full tier
    assert t["n_total"] == t["n_full"] + t["n_before"] + t["n_after"]
    # slot 0 | (slot 1 missing) | full | slot 3: a NaN point on the 10 s hole, none at the contiguous edge
    assert t["gap_between_slot0_and_full"] is True
    assert t["gap_between_full_and_slot3"] is False
    assert t["gap_points_outside_full"] == 1


def test_digest_bins_are_clipped_per_bin_at_the_resident_edge(out):
    c = out["clip"]
    assert c["rr0"] == pytest.approx(25.0)
    assert c["env_t"] == pytest.approx([20.5, 21.5, 22.5, 23.5, 24.5])   # the bins before the window are present
    assert c["inside"] == 0 and c["split"] == 5                          # none inside it


def test_envelope_hook_draws_only_over_digest_regions(out):
    h = out["env_hook"]
    assert not any(h["before"]) and all(h["installed"]) and len(h["installed"]) == 9
    assert h["order"].index("drawReplayEnvelope") < h["order"].index("drawReplayPlayhead")
    assert h["n_fills"] >= 2 and h["n_fills"] % 2 == 0        # one run per region (slot 0, slot 3) per signal
    assert h["alpha_ok"] and h["has_before"] and h["has_after"]
    assert h["inside_full"] == 0 and h["none_drawn"] == 0
    assert all(h["removed"])


def test_digest_installs_repaint_once_per_frame(out):
    c = out["coalesce"]
    assert set(c["burst_before_frame"]) == {0}                # a burst of installs repaints nothing yet
    assert set(c["burst_after_frame"]) == {1}                 # ... then exactly one setData per chart
    assert set(c["rebuild_immediate"]) == {1} and set(c["rebuild_after_frame"]) == {0}


def test_y_range_covers_the_digest_envelope_over_the_visible_window(out):
    r = out["range_env"]
    assert r["emax_finite"] is True and r["covers"] is True


def test_typed_derivation_hydrates_nothing_and_derives_each_chunk_once(out):
    d = out["derive_once"]
    assert d["hydrations"] == 0                       # robot_state / echo / hand are read from the typed columns
    assert d["resident_calls"] > 0
    assert d["digest_resident_extra"] == 0            # digest of a resident chunk reuses setResident's derivation
    assert d["far_calls"] > 0                         # a non-resident chunk is derived once for its digest ...
    assert d["promote_extra"] == 0                    # ... and not again when it becomes resident
    assert d["again_extra"] == 0


def test_y_range_never_stalls_uplots_tick_loop(out):
    """Owner 2026-10-11: 2.0 GB tab while scrubbing. A y span of a few ulps at |v| >= ~1e9 makes
    uPlot's numAxisSplits never advance; it grows one array to V8's max length (~1.1 GB, ~6 s)
    and throws "Invalid array length". Recipe confirmed in headless Chromium 154 with the vendored
    uPlot 1.6.31: data [1.79e9, 1.79e9 + 4.8e-7] and [3e9, 3e9 + 1e-6]. The harness mirrors
    findIncr + numAxisSplits; the OLD range formula must stall on those inputs (so the mirror is
    faithful) and yRangeFor must not, on any axis height / tick spacing tried."""
    y = out["y_split"]
    rows = {(r["a"], r["b"]): r for r in y["rows"]}
    assert rows[(1.79e9, 1.79e9 + 4.8e-7)]["oldWorst"] == -1
    assert rows[(3e9, 3e9 + 1e-6)]["oldWorst"] == -1
    for r in y["rows"]:
        assert r["worst"] >= 0, r                      # the tick loop terminates (-1 = stalled) on every axis size tried
        lo, hi = r["r"]
        assert hi - lo > max(abs(lo), abs(hi)) * y["Y_MIN_REL_SPAN"]
    # Ordinary data keeps the old range exactly (flat and spanning cases).
    for key in ((100, 100.5), (5, 5), (0, 0), (-2, 7)):
        assert rows[key]["same"] is True, rows[key]
    assert all(r == [0, 1] for r in y["nulls"])
