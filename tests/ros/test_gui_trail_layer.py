# -*- coding: utf-8 -*-
"""trails.js (ribbon layer) + trail-settings.js under node with the vendored Three.js.

Sandbox: shipped trails.js / trail-settings.js plus ``node_modules/three`` (a copy of
``ros_ws/gui/lib/three/build/three.module.js`` with a package.json), so the bare
``import 'three'`` resolves exactly as the browser importmap does.
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
THREE = REPO / "ros_ws" / "gui" / "lib" / "three" / "build" / "three.module.js"
HARNESS = REPO / "tests" / "ros" / "js" / "trail_layer_harness.js"


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
    sb = tmp_path_factory.mktemp("trail_layer")
    for name in ("trails.js", "trail-settings.js"):
        shutil.copy(JS / name, sb / name)
    t = sb / "node_modules" / "three"
    t.mkdir(parents=True)
    shutil.copy(THREE, t / "three.module.js")
    (t / "package.json").write_text('{"name": "three", "type": "module", "main": "three.module.js"}\n')
    shutil.copy(HARNESS, sb / "trail_layer_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "trail_layer_harness.js")],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout)


def test_decimation_200hz_to_100hz(out):
    assert out["dec_active"] == 1
    assert out["dec_decimated"] == 200            # every other push skipped
    assert out["dec_vertices"] == 2 * 200


def test_ring_wrap_keeps_newest_contiguous_range(out):
    assert out["wrap_vertices"] == 2 * 64          # capacity samples kept
    assert out["wrap_last_x"] == pytest.approx(199.0, abs=1e-2)
    assert out["wrap_first_x"] == pytest.approx(199.0 - 63, abs=1e-2)
    assert out["wrap_contiguous"] is True and out["wrap_max_end"] is True


def test_ended_track_fades_then_is_released(out):
    assert out["end_active_in_tail"] == 1 and out["end_drawing_in_tail"] is True
    assert out["end_active_after_tail"] == 0 and out["end_free_after"] == 4
    assert out["endmsg_active"] == 2
    # one segment (2 samples) after the revive; the control without end() draws three (4 samples)
    assert out["revive_indices"] == 6
    assert out["revive_keep_all"] == 18


def test_pool_exhaustion_drops_without_growth(out):
    assert out["pool_active"] == 3 and out["pool_dropped"] == 7 and out["pool_meshes_same"] is True


def test_reset_backwards_tail0_and_coordinates(out):
    assert out["reset_before"] == 4 and out["reset_after"] == 0 and out["reset_free"] == 4
    assert out["backwards_vertices"] == 0 and out["backwards_vertices2"] == 4
    assert out["tail0"] == 0
    assert out["pos_first"] == pytest.approx([1.0, 3.0, -2.0])


def test_settings_clamp_snap_default_and_no_storage(out):
    s = out["settings"]
    assert s["default"] == 1000 and s["no_storage"] is True
    assert (s["set_1234"], s["set_neg"], s["set_big"], s["set_nan"], s["set_2500"]) == (1200, 0, 100000, 1000, 2500)
    assert s["seen"] == [1200, 0, 5000, 1000, 2500]    # unsubscribed before the 300 write
    assert s["throwing_set"] == 700
    assert s["stored"] == "1700"


def test_settings_max_effective_near_and_listener_shape(out):
    r = out["settings2"]
    assert r["max0"] == 5000
    assert r["live"] == [60000, 5000, 5000]           # stored survives, effective clamps to the live max
    assert r["long"] == [60000, 60000, 5000]          # replay max: effective follows, NEAR tier stays 5 s
    assert r["shrunk"] == [60000, 30000, 5000]
    assert r["back"] == [60000, 5000, 5000]
    assert r["regrown"] == 60000                      # the stored request returns on re-grow
    # events fire on EFFECTIVE or MAX change only (setTailMs(60000) at live max 5000: effective 1000->5000;
    # the repeated setTailMaxMs(30000) is silent), shape {effectiveMs, maxMs, storedMs}
    assert r["ev"] == [[5000, 5000, 60000], [60000, 1086000, 60000], [30000, 30000, 60000],
                       [5000, 5000, 60000], [60000, 1086000, 60000]]
    assert r["snap"] == [4900, 5000, 60000, 0, 1000]
    assert r["max_bad"] == 5000
    assert r["persisted"] == "120000"


def test_slider_mapping_monotonic_roundtrip_endpoints(out):
    for max_ms, m in out["settings2"]["maps"].items():
        assert m["mono"] is True, max_ms
        assert m["first"] == 0 and m["last"] == int(max_ms), max_ms
        assert m["worst"] < 0.06, (max_ms, m["worst"])        # within one snap step (>= 100 ms: 5 s steps at >60 s)
        assert abs(m["posMax"] - 1.0) < 1e-9, max_ms
    assert out["settings2"]["maps"]["20000"]["at600"] == 5000      # linear part ends at 60 % of travel
    assert abs(out["settings2"]["maps"]["20000"]["pos5"] - 0.6) < 1e-9
    assert out["settings2"]["maps"]["3000"]["at600"] == 1800       # max <= 5 s: linear throughout
    assert out["settings2"]["maps"]["5000"]["at600"] == 3000


def test_settings_formatting(out):
    r = out["settings2"]
    assert r["fmt"] == ["off", "0.8 s", "5.0 s", "12 s", "1:00", "1:00", "3:20", "18:06", "2:02:00"]
    assert r["fmtMax"] == ["5 s", "18:06", "30 s"]


def test_slider_right_end_reaches_the_whole_non_round_max(out):
    # audit 2026-10-11: setTailMs used to re-snap 1,081,300 ms to 1,080,000 (tail 1.3 s short of the start)
    assert out["settings2"]["wholeMax"] == [1081300, 1081300, 1]
    assert out["settings2"]["longer"] == [2000000, 1081300]
