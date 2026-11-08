# -*- coding: utf-8 -*-
"""Every GUI chart axis shows at least two ticks at every size and zoom (BUG-37).

``tests/ros/js/axis_ticks_harness.js`` runs the real ``ros_ws/gui/js/axis-ticks.js``
under node against a replay of uPlot's step choice over a grid of plot sizes,
spans and offsets; the source-scan half pins that both uPlot chart builders
(``telemetry-charts.js``, ``can-traffic.js``) use it on every axis.
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
JS = REPO / "ros_ws" / "gui" / "js"
HARNESS = REPO / "tests" / "ros" / "js" / "axis_ticks_harness.js"


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


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    if NODE is None:
        pytest.skip("node not installed")
    sb = tmp_path_factory.mktemp("axisticks")
    shutil.copy(JS / "axis-ticks.js", sb / "axis-ticks.js")
    shutil.copy(HARNESS, sb / "harness.js")
    (sb / "package.json").write_text('{"type": "module"}')
    res = subprocess.run([NODE, str(sb / "harness.js")], capture_output=True, text=True, timeout=120,
                         env={**os.environ, "TZ": "Australia/Sydney"})
    assert res.returncode == 0, res.stderr
    return json.loads(res.stdout)


def test_ladder_ratios_match_the_constants_the_guarantee_uses(out):
    assert out["numRatio"] == pytest.approx(out["NUM_LADDER_RATIO"])
    assert out["timeRatio"] == pytest.approx(out["TIME_LADDER_RATIO"])
    assert out["stockTimeRatio"] == pytest.approx(5)   # why TIME_INCRS adds 2 s and 2 min
    assert out["timeSorted"] is True


def test_at_least_two_ticks_everywhere(out):
    assert out["num"]["worst"] >= 2, out["num"]
    assert out["time"]["worst"] >= 2, out["time"]
    assert out["num"]["cases"] > 10000 and out["time"]["cases"] > 3000


def test_stock_defaults_do_show_one_tick(out):
    """The bug reproduces in the same sweep without the fix."""
    assert out["before"]["num"] <= 1
    assert out["before"]["time"] <= 1


def test_large_charts_keep_the_default_spacing(out):
    assert out["bigNumSpace"] == 30
    assert out["bigTimeSpace"] == 50


def test_time_labels_gain_fractional_seconds_below_one_second_steps(out):
    lab = out["labels"]
    assert lab["whole"] == ["12:34:56", "12:35:01"]
    assert lab["tenths"] == ["12:34:56.0", "12:34:56.2", "12:34:56.4"]
    assert lab["hundredths"] == ["12:34:56.05", "12:34:56.10"]
    assert lab["ms"] == ["12:34:56.001", "12:34:56.003"]
    assert lab["single"] == ["12:34:56"]


@pytest.mark.parametrize("module, n_numeric", [("telemetry-charts.js", 1), ("can-traffic.js", 1)])
def test_chart_builders_use_it_on_every_axis(module, n_numeric):
    src = (JS / module).read_text()
    assert "from './axis-ticks.js'" in src
    assert src.count("incrs: TIME_INCRS") == 1
    assert src.count("space: timeTickSpace(50)") == 1
    assert "timeTickLabels(splits)" in src
    assert src.count("space: numTickSpace(30)") == n_numeric
    # no hand-rolled HH:MM:SS formatter left behind
    assert not re.search(r"getSeconds\(\)\)?\.padStart", src)
