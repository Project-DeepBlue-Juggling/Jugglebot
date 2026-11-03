# -*- coding: utf-8 -*-
"""JB-159: hover callout pills must not overlap.

Drives the REAL ros_ws/gui/js/callout-layout.js under node (copied into a tmp dir with an ES-module
package.json); a source grep pins that both chart modules import and call it.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_JS = os.path.abspath(os.path.join(_HERE, '..', '..', 'ros_ws', 'gui', 'js'))
_NODE = shutil.which('node') or shutil.which('nodejs')

pytestmark = pytest.mark.skipif(_NODE is None, reason='node not found')


@pytest.fixture(scope='module')
def layout(tmp_path_factory):
    sb = str(tmp_path_factory.mktemp('callout_layout'))
    shutil.copy(os.path.join(_JS, 'callout-layout.js'), sb)
    with open(os.path.join(sb, 'package.json'), 'w') as f:
        f.write('{"type": "module"}\n')
    with open(os.path.join(sb, 'run.js'), 'w') as f:
        f.write("import { layoutCallouts } from './callout-layout.js';\n"
                "const a = JSON.parse(process.argv[2]);\n"
                "console.log(JSON.stringify(layoutCallouts(a[0], a[1], a[2])));\n")

    def run(desired, h, plot_h):
        p = subprocess.run([_NODE, os.path.join(sb, 'run.js'), json.dumps([desired, h, plot_h])],
                           capture_output=True, text=True, timeout=30)
        assert p.returncode == 0, p.stderr
        return json.loads(p.stdout.strip().splitlines()[-1])
    return run


def test_close_pills_separate_and_keep_order(layout):
    out = layout([100, 102], 16, 300)
    assert out[1] - out[0] >= 16 - 1e-9
    assert out[0] < out[1]


def test_input_order_is_preserved_per_index(layout):
    out = layout([102, 100], 16, 300)          # index 0 is the lower one on screen
    assert out[0] > out[1] and out[0] - out[1] >= 16 - 1e-9


def test_far_apart_unchanged(layout):
    assert layout([50, 150, 250], 16, 300) == [50, 150, 250]


def test_bottom_cluster_pushed_up_inside_plot(layout):
    out = layout([298, 299, 300], 16, 300)
    s = sorted(out)
    assert s[-1] <= 300 - 8 + 1e-9
    assert all(b - a >= 16 - 1e-9 for a, b in zip(s, s[1:]))


def test_too_many_stack_from_top_without_nan(layout):
    out = layout([10, 11, 12, 13], 16, 40)     # 4 pills need 48 px, plot is 40
    assert all(isinstance(v, (int, float)) and v == v for v in out)
    assert out == [8, 24, 40, 56]


def test_empty(layout):
    assert layout([], 16, 300) == []


@pytest.mark.parametrize('mod', ['telemetry-charts.js', 'can-traffic.js'])
def test_chart_modules_import_and_call_layout(mod):
    src = open(os.path.join(_JS, mod), encoding='utf-8').read()
    assert "from './callout-layout.js'" in src
    assert 'layoutCallouts(' in src
