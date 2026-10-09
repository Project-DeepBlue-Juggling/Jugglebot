# -*- coding: utf-8 -*-
"""Replay seek/exit must empty the CAN and UDP traffic panels' sample rings.

Drives the REAL can-traffic.js / udp-traffic.js (tests/ros/js/traffic_reset_harness.js)
under node with a fake DOM + fake uPlot, in the fk-harness pattern.  After a
reset the next sample must be treated as the first: no ring columns left over,
and for UDP no rate (a single sample has no pair, so '--', never a spike or a
negative difference against replayed counters).
"""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_GUI_JS = os.path.abspath(os.path.join(_HERE, '..', '..', 'ros_ws', 'gui', 'js'))
_NODE = shutil.which('node') or shutil.which('nodejs')

pytestmark = pytest.mark.skipif(_NODE is None, reason='node not found')


@pytest.fixture(scope='module')
def result(tmp_path_factory):
    sb = str(tmp_path_factory.mktemp('traffic_reset'))
    for name in ('can-traffic.js', 'udp-traffic.js', 'clock.js', 'geometry-config.js'):
        shutil.copy(os.path.join(_GUI_JS, name), os.path.join(sb, name))
    with open(os.path.join(sb, 'telemetry-charts.js'), 'w') as f:
        f.write('export function nanGaps() { return []; }\n')
    with open(os.path.join(sb, 'package.json'), 'w') as f:
        f.write('{"type": "module"}\n')
    shutil.copy(os.path.join(_HERE, 'js', 'traffic_reset_harness.js'), sb)
    proc = subprocess.run([_NODE, os.path.join(sb, 'traffic_reset_harness.js')],
                          capture_output=True, text=True, timeout=30)
    assert proc.returncode == 0, proc.stderr
    return json.loads(proc.stdout.strip().splitlines()[-1])


def test_can_ring_emptied_and_next_sample_is_first(result):
    assert result['can_before']['cols'] == 3
    assert result['can_after_reset']['cols'] == 0
    assert result['can_after_reset']['rates'] == [0, 0, 0]
    assert result['can_next']['cols'] == 1
    assert result['can_next']['rate2'] == [100]


def test_udp_ring_emptied_and_next_sample_has_no_rate(result):
    before = re.findall(r'<td class="col-rate[^"]*">([^<]*)</td>', result['udp_before'])
    assert before and before[0].strip() != '--', before   # premise: a rate was shown
    assert 'Waiting for udp_diag' in result['udp_after_reset']
    nxt = result['udp_next']
    assert '<table' in nxt
    rates = re.findall(r'<td class="col-rate[^"]*">([^<]*)</td>', nxt)
    assert rates and all(r.strip() == '--' for r in rates), rates
