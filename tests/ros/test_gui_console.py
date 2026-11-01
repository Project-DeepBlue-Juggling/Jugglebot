# -*- coding: utf-8 -*-
"""CONSOLE view (JB-130): the read-only /rosout log viewer in the topics panel.

Drives the REAL console-logic.js / console-view.js / udp-traffic.js under node with a fake DOM
(tests/ros/js/console_harness.js).  Pins: three modes + persisted CONSOLE; severity chips and
search combine; message text is escaped; ring cap; replay current/prev/next hit selection
relative to a playhead; and the replay allowlist/policy carry /rosout.
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_GUI = os.path.abspath(os.path.join(_HERE, '..', '..', 'ros_ws', 'gui'))
_GUI_JS = os.path.join(_GUI, 'js')
_NODE = shutil.which('node') or shutil.which('nodejs')

pytestmark = pytest.mark.skipif(_NODE is None, reason='node not found')


@pytest.fixture(scope='module')
def r(tmp_path_factory):
    sb = str(tmp_path_factory.mktemp('console'))
    for name in ('udp-traffic.js', 'console-view.js', 'console-logic.js', 'clock.js'):
        shutil.copy(os.path.join(_GUI_JS, name), os.path.join(sb, name))
    with open(os.path.join(sb, 'package.json'), 'w') as f:
        f.write('{"type": "module"}\n')
    shutil.copy(os.path.join(_HERE, 'js', 'console_harness.js'), sb)
    proc = subprocess.run([_NODE, os.path.join(sb, 'console_harness.js')],
                          capture_output=True, text=True, timeout=30)
    assert proc.returncode == 0, proc.stderr
    return json.loads(proc.stdout.strip().splitlines()[-1])


def test_three_modes_and_console_persists(r):
    assert r['modes'] == ['ros', 'udp', 'console']
    assert r['parse'] == {'console': 'console', 'garbled': 'ros', 'nul': 'ros'}
    assert r['title'] == 'Console'                      # restored from the persisted mode
    assert r['display'] == {'console': '', 'udp': 'none', 'ros': 'none'}


def test_console_button_in_index_html():
    html = open(os.path.join(_GUI, 'index.html'), encoding='utf-8').read()
    assert 'data-mode="console"' in html and 'id="console-container"' in html


def test_filter_and_search_combine(r):
    assert r['f_all'] == [0, 1, 2, 3, 4]
    assert r['f_q'] == [0, 1, 3, 4]                     # name OR message, case-insensitive
    assert r['f_q_nodeonly'] == [1]                     # node-name match alone counts
    assert r['f_q_warnoff'] == [0, 3, 4]                # chip off removes the WARN match
    assert r['f_set'] == [2, 4]                         # a Set of enabled keys works too
    assert r['counts_q'] == {'DEBUG': 1, 'INFO': 1, 'WARN': 1, 'ERROR': 0, 'FATAL': 1}
    assert sum(r['counts_all'].values()) == 5
    assert r['lvl'] == ['DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL', 'DEBUG', 'INFO', 'FATAL']


def test_text_is_escaped(r):
    assert '<script>' not in r['html'] and '<img' not in r['html']
    assert 'n&lt;script&gt;' in r['html'] and '&lt;img src=x onerror=alert(1)&gt; &amp; &quot;q&quot;' in r['html']
    assert '<mark>img</mark>' in r['html_hl'] and '<img' not in r['html_hl']
    assert r['hl'] == 'a&lt;<mark>b</mark>&gt;<mark>B</mark>'
    assert r['norm']['t'] == 5.5 and r['norm']['name'] == 'x'
    assert r['parseEnabled'][0] == dict.fromkeys(['DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'], True)
    assert r['parseEnabled'][1] == r['parseEnabled'][0]
    assert r['parseEnabled'][2]['INFO'] is False and r['parseEnabled'][2]['WARN'] is True


def test_ring_cap(r):
    assert r['ring'] == {'len': r['ring']['cap'], 'first': '123', 'last': str(r['ring']['cap'] + 122), 'cap': 5000}
    assert r['viewHost'] is True


def test_replay_hit_selection(r):
    h = r['hit']
    assert h['hits'] == [0, 2, 4]
    assert (h['cur_before'], h['cur_mid'], h['cur_on']) == (0, 2, 2)
    assert (h['prev_mid'], h['prev_on'], h['prev_start']) == (2, 0, -1)    # strictly before; on-the-line skips it
    assert (h['next_mid'], h['next_on'], h['next_end']) == (4, 4, -1)
    assert h['none'] == [-1, -1, -1]


def test_hover_time_pure(r):
    h = r['hover']
    assert h['row'] == 12.5
    assert h['nan'] is None and h['missing'] is None        # non-finite data-t -> no marker
    assert h['nonrow'] is None and h['nul'] is None and h['nofn'] is None


def test_hover_publishes_only_in_replay(r):
    h = r['hover']
    assert {'mouseover', 'mouseout'} <= set(h['registered'])
    assert h['live'] == []                                    # live mode has no trackbar: nothing published
    assert h['replay'] == [7, None, 9]                        # row, leave, row-to-row
    assert h['hidden'] == [7, None, 9, None]                  # hiding the view clears (no mouseout fires)
    assert h['after_exit']['calls'] == [7, None, 9, None, None]  # detach clears once
    assert h['after_exit']['extra'] == 0                      # and later hovers publish nothing


def test_replay_carries_rosout():
    allow = open(os.path.join(_GUI_JS, 'replay', 'allowlist.js'), encoding='utf-8').read()
    pol = open(os.path.join(_GUI_JS, 'replay', 'policy.js'), encoding='utf-8').read()
    assert "'/rosout'" in allow and "'/rosout':" in pol
    main = open(os.path.join(_GUI_JS, 'main.js'), encoding='utf-8').read()
    assert "ros.subscribe('rosout', 'rcl_interfaces/msg/Log', consoleOnLog, 0)" in main
    assert "'rosout'," in main                                              # GUI_SUBSCRIBED_TOPICS
