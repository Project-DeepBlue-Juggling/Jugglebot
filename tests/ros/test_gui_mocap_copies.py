"""The GUI side of JB-76 option 1, pinned at source level (2026-10-28).

Live, the GUI must take mocap_node's 20 Hz ``gui/`` copies and never the
300 Hz originals (a decoded subscription OR a topic-monitor spy on
``/mocap_data``, ``/rigid_body_poses`` or ``/bb/markers`` puts ~900 msg/s back
through rosbridge: 102 % -> 53 % of a core was the measured difference);
replay maps the bag's originals onto the copy handlers, and only replay does.
The JS is not importable from the mocked-ROS gate, so this reads the source.

Entry: logbook/2026-10-28-gui-mocap-copies-cut-rosbridge-load.md
"""
from __future__ import annotations

import pathlib
import re

GUI = pathlib.Path(__file__).resolve().parents[2] / 'ros_ws' / 'gui' / 'js'
ORIGINALS = ('mocap_data', 'rigid_body_poses', 'bb/markers')


def _src(name):
    return (GUI / name).read_text(encoding='utf-8')


def test_the_gui_never_subscribes_a_300_hz_mocap_original_live():
    main = _src('main.js')
    for t in ORIGINALS:
        assert not re.search(r"subscribe\(\s*['\"]/?%s['\"]" % re.escape(t), main), t
    for t in ('gui/mocap_data', 'gui/rigid_body_poses'):
        assert re.search(r"subscribe\(\s*['\"]%s['\"]" % re.escape(t), main), t


def test_discovery_skips_the_reported_topics_so_no_spy_is_created():
    panels = _src('panels.js')
    m = re.search(r"export const REPORTED_TOPICS = new Set\(\[([^\]]*)\]\)", panels)
    assert m, 'REPORTED_TOPICS set missing from panels.js'
    assert set(re.findall(r"'([^']+)'", m.group(1))) == set(ORIGINALS)
    main = _src('main.js')
    assert 'if (REPORTED_TOPICS.has(name)) continue;' in main, \
        'runTopicDiscovery no longer skips REPORTED_TOPICS (a cbor-raw spy would come back)'


def test_replay_aliases_the_originals_onto_the_copies_and_only_in_dispatch_local():
    rb = _src('ros-bridge.js')
    block = re.search(r"const REPLAY_TOPIC_ALIASES = Object\.freeze\(\{(.*?)\}\)", rb, re.S)
    assert block, 'REPLAY_TOPIC_ALIASES missing'
    pairs = dict(re.findall(r"'(/?[\w/]+)'\s*:\s*'(/?[\w/]+)'", block.group(1)))
    norm = {k.lstrip('/'): v.lstrip('/') for k, v in pairs.items()}
    assert norm == {'mocap_data': 'gui/mocap_data',
                    'rigid_body_poses': 'gui/rigid_body_poses'}, norm
    lines = rb.splitlines()
    uses = [i for i, l in enumerate(lines) if 'REPLAY_TOPIC_ALIASES[' in l]
    start = next(i for i, l in enumerate(lines) if 'function dispatchLocal(' in l)
    end = next((i for i, l in enumerate(lines[start + 1:], start + 1)
                if l.startswith('export function') or l.startswith('function ')), len(lines))
    assert uses and all(start < i < end for i in uses), \
        'the alias must be applied inside dispatchLocal only, never to live traffic'
