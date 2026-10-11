# -*- coding: utf-8 -*-
"""Source-scan pins for the Phase 5 live trail wiring in ``ros_ws/gui/js/main.js``.

A node harness would need the whole browser module graph (viewer, panels, DOM), so these pin the
load-bearing structure by source instead: the ``/balls`` subscription at 20 ms, the D4 rule that
the live handlers never write trails while ``clock.isReplay()``, the live-only feed resets, and the
settings-row hook beside the camera presets.
"""
from __future__ import annotations

import os
import re

_GUI_JS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'ros_ws', 'gui', 'js')


def _src(name):
    with open(os.path.join(_GUI_JS, name), encoding='utf-8') as f:
        return f.read()


def _func_body(src, name):
    m = re.search(r'(?:export )?function %s\([^)]*\) \{' % re.escape(name), src)
    assert m, name
    depth, i = 1, m.end()
    while depth:
        c = src[i]
        depth += (c == '{') - (c == '}')
        i += 1
    return src[m.end():i]


def test_balls_subscribed_at_20ms_with_the_ball_state_array_type():
    src = _src('main.js')
    assert re.search(r"ros\.subscribe\('balls',\s*'jugglebot_interfaces/msg/BallStateArray',\s*onBalls,\s*20\)", src)


def test_balls_is_in_the_topic_spy_exclusion_set():
    src = _src('main.js')
    block = re.search(r'const GUI_SUBSCRIBED_TOPICS = new Set\(\[(.*?)\]\)', src, re.S).group(1)
    assert "'balls'" in block


def test_on_balls_writes_trails_only_when_not_replay():
    body = _func_body(_src('main.js'), 'onBalls')
    assert re.search(r'if \(clock\.isReplay\(\)\) return;', body)
    assert body.index('isReplay') < body.index('beginBalls')
    for call in ('beginBalls', 'feed.ball(', 'endBalls'):
        assert call in body


def test_on_mocap_data_trail_lines_are_guarded_by_not_replay():
    body = _func_body(_src('main.js'), 'onMocapData')
    guard = body.index('if (!clock.isReplay())')
    for call in ('beginMocap', 'feed.marker(', 'endMocap'):
        assert body.index(call) > guard, call


def test_feed_reset_is_live_only_in_timeout_and_disconnect_blanking():
    src = _src('main.js')
    resets = [m.start() for m in re.finditer(r'f\.reset\(\)', src)]
    assert len(resets) == 2
    for pos in resets:
        assert 'if (!clock.isReplay())' in src[max(0, pos - 80):pos]


def test_init_trails_follows_init_mocap_markers_and_ui_follows_camera_presets():
    src = _src('main.js')
    assert re.search(r'initMocapMarkers\(\);\s*initTrails\(\);', src)
    assert re.search(r'initCameraPresets\(\);\s*initTrailSettingsUi\(\);', src)
