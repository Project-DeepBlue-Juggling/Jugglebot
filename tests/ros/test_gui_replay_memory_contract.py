# -*- coding: utf-8 -*-
"""Source-contract tests for the replay memory block (owner-approved 2026-10-10).

Grep-based, like ``test_gui_clock_contract.py``. Root causes guarded:

* The live "session buffer" (a 600 s in-memory ring fed by a tap inside every ros-bridge subscription)
  cost heap on every live message for a feature the owner dropped; re-adding the tap silently
  re-introduces that cost on the live GUI.
* The catching cone hid on ``performance.now()`` staleness, which keeps running while a replay is paused,
  so a paused replay lost the cone. Staleness must follow the replay-aware clock.
"""
from __future__ import annotations

import glob
import os
import re

_GUI_JS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', '..', 'ros_ws', 'gui', 'js')


def _text(name):
    with open(os.path.join(_GUI_JS, name), encoding='utf-8') as f:
        return f.read()


def test_ros_bridge_has_no_session_buffer_tap():
    src = _text('ros-bridge.js')
    assert 'replay/session' not in src
    assert 'getSessionBuffer' not in src
    assert not os.path.exists(os.path.join(_GUI_JS, 'replay', 'session.js'))


def test_no_session_buffer_symbol_remains_in_gui_js():
    bad = []
    pat = re.compile(r'getSessionBuffer|SessionBufferSource|createSessionBuffer|SESSION_BUFFER_SEC')
    for path in sorted(glob.glob(os.path.join(_GUI_JS, '**', '*.js'), recursive=True)):
        with open(path, encoding='utf-8') as f:
            for i, line in enumerate(f, 1):
                if pat.search(line):
                    bad.append('%s:%d: %s' % (os.path.relpath(path, _GUI_JS), i, line.strip()))
    assert not bad, 'session-buffer symbol survives:\n' + '\n'.join(bad)


def test_catching_cone_staleness_uses_the_replay_aware_clock():
    src = _text('catching-cone-model.js')
    assert re.search(r"import \* as clock from '\./clock\.js'", src)
    assert re.search(r'clock\.now\(\)\s*-\s*lastSeen\s*>\s*1500', src)
    assert 'performance.now(' not in src and 'Date.now(' not in src
    assert 'wall-clock:' not in src
