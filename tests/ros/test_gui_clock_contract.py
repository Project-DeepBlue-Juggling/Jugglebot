# -*- coding: utf-8 -*-
"""Grep contract for the GUI's time sources (replay design § 2).

Every ``Date.now(`` / ``performance.now(`` / ``new Date()`` in
``ros_ws/gui/js/*.js`` (outside ``clock.js``; ``lib/`` is not scanned) must either
be migrated to ``clock.now()`` or carry a same-line ``// wall-clock: <reason>``
marker, so a new site cannot silently read wall time. ``new Date(x)`` with an
argument (formatters) is exempt. The nine data-staleness watchdogs must use
``clock.setTimeout`` / ``clock.clearTimeout``.
"""
from __future__ import annotations

import glob
import os
import re

_GUI_JS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', '..', 'ros_ws', 'gui', 'js')
_WALL_RE = re.compile(r'Date\.now\(|performance\.now\(|new Date\(\)')
_MARKER_RE = re.compile(r'//\s*wall-clock:\s*\S')


def _read(name):
    with open(os.path.join(_GUI_JS, name), encoding='utf-8') as f:
        return f.read().split('\n')


def test_every_wall_clock_read_is_marked():
    bad = []
    for path in sorted(glob.glob(os.path.join(_GUI_JS, '*.js'))):
        name = os.path.basename(path)
        if name == 'clock.js':
            continue
        for i, line in enumerate(_read(name), 1):
            if _WALL_RE.search(line) and not _MARKER_RE.search(line):
                bad.append('%s:%d: %s' % (name, i, line.strip()))
    assert not bad, ('wall-clock read without a `// wall-clock: <reason>` marker '
                     '(use clock.now() if it should follow replay):\n' + '\n'.join(bad))


#: file -> (regex locating the arming call, expected function context)
_WATCHDOGS = [
    ('main.js', r'handTelemTimeout = clock\.setTimeout\(', 'hand pill'),
    ('main.js', r'mocapConnTimeout = clock\.setTimeout\(', 'mocap pill'),
    ('main.js', r'legEchoTimeout = clock\.setTimeout\(', 'leg-echo pill'),
    ('panels.js', r'motionTimeout = clock\.setTimeout\(', 'motion panel'),
    ('can-traffic.js', r'profileStaleTimer = clock\.setTimeout\(', 'CAN profile'),
    ('can-traffic.js', r'healthStaleTimer = clock\.setTimeout\(', 'CAN health'),
    ('udp-traffic.js', r'diagStaleTimer = clock\.setTimeout\(', 'UDP diag'),
    ('udp-traffic.js', r'healthStaleTimer = clock\.setTimeout\(', 'UDP health'),
    ('hardware-versions.js', r'staleTimer = clock\.setTimeout\(', 'hardware versions'),
]


def test_watchdogs_use_clock_set_timeout():
    for fname, pat, what in _WATCHDOGS:
        text = '\n'.join(_read(fname))
        assert re.search(pat, text), '%s watchdog (%s) not on clock.setTimeout' % (what, fname)


def test_watchdog_files_import_clock_and_clear_via_clock():
    for fname in {w[0] for w in _WATCHDOGS}:
        lines = _read(fname)
        assert any(re.match(r"import \* as clock from '\./clock\.js';", l) for l in lines), fname
        # the watchdog timer variables are only cleared through clock.clearTimeout
        for i, l in enumerate(lines, 1):
            m = re.search(r'(?<![\w.])clearTimeout\((\w*(?:Timeout|StaleTimer|staleTimer))\)', l)
            assert not m, '%s:%d bare clearTimeout on a watchdog: %s' % (fname, i, l.strip())
