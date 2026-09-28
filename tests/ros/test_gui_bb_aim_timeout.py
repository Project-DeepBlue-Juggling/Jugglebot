# -*- coding: utf-8 -*-
"""GUI regression: an unanswered bb/aim must not freeze the widget forever.

2026-09-28 sitting: the owner reported that editing BB's Yaw/Pitch fields
while the state machine was IDLE "isn't working, and appears to brick/freeze
the GUI until I refresh it." rosbridge logged `bb/aim` service calls timing
out after its own 50 s server-side bound
(`rosbridge_websocket_lean.py::CALL_SERVICE_TIMEOUT_S`) with no reply —
ball_butler_node was alive but not processing callbacks that sitting (a
separate issue, tracked elsewhere).

Root cause in the GUI: `ros_ws/gui/js/ros-bridge.js::callService` sets no
client-side timeout, so `ros_ws/gui/js/bb-aim.js::sendAim` had nothing to
give up on short of rosbridge's 50 s bound — and rosbridge runs ONE thread
per browser tab that executes every incoming message (service calls AND
publishes) FIFO, so for as long as that one call sat unanswered, every OTHER
GUI action on the same tab queued up behind it too. See this repo's
`logbook/2026-09-15-rosbridge-hung-call-and-close-teardown.md` for the
server-side half of that mechanism (already bounded to 50 s there; this test
covers the GUI's own client-side half, which had no bound at all).

The fix adds a client-side timeout (`AIM_TIMEOUT_MS`) and a single-flight
guard (`renewInFlight`, now checked on both the renewal timer AND a fresh
submit) to `bb-aim.js`. This test drives the REAL, unmodified `bb-aim.js`
under node against a fake `ros-bridge.js` whose `callService` can be told to
hang forever, exactly like the fk-golden harness
(`tests/ros/test_gui_fk_golden.py`) drives the real `stewart-fk.js` — see
that file's docstring for why the sandbox copies sources verbatim rather
than reimplementing them.

Two scenarios (`tests/ros/js/bb_aim_harness.js`):
  - "concurrent": submit yaw, then (while that call is still pending)
    submit pitch. Before the fix, `sendAim` had no in-flight guard on the
    submit path (only the renewal timer checked `renewInFlight`), so this
    fired a SECOND real `callService` call while the first was still
    outstanding — exactly the pattern that piles a second call behind the
    first on rosbridge's single per-tab queue thread, extending the freeze.
  - "timeout": submit yaw against a call that never answers, then wait past
    the client-side timeout. Before the fix, `bb-aim.js`'s status text
    stayed blank forever (nothing ever settled the promise) — indistinguishable,
    from the operator's chair, from a locked-up GUI. After the fix it flips to
    an error message once the client-side timeout fires.
"""

from __future__ import annotations

import glob
import json
import os
import re
import shutil
import subprocess

import pytest

_TESTS_ROS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_TESTS_ROS_DIR))
_GUI_JS_DIR = os.path.join(_REPO_ROOT, 'ros_ws', 'gui', 'js')
_JS_DIR = os.path.join(_TESTS_ROS_DIR, 'js')
_HARNESS = os.path.join(_JS_DIR, 'bb_aim_harness.js')
_FAKE_ROS_BRIDGE = os.path.join(_JS_DIR, 'fake_ros_bridge.js')

#: bb-aim.js's own real dependencies, copied verbatim into the sandbox.
_REAL_SOURCES = ('bb-aim.js', 'event-store.js', 'geometry-config.js')


# ---------------------------------------------------------------------------
# node discovery — identical to test_gui_fk_golden.py, duplicated rather than
# imported so this file stays independently readable and because pytest
# collects test_*.py modules independently (no shared conftest hook for it).
# ---------------------------------------------------------------------------

def _find_node():
    found = shutil.which('node') or shutil.which('nodejs')
    if found:
        return found
    patterns = (
        os.path.expanduser('~/.nvm/versions/node/*/bin/node'),
        '/usr/local/bin/node',
        '/usr/bin/node',
        '/opt/node*/bin/node',
    )
    candidates = []
    for pattern in patterns:
        candidates.extend(glob.glob(pattern))
    for candidate in sorted(candidates, key=_path_version, reverse=True):
        if os.access(candidate, os.X_OK):
            return candidate
    return None


def _path_version(path):
    match = re.search(r'/v(\d+)\.(\d+)\.(\d+)/', path)
    return tuple(int(g) for g in match.groups()) if match else (0, 0, 0)


_NODE = _find_node()


def _node_major(binary):
    try:
        out = subprocess.check_output([binary, '--version'],
                                      stderr=subprocess.STDOUT).decode()
    except Exception:                                    # pragma: no cover
        return 0
    match = re.match(r'v(\d+)\.', out.strip())
    return int(match.group(1)) if match else 0


_NODE_MAJOR = _node_major(_NODE) if _NODE else 0

requires_node = pytest.mark.skipif(
    _NODE is None or _NODE_MAJOR < 14,
    reason='node >= 14 not found (looked on PATH, ~/.nvm, /usr/local, /usr, /opt)')


@pytest.fixture()
def sandbox(tmp_path):
    """A sandbox with the REAL bb-aim.js + deps, the fake ros-bridge.js at
    the path bb-aim.js actually imports (see fake_ros_bridge.js's docstring
    for why its `withTimeout` is an inlined copy rather than an import of
    the real one), and the harness — marked as an ESM package."""
    sb = str(tmp_path)
    for name in _REAL_SOURCES:
        shutil.copy(os.path.join(_GUI_JS_DIR, name), os.path.join(sb, name))
    shutil.copy(_FAKE_ROS_BRIDGE, os.path.join(sb, 'ros-bridge.js'))
    shutil.copy(_HARNESS, os.path.join(sb, 'bb_aim_harness.js'))
    with open(os.path.join(sb, 'package.json'), 'w', encoding='utf-8') as f:
        f.write('{"type": "module"}\n')
    return sb


def _extract_with_timeout(js_source):
    match = re.search(
        r'export function withTimeout\(.*?\n\}\n', js_source, re.DOTALL)
    assert match, 'withTimeout() not found'
    return match.group(0)


def test_fake_with_timeout_matches_the_real_one():
    """fake_ros_bridge.js inlines `withTimeout` (so the sandbox loads
    standalone against the PRE-FIX bb-aim.js/ros-bridge.js too, which has no
    such export) — pin it byte-identical to the real
    ros_ws/gui/js/ros-bridge.js so the two cannot silently drift apart."""
    with open(os.path.join(_GUI_JS_DIR, 'ros-bridge.js'), encoding='utf-8') as f:
        real = _extract_with_timeout(f.read())
    with open(_FAKE_ROS_BRIDGE, encoding='utf-8') as f:
        fake = _extract_with_timeout(f.read())
    assert real == fake, (
        'fake_ros_bridge.js::withTimeout has drifted from the real '
        'ros_ws/gui/js/ros-bridge.js::withTimeout — update the inlined copy')


def _run(sandbox, scenario, timeout):
    proc = subprocess.run(
        [_NODE, os.path.join(sandbox, 'bb_aim_harness.js'), scenario],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)
    if proc.returncode != 0:
        pytest.fail('node harness failed (exit %d) for scenario %r in %s\nstderr:\n%s'
                    % (proc.returncode, scenario, sandbox,
                       proc.stderr.decode('utf-8', 'replace')))
    return json.loads(proc.stdout.decode('utf-8'))


@requires_node
def test_concurrent_submit_does_not_stack_a_second_call(sandbox):
    """Submitting a second axis while the first is still pending must not
    fire a second real bb/aim call — that second call would queue up behind
    the first on rosbridge's single per-tab FIFO thread and extend, not
    shorten, how long the connection stays blocked."""
    result = _run(sandbox, 'concurrent', timeout=15)
    assert result['calls_after_first_submit'] == 1
    assert result['calls_after_second_submit'] == 1, (
        'a second bb/aim call was made while the first was still pending — '
        'the two would serialise on rosbridge\'s per-tab queue thread')
    assert result['status_after_second_submit'], 'no feedback shown for the refused second submit'
    assert 'pending' in result['status_after_second_submit'].lower()


@requires_node
def test_unanswered_call_recovers_after_client_timeout(sandbox):
    """An unanswered bb/aim must surface to the operator well before
    rosbridge's 50 s server-side bound, and must not linger blank/stuck."""
    result = _run(sandbox, 'timeout', timeout=15)
    # While genuinely pending, nothing has been claimed yet.
    assert result['status_immediately'] == ''
    # A single call was made, and no extra ones were queued up during the wait.
    assert result['calls_total'] == 1
    # The status must have moved off blank/pending WITHOUT the operator doing
    # anything — this is the "GUI stays responsive" assertion. Before the
    # fix this stays '' forever (nothing ever settles the promise).
    status = result['status_after_wait']
    assert status, 'status stayed blank — the widget never recovered from the unanswered call'
    assert 'timed out' in status.lower() or 'failed' in status.lower(), status
