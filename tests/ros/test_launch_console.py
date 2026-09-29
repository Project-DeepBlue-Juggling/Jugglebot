"""The operator console on launch's screen handler (jugglebot/launch_console.py).

What is pinned: the screen format (local clock time, short node name, no
``[skill_node-10]`` prefix, WARN/ERROR tags), DEBUG never on the screen, the
third-party reductions (rosbridge, rosbag2, launch lifecycle), the CONTRACT
that jugglebot's own messages are never reworded or hidden, the fail-safe
(a console bug prints the stock line, never loses it), and — end to end
through a real Foxy ``LaunchService`` — that the screen changes while
launch.log keeps every raw line.
"""

from __future__ import annotations

import io
import logging
import os
import re
import subprocess
import sys
import textwrap

import pytest

from jugglebot import launch_console as lc

T = 1790673112.036   # an rcutils stamp from the 2026-09-29 19:11 sitting
CLOCK = r'\d\d:\d\d:\d\d\.\d{3}'


def _screen(lines, colour=False):
    """What the screen shows for these stock-launch screen lines."""
    return lc.render_lines(lines, colour=colour)


def _rc(sev, logger, msg, proc=None, n=10, t=T):
    return '[%s-%d] [%s] [%.9f] [%s]: %s' % (proc or logger, n, sev, t,
                                             logger, msg)


# ── format ─────────────────────────────────────────────────────────────────

def test_a_node_line_gets_clock_time_a_short_name_and_no_process_prefix():
    out = _screen([_rc('INFO', 'skill_node', 'skill_node ready')])
    assert out == ['%s %-12s skill_node ready' % (lc.clock(T), 'skill')]
    assert re.match(CLOCK + ' ', out[0])
    assert '[skill_node-10]' not in out[0] and '1790673112' not in out[0]


def test_short_names():
    assert lc.short_name('trajectory_node') == 'trajectory'
    assert lc.short_name('teensy_bridge_node') == 'teensy'
    assert lc.short_name('catch_correlation_node') == 'cone'
    assert lc.short_name('rosbridge_websocket') == 'rosbridge'
    assert lc.short_name('rosbag2_transport') == 'rosbag'
    assert lc.short_name('launch.user') == 'launch'


def test_warn_and_error_are_tagged_and_info_is_not():
    info, warn, err = _screen([
        _rc('INFO', 'teensy_bridge_node', 'a'),
        _rc('WARN', 'teensy_bridge_node', 'b'),
        _rc('ERROR', 'teensy_bridge_node', 'c')])
    assert info.endswith('teensy       a')
    assert warn.endswith('teensy       WARN  b')
    assert err.endswith('teensy       ERROR c')


def test_colour_marks_warnings_yellow_and_errors_red():
    warn, err = _screen([_rc('WARN', 'skill_node', 'w'),
                         _rc('ERROR', 'skill_node', 'e')], colour=True)
    assert '\x1b[33m' in warn and 'w\x1b[0m' in warn
    assert '\x1b[31m' in err and 'e\x1b[0m' in err


def test_debug_never_reaches_the_screen():
    assert _screen([_rc('DEBUG', 'skill_node', 'detail')]) == []


def test_a_raw_line_keeps_its_process_name_and_the_last_stamp():
    out = _screen([_rc('ERROR', 'skill_node', 'boom'),
                   '[skill_node-10] Traceback (most recent call last):'])
    assert out[1] == '%s %-12s Traceback (most recent call last):' % (
        lc.clock(T), 'skill')


# ── the contract: jugglebot's own messages are never reworded or hidden ──

_OURS = ('skill_node', 'trajectory_node', 'teensy_bridge_node',
         'orchestrator_node', 'ball_tracker_node', 'ball_butler_node',
         'mocap_node', 'catch_correlation_node', 'spacemouse_handler')


@pytest.mark.parametrize('node', _OURS)
def test_our_own_lines_pass_verbatim_even_when_they_look_like_third_party(node):
    # Every third-party pattern the rules match, said by one of OUR nodes.
    for msg in ('Subscribed to topic /x', 'Client connected. 1 clients total.',
                'process started with pid [1]', 'WebSocketClosedError: x',
                'Listening for topics...', 'Hidden topics are not recorded'):
        for sev in ('INFO', 'WARN'):
            out = _screen([_rc(sev, node, msg)])
            assert len(out) == 1 and out[0].endswith(msg), (node, sev, msg)


def test_no_rule_touches_a_jugglebot_node_except_its_raw_library_lines():
    ours = {lc.short_name(n) for n in _OURS}
    for name, severities, _pattern, _action in lc.RULES:
        if name in ours:
            assert severities == (lc.RAW,), name


# ── third-party reductions ─────────────────────────────────────────────────

def test_rosbridge_is_one_line_per_gui_connect_and_disconnect_plus_errors():
    rb = 'rosbridge_websocket'
    proc = 'rosbridge_websocket_lean'
    out = _screen([
        _rc('INFO', rb, 'Rosbridge WebSocket server started on port 9090', proc),
        _rc('INFO', rb, 'Client connected. 1 clients total.', proc),
        _rc('INFO', rb, '[Client 98aa6e94-4293-4773-80e6-40a384f81744] '
                        'Subscribed to robot_state', proc),
        _rc('WARN', rb, 'WebSocketClosedError: Tried to write to a closed '
                        'websocket', proc),
        _rc('INFO', rb, 'rosbridge on_close: scheduled immediate teardown of '
                        '52 subscription(s) (client x)', proc),
        _rc('ERROR', rb, "[Client 10c21347-257e-4a9f-a0a9-259046c0776e] "
                         "[id: call_service:bb/aim:85] call_service "
                         "TimeoutError: service call to '/bb/aim' timed out",
            proc),
        _rc('INFO', rb, 'Client disconnected. 0 clients total.', proc),
        '[rosbridge_websocket_lean-1] Traceback (most recent call last):',
    ])
    texts = [line[len('00:00:00.000 ') + 13:] for line in out]
    assert texts == [
        'GUI websocket listening on :9090',
        'GUI connected (1 client now)',
        "ERROR GUI request failed: call_service TimeoutError: service call "
        "to '/bb/aim' timed out",
        'GUI disconnected (0 clients left)',
        'Traceback (most recent call last):',
    ]


def test_rosbag_topic_list_is_hidden_and_its_errors_are_not():
    rb = 'rosbag2_transport'
    out = _screen([_rc('INFO', rb, 'Listening for topics...', 'rosbag_record'),
                   _rc('INFO', rb, "Subscribed to topic '/rosout'",
                       'rosbag_record'),
                   _rc('WARN', rb, 'Hidden topics are not recorded. Enable '
                                   'them with --include-hidden-topics',
                       'rosbag_record'),
                   _rc('ERROR', rb, 'disk full', 'rosbag_record')])
    assert len(out) == 1 and out[0].endswith('rosbag       ERROR disk full')


def test_qtm_library_chatter_is_hidden_but_mocap_node_is_not():
    out = _screen([
        _rc('INFO', 'mocap_node', 'Connected to QTM.'),
        '[mocap_node-4] 2026-09-13 11:30:08,123 - qtm_rt - INFO - '
        'QRTEvent.EventCameraSettingsChanged',
        '[mocap_node-4] 2026-09-13 11:30:08,123 - qtm_rt - WARNING - x'])
    assert [o.split(None, 2)[2] for o in out] == [
        'Connected to QTM.', '2026-09-13 11:30:08,123 - qtm_rt - WARNING - x']


def test_launch_lifecycle_lines():
    out = _screen([
        '[INFO] [skill_node-10]: process started with pid [1826577]',
        '[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)',
        '[INFO] [skill_node-10]: process has finished cleanly [pid 1826577]',
        "[ERROR] [trajectory_node-9]: process has died [pid 2807828, exit "
        "code -2, cmd '/x/trajectory_node --ros-args']."])
    texts = [o[len('00:00:00.000 '):] for o in out]
    assert texts == [
        '%-12s WARN  Ctrl-C: shutting down' % 'launch',
        '%-12s exited cleanly' % 'skill',
        '%-12s ERROR PROCESS DIED (killed by SIGINT)' % 'trajectory']


# ── install ────────────────────────────────────────────────────────────────

def _handler():
    handler = logging.StreamHandler(io.StringIO())
    handler.setFormatter(logging.Formatter('{msg}', style='{'))
    return handler


def _emit(handler, name, msg, level=logging.INFO):
    handler.handle(logging.LogRecord(name, level, '', 0, msg, None, None))
    return handler.stream.getvalue()


def test_install_formats_and_filters_the_handler_it_is_given():
    handler = _handler()
    assert lc.install(handler, environ={'NO_COLOR': ''}) is True
    _emit(handler, 'skill_node-10-stderr', _rc('INFO', 'skill_node', 'hi'))
    _emit(handler, 'skill_node-10', 'process started with pid [7]')
    assert handler.stream.getvalue() == '%s %-12s hi\n' % (lc.clock(T),
                                                            'skill')


def test_install_is_idempotent():
    handler = _handler()
    lc.install(handler, environ={})
    console = handler._jugglebot_console
    lc.install(handler, environ={})
    assert handler._jugglebot_console is console
    assert len(handler.filters) == 1


def test_raw_env_leaves_the_handler_stock():
    handler = _handler()
    assert lc.install(handler, environ={'JUGGLEBOT_CONSOLE': 'raw'}) is False
    line = _rc('INFO', 'skill_node', 'hi')
    assert _emit(handler, 'skill_node-10-stderr', line) == line + '\n'


def test_colour_switches():
    assert lc.colour_wanted({}) is True
    assert lc.colour_wanted({'NO_COLOR': ''}) is False
    assert lc.colour_wanted({'JUGGLEBOT_CONSOLE_COLOR': '0'}) is False


def test_a_console_bug_prints_the_stock_line_instead_of_losing_it(monkeypatch):
    handler = _handler()
    lc.install(handler, environ={})

    def boom(record):
        raise RuntimeError('console bug')

    monkeypatch.setattr(lc, 'parse', boom)
    line = _rc('ERROR', 'teensy_bridge_node', 'GUARD LATCHED')
    assert _emit(handler, 'teensy_bridge_node-11-stderr', line) == line + '\n'


# ── wiring ─────────────────────────────────────────────────────────────────

_LAUNCH_DIR = os.path.join(os.path.dirname(os.path.dirname(lc.__file__)),
                           'launch')


@pytest.mark.parametrize('name', ['jugglebot_launch.py',
                                  'teensy_bridge_launch.py'])
def test_both_launch_files_install_the_console(name):
    with open(os.path.join(_LAUNCH_DIR, name)) as f:
        text = f.read()
    assert 'from jugglebot.launch_console import install' in text
    assert "output='screen'" not in text   # 'both': launch.log gets it too


_E2E = textwrap.dedent(r'''
    import sys
    import launch.logging
    from launch import LaunchDescription, LaunchService
    from launch.actions import ExecuteProcess
    from jugglebot.launch_console import install

    launch.logging.launch_config.log_dir = sys.argv[1]
    install(launch.logging.launch_config.get_screen_handler(),
            environ={'NO_COLOR': ''})
    emit = (
        "import sys\n"
        "for l in ['[INFO] [%(t)s] [skill_node]: skill_node ready',\n"
        "          '[DEBUG] [%(t)s] [skill_node]: fine detail',\n"
        "          '[WARN] [%(t)s] [skill_node]: careful']:\n"
        "    print(l, file=sys.stderr)\n") % {'t': sys.argv[2]}
    ls = LaunchService()
    ls.include_launch_description(LaunchDescription([
        ExecuteProcess(name='skill_node', cmd=[sys.executable, '-c', emit],
                       output='both')]))
    sys.exit(ls.run())
''')


def test_end_to_end_through_a_real_launch_service(tmp_path):
    """The screen changes and launch.log does not — through Foxy's own
    LaunchService, in a subprocess so launch's process-global logging
    setup (its logger class, the root level) never touches this worker."""
    pytest.importorskip('launch.logging')
    env = dict(os.environ)
    env['PYTHONPATH'] = os.pathsep.join(
        [os.path.dirname(os.path.dirname(lc.__file__))]
        + [p for p in env.get('PYTHONPATH', '').split(os.pathsep) if p])
    proc = subprocess.run([sys.executable, '-c', _E2E, str(tmp_path),
                           '%.9f' % T], capture_output=True, text=True,
                          env=env, timeout=60)
    assert proc.returncode == 0, proc.stderr
    screen = proc.stdout.splitlines()
    # Launch's own two opening lines (formatted here only because this
    # script installs before `run`; under `ros2 launch` they print stock).
    assert len(screen) == 5, screen
    assert re.match(CLOCK + r' launch +All log files can be found below ',
                    screen[0])
    assert re.match(CLOCK + r' launch +Default logging verbosity', screen[1])
    assert screen[2:4] == [
        '%s %-12s skill_node ready' % (lc.clock(T), 'skill'),
        '%s %-12s WARN  careful' % (lc.clock(T), 'skill'),
    ]
    assert re.match(CLOCK + ' skill +exited cleanly$', screen[4])
    with open(os.path.join(str(tmp_path), 'launch.log')) as f:
        record = f.read()
    for raw in ('[skill_node-1] [INFO] [%.9f] [skill_node]: skill_node '
                'ready' % T,
                '[skill_node-1] [DEBUG] [%.9f] [skill_node]: fine detail' % T,
                'process started with pid'):
        assert raw in record
