"""The operator's launch shell: how each line is SHOWN, not what it says.

`ros2 launch` prints every process's output through ONE screen handler
(`launch.logging.launch_config.get_screen_handler()`). :func:`install` puts a
formatter and a filter on that handler, and on nothing else — so the screen
changes while every RECORD stays raw: `~/.ros/log/<run>/launch.log`, the
per-process rcutils files (`~/.ros/log/python3_<pid>_*.log`) and `/rosout` in
the bag keep the full `[INFO] [<epoch>] [<logger>]: ...` lines, and anything
that parses logs keeps working against those.

What the screen gets, per line::

    19:15:42.088 skill        self_toss ×5 started ...
    19:15:42.090 teensy       WARN  leg 4 heartbeat dropout ...

- local clock time (from the node's own rcutils stamp when the line has one,
  otherwise when launch received it) instead of epoch nanoseconds;
- a short node name instead of launch's ``[skill_node-10]`` process prefix
  and rcutils' ``[skill_node]`` logger field;
- no tag for INFO, a WARN / ERROR / FATAL tag otherwise, and colour
  (yellow warnings, red errors, one stable colour per node name);
- DEBUG never reaches the screen (it still reaches every record).

THE CONTRACT: this layer rewords or hides only THIRD-PARTY output — launch
itself, rosbridge, rosbag2, the qtm_rt library — whose source we do not
own. Jugglebot's own messages are never reworded or hidden here (DEBUG
aside); if one of those is noisy or opaque, fix it at the ``get_logger()``
call. A presentation layer that quietly rewrites our own messages is a
second place their meaning lives, and the two drift.

Escape hatches (read at :func:`install` time): ``JUGGLEBOT_CONSOLE=raw``
leaves the screen exactly as stock launch prints it; ``NO_COLOR`` (any value)
or ``JUGGLEBOT_CONSOLE_COLOR=0`` drops the colour but keeps the format.

Pure Python, stdlib only: importable in the launch process without ROS
(and by the tests without it).
"""

from __future__ import annotations

import datetime
import logging
import os
import re
import signal
from typing import Callable, List, Optional, Pattern, Tuple, Union

#: Width the short node name is padded to, so messages start in one column.
NAME_WIDTH = 12

#: Display names where stripping a trailing ``_node`` is not enough.
_ALIASES = {
    'teensy_bridge_node': 'teensy',
    'catch_correlation_node': 'cone',
    'spacemouse_handler': 'spacemouse',
    'rosbridge_websocket': 'rosbridge',
    'rosbridge_websocket_lean': 'rosbridge',
    'rosapi_node': 'rosapi',
    'rosbag2_transport': 'rosbag',
    'rosbag2_cpp': 'rosbag',
    'rosbag_record': 'rosbag',
    'ros2': 'rosbag',            # the bare `ros2 bag record` process name
    'launch.user': 'launch',
}

_SEVERITIES = ('DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL')
#: The severity of a line with none of its own (a traceback, a library's own
#: print or logging): shown untagged like INFO, but never matched by an
#: INFO rule — so hiding a source's INFO chatter cannot hide its tracebacks.
RAW = 'RAW'

#: ``[<process>-<n>] <line>`` — launch's default per-process ``output_format``.
_PROC_PREFIX = re.compile(r'^\[([^\]\s]+)-(\d+)\] ?(.*)$', re.S)
#: rcutils' default console format ``[SEV] [<epoch>] [<logger>]: <message>``.
_RCUTILS = re.compile(
    r'^\[(DEBUG|INFO|WARN|ERROR|FATAL)\] \[(\d+(?:\.\d+)?)\] '
    r'\[([^\]]+)\]: ?(.*)$', re.S)
#: A launch output logger's name: ``<process>-<n>-stdout`` / ``-stderr``.
_OUTPUT_LOGGER = re.compile(r'^(.+)-(\d+)-(stdout|stderr)$')
#: A launch per-process lifecycle logger's name: ``<process>-<n>``.
_PROCESS_LOGGER = re.compile(r'^(.+)-(\d+)$')

_ANSI = {
    'dim': '\x1b[2m', 'bold': '\x1b[1m', 'reset': '\x1b[0m',
    'yellow': '\x1b[33m', 'red': '\x1b[31m', 'bold_red': '\x1b[1;31m',
    'bold_yellow': '\x1b[1;33m',
}
#: Node-name colours (yellow and red are kept for severity).
_NAME_COLOURS = ('\x1b[36m', '\x1b[32m', '\x1b[35m', '\x1b[34m',
                 '\x1b[96m', '\x1b[92m', '\x1b[95m', '\x1b[94m')

_HIDE = None
#: A rule's action: hide the line (``_HIDE``), or a replacement template
#: (``re.sub`` syntax, applied to the message) / callable(match) -> str.
_Action = Union[None, str, Callable[['re.Match'], str]]


class Line:
    """One screen line, parsed. ``source`` is the display name's key (a
    logger or process name before aliasing), ``severity`` one of
    :data:`_SEVERITIES`, or :data:`RAW`."""

    __slots__ = ('t', 'source', 'severity', 'text', 'lifecycle')

    def __init__(self, t: float, source: str, severity: str, text: str,
                 lifecycle: bool = False) -> None:
        self.t = t
        self.source = source
        self.severity = severity
        self.text = text
        #: launch's own per-process line ("process started with pid ...").
        self.lifecycle = lifecycle


def short_name(source: str) -> str:
    """``skill_node`` -> ``skill``; the :data:`_ALIASES` for the rest."""
    if source in _ALIASES:
        return _ALIASES[source]
    if source.endswith('_node'):
        return source[:-len('_node')]
    return source


def parse(record: logging.LogRecord) -> Line:
    """What ``record`` says, whichever of launch's three shapes it has:

    - a process's output line (logger ``<proc>-<n>-stdout|stderr``, message
      ``[<proc>-<n>] <line>``), where ``<line>`` is either an rcutils line
      or a raw one (a traceback, a library's own logging);
    - launch's per-process lifecycle line (logger ``<proc>-<n>``: "process
      started with pid ...");
    - launch's own line (logger ``launch`` / ``launch.user``).
    """
    msg = record.getMessage()
    severity = _launch_severity(record.levelname)
    out = _OUTPUT_LOGGER.match(record.name)
    if out is not None:
        proc = out.group(1)
        pre = _PROC_PREFIX.match(msg)
        body = pre.group(3) if pre is not None else msg
        rc = _RCUTILS.match(body)
        if rc is not None:
            return Line(float(rc.group(2)), rc.group(3), rc.group(1),
                        rc.group(4))
        return Line(record.created, proc, RAW, body)
    lifecycle = _PROCESS_LOGGER.match(record.name)
    if lifecycle is not None:
        return Line(record.created, lifecycle.group(1), severity, msg,
                    lifecycle=True)
    return Line(record.created, record.name, severity, msg)


def _launch_severity(levelname: str) -> str:
    if levelname == 'WARNING':
        return 'WARN'
    if levelname == 'CRITICAL':
        return 'FATAL'
    return levelname if levelname in _SEVERITIES else 'INFO'


# ── third-party rules ─────────────────────────────────────────────────────
#
# (display name, severities or None for any, message pattern, action).
# First match wins. ONLY third-party sources belong here (see the module
# docstring's contract); `test_launch_console` pins that no rule names a
# jugglebot node.

def _rule(name: str, severities: Optional[Tuple[str, ...]], pattern: str,
          action: _Action) -> Tuple[str, Optional[Tuple[str, ...]],
                                    Pattern[str], _Action]:
    return (name, severities, re.compile(pattern, re.S), action)


_CLIENT_TAG = re.compile(r'\[Client [0-9a-fA-F-]+\] ')
_CALL_ID_TAG = re.compile(r'\[id: [^\]]*\] ')


def _gui_error(match: 're.Match') -> str:
    text = _CALL_ID_TAG.sub('', _CLIENT_TAG.sub('', match.group(0)))
    return 'GUI request failed: %s' % (text,)


def _clients(n: str) -> str:
    return '%s client%s' % (n, '' if n == '1' else 's')


def _died(match: 're.Match') -> str:
    code = int(match.group(1))
    if code < 0:
        try:
            return 'PROCESS DIED (killed by %s)' % (signal.Signals(-code).name,)
        except ValueError:
            pass
    return 'PROCESS DIED (exit code %d)' % (code,)


RULES = (
    # launch. (Its first two lines, "All log files can be found below ..."
    # and "Default logging verbosity ...", print before any launch file
    # loads — i.e. before `install` can run — so they stay stock.)
    _rule('launch', ('WARN',), r'^user interrupted with ctrl-c \(SIGINT\)$',
          'Ctrl-C: shutting down'),
    _rule('*process*', ('INFO',), r'^process started with pid \[\d+\]$', _HIDE),
    _rule('*process*', ('INFO',), r'^process has finished cleanly \[pid \d+\]$',
          'exited cleanly'),
    _rule('*process*', None,
          r'^process has died \[pid \d+, exit code (-?\d+), cmd .*\]\.?$',
          _died),
    # rosbridge: one line per GUI (dis)connect, errors, nothing else
    _rule('rosbridge', ('INFO',),
          r'^Rosbridge WebSocket server started on port (\d+)$',
          r'GUI websocket listening on :\1'),
    _rule('rosbridge', ('INFO',), r'^Client connected\. (\d+) clients? total\.$',
          lambda m: 'GUI connected (%s now)' % (_clients(m.group(1)),)),
    _rule('rosbridge', ('INFO',),
          r'^Client disconnected\. (\d+) clients? total\.$',
          lambda m: 'GUI disconnected (%s left)' % (_clients(m.group(1)),)),
    _rule('rosbridge', ('INFO', 'WARN'), r'^WebSocketClosedError', _HIDE),
    _rule('rosbridge', ('ERROR', 'FATAL'), r'^\[Client [0-9a-fA-F-]+\] .*$',
          _gui_error),
    _rule('rosbridge', ('INFO',), r'', _HIDE),
    # rosbag2: the recording line comes from the launch file instead
    _rule('rosbag', ('INFO',), r'^(Subscribed to topic|Listening for topics)',
          _HIDE),
    _rule('rosbag', ('WARN',), r'^Hidden topics are not recorded', _HIDE),
    # the qtm_rt library's own Python logging, from inside mocap_node
    _rule('mocap', (RAW,), r'^\S+ \S+ - qtm_rt - INFO - ', _HIDE),
)

#: The display names a ``'*process*'`` rule applies to: any launch
#: per-process lifecycle line, whatever the process.
_PROCESS_RULE = '*process*'


def apply_rules(line: Line) -> Optional[str]:
    """``line``'s text after the third-party rules, or ``None`` to hide it."""
    name = short_name(line.source)
    for rule_name, severities, pattern, action in RULES:
        if rule_name == _PROCESS_RULE:
            if not line.lifecycle:
                continue
        elif line.lifecycle or rule_name != name:
            continue
        if severities is not None and line.severity not in severities:
            continue
        match = pattern.search(line.text)
        if match is None:
            continue
        if action is _HIDE:
            return None
        if callable(action):
            return action(match)
        return pattern.sub(action, line.text, count=1)
    return line.text


#: :meth:`Console.format`'s answer when rendering raised: print the line the
#: way stock launch would. The console must never be the reason a line is
#: lost — a logging handler that raises drops the record.
FALLBACK = object()


class Console:
    """Parses, filters and renders records for the screen handler.

    It is both halves of :func:`install`: the handler's filter (``filter``
    decides, and caches the rendered line on the record) and the source of
    its formatted text (``format`` returns that cached line)."""

    _CACHE_ATTR = '_jugglebot_console_line'

    def __init__(self, colour: bool = True) -> None:
        self.colour = colour
        self._name_colours = {}

    def filter(self, record: logging.LogRecord) -> bool:
        """``logging.Filterer`` protocol: False hides ``record``."""
        try:
            rendered = self._render(record)
        except Exception:  # noqa: BLE001 — see FALLBACK
            rendered = FALLBACK
        setattr(record, self._CACHE_ATTR, rendered)
        return rendered is not None

    def format(self, record: logging.LogRecord):
        """The screen text for ``record``: a string, ``None`` (hidden), or
        :data:`FALLBACK`."""
        cached = getattr(record, self._CACHE_ATTR, False)
        if cached is not False:
            return cached
        try:
            return self._render(record)
        except Exception:  # noqa: BLE001 — see FALLBACK
            return FALLBACK

    def _render(self, record: logging.LogRecord) -> Optional[str]:
        line = parse(record)
        if line.severity == 'DEBUG':
            return None
        text = apply_rules(line)
        if text is None:
            return None
        return self.render_line(line.t, short_name(line.source),
                                line.severity, text)

    def render_line(self, t: float, name: str, severity: str,
                    text: str) -> str:
        stamp = clock(t)
        tag = '' if severity in ('INFO', RAW) else '%-5s ' % (severity,)
        if not self.colour:
            return '%s %-*s %s%s' % (stamp, NAME_WIDTH, name, tag, text)
        a = _ANSI
        head = '%s%s%s %s%-*s%s ' % (a['dim'], stamp, a['reset'],
                                     self._colour_for(name), NAME_WIDTH, name,
                                     a['reset'])
        if severity in ('ERROR', 'FATAL'):
            return '%s%s%s%s%s%s' % (head, a['bold_red'], tag, a['reset'] +
                                     a['red'], text, a['reset'])
        if severity == 'WARN':
            return '%s%s%s%s%s%s' % (head, a['bold_yellow'], tag,
                                     a['reset'] + a['yellow'], text,
                                     a['reset'])
        return head + text

    def _colour_for(self, name: str) -> str:
        if name not in self._name_colours:
            index = sum(name.encode('utf-8')) % len(_NAME_COLOURS)
            self._name_colours[name] = _NAME_COLOURS[index]
        return self._name_colours[name]


def clock(t: float) -> str:
    """Local wall-clock ``HH:MM:SS.mmm`` for an epoch-seconds instant."""
    dt = datetime.datetime.fromtimestamp(t)
    return '%s.%03d' % (dt.strftime('%H:%M:%S'), dt.microsecond // 1000)


def colour_wanted(environ=None) -> bool:
    env = os.environ if environ is None else environ
    if 'NO_COLOR' in env:
        return False
    return env.get('JUGGLEBOT_CONSOLE_COLOR', '1') not in ('0', 'false', 'no')


def install(handler=None, environ=None) -> bool:
    """Put the console on ``handler`` (default: launch's one screen handler).

    Returns whether it is installed. Idempotent — both launch files call it,
    and one may include the other. ``JUGGLEBOT_CONSOLE=raw`` leaves the
    handler untouched."""
    env = os.environ if environ is None else environ
    if env.get('JUGGLEBOT_CONSOLE', '').lower() == 'raw':
        return False
    if handler is None:
        import launch.logging  # the launch process only; tests pass a handler
        handler = launch.logging.launch_config.get_screen_handler()
    if getattr(handler, '_jugglebot_console', None) is not None:
        return True
    console = Console(colour=colour_wanted(env))
    base_format = handler.format

    def _format(record: logging.LogRecord) -> str:
        rendered = console.format(record)
        if rendered is None or rendered is FALLBACK:
            return base_format(record)
        return rendered

    handler.format = _format
    handler.addFilter(console)
    handler._jugglebot_console = console
    return True


def render_lines(lines: List[str], colour: bool = False) -> List[str]:
    """Replay a tee'd launch-shell capture (what stock launch printed)
    through the console -- for previews and tests. A line with no stamp of
    its own takes the last stamp seen, as it would have live."""
    console = Console(colour=colour)
    launch_line = re.compile(
        r'^\[(INFO|WARNING|ERROR|DEBUG)\] \[([^\]]+)\]: (.*)$')
    out = []
    last_t = 0.0
    for text in lines:
        text = text.rstrip('\n')
        m = launch_line.match(text)
        if m is not None:
            record = logging.LogRecord(m.group(2), getattr(logging, m.group(1)),
                                       '', 0, m.group(3), None, None)
        else:
            pre = _PROC_PREFIX.match(text)
            name = ('%s-%s-stderr' % (pre.group(1), pre.group(2))
                    if pre is not None else 'unknown-0-stderr')
            record = logging.LogRecord(name, logging.INFO, '', 0, text, None,
                                       None)
            rc = _RCUTILS.match(pre.group(3)) if pre is not None else None
            if rc is not None:
                last_t = float(rc.group(2))
        record.created = last_t
        if console.filter(record):
            rendered = console.format(record)
            out.append(record.getMessage() if rendered is FALLBACK
                       else rendered)
    return out
