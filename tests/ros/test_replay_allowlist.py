"""The replay allowlist (ros_ws/gui/replay/schema.py) tracks the GUI's live
subscribe set: every ``subscribe('<topic>', ...)`` call with a literal topic in
ros_ws/gui/js/*.js must be in ``schema.SUBSCRIBED`` and vice versa, and the
allowlist is exactly SUBSCRIBED plus the plan's PLANNED extras. A topic the
GUI starts consuming live that the converter does not keep would replay blank.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
GUI = REPO / "ros_ws" / "gui"
if str(GUI) not in sys.path:
    sys.path.insert(0, str(GUI))

from replay import schema  # noqa: E402

# `ros.subscribe('robot_state', ...)` and bare `subscribe("x", ...)`; the
# definition `export function subscribe(topicName, ...)` has no quoted literal.
_SUBSCRIBE_RE = re.compile(r"""\bsubscribe\(\s*(['"])([^'"]+)\1""")


def _js_topics():
    topics = {}
    for js in sorted((GUI / "js").glob("*.js")):
        for lineno, line in enumerate(js.read_text().splitlines(), 1):
            if line.lstrip().startswith(("//", "*", "/*")):
                continue
            for m in _SUBSCRIBE_RE.finditer(line):
                t = m.group(2)
                t = t if t.startswith("/") else "/" + t
                topics.setdefault(t, "%s:%d" % (js.name, lineno))
    return topics


def test_js_subscribe_set_matches_schema_subscribed():
    js = _js_topics()
    assert js, "no subscribe('<topic>', ...) calls found under ros_ws/gui/js/ - regex rotted?"
    sub = set(schema.SUBSCRIBED)
    only_js = sorted(set(js) - sub)
    only_schema = sorted(sub - set(js))
    msg = []
    if only_js:
        msg.append("subscribed in the GUI JS but missing from schema.SUBSCRIBED: "
                   + ", ".join("%s (%s)" % (t, js[t]) for t in only_js))
    if only_schema:
        msg.append("in schema.SUBSCRIBED but no longer subscribed in ros_ws/gui/js/: "
                   + ", ".join(only_schema))
    assert not msg, ("; ".join(msg)
                     + " - update SUBSCRIBED in ros_ws/gui/replay/schema.py (the replay "
                       "cache contract) so replay converts exactly what the GUI shows live")


def test_allowlist_is_subscribed_plus_planned():
    allow = set(schema.ALLOWLIST)
    want = set(schema.SUBSCRIBED) | set(schema.PLANNED)
    assert allow == want, (
        "schema.ALLOWLIST has extra %s and is missing %s - fix ros_ws/gui/replay/schema.py"
        % (sorted(allow - want), sorted(want - allow)))
    assert len(schema.ALLOWLIST) == len(allow), "duplicate topic in schema.ALLOWLIST"
