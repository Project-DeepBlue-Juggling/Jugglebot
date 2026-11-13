#!/usr/bin/env python3
"""tools/night_shift_bash_guard.py: the Night Shift's PreToolUse hook for the Bash tool.

The ONE enforcement point for probes, bag reads and the evidence directories at night
(closeout C7a, 2026-11-14). tools/night_shift_settings.json wires it for every Bash
call, the orchestrator's and every subagent's, and runs MAIN's copy, never the
checked-out branch's (the night switches branches in the clone, and a unit's edit to
this file must not change the guard before it passes the merge gate):

    g=$(timeout 20 git -C "$CLAUDE_PROJECT_DIR" show origin/main:tools/night_shift_bash_guard.py) \
      && /usr/bin/python3 -I -c "$g" || exit 2

THE INVARIANT. A command that mentions a guarded token -- ``probes`` (tools/probes,
tools.probes, temp/probes, a ``cd`` into the directory followed by ``probes/x.py``),
``ros2 bag``, ``rosbag`` (~/Desktop/rosbags) or ``.ros`` (~/.ros/log) -- runs only when
the WHOLE command is one invocation of the trusted no-network wrapper:

    tools/night_shift_nonet.sh COMMAND [ARGS...]

and ``ros2 bag play|record|reindex|convert`` never runs, wrapped or not.

  * "Whole command": after leading whitespace the first word is the wrapper, and the
    line holds none of ; & | < > ` $( or a newline, so nothing can start a second,
    unwrapped command or write outside the namespace by redirection (``2>&1`` and
    ``> file`` are refused too: give the probe its own --out flag, or read its stdout).
  * "Trusted wrapper": the file the first word names (relative to the hook's cwd) must
    be byte-identical to ``origin/main:tools/night_shift_nonet.sh`` in the project
    repo. The location does not matter -- the clone's tools/, ./tools/ or the runner's
    D/gate-base/tools/ copy all pass while they match main -- but a branch's edited or
    fake wrapper does not, so a unit that changes the wrapper cannot run probes
    through its own copy. Trust is in the content main has, not in a path.
  * Matching is on a normalised copy of the line: quotes and backslashes removed and
    whitespace collapsed, so ``ros2  "bag"`` and ``pro''bes`` are still seen.

Blocks by exit code 2 with the reason on stderr (the Claude Code hook contract: exit 2
on PreToolUse stops the call and the stderr goes to the model; any other non-zero exit
is a NON-blocking error, hence the ``|| exit 2`` in the settings). Fails closed: input
that does not parse is refused when its raw text mentions a guarded token; an internal
error does the same; a guard that cannot be read from main or cannot start refuses
every Bash call.

Stdlib only, Python 3.8 (the hook runs on the system interpreter).
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import sys

WRAPPER_REL = "tools/night_shift_nonet.sh"
TRUSTED_REF = "origin/main"
GUARDED = re.compile(r"probes|ros2 bag|rosbag|\.ros(?![A-Za-z0-9_])", re.IGNORECASE)
FORBIDDEN_BAG = re.compile(r"ros2 bag (?:play|record|reindex|convert)\b", re.IGNORECASE)
OPERATORS = re.compile(r"[;&|<>`\n\r]|\$\(")

SHAPE = (
    "Accepted shape: the wrapper FIRST and nothing else on the line -- "
    "`tools/night_shift_nonet.sh python tools/probes/<probe>.py --bag ... --out temp/probes/...` "
    "(timeout/env go after the wrapper; no ; && || | & > < $( or backticks, no newline, "
    "no `cd X &&` prefix: run from the clone root). The wrapper file must equal "
    "origin/main:tools/night_shift_nonet.sh (the clone's tools/ copy on a branch that has "
    "not changed it, or D/gate-base/tools/night_shift_nonet.sh). This applies to every "
    "command naming probes, ros2 bag, rosbags or ~/.ros, reads included. "
    "ros2 bag play/record/reindex/convert never run."
)


def normalise(text):
    return " ".join(re.sub(r"[\"'\\]", "", text).split())


def mentions_guarded(text):
    return bool(GUARDED.search(normalise(text)))


def _blob_sha(path):
    with open(path, "rb") as f:
        data = f.read()
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def trusted_wrapper(word, cwd, project):
    """True when ``word`` names a wrapper file byte-identical to main's."""
    if not (word == WRAPPER_REL or word.endswith("/" + WRAPPER_REL)):
        return False
    path = os.path.join(cwd, os.path.expanduser(word))
    if not os.path.isfile(path):
        return False
    r = subprocess.run(["git", "-C", project, "rev-parse", "--verify", "-q",
                        "%s:%s" % (TRUSTED_REF, WRAPPER_REL)],
                       stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                       universal_newlines=True, timeout=20)
    want = r.stdout.strip()
    return r.returncode == 0 and bool(want) and _blob_sha(path) == want


def verdict(command, cwd, project):
    """None to allow, else the reason to refuse."""
    norm = normalise(command)
    if FORBIDDEN_BAG.search(norm):
        return "ros2 bag play/record/reindex/convert never run at night."
    if not GUARDED.search(norm):
        return None
    if OPERATORS.search(command):
        return "a guarded command may not chain, pipe, background or redirect."
    words = command.split()
    if not words or not trusted_wrapper(words[0], cwd, project):
        return "a guarded command must start with the trusted no-network wrapper."
    return None


def main():
    raw = sys.stdin.buffer.read().decode("utf-8", "replace")
    try:
        event = json.loads(raw)
        command = event["tool_input"]["command"]
        if not isinstance(command, str):
            raise TypeError("command is not a string")
        cwd = event.get("cwd") or os.getcwd()
        project = os.environ.get("CLAUDE_PROJECT_DIR") or cwd
    except Exception as e:  # noqa: BLE001 -- any malformed input
        if mentions_guarded(raw):
            sys.stderr.write("Night Shift guard: cannot read the hook input (%s); refused "
                             "because it mentions a guarded path. %s\n" % (e, SHAPE))
            return 2
        return 0
    try:
        reason = verdict(command, cwd, project)
    except Exception as e:  # noqa: BLE001 -- fail closed on anything unexpected
        reason = "guard error (%s: %s)." % (type(e).__name__, e)
    if reason is None:
        return 0
    sys.stderr.write("Night Shift guard refused this command: %s %s\n" % (reason, SHAPE))
    return 2


if __name__ == "__main__":
    sys.exit(main())
