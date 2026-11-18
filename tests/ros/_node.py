"""The ONE Node.js lookup for the GUI harness tests (BUG-103).

Node on the Jetson lives only under ``~/.nvm``, which is on an interactive
shell's PATH and NOT on the 02:00 systemd run's. Thirty test files carried a
copy of this lookup and eight used a bare ``shutil.which``; those eight (59
tests) skipped on every nightly run from 2026-10-17 to 2026-11-16. Import
``find_node`` here instead of writing another copy.
"""
from __future__ import annotations

import glob
import os
import re
import shutil
from typing import Optional, Tuple

_PATTERNS = (
    "~/.nvm/versions/node/*/bin/node",
    "/usr/local/bin/node",
    "/usr/bin/node",
    "/opt/node*/bin/node",
)


def _path_version(path: str) -> Tuple[int, int, int]:
    """Version tuple parsed out of an nvm-style path; zeros when absent."""
    match = re.search(r"/v(\d+)\.(\d+)\.(\d+)/", path)
    return tuple(int(g) for g in match.groups()) if match else (0, 0, 0)


def find_node() -> Optional[str]:
    """Absolute path to a usable node binary, or ``None``.

    PATH first, then the nvm tree and the usual prefixes. Candidates are
    sorted NUMERICALLY by version: a lexicographic sort puts nvm's "v9.x"
    above "v22.x" and would pick an ancient interpreter.
    """
    found = shutil.which("node") or shutil.which("nodejs")
    if found:
        return found
    candidates = []
    for pattern in _PATTERNS:
        candidates.extend(glob.glob(os.path.expanduser(pattern)))
    for candidate in sorted(candidates, key=_path_version, reverse=True):
        if os.access(candidate, os.X_OK):
            return candidate
    return None
