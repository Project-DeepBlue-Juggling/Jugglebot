"""CLAUDE.md glossary term block writer.

NO-OP until P1: returns ``[]`` and touches nothing when ``CLAUDE.md`` has no
``<!-- glossary-terms:start -->`` marker. P1 supplies the marker and the
CONTEXT.md term source; this entry point is wired into ``gen`` now.
"""
from __future__ import annotations

import os
from typing import List

START_MARKER = "<!-- glossary-terms:start -->"


def write_term_block(repo_root: str, check: bool = False) -> List[str]:
    """Paths changed by regenerating the term block (always ``[]`` before P1)."""
    path = os.path.join(repo_root, "CLAUDE.md")
    if not os.path.isfile(path):
        return []
    with open(path, encoding="utf-8") as fh:
        if START_MARKER not in fh.read():
            return []
    return []  # P1: render the block between the markers here.
