"""``docs/project-map.md`` and ``docs/glossary.md`` writer.

NO-OP until P2: returns ``[]`` and touches nothing when ``<repo>/map/`` does
not exist. P2 supplies the map items these pages are rendered from; the entry
point is wired into ``gen`` now.
"""
from __future__ import annotations

import os
from typing import List


def write_docs(repo_root: str, check: bool = False) -> List[str]:
    """Paths changed by regenerating the docs pages (always ``[]`` before P2)."""
    if not os.path.isdir(os.path.join(repo_root, "map")):
        return []
    return []  # P2: render docs/project-map.md and docs/glossary.md here.
