"""YAML frontmatter splitting for plan and map-item markdown files.

A file whose first line is exactly ``---`` carries a frontmatter block that
ends at the next line that is exactly ``---``. The block is parsed with
``yaml.safe_load`` semantics, through libyaml's ``CSafeLoader`` when PyYAML
was built with it (about ten times faster: the map app rebuilds its whole index
on every file change, and the pure-Python scanner was ~0.5 s of a 0.67 s
rebuild on the Jetson). Files are only ever read here, never rewritten.
"""
from __future__ import annotations

from typing import Any, Dict, Tuple

import yaml

_Loader = getattr(yaml, "CSafeLoader", yaml.SafeLoader)


class FrontmatterError(ValueError):
    """The frontmatter block is not valid YAML, or is not a mapping."""


def split_frontmatter(text: str) -> Tuple[Dict[str, Any], str]:
    """Return ``(meta, body)``.

    A text without a leading ``---`` line returns ``({}, text)``. An opening
    ``---`` with no closing line, invalid YAML, or a non-mapping block raises
    :class:`FrontmatterError` (the message carries no path; callers add it).
    An empty block yields ``{}``.
    """
    if text.startswith("﻿"):
        text = text[1:]
    lines = text.split("\n")
    if not lines or lines[0].rstrip("\r") != "---":
        return {}, text
    end = None
    for i in range(1, len(lines)):
        if lines[i].rstrip("\r") == "---":
            end = i
            break
    if end is None:
        raise FrontmatterError("frontmatter opened with '---' but never closed")
    block = "\n".join(lines[1:end])
    body = "\n".join(lines[end + 1:])
    try:
        meta = yaml.load(block, Loader=_Loader)  # noqa: S506 - a safe loader
    except yaml.YAMLError as exc:
        raise FrontmatterError("invalid YAML in frontmatter: %s" % exc) from exc
    if meta is None:
        meta = {}
    if not isinstance(meta, dict):
        raise FrontmatterError(
            "frontmatter is a %s, not a mapping" % type(meta).__name__)
    return meta, body
