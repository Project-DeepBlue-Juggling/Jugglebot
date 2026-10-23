"""GLOSSARY.md parser and the CLAUDE.md glossary term-block writer.

``GLOSSARY.md`` entries are ``**Term**:`` lines, a definition on the following
line(s), an optional ``_Avoid_: a, b (note)`` line, grouped under ``### Cluster``
headings. The term-name list in ``CLAUDE.md`` between the
``<!-- glossary-terms:start -->`` / ``<!-- glossary-terms:end -->`` markers is
generated here; ``tests/sim/test_mapkit_glossary.py`` fails when it is stale.
No-op (returns ``[]``) when ``CLAUDE.md`` has no markers.
"""
from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from typing import List

START_MARKER = "<!-- glossary-terms:start -->"
END_MARKER = "<!-- glossary-terms:end -->"
LIMIT = 1200

_TERM_RE = re.compile(r"^\*\*(.+?)\*\*:\s*(.*)$")
_AVOID_RE = re.compile(r"^_Avoid_:\s*(.*)$")
_CLUSTER_RE = re.compile(r"^###\s+(.+?)\s*$")


@dataclass
class Entry:
    term: str
    definition: str
    avoid: List[str] = field(default_factory=list)
    cluster: str = ""


def split_top_level(text: str) -> List[str]:
    """Split on commas outside parentheses and double quotes."""
    items: List[str] = []
    depth = 0
    quoted = False
    cur = []
    for ch in text:
        if ch == '"':
            quoted = not quoted
        elif not quoted and ch == "(":
            depth += 1
        elif not quoted and ch == ")":
            depth = max(0, depth - 1)
        if ch == "," and depth == 0 and not quoted:
            items.append("".join(cur).strip())
            cur = []
        else:
            cur.append(ch)
    items.append("".join(cur).strip())
    return [i for i in items if i]


def parse_glossary(text: str) -> List[Entry]:
    entries: List[Entry] = []
    cluster = ""
    cur = None  # type: Entry
    in_language = False
    for raw in text.splitlines():
        line = raw.rstrip()
        if line.startswith("## "):
            in_language = line[3:].strip() == "Language"
            cur = None
            continue
        if not in_language:
            continue
        m = _CLUSTER_RE.match(line)
        if m:
            cluster = m.group(1)
            cur = None
            continue
        m = _TERM_RE.match(line)
        if m:
            cur = Entry(term=m.group(1).strip(), definition=m.group(2).strip(), cluster=cluster)
            entries.append(cur)
            continue
        if cur is None or not line.strip():
            continue
        m = _AVOID_RE.match(line)
        if m:
            cur.avoid = split_top_level(m.group(1))
        else:
            cur.definition = (cur.definition + " " + line.strip()).strip()
    return entries


def load_glossary(repo_root: str) -> List[Entry]:
    with open(os.path.join(repo_root, "GLOSSARY.md"), encoding="utf-8") as fh:
        return parse_glossary(fh.read())


def render_term_block(entries: List[Entry], limit: int = LIMIT) -> str:
    """Term names grouped by cluster, one line per cluster; drops cluster names if over ``limit``."""
    clusters: List[str] = []
    by_cluster = {}  # type: dict
    for e in entries:
        if e.cluster not in by_cluster:
            by_cluster[e.cluster] = []
            clusters.append(e.cluster)
        by_cluster[e.cluster].append(e.term)
    with_names = "\n".join("**%s**: %s" % (c, ", ".join(by_cluster[c])) for c in clusters)
    if len(with_names) <= limit:
        return with_names
    bare = "\n".join(", ".join(by_cluster[c]) for c in clusters)
    if len(bare) <= limit:
        return bare
    raise ValueError("glossary term block is %d chars even without cluster names (limit %d); "
                     "shorten the glossary or raise the ceiling deliberately" % (len(bare), limit))


def extract_block(claude_text: str) -> str:
    """Text strictly between the markers (newline-trimmed); raises if markers are absent."""
    s = claude_text.index(START_MARKER) + len(START_MARKER)
    e = claude_text.index(END_MARKER, s)
    return claude_text[s:e].strip("\n")


def write_term_block(repo_root: str, check: bool = False) -> List[str]:
    """Rewrite the block between the markers; return ``[path]`` if it changed, else ``[]``."""
    path = os.path.join(repo_root, "CLAUDE.md")
    if not os.path.isfile(path):
        return []
    with open(path, encoding="utf-8") as fh:
        old = fh.read()
    if START_MARKER not in old or END_MARKER not in old:
        return []
    s = old.index(START_MARKER) + len(START_MARKER)
    e = old.index(END_MARKER, s)
    new = old[:s] + "\n" + render_term_block(load_glossary(repo_root)) + "\n" + old[e:]
    if new == old:
        return []
    if not check:
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(new)
    return [path]
