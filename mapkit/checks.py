"""The map-write check: four checks over ``map/``, one enforcement point.

Each function returns a list of problem strings (empty = clean). The four map
tests call these same functions, and every ``mapkit item ... --commit`` calls
:func:`run_all` on the working tree BEFORE it commits, so the test and the
commit path cannot drift (plan ``night-shift-closeout.md``, C13):

1. :func:`check_map_loads`  - every ``map/*.md`` loads (``test_every_item_loads``).
2. :func:`check_rated`      - an Item past ``inbox`` is fully rated
   (``test_map_items_rated.py``).
3. :func:`check_readability` - the per-block readability cap
   (``test_item_bodies_are_readable``).
4. :func:`check_avoid_words` - no GLOSSARY ``_Avoid_`` word
   (``test_glossary_avoid_words.py``).

Pure Python, no ROS2. ``python -m mapkit check`` runs all four by hand.
"""
from __future__ import annotations

import glob
import os
import re
from typing import Dict, List, Optional

from mapkit.items import METRIC_FIELDS, ItemError, load_item, readability_violations

LOGBOOK_FROM = "2026-09-21"
OK_TAG = "<!-- avoid-ok -->"
CHECKED_REL = os.path.join("tests", "sim", "avoid_words_checked.txt")

_INLINE_CODE = re.compile(r"`[^`]*`")
_QUOTED = re.compile(r'"[^"]*"')
# A token with a slash, or ending in a file extension, is a path or identifier.
_PATHLIKE = re.compile(r"\S*/\S*|\S+\.(?:md|py|yaml|yml|json|action|msg|srv|ino|h|cpp|js|csv|log|txt|sh)\b")
# A frontmatter REFERENCE is an identifier like a filename: `parent:` /
# `promoted_to:` hold an Item stem or pathway slug, and a bare `- <stem>` list
# item under `blocked_by:` / `links:` holds a stem (`mapkit` stores stems, never
# titles). A question ticket whose parent is `bug-sim-reload-...` must not fail
# for the bug's own stem (BUG-45, 2026-11-01). Body text is never exempted.
# A stem or slug always carries a hyphen or a digit; a single bare word
# (`- reload` as a tag, say) is still scanned.
_FRONT_REF = re.compile(r"^(?:(?:parent|promoted_to):\s*\S+|- (?=[A-Za-z0-9._-]*[-0-9])[A-Za-z0-9._-]+)\s*$")


def map_paths(repo: str) -> List[str]:
    return sorted(glob.glob(os.path.join(repo, "map", "*.md")))


# 1. whole-map load -----------------------------------------------------------

def check_map_loads(repo: str) -> List[str]:
    bad = []
    paths = map_paths(repo)
    for p in paths:
        try:
            load_item(p)
        except ItemError as exc:
            bad.append(str(exc))
    if not paths:
        bad.append("map/ holds no items")
    return bad


# 2. rated on leaving the inbox -----------------------------------------------

def frontmatter(path: str) -> Dict[str, str]:
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    assert text.startswith("---\n"), path
    body = text.split("\n---", 1)[0][4:]
    out = {}
    for line in body.splitlines():
        if line and not line.startswith(" ") and ":" in line:
            k, v = line.split(":", 1)
            out[k.strip()] = v.strip()
    return out


def is_inbox(path: str) -> bool:
    return frontmatter(path).get("status") == "inbox"


def unrated_problems(path: str) -> List[str]:
    """One Item: the metric fields it is missing, or [] (inbox Items are exempt)."""
    fm = frontmatter(path)
    if fm.get("status") == "inbox":
        return []
    required = [f for f in METRIC_FIELDS if f != "diagnosis" or fm.get("kind") == "bug"]
    missing = [f for f in required if not fm.get(f)]
    return ["%s is unrated: %s" % (os.path.basename(path), missing)] if missing else []


def check_rated(repo: str) -> List[str]:
    out: List[str] = []
    for p in map_paths(repo):
        try:
            out += unrated_problems(p)
        except AssertionError:
            out.append("%s: no frontmatter" % os.path.basename(p))
    return out


# 3. readability cap ----------------------------------------------------------

def check_readability(repo: str) -> List[str]:
    out: List[str] = []
    for p in map_paths(repo):
        try:
            item = load_item(p)
        except ItemError:
            continue  # reported by check_map_loads
        out += ["%s: %s" % (item.filename, v) for v in readability_violations(item.body)]
    return out


# 4. glossary avoid-words -----------------------------------------------------

def load_phrases(path: str) -> List[str]:
    out = []
    with open(path, encoding="utf-8") as fh:
        for raw in fh:
            s = raw.strip()
            if s and not s.startswith("#"):
                out.append(s)
    return out


def scoped_files(repo: str, map_only: bool = False) -> List[str]:
    """Plans, recent logbook entries and map Items (``map_only``: Items alone)."""
    files: List[str] = []
    if not map_only:
        files = sorted(glob.glob(os.path.join(repo, "plans", "active", "*.md")))
        files += sorted(glob.glob(os.path.join(repo, "plans", "parked", "*.md")))
        for p in sorted(glob.glob(os.path.join(repo, "logbook", "*.md"))):
            m = re.match(r"(\d{4}-\d{2}-\d{2})-", os.path.basename(p))
            if m and m.group(1) >= LOGBOOK_FROM:
                files.append(p)
    files += map_paths(repo)
    return files


def find_offenders(paths: List[str], phrases: List[str], root: str = "") -> List[str]:
    pats = [(p, re.compile(r"\b" + re.escape(p) + r"\b", re.IGNORECASE)) for p in phrases]
    out: List[str] = []
    for path in paths:
        shown = os.path.relpath(path, root) if root else path
        fenced = False
        front = None  # None before the opening ---, True inside, False after
        with open(path, encoding="utf-8") as fh:
            for n, raw in enumerate(fh, 1):
                line = raw.rstrip("\n")
                if line.strip() == "---" and front is not False:
                    front = True if front is None and n == 1 else False
                    continue
                if front is None:
                    front = False
                if line.lstrip().startswith("```"):
                    fenced = not fenced
                    continue
                if fenced or line.rstrip().endswith(OK_TAG):
                    continue
                if front and _FRONT_REF.match(line):
                    continue
                bare = _PATHLIKE.sub(" ", _QUOTED.sub("", _INLINE_CODE.sub("", line)))
                for phrase, pat in pats:
                    if pat.search(bare):
                        out.append("%s:%d: %s" % (shown, n, phrase))
    return out


def check_avoid_words(repo: str, map_only: bool = False, phrases_file: Optional[str] = None) -> List[str]:
    """Avoid-words in plans, logbook and map. ``map_only`` (the commit path) scans
    the Items alone: a map write changes nothing else, and a logbook offender
    must not block it. A repo without the phrase list has nothing to check."""
    phrases_file = phrases_file or os.path.join(repo, CHECKED_REL)
    if not os.path.isfile(phrases_file):
        return []
    return find_offenders(scoped_files(repo, map_only), load_phrases(phrases_file), repo)


# all four --------------------------------------------------------------------

def run_all(repo: str, map_only_avoid: bool = True) -> List[str]:
    """Every problem across the four checks, each prefixed with its check name."""
    out: List[str] = []
    for name, fn in (("map-loads", check_map_loads), ("rated", check_rated),
                     ("readability", check_readability)):
        out += ["[%s] %s" % (name, p) for p in fn(repo)]
    out += ["[avoid-words] %s" % p for p in check_avoid_words(repo, map_only=map_only_avoid)]
    return out
