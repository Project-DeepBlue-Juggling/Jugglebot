"""No GLOSSARY.md _Avoid_ word in live plans, recent logbook entries or map/ items.

One looping test lists every offender as file:line: phrase. Exempt: text inside
backticks (inline or fenced), text inside double quotes on the same line, lines
ending ``<!-- avoid-ok -->``, and path-like tokens (anything containing ``/`` or
ending in a file extension) -- a filename such as
``2026-10-19-held-axis-reload-catch-refuses-tracker-reaim.md`` or a service name
such as ``bb/reload`` is an identifier even when it is not backticked, and a
filename never changes. Phrases live in ``avoid_words_checked.txt``.
"""
from __future__ import annotations

import glob
import os
import re
from typing import List

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CHECKED = os.path.join(os.path.dirname(os.path.abspath(__file__)), "avoid_words_checked.txt")
LOGBOOK_FROM = "2026-09-21"
OK_TAG = "<!-- avoid-ok -->"

_INLINE_CODE = re.compile(r"`[^`]*`")
_QUOTED = re.compile(r'"[^"]*"')
# A token with a slash, or ending in a file extension, is a path or identifier.
_PATHLIKE = re.compile(r"\S*/\S*|\S+\.(?:md|py|yaml|yml|json|action|msg|srv|ino|h|cpp|js|csv|log|txt|sh)\b")


def load_phrases(path: str = CHECKED) -> List[str]:
    out = []
    with open(path, encoding="utf-8") as fh:
        for raw in fh:
            s = raw.strip()
            if s and not s.startswith("#"):
                out.append(s)
    return out


def scoped_files(repo: str = REPO) -> List[str]:
    files = sorted(glob.glob(os.path.join(repo, "plans", "active", "*.md")))
    files += sorted(glob.glob(os.path.join(repo, "plans", "parked", "*.md")))
    for p in sorted(glob.glob(os.path.join(repo, "logbook", "*.md"))):
        m = re.match(r"(\d{4}-\d{2}-\d{2})-", os.path.basename(p))
        if m and m.group(1) >= LOGBOOK_FROM:
            files.append(p)
    files += sorted(glob.glob(os.path.join(repo, "map", "*.md")))
    return files


def find_offenders(paths: List[str], phrases: List[str], root: str = "") -> List[str]:
    pats = [(p, re.compile(r"\b" + re.escape(p) + r"\b", re.IGNORECASE)) for p in phrases]
    out: List[str] = []
    for path in paths:
        shown = os.path.relpath(path, root) if root else path
        fenced = False
        with open(path, encoding="utf-8") as fh:
            for n, raw in enumerate(fh, 1):
                line = raw.rstrip("\n")
                if line.lstrip().startswith("```"):
                    fenced = not fenced
                    continue
                if fenced or line.rstrip().endswith(OK_TAG):
                    continue
                bare = _PATHLIKE.sub(" ", _QUOTED.sub("", _INLINE_CODE.sub("", line)))
                for phrase, pat in pats:
                    if pat.search(bare):
                        out.append("%s:%d: %s" % (shown, n, phrase))
    return out


def test_no_avoid_words_in_live_docs():
    offenders = find_offenders(scoped_files(), load_phrases(), REPO)
    assert not offenders, "Avoid-words found (use the glossary term, or backtick/quote/avoid-ok):\n" + \
        "\n".join(offenders)


def test_find_offenders_negative_case(tmp_path):
    plan = tmp_path / "plan.md"
    plan.write_text(
        "clean line\n"
        "the leg 6 motor is hot\n"
        "backticked `leg 6` is fine\n"
        'quoted "leg 6" is fine\n'
        "tagged leg 6 line <!-- avoid-ok -->\n"
        "```\nleg 6 in a fence\n```\n"
        "tosses are not toss-the-word\n"
        "  - 2026-10-19-held-axis-reload-catch.md and bb/reload are paths\n"
        "but a bare reload here is not\n",
        encoding="utf-8")
    assert find_offenders([str(plan)], ["leg 6", "toss", "reload"], str(tmp_path)) == [
        "plan.md:2: leg 6", "plan.md:9: toss", "plan.md:11: reload"]


def test_checked_list_seed():
    assert load_phrases() == ["leg 6", "can-hub", "session limits", "guard E-STOP",
                              "runsheet", "toss", "reload", "ticket number"]
