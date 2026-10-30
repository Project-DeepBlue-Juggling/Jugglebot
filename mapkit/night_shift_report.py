"""Night Shift report layout: the eight sections and a checker (T-U7).

The report is ``docs/night-shift/<date>.md`` (layout in
``docs/night-shift/README.md``). The map app renders it and links each Item by
its bare ``JB-n``, so a row that names an Item any other way is an error here.
Pure functions, stdlib only.
"""
from __future__ import annotations

import re
from typing import Dict, List

SECTIONS = (
    "Nightly",
    "Merged to main",
    "Held",
    "Questions for the Operator",
    "Unfinished",
    "Skipped",
    "Unaddressed Sitting results",
    "Cost",
)
# sections whose every row begins with the Item it is about
ITEM_ROW_SECTIONS = ("Merged to main", "Held", "Questions for the Operator", "Skipped",
                     "Unaddressed Sitting results")
_TITLE = re.compile(r"^# Night Shift \d{4}-\d{2}-\d{2}$")
_BARE_ID = re.compile(r"^JB-\d+$")


def _headings(text: str) -> List[str]:
    return [l[3:].strip() for l in text.splitlines() if l.startswith("## ")]


def parse_report(text: str) -> Dict[str, List[str]]:
    """Heading -> its non-blank body lines (stripped), in file order."""
    out: Dict[str, List[str]] = {}
    cur = None
    for line in text.splitlines():
        if line.startswith("## "):
            cur = line[3:].strip()
            out[cur] = []
        elif cur is not None and line.strip():
            out[cur].append(line.strip())
    return out


def report_errors(text: str) -> List[str]:
    """Problems with a report: title, missing or misordered sections, malformed Item rows."""
    errs: List[str] = []
    first = next((l for l in text.splitlines() if l.strip()), "")
    if not _TITLE.match(first.rstrip()):
        errs.append("title must be '# Night Shift <YYYY-MM-DD>', got %r" % first)
    heads = _headings(text)
    for s in SECTIONS:
        if s not in heads:
            errs.append("missing section '## %s'" % s)
    present = [h for h in heads if h in SECTIONS]
    if not any(e.startswith("missing") for e in errs) and present != list(SECTIONS):
        errs.append("sections out of order: found %s, want %s" % (present, list(SECTIONS)))
    parsed = parse_report(text)
    for s in ITEM_ROW_SECTIONS:
        for row in parsed.get(s, []):
            if row == "none" or row.startswith("none "):
                continue
            first_tok = row.split("|")[0].split()[0] if row.split("|")[0].split() else ""
            if not _BARE_ID.match(first_tok):
                errs.append("'## %s' row must start with a bare JB-n: %r" % (s, row))
    return errs
