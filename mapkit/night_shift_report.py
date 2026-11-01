"""Night Shift report layout: the eight sections and a checker (T-U7).

The report is ``docs/night-shift/<date>.md`` (layout in
``docs/night-shift/README.md``). The map app renders it (GFM, tables included)
and links each Item by its bare ``JB-n``, so a row that names an Item any other
way is an error here. Item sections are GFM tables since 2026-11-01 (owner:
readability); a table's header row and its ``|---|`` rule are layout, not rows,
and the older bare ``JB-n | ...`` lines still pass. Pure functions, stdlib only.
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
RESPOND_IN = "Respond in"
_RULE_CELL = re.compile(r"^:?-{3,}:?$")


def cells(row: str) -> List[str]:
    """A row's cells: a GFM table row loses its outer pipes; a bare pipe line splits the same way."""
    s = row.strip()
    if s.startswith("|"):
        s = s[1:]
    if s.endswith("|"):
        s = s[:-1]
    return [c.strip() for c in s.split("|")]


def is_table_rule(row: str) -> bool:
    cs = cells(row)
    return bool(cs) and all(_RULE_CELL.match(c) for c in cs)


def item_rows(rows: List[str]) -> List[str]:
    """The rows of an Item section that must name an Item: drops a table's rule row and the
    header row immediately before it. Everything else is a row to check."""
    out: List[str] = []
    for i, row in enumerate(rows):
        if is_table_rule(row):
            continue
        if i + 1 < len(rows) and is_table_rule(rows[i + 1]):
            continue  # the header row
        out.append(row)
    return out


def _respond_in_errors(rows: List[str]) -> List[str]:
    """A Held table with a ``Respond in`` header (reports from 2026-11-02) must fill that cell on
    every row with a bare ``JB-m`` (the question ticket) or ``none`` (a hold only a Sitting
    unblocks). A Held table without the column is older history and passes."""
    for i, row in enumerate(rows):
        if i + 1 < len(rows) and is_table_rule(rows[i + 1]) and RESPOND_IN in cells(row):
            col = cells(row).index(RESPOND_IN)
            break
    else:
        return []
    errs: List[str] = []
    for r in item_rows(rows):
        cs = cells(r)
        cell = cs[col] if col < len(cs) else ""
        if cell != "none" and not _BARE_ID.match(cell):
            errs.append("'## Held' row's Respond in must be a bare JB-m or 'none': %r" % r)
    return errs


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
        for row in item_rows(parsed.get(s, [])):
            first = cells(row)[0]
            if first == "none" or first.startswith("none "):
                continue
            first_tok = first.split()[0] if first.split() else ""
            if not _BARE_ID.match(first_tok):
                errs.append("'## %s' row must start with a bare JB-n: %r" % (s, row))
    errs.extend(_respond_in_errors(parsed.get("Held", [])))
    return errs
