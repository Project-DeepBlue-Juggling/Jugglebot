"""Night Shift report layout: the nine sections and a checker (T-U7).

The report is ``docs/night-shift/<date>.md`` (layout in
``docs/night-shift/README.md``). The map app renders it (GFM, tables included)
and links each Item by its bare Item ID (``BUG-/FEAT-/IDEA-/PWAY-/TKT-n``), so a row that names an Item any other
way is an error here. Item sections are GFM tables since 2026-11-01 (owner:
readability); a table's header row and its ``|---|`` rule are layout, not rows,
and the older bare ``<ID> | ...`` lines still pass. Pure functions, stdlib only.
"""
from __future__ import annotations

import re
from typing import Dict, List

INPUTS_SECTION = "Items the Night Shift could resolve with a small amount of input from you"
SECTIONS = (
    "Nightly",
    "Merged to main",
    "Held",
    "Questions for the Operator",
    INPUTS_SECTION,
    "Unfinished",
    "Skipped",
    "Unaddressed Sitting results",
    "Cost",
)
#: Reports dated before this keep passing as eight-section reports (no INPUTS_SECTION);
#: reports dated on or after it must carry all nine (night-shift close-out C5).
INPUTS_CUTOVER = "2026-11-15"
LEGACY_SECTIONS = tuple(s for s in SECTIONS if s != INPUTS_SECTION)
#: The generated section's header row (``python -m mapkit night inputs`` writes it).
INPUTS_HEADER = ("Item", "What it is", "Input", "Exactly what is needed",
                 "What the night then does", "Respond in")
INPUT_TYPES = ("fact", "decision", "sitting")
#: Skipped ends with exactly this one line: the count of Items the night did not re-examine.
_UNCHANGED_LINE = re.compile(
    r"^unchanged: \d+ Items not re-examined \(reasons on each Item: night_skip\)$")
# sections whose every row begins with the Item it is about
# (INPUTS_SECTION is deliberately NOT one: it is a standing generated list of every open
# input-* Item, and counting it would mark each of them walked tonight in the carry rule.)
ITEM_ROW_SECTIONS = ("Merged to main", "Held", "Questions for the Operator", "Skipped",
                     "Unaddressed Sitting results")
_TITLE = re.compile(r"^# Night Shift (\d{4}-\d{2}-\d{2})$")
#: The bare Item ID; the prefixes are ``mapkit.ids.ALL_PREFIXES`` (pinned by a test,
#: kept literal here so the checker stays stdlib-only).
_BARE_ID = re.compile(r"^(?:BUG|FEAT|IDEA|PWAY|TKT)-[1-9]\d*$")
RESPOND_IN = "Respond in"
_RULE_CELL = re.compile(r"^:?-{3,}:?$")
#: Questions ends with exactly this one line in a nine-section report (the count of open
#: question tickets from earlier nights that are not re-listed); never an Item row.
_EARLIER_LINE = re.compile(r"^earlier: \d+ open questions from earlier nights, not re-listed$")


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
    every row with a bare ``TKT-m`` (the question ticket) or ``none`` (a hold only a Sitting
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
            errs.append("'## Held' row's Respond in must be a bare Item ID (BUG-/FEAT-/IDEA-/PWAY-/TKT-m) or 'none': %r" % r)
    return errs


def _inputs_errors(rows: List[str]) -> List[str]:
    """The generated section: ``none`` / ``none recorded``, or a table with the fixed header
    whose rows name an Item, an Input type and a Respond in (bare ID, ``sheet`` or ``none``)."""
    if rows in (["none"], ["none recorded"]) or not rows:
        return [] if rows else ["'## %s' is empty: write 'none'" % INPUTS_SECTION]
    if len(rows) < 2 or not is_table_rule(rows[1]) or tuple(cells(rows[0])) != INPUTS_HEADER:
        return ["'## %s' must be 'none' or a table headed | %s |" % (INPUTS_SECTION, " | ".join(INPUTS_HEADER))]
    errs: List[str] = []
    for r in rows[2:]:
        cs = cells(r)
        if len(cs) != len(INPUTS_HEADER):
            errs.append("'## %s' row has %d cells, want %d: %r" % (INPUTS_SECTION, len(cs), len(INPUTS_HEADER), r))
        elif not _BARE_ID.match(cs[0]):
            errs.append("'## %s' row must start with a bare Item ID: %r" % (INPUTS_SECTION, r))
        elif cs[2] not in INPUT_TYPES:
            errs.append("'## %s' row's Input must be one of %s: %r" % (INPUTS_SECTION, "/".join(INPUT_TYPES), r))
        elif cs[5] not in ("none", "sheet") and not _BARE_ID.match(cs[5]):
            errs.append("'## %s' row's Respond in must be a bare Item ID, 'sheet' or 'none': %r" % (INPUTS_SECTION, r))
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
    m = _TITLE.match(first.rstrip())
    heads = _headings(text)
    # before the cutover a report may still carry the new section (a stub written early)
    legacy = (not m or m.group(1) < INPUTS_CUTOVER) and INPUTS_SECTION not in heads  # no date: the title error speaks
    wanted = LEGACY_SECTIONS if legacy else SECTIONS
    for s in wanted:
        if s not in heads:
            errs.append("missing section '## %s'" % s)
    present = [h for h in heads if h in SECTIONS]
    if not any(e.startswith("missing") for e in errs) and present != list(wanted):
        errs.append("sections out of order: found %s, want %s" % (present, list(wanted)))
    parsed = parse_report(text)
    for s in ITEM_ROW_SECTIONS:
        rows = parsed.get(s, [])
        if s == "Skipped" and rows and _UNCHANGED_LINE.match(rows[-1]):
            rows = rows[:-1]  # the one trailer line, allowed only as Skipped's last line
        if s == "Questions for the Operator" and not legacy and rows and _EARLIER_LINE.match(rows[-1]):
            rows = rows[:-1]  # likewise, in a nine-section report only
        for row in item_rows(rows):
            first = cells(row)[0]
            if first == "none" or first.startswith("none "):
                continue
            first_tok = first.split()[0] if first.split() else ""
            if not _BARE_ID.match(first_tok):
                errs.append("'## %s' row must start with a bare Item ID (BUG-/FEAT-/IDEA-/PWAY-/TKT-n): %r" % (s, row))
    errs.extend(_respond_in_errors(parsed.get("Held", [])))
    if INPUTS_SECTION in parsed:
        errs.extend(_inputs_errors(parsed[INPUTS_SECTION]))
    return errs
