"""Step headings in plan bodies, and the steps-to-headings mirror rule.

The heading rule (one rule, ticket #27)
---------------------------------------
A step heading is a markdown heading of level ``##`` to ``####`` (outside
fenced code) whose text STARTS -- after an optional ``**`` -- with a step id:

* ``Phase <ID>``, ``Rung <ID>`` or ``Stage <ID>`` (word, whitespace, token), or
* ``P<ID>`` or ``R<ID>`` (letter glued to the token),

where ``<ID>`` is ``[A-Z]?\\d+[A-Za-z]*`` with an optional lowercase hyphen
tail (``10b-cone``) and an optional ``+`` (``Rung 4+``), and the token is not
followed by a letter or digit. A heading that merely mentions a phase
mid-line (``Appendix A: Phase 0 ...``, ``0.4 Phase 0 outcome``) is not a step,
nor are the container headings (``Implementation Phases``, ``Phase Summary``,
``Implementation Phase Summary``, ``3. Phases``), which carry no id token and
are also excluded explicitly. ``S1`` is not a step prefix (soak sittings).

Id normalisation
----------------
``Phase`` and ``P`` share the canonical prefix ``P``; ``Rung`` and ``R`` share
``R``; ``Stage`` is kept distinct as ``STAGE``. The token is upper-cased:
``Phase 1`` == ``P1`` -> ``P1``; ``Phase W4a`` == ``PW4a`` -> ``PW4A``;
``Rung 2`` == ``R2`` -> ``R2``; ``Rung 4+`` -> ``R4+``;
``Stage 1a`` -> ``STAGE1A``.

Title and status suffix
-----------------------
The title is the heading text after the id and its separator (``:``, ``—``,
``–``, ``-``, ``/``; surrounding ``**`` dropped). A trailing status suffix is
free text, stripped and never read: split the title on spaced dashes
(`` — ``, `` – ``, `` - ``); the suffix starts at the LAST segment (never the
first) whose first word, after an optional ``✅``/``**``, is an UPPERCASE word
from :data:`STATUS_WORDS` (``NOT STARTED``, ``COMPLETE 2026-10-18``,
``DONE (2026-03-18)``, ``✅ COMPLETE``, ``PREPPED 2026-09-05, NOT FLOWN``...).
A trailing ``✅ ...`` separated by whitespace only is stripped too.

Summary
-------
The first non-empty paragraph after the heading (consecutive non-blank lines,
markdown left in, verbatim); empty when the next heading comes first.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from typing import TYPE_CHECKING, Dict, List, Optional

if TYPE_CHECKING:  # pragma: no cover
    from mapkit.plans import Plan

STEP_STATUSES = ("not_started", "in_progress", "complete", "retired")
STEP_KINDS = ("phase", "rung")

STATUS_WORDS = frozenset((
    "NOT", "COMPLETE", "COMPLETED", "DONE", "RETIRED", "IN", "PREPPED",
    "LANDED", "DEFERRED", "SUPERSEDED", "PARKED", "FLOWN", "STARTED",
    "BLOCKED", "CLOSED", "PASSED", "PASS", "FAILED", "ACTIVE", "DROPPED",
    "ABANDONED", "WIP", "OPEN", "MET", "CANCELLED", "SKIPPED", "PENDING",
    "PARTIAL", "MERGED", "SHIPPED",
))

_ID = r"[A-Z]?\d+[A-Za-z]*(?:-[a-z][a-z0-9]*)?\+?"

#: Matches a step heading's TEXT (the part after the ``#`` run).
STEP_HEADING_RE = re.compile(
    r"^(?P<bold>\*\*)?"
    r"(?:(?P<word>Phase|Rung|Stage)[ \t]+(?P<wid>" + _ID + r")"
    r"|(?P<letter>P|R)(?P<lid>" + _ID + r"))"
    r"(?![A-Za-z0-9])"
    r"(?P<rest>.*)$"
)

#: Container headings: excluded explicitly (they also carry no id token).
CONTAINER_RE = re.compile(
    r"^(?:\d+\.\s*)?(?:Implementation\s+)?(?:Phases?|Rungs?|Stages?)"
    r"(?:\s+Summary)?\b(?!\s+" + _ID + r")", re.IGNORECASE)

_HEADING_LINE_RE = re.compile(r"^(#{1,6})[ \t]+(.*?)[ \t]*$")
_FENCE_RE = re.compile(r"^[ \t]*(```|~~~)")
_SPACED_DASH_RE = re.compile(r"\s+[—–-]\s+")
_TRAILING_CHECK_RE = re.compile(r"\s+✅.*$")
_PREFIX = {"Phase": "P", "P": "P", "Rung": "R", "R": "R", "Stage": "STAGE"}
_NORM_RE = re.compile(
    r"^\s*(?:\*\*)?\s*(?:(Phase|Rung|Stage)\s+(" + _ID + r")|(P|R)(" + _ID
    + r"))\s*(?:\*\*)?\s*$", re.IGNORECASE)


@dataclass
class StepHeading:
    id: str            # as written, e.g. "Phase 1", "P1", "Rung 4+"
    norm_id: str       # canonical, e.g. "P1", "R4+", "STAGE1A"
    title: str         # suffix-stripped title
    level: int         # 2..4
    line_no: int       # 1-based line in the source file (body + offset)
    summary: str       # first paragraph after the heading


def normalise_step_id(raw: object) -> str:
    """Canonical form of a step id (see the module docstring).

    Raises ValueError for text that is not a step id.
    """
    text = str(raw)
    m = _NORM_RE.match(text)
    if not m:
        raise ValueError("not a step id: %r" % text)
    if m.group(1):
        word = m.group(1).capitalize()
        token = m.group(2)
    else:
        word = m.group(3).upper()
        token = m.group(4)
    return _PREFIX[word] + token.upper()


def _strip_status_suffix(title: str) -> str:
    parts = _SPACED_DASH_RE.split(title)
    seps = _SPACED_DASH_RE.findall(title)
    cut = None
    for i in range(1, len(parts)):
        first = parts[i].lstrip("*✅ \t").split(" ", 1)[0]
        first = re.split(r"[^A-Za-z]", first, 1)[0]
        if first and first.isupper() and first in STATUS_WORDS:
            cut = i
    if cut is not None:
        out = parts[0]
        for i in range(1, cut):
            out += seps[i - 1] + parts[i]
        title = out
    title = _TRAILING_CHECK_RE.sub("", title)
    title = re.sub(r"\s+[—–-]\s*$", "", title)
    return title.strip()


def _title_from_rest(rest: str, bold: bool) -> str:
    t = rest
    if bold:
        # "**Phase 1** — x" or "**Phase 1 — x**": drop the closing marker.
        if t.startswith("**"):
            t = t[2:]
        elif t.rstrip().endswith("**"):
            t = t.rstrip()[:-2]
    t = re.sub(r"^[\s:—–\-/]+", "", t)
    return _strip_status_suffix(t)


def _is_heading(line: str) -> bool:
    return bool(_HEADING_LINE_RE.match(line))


def _summary_after(lines: List[str], idx: int) -> str:
    j = idx + 1
    while j < len(lines) and not lines[j].strip():
        j += 1
    para: List[str] = []
    while j < len(lines) and lines[j].strip() and not _is_heading(lines[j]):
        para.append(lines[j].rstrip())
        j += 1
    return "\n".join(para)


def parse_step_heading_text(text: str) -> Optional[Dict[str, str]]:
    """Return ``{id, norm_id, title}`` when heading text is a step, else None."""
    if CONTAINER_RE.match(text):
        return None
    m = STEP_HEADING_RE.match(text)
    if not m:
        return None
    if m.group("word"):
        raw_id = "%s %s" % (m.group("word"), m.group("wid"))
    else:
        raw_id = m.group("letter") + m.group("lid")
    return {
        "id": raw_id,
        "norm_id": normalise_step_id(raw_id),
        "title": _title_from_rest(m.group("rest"), bool(m.group("bold"))),
    }


def extract_step_headings(body: str, line_offset: int = 0) -> List[StepHeading]:
    """Every step heading in ``body`` (fenced code skipped), in order.

    ``line_offset`` is added to the 1-based body line number so callers can
    report file line numbers.
    """
    lines = body.split("\n")
    out: List[StepHeading] = []
    in_fence = False
    for i, line in enumerate(lines):
        if _FENCE_RE.match(line):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        m = _HEADING_LINE_RE.match(line)
        if not m or not (2 <= len(m.group(1)) <= 4):
            continue
        parsed = parse_step_heading_text(m.group(2))
        if parsed is None:
            continue
        out.append(StepHeading(
            id=parsed["id"], norm_id=parsed["norm_id"], title=parsed["title"],
            level=len(m.group(1)), line_no=i + 1 + line_offset,
            summary=_summary_after(lines, i)))
    return out


def normalise_title(title: str) -> str:
    """Mirror comparison form: case-folded, whitespace collapsed, trailing
    punctuation stripped."""
    t = " ".join(str(title).split()).casefold()
    return t.rstrip(" .,:;!?")


def check_mirror(plan: "Plan") -> List[str]:
    """Offender messages for the mirror rule; empty means the plan passes.

    Only asserted when the plan's frontmatter has ``steps:``: every listed id
    matches exactly one heading (after normalisation), every step heading is
    listed, ids are listed once, and listed titles equal heading titles
    (:func:`normalise_title`). With no ``steps:`` nothing is asserted.
    """
    if not plan.has_steps:
        return []
    msgs: List[str] = []
    name = plan.filename
    by_id: Dict[str, List[StepHeading]] = {}
    for h in plan.headings:
        by_id.setdefault(h.norm_id, []).append(h)
    listed: Dict[str, int] = {}
    for s in plan.steps:
        listed[s.norm_id] = listed.get(s.norm_id, 0) + 1
    for nid, n in listed.items():
        if n > 1:
            msgs.append("%s: step %s listed %d times in steps:" % (name, nid, n))
    for s in plan.steps:
        hs = by_id.get(s.norm_id, [])
        if not hs:
            msgs.append("%s: step %s (%r) has no heading" % (name, s.id, s.title))
        elif len(hs) > 1:
            msgs.append("%s: step %s matches %d headings (lines %s)" % (
                name, s.id, len(hs), ", ".join(str(h.line_no) for h in hs)))
        elif normalise_title(hs[0].title) != normalise_title(s.title):
            msgs.append("%s: step %s title %r != heading title %r (line %d)" % (
                name, s.id, s.title, hs[0].title, hs[0].line_no))
    for h in plan.headings:
        if h.norm_id not in listed:
            msgs.append("%s: heading %r (line %d) is not listed in steps:" % (
                name, h.id, h.line_no))
    return msgs


def check_step_vocab(plan: "Plan") -> List[str]:
    """Offenders whose step ``kind``/``status`` fall outside the vocabulary."""
    msgs: List[str] = []
    for s in plan.steps:
        if s.status is not None and s.status not in STEP_STATUSES:
            msgs.append("%s: step %s status %r not in %s" % (
                plan.filename, s.id, s.status, STEP_STATUSES))
        if s.kind is not None and s.kind not in STEP_KINDS:
            msgs.append("%s: step %s kind %r not in %s" % (
                plan.filename, s.id, s.kind, STEP_KINDS))
    return msgs
