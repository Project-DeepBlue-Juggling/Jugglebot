"""``## Sitting`` sections on hardware Items and the Next Sitting Runbook page.

A hardware Item (``hardware: bench-leg | bench-hand | robot``) says what it needs from the
next Sitting in a body section headed exactly ``## Sitting``: a bullet list
with fixed keys (``setup``, ``why``, ``do``, ``pass``, ``record``, ``minutes``,
optional ``branch`` and ``first``). ``render_runbook`` turns every open hardware Item's
section into ``docs/runbook.md``, the Next Sitting Runbook. The generator never
invents a ``do:`` step: an Item without a valid section is listed under
"Needs a Sitting section" with the parse errors, never as a row.

The Runbook is an index, not an ordering authority (close-out C11): per rig a table of the
waiting Items (minutes, setup set, held branch, and the open Sitting sheet that includes the
Item), sorted by (rig, priority, ID). The order a Sitting flies in is the sheet's
(``/sitting-sheet <rig>``, ``mapkit/sheet.py``). ``first:`` is still parsed, and ignored here.

A result typed in the map app lands on the Item as ``## Sitting result (<date>)`` or, since
2026-11-03, ``## Sitting result (<date> HH:MM)`` (24-hour local clock, the time it was saved);
``parse_sitting_results`` reads both forms.

The page carries no wall-clock date, so ``python -m mapkit gen --check`` can
tell a stale committed page from a fresh one.
"""
from __future__ import annotations

import os
import re
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from mapkit.items import INBOX_KINDS, Item, load_items, open_questions

SETUPS = ("level", "tilt", "armed", "bb-fed", "bridge-cycled")
REQUIRED_KEYS = ("setup", "why", "do", "pass", "record", "minutes")
OPTIONAL_KEYS = ("branch", "first")
HARDWARE = ("robot", "bench-leg", "bench-hand")  # also the rig display order
RIG_LABELS = {"robot": "Robot", "bench-leg": "Bench leg", "bench-hand": "Bench hand"}
OPEN_STATUSES = ("inbox", "backlog", "in_progress")
PRIORITY_ORDER = {"high": 0, "medium": 1, "low": 2}

_HEADING = re.compile(r"^##\s+Sitting\s*$")
_ANY_H2 = re.compile(r"^##\s")
_KEY = re.compile(r"^[-*]\s+([A-Za-z][A-Za-z-]*)\s*:\s*(.*)$")
_SUB = re.compile(r"^\s+[-*]\s+(.*)$")


@dataclass
class SittingSpec:
    setup: Tuple[str, ...]
    why: str
    do: List[str]
    pass_: str
    record: str
    minutes: int
    branch: Optional[str] = None
    first: Optional[str] = None  # parsed, ignored by the Runbook (the sheet orders)


@dataclass
class RunbookRow:
    item_id: str
    stem: str
    title: str
    pathway: str
    priority: str
    spec: SittingSpec
    score: float = 0.0  # compatibility only (the app of 2026-11-14 reads it); always 0
    hardware: str = "robot"
    on_sheet: str = ""  # name of the open sheet that includes this Item, else ""


def has_sitting_section(body: str) -> bool:
    return any(_HEADING.match(line) for line in body.splitlines())


def _section_lines(body: str) -> List[str]:
    out: List[str] = []
    inside = False
    for line in body.splitlines():
        if _HEADING.match(line):
            inside = True
        elif _ANY_H2.match(line):
            inside = False
        elif inside:
            out.append(line)
    return out


def parse_sitting(body: str) -> Tuple[Optional[SittingSpec], List[str]]:
    """Parse the ``## Sitting`` section: ``(spec, [])`` or ``(None, errors)``."""
    if not has_sitting_section(body):
        return None, ["no ## Sitting section"]
    errors: List[str] = []
    vals: Dict[str, List[str]] = {}
    cur: Optional[str] = None
    for raw in _section_lines(body):
        if not raw.strip():
            continue
        m = _KEY.match(raw)
        if m:
            key, val = m.group(1).lower(), m.group(2).strip()
            if key not in REQUIRED_KEYS + OPTIONAL_KEYS:
                errors.append("unknown key '%s'" % key)
                cur = None
                continue
            vals.setdefault(key, [])
            cur = key
            if val:
                vals[key].append(val)
            continue
        sub = _SUB.match(raw)
        if cur is None:
            if raw.lstrip().startswith(("-", "*")) or raw.strip():
                errors.append("text outside a key: '%s'" % raw.strip()[:40])
            continue
        if sub:
            vals[cur].append(sub.group(1).strip())
        elif raw[0] in " \t":
            vals[cur].append(raw.strip())
        elif raw.startswith(("- ", "* ")) and cur == "do":
            vals[cur].append(raw[2:].strip())  # a further top-level bullet is another do step
        else:
            errors.append("text outside a key: '%s'" % raw.strip()[:40])
    for key in REQUIRED_KEYS:
        if not vals.get(key):
            errors.append("missing required key '%s'" % key)
    setup: Tuple[str, ...] = ()
    if vals.get("setup"):
        words = [w.strip() for w in " ".join(vals["setup"]).split(",") if w.strip()]
        for w in words:
            if w not in SETUPS:
                errors.append("unknown setup word '%s' (one of %s)" % (w, ", ".join(SETUPS)))
        setup = tuple(w for w in SETUPS if w in words)
    minutes = 0
    if vals.get("minutes"):
        text = " ".join(vals["minutes"]).strip()
        if re.fullmatch(r"\d+", text) and int(text) > 0:
            minutes = int(text)
        else:
            errors.append("minutes must be a positive integer, got '%s'" % text)
    if "first" in vals and not " ".join(vals["first"]).strip():
        errors.append("first needs a reason (one line saying why this row is flown first)")
    if errors:
        return None, errors
    branch = " ".join(vals["branch"]).strip().strip("`") if vals.get("branch") else None
    first = " ".join(vals["first"]).strip() if vals.get("first") else None
    return SittingSpec(setup=setup, why=" ".join(vals["why"]), do=list(vals["do"]),
                       pass_=" ".join(vals["pass"]), record=" ".join(vals["record"]),
                       minutes=minutes, branch=branch or None, first=first or None), []


def _bid(item: Item) -> str:
    return item.short_id or item.slug


def _open_sheets(repo_root: str, items: List[Item]) -> Dict[str, str]:
    """Item ID -> name of the newest sheet that serves it and is still open for it.

    A sheet is open for an Item until the Item carries a ``## Sitting result`` dated on or
    after the sheet's date. Derived from the Items and the sheet files only (never from the
    app's ``.results.yaml``), so ``docs/runbook.md`` stays reproducible from a commit."""
    from mapkit import sheet as sheet_mod  # lazy: sheet imports this module
    done: Dict[str, str] = {}
    for i in items:
        dates = [r["date"] or "" for r in parse_sitting_results(i.body)]
        if dates:
            done[_bid(i)] = max(dates)
    out: Dict[str, Tuple[str, str]] = {}
    try:
        listing = sheet_mod.list_sheets(repo_root)  # newest first
    except Exception:  # a bad sheet must not take the Runbook down
        return {}
    for row in reversed(listing):  # oldest first, so the newest overwrites
        try:
            data = sheet_mod.load_sheet(sheet_mod.sheet_path(repo_root, row["name"]))
        except Exception:
            continue
        for st in data.get("steps") or []:
            for sv in (st.get("serves") or []) if isinstance(st, dict) else []:
                item = str(sv.get("item")) if isinstance(sv, dict) else ""
                if item and done.get(item, "") < str(data.get("date", "")):
                    out[item] = (row["name"], str(data.get("date", "")))
    return {k: v[0] for k, v in out.items()}


def collect(repo_root: str) -> Dict:
    """Gather the Runbook's content from ``<repo>/map/``.

    Keys: ``rows`` (valid hardware Items sorted by rig, priority high to low, then ID),
    ``needs_section`` ``(id, title, errors, hardware)``, ``questions`` ``(ticket id, title,
    parent id)`` for open question tickets under a hardware Item, and, as compatibility fields
    for a map app built before 2026-11-14, ``rigs`` (``(hardware, blocks)`` in rig order for
    rigs with a row) and ``blocks`` (one block per rig holding that rig's rows, every score 0).
    The order a Sitting flies in is the sheet's, not this one's.
    """
    items = load_items(os.path.join(repo_root, "map"))
    hw = [i for i in items if i.kind in INBOX_KINDS and i.meta.get("hardware") in HARDWARE
          and i.status in OPEN_STATUSES]
    rows: List[RunbookRow] = []
    needs: List[Tuple[str, str, List[str], str]] = []
    for i in hw:
        spec, errs = parse_sitting(i.body)
        if spec is None:
            needs.append((_bid(i), i.title, errs, str(i.meta.get("hardware"))))
            continue
        pri = str(i.meta.get("priority") or "")
        rows.append(RunbookRow(_bid(i), "%s-%s" % (i.kind, i.slug), i.title,
                               str(i.meta.get("parent") or ""), pri, spec, 0.0,
                               str(i.meta.get("hardware"))))
    if rows:
        on = _open_sheets(repo_root, hw)
        for r in rows:
            r.on_sheet = on.get(r.item_id, "")
    rows.sort(key=lambda r: (HARDWARE.index(r.hardware), PRIORITY_ORDER.get(r.priority, 3),
                             r.item_id))
    needs.sort()
    questions: List[Tuple[str, str, str]] = []
    for i in items:
        if i.kind in INBOX_KINDS and i.meta.get("hardware") in HARDWARE:
            for t in open_questions(items, i.short_id or "%s-%s" % (i.kind, i.slug)):
                questions.append((_bid(t), t.title, _bid(i)))
    questions = sorted(set(questions))
    blocks: List[List[RunbookRow]] = []
    rigs: List[Tuple[str, List[List[RunbookRow]]]] = []
    for rig in HARDWARE:
        rig_rows = [r for r in rows if r.hardware == rig]
        if rig_rows:
            rigs.append((rig, [rig_rows]))
            blocks.append(rig_rows)
    return {"rows": rows, "needs_section": needs, "questions": questions, "blocks": blocks,
            "rigs": rigs}


def _cell(text: str) -> str:
    return text.replace("|", "\\|")


def render_runbook(repo_root: str) -> str:
    data = collect(repo_root)
    out: List[str] = [
        "# Next Sitting Runbook", "",
        "Generated by `python -m mapkit gen` from the `## Sitting` sections of the open hardware",
        "Items; never hand-edited (`tests/sim/test_sitting_sections.py`). This page is an index of the",
        "waiting Items: the order a Sitting flies in, and every step, is on its Sitting sheet.", "",
        "Fly from a Sitting sheet: /sitting-sheet robot|bench-leg|bench-hand", ""]
    for rig in HARDWARE:
        rows = [r for r in data["rows"] if r.hardware == rig]
        if not rows:
            continue
        out += ["## %s  (%d Item%s, %d min)" % (RIG_LABELS[rig], len(rows),
                                               "" if len(rows) == 1 else "s",
                                               sum(r.spec.minutes for r in rows)), "",
                "| Item | Minutes | Setup | Branch | On a sheet |",
                "| --- | ---: | --- | --- | --- |"]
        for r in rows:
            out.append("| %s | %d | %s | %s | %s |" % (
                _cell("%s %s" % (r.item_id, r.title)), r.spec.minutes,
                ", ".join(r.spec.setup) or "-", ("`%s`" % r.spec.branch) if r.spec.branch else "-",
                r.on_sheet or "-"))
        out.append("")
    out += ["## Needs a Sitting section", ""]
    out += ["- %s [%s] %s: %s" % (i, RIG_LABELS.get(h, h), t, "; ".join(e))
            for i, t, e, h in data["needs_section"]] or ["- none"]
    out += ["", "## Questions a Sitting can answer", ""]
    out += ["- %s %s (parent %s)" % q for q in data["questions"]] or ["- none"]
    out.append("")
    return "\n".join(out)


# A Sitting result's verdict. ``pending``: the row was flown but its pass needs logs or an
# analysis the operator cannot do live; ``/sitting-debrief`` derives the verdict. ``data``: flown
# for the record, no criterion to judge (FEAT-42); ``inconclusive``: flown but the logs cannot say.
# The map app reads this tuple, so a new verdict lands here first.
VERDICTS = ("pass", "fail", "pending", "data", "inconclusive")


RESULT_HEADING = re.compile(r"^##\s+Sitting result \((\d{4}-\d{2}-\d{2})(?: (\d{2}:\d{2}))?\)\s*$")
_DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")
_TIME_RE = re.compile(r"^(\d{2}):(\d{2})$")


def append_sitting_result(body: str, date: str, verdict: str, notes: str,
                          time: Optional[str] = None) -> str:
    """Append ``## Sitting result (<date>)`` (``(<date> HH:MM)`` with ``time``) before any
    ``## Progress`` section, else at the end."""
    if verdict not in VERDICTS:
        raise ValueError("verdict must be one of %s, got %r" % (", ".join(VERDICTS), verdict))
    if not _DATE_RE.fullmatch(date) or not (1 <= int(date[5:7]) <= 12 and 1 <= int(date[8:10]) <= 31):
        raise ValueError("date must be YYYY-MM-DD, got %r" % (date,))
    stamp = date
    if time is not None:
        m = _TIME_RE.fullmatch(time)
        if not m or int(m.group(1)) > 23 or int(m.group(2)) > 59:
            raise ValueError("time must be HH:MM (00:00-23:59), got %r" % (time,))
        stamp = "%s %s" % (date, time)
    block = "## Sitting result (%s)\n\n- verdict: %s\n- notes: %s\n" % (
        stamp, verdict, " ".join(notes.split()))
    lines = body.split("\n")
    for idx, line in enumerate(lines):
        if re.match(r"^##\s+progress\s*$", line, re.IGNORECASE):
            head = "\n".join(lines[:idx]).rstrip("\n")
            tail = "\n".join(lines[idx:])
            return (head + "\n\n" if head else "") + block + "\n" + tail
    return body.rstrip("\n") + ("\n\n" if body.strip() else "") + block


def parse_sitting_results(body: str) -> List[Dict[str, Optional[str]]]:
    """Every ``## Sitting result`` section in file order: date, time (or None), verdict, notes."""
    out: List[Dict[str, Optional[str]]] = []
    cur: Optional[Dict[str, Optional[str]]] = None
    last = ""  # the bullet key the previous line set, for indented notes continuations
    for line in body.split("\n"):
        m = RESULT_HEADING.match(line)
        if m:
            cur = {"date": m.group(1), "time": m.group(2), "verdict": "", "notes": ""}
            last = ""
            out.append(cur)
        elif re.match(r"^##\s", line):
            cur = None
        elif cur is not None:
            # ``- notes:`` as written by the app, or a hand-edited variant such as
            # ``- notes (split into bullets):`` whose text continues in indented bullets.
            b = re.match(r"^[-*]\s+(verdict|notes)\b[^:]*:\s*(.*?)\s*$", line)
            if b:
                cur[b.group(1)] = b.group(2)
                last = b.group(1)
            elif last == "notes" and re.match(r"^\s+[-*]\s+\S", line):
                cur["notes"] = (cur["notes"] + " " + line.strip()[2:].strip()).strip()
    return out
