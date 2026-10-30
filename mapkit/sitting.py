"""``## Sitting`` sections on hardware Items and the Next Sitting Runbook page.

A hardware Item (``hardware: bench-leg | robot``) says what it needs from the
next Sitting in a body section headed exactly ``## Sitting``: a bullet list
with fixed keys (``setup``, ``why``, ``do``, ``pass``, ``record``, ``minutes``,
optional ``branch``). ``render_runbook`` turns every open hardware Item's
section into ``docs/runbook.md``, the Next Sitting Runbook. The generator never
invents a ``do:`` step: an Item without a valid section is listed under
"Needs a Sitting section" with the parse errors, never as a row.

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
OPTIONAL_KEYS = ("branch",)
HARDWARE = ("bench-leg", "robot")
OPEN_STATUSES = ("inbox", "backlog", "in_progress")
PRIORITY_WEIGHT = {"high": 3, "medium": 2, "low": 1}

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


@dataclass
class RunbookRow:
    item_id: str
    stem: str
    title: str
    pathway: str
    priority: str
    spec: SittingSpec
    score: float


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
    if errors:
        return None, errors
    branch = " ".join(vals["branch"]).strip().strip("`") if vals.get("branch") else None
    return SittingSpec(setup=setup, why=" ".join(vals["why"]), do=list(vals["do"]),
                       pass_=" ".join(vals["pass"]), record=" ".join(vals["record"]),
                       minutes=minutes, branch=branch or None), []


def _score(priority: str, spec: SittingSpec) -> float:
    return PRIORITY_WEIGHT.get(priority, 1) * (2 if spec.branch else 1) / float(spec.minutes)


def _bid(item: Item) -> str:
    return item.short_id or item.slug


def collect(repo_root: str) -> Dict:
    """Gather the Runbook's content from ``<repo>/map/``.

    Keys: ``rows`` (valid hardware Items by score desc, then ID), ``needs_section``
    ``(id, title, errors)``, ``questions`` ``(ticket id, title, parent id)`` for open
    question tickets under a hardware Item, and ``blocks``: rows grouped by their
    exact setup set. The greedy pass walks rows in score order; a row joins the
    block already opened for its setup set (a Sitting sets up once per set, so a
    later row never reopens a set) and opens a new block only for a set not seen
    yet. Blocks are thus ordered by their best row's score, rows within a block by
    score.
    """
    items = load_items(os.path.join(repo_root, "map"))
    hw = [i for i in items if i.kind in INBOX_KINDS and i.meta.get("hardware") in HARDWARE
          and i.status in OPEN_STATUSES]
    rows: List[RunbookRow] = []
    needs: List[Tuple[str, str, List[str]]] = []
    for i in hw:
        spec, errs = parse_sitting(i.body)
        if spec is None:
            needs.append((_bid(i), i.title, errs))
            continue
        pri = str(i.meta.get("priority") or "")
        rows.append(RunbookRow(_bid(i), "%s-%s" % (i.kind, i.slug), i.title,
                               str(i.meta.get("parent") or ""), pri, spec, _score(pri, spec)))
    rows.sort(key=lambda r: (-r.score, r.item_id))
    needs.sort()
    questions: List[Tuple[str, str, str]] = []
    for i in items:
        if i.kind in INBOX_KINDS and i.meta.get("hardware") in HARDWARE:
            for t in open_questions(items, i.short_id or "%s-%s" % (i.kind, i.slug)):
                questions.append((_bid(t), t.title, _bid(i)))
    questions = sorted(set(questions))
    blocks: List[List[RunbookRow]] = []
    by_set: Dict[Tuple[str, ...], List[RunbookRow]] = {}
    for r in rows:
        if r.spec.setup not in by_set:
            by_set[r.spec.setup] = []
            blocks.append(by_set[r.spec.setup])
        by_set[r.spec.setup].append(r)
    return {"rows": rows, "needs_section": needs, "questions": questions, "blocks": blocks}


def render_runbook(repo_root: str) -> str:
    data = collect(repo_root)
    out: List[str] = [
        "# Next Sitting Runbook", "",
        "Generated by `python -m mapkit gen` from the `## Sitting` sections of the open hardware",
        "Items and the held Night Shift branches; never hand-edited (`tests/sim/test_sitting_sections.py`).",
        "Candidate branch: `night-shift/sitting-<date>`, built at the Sitting by",
        "`tools/night_shift_candidate.sh --date <date> --push`. Existing `tests/hardware/` Runbooks are",
        "linked from rows, never duplicated.", "",
        "## Setup 0: before power", "",
        "- Power-cycle the can-bridge after any re-cabling (sticky presence set trips CAN_BUS_DOWN).",
        "- Latch `hand_source` before arming (it resets to LEGACY on every bridge reboot).",
        "- Check out the candidate branch in the live checkout only if a row below names a `branch:`.", ""]
    for n, block in enumerate(data["blocks"], 1):
        setup = ", ".join(block[0].spec.setup) or "no setup"
        out += ["## Block %d: %s  (%d rows, %d min)" % (n, setup, len(block),
                                                      sum(r.spec.minutes for r in block)), ""]
        for r in block:
            out += ["### %s %s  (%d min, %s, %s)" % (r.item_id, r.title, r.spec.minutes,
                                                     r.priority or "unrated", r.pathway or "unattached"),
                    "- why: %s" % r.spec.why, "- do:"]
            out += ["  - %s" % s for s in r.spec.do]
            out += ["- pass: %s" % r.spec.pass_, "- record: %s" % r.spec.record]
            if r.spec.branch:
                out.append("- branch: `%s`" % r.spec.branch)
            out.append("")
    out += ["## Needs a Sitting section", ""]
    out += ["- %s %s: %s" % (i, t, "; ".join(e)) for i, t, e in data["needs_section"]] or ["- none"]
    out += ["", "## Questions a Sitting can answer", ""]
    out += ["- %s %s (parent %s)" % q for q in data["questions"]] or ["- none"]
    out += ["", "## Close-out", "",
            "- Type each row's result in the map app (pass or fail, notes); it lands on the Item as "
            "`## Sitting result (<date>)`.",
            "- Then, in a Session in the live checkout: `/sitting-wrapup <date>`.", ""]
    return "\n".join(out)


def append_sitting_result(body: str, date: str, verdict: str, notes: str) -> str:
    """Append ``## Sitting result (<date>)`` before any ``## Progress`` section, else at the end."""
    if verdict not in ("pass", "fail"):
        raise ValueError("verdict must be 'pass' or 'fail', got %r" % (verdict,))
    block = "## Sitting result (%s)\n\n- verdict: %s\n- notes: %s\n" % (
        date, verdict, " ".join(notes.split()))
    lines = body.split("\n")
    for idx, line in enumerate(lines):
        if re.match(r"^##\s+progress\s*$", line, re.IGNORECASE):
            head = "\n".join(lines[:idx]).rstrip("\n")
            tail = "\n".join(lines[idx:])
            return (head + "\n\n" if head else "") + block + "\n" + tail
    return body.rstrip("\n") + ("\n\n" if body.strip() else "") + block
