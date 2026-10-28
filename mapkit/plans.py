"""Plan loading over the three boards (``plans/active``, ``plans/parked``,
``plans/archived``).

Normalisation happens here, in the parser, never in the files:

* ``status: complete`` -> ``completed``;
* ``related_plan`` given as a string -> a one-element list;
* every date field (``created``, ``archived``, ``completed``,
  ``last_updated``, ``parked`` and each step's ``date``) -> an ISO
  ``YYYY-MM-DD`` string (PyYAML yields ``datetime.date``).

``INDEX.md`` is skipped on every board; ``PROMPT-*.md`` files are plans too.
"""
from __future__ import annotations

import datetime
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from mapkit.frontmatter import FrontmatterError, split_frontmatter
from mapkit.steps import StepHeading, extract_step_headings, normalise_step_id

BOARDS = ("active", "parked", "archived")
DATE_FIELDS = ("created", "archived", "completed", "last_updated", "parked")


class PlanError(ValueError):
    """A plan file could not be parsed; the message names the path."""


@dataclass
class Step:
    id: str
    norm_id: str
    title: str
    kind: Optional[str] = None
    status: Optional[str] = None
    date: Optional[str] = None
    effort: Optional[str] = None
    hardware: Optional[str] = None


@dataclass
class Plan:
    path: str
    board: str
    filename: str
    meta: Dict[str, Any]
    body: str
    steps: List[Step] = field(default_factory=list)
    headings: List[StepHeading] = field(default_factory=list)

    @property
    def has_steps(self) -> bool:
        return "steps" in self.meta

    @property
    def slug(self) -> str:
        return self.filename[:-3] if self.filename.endswith(".md") else self.filename


def iso_date(value: Any) -> Any:
    """``datetime.date``/``datetime.datetime`` -> ``YYYY-MM-DD``; else unchanged."""
    if isinstance(value, datetime.datetime):
        return value.date().isoformat()
    if isinstance(value, datetime.date):
        return value.isoformat()
    return value


def normalise_plan_meta(meta: Dict[str, Any]) -> Dict[str, Any]:
    """Return a normalised copy of plan frontmatter (input left untouched)."""
    out = dict(meta)
    if out.get("status") == "complete":
        out["status"] = "completed"
    rp = out.get("related_plan")
    if isinstance(rp, str):
        out["related_plan"] = [rp]
    for key in DATE_FIELDS:
        if key in out:
            out[key] = iso_date(out[key])
    steps = out.get("steps")
    if isinstance(steps, list):
        new_steps = []
        for s in steps:
            if isinstance(s, dict):
                s = dict(s)
                if "date" in s:
                    s["date"] = iso_date(s["date"])
            new_steps.append(s)
        out["steps"] = new_steps
    return out


def _parse_steps(raw: Any, where: str) -> List[Step]:
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise PlanError("%s: steps: must be a list" % where)
    out: List[Step] = []
    for i, s in enumerate(raw):
        if not isinstance(s, dict) or "id" not in s:
            raise PlanError("%s: steps[%d] must be a mapping with an id" % (where, i))
        try:
            norm = normalise_step_id(s["id"])
        except ValueError as exc:
            raise PlanError("%s: steps[%d]: %s" % (where, i, exc)) from exc
        out.append(Step(id=str(s["id"]), norm_id=norm, title=str(s.get("title", "")),
                        kind=s.get("kind"), status=s.get("status"),
                        date=s.get("date"), effort=s.get("effort"),
                        hardware=s.get("hardware")))
    return out


def load_plan(path: str, board: Optional[str] = None) -> Plan:
    """Parse one plan file. ``board`` defaults to the parent directory name."""
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    try:
        raw_meta, body = split_frontmatter(text)
    except FrontmatterError as exc:
        raise PlanError("%s: %s" % (path, exc)) from exc
    meta = normalise_plan_meta(raw_meta)
    # Offset so heading line numbers are file line numbers.
    offset = text.count("\n") - body.count("\n")
    if board is None:
        board = os.path.basename(os.path.dirname(os.path.abspath(path)))
    return Plan(path=path, board=board, filename=os.path.basename(path),
                meta=meta, body=body, steps=_parse_steps(meta.get("steps"), path),
                headings=extract_step_headings(body, line_offset=offset))


def plan_paths(repo_root: str) -> List[str]:
    """Every plan file on the three boards, board order then filename order."""
    out: List[str] = []
    for board in BOARDS:
        d = os.path.join(repo_root, "plans", board)
        if not os.path.isdir(d):
            continue
        for name in sorted(os.listdir(d)):
            if name.endswith(".md") and name != "INDEX.md":
                out.append(os.path.join(d, name))
    return out


def load_plans(repo_root: str) -> List[Plan]:
    """Load every plan; the first unparsable file raises :class:`PlanError`."""
    return [load_plan(p) for p in plan_paths(repo_root)]
