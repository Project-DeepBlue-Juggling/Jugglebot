"""Computed, never stored (ticket #38): pathway order, progress, Next, lanes.

Pure functions over loaded :class:`~mapkit.plans.Plan` and
:class:`~mapkit.items.Item` lists; no I/O, no clock, so every result is
deterministic. The generated ``docs/project-map.md`` and the fork's server
both import from here.

* **order**: (priority high > medium > low, status active < complete <
  dropped, title). A missing priority counts as medium.
* **progress units**: a plan's ``steps:``, else the plan itself as one unit.
  ``retired`` steps are excluded from the denominator. A unit is done when
  its step is ``complete``, or when its plan's status is
  ``completed``/``superseded``.
* **Next**: the first unit that is neither done nor retired of the first plan
  (filename order, ``active`` board before ``parked``) with no open blocker;
  a parked plan can therefore be the Next stop when the pathway has nothing
  active, which is deliberate: its unpark gate is what the pathway waits on.
  A plan carrying ``blocked_by`` (hook; no plan does today) waits while any
  target is open; when every open plan waits, Next is the first blocker's
  title.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from mapkit.items import INBOX_KINDS, Item
from mapkit.plans import Plan
from mapkit.tickets import is_open_blocker

_PRIORITY_RANK = {"high": 0, "medium": 1, "low": 2}
_PATHWAY_STATUS_RANK = {"active": 0, "complete": 1, "dropped": 2}
_DONE_PLAN_STATUSES = ("completed", "superseded")
_OPEN_BOARDS = ("active", "parked")
OPEN_INBOX_STATUSES = ("inbox", "backlog", "in_progress")


@dataclass
class Stop:
    """One progress unit: a plan step, or a whole plan with no ``steps:``."""
    label: str             # step title, or the plan title for a plan unit
    plan: str              # plan filename
    board: str
    step_id: Optional[str]  # None for a plan-as-unit
    status: str            # not_started | in_progress | complete | retired
    kind: str = "plan"     # "plan" | "blocker"


def pathway_order(items: List[Item]) -> List[Item]:
    """Pathway items in Front-page order."""
    paths = [i for i in items if i.kind == "pathway"]
    return sorted(paths, key=lambda p: (
        _PRIORITY_RANK.get(p.meta.get("priority") or "medium", 1),
        _PATHWAY_STATUS_RANK.get(p.status, 3),
        str(p.title)))


def pathway_plans(pathway: Item, plans: List[Plan]) -> List[Plan]:
    """Plans of the pathway: ``active`` then ``parked`` then ``archived``,
    filename order inside a board."""
    mine = [p for p in plans if p.meta.get("pathway") == pathway.slug]
    boards = _OPEN_BOARDS + ("archived",)
    return sorted(mine, key=lambda p: (boards.index(p.board) if p.board in boards
                                       else len(boards), p.filename))


def _plan_title(plan: Plan) -> str:
    return str(plan.meta.get("title") or plan.slug)


def units(pathway: Item, plans: List[Plan]) -> List[Stop]:
    """Every progress unit (retired ones included, flagged), in plan order."""
    out: List[Stop] = []
    for plan in pathway_plans(pathway, plans):
        plan_done = plan.meta.get("status") in _DONE_PLAN_STATUSES
        if plan.has_steps and plan.steps:
            for s in plan.steps:
                status = s.status or "not_started"
                if plan_done and status != "retired":
                    status = "complete"
                out.append(Stop(s.title or s.id, plan.filename, plan.board, s.id, status))
        else:
            status = "complete" if plan_done else (
                "in_progress" if plan.meta.get("status") == "in_progress" else "not_started")
            out.append(Stop(_plan_title(plan), plan.filename, plan.board, None, status))
    return out


def progress(pathway: Item, plans: List[Plan]) -> Tuple[int, int]:
    """``(done, total)`` with retired units out of the denominator."""
    live = [u for u in units(pathway, plans) if u.status != "retired"]
    return sum(1 for u in live if u.status == "complete"), len(live)


def attached(pathway: Item, items: List[Item]) -> List[Item]:
    """Open inbox items (idea/feature/bug) filed against the pathway."""
    return [i for i in items if i.kind in INBOX_KINDS
            and i.meta.get("parent") == pathway.slug
            and i.status in OPEN_INBOX_STATUSES]


def open_tickets(pathway: Item, items: List[Item]) -> List[Item]:
    """Open tickets whose parent is the pathway, by (rank, slug)."""
    t = [i for i in items if i.kind == "ticket" and i.status == "open"
         and i.meta.get("parent") == pathway.slug]
    return sorted(t, key=lambda i: (i.meta.get("rank") if isinstance(i.meta.get("rank"), int)
                                    else 10 ** 9, i.slug))


def _open_blockers(plan: Plan, items: List[Item], plans: List[Plan]) -> List[str]:
    by_stem = {"%s-%s" % (i.kind, i.slug): i for i in items}
    by_name = {p.filename: p for p in plans}
    return [t for t in (plan.meta.get("blocked_by") or [])
            if is_open_blocker(t, by_stem, by_name)]


def _blocker_title(target: str, items: List[Item]) -> str:
    for i in items:
        if "%s-%s" % (i.kind, i.slug) == target:
            return str(i.title)
    return target


def next_stop(pathway: Item, plans: List[Plan], items: List[Item]) -> Optional[Stop]:
    """The computed Next stop, or None when nothing is open."""
    if pathway.status != "active":
        return None
    first_blocker: Optional[str] = None
    for plan in pathway_plans(pathway, plans):
        if plan.board not in _OPEN_BOARDS or plan.meta.get("status") in _DONE_PLAN_STATUSES:
            continue
        blockers = _open_blockers(plan, items, plans)
        if blockers:
            if first_blocker is None:
                first_blocker = blockers[0]
            continue
        for u in units(pathway, [plan]):
            if u.status not in ("complete", "retired"):
                return u
    if first_blocker is not None:
        return Stop(_blocker_title(first_blocker, items), "", "", None,
                    "not_started", kind="blocker")
    return None


def lanes(pathway: Item, plans: List[Plan], items: List[Item]) -> Dict[str, list]:
    """``{"now", "next", "later", "done"}``: Now = in-progress stops, Next =
    the computed next (one entry or none), Later = remaining not-started stops
    then attached inbox items, Done = complete stops."""
    us = [u for u in units(pathway, plans) if u.status != "retired"]
    nxt = next_stop(pathway, plans, items)
    now = [u for u in us if u.status == "in_progress"]
    nxt_key = (nxt.plan, nxt.step_id) if nxt is not None and nxt.kind == "plan" else None
    later = [u for u in us if u.status == "not_started"
             and (u.plan, u.step_id) != nxt_key]
    later_all = later + attached(pathway, items)
    now_next = [] if nxt is None or nxt in now or (
        nxt_key is not None and any((u.plan, u.step_id) == nxt_key for u in now)) else [nxt]
    return {"now": now, "next": now_next, "later": later_all,
            "done": [u for u in us if u.status == "complete"]}
