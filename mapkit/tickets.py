"""Ticket commands: ``frontier``, ``claim``, ``resolve`` (decisions #34, #30).

A ticket is ``map/ticket-<slug>.md``. The frontier of a pathway is its open,
unowned tickets none of whose ``blocked_by`` targets is still open, lowest
``rank`` first. ``claim`` and ``resolve`` rewrite one ticket file and commit
exactly that file (no ``Logbook-Entry`` trailer), then push.
"""
from __future__ import annotations

import datetime
import os
import subprocess
import sys
from typing import Dict, List

from mapkit.items import INBOX_KINDS, Item, ItemError, load_item, load_items, write_item
from mapkit.plans import load_plans

MAP_DIR = "map"
_TERMINAL_INBOX = ("complete", "dropped", "promoted")


class TicketError(Exception):
    """A refused ticket operation; exit code 2."""


def _now() -> str:
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def _stem(item: Item) -> str:
    return item.kind + "-" + item.slug


def is_open_blocker(target: str, items_by_slug: Dict[str, Item],
                    plans_by_filename: Dict[str, object]) -> bool:
    """True when ``target`` still blocks; an unresolvable target blocks.

    Item targets are keyed by filename stem (``ticket-foo``).
    """
    item = items_by_slug.get(target)
    if item is not None:
        if item.kind == "ticket":
            return item.status == "open"
        if item.kind == "pathway":
            return item.status == "active"
        if item.kind in INBOX_KINDS:
            return item.status not in _TERMINAL_INBOX
        return True
    plan = plans_by_filename.get(target)
    if plan is not None:
        return getattr(plan, "board") in ("active", "parked")
    return True


def _rank_key(item: Item):
    rank = item.meta.get("rank")
    return (rank if isinstance(rank, int) else 10 ** 9, item.slug)


def frontier(repo_root: str, pathway: str) -> List[Item]:
    items = load_items(os.path.join(repo_root, MAP_DIR))
    by_stem = {_stem(i): i for i in items}
    plans = {p.filename: p for p in load_plans(repo_root)}
    out: List[Item] = []
    for it in items:
        if it.kind != "ticket" or it.status != "open":
            continue
        if it.meta.get("type") == "question":
            continue  # a question hangs off an Item, not a Pathway: never in a frontier
        if it.meta.get("parent") != pathway or it.meta.get("owners"):
            continue
        blocked = False
        for target in it.meta.get("blocked_by") or []:
            if is_open_blocker(target, by_stem, plans):
                blocked = True
                if target not in by_stem and target not in plans:
                    print("mapkit: %s: unresolvable blocker %r (counted open)"
                          % (it.slug, target), file=sys.stderr)
        if not blocked:
            out.append(it)
    out.sort(key=_rank_key)
    return out


def _load_ticket(repo_root: str, slug: str) -> Item:
    """A ticket by stem, slug or ``jb`` ID (write forms: no bare digits)."""
    from mapkit.itemcli import ItemCliError, find_item  # local: itemcli is the CLI layer
    try:
        item = find_item(repo_root, slug)
    except ItemCliError as exc:
        raise TicketError(str(exc))
    if item.kind != "ticket":
        raise TicketError("%r is a %s (%s), not a ticket" % (slug, item.kind, _stem(item)))
    return item


def _label(item: Item) -> str:
    """``JB-9 <slug>`` (or the slug alone for an ID-less ticket): subjects and echo."""
    return ("%s %s" % (item.short_id, item.slug)) if item.short_id else item.slug


def _git(repo_root: str, *args: str) -> None:
    subprocess.run(["git"] + list(args), cwd=repo_root, check=True)


def _commit(repo_root: str, item: Item, message: str, push: bool) -> None:
    rel = os.path.relpath(item.path, repo_root)
    _git(repo_root, "add", "--", rel)
    _git(repo_root, "commit", "-m", message, "--", rel)
    if push:
        _git(repo_root, "push")


def claim(repo_root: str, slug: str, user: str, push: bool = True) -> Item:
    item = _load_ticket(repo_root, slug)
    if item.status != "open":
        raise TicketError("ticket %r is %s, not open" % (slug, item.status))
    owners = list(item.meta.get("owners") or [])
    if user in owners:
        return item  # idempotent: already claimed by this user, nothing to commit
    owners.append(user)
    item.meta["owners"] = owners
    item.meta["updated"] = _now()
    write_item(item.path, item.meta, item.body)
    _commit(repo_root, item, "map: claim %s" % _label(item), push)
    return item


def resolve(repo_root: str, slug: str, answer: str, push: bool = True) -> Item:
    item = _load_ticket(repo_root, slug)
    if item.status == "complete":
        raise TicketError("ticket %r is already complete" % slug)
    body = item.body.rstrip("\n") + "\n\n## Answer\n\n" + answer.strip("\n") + "\n"
    item.meta["status"] = "complete"
    item.meta["updated"] = _now()
    item.body = body
    write_item(item.path, item.meta, body)
    _commit(repo_root, item, "map: resolve %s" % _label(item), push)
    return item


def _repo(args) -> str:
    return os.path.abspath(args.repo)


def _run_frontier(args) -> int:
    for it in frontier(_repo(args), args.pathway):
        print("%s\t%s\t%s\t%s" % (it.meta.get("rank", ""), it.short_id or "-", it.slug, it.title))
    return 0


def _run_claim(args) -> int:
    try:
        item = claim(_repo(args), args.slug, args.user, push=not args.no_push)
    except TicketError as exc:
        print("mapkit: %s" % exc, file=sys.stderr)
        return 2
    print("%s %s-%s claimed (%s)" % (item.short_id or "-", item.kind, item.slug, item.title))
    return 0


def _run_resolve(args) -> int:
    if args.answer is not None:
        answer = args.answer
    else:
        with open(args.answer_file, encoding="utf-8") as fh:
            answer = fh.read()
    try:
        item = resolve(_repo(args), args.slug, answer, push=not args.no_push)
    except TicketError as exc:
        print("mapkit: %s" % exc, file=sys.stderr)
        return 2
    print("%s %s-%s resolved (%s)" % (item.short_id or "-", item.kind, item.slug, item.title))
    return 0


def register(subparsers) -> None:
    def common(p):
        p.add_argument("--repo", default=".", help="repo root (default .)")

    p = subparsers.add_parser("frontier", help="open, unowned, unblocked tickets")
    p.add_argument("pathway")
    common(p)
    p.set_defaults(func=_run_frontier)

    p = subparsers.add_parser("claim", help="claim a ticket (commit + push)")
    p.add_argument("slug")
    p.add_argument("--as", dest="user", required=True)
    p.add_argument("--no-push", action="store_true")
    common(p)
    p.set_defaults(func=_run_claim)

    p = subparsers.add_parser("resolve", help="resolve a ticket (commit + push)")
    p.add_argument("slug")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--answer")
    g.add_argument("--answer-file")
    p.add_argument("--no-push", action="store_true")
    common(p)
    p.set_defaults(func=_run_resolve)
