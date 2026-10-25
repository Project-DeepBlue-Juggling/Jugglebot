"""Item commands: ``item set-status``, ``show`` and ``items`` (P9a).

``set-status`` encodes the shared map-write recipe: validate against the
kind's status set, refuse to re-open a complete item or to start an item
another owner holds (``--force`` overrides both), append a dated line under
``## Progress``, add links without duplicates, write through ``write_item``
and regenerate ``docs/project-map.md``. Without ``--commit`` nothing is
staged and the two paths to stage are printed, so a fix's commit can carry
the close (hybrid rule: start/progress = own ``map:`` commit, close = the
fix's commit with its Logbook-Entry trailer). ``show`` and ``items`` are the
read side.
"""
from __future__ import annotations

import datetime
import json
import os
import subprocess
import sys
from typing import Dict, List, Optional

from mapkit import derive, gen_docs
from mapkit.items import (KINDS, STATUS_SETS, Item, ItemError, is_stale, load_item,
                          load_items, stale_clock, write_item)
from mapkit.plans import load_plans

MAP_DIR = "map"
DOCS_PAGE = os.path.join("docs", "project-map.md")
_CLOSED = ("complete", "dropped", "promoted")


class ItemCliError(Exception):
    """A refused operation; exit code 2."""


def _now() -> str:
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def _today() -> str:
    return datetime.date.today().isoformat()


def _load(path: str) -> Item:
    try:
        return load_item(path)
    except ItemError as exc:
        raise ItemCliError(str(exc))


def find_item(repo_root: str, slug: str) -> Item:
    """Resolve a filename stem (``bug-x``) or a bare slug (``x``)."""
    map_dir = os.path.join(repo_root, MAP_DIR)
    stem = slug[:-3] if slug.endswith(".md") else slug
    path = os.path.join(map_dir, stem + ".md")
    if os.path.isfile(path):
        return _load(path)
    hits = [os.path.join(map_dir, "%s-%s.md" % (k, stem)) for k in KINDS]
    hits = [h for h in hits if os.path.isfile(h)]
    if not hits:
        raise ItemCliError("no item %r in %s" % (slug, map_dir))
    if len(hits) > 1:
        raise ItemCliError("slug %r is ambiguous (%s); use the full stem" % (
            slug, ", ".join(os.path.basename(h)[:-3] for h in hits)))
    return _load(hits[0])


def _git(repo_root: str, *args: str, check: bool = True) -> int:
    return subprocess.run(["git"] + list(args), cwd=repo_root, check=check).returncode


def _append_progress(body: str, line: str) -> str:
    lines = body.rstrip("\n").split("\n") if body.strip() else []
    idx = next((i for i, l in enumerate(lines) if l.strip() == "## Progress"), None)
    if idx is None:
        return "\n".join(lines + ([""] if lines else []) + ["## Progress", "", line]) + "\n"
    end = idx + 1
    while end < len(lines) and not lines[end].startswith("## "):
        end += 1
    ins = end
    while ins > idx + 1 and not lines[ins - 1].strip():
        ins -= 1
    if ins == idx + 1:  # empty section: keep a blank line after the heading
        new = lines[:idx + 1] + ["", line] + lines[idx + 1:]
    else:
        new = lines[:ins] + [line] + lines[ins:]
    return "\n".join(new).rstrip("\n") + "\n"


def set_status(repo_root: str, slug: str, status: str, note: Optional[str] = None,
               links: Optional[List[str]] = None, owner: Optional[str] = None,
               force: bool = False, commit: bool = False, push: bool = True) -> Item:
    if commit and push:
        # the app writes map/ concurrently: read the current tree, not a stale one
        if _git(repo_root, "pull", "--ff-only", check=False) != 0:
            print("mapkit: warning: git pull --ff-only failed; continuing on the local tree",
                  file=sys.stderr)
    item = find_item(repo_root, slug)
    if status not in STATUS_SETS[item.kind]:
        raise ItemCliError("status %r not valid for a %s (one of %s)" % (
            status, item.kind, ", ".join(sorted(STATUS_SETS[item.kind]))))
    if item.status == "complete" and status != "complete" and not force:
        raise ItemCliError("%s is complete; refusing to re-open it (use --force)" % item.slug)
    owners = [str(o) for o in (item.meta.get("owners") or [])]
    if status == "in_progress":
        others = [o for o in owners if o != owner]
        if others and not force:
            raise ItemCliError("%s is held by %s; refusing in_progress (use --force)" % (
                item.slug, ", ".join(others)))
    if owner and owner not in owners:
        owners.append(owner)
        item.meta["owners"] = owners
    if links:
        cur = list(item.meta.get("links") or [])
        for l in links:
            if l not in cur:
                cur.append(l)
        item.meta["links"] = cur
    item.meta["status"] = status
    item.meta["updated"] = _now()
    body = item.body
    if note:
        body = _append_progress(body, "- %s: %s" % (_today(), " ".join(note.split())))
    item.body = body
    item.changed = write_item(item.path, item.meta, body)
    if not item.changed:
        return item  # no-op: nothing written, regenerated or committed
    gen_docs.write_docs(repo_root)
    if commit:
        rel = os.path.relpath(item.path, repo_root)
        paths = [rel] + ([DOCS_PAGE] if os.path.isfile(os.path.join(repo_root, DOCS_PAGE)) else [])
        _git(repo_root, "add", "--", *paths)
        _git(repo_root, "commit", "-m", "map: %s %s" % (status, item.title), "--", *paths)
        if push:
            _git(repo_root, "pull", "--rebase", "--autostash")
            _git(repo_root, "push")
    return item


def _age_days(item: Item) -> int:
    try:
        d = datetime.date.fromisoformat(str(item.meta.get("created"))[:10])
    except ValueError:
        return 0
    return max(0, (datetime.date.today() - d).days)


def _row(i: Item) -> Dict[str, object]:
    return {"kind": i.kind, "slug": i.slug, "status": i.status,
            "priority": i.meta.get("priority") or "",
            "pathway": i.slug if i.kind == "pathway" else (i.meta.get("parent") or ""),
            "age_days": _age_days(i), "title": i.title}


def list_items(repo_root: str, pathway: Optional[str] = None, kind: Optional[str] = None,
               status: Optional[str] = None, open_only: bool = False,
               stale: bool = False, today: Optional[datetime.date] = None) -> List[Item]:
    today = today or datetime.date.today()
    out = []
    for i in load_items(os.path.join(repo_root, MAP_DIR)):
        if kind and i.kind != kind:
            continue
        if pathway and not (i.meta.get("parent") == pathway
                            or (i.kind == "pathway" and i.slug == pathway)):
            continue
        if status and i.status != status:
            continue
        if open_only and i.status in _CLOSED:
            continue
        if stale and not is_stale(i, today):
            continue
        out.append(i)
    return out


def show(repo_root: str, pathway: str) -> Dict[str, object]:
    items = load_items(os.path.join(repo_root, MAP_DIR))
    pw = next((i for i in items if i.kind == "pathway" and i.slug == pathway), None)
    if pw is None:
        raise ItemCliError("no pathway %r" % pathway)
    plan_rows = []
    for p in derive.pathway_plans(pw, load_plans(repo_root)):
        if p.board not in ("active", "parked"):
            continue
        plan_rows.append({"filename": p.filename, "board": p.board,
                          "status": p.meta.get("status"),
                          "steps": [{"id": s.id, "title": s.title,
                                     "status": s.status or "not_started"} for s in p.steps]})
    children: Dict[str, list] = {}
    for i in items:
        if i.meta.get("parent") != pathway or i.kind == "pathway":
            continue
        open_ = (i.status == "open") if i.kind == "ticket" else i.status in derive.OPEN_INBOX_STATUSES
        if open_:
            children.setdefault(i.kind, []).append(_row(i))
    return {"slug": pw.slug, "title": pw.title, "status": pw.status,
            "destination": " ".join(str(pw.meta["destination"]).split()),
            "plans": plan_rows, "children": children}


def _render_show(d: Dict[str, object]) -> str:
    out = ["%s [%s] (%s)" % (d["title"], d["status"], d["slug"]),
           "Destination: %s" % d["destination"], ""]
    out.append("Active plans:" if d["plans"] else "Active plans: none")
    for p in d["plans"]:
        out.append("  %s (%s, %s)" % (p["filename"], p["board"], p["status"]))
        for s in p["steps"]:
            out.append("    %-6s %-12s %s" % (s["id"], s["status"], s["title"]))
    for kind, label in (("bug", "Bugs"), ("feature", "Features"),
                        ("idea", "Ideas"), ("ticket", "Tickets")):
        rows = d["children"].get(kind, [])
        out += ["", "%s (%d open)" % (label, len(rows))]
        for r in rows:
            out.append("  %-50s %-8s %s" % (r["slug"], r["priority"] or "-", r["status"]))
    return "\n".join(out)


def _repo(args) -> str:
    return os.path.abspath(args.repo)


def _run_set_status(args) -> int:
    try:
        item = set_status(_repo(args), args.slug, args.status, note=args.note,
                          links=args.link, owner=args.owner, force=args.force,
                          commit=args.commit, push=not args.no_push)
    except ItemCliError as exc:
        print("mapkit: %s" % exc, file=sys.stderr)
        return 2
    if not getattr(item, "changed", True):
        print("%s-%s unchanged (%s)" % (item.kind, item.slug, item.status))
        return 0
    print("%s-%s -> %s" % (item.kind, item.slug, item.status))
    if not args.commit:
        print("stage: %s" % os.path.relpath(item.path, _repo(args)))
        print("stage: %s" % DOCS_PAGE)
    return 0


def _run_show(args) -> int:
    try:
        d = show(_repo(args), args.pathway)
    except (ItemCliError, ItemError) as exc:
        print("mapkit: %s" % exc, file=sys.stderr)
        return 2
    print(json.dumps(d, indent=2) if args.json else _render_show(d))
    return 0


def _run_items(args) -> int:
    stale = bool(getattr(args, "stale", False))
    try:
        rows = [_row(i) for i in list_items(_repo(args), args.pathway, args.kind,
                                            args.status, args.open, stale)]
    except ItemError as exc:
        print("mapkit: %s" % exc, file=sys.stderr)
        return 2
    if stale:
        today = datetime.date.today()
        for r, i in zip(rows, list_items(_repo(args), args.pathway, args.kind,
                                         args.status, args.open, True)):
            clock = stale_clock(i)
            r["stale_days"] = (today - clock).days
            r["stale_clock"] = clock.isoformat()
    if args.json:
        print(json.dumps(rows, indent=2))
    else:
        for r in rows:
            line = "%-8s %-50s %-12s %-6s %-30s %dd" % (
                r["kind"], r["slug"], r["status"], r["priority"] or "-",
                r["pathway"] or "-", r["age_days"])
            if stale:
                line += "  stale %dd since %s" % (r["stale_days"], r["stale_clock"])
            print(line)
    return 0


def register(subparsers) -> None:
    def common(p):
        p.add_argument("--repo", default=".", help="repo root (default .)")

    item = subparsers.add_parser("item", help="item operations")
    isub = item.add_subparsers(dest="item_command")
    p = isub.add_parser("set-status", help="move an item to a status (optionally commit)")
    p.add_argument("slug")
    p.add_argument("status")
    p.add_argument("--note")
    p.add_argument("--link", action="append", default=[])
    p.add_argument("--owner")
    p.add_argument("--force", action="store_true")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--commit", dest="commit", action="store_true")
    g.add_argument("--no-commit", dest="commit", action="store_false")
    p.set_defaults(commit=False)
    p.add_argument("--no-push", action="store_true")
    common(p)
    p.set_defaults(func=_run_set_status)

    p = subparsers.add_parser("show", help="a pathway: destination, plans and steps, open children")
    p.add_argument("pathway")
    p.add_argument("--json", action="store_true")
    common(p)
    p.set_defaults(func=_run_show)

    p = subparsers.add_parser("items", help="one line per item, filtered")
    p.add_argument("--pathway")
    p.add_argument("--kind", choices=KINDS)
    g = p.add_mutually_exclusive_group()
    g.add_argument("--status")
    g.add_argument("--open", action="store_true")
    p.add_argument("--stale", action="store_true",
                   help="only stale items (clock = newest dated Progress line, else created; "
                        "inbox 30 d, backlog 60 d, in_progress 14 d); adds age and clock date")
    p.add_argument("--json", action="store_true")
    common(p)
    p.set_defaults(func=_run_items)
