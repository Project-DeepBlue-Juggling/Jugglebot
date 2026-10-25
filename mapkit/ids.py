"""Short Item IDs: ``JB-<n>``, one global sequence (plan ``map-item-ids.md``).

The ID lives in frontmatter ``id:`` only, as the first key, written as the
canonical string ``JB-<n>`` (no padding). The filename stem stays the stored
cross-reference; the ID is for people and commands.

* :func:`allocate` -- 1 + max over the working tree's IDs and those on a ref
  (default ``origin/main``, read with ``git grep``; no network).
* :func:`duplicates` / :func:`check` -- the uniqueness, format and
  immutability checks (the loader deliberately does NOT validate ``id``: a
  malformed value must never hide an Item or break ``mapkit gen``).
* :func:`repair` -- renumber the loser of each duplicate. Keeper rule: the
  Item that holds the ID on the ref keeps it; otherwise the earliest by
  (``created``, first-add commit position on the ref, stem).
* :func:`assign_missing` -- number ID-less Items in that same order (the
  backfill). Neither repair nor assign bumps ``updated`` or adds a Progress
  line.
* :func:`resolve` -- stem, then slug, then ID; bare digits only when the
  caller is a read command (``allow_bare``).
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from typing import Dict, Iterable, List, Optional, Tuple

from mapkit.items import Item, ItemError, load_items, write_item

PREFIX = "JB"
DEFAULT_REF = "origin/main"
RENUMBER_SUBJECT = "map: renumber"
MAP_DIR = "map"

_CANON = re.compile(r"^%s-([1-9][0-9]*)$" % PREFIX)
_LOOSE = re.compile(r"^%s-?([0-9]+)$" % PREFIX, re.IGNORECASE)
_BARE = re.compile(r"^[0-9]+$")
_INF = float("inf")


class IdError(LookupError):
    """An unresolvable or ambiguous reference; the message says which."""


def format_id(n: int) -> str:
    if not isinstance(n, int) or isinstance(n, bool) or n < 1:
        raise ValueError("an ID number is a positive int, not %r" % (n,))
    return "%s-%d" % (PREFIX, n)


def canonical(value: object) -> Optional[int]:
    """The number of a canonical ``JB-<n>`` string; None for anything else."""
    if not isinstance(value, str):
        return None
    m = _CANON.match(value)
    return int(m.group(1)) if m else None


def parse(text: str, allow_bare: bool = False) -> Optional[int]:
    """A typed ID (``JB-12``, ``jb12``, ``jb-12``; bare ``12`` if allowed) -> 12."""
    t = text.strip()
    m = _LOOSE.match(t)
    digits = m.group(1) if m else (t if allow_bare and _BARE.match(t) else None)
    if digits is None:
        return None
    n = int(digits)
    return n if n >= 1 else None


def short_id(item: Item) -> Optional[str]:
    v = item.meta.get("id")
    return v if canonical(v) is not None else None


def stem(item: Item) -> str:
    return "%s-%s" % (item.kind, item.slug)


# --- git helpers ----------------------------------------------------------
def _git(repo_root: str, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(["git"] + list(args), cwd=repo_root, capture_output=True, text=True)


def ref_ids(repo_root: str, ref: str = DEFAULT_REF) -> Dict[str, str]:
    """``{stem: id value}`` for the Items on ``ref``; ``{}`` if the ref is absent."""
    r = _git(repo_root, "grep", "-e", "^id: ", ref, "--", MAP_DIR + "/")
    out: Dict[str, str] = {}
    if r.returncode not in (0, 1):
        return out
    for line in r.stdout.splitlines():
        # <ref>:map/<stem>.md:id: JB-3
        try:
            _, rest = line.split(":" + MAP_DIR + "/", 1)
            name, value = rest.split(":id: ", 1)
        except ValueError:
            continue
        if name.endswith(".md") and "/" not in name:
            out[name[:-3]] = value.strip().strip("'\"")
    return out


def add_order(repo_root: str, ref: str = DEFAULT_REF) -> Dict[str, int]:
    """``{map/<file>: n}``: the history position of the commit that first added
    each file on ``ref`` (oldest = 0). One ``git log``; commit order, not commit
    time, so commits made within the same second still order correctly."""
    r = _git(repo_root, "log", "--reverse", "--diff-filter=A", "--name-only",
             "--format=@%H", ref, "--", MAP_DIR + "/")
    out: Dict[str, int] = {}
    n = -1
    if r.returncode != 0:
        return out
    for line in r.stdout.splitlines():
        if line.startswith("@"):
            n += 1
        elif line.strip():
            out.setdefault(line.strip(), n)
    return out


def _order_key(repo_root: str, item: Item, order: Dict[str, int]) -> Tuple[str, float, str]:
    """(``created``, first-add position on the ref, stem); unpublished files last."""
    rel = os.path.relpath(item.path, repo_root).replace(os.sep, "/")
    return (str(item.meta.get("created") or "9999-99-99"),
            float(order.get(rel, _INF)), stem(item))


# --- allocation and checks ------------------------------------------------
def _load(repo_root: str) -> List[Item]:
    return load_items(os.path.join(repo_root, MAP_DIR))


def _max_number(values: Iterable[object]) -> int:
    nums = [canonical(v) for v in values]
    return max([n for n in nums if n is not None] + [0])


def allocate(repo_root: str, ref: str = DEFAULT_REF,
             items: Optional[List[Item]] = None) -> str:
    items = _load(repo_root) if items is None else items
    top = max(_max_number(i.meta.get("id") for i in items),
              _max_number(ref_ids(repo_root, ref).values()))
    return format_id(top + 1)


def duplicates(items: List[Item]) -> Dict[str, List[Item]]:
    by: Dict[str, List[Item]] = {}
    for i in items:
        sid = short_id(i)
        if sid is not None:
            by.setdefault(sid, []).append(i)
    return {k: sorted(v, key=stem) for k, v in by.items() if len(v) > 1}


def missing(items: List[Item]) -> List[Item]:
    return [i for i in items if i.meta.get("id") in (None, "")]


def malformed(items: List[Item]) -> List[Item]:
    return [i for i in items if i.meta.get("id") not in (None, "") and short_id(i) is None]


def changed_ids(repo_root: str, items: List[Item], ref: str = DEFAULT_REF) -> List[Tuple[Item, str]]:
    """Items whose ``id`` differs from the ref's with no renumber commit in ``ref..HEAD``."""
    on_ref = ref_ids(repo_root, ref)
    out = []
    for i in items:
        old = on_ref.get(stem(i))
        if old is None or old == i.meta.get("id"):
            continue
        rel = os.path.relpath(i.path, repo_root)
        r = _git(repo_root, "log", "--format=%s", "%s..HEAD" % ref, "--", rel)
        if not any(s.startswith(RENUMBER_SUBJECT) for s in r.stdout.splitlines()):
            out.append((i, old))
    return out


def check(repo_root: str, ref: str = DEFAULT_REF,
          items: Optional[List[Item]] = None) -> Dict[str, List[str]]:
    """Problems by class; ``missing`` is informational until every writer allocates."""
    items = _load(repo_root) if items is None else items
    return {
        "duplicate": ["%s: %s" % (k, ", ".join(stem(i) for i in v))
                      for k, v in sorted(duplicates(items).items())],
        "malformed": ["%s: id %r is not %s-<n>" % (stem(i), i.meta.get("id"), PREFIX)
                      for i in malformed(items)],
        "changed": ["%s: id %s on %s, %s here, with no '%s' commit" % (
            stem(i), old, ref, i.meta.get("id"), RENUMBER_SUBJECT)
            for i, old in changed_ids(repo_root, items, ref)],
        "missing": [stem(i) for i in missing(items)],
    }


_ID_LINE = re.compile(r"^id:.*\n", re.MULTILINE)  # top-level key only (column 0)


def set_id(item: Item, new_id: str) -> None:
    """Make ``id: <new_id>`` the file's first frontmatter key, as a text edit.

    Nothing else in the file changes, byte for byte: re-dumping the loaded
    meta would re-quote titles and add the loader's defaults (``branch``).
    """
    with open(item.path, encoding="utf-8", newline="") as fh:  # keep CRLF bodies as they are
        text = fh.read()
    if not text.startswith("---\n"):
        raise ItemError("%s: no frontmatter" % item.path)
    end = text.find("\n---", 3)
    head, rest = text[4:end + 1], text[end + 1:]
    head = _ID_LINE.sub("", head, count=1)
    with open(item.path, "w", encoding="utf-8", newline="") as fh:
        fh.write("---\nid: %s\n%s%s" % (new_id, head, rest))
    meta = {"id": new_id}
    meta.update((k, v) for k, v in item.meta.items() if k != "id")
    item.meta = meta


def repair(repo_root: str, ref: str = DEFAULT_REF,
           own: Optional[Iterable[str]] = None) -> List[Tuple[str, str, str]]:
    """Renumber each duplicate's losers; returns ``(old, new, stem)`` rows.

    ``own`` (stems) limits renumbering to those Items (a writer repairs only
    its own unpushed files); a group with no own loser is left for the gate.
    """
    items = _load(repo_root)
    on_ref = ref_ids(repo_root, ref)
    own_set = set(own) if own is not None else None
    order = add_order(repo_root, ref)
    rows = []
    for sid, group in sorted(duplicates(items).items()):
        holders = [i for i in group if on_ref.get(stem(i)) == sid]
        keeper = min(holders or group, key=lambda i: _order_key(repo_root, i, order))
        for loser in group:
            if loser is keeper or (own_set is not None and stem(loser) not in own_set):
                continue
            new = allocate(repo_root, ref, items)
            set_id(loser, new)
            rows.append((sid, new, stem(loser)))
    return rows


def assign_missing(repo_root: str, ref: str = DEFAULT_REF) -> List[Tuple[str, str]]:
    """Give every ID-less Item the next ID, oldest first; returns ``(id, stem)``."""
    items = _load(repo_root)
    order = add_order(repo_root, ref)
    todo = sorted(missing(items), key=lambda i: _order_key(repo_root, i, order))
    rows = []
    for item in todo:
        new = allocate(repo_root, ref, items)
        set_id(item, new)
        rows.append((new, stem(item)))
    return rows


def resolve(items: List[Item], text: str, allow_bare: bool = False) -> Item:
    """Exact stem, then slug, then ID; an error when a slug and an ID disagree."""
    t = text.strip()
    if t.endswith(".md"):
        t = t[:-3]
    for i in items:
        if stem(i) == t:
            return i
    by_slug = [i for i in items if i.slug == t]
    if len(by_slug) > 1:
        raise IdError("slug %r is ambiguous (%s); use the full stem"
                      % (t, ", ".join(stem(i) for i in by_slug)))
    n = parse(t, allow_bare=allow_bare)
    by_id = [i for i in items if n is not None and canonical(i.meta.get("id")) == n]
    if len(by_id) > 1:
        raise IdError("%s names %d Items (%s); run python -m mapkit ids --repair"
                      % (format_id(n), len(by_id), ", ".join(stem(i) for i in by_id)))
    if by_slug and by_id and by_slug[0] is not by_id[0]:
        raise IdError("%r is both the slug of %s and the ID of %s; use the full stem"
                      % (t, stem(by_slug[0]), stem(by_id[0])))
    if by_slug:
        return by_slug[0]
    if by_id:
        return by_id[0]
    hint = "" if allow_bare or not _BARE.match(t) else " (write commands need %s-<n>)" % PREFIX
    raise IdError("no Item %r%s" % (text, hint))


# --- CLI ------------------------------------------------------------------
def _run(args: argparse.Namespace) -> int:
    root = os.path.abspath(args.repo)
    try:
        if args.assign_missing:
            for sid, st in assign_missing(root, args.ref):
                print("%s %s" % (sid, st))
            return 0
        if args.repair:
            for old, new, st in repair(root, args.ref):
                print("%s -> %s %s" % (old, new, st))
            return 0
        problems = check(root, args.ref)
    except ItemError as exc:
        print("mapkit: %s" % exc, file=sys.stderr)
        return 2
    bad = 0
    for cls in ("duplicate", "malformed", "changed"):
        for line in problems[cls]:
            print("%s: %s" % (cls, line))
            bad += 1
    if problems["missing"]:
        print("missing: %d Items have no id (python -m mapkit ids --assign-missing)"
              % len(problems["missing"]))
    return 1 if bad else 0


def register(subparsers) -> None:
    p = subparsers.add_parser("ids", help="Item IDs: check, backfill, repair duplicates")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--check", action="store_true", help="report problems (default)")
    g.add_argument("--assign-missing", action="store_true", help="number ID-less Items, oldest first")
    g.add_argument("--repair", action="store_true", help="renumber the loser of each duplicate")
    p.add_argument("--ref", default=DEFAULT_REF, help="published ref (default origin/main)")
    p.add_argument("--repo", default=".", help="repo root (default .)")
    p.set_defaults(func=_run)
