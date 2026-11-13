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

from mapkit import derive, gen_docs, ids, similar
from mapkit.items import (KINDS, NIGHT_SKIP_FIELDS, NIGHT_SKIP_REASONS, METRIC_ENUMS, METRIC_FIELDS, STATUS_SETS, Item, ItemError, is_stale, has_answer,
                          load_item, load_items, stale_clock, write_item)
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


def find_item(repo_root: str, slug: str, allow_bare: bool = False) -> Item:
    """Resolve a stem (``bug-x``), a bare slug (``x``) or an ID (``BUG-12``, ``bug12``).

    One resolver for every command (``mapkit.ids.resolve``: stem, then slug,
    then ID). Bare digits (``12``) are an ID only when ``allow_bare`` (read
    commands); a write needs the prefix or a stem. The exact-stem path
    is tried first and loads one file, so a malformed unrelated Item never
    blocks a write by stem.
    """
    map_dir = os.path.join(repo_root, MAP_DIR)
    stem = slug[:-3] if slug.endswith(".md") else slug
    path = os.path.join(map_dir, stem + ".md")
    if os.path.isfile(path):
        return _load(path)
    try:
        items = load_items(map_dir)
    except ItemError as exc:
        raise ItemCliError(str(exc))
    legacy = _legacy(repo_root)
    try:
        item = ids.resolve(items, stem, allow_bare=allow_bare, legacy=legacy)
    except ids.LegacyIdError as exc:
        raise ItemCliError(str(exc))
    except ids.IdError as exc:
        raise ItemCliError("%s in %s" % (exc, map_dir))
    _note_translation(stem, item, legacy)
    return item


def _legacy(repo_root: str) -> Dict[str, str]:
    """The crosswalk (``ids.legacy``); a malformed one is this command's error, named by
    file and line, never a traceback (P4 audit B3)."""
    try:
        return ids.legacy(repo_root)
    except ItemError as exc:
        raise ItemCliError(str(exc))


def _note_translation(text: str, item: Item, legacy: Dict[str, str]) -> None:
    """D-3: a read that took an old ID says what it is called now (stderr)."""
    new = ids.legacy_ref(text, legacy)
    if new is not None and new == item.meta.get("id"):
        print("%s -> %s" % (text.strip(), new), file=sys.stderr)


def _typed_id(text: str, legacy: Dict[str, str]) -> bool:
    """Is ``text`` an ID, current or old? An old one must reach :func:`ids.resolve`,
    which refuses it on a write, never be stored as if it were a stem."""
    return ids.parse_ref(text) is not None or ids.legacy_ref(text, legacy) is not None


def label(item: Item) -> str:
    """``BUG-12 bug-x`` (``- bug-x`` when ID-less): every write's echo starts with it."""
    return "%s %s-%s" % (item.short_id or "-", item.kind, item.slug)


def _subject(verb: str, item: Item) -> str:
    """``map: <verb> BUG-12 <title>``; the ID is omitted while an Item has none."""
    sid = item.short_id
    return "map: %s %s%s" % (verb, (sid + " ") if sid else "", item.title)


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
    if (status == "complete" and item.kind == "ticket" and item.meta.get("type") == "question"
            and not has_answer(item.body)):
        # the app writes body and status in one save, so order is unknown: warn, do not refuse
        print("mapkit: warning: %s completed without an ## Answer" % (item.short_id or item.slug),
              file=sys.stderr)
    item.meta["status"] = status
    if status in ("complete", "dropped"):  # a closed Item is never a night candidate
        for f in NIGHT_SKIP_FIELDS:
            item.meta.pop(f, None)
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
        _git(repo_root, "commit", "-m", _subject(status, item), "--", *paths)
        if push:
            _git(repo_root, "pull", "--rebase", "--autostash")
            _git(repo_root, "push")
    return item


def _check_metrics(kind: str, metrics: Dict[str, Optional[str]]) -> Dict[str, str]:
    """The given (non-None) metrics, validated against the enums and the bug-only rule."""
    out: Dict[str, str] = {}
    for f, v in metrics.items():
        if v is None:
            continue
        if f not in METRIC_ENUMS:
            raise ItemCliError("unknown metric %r (one of %s)" % (f, ", ".join(METRIC_FIELDS)))
        if v not in METRIC_ENUMS[f]:
            raise ItemCliError("%s %r not in %s" % (f, v, ", ".join(METRIC_ENUMS[f])))
        if f == "diagnosis" and kind != "bug":
            raise ItemCliError("diagnosis is for bugs only (this is a %s)" % kind)
        out[f] = v
    return out


def _place_metrics(meta: Dict[str, object], metrics: Dict[str, str]) -> Dict[str, object]:
    """``meta`` with ``metrics`` set, the six fields in canonical order right after ``priority``."""
    rest = {k: v for k, v in meta.items() if k not in METRIC_FIELDS}
    cur = {f: (metrics[f] if f in metrics else meta[f]) for f in METRIC_FIELDS
           if f in metrics or meta.get(f) is not None}
    out: Dict[str, object] = {}
    placed = False
    for k, v in rest.items():
        out[k] = v
        if k == "priority":
            out.update(cur)
            placed = True
    if not placed:
        out.update(cur)
    return out


def rate_item(repo_root: str, slug: str, metrics: Dict[str, Optional[str]],
              commit: bool = False, push: bool = True) -> Item:
    """Set item metrics (effort/hardware/owner/radius/diagnosis/area) on an existing Item."""
    item = find_item(repo_root, slug)
    given = _check_metrics(item.kind, metrics)
    if not given:
        raise ItemCliError("nothing to set: pass at least one of %s" % ", ".join(
            "--" + f for f in METRIC_FIELDS))
    meta = _place_metrics(item.meta, given)
    meta["updated"] = _now()
    item.meta = meta
    item.changed = write_item(item.path, meta, item.body)
    if not item.changed:
        return item
    gen_docs.write_docs(repo_root)
    if commit:
        rel = os.path.relpath(item.path, repo_root)
        paths = [rel] + ([DOCS_PAGE] if os.path.isfile(os.path.join(repo_root, DOCS_PAGE)) else [])
        _git(repo_root, "add", "--", *paths)
        _git(repo_root, "commit", "-m", _subject("rate", item), "--", *paths)
        if push:
            _git(repo_root, "pull", "--rebase", "--autostash")
            _git(repo_root, "push")
    return item


def _apply_skip(meta: Dict[str, object], reason: Optional[str], note: Optional[str],
                input_: Optional[str], on: str) -> Dict[str, object]:
    """``meta`` with the four skip keys set (``reason`` None clears them), placed last.

    ``updated`` is deliberately NOT touched: the Night Shift's delta walk (C4) treats an
    Item whose ``updated`` or ``status`` moved as changed and walks it, and a skip written
    by the night itself must not make the Item look changed the next night.
    """
    out = {k: v for k, v in meta.items() if k not in NIGHT_SKIP_FIELDS}
    if reason is not None:
        out["night_skip"] = reason
        out["night_skip_on"] = on
        out["night_skip_note"] = " ".join((note or "").split())
        if input_ is not None:
            out["night_skip_input"] = " ".join(input_.split())
    return out


def _skip_write(item: Item, meta: Dict[str, object]) -> None:
    """Validate ``meta`` through the one enforcement point, then write (no ``updated`` bump)."""
    from mapkit.items import _night_skip_errors
    errs = _night_skip_errors(item.kind, meta)
    if errs:
        raise ItemCliError("%s: %s" % (item.slug, "; ".join(errs)))
    item.meta = meta
    item.changed = write_item(item.path, meta, item.body)


def skip_item(repo_root: str, slug: str, reason: Optional[str] = None, note: Optional[str] = None,
              input_: Optional[str] = None, clear: bool = False, on: Optional[str] = None,
              commit: bool = False, push: bool = True) -> Item:
    """Set (or ``clear``) one Item's Night Shift skip reason."""
    if clear == (reason is not None):
        raise ItemCliError("pass exactly one of --reason (with --note) or --clear")
    if not clear and reason not in NIGHT_SKIP_REASONS:
        raise ItemCliError("reason %r not in %s" % (reason, ", ".join(NIGHT_SKIP_REASONS)))
    item = find_item(repo_root, slug)
    _skip_write(item, _apply_skip(item.meta, None if clear else reason, note, input_,
                                  on or _today()))
    if item.changed:
        gen_docs.write_docs(repo_root)
        if commit:
            _commit_paths(repo_root, [item], "map: %s %s%s" % (
                "clear night skip" if clear else "night skip", (item.short_id + " ") if item.short_id else "",
                item.title), push)
    return item


def skip_batch(repo_root: str, rows: List[Dict[str, object]], on: Optional[str] = None,
               commit: bool = False, push: bool = True) -> List[Item]:
    """Apply a batch of skip rows atomically: validate every row first, then write all.

    Each row: ``{"item": ID|stem, "reason": R, "note": N, "input": I?}``, or
    ``{"item": ..., "clear": true}``. Returns the Items that changed.
    """
    on = on or _today()
    plan = []
    errs: List[str] = []
    seen = set()
    for n, row in enumerate(rows):
        try:
            if not isinstance(row, dict) or not row.get("item"):
                raise ItemCliError("row %d: needs an 'item'" % n)
            extra = set(row) - {"item", "reason", "note", "input", "clear"}
            if extra:
                raise ItemCliError("row %d: unknown keys %s" % (n, sorted(extra)))
            item = find_item(repo_root, str(row["item"]))
            if item.path in seen:
                raise ItemCliError("row %d: %s listed twice" % (n, item.slug))
            seen.add(item.path)
            clear = bool(row.get("clear"))
            if clear == (row.get("reason") is not None):
                raise ItemCliError("row %d: exactly one of 'reason' or 'clear'" % n)
            if not clear and row["reason"] not in NIGHT_SKIP_REASONS:
                raise ItemCliError("row %d: reason %r not in %s" % (
                    n, row["reason"], ", ".join(NIGHT_SKIP_REASONS)))
            meta = _apply_skip(item.meta, None if clear else str(row["reason"]),
                               row.get("note"), row.get("input"), on)  # type: ignore[arg-type]
            from mapkit.items import _night_skip_errors
            e = _night_skip_errors(item.kind, meta)
            if e:
                raise ItemCliError("row %d (%s): %s" % (n, item.slug, "; ".join(e)))
            plan.append((item, meta))
        except (ItemCliError, ItemError, ids.IdError) as exc:
            errs.append(str(exc))
    if errs:
        raise ItemCliError("batch refused, nothing written:\n  " + "\n  ".join(errs))
    changed = []
    for item, meta in plan:
        item.meta = meta
        item.changed = write_item(item.path, meta, item.body)
        if item.changed:
            changed.append(item)
    if changed:
        gen_docs.write_docs(repo_root)
        if commit:
            _commit_paths(repo_root, changed, "map: night skip reasons %s (%d Items)" % (
                on, len(changed)), push)
    return changed


def _commit_paths(repo_root: str, items: List[Item], subject: str, push: bool) -> None:
    paths = [os.path.relpath(i.path, repo_root) for i in items]
    if os.path.isfile(os.path.join(repo_root, DOCS_PAGE)):
        paths.append(DOCS_PAGE)
    _git(repo_root, "add", "--", *paths)
    _git(repo_root, "commit", "-m", subject, "--", *paths)
    if push:
        _git(repo_root, "pull", "--rebase", "--autostash")
        _git(repo_root, "push")


PUSH_ROUNDS = 3
_DEFAULT_STATUS = {"pathway": "active", "ticket": "open"}  # inbox kinds: "inbox"


def _git_out(repo_root: str, *args: str) -> "subprocess.CompletedProcess":
    return subprocess.run(["git"] + list(args), cwd=repo_root, capture_output=True, text=True)


def _unique_slug(map_dir: str, base: str) -> str:
    """``base``, else ``base-2``... free across EVERY kind (the app's uuid is
    keyed on the slug alone, so ``bug-x`` beside ``idea-x`` collides there)."""
    slug, n = base, 2
    while any(os.path.exists(os.path.join(map_dir, "%s-%s.md" % (k, slug))) for k in KINDS):
        slug, n = "%s-%d" % (base, n), n + 1
    return slug


def _ref_or_stem(items: List[Item], text: str, legacy: Optional[Dict[str, str]] = None) -> str:
    """A typed reference (``BUG-3`` or a stem/slug) -> the stem stored on disk.

    A write: an old ID the crosswalk knows is refused, naming the new ID.
    """
    try:
        return ids.stem(ids.resolve(items, text, legacy=legacy))
    except ids.IdError as exc:
        raise ItemCliError(str(exc))


def new_item(repo_root: str, kind: str, title: str, parent: Optional[str] = None,
             priority: str = "medium", filed_by: str = "claude", body: str = "",
             links: Optional[List[str]] = None, ticket_type: Optional[str] = None,
             destination: Optional[str] = None, commit: bool = False,
             push: bool = True, metrics: Optional[Dict[str, Optional[str]]] = None,
             allow_similar: bool = False, **metric_kw: Optional[str]) -> Item:
    """File one Item with the next ID: the one Claude-side creation path.

    Refused before anything is written when the title duplicates an open Item,
    or is near another Item (:mod:`mapkit.similar`) and ``allow_similar`` is
    not set; the message lists the matches.

    With ``commit`` and ``push``: pull ``--ff-only``, write, gen, commit
    ``map: file <kind> <ID> <title>``, then up to :data:`PUSH_ROUNDS` rounds of
    ``pull --rebase --autostash`` -> repair this Item's ID if a rebase brought
    in the same one (``map: renumber`` commit) -> push. A rejected push goes
    round again; git refuses non-fast-forward pushes, so the check before the
    push that lands always saw the final remote.
    """
    from mapkit.import_issues import slugify  # local: keeps the CLI import light
    if kind not in KINDS:
        raise ItemCliError("kind %r not in %s" % (kind, ", ".join(KINDS)))
    given_metrics = _check_metrics(kind, dict(metrics or {}, **metric_kw))  # before any pull or write
    if commit and push and _git(repo_root, "pull", "--ff-only", check=False) != 0:
        print("mapkit: warning: git pull --ff-only failed; continuing on the local tree",
              file=sys.stderr)
    map_dir = os.path.join(repo_root, MAP_DIR)
    items = ids.load_lenient(repo_root)  # one malformed sibling must not block filing
    dup, near = similar.screen(items, title)
    if dup is not None:
        raise ItemCliError("duplicate of %s: not filed (an open Item has this title)"
                           % dup.describe())
    if near and not allow_similar:
        raise ItemCliError("similar Items exist, not filed (check them; --allow-similar files anyway):\n  "
                           + "\n  ".join(m.describe() for m in near))
    legacy = _legacy(repo_root)
    upstream = ids.upstream_ref(repo_root)  # the ref the push loop rebases onto
    meta: Dict[str, object] = {
        "id": ids.allocate(repo_root, upstream, items=items, kind=kind), "kind": kind, "title": title,
        "status": _DEFAULT_STATUS.get(kind, "inbox"), "created": _today(), "updated": _now(),
        "owners": [], "priority": priority, "blocked_by": [], "links": [],
    }
    meta = _place_metrics(meta, given_metrics)
    if parent and kind == "ticket" and ticket_type == "question":
        try:
            blocked = ids.resolve(items, parent, legacy=legacy)
        except ids.LegacyIdError as exc:
            raise ItemCliError(str(exc))
        except ids.IdError:
            raise ItemCliError("parent %r not found" % parent)
        if blocked.kind in ("ticket", "pathway"):
            raise ItemCliError("a question hangs off a bug, feature or idea (parent %r is a %s)"
                               % (parent, blocked.kind))
        meta["parent"] = ids.stem(blocked)
    elif parent:
        pw = next((i for i in items if i.kind == "pathway" and i.slug == parent), None)
        if pw is None:
            pw = ids.resolve(items, parent, legacy=legacy)  # a write: prefixed ID or stem, never bare digits
        if pw.kind != "pathway":
            raise ItemCliError("parent %r is a %s, not a pathway" % (parent, pw.kind))
        meta["parent"] = pw.slug
    for l in links or []:
        ref = _ref_or_stem(items, l, legacy) if _typed_id(l, legacy) else l
        if ref not in meta["links"]:
            meta["links"].append(ref)
    if kind in ("idea", "feature", "bug"):
        meta["filed_by"] = filed_by
    if kind == "ticket":
        meta["type"] = ticket_type or "task"
    if kind == "pathway":
        meta["destination"] = destination or ""
    slug = _unique_slug(map_dir, slugify(title)[:60].strip("-") or kind)
    path = os.path.join(map_dir, "%s-%s.md" % (kind, slug))
    write_item(path, meta, (body.strip() + "\n") if body.strip() else "")
    try:
        item = _load(path)  # validates required fields (a pathway without destination fails here)
    except ItemCliError:
        os.remove(path)  # an invalid file in map/ would break every later load_items
        raise
    try:
        gen_docs.write_docs(repo_root)
    except ItemError as exc:  # another Item in map/ is malformed: the page cannot be built
        os.remove(path)
        raise ItemCliError("cannot regenerate %s, so nothing was filed: %s" % (DOCS_PAGE, exc))
    if not commit:
        return item
    rel = os.path.relpath(path, repo_root)
    paths = [rel] + ([DOCS_PAGE] if os.path.isfile(os.path.join(repo_root, DOCS_PAGE)) else [])
    _git(repo_root, "add", "--", *paths)
    _git(repo_root, "commit", "-q", "-m", _subject("file " + kind, item), "--", *paths)
    stem = "%s-%s" % (kind, slug)
    if not push:
        return item
    return push_item(repo_root, stem)


def _waiting_note(repo_root: str, stem: str) -> str:
    """Appended to any failure after the Item's commit exists."""
    head = _git_out(repo_root, "log", "-1", "--oneline").stdout.strip()
    return ("\nA local commit for %s is waiting, NOT pushed (HEAD: %s). Push it with "
            "python -m mapkit item push %s, or undo it with git reset --soft HEAD~1 "
            "(only while it is still the top commit; check git log -1; --soft keeps "
            "your other edits), then delete map/%s.md and revert the %s change "
            "if you do not want them." % (stem, head, stem, stem, DOCS_PAGE))


def _head_adds(repo_root: str, rel: str, upstream: str) -> bool:
    """True when HEAD is the commit that adds ``rel`` AND is not yet on ``upstream``.

    Amending is safe only for an unpublished commit: re-running ``item push``
    after it succeeded must not rewrite a commit the remote already has.
    """
    if _git_out(repo_root, "merge-base", "--is-ancestor", "HEAD", upstream).returncode == 0:
        return False
    out = _git_out(repo_root, "diff-tree", "--no-commit-id", "--name-status", "-r", "HEAD").stdout
    return any(l.split("\t")[:1] == ["A"] and l.split("\t")[-1] == rel for l in out.splitlines())


def push_item(repo_root: str, stem: str) -> Item:
    """Land an Item's local commit: up to :data:`PUSH_ROUNDS` rounds of
    ``pull --rebase --autostash`` -> regenerate the docs page -> repair this
    Item's ID if a rebase brought in the same one -> push.

    A renumber is folded into the Item's own unpushed commit (amended, subject
    rewritten) when that commit is HEAD, so no commit subject on the pushed
    branch cites a wrong ID; otherwise it is a separate ``map: renumber``
    commit. A rejected push goes round again; git refuses non-fast-forward
    pushes, so the check before the push that lands always saw the final
    remote. Every failure names the waiting local commit.
    """
    item0 = find_item(repo_root, stem)  # ID or stem; refuses BEFORE any git call
    stem = "%s-%s" % (item0.kind, item0.slug)
    path = item0.path
    rel = os.path.relpath(path, repo_root)
    paths = [rel] + ([DOCS_PAGE] if os.path.isfile(os.path.join(repo_root, DOCS_PAGE)) else [])
    try:
        for _ in range(PUSH_ROUNDS):
            _rebase_or_abort(repo_root, stem)
            upstream = ids.upstream_ref(repo_root)
            # a textual merge of two generated tables can differ from a fresh gen
            gen_docs.write_docs(repo_root)
            if DOCS_PAGE in paths and _git_out(repo_root, "diff", "--quiet", "--", DOCS_PAGE).returncode:
                if _head_adds(repo_root, rel, upstream):
                    _git(repo_root, "commit", "-q", "--amend", "--no-edit", "--", DOCS_PAGE)
                else:
                    _git(repo_root, "commit", "-q", "-m", "map: regenerate project-map page",
                         "--", DOCS_PAGE)
            for old, new, _st in ids.repair(repo_root, upstream, own=[stem]):
                item = _load(path)
                gen_docs.write_docs(repo_root)
                _git(repo_root, "add", "--", *paths)
                if _head_adds(repo_root, rel, upstream):
                    _git(repo_root, "commit", "-q", "--amend", "-m",
                         _subject("file " + item.kind, item), "--", *paths)
                else:
                    _git(repo_root, "commit", "-q", "-m",
                         "map: renumber %s -> %s %s" % (old, new, item.title), "--", *paths)
                print("mapkit: %s collided after the rebase; renumbered %s -> %s" % (stem, old, new),
                      file=sys.stderr)
            if _git_out(repo_root, "push").returncode == 0:
                return _load(path)
        local = _git_out(repo_root, "log", "--oneline", "@{u}..HEAD").stdout.strip()
        raise ItemCliError("push rejected %d times; local commits ahead:\n%s\n(the Item is %s; "
                           "re-run the push by hand after python -m mapkit ids --check)"
                           % (PUSH_ROUNDS, local, stem))
    except (ItemCliError, ItemError, ids.IdError, subprocess.CalledProcessError, OSError) as exc:
        if isinstance(exc, subprocess.CalledProcessError):
            msg = "git %s failed (exit %d)" % (" ".join(exc.cmd[1:3]), exc.returncode)
        else:
            msg = str(exc)
        raise ItemCliError(msg + _waiting_note(repo_root, stem))


def _rebase_or_abort(repo_root: str, stem: str) -> None:
    """``pull --rebase --autostash``; on failure never leave the repo mid-rebase.

    A conflict (two writers chose the same slug, say) is aborted, which
    restores the local commit and re-applies any autostash; the error names
    the unmerged paths, the local commit and a still-pending autostash.
    """
    r = _git_out(repo_root, "pull", "--rebase", "--autostash")
    if r.returncode == 0:
        return
    unmerged = _git_out(repo_root, "diff", "--name-only", "--diff-filter=U").stdout.split()
    mid = any(os.path.exists(os.path.join(  # --git-path is relative to repo_root
        repo_root, _git_out(repo_root, "rev-parse", "--git-path", d).stdout.strip()))
        for d in ("rebase-merge", "rebase-apply"))
    if mid:
        _git_out(repo_root, "rebase", "--abort")
    head = _git_out(repo_root, "log", "-1", "--oneline").stdout.strip()
    stash = [l for l in _git_out(repo_root, "stash", "list").stdout.splitlines() if "autostash" in l]
    raise ItemCliError(
        "pull --rebase failed%s; the rebase was aborted, nothing was pushed.\n"
        "  unmerged: %s\n  local commit: %s\n  pending autostash: %s\n"
        "Resolve by hand (for a slug clash: git mv the file to a new slug, amend, push), then "
        "python -m mapkit ids --check. The Item is %s.\n  git: %s" % (
            " (conflict)" if unmerged else "", ", ".join(unmerged) or "none", head,
            "; ".join(stash) or "none", stem, r.stderr.strip()[-300:]))


def _age_days(item: Item) -> int:
    try:
        d = datetime.date.fromisoformat(str(item.meta.get("created"))[:10])
    except ValueError:
        return 0
    return max(0, (datetime.date.today() - d).days)


def _row(i: Item) -> Dict[str, object]:
    return {"id": i.short_id or "", "kind": i.kind, "slug": i.slug, "status": i.status,
            "priority": i.meta.get("priority") or "",
            "pathway": i.slug if i.kind == "pathway" else (i.meta.get("parent") or ""),
            "age_days": _age_days(i), "title": i.title}


def list_items(repo_root: str, pathway: Optional[str] = None, kind: Optional[str] = None,
               status: Optional[str] = None, open_only: bool = False,
               stale: bool = False, today: Optional[datetime.date] = None) -> List[Item]:
    today = today or datetime.date.today()
    out = []
    items = load_items(os.path.join(repo_root, MAP_DIR))
    legacy = _legacy(repo_root) if pathway else {}
    if pathway and (_typed_id(pathway, legacy) or pathway.strip().isdigit()):
        pathway = _pathway(items, pathway, legacy).slug
    for i in items:
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


def _pathway(items: List[Item], text: str, legacy: Optional[Dict[str, str]] = None) -> Item:
    """A Pathway by slug, stem or ID (bare digits allowed: read commands only).

    An old ID the crosswalk knows reads as its new ID (one stderr line, D-3).
    """
    pw = next((i for i in items if i.kind == "pathway" and i.slug == text), None)
    if pw is None:
        try:
            pw = ids.resolve(items, text, allow_bare=True, legacy=legacy)
        except ids.IdError:
            pw = None
        else:
            _note_translation(text, pw, legacy or {})
    if pw is None or pw.kind != "pathway":
        raise ItemCliError("no pathway %r" % text)
    return pw


def show(repo_root: str, pathway: str) -> Dict[str, object]:
    items = load_items(os.path.join(repo_root, MAP_DIR))
    pw = _pathway(items, pathway, _legacy(repo_root))
    pathway = pw.slug
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
    return {"id": pw.short_id or "", "slug": pw.slug, "title": pw.title, "status": pw.status,
            "destination": " ".join(str(pw.meta["destination"]).split()),
            "plans": plan_rows, "children": children}


def _render_show(d: Dict[str, object]) -> str:
    out = ["%s %s [%s] (%s)" % (d["id"] or "-", d["title"], d["status"], d["slug"]),
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
            out.append("  %-6s %-50s %-8s %s" % (r["id"] or "-", r["slug"], r["priority"] or "-",
                                                 r["status"]))
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
        print("%s unchanged (%s) %s" % (label(item), item.status, item.title))
        return 0
    print("%s -> %s %s" % (label(item), item.status, item.title))
    if not args.commit:
        print("stage: %s" % os.path.relpath(item.path, _repo(args)))
        print("stage: %s" % DOCS_PAGE)
    return 0


def _metrics_arg(args) -> Dict[str, Optional[str]]:
    return {f: getattr(args, f, None) for f in METRIC_FIELDS}


def _run_rate(args) -> int:
    try:
        item = rate_item(_repo(args), args.item, _metrics_arg(args),
                         commit=args.commit, push=not args.no_push)
    except (ItemCliError, ItemError, ids.IdError) as exc:
        print("mapkit: %s" % exc, file=sys.stderr)
        return 2
    if not getattr(item, "changed", True):
        print("%s unchanged %s" % (label(item), item.title))
        return 0
    print("%s rated %s" % (label(item), " ".join(
        "%s=%s" % (f, item.meta[f]) for f in METRIC_FIELDS if item.meta.get(f) is not None)))
    if not args.commit:
        print("stage: %s" % os.path.relpath(item.path, _repo(args)))
        print("stage: %s" % DOCS_PAGE)
    return 0


def _run_skip(args) -> int:
    try:
        if args.batch:
            if args.item or args.reason or args.clear or args.note or args.input:
                raise ItemCliError("--batch takes no item, --reason, --note, --input or --clear")
            with open(args.batch, encoding="utf-8") as fh:
                rows = json.load(fh)
            if not isinstance(rows, list):
                raise ItemCliError("batch file must hold a JSON list of rows")
            changed = skip_batch(_repo(args), rows, commit=args.commit, push=not args.no_push)
            print("%d of %d Items changed" % (len(changed), len(rows)))
            if changed and not args.commit:
                for i in changed:
                    print("stage: %s" % os.path.relpath(i.path, _repo(args)))
                print("stage: %s" % DOCS_PAGE)
            return 0
        if not args.item:
            raise ItemCliError("name an Item, or pass --batch FILE")
        item = skip_item(_repo(args), args.item, reason=args.reason, note=args.note,
                         input_=args.input, clear=args.clear, commit=args.commit,
                         push=not args.no_push)
    except (ItemCliError, ItemError, ids.IdError, OSError, ValueError) as exc:
        print("mapkit: %s" % exc, file=sys.stderr)
        return 2
    if not getattr(item, "changed", True):
        print("%s unchanged %s" % (label(item), item.title))
        return 0
    print("%s %s" % (label(item), "skip cleared" if args.clear else "skip %s" % item.meta["night_skip"]))
    if not args.commit:
        print("stage: %s" % os.path.relpath(item.path, _repo(args)))
        print("stage: %s" % DOCS_PAGE)
    return 0


def _run_new(args) -> int:
    body = ""
    if args.body_file:
        with open(args.body_file, encoding="utf-8") as fh:
            body = fh.read()
    elif args.body:
        body = args.body
    try:
        item = new_item(_repo(args), args.kind, args.title, parent=args.parent,
                        priority=args.priority, filed_by=args.filed_by, body=body,
                        links=args.link, ticket_type=args.type, destination=args.destination,
                        commit=args.commit, push=not args.no_push, metrics=_metrics_arg(args),
                        allow_similar=args.allow_similar)
    except (ItemCliError, ItemError, ids.IdError) as exc:
        print("mapkit: %s" % exc, file=sys.stderr)
        return 2
    except subprocess.CalledProcessError as exc:
        print("mapkit: git %s failed (exit %d); the Item file is written, check git status"
              % (" ".join(exc.cmd[1:3]), exc.returncode), file=sys.stderr)
        return 2
    print("%s filed (%s) %s" % (label(item), item.status, item.title))
    if not args.commit:
        print("stage: %s" % os.path.relpath(item.path, _repo(args)))
        print("stage: %s" % DOCS_PAGE)
    return 0


def _run_push(args) -> int:
    try:
        item = push_item(_repo(args), args.item)
    except (ItemCliError, ItemError, ids.IdError) as exc:
        print("mapkit: %s" % exc, file=sys.stderr)
        return 2
    print("%s pushed %s" % (label(item), item.title))
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
    except (ItemError, ItemCliError) as exc:
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
            line = "%-6s %-8s %-50s %-12s %-6s %-30s %dd" % (
                r["id"] or "-", r["kind"], r["slug"], r["status"], r["priority"] or "-",
                r["pathway"] or "-", r["age_days"])
            if stale:
                line += "  stale %dd since %s" % (r["stale_days"], r["stale_clock"])
            print(line)
    return 0


def register(subparsers) -> None:
    def common(p):
        p.add_argument("--repo", default=".", help="repo root (default .)")

    def metric_flags(p):
        for f in METRIC_FIELDS:
            p.add_argument("--" + f, choices=METRIC_ENUMS[f],
                           help="metric %s (optional; validated)" % f)

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

    p = isub.add_parser("new", help="file a new Item with the next ID (optionally commit + push)")
    p.add_argument("kind", choices=KINDS)
    p.add_argument("title")
    p.add_argument("--parent", help="pathway slug or ID")
    p.add_argument("--priority", default="medium", choices=("low", "medium", "high"))
    p.add_argument("--filed-by", default="claude")
    p.add_argument("--body", help="body text (or --body-file)")
    p.add_argument("--body-file")
    p.add_argument("--link", action="append", default=[], help="path, URL, stem or ID (stored as a stem)")
    p.add_argument("--type", choices=("research", "prototype", "grilling", "task", "question"), help="ticket type")
    p.add_argument("--destination", help="pathway Destination (required for a pathway)")
    metric_flags(p)
    p.add_argument("--allow-similar", action="store_true",
                   help="file even though similar Items exist (an exact open duplicate is always refused)")
    p.add_argument("--commit", action="store_true")
    p.add_argument("--no-push", action="store_true")
    common(p)
    p.set_defaults(func=_run_new)

    p = isub.add_parser("rate", help="set item metrics (effort, hardware, owner, radius, "
                                     "diagnosis, area) on an existing Item")
    p.add_argument("item", help="the Item (ID or stem)")
    metric_flags(p)
    p.add_argument("--commit", action="store_true")
    p.add_argument("--no-push", action="store_true")
    common(p)
    p.set_defaults(func=_run_rate)

    p = isub.add_parser("skip", help="set or clear an Item's Night Shift skip reason "
                                     "(does not bump `updated`); --batch FILE.json for many")
    p.add_argument("item", nargs="?", help="the Item (ID or stem)")
    p.add_argument("--reason", choices=NIGHT_SKIP_REASONS)
    p.add_argument("--note", help="one sentence, <= 160 chars")
    p.add_argument("--input", help="what the owner must supply (<= 200 chars); required for input-* reasons")
    p.add_argument("--clear", action="store_true", help="remove the skip reason")
    p.add_argument("--batch", metavar="FILE.json",
                   help='JSON list of {"item","reason","note","input"?} or {"item","clear":true}; '
                        "all-or-nothing")
    p.add_argument("--commit", action="store_true")
    p.add_argument("--no-push", action="store_true")
    common(p)
    p.set_defaults(func=_run_skip)

    p = isub.add_parser("push", help="push an Item's local commit (item new --commit --no-push) "
                                     "through the bounded rebase / renumber / push loop")
    p.add_argument("item", help="the Item (ID or stem), e.g. BUG-14 or idea-my-title")
    common(p)
    p.set_defaults(func=_run_push)

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
