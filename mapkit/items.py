"""Map items: ``map/<kind>-<slug>.md`` in one flat directory (ticket #25).

Identity is the filename; the slug is the filename without ``<kind>-`` and
``.md``. The loader rejects a file whose frontmatter ``kind`` does not match
the filename prefix, a status outside its kind's set, a missing required
field, and malformed typed fields. Non-markdown files are ignored. Dates
(``created``) become ISO ``YYYY-MM-DD`` strings and ``updated`` an ISO
timestamp string; files are never rewritten. The short ``id:`` (``JB-<n>``,
``mapkit/ids.py``) is deliberately NOT validated here: a malformed value must
never hide an Item; the gate and ``mapkit ids --check`` report it.
``item_uuid(slug)`` is ``uuid5(ITEM_NAMESPACE, slug)``, stable across runs and
machines.
"""
from __future__ import annotations

import datetime
import os
import re
import uuid
from dataclasses import dataclass
from typing import Any, Dict, FrozenSet, List, Optional, Tuple

from mapkit.frontmatter import FrontmatterError, split_frontmatter
from mapkit.plans import iso_date

KINDS: Tuple[str, ...] = ("pathway", "idea", "feature", "bug", "ticket")
INBOX_KINDS: Tuple[str, ...] = ("idea", "feature", "bug")

_INBOX_STATUSES = frozenset(
    ("inbox", "backlog", "in_progress", "complete", "dropped", "promoted"))
STATUS_SETS: Dict[str, FrozenSet[str]] = {
    "pathway": frozenset(("active", "complete", "dropped")),
    "idea": _INBOX_STATUSES,
    "feature": _INBOX_STATUSES,
    "bug": _INBOX_STATUSES,
    "ticket": frozenset(("open", "complete", "dropped")),
}

_COMMON_REQUIRED = ("kind", "title", "status", "created")
REQUIRED_FIELDS: Dict[str, Tuple[str, ...]] = {
    "pathway": _COMMON_REQUIRED + ("destination",),
    "idea": _COMMON_REQUIRED,
    "feature": _COMMON_REQUIRED,
    "bug": _COMMON_REQUIRED,
    "ticket": _COMMON_REQUIRED + ("parent", "type"),
}

PRIORITIES = ("low", "medium", "high")
TICKET_TYPES = ("research", "prototype", "grilling", "task", "question")

#: Item metrics (owner-approved 2026-10-28): optional on a new Item, validated
#: when present. Enum values are in rank order; the field order is the
#: canonical frontmatter order (after ``priority``). ``diagnosis`` is bugs only.
METRIC_ENUMS: Dict[str, Tuple[str, ...]] = {
    "effort": ("xs", "s", "m", "l", "xl"),
    "hardware": ("none", "bench-leg", "robot"),
    "owner": ("none", "decision", "hands"),
    "radius": ("local", "subsystem", "cross-cutting", "normative"),
    "diagnosis": ("unknown", "hypothesis", "confirmed"),
    "area": ("firmware", "can-bridge", "legs", "hand", "platform-teensy", "tracking",
             "ball-butler", "skill-stack", "learner", "gui", "map-app", "tests-tooling",
             "docs", "sim"),
}
METRIC_FIELDS: Tuple[str, ...] = tuple(METRIC_ENUMS)
#: The two metrics a plan step may carry (same enums).
STEP_METRIC_FIELDS: Tuple[str, ...] = ("effort", "hardware")

LIST_FIELDS = ("owners", "blocked_by", "links")
DEFAULT_BRANCH = "main"

#: Fixed namespace for item uuids. Never change it: uuids are identities.
ITEM_NAMESPACE = uuid.UUID("6f1c3b0e-5d7a-5e2b-9a61-4a1f2c8d9e30")


class ItemError(ValueError):
    """A map item file is malformed; the message names the path."""


@dataclass
class Item:
    path: str
    kind: str
    slug: str
    meta: Dict[str, Any]
    body: str
    changed: bool = True  # set by set-status: False when the save was a no-op

    @property
    def filename(self) -> str:
        return os.path.basename(self.path)

    @property
    def uuid(self) -> str:
        return item_uuid(self.slug)

    @property
    def short_id(self) -> Optional[str]:
        """The canonical ``JB-<n>`` ID, or None when absent or malformed."""
        from mapkit.ids import short_id  # local: ids imports this module
        return short_id(self)

    @property
    def status(self) -> str:
        return self.meta["status"]

    @property
    def title(self) -> str:
        return self.meta["title"]


def has_answer(body: str) -> bool:
    """True when ``body`` has a ``## Answer`` section with at least one non-blank line.

    Used by ``item set-status`` (warns on completing a question ticket without
    one) and by the map app's answer box (refuses a second answer). The Night
    Shift merge gate does NOT read it: a question releases its parent Item only
    when its ``status`` is ``complete`` or ``dropped``, which the app's save sets
    together with the answer; an answered-but-open question still holds.
    """
    in_answer = False
    for line in body.splitlines():
        if line.rstrip() == "## Answer":
            in_answer = True
        elif line.startswith("## "):
            if in_answer:
                return False
        elif in_answer and line.strip():
            return True
    return False


def question_tickets(items: List[Item], parent_item: str) -> List[Item]:
    """The question tickets whose ``parent`` names ``parent_item`` (stem, slug or ``JB-n``), any status."""
    names = set([parent_item.strip()])
    for i in items:
        if i.kind == "ticket":
            continue
        if parent_item.strip() in ("%s-%s" % (i.kind, i.slug), i.slug, i.short_id):
            names.update(["%s-%s" % (i.kind, i.slug), i.slug])
            if i.short_id:
                names.add(i.short_id)
    return [t for t in items
            if t.kind == "ticket" and t.meta.get("type") == "question" and t.meta.get("parent") in names]


def open_questions(items: List[Item], parent_item: str) -> List[Item]:
    """The ``open`` ones among :func:`question_tickets`: each holds its parent at the Night Shift merge gate."""
    return [t for t in question_tickets(items, parent_item) if t.status == "open"]


def item_uuid(slug: str) -> str:
    """Stable uuid string for an item slug."""
    return str(uuid.uuid5(ITEM_NAMESPACE, slug))


def split_item_filename(filename: str) -> Tuple[str, str]:
    """``'bug-foo.md'`` -> ``('bug', 'foo')``; ValueError otherwise."""
    if not filename.endswith(".md"):
        raise ValueError("not a markdown file: %r" % filename)
    stem = filename[:-3]
    for kind in KINDS:
        prefix = kind + "-"
        if stem.startswith(prefix) and len(stem) > len(prefix):
            return kind, stem[len(prefix):]
    raise ValueError("filename %r does not start with one of %s" % (
        filename, ", ".join(k + "-" for k in KINDS)))


def _iso_timestamp(value: Any) -> Any:
    if isinstance(value, (datetime.datetime, datetime.date)):
        return value.isoformat()
    return value


def load_item(path: str) -> Item:
    """Parse and validate one item file; raises :class:`ItemError`."""
    filename = os.path.basename(path)
    try:
        kind, slug = split_item_filename(filename)
    except ValueError as exc:
        raise ItemError("%s: %s" % (path, exc)) from exc
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    try:
        raw, body = split_frontmatter(text)
    except FrontmatterError as exc:
        raise ItemError("%s: %s" % (path, exc)) from exc
    meta = dict(raw)
    errs: List[str] = []
    if meta.get("kind") != kind:
        errs.append("frontmatter kind %r does not match filename prefix %r"
                    % (meta.get("kind"), kind))
    for f in REQUIRED_FIELDS[kind]:
        if meta.get(f) in (None, ""):
            errs.append("missing required field %r" % f)
    status = meta.get("status")
    if status is not None and status not in STATUS_SETS[kind]:
        errs.append("status %r not in %s" % (status, sorted(STATUS_SETS[kind])))
    if "created" in meta:
        meta["created"] = iso_date(meta["created"])
    if "updated" in meta:
        meta["updated"] = _iso_timestamp(meta["updated"])
    for f in LIST_FIELDS:
        if f in meta and meta[f] is not None and not isinstance(meta[f], list):
            errs.append("field %r must be a list" % f)
    if meta.get("priority") is not None and meta["priority"] not in PRIORITIES:
        errs.append("priority %r not in %s" % (meta["priority"], PRIORITIES))
    for f in METRIC_FIELDS:
        v = meta.get(f)
        if v is None:
            continue
        if not isinstance(v, str) or v not in METRIC_ENUMS[f]:
            errs.append("%s %r not in %s" % (f, v, METRIC_ENUMS[f]))
    if kind != "bug" and meta.get("diagnosis") is not None:
        errs.append("field 'diagnosis' is for bugs only (this is a %s)" % kind)
    if kind == "ticket":
        if meta.get("type") is not None and meta["type"] not in TICKET_TYPES:
            errs.append("ticket type %r not in %s" % (meta["type"], TICKET_TYPES))
        rank = meta.get("rank")
        if rank is not None and (isinstance(rank, bool) or not isinstance(rank, int)):
            errs.append("ticket rank %r must be an int" % (rank,))
    if kind == "pathway":
        meta.setdefault("branch", DEFAULT_BRANCH)
    if errs:
        raise ItemError("%s: %s" % (path, "; ".join(errs)))
    return Item(path=path, kind=kind, slug=slug, meta=meta, body=body)


def load_items(map_dir: str) -> List[Item]:
    """Every ``*.md`` item in ``map_dir`` (flat, sorted); non-markdown ignored.

    Raises :class:`ItemError` on the first malformed item.
    """
    if not os.path.isdir(map_dir):
        return []
    out: List[Item] = []
    for name in sorted(os.listdir(map_dir)):
        p = os.path.join(map_dir, name)
        if name.endswith(".md") and os.path.isfile(p):
            out.append(load_item(p))
    return out


def load_items_lenient(map_dir: str) -> "tuple":
    """``(items, skipped)``: every loadable item, plus ``[(path, message)]`` for
    the files that raised :class:`ItemError`. For writers (ID allocation,
    repair) that must not die on one malformed sibling."""
    if not os.path.isdir(map_dir):
        return [], []
    out: List[Item] = []
    skipped = []
    for name in sorted(os.listdir(map_dir)):
        p = os.path.join(map_dir, name)
        if name.endswith(".md") and os.path.isfile(p):
            try:
                out.append(load_item(p))
            except ItemError as exc:
                skipped.append((p, str(exc)))
    return out, skipped


def dump_item(meta: Dict[str, Any], body: str) -> str:
    """Item file text: frontmatter, one blank line, the body.

    Leading newlines of ``body`` are dropped (the loader returns the blank
    line as part of the body), so load -> dump of a file written by this
    function is byte-identical.
    """
    import yaml  # local: items.py stays importable without it for readers

    meta = dict(meta)
    if "id" in meta:  # the short ID is always the first key (grep '^id: JB-n$')
        meta = dict([("id", meta.pop("id"))] + list(meta.items()))
    block = yaml.safe_dump(meta, sort_keys=False, allow_unicode=True,
                           width=1000)
    return "---\n" + block + "---\n\n" + body.lstrip("\n")


def write_item(path: str, meta: Dict[str, Any], body: str) -> bool:
    """Write an item file with :func:`dump_item`; True if the file changed.

    A no-op save is not written and does not bump ``updated``: when the
    canonical text built with the file's OLD ``updated`` equals the file's
    current content, nothing is touched (callers set ``updated`` to now
    before calling; it is ignored in that case). Otherwise ``meta`` is
    written as given.
    """
    try:
        with open(path, encoding="utf-8", newline="") as fh:
            current = fh.read()
        raw, _ = split_frontmatter(current)
    except (OSError, FrontmatterError):
        current, raw = None, {}
    if current is not None and "updated" in raw:
        probe = dict(meta)
        probe["updated"] = raw["updated"]
        if dump_item(probe, body) == current:
            return False
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(dump_item(meta, body))
    return True


#: Days an item may sit in a status before it is flagged stale.
#: The one place thresholds live on the Jugglebot side; the app keeps its own
#: copy in its config and both are tested against the same fixture.
STALE_DAYS: Dict[str, int] = {"inbox": 30, "backlog": 60, "in_progress": 14}

#: The one Progress rule, mirrored line for line by the app's ``progressDates``
#: (``frontend/src/lib/done.ts`` in the fork) and pinned by the shared
#: fixture: a line counts when it opens with a date (optionally after a
#: ``-``/``*`` bullet) followed by a separator (``:``, ``-``, en or em dash);
#: every ``## Progress`` section counts, the heading matched case-insensitively;
#: an impossible date (``2026-02-30``) is skipped.
_PROGRESS_HEAD = re.compile(r"^##\s+progress\s*$", re.IGNORECASE)
_PROGRESS_DATE = re.compile(r"^\s*(?:[-*]\s+)?(\d{4}-\d{2}-\d{2})\s*[:\-\u2013\u2014]")


def _progress_dates(body: str) -> List[datetime.date]:
    out: List[datetime.date] = []
    in_progress = False
    for line in body.split("\n"):
        if re.match(r"##\s", line):
            in_progress = _PROGRESS_HEAD.match(line) is not None
            continue
        if in_progress:
            m = _PROGRESS_DATE.match(line)
            if m:
                try:
                    out.append(datetime.date.fromisoformat(m.group(1)))
                except ValueError:
                    pass
    return out


#: The readability rule for an Item body (owner, 2026-10-30, after JB-114 was
#: filed as one 3,500-character paragraph): every paragraph and every list
#: item (with its continuation lines) at most BLOCK_CAP characters, every
#: ``## Progress`` line at most PROGRESS_CAP, and a body over HEADINGLESS_CAP
#: (Progress excluded) carries at least one ``##`` heading. Tables and fenced
#: code (``` or ~~~) are exempt. Pinned for every committed Item by
#: ``tests/sim/test_map_items.py::test_item_bodies_are_readable``; the writing
#: guidance is ``docs/agents/map-writes.md``.
BLOCK_CAP = 500
PROGRESS_CAP = 400
HEADINGLESS_CAP = 1200
_HEADING = re.compile(r"^#{1,6}\s")
_SECTION = re.compile(r"^#{1,2}\s")
_H2 = re.compile(r"^##\s")
_LIST_ITEM = re.compile(r"^\s*(?:[-*+]|\d+[.)])\s+")


def readability_violations(body: str) -> List[str]:
    """Every breach of the readability rule in ``body``, one string each."""
    out: List[str] = []
    blocks: List[List[str]] = []
    cur: Optional[List[str]] = None
    in_fence = in_progress = has_heading = False
    main_len = 0
    for line in body.split("\n"):
        s = line.strip()
        if s.startswith(("```", "~~~")):
            in_fence, cur = not in_fence, None
            continue
        if in_fence:
            continue
        if _HEADING.match(s):
            cur = None
            # only an H1/H2 opens or closes Progress, as in _progress_dates
            if _SECTION.match(s):
                in_progress = _PROGRESS_HEAD.match(s) is not None
            has_heading = has_heading or (not in_progress and _H2.match(s) is not None)
            continue
        if in_progress:
            if len(s) > PROGRESS_CAP:
                out.append("Progress line of %d chars (cap %d): %.60s..." % (len(s), PROGRESS_CAP, s))
            continue
        main_len += len(line) + 1
        if not s or s.startswith("|"):
            cur = None
        elif cur is None or _LIST_ITEM.match(line):
            cur = [s]
            blocks.append(cur)
        else:
            cur.append(s)
    for b in blocks:
        text = " ".join(b)
        if len(text) > BLOCK_CAP:
            out.append("block of %d chars (cap %d): %.60s..." % (len(text), BLOCK_CAP, text))
    if main_len > HEADINGLESS_CAP and not has_heading:
        out.append("body of %d chars with no ## heading (cap %d)" % (main_len, HEADINGLESS_CAP))
    return out


def stale_clock(item: Item) -> datetime.date:
    """Date the staleness clock started: the newest dated ``## Progress``
    line, else ``created``.

    ``updated`` is deliberately ignored: the app and every CLI write bump it,
    so it measures the last save, not the last attention paid to the item.
    """
    dates = _progress_dates(item.body)
    if dates:
        return max(dates)
    return datetime.date.fromisoformat(str(item.meta["created"])[:10])


def is_stale(item: Item, today: datetime.date) -> bool:
    """True when ``today - stale_clock`` is strictly greater than the
    status's :data:`STALE_DAYS`; statuses not listed are never stale."""
    limit = STALE_DAYS.get(item.status)
    if limit is None:
        return False
    return (today - stale_clock(item)).days > limit
