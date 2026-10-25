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
TICKET_TYPES = ("research", "prototype", "grilling", "task")
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

_PROGRESS_DATE = re.compile(r"^\s*(?:[-*]\s+)?(\d{4}-\d{2}-\d{2})(?!\d)")


def _progress_dates(body: str) -> List[datetime.date]:
    out: List[datetime.date] = []
    in_progress = False
    for line in body.split("\n"):
        if line.startswith("## "):
            in_progress = line.strip() == "## Progress"
            continue
        if in_progress:
            m = _PROGRESS_DATE.match(line)
            if m:
                try:
                    out.append(datetime.date.fromisoformat(m.group(1)))
                except ValueError:
                    pass
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
