"""Short Item IDs (plans ``map-item-ids.md``, ``item-id-per-kind.md``).

The ID lives in frontmatter ``id:`` only, as the first key, written as a
canonical string with no padding. The filename stem stays the stored
cross-reference; the ID is for people and commands.

This module is the only place the ID format exists (``DOCUMENTATION_GUIDE.md``
section 2.10 states it): ``<PREFIX>-<n>`` with the prefix of the Item's kind
(:data:`PREFIXES`: ``BUG-12``, ``FEAT-7``, ``IDEA-3``, ``PWAY-2``, ``TKT-31``),
canonical upper-case and unpadded; typed forms are loose (``bug12``,
``Bug-12``). The string is the identity: ``BUG-3`` and ``FEAT-3`` are two IDs.

The retired global form of 2026-10-26 to 2026-11-08 is known only to the frozen
crosswalk :data:`LEGACY_IDS` (D-3): :func:`legacy` reads it and
:func:`legacy_ref` looks a typed old ID up in it, so an old ID resolves on a
read and is refused on a write, and no code here names its prefix.

* :func:`allocate` -- 1 + max over the working tree's IDs of one kind and those
  on a ref (default ``origin/main``, read with ``git grep``; no network).
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
  caller is a read command (``allow_bare``), and then every Item carrying that
  number under any prefix is listed when there is more than one.
* :func:`mismatch` -- an ID whose prefix contradicts its Item's kind.
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import subprocess
import sys
from typing import Dict, Iterable, List, Optional, Tuple

from mapkit.atomic import write_text_atomic
from mapkit.items import Item, ItemError, load_items, load_items_lenient, write_item

PREFIXES: Dict[str, str] = {"bug": "BUG", "feature": "FEAT", "idea": "IDEA",
                            "pathway": "PWAY", "ticket": "TKT"}
KIND_OF_PREFIX: Dict[str, str] = {p: k for k, p in PREFIXES.items()}
ALL_PREFIXES: Tuple[str, ...] = tuple(PREFIXES.values())
DEFAULT_REF = "origin/main"
RENUMBER_SUBJECT = "map: renumber"
MAP_DIR = "map"
LEGACY_IDS = "mapkit/legacy_ids.csv"  # the frozen P4 crosswalk: old ID -> new ID (D-3)

_BARE = re.compile(r"^[0-9]+$")
_ANY = "|".join(ALL_PREFIXES)
_CANON_REF = re.compile(r"^(%s)-([1-9][0-9]*)$" % _ANY)
_LOOSE_REF = re.compile(r"^(%s)-?([1-9][0-9]*)$" % _ANY, re.IGNORECASE)
#: Any typed ID shape, prefix unknown: letters, an optional hyphen, a number
#: that may be padded. Only :func:`legacy_ref` uses it, as the key to the crosswalk.
_ANY_SHAPE = re.compile(r"^([A-Za-z]+)-?0*([1-9][0-9]*)$")
_INF = float("inf")


class IdError(LookupError):
    """An unresolvable or ambiguous reference; the message says which.

    ``matches`` holds the Items an ambiguous reference named (empty otherwise),
    so a read command can list them instead of parsing the message.
    """

    def __init__(self, message: str, matches: Optional[List[Item]] = None) -> None:
        super().__init__(message)
        self.matches: List[Item] = list(matches or [])


class LegacyIdError(IdError):
    """A write named an old ID the crosswalk knows; ``new_id`` is what it is called now (D-3)."""

    def __init__(self, message: str, new_id: str) -> None:
        super().__init__(message)
        self.new_id = new_id


def _positive(n: object) -> int:
    if not isinstance(n, int) or isinstance(n, bool) or n < 1:
        raise ValueError("an ID number is a positive int, not %r" % (n,))
    return n


def format_ref(kind: str, n: int) -> str:
    """The per-kind ID of Item kind ``kind`` and number ``n``: ``("bug", 12)`` -> ``BUG-12``."""
    if kind not in PREFIXES:
        raise ValueError("no ID prefix for kind %r (kinds: %s)" % (kind, ", ".join(PREFIXES)))
    return "%s-%d" % (PREFIXES[kind], _positive(n))


def canonical_ref(value: object) -> Optional[str]:
    """``value`` itself when it is a canonical ID, else None.

    Case-sensitive and unpadded: ``BUG-12`` passes; ``bug-12``, ``BUG12``,
    ``BUG-01``, an unknown prefix and a YAML int do not.
    """
    if not isinstance(value, str):
        return None
    return value if _CANON_REF.match(value) else None


def split_ref(value: object) -> Optional[Tuple[str, int]]:
    """``(prefix, number)`` of a canonical ID: ``TKT-31`` -> ``("TKT", 31)``; else None."""
    if not isinstance(value, str):
        return None
    m = _CANON_REF.match(value)
    return (m.group(1), int(m.group(2))) if m else None


def parse_ref(text: str) -> Optional[str]:
    """A typed ID -> its canonical string, else None.

    Loose as typing is: any case, hyphen optional (``bug12``, ``Bug-12``).
    Never an old ID (that is :func:`legacy_ref`'s), never a bare number (that is :func:`resolve`'s read-only
    case) and never a padded one (``BUG-01``). Note ``bug-12`` is also a legal
    stem shape; :func:`resolve` handles that collision.
    """
    if not isinstance(text, str):
        return None
    m = _LOOSE_REF.match(text.strip())
    return "%s-%s" % (m.group(1).upper(), m.group(2)) if m else None


def mismatch(item_id: object, kind: str) -> bool:
    """True when canonical ``item_id``'s prefix names a kind other than ``kind``.

    A non-canonical value is not this class (it is :func:`malformed`'s), so it
    returns False.
    """
    sp = split_ref(item_id)
    if sp is None:
        return False
    return KIND_OF_PREFIX[sp[0]] != kind


def short_id(item: Item) -> Optional[str]:
    """The Item's canonical ID, else None."""
    return canonical_ref(item.meta.get("id"))


def stem(item: Item) -> str:
    return "%s-%s" % (item.kind, item.slug)


# --- git helpers ----------------------------------------------------------
def _git(repo_root: str, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(["git"] + list(args), cwd=repo_root, capture_output=True, text=True)


def upstream_ref(repo_root: str) -> str:
    """The checked-out branch's upstream (``origin/feat``), else :data:`DEFAULT_REF`.

    ``item new --commit`` rebases onto the upstream, so its allocation and its
    repair must read the same ref the rebase brought in.
    """
    r = _git(repo_root, "rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{u}")
    name = r.stdout.strip()
    return name if r.returncode == 0 and name else DEFAULT_REF


def _ref_matches(repo_root: str, ref: str, pattern: str) -> Dict[str, List[Tuple[int, str]]]:
    """``{file name: [(line number, text)]}`` for ``git grep -n`` hits in ``ref:map/``."""
    r = _git(repo_root, "grep", "-n", "-e", pattern, ref, "--", MAP_DIR + "/")
    out: Dict[str, List[Tuple[int, str]]] = {}
    if r.returncode != 0:
        return out
    for line in r.stdout.splitlines():
        # <ref>:map/<name>.md:<n>:<text>
        try:
            _, rest = line.split(":" + MAP_DIR + "/", 1)
            name, num, text = rest.split(":", 2)
            out.setdefault(name, []).append((int(num), text))
        except ValueError:
            continue
    return out


def ref_ids(repo_root: str, ref: str = DEFAULT_REF) -> Dict[str, str]:
    """``{stem: id value}`` for the Items on ``ref``; ``{}`` if the ref is absent.

    Only the frontmatter ``id:`` key counts: a column-0 ``id: BUG-n`` in an
    Item's body (a fenced example) lies after the closing ``---`` and is skipped.
    """
    hits = _ref_matches(repo_root, ref, "^id: ")
    if not hits:
        return {}
    fences = _ref_matches(repo_root, ref, "^---$")
    out: Dict[str, str] = {}
    for name, rows in hits.items():
        if not name.endswith(".md") or "/" in name:
            continue
        marks = sorted(n for n, _ in fences.get(name, []))
        if len(marks) < 2 or marks[0] != 1:
            continue  # no frontmatter block: not an Item the loader would read
        inside = [t for n, t in rows if 1 < n < marks[1]]
        if inside:
            out[name[:-3]] = inside[0][len("id: "):].strip().strip("'\"")
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
    """Strict: the first malformed Item raises (``ids --check`` is a gate)."""
    return load_items(os.path.join(repo_root, MAP_DIR))


def load_lenient(repo_root: str) -> List[Item]:
    """Every loadable Item; a file that fails to parse is skipped and named on
    stderr, so one malformed Item never stops an ID write (its own ``id:`` line
    still counts for allocation: :func:`file_ids`)."""
    items, skipped = load_items_lenient(os.path.join(repo_root, MAP_DIR))
    for path, msg in skipped:
        print("mapkit: skipping unreadable Item %s (%s)" % (os.path.relpath(path, repo_root), msg),
              file=sys.stderr)
    return items


_FM_ID = re.compile(r"^id:\s*(.*?)\s*$")


def file_ids(repo_root: str) -> List[str]:
    """The raw frontmatter ``id:`` value of every ``map/*.md``, parsed or not.

    A text scan (first ``---`` block only), so a file the loader rejects still
    reserves its number: a malformed Item's ID is never handed out again.
    """
    map_dir = os.path.join(repo_root, MAP_DIR)
    out: List[str] = []
    if not os.path.isdir(map_dir):
        return out
    for name in sorted(os.listdir(map_dir)):
        p = os.path.join(map_dir, name)
        if not (name.endswith(".md") and os.path.isfile(p)):
            continue
        try:
            with open(p, encoding="utf-8", errors="replace") as fh:
                if fh.readline().rstrip("\r\n") != "---":
                    continue
                for line in fh:
                    if line.rstrip("\r\n") == "---":
                        break
                    m = _FM_ID.match(line)
                    if m:
                        out.append(m.group(1).strip("'\""))
                        break
        except OSError:
            continue
    return out


def _max_number(values: Iterable[object], prefix: str) -> int:
    """The highest number among canonical IDs carrying ``prefix``; 0 for none."""
    nums = [split_ref(v) for v in values]
    return max([sp[1] for sp in nums if sp is not None and sp[0] == prefix] + [0])


def allocate(repo_root: str, ref: str = DEFAULT_REF,
             items: Optional[List[Item]] = None, kind: Optional[str] = None) -> str:
    """The next ID of ``kind``: 1 + max over that kind's IDs in the tree (every
    ``map/*.md`` frontmatter, loadable or not), ``ref``'s and :data:`DEFAULT_REF`'s
    (both read: a feature branch's upstream may lag or lead ``origin/main``, and a
    higher max is never wrong).

    ``kind`` is required (keyword, so the signature the map app calls is
    unchanged); each kind counts its own Items.
    """
    if kind not in PREFIXES:
        raise ValueError("allocate needs kind= one of %s, not %r" % (", ".join(PREFIXES), kind))
    prefix = PREFIXES[kind]
    items = load_lenient(repo_root) if items is None else items
    top = max(_max_number((i.meta.get("id") for i in items), prefix),
              _max_number(file_ids(repo_root), prefix))
    for r in {ref, DEFAULT_REF}:
        top = max(top, _max_number(ref_ids(repo_root, r).values(), prefix))
    return format_ref(kind, top + 1)


def duplicates(items: List[Item]) -> Dict[str, List[Item]]:
    """``{id: Items}`` for each ID held more than once. Keyed on the whole
    canonical string, so ``BUG-3`` and ``FEAT-3`` are never a duplicate."""
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


def mismatched(items: List[Item]) -> List[Item]:
    """Items whose canonical ID's prefix contradicts their kind (:func:`mismatch`)."""
    return [i for i in items if mismatch(i.meta.get("id"), i.kind)]


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
        "malformed": ["%s: id %r is not <PREFIX>-<n> (PREFIX one of %s)"
                      % (stem(i), i.meta.get("id"), ", ".join(ALL_PREFIXES))
                      for i in malformed(items)],
        "mismatch": ["%s: id %s names kind %s, but the Item is a %s" % (
            stem(i), i.meta["id"], KIND_OF_PREFIX[split_ref(i.meta["id"])[0]], i.kind)  # type: ignore[index]
            for i in mismatched(items)],
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
    write_text_atomic(item.path, "---\nid: %s\n%s%s" % (new_id, head, rest), newline="")
    meta = {"id": new_id}
    meta.update((k, v) for k, v in item.meta.items() if k != "id")
    item.meta = meta


def repair(repo_root: str, ref: str = DEFAULT_REF,
           own: Optional[Iterable[str]] = None) -> List[Tuple[str, str, str]]:
    """Renumber each duplicate's losers; returns ``(old, new, stem)`` rows.

    ``own`` (stems) limits renumbering to those Items (a writer repairs only
    its own unpushed files). With ``own``, any group member outside it is the
    keeper: a file the writer has not pushed can never be the older holder,
    whatever its stem or ``created`` says. A group with no own member is left
    for the gate.
    """
    items = load_lenient(repo_root)
    on_ref = ref_ids(repo_root, ref)
    own_set = set(own) if own is not None else None
    order = add_order(repo_root, ref)
    rows = []
    for sid, group in sorted(duplicates(items).items()):
        if own_set is not None and not any(stem(i) in own_set for i in group):
            continue
        others = [i for i in group if own_set is not None and stem(i) not in own_set]
        holders = [i for i in (others or group) if on_ref.get(stem(i)) == sid]
        keeper = min(holders or others or group, key=lambda i: _order_key(repo_root, i, order))
        for loser in group:
            if loser is keeper or (own_set is not None and stem(loser) not in own_set):
                continue
            new = allocate(repo_root, ref, items, kind=loser.kind)
            set_id(loser, new)
            rows.append((sid, new, stem(loser)))
    return rows


def assign_missing(repo_root: str, ref: str = DEFAULT_REF) -> List[Tuple[str, str]]:
    """Give every ID-less Item the next ID, oldest first; returns ``(id, stem)``."""
    items = load_lenient(repo_root)
    order = add_order(repo_root, ref)
    todo = sorted(missing(items), key=lambda i: _order_key(repo_root, i, order))
    rows = []
    for item in todo:
        new = allocate(repo_root, ref, items, kind=item.kind)
        set_id(item, new)
        rows.append((new, stem(item)))
    return rows


CROSSWALK_COLUMNS = ("old_id", "new_id", "kind", "stem")


def legacy(repo_root: str) -> Dict[str, str]:
    """``{old ID: new ID}`` from :data:`LEGACY_IDS`; ``{}`` when the file is absent.

    The file is frozen and its ``old_id`` column is the only record of the
    retired form, so it outlives that form: commit messages, bookmarks and old
    logs cite old IDs forever. A malformed file raises :class:`ItemError` naming
    the line (wrong header, a missing or empty cell, a ``new_id`` that is not a
    canonical ID of the row's kind, an ``old_id`` listed twice), never a bare
    ``KeyError`` from deep in a command.
    """
    path = os.path.join(repo_root, LEGACY_IDS)
    if not os.path.isfile(path):
        return {}
    out: Dict[str, str] = {}
    with open(path, newline="", encoding="utf-8") as fh:
        rows = csv.reader(fh)
        head = next(rows, None)
        if head is None or tuple(head) != CROSSWALK_COLUMNS:
            raise ItemError("%s:1: header %r, want %s" % (LEGACY_IDS, head, ",".join(CROSSWALK_COLUMNS)))
        for n, row in enumerate(rows, 2):
            if len(row) != len(CROSSWALK_COLUMNS) or not all(c.strip() for c in row):
                raise ItemError("%s:%d: want %d non-empty cells, got %r"
                                % (LEGACY_IDS, n, len(CROSSWALK_COLUMNS), row))
            old, new, kind = row[0], row[1], row[2]
            sp = split_ref(new)
            if sp is None or KIND_OF_PREFIX[sp[0]] != kind:
                raise ItemError("%s:%d: new_id %r is not a canonical ID of kind %r" % (LEGACY_IDS, n, new, kind))
            if old in out:
                raise ItemError("%s:%d: old_id %s listed twice" % (LEGACY_IDS, n, old))
            out[old] = new
    return out


def legacy_ref(text: object, crosswalk: Dict[str, str]) -> Optional[str]:
    """The new ID a typed old ID names in ``crosswalk`` (:func:`legacy`), else None.

    Any spelling of the old ID (owner, 2026-11-08): case and the hyphen are
    free and padding is ignored, so an old ID typed in lower case, unhyphenated
    or zero-padded finds the row of its canonical spelling. The crosswalk's keys are the only knowledge of the
    old form; a current ID (``BUG-12``) is in no row and gets None.
    """
    if not isinstance(text, str) or not crosswalk:
        return None
    m = _ANY_SHAPE.match(text.strip())
    return crosswalk.get("%s-%s" % (m.group(1).upper(), m.group(2))) if m else None


def _bare_number(t: str) -> Optional[int]:
    return int(t) if _BARE.match(t) and int(t) >= 1 else None


def resolve(items: List[Item], text: str, allow_bare: bool = False,
            legacy: Optional[Dict[str, str]] = None) -> Item:
    """Exact stem, then slug, then ID.

    Errors (:class:`IdError`, with ``matches``) rather than guesses when two
    readings disagree: a stem that is another Item's typed ID (``bug-12`` is
    both a stem shape and ``BUG-12``), a slug that is another Item's ID, or one
    ID held twice. Bare digits are an ID only on a read (``allow_bare``), and
    there they match the number in every form, so a number shared across kinds
    lists every holder instead of picking one.

    ``legacy`` is the crosswalk (:func:`legacy`, D-3): an old ID in any
    spelling (:func:`legacy_ref`) that is no Item's stem or current ID
    resolves, on a read (``allow_bare``), to the Item holding the new ID; on a
    write it raises :class:`LegacyIdError` naming the new ID. Absent or empty,
    an old ID is simply no Item.
    """
    t = text.strip()
    if t.endswith(".md"):
        t = t[:-3]
    ref = parse_ref(t)
    by_ref = [i for i in items if ref is not None and short_id(i) == ref]
    new_ref = legacy_ref(t, legacy) if (legacy and not by_ref
                                        and not any(stem(i) == t for i in items)) else None
    if new_ref is not None:
        holder = [i for i in items if short_id(i) == new_ref]
        if not allow_bare:
            raise LegacyIdError("%s is now %s%s; writes take the new ID" % (
                t, new_ref, " (%s)" % stem(holder[0]) if holder else ""), new_ref)
        ref = new_ref
        by_ref = holder
    for i in items:
        if stem(i) == t:
            others = [j for j in by_ref if j is not i]
            if others:
                raise IdError("%r is both the stem of %s and the ID of %s: name %s by "
                              "its ID or slug, %s by its stem"
                              % (t, stem(i), ", ".join(stem(j) for j in others),
                                 stem(i), ", ".join(stem(j) for j in others)),
                              [i] + others)
            return i
    by_slug = [i for i in items if i.slug == t]
    if len(by_slug) > 1:
        raise IdError("slug %r is ambiguous (%s); use the full stem"
                      % (t, ", ".join(stem(i) for i in by_slug)), by_slug)
    n = _bare_number(t) if ref is None and allow_bare else None
    if n is not None:
        by_id = sorted((i for i in items if (split_ref(i.meta.get("id")) or ("", 0))[1] == n), key=stem)
        if len({short_id(i) for i in by_id}) > 1:
            raise IdError("bare %s names %d Items (%s); use the prefixed ID" % (
                t, len(by_id), ", ".join("%s %s" % (short_id(i), stem(i)) for i in by_id)), by_id)
    else:
        by_id = by_ref
    if len(by_id) > 1:
        raise IdError("%s names %d Items (%s); run python -m mapkit ids --repair"
                      % (short_id(by_id[0]), len(by_id), ", ".join(stem(i) for i in by_id)), by_id)
    if by_slug and by_id and by_slug[0] is not by_id[0]:
        raise IdError("%r is both the slug of %s and the ID of %s; use the full stem"
                      % (t, stem(by_slug[0]), stem(by_id[0])), [by_slug[0], by_id[0]])
    if by_slug:
        return by_slug[0]
    if by_id:
        return by_id[0]
    hint = ("" if allow_bare or not _BARE.match(t)
            else " (write commands need a prefixed ID, e.g. BUG-12)")
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
    for cls in ("duplicate", "malformed", "mismatch", "changed"):
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
