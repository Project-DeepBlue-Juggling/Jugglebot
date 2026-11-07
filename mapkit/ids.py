"""Short Item IDs (plans ``map-item-ids.md``, ``item-id-per-kind.md``).

The ID lives in frontmatter ``id:`` only, as the first key, written as a
canonical string with no padding. The filename stem stays the stored
cross-reference; the ID is for people and commands.

Two forms are canonical during the per-kind expansion (P1 to P5): the legacy
global ``JB-<n>`` and the per-kind ``<PREFIX>-<n>`` of :data:`PREFIXES`
(``BUG-12``, ``FEAT-7``, ``IDEA-3``, ``PWAY-2``, ``TKT-31``). The string is the
identity: ``BUG-3`` and ``FEAT-3`` are two IDs. :func:`parse_ref`,
:func:`canonical_ref` and :func:`format_ref` speak both forms; the int-based
:func:`parse`, :func:`canonical` and :func:`format_id` are ``JB-<n>`` only and
keep their meaning until P5, because the live map app imports them.

* :func:`allocate` -- 1 + max over the working tree's IDs and those on a ref
  (default ``origin/main``, read with ``git grep``; no network). The tree
  decides the form (:func:`per_kind`): without ``mapkit/legacy_ids.csv`` it is
  one global ``JB-<n>`` sequence, with it one sequence per kind.
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
  number in any form is listed when there is more than one.
* :func:`mismatch` -- an ID whose prefix contradicts its Item's kind
  (``JB-<n>`` matches every kind).
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from typing import Dict, Iterable, List, Optional, Tuple

from mapkit.items import Item, ItemError, load_items, load_items_lenient, write_item

PREFIX = "JB"  # the legacy global form; still canonical until P5
PREFIXES: Dict[str, str] = {"bug": "BUG", "feature": "FEAT", "idea": "IDEA",
                            "pathway": "PWAY", "ticket": "TKT"}
KIND_OF_PREFIX: Dict[str, str] = {p: k for k, p in PREFIXES.items()}
ALL_PREFIXES: Tuple[str, ...] = (PREFIX,) + tuple(PREFIXES.values())
DEFAULT_REF = "origin/main"
RENUMBER_SUBJECT = "map: renumber"
MAP_DIR = "map"
LEGACY_IDS = "mapkit/legacy_ids.csv"  # the P4 crosswalk; present = per-kind tree

_CANON = re.compile(r"^%s-([1-9][0-9]*)$" % PREFIX)
_LOOSE = re.compile(r"^%s-?([0-9]+)$" % PREFIX, re.IGNORECASE)
_BARE = re.compile(r"^[0-9]+$")
_ANY = "|".join(ALL_PREFIXES)
_CANON_REF = re.compile(r"^(%s)-([1-9][0-9]*)$" % _ANY)
_LOOSE_REF = re.compile(r"^(%s)-?([1-9][0-9]*)$" % _ANY, re.IGNORECASE)
_INF = float("inf")


class IdError(LookupError):
    """An unresolvable or ambiguous reference; the message says which.

    ``matches`` holds the Items an ambiguous reference named (empty otherwise),
    so a read command can list them instead of parsing the message.
    """

    def __init__(self, message: str, matches: Optional[List[Item]] = None) -> None:
        super().__init__(message)
        self.matches: List[Item] = list(matches or [])


def _positive(n: object) -> int:
    if not isinstance(n, int) or isinstance(n, bool) or n < 1:
        raise ValueError("an ID number is a positive int, not %r" % (n,))
    return n


def format_id(n: int) -> str:
    """``JB-<n>``. Legacy form, kept for the live app until P5."""
    return "%s-%d" % (PREFIX, _positive(n))


def format_ref(kind: str, n: int) -> str:
    """The per-kind ID of Item kind ``kind`` and number ``n``: ``("bug", 12)`` -> ``BUG-12``."""
    if kind not in PREFIXES:
        raise ValueError("no ID prefix for kind %r (kinds: %s)" % (kind, ", ".join(PREFIXES)))
    return "%s-%d" % (PREFIXES[kind], _positive(n))


def canonical_ref(value: object) -> Optional[str]:
    """``value`` itself when it is a canonical ID in either form, else None.

    Case-sensitive and unpadded: ``BUG-12`` and ``JB-149`` pass; ``bug-12``,
    ``BUG12``, ``BUG-01`` and a YAML int do not.
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
    """A typed ID in either form -> its canonical string, else None.

    Loose as typing is: any case, hyphen optional (``bug12``, ``Bug-12``,
    ``jb149``). Never a bare number (that is :func:`resolve`'s read-only
    case) and never a padded one (``BUG-01``). Note ``bug-12`` is also a legal
    stem shape; :func:`resolve` handles that collision.
    """
    if not isinstance(text, str):
        return None
    m = _LOOSE_REF.match(text.strip())
    return "%s-%s" % (m.group(1).upper(), m.group(2)) if m else None


def mismatch(item_id: object, kind: str) -> bool:
    """True when canonical ``item_id``'s prefix names a kind other than ``kind``.

    ``JB-<n>`` matches every kind; a non-canonical value is not this class
    (it is :func:`malformed`'s), so it returns False.
    """
    sp = split_ref(item_id)
    if sp is None or sp[0] == PREFIX:
        return False
    return KIND_OF_PREFIX[sp[0]] != kind


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
    """The Item's canonical ID in either form (unchanged for every ``JB-<n>``), else None."""
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

    Only the frontmatter ``id:`` key counts: a column-0 ``id: JB-n`` in an
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


def _max_number(values: Iterable[object], prefix: str = PREFIX) -> int:
    """The highest number among canonical IDs carrying ``prefix``; 0 for none."""
    nums = [split_ref(v) for v in values]
    return max([sp[1] for sp in nums if sp is not None and sp[0] == prefix] + [0])


def per_kind(repo_root: str) -> bool:
    """The tree's allocator form: :data:`LEGACY_IDS` present means per kind.

    The file arrives with the renumber commit (P4) and leaves with its revert,
    so every clone switches with its own tree and no flag has to be kept in step.
    """
    return os.path.isfile(os.path.join(repo_root, LEGACY_IDS))


def allocate(repo_root: str, ref: str = DEFAULT_REF,
             items: Optional[List[Item]] = None, kind: Optional[str] = None) -> str:
    """1 + max over the tree's IDs (every ``map/*.md`` frontmatter, loadable or not),
    ``ref``'s and :data:`DEFAULT_REF`'s (both read: a feature branch's upstream may lag or lead
    ``origin/main``, and a higher max is never wrong).

    Without :data:`LEGACY_IDS` the sequence is the global ``JB-<n>`` one, exactly
    as before the expansion, and ``kind`` (validated if given) does not change
    the result. With it the max is scoped to ``kind``'s prefix and ``kind`` is
    required: allocating ``JB-<n>`` into a per-kind tree would be silent.
    """
    if kind is not None and kind not in PREFIXES:
        raise ValueError("no ID prefix for kind %r (kinds: %s)" % (kind, ", ".join(PREFIXES)))
    if per_kind(repo_root):
        if kind is None:
            raise ValueError("%s is present, so IDs are per kind: allocate needs kind="
                             % LEGACY_IDS)
        prefix = PREFIXES[kind]
    else:
        prefix = PREFIX
    items = load_lenient(repo_root) if items is None else items
    top = max(_max_number((i.meta.get("id") for i in items), prefix),
              _max_number(file_ids(repo_root), prefix))
    for r in {ref, DEFAULT_REF}:
        top = max(top, _max_number(ref_ids(repo_root, r).values(), prefix))
    return format_id(top + 1) if prefix == PREFIX else format_ref(kind, top + 1)  # type: ignore[arg-type]


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
    with open(item.path, "w", encoding="utf-8", newline="") as fh:
        fh.write("---\nid: %s\n%s%s" % (new_id, head, rest))
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


def typed_ref(t: object) -> Optional[str]:
    """The canonical ID string a typed or stored value names, either form, else None.

    :func:`parse_ref`, plus the legacy parser's padded ``jb-01`` so every ``JB`` input
    resolves exactly as it did before the expansion. Never a number (a number compare
    made ``BUG-3 == FEAT-3``); a non-string is None.
    """
    if not isinstance(t, str):
        return None
    ref = parse_ref(t)
    if ref is None:
        n = parse(t)
        ref = format_id(n) if n is not None else None
    return ref


def _bare_number(t: str) -> Optional[int]:
    return int(t) if _BARE.match(t) and int(t) >= 1 else None


def resolve(items: List[Item], text: str, allow_bare: bool = False) -> Item:
    """Exact stem, then slug, then ID, in either ID form.

    Errors (:class:`IdError`, with ``matches``) rather than guesses when two
    readings disagree: a stem that is another Item's typed ID (``bug-12`` is
    both a stem shape and ``BUG-12``), a slug that is another Item's ID, or one
    ID held twice. Bare digits are an ID only on a read (``allow_bare``), and
    there they match the number in every form, so a number shared across kinds
    lists every holder instead of picking one.
    """
    t = text.strip()
    if t.endswith(".md"):
        t = t[:-3]
    ref = typed_ref(t)
    by_ref = [i for i in items if ref is not None and short_id(i) == ref]
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
            else " (write commands need a prefixed ID, e.g. %s-12 or BUG-12)" % PREFIX)
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
