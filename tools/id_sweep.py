"""The per-kind Item ID sweeper (plan ``item-id-per-kind.md`` P3).

Finds every token in the tracked tree that matches the deliberately broad
:data:`PATTERN` and gives each exactly one class from the plan's table:

=============== =============================================================
``citation``     rewrite by full-token crosswalk lookup
``branch``       rewrite; the live branch is renamed at P4
``list``         expand by grammar (``JB-120/139``, ``JB-102, 166, 167``)
``range``        expand by grammar (``JB-68..71``, ``JB-152 to JB-157``)
``placeholder``  hand-written per context (overrides file)
``loose``        hand-written per context (overrides file)
``fixture``      hand-converted: the test files whose IDs assert parser behaviour
                 or build synthetic map repos (:data:`FIXTURE_FILES`)
``code_literal`` leave at P4: literals, regexes, identifiers and ````...````
                 examples in the ID code that implement dual-form reading of the
                 old form; P5 (the contract phase) retires them
``abbreviation`` leave: ``JB`` meaning Jugglebot (``JB_*``, ``defer jb``, ...)
``frozen``       leave: filenames, stems, ``Logbook-Entry:``, ``plant_id``,
                 real paths, text whose subject is the old scheme, the sweeper
=============== =============================================================

The classes come from an ordered, declarative rule table (:data:`RULES`); the
first rule whose predicates all hold wins and its name is reported. A token no
rule takes, or a rewritten token whose number is not an Item ID in ``map/``,
is an error: the run names every such token and writes nothing. Nothing
defaults to "leave" or to "rewrite".

The five per-kind prefixes are walked too (:data:`PER_KIND_PATTERN`), so the
census shows the option text P4 must backtick (``option_text``, in ``map/``)
and every other per-kind ID already in the tree (``per_kind``). They are
counted apart from the broad-pattern tokens.

A ``placeholder`` or ``loose`` token is rewritten only from the overrides file
(:data:`OVERRIDE_COLUMNS`; keyed by path and the exact line), and a token no
rule takes but an override row's ``old`` span covers gets that row's class
(rule ``override``).

Usage::

    python tools/id_sweep.py --census [--root DIR] [--out DIR] [--overrides TSV]
    python tools/id_sweep.py --apply  --crosswalk CSV [--dry-run] [--root DIR] [--out DIR] [--overrides TSV]
    python tools/id_sweep.py --verify --crosswalk CSV [--root DIR] [--overrides TSV]

``--apply`` is single-pass (each line rewritten once, right to left, never
re-scanned), full-token (a lookup takes the whole token, so ``JB-1`` never
touches ``JB-12``), idempotent, and line-count-preserving; it refuses, writing
nothing, on any unclassified token, any number missing from the crosswalk, any
stale override key, or any ``placeholder``/``loose`` token no override covers.
``--verify`` re-runs the finder and proves nothing rewritable is left in the
old form and that a second ``--apply`` would be a no-op.
"""
from __future__ import annotations

import argparse
import contextlib
import csv
import os
import re
import subprocess
import sys
from collections import Counter, defaultdict
from typing import Dict, List, NamedTuple, Optional, Pattern, Set, Tuple

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

from mapkit import ids  # noqa: E402
from mapkit import items as mitems  # noqa: E402

#: The broad pattern (plan P3). The single source: the finder and T-R1 use it.
PATTERN = re.compile(r"(?i)(?<![a-z0-9])jb(?![a-z])")
#: The five per-kind forms, canonical case only (``bug-40-hz-...`` is a stem).
PER_KIND_PATTERN = re.compile(r"(?<![A-Za-z0-9_])(?:%s)-[0-9]+(?![A-Za-z0-9_])"
                              % "|".join(ids.PREFIXES.values()))
PER_KIND_CLASSES = frozenset({"option_text", "per_kind"})
REWRITE_CLASSES = frozenset({"citation", "branch", "list", "range"})
LEAVE_CLASSES = frozenset({"abbreviation", "frozen", "code_literal"})
#: Classes ``--apply`` never touches (``fixture`` is converted by hand at P4).
UNTOUCHED_CLASSES = LEAVE_CLASSES | {"fixture"}

#: Suffixes never read. Every one is a binary format the null-byte probe would
#: also catch except ``.bkp``: the KiCad backups in ``Circuit Diagrams/`` are
#: text whose base64 runs match the pattern by accident (plan § 2.2, "noise").
SKIP_SUFFIXES = (".bkp", ".png", ".jpg", ".jpeg", ".gif", ".pdf", ".xlsx", ".stl",
                 ".npz", ".npy", ".zip", ".joblib", ".glb")

# The token as the classifier sees it: ``jb`` plus identifier characters, any
# hyphen-joined segments (numbers, words, ``<n>``, ``*``, ``%d``, ``\d+``,
# ``{n}``), then any bare-number range or list tail (``..93``, ``/139``).
_SEG = r"(?:<[^<>\s]{1,16}>|\{[^{}\s]{0,16}\}|[A-Za-z0-9_]+|\*|%d|\\d\+?)"
CORE = re.compile(r"jb[A-Za-z0-9_]*(?:-%s)*(?:(?:\.\.|–|/)[0-9]+(?![A-Za-z0-9_]))*" % _SEG,
                  re.IGNORECASE)
# The path-like word around a token (for filename, stem and branch context).
_WORDCH = re.compile(r"[A-Za-z0-9_./~<>*%{}-]")
_NUM = re.compile(r"[0-9]+")

#: Files whose subject is the old ``JB-<n>`` scheme itself (plan § 2.2 "Text
#: whose subject is the old scheme", enumerated here at P3): the map-item-ids
#: plan and its logbook arc, this plan with its ADR and logbook entries, and
#: the collision bug whose fix is the renumber narrative (612ce236, 67b8d886).
#: Rewriting any token in them makes the text false, so the whole file is
#: ``frozen``; the crosswalk resolves its citations.
SCHEME_FILES = re.compile(r"^(?:"
                          r"plans/archived/map-item-ids\.md"
                          r"|plans/active/item-id-per-kind\.md"
                          r"|docs/adr/0015-per-kind-item-ids\.md"
                          r"|logbook/\d{4}-\d{2}-\d{2}-(?:map-item-ids|item-id-per-kind)[^/]*\.md"
                          r"|map/bug-map-app-allocates-jb-ids-from-its-own-clone-and-collides\.md"
                          r")$")
#: The hand-converted fixtures, in one place for the audit. The first four
#: are named by plan P3 (their IDs assert parser behaviour); the other nine
#: build synthetic map repos whose IDs are invented (a lookup would give a
#: synthetic ``kind: idea`` Item a real bug's per-kind ID). P4 converts them
#: by hand; ``--apply`` leaves them and ``--verify`` lists their lines.
FIXTURE_FILES: Tuple[str, ...] = tuple("tests/sim/test_%s.py" % n for n in (
    "mapkit_ids", "sitting_sections", "mapkit_item_ids_cli", "night_shift_report",
    "night_shift_gate", "night_shift_candidate", "mapkit_similar", "night_shift_deploy",
    "night_shift_preflight", "question_tickets", "mapkit_docs", "night_shift_settings",
    "night_shift_runner"))
_FIXTURE_RE = "^(?:%s)$" % "|".join(re.escape(f) for f in FIXTURE_FILES)
#: The sweeper's own files: its patterns, tests and override keys name the
#: old form on purpose, and sweeping them would break the sweeper.
SWEEPER_FILES: Tuple[str, ...] = ("tools/id_sweep.py", "tools/id_sweep_overrides.tsv",
                                  "tests/sim/test_id_sweep.py")
_SWEEPER_RE = "^(?:%s)$" % "|".join(re.escape(f) for f in SWEEPER_FILES)
#: P4's own artefacts: the frozen crosswalk (every old ID, by design) and the cutover proof
#: scripts, which name old IDs and old file names in their own text. Both are tracked after
#: the renumber commit and so scanned; without this freeze --verify reads the crosswalk as
#: unrewritten citations and --apply refuses the proofs' placeholders.
_CUTOVER_RE = r"^(?:mapkit/legacy_ids\.csv|tools/probes/id_cutover/.*)$"
#: Rules whose whole-file freeze the summary lists by name (owner veto, stop point 3).
WHOLE_FILE_RULES = ("scheme-history-file", "sweeper-self", "cutover-self")
_PATH_EXT = r"\.(?:md|log|csv|json|ndjson|txt|png|jpg|html|yaml|yml|py|sh|mcap|db3|cpp|h)\b"


class Rule(NamedTuple):
    """One row of the rule table. ``token`` is full-matched against the token;
    ``path`` and ``line`` are searched; ``before`` is searched in the line text
    before the token, ``after`` is matched at the text after it, ``word`` is
    searched in the path-like word around it. ``not_path`` / ``not_before``
    must NOT be found. A ``None`` predicate holds."""
    name: str
    cls: str
    token: str
    path: Optional[str] = None
    line: Optional[str] = None
    before: Optional[str] = None
    after: Optional[str] = None
    word: Optional[str] = None
    not_path: Optional[str] = None
    not_before: Optional[str] = None
    pred: Optional[str] = None      # a name in _PREDICATES, called with the Token


_ID = r"JB-[1-9][0-9]*"
_LOOSE = r"jb-?[0-9]+[a-z0-9_]*|JB[0-9]+|Jb-?[0-9]+"  # lower or unhyphenated: never JB-7q
_PLACEHOLDER = (r"JB-(?:(?:<[^<>\s]+>|[A-Za-z])(?:-[A-Za-z0-9]+)*|\*|%d|\\d\+?|\{[^{}\s]*\})")
# A letter or ``<n>`` number part is a template, never a real ID: it is taken
# here (before the path rules), so ``JB-n-question.md`` is never frozen as a
# file; a digit number part (``jb-12-fix.md``) never matches and is frozen.
#: Where ``JB`` names the ID form itself (the ID code, its tests, the commands
#: and agent docs that teach it), so a bare ``JB``/``jb`` there is not the robot.
_ID_MACHINERY = (r"^(?:mapkit/|tools/night_shift|tests/sim/test_(?:night_shift|mapkit|question_tickets)"
                 r"|\.claude/commands/|docs/agents/)")
#: Test files that build synthetic map repos: their IDs are invented fixtures
#: (``JB-404``, ``id="JB-5"``), not citations, but the plan's fixture list does
#: not name them, so no rule takes them and the run lists them for the owner.
_SYNTHETIC = r"^tests/sim/test_(?:night_shift_|mapkit_|question_tickets)"
#: The ID code: a ````JB-12```` there is a format example, not a citation.
_ID_CODE = r"^(?:mapkit/|tools/night_shift)"

#: The bare-number tail of ``JB-1, 2, 3``: each number must end at a comma,
#: ``;``, ``)`` , `` and``/``or`` or the end of the line. Anything else (``3.``)
#: leaves the last number unrewritable, so the whole token is an error.
_LIST_TAIL = r"(?:,\s*[1-9][0-9]{0,2}(?=[,;)]|\s+(?:and|or)\b|\s*$))+"

_LIST_BAD = (r"(?:,\s*[1-9][0-9]{0,2}(?=[,;)]|\s+(?:and|or)\b|\s*$))*"
             r",\s*[1-9][0-9]{0,2}(?:\.(?:\s|$)|\s+(?:and|or)\s+[1-9])")

#: Ordered: the first rule whose predicates all hold wins.
RULES: Tuple[Rule, ...] = (
    # --- an Item's own frontmatter ID
    # (first, so it holds even inside a whole-file freeze)
    Rule("frontmatter-id", "citation", _ID, path=r"^map/[^/]+\.md$", line=r"^id:\s*JB-"),
    # --- the sweeper's own source, tests and overrides
    Rule("sweeper-self", "frozen", r"(?s).*", path=_SWEEPER_RE),
    # --- P4's crosswalk and cutover proofs (:data:`_CUTOVER_RE`)
    Rule("cutover-self", "frozen", r"(?s).*", path=_CUTOVER_RE),
    # --- hand-converted fixtures (:data:`FIXTURE_FILES`)
    Rule("fixture-file", "fixture", r"(?s).*", path=_FIXTURE_RE),
    # --- JB meaning Jugglebot: identifiers (no Item number in them)
    Rule("identifier-prefix", "abbreviation", r"(?i:jb)_[A-Za-z_][A-Za-z0-9_]*"),
    Rule("identifier-suffix", "abbreviation", r"(?i:jb)(?:_[A-Za-z0-9_]*)?", before=r"[A-Za-z0-9]_$"),
    # --- the ID code's own dual-form literals (P5 retires them): see _code_literal
    Rule("code-literal", "code_literal", r"(?s).*", path=_ID_CODE, pred="code_literal"),
    # --- scheme-history text (whole files)
    Rule("scheme-history-file", "frozen", r"(?s).*", path=SCHEME_FILES.pattern),
    # --- immutable references
    Rule("logbook-entry-trailer", "frozen", r"(?s).*", line=r"^\s*Logbook-Entry:"),
    Rule("plant-id-key", "frozen", r"(?i:jb)-?[0-9]+(?:-[A-Za-z0-9<>]+)+", line=r"plant_id|temp/learn/"),
    Rule("map-stem", "frozen", r"jb-?[0-9]*(?:-[A-Za-z0-9]+)*",
         before=r"(?:^|[^A-Za-z0-9])(?:bug|feature|idea|pathway|ticket)-(?:[a-z0-9]+-)*$"),
    # --- templates
    Rule("placeholder", "placeholder", _PLACEHOLDER),
    # --- night-shift branch names (a templated one is a placeholder above)
    Rule("night-shift-branch", "branch", _ID, before=r"night-shift/[^\s/]*-$", not_path=_SYNTHETIC),
    # --- real files and directories on disk
    Rule("path-with-extension", "frozen", r"(?s).*", word=r"(?i)jb[^/\s]*" + _PATH_EXT),
    Rule("path-under-temp-or-logbook", "frozen", r"(?s).*", word=r"(?:^|/)(?:temp|logbook|map)/"),
    Rule("filename-slug", "frozen", r"jb-?[0-9]+(?:-[a-z0-9]+)+"),
    Rule("citation-after-pre-post", "citation", _ID, before=r"(?:^|[^A-Za-z0-9-])(?:pre|post|non)-$",
         not_path=_SYNTHETIC),
    # an Attempt label / launch-log directory such as ``Higher-throws-with-JB-144-update``
    Rule("inside-hyphenated-name", "frozen", _ID + r"(?:-[A-Za-z0-9]+)*", before=r"[A-Za-z]-$",
         not_path=_SYNTHETIC),
    # --- lists and ranges (expanded by grammar, every instance reviewed)
    Rule("list-slash", "list", _ID + r"(?:/[0-9]+)+", not_path=_SYNTHETIC),
    # a bare ", <n>" tail the list rule cannot take whole (``JB-1, 2, 3.``,
    # ``JB-1, 2 and 3``): refused, never a partial rewrite
    Rule("list-tail-unrewritable", "ambiguous", _ID, after=_LIST_BAD, not_path=_SYNTHETIC),
    Rule("list-comma-tail", "list", _ID, after=_LIST_TAIL, not_path=_SYNTHETIC),
    Rule("range-dots", "range", _ID + r"(?:\.\.|–)[0-9]+", not_path=_SYNTHETIC),
    Rule("range-open", "range", _ID, after=r"(?:\.\.|–|\s+(?:to|through)\s+)JB-[0-9]", not_path=_SYNTHETIC),
    Rule("range-close", "range", _ID, before=r"JB-[0-9]+(?:\.\.|–|\s+(?:to|through)\s+)$",
         not_path=_SYNTHETIC),
    # --- plain citations (a hyphen-word suffix such as ``JB-151-style`` keeps its ID)
    Rule("citation", "citation", _ID + r"(?:-[a-z]+)?", not_path=_SYNTHETIC + "|" + _ID_CODE),
    Rule("citation-in-id-code", "citation", _ID, path=_ID_CODE, not_before=r"``$"),
    # --- loose forms: jb12, jb-12 in prose, test_jb21_*, jb75_keep_fresh
    Rule("loose", "loose", _LOOSE, not_path=_SYNTHETIC),
    # --- JB meaning Jugglebot in prose and code, and accidental matches
    Rule("robot-word-hyphen", "abbreviation", r"JB-(?:side|thrown)"),
    Rule("kicad-footprint-cache", "abbreviation", r"JB", path=r"(?:^|/)fp-info-cache$"),
    # ``JB ids``, ``a JB- prefix``, ``the named JB item``: the old scheme as the
    # subject (plan § 2.2, scheme-history text). In the ID machinery the same
    # words are live instructions that P5 rewrites by hand, so no rule there.
    Rule("scheme-word", "frozen", r"(?i:jb)", not_path=_ID_MACHINERY, not_before=r"/$",
         after=r"-(?=[\s`'\"])|\s*(?:IDs?|ids?|items?|prefix|duplicates?)\b"),
    Rule("robot-name", "abbreviation", r"JB", not_path=_ID_MACHINERY,
         after=r"(?!-|\s*(?:IDs?|ids?|items?|prefix|inputs?|duplicates?|form)\b)"),
    Rule("lowercase-jb-bus-or-variable", "abbreviation", r"jb", not_path=_ID_MACHINERY),
)
_APOSTROPHE = re.compile(r"(?<=[A-Za-z0-9])'(?=[A-Za-z])")
_IDENT = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")


def _code_literal(tok: "Token") -> bool:
    """In the ID code (path checked by the rule), is the token a literal?

    True when it sits inside a double-backtick span, inside a quoted string
    or regex (triple quotes and apostrophes ignored, so docstring prose is
    not a string), or is a bare identifier outside a ``#`` comment. A
    ``JB-<n>`` in a comment stays a citation even when quoted; only a
    ````...```` span makes a comment's ``JB-<n>`` a literal.
    """
    before = tok.text[:tok.col - 1]
    if before.count("``") % 2:
        return True
    s = before.replace('"""', "   ").replace("'''", "   ")
    quote, comment, i = None, False, 0
    while i < len(s):
        c = s[i]
        if quote:
            if c == "\\":
                i += 2
                continue
            if c == quote:
                quote = None
        elif c == "#":
            comment = True
        elif c == '"' or (c == "'" and not _APOSTROPHE.match(s, i)):
            quote = c
        i += 1
    if quote:
        return not (comment and re.fullmatch(_ID, tok.token))
    return not comment and _IDENT.fullmatch(tok.token) is not None


_PREDICATES = {"code_literal": _code_literal}
_COMPILED = [(r, re.compile(r.token), *(re.compile(p) if p else None
                                        for p in (r.path, r.line, r.before, r.after, r.word,
                                                  r.not_path, r.not_before)),
              _PREDICATES[r.pred] if r.pred else None)
             for r in RULES]


class Token(NamedTuple):
    file: str
    line: int
    col: int           # 1-based
    token: str
    word: str
    text: str          # the whole line
    end: int           # 0-based end of the token in the line


class Row(NamedTuple):
    file: str
    line: int
    col: int
    cls: str
    rule: str
    token: str
    before: str
    after: str
    context: str


# ------------------------------------------------------------------ finder

def skip_reason(root: str, rel: str) -> Optional[str]:
    """Why the finder never reads ``rel`` (None: it reads it). The ONE skip
    predicate: the census, the finder and T-R1 all use it."""
    p = os.path.join(root, rel)
    if os.path.islink(p):
        return "symlink"
    if not os.path.isfile(p):
        return "not-a-file"
    if rel.lower().endswith(SKIP_SUFFIXES):
        return "suffix"
    with open(p, "rb") as fh:
        if b"\0" in fh.read():
            return "binary"
    return None


def scan_files(root: str) -> Tuple[List[str], List[Tuple[str, str]]]:
    """(files read, [(path, reason)] skipped) of ``git ls-files``, both sorted."""
    out = subprocess.run(["git", "ls-files", "-z"], cwd=root, check=True,
                         capture_output=True).stdout.decode("utf-8", "surrogateescape")
    keep: List[str] = []
    skipped: List[Tuple[str, str]] = []
    for rel in filter(None, out.split("\0")):
        why = skip_reason(root, rel)
        if why:
            skipped.append((rel, why))
        else:
            keep.append(rel)
    return sorted(keep), sorted(skipped)


def tracked_text_files(root: str) -> List[str]:
    """``git ls-files`` minus symlinks, :data:`SKIP_SUFFIXES` and null-byte files."""
    return scan_files(root)[0]


def _lines(root: str, rel: str) -> List[str]:
    with open(os.path.join(root, rel), "rb") as fh:
        return fh.read().decode("utf-8", "replace").split("\n")


def _word(line: str, start: int, end: int) -> str:
    a, b = start, end
    while a > 0 and _WORDCH.match(line[a - 1]):
        a -= 1
    while b < len(line) and _WORDCH.match(line[b]):
        b += 1
    return line[a:b].rstrip(".")


def line_tokens(rel: str, n: int, line: str) -> Tuple[List[Token], List[Token]]:
    """(broad-pattern tokens, per-kind tokens) of one line, in column order."""
    broad: List[Token] = []
    kinds: List[Token] = []
    for m in PATTERN.finditer(line):
        core = CORE.match(line, m.start())
        end = core.end() if core else m.end()
        broad.append(Token(rel, n, m.start() + 1, line[m.start():end],
                           _word(line, m.start(), end), line, end))
    for m in PER_KIND_PATTERN.finditer(line):
        kinds.append(Token(rel, n, m.start() + 1, m.group(0),
                           _word(line, m.start(), m.end()), line, m.end()))
    return broad, kinds


def find_tokens(root: str) -> Tuple[List[Token], List[Token]]:
    """(broad-pattern tokens, per-kind tokens), each ordered by (file, line, col)."""
    broad: List[Token] = []
    kinds: List[Token] = []
    for rel in tracked_text_files(root):
        for n, line in enumerate(_lines(root, rel), 1):
            b, k = line_tokens(rel, n, line)
            broad += b
            kinds += k
    return broad, kinds


# -------------------------------------------------------------- classifier

def match_rule(tok: Token) -> Optional[Rule]:
    """The first rule of :data:`RULES` that takes ``tok``, else None."""
    before, after = tok.text[:tok.col - 1], tok.text[tok.end:]
    for rule, t, path, line, bef, aft, word, npath, nbef, pred in _COMPILED:
        if not t.fullmatch(tok.token):
            continue
        if (npath and npath.search(tok.file)) or (nbef and nbef.search(before)):
            continue
        if path and not path.search(tok.file):
            continue
        if line and not line.search(tok.text):
            continue
        if bef and not bef.search(before):
            continue
        if aft and not aft.match(after):
            continue
        if word and not word.search(tok.word):
            continue
        if pred and not pred(tok):
            continue
        return rule
    return None


def _span(tok: Token, rule: Rule) -> str:
    """The text the rewrite replaces: the token, plus a list's bare tail."""
    if rule.name == "list-comma-tail":
        m = re.compile(rule.after).match(tok.text, tok.end)
        return tok.token + m.group(0)
    if rule.cls == "citation":
        return re.match(_ID, tok.token).group(0)
    return tok.token


def _numbers(tok: Token, rule: Rule) -> List[int]:
    return [int(x) for x in _NUM.findall(_span(tok, rule))]


#: The retired global form. Since P5 ``mapkit.ids`` no longer knows it, so the sweeper
#: (frozen, kept until P6 for held branches that predate the renumber) carries its own.
OLD_PREFIX = "JB"
_OLD_CANON = re.compile(r"^%s-([1-9][0-9]*)$" % OLD_PREFIX)


def _old_number(value: object) -> Optional[int]:
    """The number of a canonical old-form ID string; None for anything else."""
    m = _OLD_CANON.match(value) if isinstance(value, str) else None
    return int(m.group(1)) if m else None


def known_numbers(root: str) -> Set[int]:
    """Item numbers of every ``JB-<n>`` frontmatter ID in ``map/``."""
    return {n for n in (_old_number(v) for v in ids.file_ids(root)) if n}


# --------------------------------------------------------------- overrides

#: Columns of the overrides file (tab-separated, ``csv`` quoting). ``line`` is
#: the exact line content (no newline) and with ``path`` is the key; ``old``
#: must occur exactly once in it and becomes ``new``; ``class`` (placeholder
#: or loose) is given to any token in ``old`` that no rule takes.
OVERRIDE_COLUMNS = ["path", "line", "old", "new", "class"]
OVERRIDE_CLASSES = frozenset({"placeholder", "loose"})
DEFAULT_OVERRIDES = os.path.join("tools", "id_sweep_overrides.tsv")


class Override(NamedTuple):
    path: str
    line: str
    old: str
    new: str
    cls: str


def read_overrides(path: Optional[str], explicit: bool = False
                   ) -> Tuple[List[Override], List[str]]:
    """(rows, errors) of an overrides file; a missing DEFAULT file is no rows,
    a missing ``explicit`` one (``--overrides PATH``) is an error."""
    if not path or not os.path.isfile(path):
        return [], (["%s: no such overrides file" % path] if explicit and path else [])
    rows: List[Override] = []
    errors: List[str] = []
    with open(path, newline="", encoding="utf-8") as fh:
        r = csv.DictReader(fh, delimiter="\t")
        missing = [c for c in OVERRIDE_COLUMNS if c not in (r.fieldnames or [])]
        if missing:
            return [], ["%s: overrides file lacks column(s) %s" % (path, ", ".join(missing))]
        for i, d in enumerate(r, 2):
            if None in d or any(d[c] is None for c in OVERRIDE_COLUMNS):
                errors.append("%s:%d: overrides row has %s columns, want %d" % (
                    path, i, "too many" if None in d else "too few", len(OVERRIDE_COLUMNS)))
                continue
            o = Override(*(d[c] or "" for c in OVERRIDE_COLUMNS))
            if o.cls not in OVERRIDE_CLASSES or not o.old or "\n" in o.new or "\n" in o.line:
                errors.append("%s:%d: bad override row (class must be placeholder|loose, old non-empty,"
                              " no newline)" % (path, i))
                continue
            rows.append(o)
    return rows, errors


#: (path, line text) -> [(override, live)]: ``live`` keys are the original
#: line; the others are the line as ``--apply`` leaves it (``new`` in place).
OverrideIndex = Dict[Tuple[str, str], List[Tuple[Override, bool]]]


def index_overrides(rows: List[Override]) -> OverrideIndex:
    idx: OverrideIndex = defaultdict(list)
    for o in rows:
        idx[(o.path, o.line)].append((o, True))
    return idx


def override_spans(idx: OverrideIndex, rel: str, text: str
                   ) -> Tuple[List[Tuple[int, int, Override, bool]], List[str]]:
    """(start, end, override, live) of every override on this line: ``old``
    on a live key, ``new`` on an applied one; each must occur exactly once."""
    spans, errors = [], []
    for o, live in idx.get((rel, text), ()):
        want = o.old if live else o.new
        if text.count(want) != 1:
            errors.append("%s: override text %r occurs %d times in its line (want once): %s"
                          % (rel, want, text.count(want), text.strip()[:120]))
            continue
        a = text.index(want)
        spans.append((a, a + len(want), o, live))
    return spans, errors


def _covering(tok: Token, spans) -> Optional[Tuple[int, int, Override, bool]]:
    for sp in spans:
        if sp[0] <= tok.col - 1 and tok.end <= sp[1]:
            return sp
    return None


def _partial(tok: Token, spans) -> bool:
    return any(sp[0] < tok.end and tok.col - 1 < sp[1] and _covering(tok, [sp]) is None
               for sp in spans)


def classify_token(tok: Token, known: Set[int], spans=()) -> Tuple[Optional[Row], Optional[str]]:
    """(row, None) or (None, error) for one broad-pattern token."""
    ctx = tok.text.strip()[:160]
    rule = match_rule(tok)
    if rule is not None and rule.cls in REWRITE_CLASSES:
        # an override row that covers a token a rewrite rule would take is the owner's
        # recorded "leave" (a synthetic or historical number): --verify must honour it,
        # as ``rewrite_line`` already does
        sp = _covering(tok, spans)
        if sp is not None:
            return Row(tok.file, tok.line, tok.col, sp[2].cls, "override", tok.token, tok.token,
                       "", ctx), None
    if rule is None:
        sp = _covering(tok, spans)
        if sp is not None:
            return Row(tok.file, tok.line, tok.col, sp[2].cls, "override", tok.token, tok.token,
                       "", ctx), None
        return None, "%s:%d: %s  [unclassified]  %s" % (tok.file, tok.line, tok.token, ctx)
    if rule.cls == "ambiguous":
        return None, "%s:%d: %s  [ambiguous list tail, rule %s: the last number cannot be rewritten]  %s" % (
            tok.file, tok.line, tok.token, rule.name, ctx)
    if rule.cls in REWRITE_CLASSES:
        nums = _numbers(tok, rule)
        unknown = [n for n in nums if n not in known]
        if rule.cls == "range":  # only the endpoints need exist
            unknown = [n for n in (nums[0], nums[-1]) if n not in known]
        if unknown:
            return None, "%s:%d: %s  [unknown number %s, rule %s]  %s" % (
                tok.file, tok.line, tok.token, ",".join(map(str, unknown)), rule.name, ctx)
    span = _span(tok, rule)
    return Row(tok.file, tok.line, tok.col, rule.cls, rule.name, tok.token, span,
               span if rule.cls in LEAVE_CLASSES else "", ctx), None


def classify(tokens: List[Token], kinds: List[Token], known: Set[int],
             overrides: Optional[OverrideIndex] = None) -> Tuple[List[Row], List[str]]:
    rows: List[Row] = []
    errors: List[str] = []
    for tok in tokens:
        spans = override_spans(overrides, tok.file, tok.text)[0] if overrides else ()
        row, err = classify_token(tok, known, spans)
        if err:
            errors.append(err)
        else:
            rows.append(row)
    for tok in kinds:
        cls = "option_text" if tok.file.startswith(ids.MAP_DIR + "/") else "per_kind"
        rows.append(Row(tok.file, tok.line, tok.col, cls, cls, tok.token, tok.token,
                        tok.token, tok.text.strip()[:160]))
    rows.sort(key=lambda r: (r.file, r.line, r.col))
    return rows, errors


# ------------------------------------------------------------------ census

COLUMNS = ["file", "line", "col", "class", "rule", "token", "before", "after", "context"]
RENUMBERED = frozenset(list(range(90, 94)) + list(range(96, 100)) + [118, 119])
NEAR_CAP = 8


@contextlib.contextmanager
def _lowered_caps(by: int):
    """mapkit's own readability measure, with both caps lowered by ``by``."""
    old = mitems.BLOCK_CAP, mitems.PROGRESS_CAP
    mitems.BLOCK_CAP, mitems.PROGRESS_CAP = old[0] - by, old[1] - by
    try:
        yield
    finally:
        mitems.BLOCK_CAP, mitems.PROGRESS_CAP = old


_VIOL = re.compile(r"^(?:Progress line|block) of (\d+) chars \(cap (\d+)\): (.*)\.\.\.$", re.S)


def near_cap_lines(root: str, rows: List[Row]) -> List[Tuple[str, int, int, int, str]]:
    """(file, line, length, cap, text) for every ``map/`` line within
    :data:`NEAR_CAP` characters of a readability cap that holds a token."""
    by_file: Dict[str, Set[int]] = defaultdict(set)
    for r in rows:
        if r.file.startswith(ids.MAP_DIR + "/") and r.file.endswith(".md"):
            by_file[r.file].add(r.line)
    out = []
    for rel in sorted(by_file):
        lines = _lines(root, rel)
        try:
            body_start = lines.index("---", 1) + 1
        except ValueError:
            continue
        body = "\n".join(lines[body_start:])
        with _lowered_caps(NEAR_CAP):
            viol = mitems.readability_violations(body)
        for v in viol:
            m = _VIOL.match(v)
            if not m:
                continue
            length, prefix = int(m.group(1)), m.group(3)
            cap = int(m.group(2)) + NEAR_CAP
            for i in range(body_start, len(lines)):
                s = lines[i].strip()
                if s and (s.startswith(prefix) or prefix.startswith(s)):
                    j = i  # the block runs to the next blank line, heading or list item
                    while True:
                        if j + 1 in by_file[rel]:
                            out.append((rel, j + 1, length, cap, lines[j].strip()[:120]))
                        j += 1
                        if (j >= len(lines) or not lines[j].strip()
                                or re.match(r"^\s*(?:[-*+]|\d+[.)])\s+|^#", lines[j])):
                            break
                    break
    return sorted(set(out))


def _byte_match(root: str, rel: str) -> str:
    """yes/no: does the skipped file hold a broad-pattern match in its bytes."""
    p = os.path.join(root, rel)
    if os.path.islink(p) or not os.path.isfile(p):
        return "n/a"
    with open(p, "rb") as fh:
        return "yes" if PATTERN.search(fh.read().decode("utf-8", "replace")) else "no"


def _top(path: str) -> str:
    return path.split("/", 1)[0] if "/" in path else "(root)"


def write_census(root: str, out_dir: str, rows: List[Row], raw: int, raw_kind: int,
                 skipped: Optional[List[Tuple[str, str]]] = None) -> None:
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "census.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(COLUMNS)
        for r in rows:
            w.writerow(list(r))
    skipped = skipped or []
    with open(os.path.join(out_dir, "skipped.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["path", "reason", "jb_byte_match"])
        for rel, why in skipped:
            w.writerow([rel, why, _byte_match(root, rel)])
    broad = [r for r in rows if r.cls not in PER_KIND_CLASSES]
    classes = sorted({r.cls for r in rows})
    tops = sorted({_top(r.file) for r in rows})
    grid = Counter((r.cls, _top(r.file)) for r in rows)
    md = ["# ID sweep census", "",
          "Raw broad-pattern token count: **%d** (classified %d, unclassified 0)." % (raw, len(broad)),
          "Per-kind tokens (`%s`): **%d**, counted apart." % (PER_KIND_PATTERN.pattern, raw_kind), "",
          "## Class totals", "", "| class | tokens |", "|---|---|"]
    md += ["| %s | %d |" % (c, sum(1 for r in rows if r.cls == c)) for c in classes]
    by_reason = Counter(why for _, why in skipped)
    hits = [rel for rel, _ in skipped if _byte_match(root, rel) == "yes"]
    md += ["", "## Skipped files (never read)", "",
           "%d files, listed in skipped.csv; %d of them have a jb byte match." % (len(skipped), len(hits))]
    md += ["- %s: %d" % (k, v) for k, v in sorted(by_reason.items())]
    md += ["- with a byte match: " + ", ".join("`%s`" % h for h in hits)] if hits else []
    md += ["", "## Rule totals", "", "| rule | class | tokens |", "|---|---|---|"]
    rule_n = Counter((r.rule, r.cls) for r in rows)
    md += ["| %s | %s | %d |" % (k[0], k[1], n) for k, n in sorted(rule_n.items())]
    md += ["", "## Class by top-level directory", "",
           "| class | " + " | ".join(tops) + " |", "|---|" + "---|" * len(tops)]
    md += ["| %s | %s |" % (c, " | ".join(str(grid[(c, t)] or "") for t in tops)) for c in classes]
    md += ["", "## Renumber review: every token naming JB-90..93, JB-96..99, JB-118, JB-119", "",
           "| file:line | class | rule | token | context |", "|---|---|---|---|---|"]
    for r in rows:
        if r.cls in PER_KIND_CLASSES:
            continue
        nums = {int(x) for x in _NUM.findall(r.before)} if re.match(r"(?i)jb-", r.token) else set()
        if nums & RENUMBERED:
            md.append("| %s:%d | %s | %s | `%s` | %s |" % (
                r.file, r.line, r.cls, r.rule, r.before, r.context[:100].replace("|", "\\|")))
    md += ["", "## map/ lines within %d characters of a readability cap that hold a token" % NEAR_CAP, "",
           "| file:line | length | cap | text |", "|---|---|---|---|"]
    md += ["| %s:%d | %d | %d | %s |" % (f, n, ln, cap, t.replace("|", "\\|"))
           for f, n, ln, cap, t in near_cap_lines(root, rows)]
    md += ["", "## Whole-file freezes (owner veto at stop point 3)", "",
           "Every token in these files is left; rules %s." % ", ".join(WHOLE_FILE_RULES), ""]
    wf = Counter((r.file, r.rule) for r in rows if r.rule in WHOLE_FILE_RULES)
    md += ["- `%s`: %d tokens (%s)" % (f, n, rule) for (f, rule), n in sorted(wf.items())]
    md += ["", "## code_literal: left at P4, retired by P5", ""]
    md += ["- %s:%d `%s`: %s" % (r.file, r.line, r.token, r.context[:100])
           for r in rows if r.cls == "code_literal"]
    md += ["", "## Hand-converted test fixtures", ""]
    fx = defaultdict(list)
    for r in rows:
        if r.cls == "fixture":
            fx[r.file].append(r.line)
    for f in sorted(fx):
        md.append("- `%s`: %d tokens on lines %s" % (f, len(fx[f]), ", ".join(map(str, sorted(set(fx[f]))))))
    with open(os.path.join(out_dir, "summary.md"), "w") as fh:
        fh.write("\n".join(md) + "\n")


def census(root: str, out_dir: str, overrides_path: Optional[str] = None,
           explicit: bool = False) -> int:
    ovr, errors = read_overrides(overrides_path, explicit)
    if errors:
        for e in errors:
            print(e, file=sys.stderr)
        return 1
    tokens, kinds = find_tokens(root)
    rows, errors = classify(tokens, kinds, known_numbers(root), index_overrides(ovr))
    if errors:
        for e in errors:
            print(e, file=sys.stderr)
        print("id_sweep: %d of %d tokens refused; nothing written" % (len(errors), len(tokens)),
              file=sys.stderr)
        return 1
    skipped = scan_files(root)[1]
    write_census(root, out_dir, rows, len(tokens), len(kinds), skipped)
    print("id_sweep: %d tokens classified, %d per-kind; %d files skipped (%s); census in %s" % (
        len(tokens), len(kinds), len(skipped),
        ", ".join("%s %d" % kv for kv in sorted(Counter(w for _, w in skipped).items())) or "none",
        out_dir))
    return 0


# ------------------------------------------------------------ crosswalk

CROSSWALK_COLUMNS = ["old_id", "new_id", "kind", "stem"]


def read_crosswalk(path: str) -> Tuple[Dict[int, str], List[str]]:
    """({old number: new ID}, errors). Extra columns are accepted; a missing
    one, a non-canonical ID, a prefix that is not its kind's, or a duplicate
    is an error."""
    xw: Dict[int, str] = {}
    errors: List[str] = []
    if not os.path.isfile(path):
        return {}, ["%s: no such crosswalk" % path]
    seen_new: Dict[str, str] = {}
    with open(path, newline="", encoding="utf-8") as fh:
        r = csv.DictReader(fh)
        missing = [c for c in CROSSWALK_COLUMNS if c not in (r.fieldnames or [])]
        if missing:
            return {}, ["%s: crosswalk lacks column(s) %s" % (path, ", ".join(missing))]
        for i, d in enumerate(r, 2):
            old, new, kind = d["old_id"] or "", d["new_id"] or "", d["kind"] or ""
            n = _old_number(old)
            sp = ids.split_ref(new)
            if n is None:
                errors.append("%s:%d: old_id %r is not a canonical JB-<n>" % (path, i, old))
            elif sp is None or sp[0] == OLD_PREFIX or ids.PREFIXES.get(kind) != sp[0]:
                errors.append("%s:%d: new_id %r is not a canonical %s ID" % (path, i, new, kind or "?"))
            elif n in xw or new in seen_new:
                errors.append("%s:%d: duplicate %s" % (path, i, old if n in xw else new))
            else:
                xw[n] = new
                seen_new[new] = old
    return xw, errors


# ----------------------------------------------------------------- apply

class Edit(NamedTuple):
    file: str
    line: int
    col: int           # 1-based start of the replaced span
    cls: str
    rule: str
    before: str
    after: str


_RANGE_CLOSE = re.compile(r"(\.\.|–|\s+(?:to|through)\s+)JB-([0-9]+)")
_LIST_NUM = re.compile(r"(?:JB-)?([0-9]+)")


def _expand(a: int, b: int, sep: str, full_close: bool, xw: Dict[int, str],
            gaps: List[str], where: str) -> str:
    """A range's new text: the same-kind contiguous case keeps the range form
    (``BUG-5..8``, ``BUG-5 to BUG-8``); anything else becomes a comma list."""
    if b <= a:
        raise ValueError("range end %d is not above its start %d" % (b, a))
    nums = [k for k in range(a, b + 1) if k in xw]
    skipped = [k for k in range(a, b + 1) if k not in xw]
    if skipped:
        gaps.append("%s: JB-%d..%d has no Item for %s (skipped)" % (
            where, a, b, ", ".join(map(str, skipped))))
    new = [xw[k] for k in nums]
    sp = [ids.split_ref(x) for x in new]
    if (len({p for p, _ in sp}) == 1
            and [k for _, k in sp] == list(range(sp[0][1], sp[0][1] + len(sp)))):
        return new[0] + sep + (new[-1] if full_close else str(sp[-1][1]))
    return ", ".join(new)


def rewrite_line(rel: str, n: int, text: str, xw: Dict[int, str], idx: OverrideIndex,
                 edits: List[Edit], gaps: List[str]) -> Tuple[str, List[str]]:
    """(new text, errors) for one line. Every edit is planned on the original
    text, checked for overlap, then applied right to left; the result is
    never re-scanned in this run."""
    errors: List[str] = []
    spans, errs = override_spans(idx, rel, text)
    errors += errs
    plan: List[Tuple[int, int, str, str, str]] = []   # (start, end, new, cls, rule)
    covered_close: Set[int] = set()
    closes: List[Token] = []
    where = "%s:%d" % (rel, n)
    for tok in line_tokens(rel, n, text)[0]:
        row, err = classify_token(tok, set(xw), spans)
        if err:
            errors.append(err)
            continue
        if _partial(tok, spans):
            errors.append("%s: %s  [partly inside an override's span]" % (where, tok.token))
            continue
        if _covering(tok, spans) is not None or row.cls in UNTOUCHED_CLASSES \
                or row.cls in PER_KIND_CLASSES:
            continue
        start = tok.col - 1
        try:
            if row.cls in ("citation", "branch"):
                new = xw[int(_NUM.search(row.before).group(0))]
                plan.append((start, start + len(row.before), new, row.cls, row.rule))
            elif row.cls == "list":
                new = _LIST_NUM.sub(lambda m: xw[int(m.group(1))], row.before)
                plan.append((start, start + len(row.before), new, row.cls, row.rule))
            elif row.rule == "range-dots":
                a, b = map(int, _NUM.findall(row.before))
                sep = row.before[len("JB-%d" % a):-len(str(b))]
                new = _expand(a, b, sep, False, xw, gaps, where)
                plan.append((start, start + len(row.before), new, row.cls, row.rule))
            elif row.rule == "range-open":
                m = _RANGE_CLOSE.match(text, tok.end)
                a, b = int(_NUM.search(tok.token).group(0)), int(m.group(2))
                new = _expand(a, b, m.group(1), True, xw, gaps, where)
                plan.append((start, m.end(), new, row.cls, row.rule))
                covered_close.add(m.end() - len("JB-") - len(m.group(2)))
            elif row.rule == "range-close":
                closes.append(tok)
            else:                                       # placeholder / loose
                errors.append("%s: %s  [%s, no override row covers it]  %s" % (
                    where, tok.token, row.cls, text.strip()[:120]))
        except (KeyError, ValueError) as e:
            errors.append("%s: %s  [%s: %s]" % (where, tok.token, row.rule, e))
    for tok in closes:
        if tok.col - 1 not in covered_close:
            errors.append("%s: %s  [range end with no range start]" % (where, tok.token))
    for a, b, o, live in spans:
        if live:
            plan.append((a, b, o.new, o.cls, "override"))
    plan.sort()
    for p, q in zip(plan, plan[1:]):
        if q[0] < p[1]:
            errors.append("%s: overlapping rewrites %r and %r" % (where, text[p[0]:p[1]], text[q[0]:q[1]]))
    if errors:
        return text, errors
    out = text
    for a, b, new, cls, rule in reversed(plan):
        if "\n" in new or "\r" in new:
            errors.append("%s: rewrite of %r would add a line" % (where, text[a:b]))
            continue
        out = out[:a] + new + out[b:]
    for a, b, new, cls, rule in plan:
        edits.append(Edit(rel, n, a + 1, cls, rule, text[a:b], new))
    return (text, errors) if errors else (out, errors)


def _read_text(root: str, rel: str) -> Optional[str]:
    with open(os.path.join(root, rel), "rb") as fh:
        data = fh.read()
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        return None


class Plan(NamedTuple):
    new_text: Dict[str, str]          # rel -> rewritten text (changed files only)
    edits: List[Edit]
    gaps: List[str]
    applied_overrides: List[Override]
    errors: List[str]


def plan_apply(root: str, xw: Dict[int, str], overrides: List[Override]) -> Plan:
    """Everything ``--apply`` would write, computed in memory."""
    idx = index_overrides(overrides)
    edits: List[Edit] = []
    gaps: List[str] = []
    errors: List[str] = []
    applied: List[Override] = []
    files = tracked_text_files(root)
    texts: Dict[str, Optional[str]] = {}
    # An override whose key line is gone is applied when its post-image (the
    # key as this run would rewrite it) is present, and stale otherwise.
    by_path: Dict[str, List[Override]] = defaultdict(list)
    for o in overrides:
        by_path[o.path].append(o)
    for rel, rows in by_path.items():
        if rel not in files:
            errors.extend("%s: stale override key (file not tracked): %s" % (rel, o.line[:120])
                          for o in rows)
            continue
        texts[rel] = _read_text(root, rel)
        lines = set((texts[rel] or "").split("\n"))
        keys: Dict[str, List[Override]] = defaultdict(list)
        for o in rows:
            keys[o.line].append(o)
        for key, group in keys.items():   # every override on one key line rewrites together
            if key in lines:
                applied += group
                continue
            post, errs = rewrite_line(rel, 0, key, xw, index_overrides(group), [], [])
            if errs or post not in lines:
                errors.extend("%s: stale override key (neither the line nor its rewrite is in the "
                              "file): %s" % (rel, key[:120]) for _ in group)
            else:
                idx[(rel, post)].extend((o, False) for o in group)
    new_text: Dict[str, str] = {}
    for rel in files:
        text = texts[rel] if rel in texts else None
        if rel not in texts:
            with open(os.path.join(root, rel), "rb") as fh:
                raw = fh.read()
            if not PATTERN.search(raw.decode("utf-8", "replace")):
                continue
            text = _read_text(root, rel)
        if text is None:
            errors.append("%s: not valid UTF-8; cannot rewrite byte-exactly" % rel)
            continue
        lines = text.split("\n")
        out = []
        for n, line in enumerate(lines, 1):
            if not PATTERN.search(line):
                out.append(line)
                continue
            new, errs = rewrite_line(rel, n, line, xw, idx, edits, gaps)
            errors += errs
            out.append(new)
        if len(out) != len(lines):
            errors.append("%s: line count would change (%d -> %d)" % (rel, len(lines), len(out)))
        joined = "\n".join(out)
        if joined != text:
            new_text[rel] = joined
    return Plan(new_text, edits, gaps, applied, errors)


def _cap_breaks(rel: str, text: str) -> List[str]:
    """Readability-cap breaches (``test_item_bodies_are_readable``'s measure)."""
    lines = text.split("\n")
    try:
        body_start = lines.index("---", 1) + 1
    except ValueError:
        return []
    return ["%s: %s" % (rel, v) for v in mitems.readability_violations("\n".join(lines[body_start:]))]


def _write_apply_report(out_dir: str, plan: Plan, cap_breaks: List[str]) -> None:
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "apply.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(list(Edit._fields))
        w.writerows(list(e) for e in plan.edits)
    md = ["# ID sweep apply", "",
          "Files rewritten: **%d**. Spans rewritten: **%d**." % (len(plan.new_text), len(plan.edits)), "",
          "| class | spans |", "|---|---|"]
    md += ["| %s | %d |" % (c, n) for c, n in sorted(Counter(e.cls for e in plan.edits).items())]
    md += ["", "## List and range expansions (every instance)", ""]
    md += ["- %s:%d `%s` -> `%s`" % (e.file, e.line, e.before, e.after)
           for e in plan.edits if e.cls in ("list", "range")]
    md += ["", "## Overrides applied", ""]
    md += ["- %s:%d `%s` -> `%s`" % (e.file, e.line, e.before, e.after)
           for e in plan.edits if e.rule == "override"]
    md += ["", "## Range numbers with no Item (skipped)", ""] + ["- " + g for g in plan.gaps]
    md += ["", "## map/ lines over a readability cap after rewriting (fix by hand before the commit)", ""]
    md += ["- " + c for c in cap_breaks]
    with open(os.path.join(out_dir, "apply-summary.md"), "w") as fh:
        fh.write("\n".join(md) + "\n")


def _load_inputs(crosswalk: str, overrides_path: Optional[str], explicit: bool = False):
    xw, errors = read_crosswalk(crosswalk)
    ovr, errs = read_overrides(overrides_path, explicit)
    return xw, ovr, errors + errs


def _refuse(errors: List[str], what: str) -> int:
    for e in errors:
        print(e, file=sys.stderr)
    print("id_sweep: %s refused: %d error(s); nothing written" % (what, len(errors)), file=sys.stderr)
    return 1


def apply(root: str, crosswalk: str, out_dir: str, overrides_path: Optional[str],
          explicit: bool = False, dry_run: bool = False) -> int:
    xw, ovr, errors = _load_inputs(crosswalk, overrides_path, explicit)
    what = "--apply --dry-run" if dry_run else "--apply"
    if errors:
        return _refuse(errors, what)
    plan = plan_apply(root, xw, ovr)
    if plan.errors:
        return _refuse(plan.errors, what)
    if not dry_run:
        for rel, text in sorted(plan.new_text.items()):
            with open(os.path.join(root, rel), "wb") as fh:
                fh.write(text.encode("utf-8"))
    caps = [c for rel, text in sorted(plan.new_text.items())
            if rel.startswith(ids.MAP_DIR + "/") and rel.endswith(".md")
            for c in _cap_breaks(rel, text)]
    _write_apply_report(out_dir, plan, caps)
    print("id_sweep: %s rewrote %d spans in %d files; %d map/ cap break(s); report in %s%s"
          % ("--apply", len(plan.edits), len(plan.new_text), len(caps), out_dir,
             " (dry run: the tree is unchanged)" if dry_run else ""))
    return 0


def verify(root: str, crosswalk: str, overrides_path: Optional[str], explicit: bool = False) -> int:
    """Nothing rewritable left in the old form, and a second apply is a no-op."""
    xw, ovr, errors = _load_inputs(crosswalk, overrides_path, explicit)
    if errors:
        return _refuse(errors, "--verify")
    tokens, kinds = find_tokens(root)
    rows, errors = classify(tokens, kinds, set(xw), index_overrides(ovr))
    fails = list(errors)
    fails += ["%s:%d: %s  [%s still in the old form, rule %s]  %s" % (
        r.file, r.line, r.token, r.cls, r.rule, r.context) for r in rows if r.cls in REWRITE_CLASSES]
    plan = plan_apply(root, xw, ovr)
    fails += ["second --apply: " + e for e in plan.errors]
    fails += ["%s:%d: a second --apply would rewrite %r -> %r" % (e.file, e.line, e.before, e.after)
              for e in plan.edits if e.before != e.after]
    fails += ["%s: a second --apply would change this file" % rel for rel in sorted(plan.new_text)]
    left = Counter(r.cls for r in rows if r.cls in UNTOUCHED_CLASSES | OVERRIDE_CLASSES)
    print("id_sweep --verify: old-form tokens left by class: %s" % (
        ", ".join("%s %d" % kv for kv in sorted(left.items())) or "none"))
    fx = sorted({(r.file, r.line) for r in rows
                 if r.cls == "fixture" and re.match(r"(?i)jb-?[0-9]", r.token)})
    print("Fixture lines still carrying a JB-<n> (hand-convert at P4): %d" % len(fx))
    for f, n in fx:
        print("  %s:%d" % (f, n))
    if fails:
        for f in fails:
            print(f, file=sys.stderr)
        print("id_sweep --verify: FAIL (%d)" % len(fails), file=sys.stderr)
        return 1
    print("id_sweep --verify: OK")
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--census", action="store_true")
    mode.add_argument("--apply", action="store_true")
    mode.add_argument("--verify", action="store_true")
    ap.add_argument("--dry-run", action="store_true",
                    help="with --apply: plan in memory, write nothing to the tree (report only);"
                         " accepted with --verify, which never writes")
    ap.add_argument("--root", default=_REPO)
    ap.add_argument("--out", default=None, help="default <root>/temp/id-sweep")
    ap.add_argument("--crosswalk", default=None, help="old_id,new_id,kind,stem CSV (--apply/--verify)")
    ap.add_argument("--overrides", default=None, help="default <root>/" + DEFAULT_OVERRIDES)
    a = ap.parse_args(argv)
    root = os.path.abspath(a.root)
    out = a.out or os.path.join(root, "temp", "id-sweep")
    ovr = a.overrides or os.path.join(root, DEFAULT_OVERRIDES)
    explicit = a.overrides is not None
    if a.dry_run and a.census:
        ap.error("--dry-run goes with --apply (or --verify)")
    if (a.apply or a.verify) and not a.crosswalk:
        ap.error("--apply and --verify need --crosswalk")
    if a.apply:
        return apply(root, a.crosswalk, out, ovr, explicit, a.dry_run)
    if a.verify:
        return verify(root, a.crosswalk, ovr, explicit)
    return census(root, out, ovr, explicit)


if __name__ == "__main__":
    sys.exit(main())
