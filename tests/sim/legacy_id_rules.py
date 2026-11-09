"""The classifier behind the no-old-Item-ID guard (``test_no_legacy_item_ids.py``).

The per-kind Item IDs replaced the global old form in one commit (plan
``item-id-per-kind.md``, P4). The tool that did the renumbering classified every
token before rewriting it; that classifier is what lets the guard tell an old
Item ID from ``JB`` meaning Jugglebot, so it outlived the tool and lives here,
beside its only reader. A test-side module on purpose: no robot code can import
it, and ``mapkit`` stays free of any knowledge of the old form.

:data:`PATTERN` is deliberately broad (any ``jb`` not followed by a letter, in
any case, so the lower-case, unhyphenated and padded forms a ``JB-\\d+`` grep
misses are found too). :func:`match_rule` gives each token the first row of the
ordered table :data:`RULES` whose predicates all hold; a token no row takes is
reported as unclassified. Nothing defaults to "leave" or to "rewrite".

=============== =============================================================
``citation``     an Item ID: a real old citation, never allowed outside the record
``branch``       a ``night-shift/`` branch carrying an old ID
``list``         ``JB-120/139``, ``JB-102, 166, 167``
``range``        ``JB-68..71``, ``JB-152 to JB-157``
``placeholder``  a template: ``JB-<n>``, ``JB-n``
``loose``        lower-case or unhyphenated: ``jb12``, ``jb-12`` in prose
``ambiguous``    a list tail no rewrite could take whole (``JB-1, 2, 3.``)
``fixture``      inside a test file listed in :data:`FIXTURE_FILES`
``code_literal`` a literal, regex or identifier in the ID code
``abbreviation`` ``JB`` meaning Jugglebot (``JB_*``, ``defer jb``)
``frozen``       filenames, stems, ``Logbook-Entry:``, ``plant_id``, real paths,
                 text whose subject is the old scheme
=============== =============================================================

The guard decides what each class is allowed to do. Every row of the table has a
planted token in that file's tests, and a row with none fails them.
"""
from __future__ import annotations

import os
import re
from typing import List, NamedTuple, Optional, Tuple


#: The broad pattern, the single source for the finder.
PATTERN = re.compile(r"(?i)(?<![a-z0-9])jb(?![a-z])")
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
                          r"|plans/(?:active|archived)/item-id-per-kind\.md"
                          r"|docs/adr/0015-per-kind-item-ids\.md"
                          r"|logbook/\d{4}-\d{2}-\d{2}-(?:map-item-ids|item-id-per-kind)[^/]*\.md"
                          r"|map/bug-map-app-allocates-jb-ids-from-its-own-clone-and-collides\.md"
                          r")$")
#: The test files converted by hand at the cutover: their IDs assert parser
#: behaviour or build synthetic map repos. Any token in one is class ``fixture``,
#: which the guard does not allow, so a converted file stays converted.
FIXTURE_FILES: Tuple[str, ...] = tuple("tests/sim/test_%s.py" % n for n in (
    "mapkit_ids", "sitting_sections", "mapkit_item_ids_cli", "night_shift_report",
    "night_shift_gate", "night_shift_candidate", "mapkit_similar", "night_shift_deploy",
    "night_shift_preflight", "question_tickets", "mapkit_docs", "night_shift_settings",
    "night_shift_runner"))
_FIXTURE_RE = "^(?:%s)$" % "|".join(re.escape(f) for f in FIXTURE_FILES)
#: The cutover's own artefacts: the frozen crosswalk (every old ID, by design) and the proof
#: scripts, which name old IDs and old file names in their own text.
_CUTOVER_RE = r"^(?:mapkit/legacy_ids\.csv|tools/probes/id_cutover/.*)$"
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
    # --- P4's crosswalk and cutover proofs (:data:`_CUTOVER_RE`)
    Rule("cutover-self", "frozen", r"(?s).*", path=_CUTOVER_RE),
    # --- hand-converted fixtures (:data:`FIXTURE_FILES`)
    Rule("fixture-file", "fixture", r"(?s).*", path=_FIXTURE_RE),
    # --- JB meaning Jugglebot: identifiers (no Item number in them)
    Rule("identifier-prefix", "abbreviation", r"(?i:jb)_[A-Za-z_][A-Za-z0-9_]*"),
    Rule("identifier-suffix", "abbreviation", r"(?i:jb)(?:_[A-Za-z0-9_]*)?", before=r"[A-Za-z0-9]_$"),
    # --- literals in the ID code: see _code_literal (a class the guard does not allow)
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
    # words would be live instructions, so no rule takes them there.
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

def skip_reason(root: str, rel: str) -> Optional[str]:
    """Why the guard never reads ``rel`` (None: it reads it)."""
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

def _word(line: str, start: int, end: int) -> str:
    a, b = start, end
    while a > 0 and _WORDCH.match(line[a - 1]):
        a -= 1
    while b < len(line) and _WORDCH.match(line[b]):
        b += 1
    return line[a:b].rstrip(".")


def line_tokens(rel: str, n: int, line: str) -> List[Token]:
    """The broad-pattern tokens of one line, in column order."""
    out: List[Token] = []
    for m in PATTERN.finditer(line):
        core = CORE.match(line, m.start())
        end = core.end() if core else m.end()
        out.append(Token(rel, n, m.start() + 1, line[m.start():end],
                         _word(line, m.start(), end), line, end))
    return out


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
