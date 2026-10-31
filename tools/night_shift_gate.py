#!/usr/bin/env python3
"""Night Shift merge gate: MERGE or HOLD for one held branch, decided by code.

Why: Night Shift works the board unattended, and this script is the single
enforcement point between that work and ``main`` (plan
``plans/active/night-shift.md`` § 2 "Merge gate rule", § 6). A false MERGE is
the one failure the design cannot survive, so every path fails closed: the
script exits 0 only on a proven MERGE, and any error, missing input or
unreadable file is a HOLD.

Usage:
  python tools/night_shift_gate.py --repo R --base origin/main \\
      --branch night-shift/<date>-JB-n --item JB-n \\
      --gate-log PATH --audit PATH [--waive c] [--json]

Output: ``MERGE`` (exit 0), or ``HOLD`` then one reason per line, each opening
with its condition letter in brackets (exit 1). ``--json`` prints
``{"verdict", "item", "branch", "tip", "reasons", "checked", "waived"}`` instead.

``--waive c`` is for ``/sitting-wrapup``: it re-runs the gate on a branch the
owner passed at a Sitting, with the night's gate log and audit (tip unchanged),
so a hardware pass overrides the hardware condition (c) and nothing else: in
particular not (g), the owner check, which is its own letter so that no Sitting
pass can clear an Item that a branch made to look workable (owner, 2026-10-31). (c)
is still evaluated and reported in ``checked`` and the reasons, but a failing
(c) does not block; the text verdict reads ``MERGE (c waived)``. Any other
letter is a usage error (HOLD). A ``gate error`` is never waived.

Preconditions ``[pre]`` (a failure here is HOLD and the conditions are not
evaluated): the branch and the base resolve; the branch contains the base and
is at least one commit ahead of it; ``--item`` parses as ``JB-<n>`` and exactly
one file under ``map/`` ON THE BRANCH carries that ``id:`` and loads as an Item;
exactly one file under ``map/`` at the BASE carries it too (a branch cannot
create the Item it merges).

The seven conditions, ALL evaluated so a HOLD lists every failing reason:
  (a) suite gate: the gate log's ``HEAD=<sha>`` line(s) equal the branch tip,
      any ``BRANCH=<name>`` line names the branch, and its LAST ``EXIT=<n>``
      line is ``EXIT=0``;
  (b) audit: the report's ``Audited HEAD: <sha>`` equals the branch tip and
      ``Behaviour-affecting findings open: <n>`` is present with n == 0;
  (c) hardware: the Item on the branch is rated ``hardware: none``, and the
      branch did not lower a ``hardware`` rating the base had;
  (d) radius: the Item on the branch is rated, not ``normative``, and the
      branch did not lower a ``radius`` rating the base had. A rating absent
      at the base may first appear on the branch (the orchestrator rating an
      unrated Item); one the base had cannot be talked down by the branch it
      would merge;
  (e) open question: no ticket on the branch with ``type: question`` that is
      not complete or dropped and whose ``parent`` names the Item (stem, slug
      or ID); a map file that cannot be parsed is a HOLD too;
  (f) paths: ``git diff -z --no-renames --name-only base...branch`` touches
      nothing in :data:`FORBIDDEN_PATHS`;
  (g) owner: the branch did not change a rated ``owner`` the base had (away
      from ``none``, or down to ``none``, the way a decision-gated Item would
      be made to look workable). Never waivable.

Input formats (written by ``.claude/commands/night-shift.md`` §§ 2e, 2f):
  gate log:      ``HEAD=<sha>`` / ``BRANCH=<name>`` lines first, the suite
                 output, then ``EXIT=<n>`` last;
  audit report:  the findings, ending with ``Audited HEAD: <sha>`` and
                 ``Behaviour-affecting findings open: <n>``.
A sha matches the tip when it is the full 40 characters or an unambiguous
prefix of at least 7.

The Item is read from the BRANCH with ``git cat-file``, never from the
working tree: the rating that merges is the rating on the branch.

Why the gate's own machinery is in :data:`FORBIDDEN_PATHS`: a branch that
edits the gate, its imports or its tests decides its own verdict. The
protocol additionally runs the gate from ``origin/main``'s copy, which is
only safe because ``mapkit/`` (imported by the gate) is frozen too.

A HOLD is never promoted to MERGE by anyone; change this file and its test
together or not at all.
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mapkit.frontmatter import FrontmatterError, split_frontmatter  # noqa: E402
from mapkit.ids import parse as parse_id  # noqa: E402
from mapkit.items import METRIC_ENUMS, Item, ItemError, load_item  # noqa: E402

#: Paths a Night Shift branch may never touch (plan § 2 "Merge gate rule" (f)),
#: matched with ``fnmatch.fnmatchcase`` against repo-root paths. ``*`` crosses
#: ``/``, so ``tools/night_shift*`` is a prefix and ``config/*.yaml`` is every
#: ``.yaml`` under ``config/`` at any depth (``config/**/*.yaml``); an entry
#: without ``*`` is one exact path (``docs/CLAUDE.md`` is not forbidden).
#: Beyond the plan's five, the safety layer itself is frozen: a branch that
#: edits the gate, the runner, the guard, the protocol, ``mapkit/`` (the gate
#: imports it) or their tests would decide its own verdict.
FORBIDDEN_PATHS: Tuple[str, ...] = (
    "Teensy_code_canbridge/*",
    "tools/night_shift*",
    "tools/lib/*",
    "tools/systemd/*",
    "tools/nightly_*",
    ".claude/*",
    "mapkit/*",
    "tests/sim/test_night_shift_*",
    "tests/sim/test_live_session_guard.py",
    "tests/sim/test_question_tickets.py",
    "tests/sim/test_sitting_*",
    "run_tests.sh",
    "pyproject.toml",
    "GLOSSARY.md",
    "CLAUDE.md",
    "DOCUMENTATION_GUIDE.md",
    "config/*.yaml",
)

#: Letters ``--waive`` accepts: only the hardware condition, which a Sitting pass overrides.
WAIVABLE: Tuple[str, ...] = ("c",)

CONDITIONS: Tuple[str, ...] = ("a", "b", "c", "d", "e", "f", "g")
MAP_DIR = "map"
MIN_SHA_PREFIX = 7
_QUESTION_CLOSED = ("complete", "dropped")

_HEAD_LINE = re.compile(r"^HEAD=(\S*)\s*$")
_BRANCH_LINE = re.compile(r"^BRANCH=(.*?)\s*$")
_EXIT_LINE = re.compile(r"^EXIT=(.*?)\s*$")
_AUDITED_LINE = re.compile(r"^Audited HEAD:\s*(\S*)\s*$")
_OPEN_LINE = re.compile(r"^Behaviour-affecting findings open:\s*(.*?)\s*$")
_HEX = re.compile(r"^[0-9a-fA-F]+$")


class PreconditionError(Exception):
    """A ``[pre]`` failure: the conditions cannot be evaluated."""


@dataclass
class GateResult:
    verdict: str  # "MERGE" or "HOLD"
    item: str
    branch: str
    tip: Optional[str]
    reasons: List[str] = field(default_factory=list)
    checked: Dict[str, bool] = field(default_factory=dict)
    waived: List[str] = field(default_factory=list)

    def to_json(self) -> Dict[str, object]:
        return {"verdict": self.verdict, "item": self.item, "branch": self.branch,
                "tip": self.tip, "reasons": list(self.reasons),
                "checked": dict(self.checked), "waived": list(self.waived)}

    def to_text(self) -> str:
        head = self.verdict
        if self.verdict == "MERGE" and self.waived:
            head = "MERGE (%s waived)" % ", ".join(self.waived)
        return "\n".join([head] + self.reasons)


# --- git ---------------------------------------------------------------------
def _git(repo: str, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(["git", "-C", repo] + list(args),
                          capture_output=True, text=True)


def _rev(repo: str, ref: str) -> Optional[str]:
    r = _git(repo, "rev-parse", "--verify", "--quiet", ref + "^{commit}")
    sha = r.stdout.strip()
    return sha if r.returncode == 0 and len(sha) == 40 else None


def _read_blobs(repo: str, branch: str, names: List[str]) -> Dict[str, Optional[str]]:
    """``{name: text or None}`` for ``branch:map/<name>``, via one ``cat-file --batch``."""
    if not names:
        return {}
    spec = "".join("%s:%s/%s\n" % (branch, MAP_DIR, n) for n in names)
    r = subprocess.run(["git", "-C", repo, "cat-file", "--batch"],
                       input=spec.encode("utf-8"), capture_output=True)
    if r.returncode != 0:
        raise PreconditionError("git cat-file failed: %s" % r.stderr.decode("utf-8", "replace").strip())
    out: Dict[str, Optional[str]] = {}
    data, pos = r.stdout, 0
    for name in names:
        nl = data.find(b"\n", pos)
        if nl < 0:
            raise PreconditionError("git cat-file output truncated at map/%s" % name)
        header = data[pos:nl].decode("utf-8", "replace").split()
        pos = nl + 1
        if len(header) != 3 or header[1] != "blob":
            out[name] = None
            continue
        size = int(header[2])
        out[name] = data[pos:pos + size].decode("utf-8", "replace")
        pos += size + 1
    return out


def _map_files(repo: str, branch: str) -> Dict[str, Optional[str]]:
    # -z: no C-quoting, so a non-ASCII name is not silently skipped.
    r = _git(repo, "ls-tree", "-z", "--name-only", branch, MAP_DIR + "/")
    if r.returncode != 0:
        raise PreconditionError("cannot list %s/ on %s: %s" % (MAP_DIR, branch, r.stderr.strip()))
    names = [p[len(MAP_DIR) + 1:] for p in r.stdout.split("\0")
             if p.startswith(MAP_DIR + "/") and p.endswith(".md")]
    return _read_blobs(repo, branch, names)


# --- preconditions -----------------------------------------------------------
def _find_item(files: Dict[str, Optional[str]], n: int, where: str = "on the branch") -> Tuple[str, str]:
    """``(name, text)`` of the one map file whose ``id:`` is JB-<n>."""
    matches = []
    for name, text in sorted(files.items()):
        if text is None:
            continue
        try:
            meta, _ = split_frontmatter(text)
        except FrontmatterError:
            continue
        v = meta.get("id")
        if isinstance(v, str) and parse_id(v) == n:
            matches.append((name, text))
    if not matches:
        raise PreconditionError("no map/ file %s carries id JB-%d" % (where, n))
    if len(matches) > 1:
        raise PreconditionError("id JB-%d is carried by %d files %s: %s"
                                % (n, len(matches), where, ", ".join(m[0] for m in matches)))
    return matches[0]


def _load_branch_item(name: str, text: str) -> Item:
    """Parse the branch copy with the mapkit loader (it needs a file of the same name)."""
    with tempfile.TemporaryDirectory(prefix="night_shift_gate_") as d:
        path = os.path.join(d, name)
        with open(path, "w", encoding="utf-8", newline="") as fh:
            fh.write(text)
        try:
            return load_item(path)
        except ItemError as exc:
            raise PreconditionError("map/%s on the branch does not load: %s"
                                    % (name, str(exc).replace(path, name)))


def _base_meta(repo: str, base_sha: str, n: int) -> Dict[str, object]:
    """The Item's frontmatter at the base; ``[pre] Item not on base`` when absent."""
    try:
        _, text = _find_item(_map_files(repo, base_sha), n, "at the base")
    except PreconditionError as exc:
        raise PreconditionError("Item not on base: %s" % exc)
    return split_frontmatter(text)[0]


def _preconditions(repo: str, base: str, branch: str, item: str) -> Tuple[str, str, Item, Dict[str, Optional[str]], Dict[str, object]]:
    """``(tip, base sha, Item, map files, base frontmatter)`` or raise :class:`PreconditionError`."""
    if not os.path.isdir(repo):
        raise PreconditionError("repo %r is not a directory" % repo)
    r = _git(repo, "rev-parse", "--git-dir")
    if r.returncode != 0:
        raise PreconditionError("repo %r is not a git repository" % repo)
    tip = _rev(repo, branch)
    if tip is None:
        raise PreconditionError("branch %r does not resolve" % branch)
    base_sha = _rev(repo, base)
    if base_sha is None:
        raise PreconditionError("base %r does not resolve" % base)
    if _git(repo, "merge-base", "--is-ancestor", base_sha, tip).returncode != 0:
        raise PreconditionError("branch %r does not contain base %r" % (branch, base))
    if tip == base_sha:
        raise PreconditionError("branch %r is not ahead of base %r" % (branch, base))
    n = parse_id(item)
    if n is None:
        raise PreconditionError("--item %r is not a JB-<n> ID" % item)
    files = _map_files(repo, tip)
    name, text = _find_item(files, n)
    it = _load_branch_item(name, text)
    return tip, base_sha, it, files, _base_meta(repo, base_sha, n)


# --- conditions --------------------------------------------------------------
def _sha_matches(repo: str, tip: str, candidate: str) -> bool:
    c = candidate.strip().lower()
    if len(c) < MIN_SHA_PREFIX or not _HEX.match(c) or not tip.startswith(c):
        return False
    return len(c) == 40 or _rev(repo, c) == tip  # a short prefix must be unambiguous


def _read(path: str) -> Optional[List[str]]:
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            return fh.read().splitlines()
    except OSError:
        return None


def _check_a(repo: str, tip: str, branch: str, gate_log: str) -> List[str]:
    lines = _read(gate_log)
    if lines is None:
        return ["[a] gate log %s missing or unreadable" % gate_log]
    out = []
    heads = [m.group(1) for m in (_HEAD_LINE.match(x) for x in lines) if m]
    if not heads:
        out.append("[a] gate log has no HEAD= line")
    for h in heads:
        if not _sha_matches(repo, tip, h):
            out.append("[a] gate log HEAD=%s is not the branch tip %s" % (h, tip))
    want = branch[len("refs/heads/"):] if branch.startswith("refs/heads/") else branch
    for m in (_BRANCH_LINE.match(x) for x in lines):
        if m and m.group(1) != want:
            out.append("[a] gate log BRANCH=%s is not %s" % (m.group(1), want))
    exits = [m.group(1) for m in (_EXIT_LINE.match(x) for x in lines) if m]
    if not exits:
        out.append("[a] gate log has no EXIT= line")
    elif exits[-1] != "0":
        out.append("[a] gate log EXIT=%s (expected 0)" % exits[-1])
    return out


def _check_b(repo: str, tip: str, audit: str) -> List[str]:
    lines = _read(audit)
    if lines is None:
        return ["[b] audit report %s missing or unreadable" % audit]
    out = []
    heads = [m.group(1) for m in (_AUDITED_LINE.match(x) for x in lines) if m]
    if not heads:
        out.append("[b] audit report has no 'Audited HEAD:' line")
    for h in heads:
        if not _sha_matches(repo, tip, h):
            out.append("[b] audit report Audited HEAD: %s is not the branch tip %s" % (h, tip))
    counts = [m.group(1) for m in (_OPEN_LINE.match(x) for x in lines) if m]
    if not counts:
        out.append("[b] audit report has no 'Behaviour-affecting findings open:' line")
    for c in counts:
        if c != "0":
            out.append("[b] behaviour-affecting findings open: %s (expected 0)" % c)
    return out


def _changed(letter: str, metric: str, base: object, branch: object) -> str:
    return "[%s] rating changed on branch: %s base=%s branch=%s" % (
        letter, metric, base, "unrated" if branch is None else branch)


def _lowered(letter: str, metric: str, base_meta: Dict[str, object], it: Item) -> List[str]:
    """A rating the base had that the branch lowered (or removed, or that cannot be ranked)."""
    old, new = base_meta.get(metric), it.meta.get(metric)
    if old is None or old == new:
        return []  # unrated at base: the branch may rate it
    ranks = METRIC_ENUMS[metric]  # rank order, lowest first
    if old not in ranks or new not in ranks or ranks.index(new) < ranks.index(old):
        return [_changed(letter, metric, old, new)]
    return []


def _check_c(it: Item, base_meta: Dict[str, object]) -> List[str]:
    out = []
    hw = it.meta.get("hardware")
    if hw is None:
        out.append("[c] hardware not rated")
    elif hw != "none":
        out.append("[c] hardware: %s (expected none)" % hw)
    out.extend(_lowered("c", "hardware", base_meta, it))
    return out


def _check_g(it: Item, base_meta: Dict[str, object]) -> List[str]:
    """Its own letter, not part of (c), so `--waive c` can never clear it."""
    old_owner, new_owner = base_meta.get("owner"), it.meta.get("owner")
    if old_owner is not None and new_owner != old_owner:
        # Any change of a rated owner: away from none, or down to none (the way
        # a decision-gated Item would be made to look workable).
        return [_changed("g", "owner", old_owner, new_owner)]
    return []


def _check_d(it: Item, base_meta: Dict[str, object]) -> List[str]:
    out = []
    radius = it.meta.get("radius")
    if radius is None:
        out.append("[d] radius not rated")
    elif radius == "normative":
        out.append("[d] radius: normative")
    out.extend(_lowered("d", "radius", base_meta, it))
    return out


def _item_names(it: Item) -> List[str]:
    names = ["%s-%s" % (it.kind, it.slug), it.slug]
    if isinstance(it.meta.get("id"), str):
        names.append(it.meta["id"].strip())
    return names


def _names_item(parent: object, it: Item, n: int) -> bool:
    if not isinstance(parent, str) or not parent.strip():
        return False
    p = parent.strip()
    if p.endswith(".md"):
        p = p[:-3]
    return p in _item_names(it) or parse_id(p) == n


def _check_e(it: Item, n: int, files: Dict[str, Optional[str]]) -> List[str]:
    out = []
    for name, text in sorted(files.items()):
        if text is None:
            out.append("[e] map/%s unreadable on the branch; cannot rule out an open question" % name)
            continue
        try:
            meta, _ = split_frontmatter(text)
        except FrontmatterError:
            out.append("[e] map/%s unparseable on the branch; cannot rule out an open question" % name)
            continue
        if meta.get("kind") != "ticket" and not name.startswith("ticket-"):
            continue
        if str(meta.get("type", "")).strip().lower() != "question":
            continue
        if str(meta.get("status", "")).strip().lower() in _QUESTION_CLOSED:
            continue
        if _names_item(meta.get("parent"), it, n):
            out.append("[e] open question ticket %s (map/%s)" % (meta.get("id") or name[:-3], name))
    return out


def _forbidden(path: str) -> bool:
    return any(fnmatch.fnmatchcase(path, entry) for entry in FORBIDDEN_PATHS)


def _check_f(repo: str, base_sha: str, tip: str) -> List[str]:
    # --no-renames: a rename lists both sides (a moved GLOSSARY.md would else
    # show only its new name); -z: no C-quoting of unusual names.
    r = _git(repo, "diff", "-z", "--no-renames", "--name-only", "%s...%s" % (base_sha, tip))
    if r.returncode != 0:
        return ["[f] git diff failed: %s" % r.stderr.strip()]
    return ["[f] touches forbidden path %s" % p
            for p in r.stdout.split("\0") if p and _forbidden(p)]


# --- verdict -----------------------------------------------------------------
def evaluate(repo: str, base: str, branch: str, item: str,
             gate_log: str, audit: str, waive: Tuple[str, ...] = ()) -> GateResult:
    """The gate verdict. MERGE only when the preconditions hold and every condition
    not in ``waive`` passes (``waive`` may hold only letters in :data:`WAIVABLE`)."""
    res = GateResult(verdict="HOLD", item=item, branch=branch, tip=None,
                     checked=dict((c, False) for c in CONDITIONS))
    bad = sorted(set(w for w in waive if w not in WAIVABLE))
    if bad:
        res.reasons = ["[pre] cannot waive %s (only %s)" % (", ".join(bad), ", ".join(WAIVABLE))]
        return res
    res.waived = sorted(set(waive))
    try:
        tip, base_sha, it, files, base_meta = _preconditions(repo, base, branch, item)
    except PreconditionError as exc:
        res.reasons = ["[pre] %s" % exc]
        return res
    except Exception as exc:  # fail closed on anything unforeseen
        res.reasons = ["[pre] gate error: %s: %s" % (type(exc).__name__, exc)]
        return res
    res.tip = tip
    n = parse_id(item)
    checks = {
        "a": lambda: _check_a(repo, tip, branch, gate_log),
        "b": lambda: _check_b(repo, tip, audit),
        "c": lambda: _check_c(it, base_meta),
        "d": lambda: _check_d(it, base_meta),
        "e": lambda: _check_e(it, n, files),
        "f": lambda: _check_f(repo, base_sha, tip),
        "g": lambda: _check_g(it, base_meta),
    }
    errored = False
    for letter in CONDITIONS:
        try:
            failed = checks[letter]()
        except Exception as exc:  # fail closed on anything unforeseen
            failed = ["[%s] gate error: %s: %s" % (letter, type(exc).__name__, exc)]
            errored = True  # a gate error is never waived
        res.checked[letter] = not failed
        res.reasons.extend(failed)
    blocking = [r for r in res.reasons if r[1:r.find("]")] not in res.waived]
    if (not errored and not blocking
            and all(res.checked[c] for c in CONDITIONS if c not in res.waived)):
        res.verdict = "MERGE"
    return res


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--repo", required=True)
    ap.add_argument("--base", required=True)
    ap.add_argument("--branch", required=True)
    ap.add_argument("--item", required=True)
    ap.add_argument("--gate-log", required=True)
    ap.add_argument("--audit", required=True)
    ap.add_argument("--waive", action="append", default=[],
                    help="a condition a Sitting pass overrides; only 'c'")
    ap.add_argument("--json", action="store_true")
    try:
        args = ap.parse_args(argv)
    except SystemExit:
        print("HOLD\n[pre] usage error (see stderr)")
        return 1
    bad = [w for w in args.waive if w not in WAIVABLE]
    if bad:
        print("HOLD\n[pre] usage error: --waive accepts only %s, not %s"
              % (", ".join(WAIVABLE), ", ".join(bad)))
        return 1
    res = evaluate(args.repo, args.base, args.branch, args.item, args.gate_log, args.audit,
                   tuple(args.waive))
    print(json.dumps(res.to_json(), indent=2) if args.json else res.to_text())
    return 0 if res.verdict == "MERGE" else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # never a traceback exit that a caller could misread
        print("HOLD\n[pre] gate error: %s: %s" % (type(exc).__name__, exc))
        sys.exit(1)
