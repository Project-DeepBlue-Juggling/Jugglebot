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
      --branch night-shift/<date>-<ID> --item <ID> \\
      --gate-log PATH --audit PATH [--app-log PATH] [--waive c] [--json] [--record [DIR]]

  # A branch of the map app (its own repo, its own clone):
  python tools/night_shift_gate.py --repo R --base origin/main --app-repo A \\
      --branch night-shift/<date>-<ID> --item <ID> --gate-log PATH --audit PATH

Run from an extract of the base, never from the working tree (protocol 2g):
``git archive origin/main tools/night_shift_gate.py mapkit | tar -x -C D/gate-base``.

Output: ``MERGE`` (exit 0), or ``HOLD`` then one reason per line, each opening
with its condition letter in brackets (exit 1). ``--json`` prints
``{"verdict", "item", "branch", "tip", "reasons", "checked", "waived"}`` instead.

``--record [DIR]`` also writes the verdict, MERGE and HOLD alike, to
``DIR/<tip>.<verdict>.json`` (``<tip>.MERGE.json`` or ``<tip>.HOLD.json``; atomically: a
temp file in DIR, then a rename). The verdict is in the name so a later HOLD of the same
tip (a resume or re-check after it merged) never replaces its MERGE record. Fields: ``schema`` (1),
``verdict``, ``item``, ``branch``, ``tip``, ``base_ref``, ``base_sha``, ``waived``,
``reasons``, ``gate_sha256`` (this file), ``gate_log_sha256``, ``audit_sha256`` and
``written_at``. DIR defaults to ``$NIGHT_SHIFT_VERDICTS``, then
``~/.local/state/jugglebot/verdicts``. The runner's ``tools/night_shift_landed.py``
reads these records after the night: a commit that reached ``main`` with no MERGE
record covering it is ``UNGATED_MERGE`` (closeout C1). A MERGE whose record cannot be
written is printed as a HOLD, so no merge stands on a verdict the check cannot see.
A precondition HOLD with no branch tip to name the file is not recorded. The record
changes no verdict otherwise.

``--waive c`` is for ``/sitting-debrief``: it runs the gate on a branch the owner
passed at a Sitting, once the branch has been brought up to date with ``main``,
with a suite log and audit for that updated tip, so a hardware pass overrides the hardware condition (c) and nothing else: in
particular not (g), the owner check, which is its own letter so that no Sitting
pass can clear an Item that a branch made to look workable (owner, 2026-10-31). (c)
is still evaluated and reported in ``checked`` and the reasons, but a failing
(c) does not block; the text verdict reads ``MERGE (c waived)``. Any other
letter is a usage error (HOLD). A ``gate error`` is never waived.

Preconditions ``[pre]`` (a failure here is HOLD and the conditions are not
evaluated): the branch and the base resolve; the branch contains the base and
is at least one commit ahead of it; ``--item`` parses as an Item ID
(``BUG-``/``FEAT-``/``IDEA-``/``PWAY-``/``TKT-<n>``, never an old ID; IDs compare as
whole strings, so ``BUG-3`` is never ``FEAT-3``) and exactly one file under ``map/``
ON THE BRANCH carries that ``id:`` and loads as an Item, whose kind the ID's prefix
names (a contradiction is a HOLD naming both);
exactly one file under ``map/`` at the BASE carries it too (a branch cannot
create the Item it merges); and the gate running is the base's own: this file
and every ``mapkit`` module it loaded are byte-identical to the base (a file the
base does not have is not compared), so a branch cannot decide its own verdict.

The eight conditions, ALL evaluated so a HOLD lists every failing reason:
  (a) suite gate: the gate log's ``HEAD=<sha>`` line(s) equal the branch tip,
      any ``BRANCH=<name>`` line names the branch, and its LAST ``EXIT=<n>``
      line is ``EXIT=0``;
  (b) audit: the report's ``Audited HEAD: <sha>`` equals the branch tip and
      ``Behaviour-affecting findings open: <n>`` is present with n == 0;
  (c) hardware: the Item on the branch is rated ``hardware: none``, the
      branch did not lower a ``hardware`` rating the base had, and the Item has
      no ``## Sitting`` section on the branch or at the base (a fix to flown
      behaviour is proven only on the robot, so it never merges at night, even
      rated ``none``; closeout C7). Only ``--waive c`` clears any of these;
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
      be made to look workable). Never waivable;
  (h) app check: a branch that touches ``mapkit/`` has an ``--app-log`` from
      ``tools/night_shift_app_suite.sh`` whose ``MAPKIT_HEAD=<sha>`` is the branch
      tip and whose last ``EXIT=`` is 0. ``pull.sh`` restarts the live map app on
      any ``mapkit/`` change within about 30 s, so the app's own suite has to have
      run against this ``mapkit`` first. Never waivable.

``--app-repo`` judges a branch of the map app instead: ``--base`` and ``--branch``
name refs in the app clone, ``--repo`` is the Jugglebot clone whose ``origin/main``
holds the Item. (a) and (b) read the same log and audit formats; (c), (d) and (e)
read the Item as it stands on Jugglebot's ``origin/main`` (the app holds no Items,
so no branch can re-rate one); (f) uses :data:`APP_FORBIDDEN_PATHS`, and a pattern
there that matches nothing at the base is itself a HOLD (the two repos cannot be
pinned by one test); (g) and (h) do not apply. Merging the app's ``main`` puts
nothing live: the runner's ``tools/night_shift_deploy.py`` does that at the end.

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
edits the gate or its tests decides its own verdict. ``mapkit/`` is NOT frozen
(owner, 2026-11-07: three Items blocked in seven nights, none near the robot);
the gate imports it, which is safe because of the base-copy precondition above,
and the frozen gate tests run against the branch's ``mapkit`` inside condition (a).

A HOLD is never promoted to MERGE by anyone; change this file and its test
together or not at all.
"""
from __future__ import annotations

import argparse
import fnmatch
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mapkit.frontmatter import FrontmatterError, split_frontmatter  # noqa: E402
from mapkit.ids import KIND_OF_PREFIX, mismatch, parse_ref, split_ref  # noqa: E402
from mapkit.items import METRIC_ENUMS, Item, ItemError, load_item  # noqa: E402
from mapkit.sitting import has_sitting_section  # noqa: E402

#: Paths a Night Shift branch may never touch (plan § 2 "Merge gate rule" (f)),
#: matched with ``fnmatch.fnmatchcase`` against repo-root paths. ``*`` crosses
#: ``/``; an entry without ``*`` is one exact path (``docs/CLAUDE.md`` is not
#: forbidden). Every entry must match at least one tracked file: until 2026-11-07
#: the firmware entry read ``Teensy_code_canbridge/*``, matched none of the 71
#: firmware files and so froze nothing (``test_every_forbidden_path_matches_a_tracked_file``).
#: Firmware, config and the normative invariants are frozen because tests cannot
#: prove them; ``jugglebot_interfaces`` because the suite mocks that package even on
#: the Jetson, so only a real two-package colcon build can show a node and its
#: interface agree (a day Session merges those). WaveCamera's ``test/`` stays open:
#: it runs natively and flashes nothing. Beyond those, the safety layer itself is
#: frozen: a branch that edits the gate, the runner, the guard, the protocol or
#: their tests would decide its own verdict. That includes what decides whether those
#: tests run at all: a ``collect_ignore`` in a conftest or a new root pytest config
#: would drop them from (a). So every conftest under ``tests/`` is frozen, at any depth
#: and whether it exists at the base or is new (``tests/conftest.py`` plus
#: ``tests/*/conftest.py``, whose ``*`` crosses ``/``; until 2026-11-14 only the
#: ``tests/sim`` one was, leaving ``tests/ros``, ``tests/teensy_link`` and any new one
#: open). ``tools/sitting_merge.sh`` is the debrief's path from a held branch to ``main``
#: (closeout C10): a branch that edits it would decide how it lands itself. The last four
#: entries name files that do not exist today; creating one is exactly what they catch.
FORBIDDEN_PATHS: Tuple[str, ...] = (
    "ros_ws/src/jugglebot/Teensy_code_canbridge/*",
    "ros_ws/src/jugglebot/Teensy_code_platform/*",
    "ros_ws/src/jugglebot/CatchingCone_code/*",
    "ros_ws/src/jugglebot/WaveCamera_ESP_code/src/*",
    "ros_ws/src/jugglebot/WaveCamera_ESP_code/include/*",
    "ros_ws/src/jugglebot/WaveCamera_ESP_code/platformio.ini",
    "ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md",
    "ros_ws/src/jugglebot_interfaces/*",
    "config/*",
    "tools/night_shift*",
    "tools/lib/*",
    "tools/systemd/*",
    "tools/nightly_*",
    "tools/sitting_merge.sh",
    ".claude/*",
    "tests/sim/test_night_shift_*",
    "tests/sim/test_live_session_guard.py",
    "tests/sim/test_question_tickets.py",
    "tests/sim/test_sitting_*",
    "run_tests.sh",
    "pyproject.toml",
    "GLOSSARY.md",
    "CLAUDE.md",
    "DOCUMENTATION_GUIDE.md",
    "tests/conftest.py",
    "tests/*/conftest.py",
    "conftest.py",
    "pytest.ini",
    "tox.ini",
    "setup.cfg",
)

#: The map app's equivalent, for ``--app-repo``. Deploy units and scripts run with
#: the owner's rights; ``gitwriter`` is what commits to Jugglebot's ``main`` and
#: ``nightshift`` is the owner's answer channel; a dependency change needs an
#: install the night does not do; ``scripts/`` holds the CI's own checks.
APP_FORBIDDEN_PATHS: Tuple[str, ...] = (
    "deploy/*",
    "deploy-jetson.sh",
    ".github/*",
    "scripts/*",
    "server/gitwriter.py",
    "server/nightshift.py",
    "pyproject.toml",
    "CLAUDE.md",
    "frontend/package.json",
    "frontend/package-lock.json",
)

#: Letters ``--waive`` accepts: only the hardware condition, which a Sitting pass overrides.
WAIVABLE: Tuple[str, ...] = ("c",)

CONDITIONS: Tuple[str, ...] = ("a", "b", "c", "d", "e", "f", "g", "h")
APP_CONDITIONS: Tuple[str, ...] = ("a", "b", "c", "d", "e", "f")
MAP_DIR = "map"
MIN_SHA_PREFIX = 7
_QUESTION_CLOSED = ("complete", "dropped")

_HEAD_LINE = re.compile(r"^HEAD=(\S*)\s*$")
_MAPKIT_HEAD_LINE = re.compile(r"^MAPKIT_HEAD=(\S*)\s*$")
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
def _item_ref(item: str) -> str:
    """``--item`` as a canonical ID string, or ``[pre]``."""
    ref = parse_ref(item)
    if ref is None:
        raise PreconditionError("--item %r is not an Item ID (BUG-, FEAT-, IDEA-, "
                                "PWAY- or TKT-<n>)" % item)
    return ref


def _find_item(files: Dict[str, Optional[str]], ref: str, where: str = "on the branch") -> Tuple[str, str]:
    """``(name, text)`` of the one map file whose ``id:`` is ``ref``, a kind its prefix names."""
    matches = []
    for name, text in sorted(files.items()):
        if text is None:
            continue
        try:
            meta, _ = split_frontmatter(text)
        except FrontmatterError:
            continue
        v = meta.get("id")
        if parse_ref(v) == ref:
            matches.append((name, text, meta.get("kind")))
    if not matches:
        raise PreconditionError("no map/ file %s carries id %s" % (where, ref))
    if len(matches) > 1:
        raise PreconditionError("id %s is carried by %d files %s: %s"
                                % (ref, len(matches), where, ", ".join(m[0] for m in matches)))
    name, text, kind = matches[0]
    if mismatch(ref, str(kind)):
        raise PreconditionError("id %s names a %s, but map/%s is a %s (%s)"
                                % (ref, KIND_OF_PREFIX[split_ref(ref)[0]], name, kind, where))
    return name, text


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


def _base_item(repo: str, base_sha: str, ref: str) -> Tuple[Dict[str, object], str]:
    """The Item's ``(frontmatter, body)`` at the base; ``[pre] Item not on base`` when absent."""
    try:
        _, text = _find_item(_map_files(repo, base_sha), ref, "at the base")
    except PreconditionError as exc:
        if str(exc).startswith("no map/ file"):
            raise PreconditionError("Item not on base: %s" % exc)
        raise
    return split_frontmatter(text)


def _preconditions(repo: str, base: str, branch: str, item: str) -> Tuple[str, str, Item, Dict[str, Optional[str]], Dict[str, object], str]:
    """``(tip, base sha, Item, map files, base frontmatter, base body)`` or raise
    :class:`PreconditionError`."""
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
    ref = _item_ref(item)
    files = _map_files(repo, tip)
    name, text = _find_item(files, ref)
    it = _load_branch_item(name, text)
    base_meta, base_body = _base_item(repo, base_sha, ref)
    return tip, base_sha, it, files, base_meta, base_body


def _loaded_gate_files() -> List[Tuple[str, str]]:
    """``(repo-relative path, file on disk)`` for this script and each loaded mapkit module."""
    out = [("tools/night_shift_gate.py", os.path.abspath(__file__))]
    top = sys.modules.get("mapkit")
    root = os.path.dirname(os.path.dirname(os.path.abspath(top.__file__))) if top else ""
    for name, mod in sorted(sys.modules.items()):
        path = getattr(mod, "__file__", None)
        if path and (name == "mapkit" or name.startswith("mapkit.")):
            path = os.path.abspath(path)
            out.append((os.path.relpath(path, root).replace(os.sep, "/"), path))
    return out


def _check_gate_is_base(repo: str, base_sha: str) -> None:
    """Raise unless the code deciding this verdict is the base's own, byte for byte.
    This catches a mistaken invocation. It is not a barrier against the branch: code that
    has already been imported could say anything, which is why the protocol runs an
    extract of the base and nothing else, and why a file the base lacks is a HOLD too."""
    for rel, path in _loaded_gate_files():
        r = subprocess.run(["git", "-C", repo, "show", "%s:%s" % (base_sha, rel)],
                           capture_output=True)
        if r.returncode != 0:
            raise PreconditionError(
                "gate is not the base's copy: the base has no %s (run it from a git "
                "archive of the base, protocol 2g)" % rel)
        with open(path, "rb") as fh:
            if fh.read() != r.stdout:
                raise PreconditionError(
                    "gate is not the base's copy: %s differs from %s at the base "
                    "(run it from a git archive of the base, protocol 2g)" % (path, rel))


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
    # /sitting-debrief names the night's branch by its remote ref; the log has the local name.
    want = want[len("origin/"):] if want.startswith("origin/") else want
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


def _check_c(it: Item, base_meta: Dict[str, object], base_body: str) -> List[str]:
    """Only ever appends reasons: the Sitting rule adds a HOLD and clears none."""
    out = []
    hw = it.meta.get("hardware")
    if hw is None:
        out.append("[c] hardware not rated")
    elif hw != "none":
        out.append("[c] hardware: %s (expected none)" % hw)
    out.extend(_lowered("c", "hardware", base_meta, it))
    # A `## Sitting` section means a Sitting flies (or flew) this Item: its fix changes
    # flown behaviour and is proven only on the robot, so it never merges at night,
    # whatever the rating says. The base copy counts too, so deleting the section on
    # the branch does not dodge it. Only the debrief's `--waive c` clears it.
    if has_sitting_section(it.body) or has_sitting_section(base_body):
        out.append("[c] changes flown behaviour (has a Sitting section)")
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


def _names_item(parent: object, it: Item, ref: str) -> bool:
    if not isinstance(parent, str) or not parent.strip():
        return False
    p = parent.strip()
    if p.endswith(".md"):
        p = p[:-3]
    return p in _item_names(it) or parse_ref(p) == ref


def _check_e(it: Item, ref: str, files: Dict[str, Optional[str]]) -> List[str]:
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
        if _names_item(meta.get("parent"), it, ref):
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


def _check_h(repo: str, base_sha: str, tip: str, app_log: Optional[str]) -> List[str]:
    r = _git(repo, "diff", "-z", "--no-renames", "--name-only", "%s...%s" % (base_sha, tip))
    if r.returncode != 0:
        return ["[h] git diff failed: %s" % r.stderr.strip()]
    if not any(p.startswith("mapkit/") for p in r.stdout.split("\0")):
        return []
    if not app_log:
        return ["[h] branch touches mapkit/ and no --app-log was given"]
    lines = _read(app_log)
    if lines is None:
        return ["[h] app log %s missing or unreadable" % app_log]
    out = []
    heads = [m.group(1) for m in (_MAPKIT_HEAD_LINE.match(x) for x in lines) if m]
    if not heads:
        out.append("[h] app log has no MAPKIT_HEAD= line")
    for h in heads:
        if not _sha_matches(repo, tip, h):
            out.append("[h] app log MAPKIT_HEAD=%s is not the branch tip %s" % (h, tip))
    last = ([m.group(1) for m in (_EXIT_LINE.match(x) for x in lines) if m] or [None])[-1]
    if last is None:
        out.append("[h] app log has no EXIT= line")
    elif last != "0":
        out.append("[h] app log EXIT=%s (expected 0)" % last)
    return out


def _check_f_app(app_repo: str, base_sha: str, tip: str) -> List[str]:
    tree = _git(app_repo, "ls-tree", "-r", "-z", "--name-only", base_sha)
    if tree.returncode != 0:
        return ["[f] git ls-tree failed: %s" % tree.stderr.strip()]
    tracked = [p for p in tree.stdout.split("\0") if p]
    out = ["[f] app forbidden pattern %s matches nothing at the base" % entry
           for entry in APP_FORBIDDEN_PATHS
           if not any(fnmatch.fnmatchcase(p, entry) for p in tracked)]
    r = _git(app_repo, "diff", "-z", "--no-renames", "--name-only", "%s...%s" % (base_sha, tip))
    if r.returncode != 0:
        return out + ["[f] git diff failed: %s" % r.stderr.strip()]
    out.extend("[f] touches forbidden app path %s" % p for p in r.stdout.split("\0")
               if p and any(fnmatch.fnmatchcase(p, e) for e in APP_FORBIDDEN_PATHS))
    return out


def evaluate_app(repo: str, app_repo: str, base: str, branch: str, item: str,
                 gate_log: str, audit: str) -> GateResult:
    """The verdict for a branch of the map app; the Item is read from ``repo``'s origin/main."""
    res = GateResult(verdict="HOLD", item=item, branch=branch, tip=None,
                     checked=dict((c, False) for c in APP_CONDITIONS))
    try:
        if not os.path.isdir(app_repo) or _git(app_repo, "rev-parse", "--git-dir").returncode != 0:
            raise PreconditionError("app repo %r is not a git repository" % app_repo)
        tip, base_sha = _rev(app_repo, branch), _rev(app_repo, base)
        if tip is None:
            raise PreconditionError("app branch %r does not resolve" % branch)
        if base_sha is None:
            raise PreconditionError("app base %r does not resolve" % base)
        if _git(app_repo, "merge-base", "--is-ancestor", base_sha, tip).returncode != 0:
            raise PreconditionError("app branch %r does not contain base %r" % (branch, base))
        if tip == base_sha:
            raise PreconditionError("app branch %r is not ahead of base %r" % (branch, base))
        ref = _item_ref(item)
        item_ref = _rev(repo, "origin/main")
        if item_ref is None:
            raise PreconditionError("origin/main does not resolve in %r" % repo)
        _check_gate_is_base(repo, item_ref)
        files = _map_files(repo, item_ref)
        name, text = _find_item(files, ref, "on Jugglebot's origin/main")
        it = _load_branch_item(name, text)
        meta = split_frontmatter(text)[0]
    except PreconditionError as exc:
        res.reasons = ["[pre] %s" % exc]
        return res
    except Exception as exc:  # fail closed on anything unforeseen
        res.reasons = ["[pre] gate error: %s: %s" % (type(exc).__name__, exc)]
        return res
    res.tip = tip
    checks = {
        "a": lambda: _check_a(app_repo, tip, branch, gate_log),
        "b": lambda: _check_b(app_repo, tip, audit),
        "c": lambda: _check_c(it, meta, it.body),
        "d": lambda: _check_d(it, meta),
        "e": lambda: _check_e(it, ref, files),
        "f": lambda: _check_f_app(app_repo, base_sha, tip),
    }
    errored = False
    for letter in APP_CONDITIONS:
        try:
            failed = checks[letter]()
        except Exception as exc:  # fail closed on anything unforeseen
            failed = ["[%s] gate error: %s: %s" % (letter, type(exc).__name__, exc)]
            errored = True
        res.checked[letter] = not failed
        res.reasons.extend(failed)
    if not errored and not res.reasons:
        res.verdict = "MERGE"
    return res


# --- verdict -----------------------------------------------------------------
def evaluate(repo: str, base: str, branch: str, item: str,
             gate_log: str, audit: str, waive: Tuple[str, ...] = (),
             app_log: Optional[str] = None) -> GateResult:
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
        tip, base_sha, it, files, base_meta, base_body = _preconditions(repo, base, branch, item)
        _check_gate_is_base(repo, base_sha)
    except PreconditionError as exc:
        res.reasons = ["[pre] %s" % exc]
        return res
    except Exception as exc:  # fail closed on anything unforeseen
        res.reasons = ["[pre] gate error: %s: %s" % (type(exc).__name__, exc)]
        return res
    res.tip = tip
    ref = _item_ref(item)  # cannot raise: the preconditions parsed it
    checks = {
        "a": lambda: _check_a(repo, tip, branch, gate_log),
        "b": lambda: _check_b(repo, tip, audit),
        "c": lambda: _check_c(it, base_meta, base_body),
        "d": lambda: _check_d(it, base_meta),
        "e": lambda: _check_e(it, ref, files),
        "f": lambda: _check_f(repo, base_sha, tip),
        "g": lambda: _check_g(it, base_meta),
        "h": lambda: _check_h(repo, base_sha, tip, app_log),
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


# --- the verdict record (--record) ---------------------------------------------
#: ``--record`` with no directory: this environment variable (the runner exports it),
#: else :data:`VERDICTS_DEFAULT`. The directory is shared by every merge to ``main``
#: (the night's and the debrief's), so the landed check covers both.
VERDICTS_ENV = "NIGHT_SHIFT_VERDICTS"
VERDICTS_DEFAULT = os.path.join("~", ".local", "state", "jugglebot", "verdicts")


def verdicts_dir(arg: Optional[str] = None) -> str:
    """The record directory: ``arg`` if non-empty, else ``$NIGHT_SHIFT_VERDICTS``, else the default."""
    d = arg or os.environ.get(VERDICTS_ENV) or VERDICTS_DEFAULT
    return os.path.abspath(os.path.expanduser(d))


def _sha256_file(path: Optional[str]) -> Optional[str]:
    try:
        with open(str(path), "rb") as fh:
            return hashlib.sha256(fh.read()).hexdigest()
    except (OSError, TypeError):
        return None


def write_record(directory: str, res: GateResult, repo: str, base: str,
                 gate_log: Optional[str], audit: Optional[str]) -> Optional[str]:
    """Write ``directory/<tip>.<verdict>.json`` atomically; return its path, or None with no tip.

    The verdict is part of the name: a HOLD never replaces or removes a MERGE record.

    ``repo`` is the repository the branch lives in (the app clone for ``--app-repo``)."""
    tip = res.tip or _rev(repo, res.branch)
    if tip is None:
        return None
    record = {
        "schema": 1,
        "verdict": res.verdict,
        "item": res.item,
        "branch": res.branch,
        "tip": tip,
        "base_ref": base,
        "base_sha": _rev(repo, base),
        "waived": list(res.waived),
        "reasons": list(res.reasons),
        "gate_sha256": _sha256_file(os.path.abspath(__file__)),
        "gate_log_sha256": _sha256_file(gate_log),
        "audit_sha256": _sha256_file(audit),
        "written_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, "%s.%s.json" % (tip, res.verdict))
    fd, tmp = tempfile.mkstemp(dir=directory, prefix="." + tip, suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            json.dump(record, fh, indent=2, sort_keys=True)
            fh.write("\n")
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return path


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--repo", required=True)
    ap.add_argument("--base", required=True)
    ap.add_argument("--branch", required=True)
    ap.add_argument("--item", required=True)
    ap.add_argument("--gate-log", required=True)
    ap.add_argument("--audit", required=True)
    ap.add_argument("--app-log", help="tools/night_shift_app_suite.sh's log; required when the branch touches mapkit/")
    ap.add_argument("--app-repo", help="judge a branch of the map app in this clone (--repo holds the Item)")
    ap.add_argument("--waive", action="append", default=[],
                    help="a condition a Sitting pass overrides; only 'c'")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--record", nargs="?", const="", default=None, metavar="DIR",
                    help="also write the verdict to DIR/<tip>.json (default $%s, then %s)"
                         % (VERDICTS_ENV, VERDICTS_DEFAULT))
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
    if args.app_repo:
        if args.waive:
            print("HOLD\n[pre] usage error: --waive does not apply to --app-repo")
            return 1
        res = evaluate_app(args.repo, args.app_repo, args.base, args.branch, args.item,
                           args.gate_log, args.audit)
    else:
        res = evaluate(args.repo, args.base, args.branch, args.item, args.gate_log, args.audit,
                       tuple(args.waive), args.app_log)
    if args.record is not None:
        try:
            written = write_record(verdicts_dir(args.record), res, args.app_repo or args.repo,
                                   args.base, args.gate_log, args.audit)
        except Exception as exc:  # fail closed: a MERGE nobody can see is not a MERGE
            res.verdict = "HOLD"
            res.reasons.append("[pre] gate error: could not write the verdict record: %s: %s"
                               % (type(exc).__name__, exc))
        else:
            if written is None:
                print("night_shift_gate: no branch tip, verdict not recorded", file=sys.stderr)
    print(json.dumps(res.to_json(), indent=2) if args.json else res.to_text())
    return 0 if res.verdict == "MERGE" else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # never a traceback exit that a caller could misread
        print("HOLD\n[pre] gate error: %s: %s" % (type(exc).__name__, exc))
        sys.exit(1)
