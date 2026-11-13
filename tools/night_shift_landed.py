#!/usr/bin/env python3
"""Night Shift landed check: everything that reached ``main`` during a night was gated.

Why: ``tools/night_shift_gate.py`` decides MERGE or HOLD, but nothing tied a push
to ``main`` to that verdict: the orchestrator could merge after a HOLD, merge with
no gate run at all, merge on an ``EXIT=0`` log written for another tip, or hand-edit
a merge commit so content no gate saw slips in with it. This script is the single
enforcement point for the invariant (closeout C1): **every non-exempt commit that
reached ``origin/main`` during the night is covered by a MERGE verdict record**
(``night_shift_gate.py --record``). The runner (``tools/night_shift.sh``) calls it
once claude has exited, on ``origin/main`` as it stood BEFORE the app deploy, whose
reverts are the runner's own commits. A failure is ``UNGATED_MERGE``: flagged in the
report and ``last_run.json``, and the next night stays HELD until the owner has
looked. Nothing is reverted automatically.

Rules, for the commits in ``FROM..TO`` (``FROM`` the night's start, ``TO`` origin/main
after claude):
  plain commit c: every path it touches (``diff-tree -r --name-only``) is EXEMPT, or c
      is an ancestor of (or is) the tip of a MERGE record. An empty commit passes.
  merge commit m: m is an ancestor of a MERGE tip (a merge INSIDE a gated branch,
      such as the re-gate's ``git merge origin/main``: the gate judged a tree that
      contains it), or m adds no non-EXEMPT content of its own (``diff-tree --cc``,
      no evil merge) AND one of its parents is a MERGE tip.
A MERGE record is ``<VERDICTS>/<tip>.MERGE.json`` with ``"schema": 1``, ``"verdict": "MERGE"``
and ``"tip"`` equal to the name's sha; anything else in the directory (HOLD records
``<tip>.HOLD.json`` included) is ignored, so a later HOLD of a merged tip cannot unmerge it.

Flagged by design: a non-exempt push to ``main`` by the owner or a day Session inside the
night window (head_before to the capture) has no record and is flagged. Merges into the
map app's own repo are outside this check (it reads Jugglebot's ``main`` only). To clear
an UNGATED_MERGE: review the commits listed, then delete ``$REPORTS/ungated.hold``.

Exempt paths are what the night (and the map app) write straight to ``main`` with no
branch: the map, the night's report, the logbook, and every page ``mapkit gen``
regenerates EXCEPT ``CLAUDE.md``. ``mapkit gen`` rewrites only the CLAUDE.md term block,
and only when ``GLOSSARY.md`` changed, which the gate forbids; CLAUDE.md is a gate
FORBIDDEN path, so an ungated push to it is exactly what this check is for. The list is
literal and pinned against ``mapkit gen``'s outputs by
``tests/sim/test_night_shift_landed.py``.

Known gaps, by design: a hand-forged record is believed (a planned follow-up replays
the gate from the record); a gated branch that merged ``main`` in brings those
commits under its record; a merge that resolves by DROPPING one side's change shows
nothing under ``--cc``.

The never-list (``--branches``, closeout C7). A night may build fixes that touch
frozen paths on a held branch, for the owner to merge by hand (owner, 2026-11-13). A
few paths it may not touch even there: the Night Shift's own machinery
(:data:`NEVER_LIST`). A held branch is merged by people, so a branch that edits the
gate, the runner, the settings or the tests that pin them is one inattentive merge
from the night deciding its own rules. ``--branches`` lists every remote branch the
night pushed and the never-list paths each touches.

"Pushed tonight" is decided by a SNAPSHOT, not a name: the runner writes
``git for-each-ref refs/remotes/origin/`` (``<sha> <ref>`` lines) after its pre-run
fetch and before claude starts. Every ``refs/remotes/origin/*`` ref other than
``main`` and ``HEAD`` that is absent from the snapshot or points elsewhere was pushed
(or force-pushed) during the night, whatever it is called: a branch outside
``night-shift/*`` is checked too, so no name dodges it, and a branch from an earlier
night that nothing moved is ignored. Its paths are ``git diff --name-only`` against
the merge-base with ``--main`` (default ``refs/remotes/origin/main``). Known gaps: a
branch pushed and deleted again before the runner's fetch with ``--prune`` leaves no
ref (nothing to merge either); the map app's repo is not read.

Usage:
  python tools/night_shift_landed.py --repo R --from SHA --to SHA [--verdicts DIR] [--json]
  python tools/night_shift_landed.py --repo R --branches --before SNAPSHOT [--main REF] [--json]

Output: one summary line ``landed: gated=k exempt=m ungated=n``, then one line per
ungated commit ``UNGATED <sha> <author> <why>: <paths>``. Exit 0 when n == 0, 1 when
not, 2 on any error (the runner treats 2 as a failed check, never as a pass).
``--branches``: ``branches: pushed=k self_edit=n``, then ``SELF_EDIT <branch> <sha>:
<paths>`` per offending branch; the same exit codes (1 when n > 0).
Stdlib only: the runner runs this file as extracted from the night's starting commit.
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import os
import subprocess
import sys
from typing import Dict, List, Optional, Set

#: Paths a commit may touch with no MERGE record, matched with ``fnmatch.fnmatchcase``
#: (``*`` crosses ``/``). Keep in step with ``mapkit gen`` (the pin test fails otherwise).
EXEMPT = (
    "map/*",
    "docs/night-shift/*",
    "logbook/*",
    # mapkit gen: the three plan boards and the docs pages (gen_index, gen_docs)
    "plans/active/INDEX.md",
    "plans/parked/INDEX.md",
    "plans/archived/INDEX.md",
    "docs/project-map.md",
    "docs/glossary.md",
    "docs/runbook.md",
)

#: Written by ``mapkit gen`` and deliberately NOT exempt (see the module docstring).
GEN_NOT_EXEMPT = ("CLAUDE.md",)

#: Paths no Night Shift branch may touch, not even a held one (closeout C7, owner
#: decision 11), matched like :data:`EXEMPT` (``*`` crosses ``/``). The machinery
#: (runner, gate, checks, nonet and bash guards, ``live_session``, systemd units, the
#: nightly), the settings and protocol that bound the night, the tests that pin them,
#: and whatever decides whether those tests run. ``pyproject.toml`` is matched whole
#: (its pytest section is the concern; the rest is rarely a night's to change).
#: ``.claude/settings*`` also covers a ``settings.local.json``.
NEVER_LIST = (
    "tools/night_shift*",
    "tools/lib/*",
    "tools/systemd/*",
    "tools/nightly_*",
    ".claude/settings*",
    ".claude/commands/night-shift.md",
    "tests/sim/test_night_shift_*",
    "tests/sim/test_live_session_guard.py",
    "tests/sim/test_pull_status.py",
    "tests/sim/test_sitting_debrief_protocol.py",
    "run_tests.sh",
    "conftest.py",
    "*/conftest.py",
    "pyproject.toml",
    "pytest.ini",
    "tox.ini",
    "setup.cfg",
)

VERDICTS_ENV = "NIGHT_SHIFT_VERDICTS"
VERDICTS_DEFAULT = os.path.join("~", ".local", "state", "jugglebot", "verdicts")


class LandedError(Exception):
    pass


def is_exempt(path: str) -> bool:
    return any(fnmatch.fnmatchcase(path, pat) for pat in EXEMPT)


def is_never(path: str) -> bool:
    return any(fnmatch.fnmatchcase(path, pat) for pat in NEVER_LIST)


def verdicts_dir(arg: Optional[str] = None) -> str:
    d = arg or os.environ.get(VERDICTS_ENV) or VERDICTS_DEFAULT
    return os.path.abspath(os.path.expanduser(d))


def _git(repo: str, *args: str) -> str:
    r = subprocess.run(["git", "-C", repo] + list(args), stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, universal_newlines=True)
    if r.returncode != 0:
        raise LandedError("git %s: %s" % (" ".join(args), r.stderr.strip()))
    return r.stdout


def _resolve(repo: str, ref: str) -> str:
    sha = _git(repo, "rev-parse", "--verify", "--quiet", ref + "^{commit}").strip()
    if len(sha) != 40:
        raise LandedError("%r does not resolve to a commit" % ref)
    return sha


def merge_tips(directory: str) -> Set[str]:
    """Tips of the MERGE records in ``directory`` (missing directory: none)."""
    tips: Set[str] = set()
    try:
        names = os.listdir(directory)
    except FileNotFoundError:
        return tips
    for name in names:
        if not name.endswith(".MERGE.json") or name.startswith("."):
            continue
        try:
            with open(os.path.join(directory, name), encoding="utf-8") as fh:
                rec = json.load(fh)
        except (OSError, ValueError):
            continue
        if (isinstance(rec, dict) and rec.get("schema") == 1 and rec.get("verdict") == "MERGE"
                and isinstance(rec.get("tip"), str) and rec["tip"] + ".MERGE.json" == name
                and len(rec["tip"]) == 40):
            tips.add(rec["tip"])
    return tips


def _existing(repo: str, shas: Set[str]) -> List[str]:
    """The shas that name a commit in ``repo`` (a record from another repo is ignored)."""
    if not shas:
        return []
    r = subprocess.run(["git", "-C", repo, "cat-file", "--batch-check"],
                       input="".join(s + "\n" for s in sorted(shas)), stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, universal_newlines=True)
    if r.returncode != 0:
        raise LandedError("git cat-file: %s" % r.stderr.strip())
    return [line.split()[0] for line in r.stdout.splitlines() if line.split()[1:2] == ["commit"]]


def check(repo: str, frm: str, to: str, directory: str) -> Dict[str, object]:
    """``{"from", "to", "gated", "exempt", "ungated": [{"commit", "author", "why", "paths"}]}``."""
    frm_sha, to_sha = _resolve(repo, frm), _resolve(repo, to)
    tips = _existing(repo, merge_tips(directory))
    covered: Set[str] = set()
    if tips:
        covered = set(_git(repo, "rev-list", *tips, "^" + frm_sha).split())
    tip_set = set(tips)
    out = {"from": frm_sha, "to": to_sha, "gated": 0, "exempt": 0, "ungated": []}

    def flag(c: str, why: str, paths: List[str]) -> None:
        author = _git(repo, "log", "-1", "--format=%an <%ae>", c).strip()
        out["ungated"].append({"commit": c, "author": author, "why": why, "paths": paths})

    for c in _git(repo, "rev-list", "--reverse", "--no-merges", to_sha, "^" + frm_sha).split():
        paths = [p for p in _git(repo, "diff-tree", "--no-commit-id", "-r", "--name-only", c).splitlines() if p]
        if all(is_exempt(p) for p in paths):
            out["exempt"] += 1
        elif c in covered:
            out["gated"] += 1
        else:
            flag(c, "no MERGE record covers it", [p for p in paths if not is_exempt(p)])
    for m in _git(repo, "rev-list", "--reverse", "--merges", to_sha, "^" + frm_sha).split():
        if m in covered:
            out["gated"] += 1
            continue
        own = [p for p in _git(repo, "diff-tree", "--no-commit-id", "--cc", "--name-only", m).splitlines()
               if p and not is_exempt(p)]
        parents = _git(repo, "rev-list", "--parents", "-n", "1", m).split()[1:]
        if own:
            flag(m, "merge carries content of its own (evil merge)", own)
        elif not any(p in tip_set for p in parents):
            flag(m, "merge of no MERGE-recorded tip", [])
        else:
            out["gated"] += 1
    return out


REMOTE_PREFIX = "refs/remotes/origin/"
_NOT_A_BRANCH = (REMOTE_PREFIX + "main", REMOTE_PREFIX + "HEAD")


def read_snapshot(path: str) -> Dict[str, str]:
    """``{ref: sha}`` from a ``for-each-ref`` snapshot; any malformed line is an error."""
    out: Dict[str, str] = {}
    with open(path, encoding="utf-8") as fh:
        for n, line in enumerate(fh, 1):
            if not line.strip():
                continue
            parts = line.split()
            if len(parts) != 2 or len(parts[0]) != 40:
                raise LandedError("snapshot %s line %d is not '<sha> <ref>': %r" % (path, n, line.rstrip()))
            out[parts[1]] = parts[0]
    return out


def remote_refs(repo: str) -> Dict[str, str]:
    out: Dict[str, str] = {}
    for line in _git(repo, "for-each-ref", "--format=%(objectname) %(refname)", REMOTE_PREFIX).splitlines():
        parts = line.split()
        if len(parts) == 2:
            out[parts[1]] = parts[0]
    return out


def check_branches(repo: str, before: Dict[str, str], main: str) -> Dict[str, object]:
    """``{"pushed": [branch...], "self_edit": [{"branch", "tip", "paths"}]}`` for every
    remote branch other than main that is new or moved since ``before``."""
    main_sha = _resolve(repo, main)
    out = {"pushed": [], "self_edit": []}
    for ref, sha in sorted(remote_refs(repo).items()):
        if ref in _NOT_A_BRANCH or before.get(ref) == sha:
            continue
        branch = ref[len(REMOTE_PREFIX):]
        out["pushed"].append(branch)
        paths = [p for p in _git(repo, "diff", "-z", "--no-renames", "--name-only",
                                 "%s...%s" % (main_sha, sha)).split("\0") if p]
        hits = [p for p in paths if is_never(p)]
        if hits:
            out["self_edit"].append({"branch": branch, "tip": sha, "paths": hits})
    return out


def summary(res: Dict[str, object]) -> str:
    return "landed: gated=%d exempt=%d ungated=%d" % (res["gated"], res["exempt"], len(res["ungated"]))


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--repo", required=True)
    ap.add_argument("--from", dest="frm")
    ap.add_argument("--to")
    ap.add_argument("--verdicts", default=None,
                    help="record directory (default $%s, then %s)" % (VERDICTS_ENV, VERDICTS_DEFAULT))
    ap.add_argument("--branches", action="store_true",
                    help="the never-list scan of the branches pushed since --before")
    ap.add_argument("--before", help="--branches: the runner's pre-claude for-each-ref snapshot")
    ap.add_argument("--main", default=REMOTE_PREFIX + "main", help="--branches: the merge-base side")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)
    if args.branches:
        if not args.before:
            ap.error("--branches needs --before")
        try:
            res = check_branches(args.repo, read_snapshot(args.before), args.main)
        except Exception as exc:  # fail closed
            print("branches: error %s: %s" % (type(exc).__name__, exc))
            return 2
        if args.json:
            print(json.dumps(res, indent=2))
        else:
            print("branches: pushed=%d self_edit=%d" % (len(res["pushed"]), len(res["self_edit"])))
            for b in res["self_edit"]:
                print("SELF_EDIT %s %s: %s" % (b["branch"], b["tip"][:12], " ".join(b["paths"])))
        return 1 if res["self_edit"] else 0
    if not args.frm or not args.to:
        ap.error("--from and --to are required (or --branches)")
    try:
        res = check(args.repo, args.frm, args.to, verdicts_dir(args.verdicts))
    except Exception as exc:  # fail closed: an unchecked night is never a pass
        print("landed: error %s: %s" % (type(exc).__name__, exc))
        return 2
    if args.json:
        print(json.dumps(res, indent=2))
    else:
        print(summary(res))
        for u in res["ungated"]:
            print("UNGATED %s %s %s: %s" % (u["commit"][:12], u["author"], u["why"],
                                            " ".join(u["paths"]) or "-"))
    return 1 if res["ungated"] else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except Exception as exc:
        print("landed: error %s: %s" % (type(exc).__name__, exc))
        sys.exit(2)
