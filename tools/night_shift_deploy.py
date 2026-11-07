#!/usr/bin/env python3
"""The Night Shift's last step: put the map app live, and take back what breaks it.

Run by ``tools/night_shift.sh`` after the orchestrator has exited (owner,
2026-11-07), never by the orchestrator: its deny list keeps it away from
``~/map-live`` and ``systemctl``, and it can die mid-night (the 2026-11-02 credit-out)
with commits already merged. A script behaves the same every night.

The rule: **going back to the last good commit is always safe, so it needs no proof; a
revert changes ``main`` for everyone, so it needs proof that tonight's work is the cause.**
Without proof, the app stays on the last good commit and a Session decides.

Two things can have changed tonight, and they reach the live app differently:

* the app's ``main``: nothing happens until this script deploys it;
* Jugglebot's ``mapkit/``: ``deploy/jetson/pull.sh`` pulls the data clone and restarts
  the server within about 30 s, so it is already live when this script starts.

The decision, with old = what the live app clone runs, new = the app's origin/main:

    new == old:  check.  Good -> NOOP.  Bad, with proof -> revert tonight's mapkit.
    new != old:  deploy new, check.  Good -> DEPLOYED.  Bad -> back to old, check again:
        old good, with proof -> the app commits were the cause: revert them;
        old bad, with proof  -> revert tonight's mapkit, and the app commits with it;
        otherwise            -> revert nothing, a Session decides.

New with new mapkit is checked first because an Item can span both repos and need both
halves; judging mapkit against the old app alone would throw away good work.

**Proof** means all of: the app was healthy before the night (``--pre-night-healthy``;
a service the owner stopped is not tonight's doing); the data clone caught up, so the
check judged what is really on ``main``; the rollback itself worked; and the commits
carry ``Night-Shift: <date>``, so a day Session's work is never touched. An app revert
also needs old to be the night's starting commit and origin/main to still be new:
otherwise an older undeployed commit could be the real cause.

Map data errors are not failures: the server reports ``degraded`` when one Item file is
bad, and no code revert would cure that. They are named in the result line.

What the check can see: a build that fails, a server that will not start, an API or page
that errors, a missing asset, and a page that renders nothing (a real headless browser,
when one is installed). What it cannot see: a feature that works badly.

A revert is a NEW commit, never a rewrite. Commit writes happen in temporary worktrees, so
a clone the orchestrator left dirty or on a branch does not matter. ``--app-live`` is the
only place the deploy touches. A marker in ``--state-dir`` covers a deploy killed halfway:
the next run first puts the last good commit back.

Status words. Healthy: NOOP, DEPLOYED. Healthy again with tonight's work reverted:
ROLLED_BACK_APP, ROLLED_BACK_MAPKIT, ROLLED_BACK_BOTH. For a Session: FAILED_NEEDS_SESSION
(the app is on the last good commit unless the detail says otherwise; nothing was
reverted, or a revert could not land), ROLLBACK_FAILED (the app may be on anything, no
revert was tried), DOWN (unhealthy at the end). Nothing touched: SKIPPED_DIRTY,
SKIPPED_NO_APP. Exit 0 for every status; a result is always written. Python 3.8, stdlib only.
"""
from __future__ import annotations

import argparse
import glob
import http.client
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from typing import Dict, List, Optional, Sequence, Tuple

API_PATHS = ("/api/v1/projects", "/api/v1/users/me/tasks", "/api/v1/night-shift/summary",
             "/api/v1/night-shift/reports", "/api/v1/night-shift/questions", "/api/v1/runbook")
PAGE_PATHS = ("/", "/night-shift", "/next-sitting")
RENDER_PATHS = ("/night-shift", "/next-sitting")
_ASSET = re.compile(r'(?:src|href)="(/assets/[^"]+)"')
_ROOT_EMPTY = re.compile(r'<div id="root">\s*</div>')
_ROOT = re.compile(r'<div id="root">')
_JB = re.compile(r"\bJB-(\d+)\b")
TRAILER = "Logbook-Entry: %s-night-shift"
#: Never restored by a mapkit revert: the entries stay as history, Items are reopened by
#: name, and the generated pages are regenerated.
_KEEP_PREFIXES = ("logbook/", "map/", "docs/")
MARKER = "deploying"
#: Short and fixed: a Progress note is capped at 400 characters, and a build's output in
#: here turned ``test_item_bodies_are_readable`` red in the audit's scenario.
REOPEN_NOTE = "App deploy failed %s and tonight's commit was reverted; see the report's Cost lines. Do not retry without a Session."


class Failed(Exception):
    """A deploy, rollback or revert step failed."""


def log(msg: str) -> None:
    print("night_shift_deploy: %s" % msg, flush=True)


def run(cmd: Sequence[str], cwd: Optional[str] = None, timeout: int = 900) -> Tuple[int, str]:
    try:
        r = subprocess.run(list(cmd), cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 124, str(exc)
    return r.returncode, r.stdout.decode("utf-8", "replace")


def git(repo: str, *args: str) -> str:
    """Git output; raises :class:`Failed` on a non-zero exit."""
    rc, out = run(["git", "-C", repo] + list(args))
    if rc != 0:
        raise Failed("git %s in %s: %s" % (" ".join(args), repo, out.strip()[-300:]))
    return out.strip()


def is_ancestor(repo: str, a: str, b: str) -> bool:
    return run(["git", "-C", repo, "merge-base", "--is-ancestor", a, b])[0] == 0


# --- the check ---------------------------------------------------------------
def _get(url: str, accept: str = "*/*", timeout: float = 15.0) -> Tuple[int, str]:
    req = urllib.request.Request(url, headers={"Accept": accept})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, resp.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as exc:
        return exc.code, ""
    except (urllib.error.URLError, http.client.HTTPException, OSError, ValueError) as exc:
        return 0, str(exc)


def find_browser() -> Optional[str]:
    found = sorted(glob.glob(os.path.expanduser(
        "~/.cache/ms-playwright/chromium_headless_shell-*/chrome-linux/headless_shell")))
    return found[-1] if found else None


def render_verdict(dom: str) -> Optional[bool]:
    """True: the page mounted. False: an empty root, the white screen. None: no verdict."""
    if not _ROOT.search(dom):
        return None
    return not _ROOT_EMPTY.search(dom)


def _one_pass(base_url: str, browser: Optional[str]) -> Tuple[bool, str]:
    code, body = _get(base_url + "/api/v1/health", timeout=5.0)
    if code != 200:
        return False, "health HTTP %s %s" % (code, body[:120])
    try:
        doc = json.loads(body)
    except ValueError:
        return False, "health is not JSON"
    note = ""
    if doc.get("status") == "degraded":
        errors = doc.get("errors") or []
        note = "; map data errors (%d), first: %s" % (len(errors), str(errors[:1])[:160])
    elif doc.get("status") != "ok":
        return False, "health says %s" % body[:200]
    for path in API_PATHS:
        code, body = _get(base_url + path)
        if code != 200:
            return False, "%s HTTP %s" % (path, code)
        try:
            json.loads(body)
        except ValueError:
            return False, "%s is not JSON" % path
    for path in PAGE_PATHS:
        code, body = _get(base_url + path, accept="text/html")
        if code != 200 or not _ROOT.search(body):
            return False, "page %s HTTP %s or no root element" % (path, code)
        for asset in sorted(set(_ASSET.findall(body))):
            acode, abody = _get(base_url + asset)
            if acode != 200 or not abody:
                return False, "asset %s HTTP %s" % (asset, acode)
    if not browser:
        return True, "healthy (no headless browser, render not checked)" + note
    for path in RENDER_PATHS:
        rc, dom = run([browser, "--no-sandbox", "--disable-gpu", "--timeout=12000", "--dump-dom",
                       base_url + path], timeout=45)
        if rc == 0 and render_verdict(dom) is False:
            return False, "page %s renders an empty root" % path
    return True, "healthy" + note


def check(base_url: str, wait_s: float, browser: Optional[str]) -> Tuple[bool, str]:
    """``(healthy, one line)``. The whole pass retries until the deadline: ``pull.sh`` can
    restart the server at any moment, and one refused request must not read as broken."""
    deadline = time.monotonic() + wait_s
    while True:
        ok, detail = _one_pass(base_url, browser)
        if ok or time.monotonic() >= deadline:
            return ok, detail
        time.sleep(1.0)


# --- the live app ------------------------------------------------------------
class Live:
    def __init__(self, a: argparse.Namespace):
        self.a = a
        self.app = a.app_live
        self.state = a.state_dir
        self.front = os.path.join(self.app, "frontend")
        self.dist = os.path.join(self.front, "dist")
        self.saved = os.path.join(self.state, "dist.last-good")
        self.marker = os.path.join(self.state, MARKER)

    def save_dist(self) -> None:
        os.makedirs(self.state, exist_ok=True)
        shutil.rmtree(self.saved, ignore_errors=True)
        if os.path.isdir(self.dist):
            shutil.copytree(self.dist, self.saved, symlinks=True)

    def _sh(self, cmd: str, cwd: str, what: str) -> None:
        rc, out = run(shlex.split(cmd), cwd=cwd, timeout=self.a.build_timeout_s)
        if rc != 0:
            raise Failed("%s failed (rc=%s): %s" % (what, rc, out.strip()[-300:]))

    def _lock_changed(self, a: str, b: str) -> bool:
        return run(["git", "-C", self.app, "diff", "--quiet", a, b, "--",
                    "frontend/package-lock.json"])[0] != 0

    def deploy(self, old: str, new: str) -> None:
        with open(self.marker, "w") as fh:
            fh.write("%s %s\n" % (old, new))
        git(self.app, "merge", "-q", "--ff-only", new)
        if self._lock_changed(old, new):
            self._sh(self.a.install_cmd, self.front, "install")
        self._sh(self.a.build_cmd, self.front, "build")
        self._sh(self.a.restart_cmd, self.app, "restart")

    def finished(self) -> None:
        if os.path.exists(self.marker):
            os.unlink(self.marker)

    def rollback(self, old: str, new: str) -> None:
        """Every step runs even after one fails, so the worst case is as close to old as it
        can get; raises :class:`Failed` naming what did not work."""
        errors: List[str] = []
        try:
            git(self.app, "reset", "-q", "--hard", old)
            if self._lock_changed(old, new):
                self._sh(self.a.install_cmd, self.front, "install")
        except Failed as exc:
            errors.append(str(exc))
        try:
            if os.path.isdir(self.saved):
                shutil.rmtree(self.dist, ignore_errors=True)
                shutil.copytree(self.saved, self.dist, symlinks=True)
            else:
                self._sh(self.a.build_cmd, self.front, "build")
        except (Failed, OSError) as exc:
            errors.append("frontend: %s" % exc)
        try:
            self._sh(self.a.restart_cmd, self.app, "restart")
        except Failed as exc:
            errors.append(str(exc))
        if errors:
            raise Failed("; ".join(errors))
        self.finished()

    def recover(self) -> Optional[str]:
        """A deploy that was killed halfway leaves its marker: put the last good commit back."""
        if not os.path.exists(self.marker):
            return None
        with open(self.marker) as fh:
            old, new = fh.read().split()
        self.rollback(old, new)
        return old


def wait_for_data_clone(a: argparse.Namespace, target: str) -> bool:
    """Wait for pull.sh to bring ``target`` into the live data clone (and restart on mapkit)."""
    if not target or not os.path.isdir(a.data_live):
        return False
    deadline = time.monotonic() + a.data_wait_s
    while True:
        run(["git", "-C", a.data_live, "fetch", "-q", "origin"], timeout=60)
        if is_ancestor(a.data_live, target, "HEAD"):
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(a.poll_s)


# --- which commits are tonight's ---------------------------------------------
class Worktree:
    """A throwaway detached checkout of origin/main of ``clone``."""

    def __init__(self, clone: str):
        self.clone = clone
        self.path = ""

    def __enter__(self) -> str:
        git(self.clone, "fetch", "-q", "origin")
        self.path = tempfile.mkdtemp(prefix="night_shift_deploy_")
        os.rmdir(self.path)
        git(self.clone, "worktree", "add", "-q", "--detach", self.path, "origin/main")
        return self.path

    def __exit__(self, *exc: object) -> None:
        run(["git", "-C", self.clone, "worktree", "remove", "--force", self.path])
        shutil.rmtree(self.path, ignore_errors=True)
        run(["git", "-C", self.clone, "worktree", "prune"])


def commits_since(repo: str, start: str, end: str, *paths: str) -> List[Tuple[str, str, bool, str]]:
    """``(sha, subject, marked as the night's, date of the mark)`` per non-merge commit, newest first."""
    args = ["log", "--no-merges", "--format=%H%x00%s%x00%(trailers:key=Night-Shift,valueonly,separator=%x20)%x01",
            "%s..%s" % (start, end)]
    if paths:
        args += ["--"] + list(paths)
    out = git(repo, *args)
    found = []
    for rec in out.split("\x01"):
        rec = rec.strip()
        if not rec:
            continue
        sha, subject, mark = (rec.split("\x00") + ["", ""])[:3]
        found.append((sha, subject, bool(mark.strip()), mark.strip()))
    return found


def all_marked(commits: List[Tuple[str, str, bool, str]], date: str) -> bool:
    return all(c[3] == date for c in commits)


def items_in(commits: List[Tuple[str, str, bool, str]]) -> List[str]:
    found: List[str] = []
    for c in commits:
        for n in _JB.findall(c[1]):
            jb = "JB-%s" % n
            if jb not in found:
                found.append(jb)
    return found


# --- reverts -----------------------------------------------------------------
def revert_app(a: argparse.Namespace, start: str, new: str, why: str) -> Tuple[List[str], List[str]]:
    """Restore the app's main to ``start`` in one new commit: ``(short shas, Items)``."""
    with Worktree(a.app_clone) as wt:
        if git(wt, "rev-parse", "HEAD") != new:
            raise Failed("the app's main moved past %s during the deploy, nothing reverted" % new[:8])
        commits = commits_since(wt, start, "HEAD")
        git(wt, "read-tree", "-u", "--reset", start)
        body = "\n".join("%s %s" % (c[0][:8], c[1]) for c in commits)
        git(wt, "commit", "-q", "-m",
            "revert(night-shift): app deploy failed %s, restore %s" % (a.date, start[:8]),
            "-m", "%s\n\n%s" % (why, body), "-m", TRAILER % a.date)
        git(wt, "push", "-q", "origin", "HEAD:main")
        return [c[0][:8] for c in commits], items_in(commits)


def _changed_files(wt: str, sha: str) -> List[str]:
    return [p for p in git(wt, "show", "--no-renames", "--name-only", "--format=", sha).splitlines() if p]


def revert_mapkit(a: argparse.Namespace, wt: str, why: str) -> Tuple[List[str], List[str], List[str]]:
    """Undo tonight's mapkit commits in ``wt`` as one commit, by restoring the files they
    changed (code and tests) to the night's start. Not ``git revert``: every real night's
    report commit also touches ``logbook/INDEX.md``, so a revert would conflict every time.
    A file a later commit also changed is left alone and named: that work is not ours to lose.
    Returns ``(short shas, Items, files left alone)``."""
    start = a.main_night_start
    mine = commits_since(wt, start, "HEAD", "mapkit")
    mine_shas = set(c[0] for c in mine)
    touched: List[str] = []
    for c in mine:
        touched += [p for p in _changed_files(wt, c[0]) if not p.startswith(_KEEP_PREFIXES)]
    others = set()
    for c in commits_since(wt, start, "HEAD"):
        if c[0] not in mine_shas:
            others.update(_changed_files(wt, c[0]))
    left = sorted(set(touched) & others)
    for path in sorted(set(touched) - others):
        if run(["git", "-C", wt, "cat-file", "-e", "%s:%s" % (start, path)])[0] == 0:
            git(wt, "checkout", "-q", start, "--", path)
        else:
            git(wt, "rm", "-q", "-f", "--ignore-unmatch", "--", path)
    run([sys.executable, "-m", "mapkit", "gen"], cwd=wt)   # the generated pages follow
    git(wt, "add", "-A")
    body = "\n".join("%s %s" % (c[0][:8], c[1]) for c in mine)
    git(wt, "commit", "-q", "-m", "revert(night-shift): mapkit change broke the map app %s" % a.date,
        "-m", "%s\n\n%s" % (why, body), "-m", TRAILER % a.date)
    return [c[0][:8] for c in mine], items_in(mine), left


def reopen(wt: str, items: List[str], date: str) -> List[str]:
    """Move each Item back to in_progress with a note; an Item that fails is named, not fatal."""
    failed = []
    for jb in items:
        rc, out = run([sys.executable, "-m", "mapkit", "item", "set-status", jb, "in_progress",
                       "--owner", "night-shift", "--note", REOPEN_NOTE % date, "--force",
                       "--commit", "--no-push"], cwd=wt)
        if rc != 0:
            log("reopen %s failed: %s" % (jb, out.strip()[-200:]))
            failed.append(jb)
    return failed


def push_main(wt: str) -> None:
    """The app's git writer pushes the owner's answers to this main at any moment, so a
    rejected push is normal: rebase our new commits and try again."""
    last = ""
    for _ in range(3):
        rc, last = run(["git", "-C", wt, "push", "-q", "origin", "HEAD:main"])
        if rc == 0:
            return
        git(wt, "fetch", "-q", "origin")
        rc, out = run(["git", "-C", wt, "rebase", "-q", "origin/main"])
        if rc != 0:
            run(["git", "-C", wt, "rebase", "--abort"])
            raise Failed("rebase onto a moved main failed: %s" % out.strip()[-200:])
    raise Failed("push to main rejected three times: %s" % last.strip()[-200:])


def land_on_main(a: argparse.Namespace, do_mapkit: bool, items: List[str], why: str,
                 result: Dict[str, object]) -> None:
    """The Jugglebot side of a revert: mapkit and/or reopened Items, one push. The result
    fields are set only once the push has landed, so the line cannot claim what did not happen."""
    if not do_mapkit and not items:
        return
    with Worktree(a.main_clone) as wt:
        reverted: List[str] = []
        left: List[str] = []
        todo = list(items)
        if do_mapkit:
            reverted, closed, left = revert_mapkit(a, wt, why)
            todo += [jb for jb in closed if jb not in todo]
        failed = reopen(wt, todo, a.date)
        push_main(wt)
    if reverted:
        result["reverted_mapkit"] = reverted
    if left:
        result["left_alone"] = left
    result["items_reopened"] = [jb for jb in todo if jb not in failed]
    result["reopen_failed"] = failed


# --- the decision ------------------------------------------------------------
def line_for(result: Dict[str, object]) -> str:
    parts = ["app deploy: %s" % result["status"]]
    if result.get("app_old") != result.get("app_new"):
        parts.append("%s -> %s" % (str(result.get("app_old"))[:8], str(result.get("app_new"))[:8]))
    parts.append(str(result.get("detail") or ""))
    for key, label in (("reverted_app", "reverted on the app's main"),
                       ("reverted_mapkit", "reverted on main"),
                       ("left_alone", "left alone, changed by later commits"),
                       ("items_reopened", "reopened"),
                       ("reopen_failed", "could NOT reopen")):
        if result.get(key):
            parts.append("%s: %s" % (label, ", ".join(result[key])))  # type: ignore
    return "; ".join(p for p in parts if p)


def decide(a: argparse.Namespace, result: Dict[str, object]) -> None:
    def status(word: str, detail: str) -> None:
        result["status"], result["detail"] = word, detail

    if not os.path.isdir(os.path.join(a.app_live, ".git")):
        return status("SKIPPED_NO_APP", "no app clone at %s" % a.app_live)
    live = Live(a)
    branch = git(a.app_live, "symbolic-ref", "-q", "--short", "HEAD")
    porcelain = git(a.app_live, "status", "--porcelain")
    if branch != "main" or porcelain:
        return status("SKIPPED_DIRTY", "app clone on %s, %d changed files" % (branch, len(porcelain.splitlines())))

    recovered = None
    try:
        recovered = live.recover()
    except (Failed, OSError, ValueError) as exc:
        return status("ROLLBACK_FAILED", "a deploy killed earlier could not be undone: %s" % exc)
    if recovered:
        result["recovered_to"] = recovered
        log("a deploy killed earlier was undone, back on %s" % recovered[:8])

    git(a.app_live, "fetch", "-q", "origin")
    old = git(a.app_live, "rev-parse", "HEAD")
    new = git(a.app_live, "rev-parse", "origin/main")
    result["app_old"], result["app_new"] = old, new
    if not is_ancestor(a.app_live, old, new):
        return status("SKIPPED_DIRTY", "app clone is not an ancestor of origin/main")

    git(a.main_clone, "fetch", "-q", "origin")
    main_tip = git(a.main_clone, "rev-parse", "origin/main")
    mapkit_commits = commits_since(a.main_clone, a.main_night_start, main_tip, "mapkit") \
        if a.main_night_start else []
    caught_up = wait_for_data_clone(a, main_tip)
    result["data_clone_caught_up"] = caught_up
    browser = None if a.no_browser else find_browser()
    health = lambda: check(a.base_url, a.health_wait_s, browser)

    # Proof that a revert is justified. Each reason it is not is named in the result.
    doubts = []
    if a.pre_night_healthy == "0":
        doubts.append("the app was already unhealthy before the night")
    if not caught_up:
        doubts.append("the data clone did not catch up, so the check did not judge what is on main")
    mapkit_provable = bool(mapkit_commits) and all_marked(mapkit_commits, a.date) and not doubts

    def revert_mapkit_and_check(word: str, why: str, items: List[str]) -> None:
        land_on_main(a, True, items, why, result)
        git(a.main_clone, "fetch", "-q", "origin")
        wait_for_data_clone(a, git(a.main_clone, "rev-parse", "origin/main"))
        ok, detail = health()
        status(word if ok else "DOWN", "%s After the revert: %s" % (why, detail))

    if old == new:
        ok, detail = health()
        if ok:
            return status("NOOP", "no new app commits; %s" % detail)
        if mapkit_provable:
            return revert_mapkit_and_check("ROLLED_BACK_MAPKIT", "App unhealthy after tonight's mapkit change: %s." % detail, [])
        reason = "; ".join(doubts) or ("no marked mapkit commit tonight" if not mapkit_commits else "a mapkit commit tonight is not marked as the night's")
        return status("DOWN", "unhealthy: %s. Nothing reverted: %s." % (detail, reason))

    try:
        live.save_dist()
        live.deploy(old, new)
        ok, detail = health()
    except Exception as exc:   # whatever it was, the rollback below must still happen
        ok, detail = False, str(exc)
    if ok:
        live.finished()
        return status("DEPLOYED", detail)

    why = "New app failed: %s." % detail
    log(why)
    try:
        live.rollback(old, new)
        ok_old, detail_old = health()
    except Exception as exc:
        return status("ROLLBACK_FAILED", "%s The rollback failed too, nothing reverted: %s" % (why, exc))

    start = a.app_night_start
    run(["git", "-C", a.app_clone, "fetch", "-q", "origin"], timeout=120)
    app_commits = commits_since(a.app_clone, start, new) if start and start != new else []
    app_provable = (not doubts and start == old and bool(app_commits)
                    and all_marked(app_commits, a.date))

    def revert_app_now() -> List[str]:
        result_shas, items = revert_app(a, start, new, why)
        result["reverted_app"] = result_shas
        return items

    if ok_old:
        if not app_provable:
            return status("FAILED_NEEDS_SESSION", "%s Back on the last good commit (%s), and nothing reverted: %s." % (
                why, detail_old, "; ".join(doubts) or "the commits are not all the night's, or an older undeployed commit could be the cause"))
        items = revert_app_now()
        land_on_main(a, False, items, why, result)
        return status("ROLLED_BACK_APP", "%s Back on the last good commit: %s" % (why, detail_old))

    # The last good commit fails too, on tonight's mapkit.
    if not mapkit_provable:
        return status("DOWN", "%s The last good commit is unhealthy too (%s). Nothing reverted: %s." % (
            why, detail_old, "; ".join(doubts) or "no provable mapkit commit tonight"))
    items = revert_app_now() if app_provable else []
    word = "ROLLED_BACK_BOTH" if app_provable else "ROLLED_BACK_MAPKIT"
    revert_mapkit_and_check(word, "%s The last good commit failed too on tonight's mapkit: %s." % (why, detail_old), items)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    home = os.path.expanduser("~")
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--date", required=True)
    ap.add_argument("--result", required=True, help="the JSON result file")
    ap.add_argument("--app-night-start", default="", help="the app's origin/main when the night began")
    ap.add_argument("--main-night-start", default="", help="Jugglebot's origin/main when the night began")
    ap.add_argument("--pre-night-healthy", choices=("1", "0", "unknown"), default="unknown",
                    help="what the runner saw before claude ran; 0 forbids every revert")
    ap.add_argument("--app-live", default=os.path.join(home, "map-live", "jugglebot-map"))
    ap.add_argument("--data-live", default=os.path.join(home, "map-live", "Jugglebot"))
    ap.add_argument("--app-clone", default=os.path.join(home, "night-shift", "jugglebot-map"))
    ap.add_argument("--main-clone", default=os.path.join(home, "night-shift", "Jugglebot"))
    ap.add_argument("--state-dir", default=os.path.join(home, "map-live", "deploy-state"))
    ap.add_argument("--base-url", default="http://127.0.0.1:8765")
    ap.add_argument("--build-cmd", default="npm run build")
    ap.add_argument("--install-cmd", default="npm ci")
    ap.add_argument("--restart-cmd", default="systemctl --user restart jugglebot-map.service")
    ap.add_argument("--build-timeout-s", type=int, default=1200)
    ap.add_argument("--health-wait-s", type=float, default=60.0)
    ap.add_argument("--data-wait-s", type=float, default=120.0)
    ap.add_argument("--poll-s", type=float, default=5.0)
    ap.add_argument("--no-browser", action="store_true")
    return ap.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    a = parse_args(argv)
    result: Dict[str, object] = {"date": a.date, "status": "FAILED_NEEDS_SESSION", "detail": "did not finish"}
    try:
        decide(a, result)
    except Failed as exc:
        result["status"] = "FAILED_NEEDS_SESSION"
        result["detail"] = "a revert or git step failed: %s" % exc
    except Exception as exc:  # the runner must get a status whatever happens here
        result["status"] = "FAILED_NEEDS_SESSION"
        result["detail"] = "internal error: %s: %s" % (type(exc).__name__, exc)
    result["line"] = line_for(result)
    tmp = a.result + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(result, fh, indent=2)
        fh.write("\n")
    os.replace(tmp, a.result)
    log(str(result["line"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
