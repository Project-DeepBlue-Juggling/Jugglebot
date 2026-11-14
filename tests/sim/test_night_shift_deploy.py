"""tools/night_shift_deploy.py: every branch of the deploy-or-take-it-back decision.

The rule under test: going back to the last good commit needs no proof, a revert needs proof
that tonight's work is the cause, and without proof nothing on ``main`` changes.

Real git, in tmp_path: bare origins for the app and Jugglebot, a live app clone, a live
data clone that a thread keeps pulling (what ``pull.sh`` does), the night's own clones,
and the real ``mapkit`` copied in so reopening an Item is the real command. The server is
a stub on an ephemeral port whose health is a function of what is actually checked out:
a ``broken`` file in the live app or in the data clone's ``mapkit/`` makes it fail, and
``needs_v2`` in the app needs ``mapkit/v2`` (an Item that spans both repos). No test
reaches the real app, ``systemctl`` or the network.
"""
from __future__ import annotations

import http.client
import http.server
import json
import os
import shutil
import subprocess
import sys
import threading
import time

import pytest

from mapkit.gen_logbook import write_index
from mapkit.items import PROGRESS_CAP

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import night_shift_deploy as deploy  # noqa: E402

DATE = "2026-11-08"
MARK = "Night-Shift: %s" % DATE
BROWSER = deploy.find_browser()   # at import: the fixture below points HOME at tmp_path


def git(cwd, *args):
    return subprocess.run(["git"] + list(args), cwd=str(cwd), check=True, capture_output=True,
                          text=True).stdout.strip()


def commit(cwd, msg, mark=True):
    git(cwd, "add", "-A")
    args = ["commit", "-q", "-m", msg]
    if mark:
        args += ["-m", MARK]
    git(cwd, *args)
    git(cwd, "push", "-q", "origin", "HEAD:main")


@pytest.fixture(autouse=True)
def _git_env(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", str(tmp_path / "gitconfig"))
    monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")
    for k in ("GIT_AUTHOR_NAME", "GIT_COMMITTER_NAME"):
        monkeypatch.setenv(k, "t")
    for k in ("GIT_AUTHOR_EMAIL", "GIT_COMMITTER_EMAIL"):
        monkeypatch.setenv(k, "t@example.com")


ITEM = """---
id: BUG-7
kind: bug
title: Wobble
status: complete
created: '2026-10-01'
updated: '2026-10-01T00:00:00Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: map-app
blocked_by: []
links: []
---

Body.
"""


class Rig:
    def __init__(self, tmp, item_file="bug-wobble.md", item_text=ITEM):
        self.tmp = tmp
        self.item_file = item_file
        self.result = tmp / "result.json"
        self.page = '<div id="root"></div>'
        # the app
        self.app_seed = tmp / "app-seed"
        self.app_seed.mkdir()
        git(self.app_seed, "init", "-q")
        git(self.app_seed, "checkout", "-q", "-b", "main")
        (self.app_seed / "frontend").mkdir()
        (self.app_seed / "frontend" / "package-lock.json").write_text("{}\n")
        (self.app_seed / "version").write_text("1\n")
        (self.app_seed / "extra").write_text("seeded\n")
        (self.app_seed / ".gitignore").write_text("frontend/dist/\n")
        git(self.app_seed, "add", "-A")
        git(self.app_seed, "commit", "-q", "-m", "seed")
        self.app_origin = tmp / "app.git"
        git(tmp, "clone", "-q", "--bare", str(self.app_seed), str(self.app_origin))
        git(self.app_seed, "remote", "add", "origin", str(self.app_origin))
        git(self.app_seed, "fetch", "-q", "origin")
        self.app_live = tmp / "live" / "app"
        self.app_clone = tmp / "night" / "app"
        self.app_live.parent.mkdir()
        self.app_clone.parent.mkdir()
        git(tmp, "clone", "-q", str(self.app_origin), str(self.app_live))
        git(tmp, "clone", "-q", str(self.app_origin), str(self.app_clone))
        (self.app_live / "frontend" / "dist").mkdir()
        (self.app_live / "frontend" / "dist" / "built").write_text("1\n")
        # Jugglebot
        self.main_seed = tmp / "main-seed"
        self.main_seed.mkdir()
        git(self.main_seed, "init", "-q")
        git(self.main_seed, "checkout", "-q", "-b", "main")
        shutil.copytree(os.path.join(ROOT, "mapkit"), str(self.main_seed / "mapkit"),
                        ignore=shutil.ignore_patterns("__pycache__"))
        for rel in ("GLOSSARY.md", "CLAUDE.md"):   # `set-status --commit` regenerates from these
            shutil.copy(os.path.join(ROOT, rel), str(self.main_seed / rel))
        (self.main_seed / "docs").mkdir()
        (self.main_seed / "docs" / "index.md").write_text("# docs\n")
        (self.main_seed / "logbook").mkdir()
        (self.main_seed / "logbook" / "INDEX.md").write_text("| row |\n")
        (self.main_seed / "map").mkdir()
        (self.main_seed / "map" / item_file).write_text(item_text)
        (self.main_seed / ".gitignore").write_text("__pycache__/\n")
        git(self.main_seed, "add", "-A")
        git(self.main_seed, "commit", "-q", "-m", "seed")
        self.main_origin = tmp / "main.git"
        git(tmp, "clone", "-q", "--bare", str(self.main_seed), str(self.main_origin))
        git(self.main_seed, "remote", "add", "origin", str(self.main_origin))
        git(self.main_seed, "fetch", "-q", "origin")
        self.data_live = tmp / "live" / "data"
        self.main_clone = tmp / "night" / "main"
        git(tmp, "clone", "-q", str(self.main_origin), str(self.data_live))
        git(tmp, "clone", "-q", str(self.main_origin), str(self.main_clone))
        self.app_start = git(self.app_seed, "rev-parse", "HEAD")
        self.main_start = git(self.main_seed, "rev-parse", "HEAD")
        # the build copies the tree's version into dist; a `build_fails` file fails it, loudly
        self.build = tmp / "build.sh"
        self.build.write_text('#!/usr/bin/env bash\nif [[ -e ../build_fails ]]; then\n'
                              '  for i in $(seq 1 40); do echo "tsc error number $i in a very long path"; done\n'
                              '  exit 3\nfi\nmkdir -p dist && cp ../version dist/built\n')
        self.build.chmod(0o755)
        self.restart_log = tmp / "restarts"
        self.restart_fails = tmp / "restart_fails"
        self.restart = tmp / "restart.sh"
        self.restart.write_text('#!/usr/bin/env bash\necho r >>"%s"\n[[ ! -e "%s" ]]\n'
                                % (self.restart_log, self.restart_fails))
        self.restart.chmod(0o755)
        self._serve()
        self._stop = threading.Event()
        self._puller = threading.Thread(target=self._pull_loop, daemon=True)
        self._puller.start()

    def healthy(self):
        app, data = self.app_live, self.data_live / "mapkit"
        if (app / "broken").exists() or (data / "broken").exists():
            return False
        if (app / "needs_v2").exists() and not (data / "v2").exists():
            return False
        if (data / "v2").exists() and not (app / "needs_v2").exists():
            return False
        return True

    def _serve(self):
        rig = self

        class H(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                ok = rig.healthy()
                if self.path == "/api/v1/health":
                    state = "degraded" if (rig.data_live / "mapkit" / "degraded").exists() else "ok"
                    code = 200 if ok else 500
                    body = json.dumps({"status": state, "errors": ["map/bad.md"] if state != "ok" else []})
                elif self.path.startswith("/api/"):
                    code, body = (200, "{}") if ok else (500, "")
                else:
                    code, body = 200, rig.page
                data = body.encode()
                self.send_response(code)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

        self.server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), H)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.base_url = "http://127.0.0.1:%d" % self.server.server_address[1]

    def _pull_loop(self):
        while not self._stop.is_set():
            subprocess.run(["git", "-C", str(self.data_live), "pull", "-q", "--ff-only"],
                           capture_output=True)
            time.sleep(0.1)

    def stop_pulling(self):
        self._stop.set()
        self._puller.join(timeout=5)

    def close(self):
        self.stop_pulling()
        self.server.shutdown()

    def app_commit(self, *files, msg="fix(app): thing (BUG-7)", remove=(), mark=True):
        for rel in files:
            (self.app_seed / rel).write_text("x\n")
        for rel in remove:
            os.unlink(str(self.app_seed / rel))
        commit(self.app_seed, msg, mark=mark)

    def main_commit(self, *files, msg="feat(mapkit): thing (BUG-7)", mark=True):
        git(self.main_seed, "pull", "-q", "--ff-only", "origin", "main")
        for rel in files:
            p = self.main_seed / rel
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text("x\n")
        commit(self.main_seed, msg, mark=mark)

    def run(self, app_start=None, main_start=None, data_wait_s=20.0, pre_night="1"):
        deploy.main([
            "--date", DATE, "--result", str(self.result),
            "--app-night-start", self.app_start if app_start is None else app_start,
            "--main-night-start", self.main_start if main_start is None else main_start,
            "--pre-night-healthy", pre_night,
            "--app-live", str(self.app_live), "--data-live", str(self.data_live),
            "--app-clone", str(self.app_clone), "--main-clone", str(self.main_clone),
            "--state-dir", str(self.tmp / "state"), "--base-url", self.base_url,
            "--build-cmd", str(self.build), "--install-cmd", "true",
            "--restart-cmd", str(self.restart), "--health-wait-s", "1", "--poll-s", "0.1",
            "--data-wait-s", str(data_wait_s), "--no-browser"])
        return json.loads(self.result.read_text())

    def origin_tip(self, origin):
        return git(self.tmp, "--git-dir", str(origin), "rev-parse", "main")

    def origin_file(self, origin, rel):
        r = subprocess.run(["git", "--git-dir", str(origin), "show", "main:%s" % rel],
                           capture_output=True, text=True)
        return r.stdout if r.returncode == 0 else None

    def live_head(self):
        return git(self.app_live, "rev-parse", "HEAD")

    def item(self):
        return self.origin_file(self.main_origin, "map/" + self.item_file)


@pytest.fixture
def rig(tmp_path):
    r = Rig(tmp_path)
    yield r
    r.close()


# --- nothing goes wrong ------------------------------------------------------
def test_noop_when_nothing_changed(rig):
    res = rig.run()
    assert res["status"] == "NOOP"
    assert not rig.restart_log.exists()
    assert res["line"].startswith("app deploy: NOOP")


def test_good_commit_is_deployed(rig):
    rig.app_commit("feature")
    tip = rig.origin_tip(rig.app_origin)
    res = rig.run()
    assert res["status"] == "DEPLOYED"
    assert rig.live_head() == tip
    assert rig.restart_log.read_text().count("r") == 1
    assert rig.origin_tip(rig.app_origin) == tip   # nothing reverted
    assert not (rig.tmp / "state" / "deploying").exists()


def test_a_pair_that_works_together_is_kept(rig):
    """Old app + new mapkit is unhealthy; new app + new mapkit is fine. Judging the mapkit
    against the old app first would throw away good work, so the pair is checked first."""
    rig.main_commit("mapkit/v2")
    rig.app_commit("needs_v2")
    tip = rig.origin_tip(rig.app_origin)
    res = rig.run()
    assert res["status"] == "DEPLOYED"
    assert rig.live_head() == tip
    assert rig.origin_file(rig.main_origin, "mapkit/v2") is not None


@pytest.mark.parametrize("with_commit,word", [(False, "NOOP"), (True, "DEPLOYED")])
def test_one_bad_map_file_is_not_a_failure(rig, with_commit, word):
    """``degraded`` means an Item file is bad. No code revert cures that, and the night's own
    mapkit commit in the same range must survive it."""
    rig.main_commit("mapkit/degraded")
    if with_commit:
        rig.app_commit("feature")
    res = rig.run()
    assert res["status"] == word
    assert "map data errors (1)" in res["line"]
    assert rig.origin_file(rig.main_origin, "mapkit/degraded") is not None


# --- the app commit is the cause --------------------------------------------
def _assert_app_reverted(rig, res, before):
    assert res["status"] in ("ROLLED_BACK_APP", "ROLLED_BACK_BOTH")
    assert rig.live_head() == before               # back on the last good commit
    assert rig.origin_file(rig.app_origin, "broken") is None   # tomorrow does not redeploy it
    assert (rig.app_live / "frontend" / "dist" / "built").read_text() == "1\n"
    log = git(rig.tmp, "--git-dir", str(rig.app_origin), "log", "-1", "--format=%B", "main")
    assert log.startswith("revert(night-shift): app deploy failed %s" % DATE)
    assert "Logbook-Entry: %s-night-shift" % DATE in log


def test_broken_commit_is_rolled_back_reverted_and_its_item_reopened(rig):
    before = rig.live_head()
    rig.app_commit("broken")
    res = rig.run()
    _assert_app_reverted(rig, res, before)
    assert res["items_reopened"] == ["BUG-7"] and res["reopen_failed"] == []
    assert "status: in_progress" in rig.item()
    assert "App deploy failed %s" % DATE in rig.item()
    assert "reopened: BUG-7" in res["line"]


def test_a_file_the_commit_deleted_comes_back(rig):
    before = rig.live_head()
    rig.app_commit("broken", remove=("extra",))
    assert rig.origin_file(rig.app_origin, "extra") is None
    res = rig.run()
    _assert_app_reverted(rig, res, before)
    assert rig.origin_file(rig.app_origin, "extra") == "seeded\n"


def test_failed_build_is_rolled_back_and_its_note_fits_the_cap(rig):
    """A build's long output once reached the Progress note, over the 400 characters that
    ``test_item_bodies_are_readable`` allows, which would have turned main red."""
    before = rig.live_head()
    rig.app_commit("build_fails")
    res = rig.run()
    assert res["status"] == "ROLLED_BACK_APP"
    assert "build failed" in res["detail"]
    assert rig.live_head() == before
    assert rig.origin_file(rig.app_origin, "build_fails") is None
    assert rig.restart_log.read_text().count("r") == 1   # the rollback's restart only
    notes = [l for l in rig.item().splitlines() if l.startswith("- ") and "App deploy failed" in l]
    assert notes and all(len(l) <= PROGRESS_CAP for l in notes)


def test_a_crash_during_the_check_still_rolls_back(rig, monkeypatch):
    before = rig.live_head()
    rig.app_commit("feature")
    real = deploy.check
    calls = []

    def flaky(*a):
        calls.append(1)
        if len(calls) == 1:
            raise http.client.IncompleteRead(b"")
        return real(*a)
    monkeypatch.setattr(deploy, "check", flaky)
    res = rig.run()
    assert res["status"] == "ROLLED_BACK_APP" and rig.live_head() == before


# --- without proof, nothing on main changes ----------------------------------
def _nothing_reverted(rig, app_tip, main_tip, res):
    assert rig.origin_tip(rig.app_origin) == app_tip
    assert rig.origin_tip(rig.main_origin) == main_tip
    assert not res.get("reverted_app") and not res.get("reverted_mapkit")


def test_a_day_commit_in_the_night_window_is_never_reverted(rig):
    before = rig.live_head()
    rig.app_commit("broken", msg="fix(app): a day Session's work (BUG-7)", mark=False)
    app_tip, main_tip = rig.origin_tip(rig.app_origin), rig.origin_tip(rig.main_origin)
    res = rig.run()
    assert res["status"] == "FAILED_NEEDS_SESSION"
    assert rig.live_head() == before
    _nothing_reverted(rig, app_tip, main_tip, res)
    assert "status: complete" in rig.item()


def test_an_older_undeployed_commit_does_not_get_tonights_blamed(rig):
    """The old night's deploy was skipped for a Sitting, so X sits undeployed before the
    night's start. Tonight's Y is good but deploys on top of X and fails. Reverting Y and
    telling the owner not to retry it would be the wrong commit, and X would still be there."""
    before = rig.live_head()
    rig.app_commit("broken", msg="fix(app): X (BUG-5)")
    start = rig.origin_tip(rig.app_origin)
    rig.app_commit("feature", msg="fix(app): Y (BUG-7)")
    app_tip, main_tip = rig.origin_tip(rig.app_origin), rig.origin_tip(rig.main_origin)
    res = rig.run(app_start=start)
    assert res["status"] == "FAILED_NEEDS_SESSION"
    assert rig.live_head() == before
    _nothing_reverted(rig, app_tip, main_tip, res)


def test_unhealthy_before_the_night_reverts_nothing(rig):
    """A service the owner stopped looks exactly like a broken mapkit afterwards."""
    rig.main_commit("mapkit/broken")
    app_tip, main_tip = rig.origin_tip(rig.app_origin), rig.origin_tip(rig.main_origin)
    res = rig.run(pre_night="0")
    assert res["status"] == "DOWN" and "already unhealthy" in res["detail"]
    _nothing_reverted(rig, app_tip, main_tip, res)


def test_a_data_clone_that_never_caught_up_reverts_nothing(rig):
    """pull.sh refuses while the server holds an unpushed commit, so the check judged a new
    app against an old mapkit: no evidence about what is on main."""
    rig.stop_pulling()
    before = rig.live_head()
    rig.main_commit("mapkit/v2")
    rig.app_commit("needs_v2")
    app_tip, main_tip = rig.origin_tip(rig.app_origin), rig.origin_tip(rig.main_origin)
    res = rig.run(data_wait_s=0.2)
    assert res["status"] == "FAILED_NEEDS_SESSION" and "did not catch up" in res["detail"]
    assert rig.live_head() == before
    _nothing_reverted(rig, app_tip, main_tip, res)


def test_already_down_with_nothing_new_reverts_nothing(rig):
    (rig.app_live / "broken").write_text("x\n")
    git(rig.app_live, "add", "-A")
    git(rig.app_live, "commit", "-q", "-m", "local")
    git(rig.app_live, "push", "-q", "origin", "HEAD:main")
    tip, main_tip = rig.origin_tip(rig.app_origin), rig.origin_tip(rig.main_origin)
    res = rig.run(app_start=tip)
    assert res["status"] == "DOWN"
    _nothing_reverted(rig, tip, main_tip, res)


def test_an_unmarked_mapkit_commit_is_not_reverted(rig):
    rig.main_commit("mapkit/broken", msg="feat(mapkit): a day Session's work (BUG-7)", mark=False)
    app_tip, main_tip = rig.origin_tip(rig.app_origin), rig.origin_tip(rig.main_origin)
    res = rig.run()
    assert res["status"] == "DOWN"
    _nothing_reverted(rig, app_tip, main_tip, res)


def test_a_failed_rollback_is_named_and_reverts_nothing(rig):
    rig.app_commit("broken")
    rig.restart_fails.write_text("x\n")
    app_tip, main_tip = rig.origin_tip(rig.app_origin), rig.origin_tip(rig.main_origin)
    res = rig.run()
    assert res["status"] == "ROLLBACK_FAILED"
    _nothing_reverted(rig, app_tip, main_tip, res)
    # Every rollback step ran anyway: the tree and the build are the last good ones.
    assert (rig.app_live / "frontend" / "dist" / "built").read_text() == "1\n"
    assert not (rig.app_live / "broken").exists()


def test_a_main_that_moved_during_the_deploy_is_not_reverted(rig, monkeypatch):
    before = rig.live_head()
    rig.app_commit("broken")
    real = deploy.Live.rollback

    def moved(self, old, new):
        real(self, old, new)
        rig.app_commit("later", msg="docs: a day Session's push", mark=False)
    monkeypatch.setattr(deploy.Live, "rollback", moved)
    res = rig.run()
    assert res["status"] == "FAILED_NEEDS_SESSION" and "moved" in res["detail"]
    assert rig.live_head() == before
    assert rig.origin_file(rig.app_origin, "later") is not None


def test_a_rejected_revert_still_leaves_the_last_good_commit_live(rig, monkeypatch):
    before = rig.live_head()
    rig.app_commit("broken")

    def refuse(*a, **k):
        raise deploy.Failed("push rejected")
    monkeypatch.setattr(deploy, "revert_app", refuse)
    res = rig.run()
    assert res["status"] == "FAILED_NEEDS_SESSION" and "push rejected" in res["detail"]
    assert rig.live_head() == before


# --- mapkit is the cause ----------------------------------------------------
def _mapkit_night(rig):
    """What a real night looks like: the fix commit carries its test, its entry and the
    regenerated logbook/INDEX.md, and the report commit after it regenerates the index too,
    so a `git revert` would conflict."""
    git(rig.main_seed, "pull", "-q", "--ff-only", "origin", "main")
    (rig.main_seed / "mapkit" / "broken").write_text("x\n")
    (rig.main_seed / "tests").mkdir()
    (rig.main_seed / "tests" / "test_new_mapkit.py").write_text("x\n")
    (rig.main_seed / "logbook" / ("%s-fix.md" % DATE)).write_text(_ENTRY % ("Fix", DATE))
    write_index(str(rig.main_seed))
    commit(rig.main_seed, "feat(mapkit): thing (BUG-7)")
    (rig.main_seed / "logbook" / ("%s-night-shift.md" % DATE)).write_text(_ENTRY % ("Report", DATE))
    write_index(str(rig.main_seed))
    commit(rig.main_seed, "docs(night-shift): report %s" % DATE, mark=False)


_ENTRY = "---\ntitle: %s\ntype: feature\ndate: %s\nstatus: complete\n---\n# entry\n"


def test_mapkit_breaking_the_app_is_undone_on_a_real_night(rig):
    _mapkit_night(rig)
    res = rig.run()
    assert res["status"] == "ROLLED_BACK_MAPKIT"
    assert rig.origin_file(rig.main_origin, "mapkit/broken") is None
    assert rig.origin_file(rig.main_origin, "tests/test_new_mapkit.py") is None   # main stays green
    assert rig.origin_file(rig.main_origin, "logbook/%s-fix.md" % DATE).startswith("---")  # history stays
    index = rig.origin_file(rig.main_origin, "logbook/INDEX.md")
    assert "](%s-fix.md)" % DATE in index and "](%s-night-shift.md)" % DATE in index
    assert res["items_reopened"] == ["BUG-7"] and "status: in_progress" in rig.item()
    log = git(rig.tmp, "--git-dir", str(rig.main_origin), "log", "-2", "--format=%B", "main")
    assert "revert(night-shift): mapkit change broke the map app" in log
    assert "Logbook-Entry: %s-night-shift" % DATE in log


def test_a_file_later_work_also_changed_is_left_alone_and_named(rig):
    rig.main_commit("mapkit/broken", "shared.txt")
    git(rig.main_seed, "pull", "-q", "--ff-only", "origin", "main")
    (rig.main_seed / "shared.txt").write_text("later work\n")
    commit(rig.main_seed, "docs: later work", mark=False)
    res = rig.run()
    assert res["status"] == "ROLLED_BACK_MAPKIT"
    assert rig.origin_file(rig.main_origin, "mapkit/broken") is None
    assert rig.origin_file(rig.main_origin, "shared.txt") == "later work\n"
    assert res["left_alone"] == ["shared.txt"] and "shared.txt" in res["line"]


def test_both_broken_are_both_reverted(rig):
    before = rig.live_head()
    rig.main_commit("mapkit/broken")
    rig.app_commit("broken")
    res = rig.run()
    assert res["status"] == "ROLLED_BACK_BOTH"
    _assert_app_reverted(rig, res, before)
    assert rig.origin_file(rig.main_origin, "mapkit/broken") is None


def test_only_mapkit_is_reverted_when_the_app_commits_cannot_be_proved(rig):
    rig.main_commit("mapkit/broken")
    rig.app_commit("feature", msg="fix(app): a day Session's work (BUG-5)", mark=False)
    app_tip = rig.origin_tip(rig.app_origin)
    res = rig.run()
    assert res["status"] == "ROLLED_BACK_MAPKIT"
    assert rig.origin_tip(rig.app_origin) == app_tip
    assert rig.origin_file(rig.main_origin, "mapkit/broken") is None


def test_a_main_the_owner_just_wrote_to_is_rebased_onto(rig, monkeypatch):
    """The app's git writer pushes answers to this main at any moment."""
    rig.app_commit("broken")
    real = deploy.reopen

    def answer_arrives(wt, items, date):
        failed = real(wt, items, date)
        git(rig.main_seed, "pull", "-q", "--ff-only", "origin", "main")
        (rig.main_seed / "map" / "answer.txt").write_text("answer\n")
        commit(rig.main_seed, "map: the owner's answer", mark=False)
        return failed
    monkeypatch.setattr(deploy, "reopen", answer_arrives)
    res = rig.run()
    assert res["status"] == "ROLLED_BACK_APP" and res["items_reopened"] == ["BUG-7"]
    assert "status: in_progress" in rig.item()
    assert rig.origin_file(rig.main_origin, "map/answer.txt") == "answer\n"


def test_a_line_never_claims_a_reopen_that_did_not_land(rig, monkeypatch):
    rig.app_commit("broken")

    def rejected(wt):
        raise deploy.Failed("push rejected")
    monkeypatch.setattr(deploy, "push_main", rejected)
    res = rig.run()
    assert res["status"] == "FAILED_NEEDS_SESSION"
    assert "reopened" not in res["line"]
    assert "status: complete" in rig.item()


# --- leaving things alone, and coming back --------------------------------------
@pytest.mark.parametrize("how", ["dirty", "branch"])
def test_a_touched_live_clone_is_left_alone(rig, how):
    rig.app_commit("feature")
    if how == "dirty":
        (rig.app_live / "stray").write_text("x\n")
    else:
        git(rig.app_live, "checkout", "-q", "-b", "experiment")
    before = rig.live_head()
    res = rig.run()
    assert res["status"] == "SKIPPED_DIRTY"
    assert rig.live_head() == before and not rig.restart_log.exists()


def test_no_live_clone(rig):
    shutil.rmtree(str(rig.app_live))
    assert rig.run()["status"] == "SKIPPED_NO_APP"


def test_a_clone_left_on_a_branch_by_a_dead_night_does_not_matter(rig):
    before = rig.live_head()
    rig.app_commit("broken")
    git(rig.app_clone, "checkout", "-q", "-b", "night-shift/x")
    (rig.app_clone / "half-done").write_text("x\n")
    git(rig.main_clone, "checkout", "-q", "-b", "night-shift/y")
    (rig.main_clone / "half-done").write_text("x\n")
    res = rig.run()
    _assert_app_reverted(rig, res, before)
    assert (rig.app_clone / "half-done").exists() and (rig.main_clone / "half-done").exists()
    assert git(rig.app_clone, "worktree", "list").count("\n") == 0


def test_a_deploy_killed_halfway_is_undone_next_run(rig):
    """New server code with an old build, and old == new, would otherwise never be rebuilt."""
    old = rig.live_head()
    (rig.app_seed / "version").write_text("2\n")
    commit(rig.app_seed, "fix(app): version two (BUG-7)")
    new = rig.origin_tip(rig.app_origin)
    live = deploy.Live(deploy.parse_args(["--date", DATE, "--result", "x", "--app-live", str(rig.app_live),
                                          "--state-dir", str(rig.tmp / "state")]))
    live.save_dist()
    with open(live.marker, "w") as fh:
        fh.write("%s %s\n" % (old, new))
    git(rig.app_live, "fetch", "-q", "origin")
    git(rig.app_live, "merge", "-q", "--ff-only", new)   # killed right here
    res = rig.run()
    assert res["recovered_to"] == old
    assert res["status"] == "DEPLOYED" and rig.live_head() == new
    assert (rig.app_live / "frontend" / "dist" / "built").read_text() == "2\n"
    assert not os.path.exists(live.marker)


def test_a_crash_inside_still_writes_a_result(rig, monkeypatch):
    def boom(*a, **k):
        raise RuntimeError("surprise")
    monkeypatch.setattr(deploy, "decide", boom)
    res = rig.run()
    assert res["status"] == "FAILED_NEEDS_SESSION" and "surprise" in res["detail"]


# --- the check ------------------------------------------------------------------
def test_render_verdict():
    assert deploy.render_verdict('<body><div id="root"></div></body>') is False
    assert deploy.render_verdict('<div id="root">\n</div>') is False
    assert deploy.render_verdict('<div id="root"><div>x</div></div>') is True
    assert deploy.render_verdict("<html></html>") is None
    assert deploy.render_verdict("") is None


def test_check_names_what_failed(rig):
    ok, detail = deploy.check(rig.base_url, 0.0, None)
    assert ok and "render not checked" in detail
    (rig.app_live / "broken").write_text("x\n")
    ok, detail = deploy.check(rig.base_url, 0.0, None)
    assert not ok and "health HTTP 500" in detail
    ok, detail = deploy.check("http://127.0.0.1:1", 0.0, None)
    assert not ok and "health" in detail


def test_check_rides_out_a_restart(rig):
    """pull.sh can restart the server at any moment: a failure that clears is not a failure."""
    (rig.app_live / "broken").write_text("x\n")
    threading.Timer(0.5, lambda: os.unlink(str(rig.app_live / "broken"))).start()
    ok, _ = deploy.check(rig.base_url, 10.0, None)
    assert ok


def test_get_survives_a_response_cut_short(monkeypatch):
    def cut(*a, **k):
        raise http.client.IncompleteRead(b"")
    monkeypatch.setattr(deploy.urllib.request, "urlopen", cut)
    assert deploy._get("http://127.0.0.1:1/")[0] == 0


@pytest.mark.skipif(not BROWSER, reason="no headless browser here")
def test_a_real_browser_tells_a_mounted_page_from_a_white_screen(rig):
    browser = BROWSER
    rig.page = ('<!doctype html><body><div id="root"></div><script>'
                'document.getElementById("root").innerHTML="<p>up</p>"</script></body>')
    ok, detail = deploy.check(rig.base_url, 0.0, browser)
    assert ok, detail
    rig.page = ('<!doctype html><body><div id="root"></div><script>'
                'throw new Error("boom")</script></body>')
    ok, detail = deploy.check(rig.base_url, 0.0, browser)
    assert not ok and "empty root" in detail


# --- item-id-per-kind P1 (plan section 5, T-U8): a rollback names Items in either ID form ---

def test_items_in_finds_every_per_kind_prefix_in_subjects():
    """T-U8: ``items_in`` reads every per-kind prefix; look-alikes and old-form IDs are not Items."""
    subjects = ["fix(app): OLD-7 and BUG-3", "feat(mapkit): thing (FEAT-12)",
                "fix: IDEA-2, PWAY-4, TKT-31; BUG-3 again", "OLD_OP_X XBUG-9 BUG-3x bug-5 FEAT"]
    commits = [("sha%d" % i, s, True, DATE) for i, s in enumerate(subjects)]
    assert deploy.items_in(commits) == ["BUG-3", "FEAT-12", "IDEA-2", "PWAY-4", "TKT-31"]


def test_items_in_knows_every_mapkit_prefix():
    """T-U8: the deploy keeps its own prefix list (a rollback must not import the mapkit it may be
    undoing), so this pins it to ``mapkit.ids.ALL_PREFIXES``."""
    from mapkit.ids import ALL_PREFIXES
    assert set(deploy.ID_PREFIXES) == set(ALL_PREFIXES)


FEAT_ITEM = ITEM.replace("id: BUG-7", "id: FEAT-12").replace("kind: bug", "kind: feature") \
    .replace("diagnosis: confirmed\n", "")


@pytest.fixture
def feat_rig(tmp_path):
    r = Rig(tmp_path, item_file="feature-tilt.md", item_text=FEAT_ITEM)
    yield r
    r.close()


def test_a_rollback_reopens_a_per_kind_item(feat_rig):
    """T-U8 (silent failure 3): a reverted commit naming FEAT-12 reports and reopens FEAT-12.

    On the unfixed deploy the old-form-only pattern matched no per-kind ID, so ``items_in`` returned [] and
    the rollback reopened nothing and named nothing.
    """
    rig = feat_rig
    before = rig.live_head()
    rig.app_commit("broken", msg="fix(app): thing (FEAT-12)")
    res = rig.run()
    _assert_app_reverted(rig, res, before)
    assert res.get("items_reopened") == ["FEAT-12"] and res.get("reopen_failed") == []
    assert "status: in_progress" in rig.item()
    assert "reopened: FEAT-12" in res["line"]
