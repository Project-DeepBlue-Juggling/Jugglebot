"""Night Shift candidate branch (plans/active/night-shift.md T-U6).

`tools/night_shift_candidate.sh` stacks the held `night-shift/<date>-JB-n` branches
whose Item has a `## Sitting` section into one `night-shift/sitting-<date>` branch.
Every test drives the real script against a temporary bare `origin` and a working
clone; nothing touches the real repository.
"""
from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
SCRIPT = REPO / "tools" / "night_shift_candidate.sh"
_GITCONFIG = "[user]\n\tname = t\n\temail = t@t\n"
_SECTION = ("## Sitting\n\n- setup: level\n- why: w\n- do: d\n- pass: p\n"
            "- record: r\n- minutes: 5\n")


def _git(*args, cwd, env):
    return subprocess.run(["git"] + list(args), cwd=str(cwd), env=env, check=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          universal_newlines=True).stdout.strip()


def _item(n, with_section):
    body = "Robot bug body.\n\n" + (_SECTION if with_section else "")
    return ("---\nid: JB-%d\nkind: bug\ntitle: item %d\nstatus: inbox\n---\n\n%s"
            % (n, n, body))


class Fx:
    def __init__(self, tmp_path):
        self.root = tmp_path
        (tmp_path / "gitconfig").write_text(_GITCONFIG)
        (tmp_path / ".gitconfig").write_text(_GITCONFIG)
        self.env = dict(os.environ, HOME=str(tmp_path),
                        GIT_CONFIG_GLOBAL=str(tmp_path / "gitconfig"),
                        GIT_CONFIG_NOSYSTEM="1")
        self.env.pop("NIGHT_SHIFT_PYTHON", None)
        self.env["NIGHT_SHIFT_PYTHON"] = os.environ.get("PYTHON_FOR_TESTS", "") or _python()
        self.origin = tmp_path / "origin.git"
        self.seed = tmp_path / "seed"
        self.clone = tmp_path / "clone"
        _git("init", "-q", "--bare", str(self.origin), cwd=tmp_path, env=self.env)
        _git("symbolic-ref", "HEAD", "refs/heads/main", cwd=self.origin, env=self.env)
        self.seed.mkdir()
        _git("init", "-q", cwd=self.seed, env=self.env)
        _git("symbolic-ref", "HEAD", "refs/heads/main", cwd=self.seed, env=self.env)
        _git("remote", "add", "origin", str(self.origin), cwd=self.seed, env=self.env)
        shutil.copytree(str(REPO / "mapkit"), str(self.seed / "mapkit"),
                        ignore=shutil.ignore_patterns("__pycache__"))
        (self.seed / "map").mkdir()
        (self.seed / "src").mkdir()
        (self.seed / "src" / "a.txt").write_text("line1\nline2\nline3\n")
        (self.seed / "map" / "pathway-p.md").write_text(
            "---\nid: JB-100\nkind: pathway\ntitle: p\nstatus: active\n---\n\nbody\n")
        for n, sec in ((1, True), (2, True), (3, False)):
            self.item(n, sec)
        self.commit("base")
        _git("push", "-q", "origin", "main", cwd=self.seed, env=self.env)
        self.branch(1, "src/a.txt", "one\nline2\nline3\n")
        self.branch(2, "src/b.txt", "b\n")
        self.branch(3, "src/a.txt", "line1\nline2\nthree\n")
        self.branch(9, "src/z.txt", "z\n")
        _git("clone", "-q", str(self.origin), str(self.clone), cwd=tmp_path, env=self.env)

    def item(self, n, sec):
        (self.seed / "map" / ("bug-%d.md" % n)).write_text(_item(n, sec))

    def commit(self, msg):
        _git("add", "-A", cwd=self.seed, env=self.env)
        _git("commit", "-q", "-m", msg, cwd=self.seed, env=self.env)

    def branch(self, n, path, content):
        name = "night-shift/2026-10-30-JB-%d" % n
        _git("checkout", "-q", "-b", name, "main", cwd=self.seed, env=self.env)
        (self.seed / path).write_text(content)
        self.commit("work JB-%d" % n)
        _git("push", "-q", "origin", name, cwd=self.seed, env=self.env)
        _git("checkout", "-q", "main", cwd=self.seed, env=self.env)

    def add_conflicting_item(self):
        """JB-4: has a section, edits a.txt line 1 differently from JB-1."""
        self.item(4, True)
        self.commit("add JB-4 item")
        _git("push", "-q", "origin", "main", cwd=self.seed, env=self.env)
        name = "night-shift/2026-10-30-JB-4"
        _git("checkout", "-q", "-b", name, "main", cwd=self.seed, env=self.env)
        (self.seed / "src" / "a.txt").write_text("four\nline2\nline3\n")
        self.commit("work JB-4")
        _git("push", "-q", "origin", name, cwd=self.seed, env=self.env)
        _git("checkout", "-q", "main", cwd=self.seed, env=self.env)
        _git("pull", "-q", "--ff-only", "origin", "main", cwd=self.clone, env=self.env)

    def run(self, *args):
        return subprocess.run(["bash", str(SCRIPT), "--repo", str(self.clone)] + list(args),
                              env=self.env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              universal_newlines=True)

    def sha(self, ref, cwd=None):
        return _git("rev-parse", ref, cwd=cwd or self.origin, env=self.env)

    def merges(self, ref):
        return _git("log", "--merges", "--format=%s", ref, cwd=self.clone, env=self.env).splitlines()


def _python():
    import sys
    return sys.executable


@pytest.fixture
def fx(tmp_path):
    if not (REPO / "mapkit" / "sitting.py").exists():
        pytest.skip("mapkit/sitting.py not present")
    return Fx(tmp_path)


def test_script_is_valid_bash():
    assert subprocess.run(["bash", "-n", str(SCRIPT)]).returncode == 0


def test_clean_stack_pushes_candidate_and_leaves_main(fx):
    origin_main = fx.sha("main")
    local_main = fx.sha("main", cwd=fx.clone)
    r = fx.run("--date", "2026-10-31", "--push")
    assert r.returncode == 0, r.stderr
    out = r.stdout
    assert "MERGED origin/night-shift/2026-10-30-JB-1 JB-1" in out
    assert "MERGED origin/night-shift/2026-10-30-JB-2 JB-2" in out
    assert "SKIP origin/night-shift/2026-10-30-JB-3 JB-3 has no ## Sitting section" in out
    assert "SKIP origin/night-shift/2026-10-30-JB-9 no Item JB-9" in out
    assert "candidate: 2 merged, 0 held, 2 skipped -> night-shift/sitting-2026-10-31" in out
    cand = "origin/night-shift/sitting-2026-10-31"
    _git("fetch", "-q", "origin", cwd=fx.clone, env=fx.env)
    assert len(fx.merges(cand)) == 2
    assert fx.sha("main") == origin_main
    assert fx.sha("main", cwd=fx.clone) == local_main
    assert not (fx.clone / "temp" / "candidate-2026-10-31").exists()
    assert (fx.clone / "temp" / "reports" / "night-shift" / "candidate-2026-10-31.txt").exists()


def test_conflicting_branch_is_held_and_others_merge(fx):
    fx.add_conflicting_item()
    origin_main = fx.sha("main")
    r = fx.run("--date", "2026-10-31", "--push")
    assert r.returncode == 0, r.stderr
    assert "HOLD origin/night-shift/2026-10-30-JB-4 JB-4 conflict: src/a.txt" in r.stdout
    assert "MERGED origin/night-shift/2026-10-30-JB-1 JB-1" in r.stdout
    assert "MERGED origin/night-shift/2026-10-30-JB-2 JB-2" in r.stdout
    assert len(fx.merges("night-shift/sitting-2026-10-31")) == 2
    status = _git("status", "--porcelain", cwd=fx.clone, env=fx.env)
    assert "UU" not in status
    assert fx.sha("main") == origin_main


def test_without_push_nothing_on_origin(fx):
    r = fx.run("--date", "2026-10-31")
    assert r.returncode == 0, r.stderr
    remote = _git("branch", "--list", "night-shift/sitting-*", cwd=fx.origin, env=fx.env)
    assert remote == ""
    local = _git("branch", "--list", "night-shift/sitting-2026-10-31", cwd=fx.clone, env=fx.env)
    assert "night-shift/sitting-2026-10-31" in local


def test_rerun_rebuilds_from_base_not_stacked_twice(fx):
    assert fx.run("--date", "2026-10-31").returncode == 0
    first = fx.merges("night-shift/sitting-2026-10-31")
    r = fx.run("--date", "2026-10-31")
    assert r.returncode == 0, r.stderr
    second = fx.merges("night-shift/sitting-2026-10-31")
    assert len(first) == len(second) == 2
