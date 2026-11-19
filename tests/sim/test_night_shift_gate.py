"""tools/night_shift_gate.py: the Night Shift merge gate (plan night-shift.md § 2, T-U1).

The gate is the single enforcement point between unattended work and ``main``,
so every test makes ONE condition fail with everything else valid and asserts
HOLD with only that condition's letter. Each scenario runs both ``evaluate``
and the CLI (``main`` in-process) and asserts they agree. Repositories are built under
tmp_path with a bare ``origin``; no ports, no fixed paths, safe under xdist.

The gate runs the suite and the collection itself (conditions (a) and (i), FEAT-51).
Here a :class:`FakeSuite` stands in for its runner through the Python-only seam
(``evaluate(..., runner=)``, ``main(argv, runner=)``); there is deliberately no CLI or
environment seam, so the subprocess tests that reach MERGE run the REAL runner on the
template's stub ``run_tests.sh`` and stub network fence, under a lock in tmp_path.
"""
from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import os
import pwd
import shutil
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

from mapkit.items import write_item

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TOOL = os.path.join(ROOT, "tools", "night_shift_gate.py")
sys.path.insert(0, os.path.join(ROOT, "tools"))
import night_shift_gate as gate  # noqa: E402

BRANCH = "night-shift/2026-10-31-BUG-7"
BUG = "map/bug-wobble.md"


@pytest.fixture(autouse=True)
def _git_env(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", str(tmp_path / "gitconfig"))
    monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")
    for k in ("GIT_AUTHOR_NAME", "GIT_COMMITTER_NAME"):
        monkeypatch.setenv(k, "t")
    for k in ("GIT_AUTHOR_EMAIL", "GIT_COMMITTER_EMAIL"):
        monkeypatch.setenv(k, "t@example.com")
    monkeypatch.delenv("NIGHT_SHIFT_VERDICTS", raising=False)  # never the night's real directory
    monkeypatch.setenv(gate.SUITE_LOCK_ENV, str(tmp_path / "suite.lock"))


def _git(cwd, *args):
    return subprocess.run(["git"] + list(args), cwd=str(cwd), check=True,
                          capture_output=True, text=True).stdout.strip()


def _meta(kind, title, status, n=None, **extra):
    meta = {"kind": kind, "title": title, "status": status, "created": "2026-10-01",
            "updated": "2026-10-01T00:00:00Z", "priority": "medium"}
    if n is not None:
        meta = dict([("id", "%s-%d" % ({"bug": "BUG", "pathway": "PWAY", "ticket": "TKT"}[kind], n))] + list(meta.items()))
    meta.update(extra)
    return dict((k, v) for k, v in meta.items() if v is not None)


def write_bug(root, body="Body\n", **over):
    fields = dict(hardware="none", radius="local", effort="s", owner="none",
                  diagnosis="confirmed")
    fields.update(over)
    write_item(str(root / BUG), _meta("bug", "Wobble", "in_progress", 7, **fields), body)


def write_ticket(root, slug, n, parent="bug-wobble", type_="question", status="open"):
    meta = _meta("ticket", "Which way?", status, n, parent=parent, type=type_)
    write_item(str(root / "map" / ("ticket-%s.md" % slug)), meta, "Question?\n")


def set_base(root, body="Body\n", **over):
    """Re-rate the Item on main (the base) and rebuild BRANCH on it with one src commit."""
    _git(root, "checkout", "-q", "main")
    write_bug(root, body=body, **over)
    _git(root, "add", "-A")
    _git(root, "commit", "-q", "--allow-empty", "-m", "map: rate BUG-7 on main")
    _git(root, "push", "-q", "origin", "main")
    _git(root, "fetch", "-q", "origin")
    _git(root, "checkout", "-q", "-B", BRANCH, "main")
    (root / "src" / "thing.py").write_text("X = 2\n")
    commit(root, "fix(thing): X is two (BUG-7)")


def commit(root, msg="edit"):
    _git(root, "add", "-A")
    _git(root, "commit", "-q", "-m", msg)
    return _git(root, "rev-parse", "HEAD")


@pytest.fixture(scope="module")
def template(tmp_path_factory):
    """The seeded clone and its bare origin, built once; copied per test (FEAT-58)."""
    tmp_path = tmp_path_factory.mktemp("gate_template")
    mp = pytest.MonkeyPatch()  # the module-scoped twin of the autouse _git_env
    mp.setenv("HOME", str(tmp_path))
    mp.setenv("GIT_CONFIG_GLOBAL", str(tmp_path / "gitconfig"))
    mp.setenv("GIT_CONFIG_NOSYSTEM", "1")
    for k in ("GIT_AUTHOR_NAME", "GIT_COMMITTER_NAME"):
        mp.setenv(k, "t")
    for k in ("GIT_AUTHOR_EMAIL", "GIT_COMMITTER_EMAIL"):
        mp.setenv(k, "t@example.com")
    try:
        root = tmp_path / "work"
        root.mkdir()
        _git(root, "init", "-q")  # this git (2.25) has no `init -b`
        _git(root, "checkout", "-q", "-b", "main")
        (root / "map").mkdir()
        write_item(str(root / "map" / "pathway-steady.md"),
                   _meta("pathway", "Steady", "active", 1, destination="It is steady."), "Body\n")
        write_bug(root)
        (root / "src").mkdir()
        (root / "src" / "thing.py").write_text("X = 1\n")
        for name in ("GLOSSARY.md", "CLAUDE.md", "DOCUMENTATION_GUIDE.md"):
            (root / name).write_text("# %s\n" % name)
        (root / "config").mkdir()
        (root / "config" / "hardware_config.yaml").write_text("a: 1\n")
        fw = root / "ros_ws" / "src" / "jugglebot" / "Teensy_code_canbridge"
        fw.mkdir(parents=True)
        (fw / "x.ino").write_text("// x\n")
        (root / "docs").mkdir()
        (root / "docs" / "CLAUDE.md").write_text("# docs copy\n")
        # The gate running must be the base's own, so the base carries the real one and its mapkit.
        (root / "tools").mkdir()
        shutil.copy(TOOL, str(root / "tools" / "night_shift_gate.py"))
        # Stubs for the subprocess tests, which run the REAL runner (no CLI seam exists):
        # the base's network fence, the suite, and one test for the floor's collection.
        (root / "tools" / "night_shift_nonet.sh").write_text('#!/usr/bin/env bash\nexec "$@"\n')
        (root / "run_tests.sh").write_text('#!/usr/bin/env bash\necho "stub suite $*"\nexit 0\n')
        for f in ("tools/night_shift_nonet.sh", "run_tests.sh"):
            os.chmod(str(root / f), 0o755)
        (root / "tests").mkdir()
        (root / "tests" / "test_stub.py").write_text("def test_stub():\n    pass\n")
        shutil.copytree(os.path.join(ROOT, "mapkit"), str(root / "mapkit"),
                        ignore=shutil.ignore_patterns("__pycache__"))
        (root / ".gitignore").write_text("__pycache__/\n")
        commit(root, "seed")
        _git(tmp_path, "clone", "-q", "--bare", str(root), str(tmp_path / "origin.git"))
        _git(root, "remote", "add", "origin", str(tmp_path / "origin.git"))
        _git(root, "fetch", "-q", "origin")
        _git(root, "push", "-q", "origin", "main")
        _git(root, "checkout", "-q", "-b", BRANCH)
        (root / "src" / "thing.py").write_text("X = 2\n")
        commit(root, "fix(thing): X is two (BUG-7)")
    finally:
        mp.undo()
    return tmp_path


@pytest.fixture
def repo(tmp_path, template):
    """A clone with origin/main and BRANCH one commit ahead (src/thing.py edited)."""
    root = tmp_path / "work"
    shutil.copytree(str(template / "work"), str(root), symlinks=True)
    shutil.copytree(str(template / "origin.git"), str(tmp_path / "origin.git"), symlinks=True)
    _git(root, "remote", "set-url", "origin", str(tmp_path / "origin.git"))
    # `clone --bare` recorded the template's work dir as the bare origin's own remote.
    _git(tmp_path / "origin.git", "remote", "set-url", "origin", str(root))
    return root


def tip(root):
    return _git(root, "rev-parse", BRANCH)


class FakeSuite:
    """The runner the gate is handed in place of :func:`gate.run_command`.

    It records what the gate asked for (the command, the worktree and the HEAD checked
    out there) and answers with ``exit_code`` for the suite (None: a timeout) and with
    ``counts[sha]`` (default ``default_count``) for a collection."""

    def __init__(self, exit_code=0, counts=None, default_count=100, collect_rc=0,
                 collect_text=None, raise_on=None):
        self.exit_code, self.counts, self.default_count = exit_code, dict(counts or {}), default_count
        self.collect_rc, self.collect_text, self.raise_on = collect_rc, collect_text, raise_on
        self.calls = []

    def __call__(self, cmd, cwd, env, log, timeout):
        collect = "--collect-only" in cmd
        head = _git(cwd, "rev-parse", "HEAD")
        self.calls.append(SimpleNamespace(cmd=list(cmd), cwd=cwd, head=head, env=dict(env),
                                          log=log, timeout=timeout, collect=collect))
        if self.raise_on == ("collect" if collect else "suite"):
            raise RuntimeError("runner blew up")
        with open(log, "a") as fh:
            if not collect:
                fh.write("....  [100%]\n")
                return self.exit_code
            n = self.counts.get(head, self.default_count)
            fh.write(self.collect_text if self.collect_text is not None
                     else "tests/test_x.py::test_a\n\n%d tests collected in 0.01s\n" % n)
        return self.collect_rc

    def suites(self):
        return [c for c in self.calls if not c.collect]

    def collections(self):
        return [c for c in self.calls if c.collect]


def gate_log(root, exit_code=0, **kw):
    """The suite's answer: what used to be a hand-written gate log is now the runner's exit."""
    return FakeSuite(exit_code=exit_code, **kw)


def logs_dir(root):
    return str(root.parent / "gate-logs")


def evaluate(repo, base, branch, item, suite, aud, waive=(), app_log=None, module=None):
    """``gate.evaluate`` in the old argument order, with ``suite`` as its runner."""
    return (module or gate).evaluate(str(repo), base, branch, item, aud, tuple(waive), app_log,
                                     logs_dir=logs_dir(repo) if hasattr(repo, "parent") else None,
                                     runner=suite)


def cli(args, suite=None, env=None):
    """``gate.main`` in-process (the only way to hand it a runner); like subprocess.run's result."""
    out, err = io.StringIO(), io.StringIO()
    saved = dict(os.environ)
    if env is not None:
        os.environ.clear()
        os.environ.update(env)
    try:
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            rc = gate.main([str(a) for a in args], runner=suite if suite is not None else FakeSuite())
    finally:
        os.environ.clear()
        os.environ.update(saved)
    return SimpleNamespace(returncode=rc, stdout=out.getvalue(), stderr=err.getvalue())


def audit(root, sha=None, open_findings=0, with_count=True):
    p = root.parent / "audit.md"
    lines = ["# Audit BUG-7", "", "- comment wording: narrative applied", "",
             "Audited HEAD: %s" % (sha or tip(root))]
    if with_count:
        lines.append("Behaviour-affecting findings open: %d" % open_findings)
    p.write_text("\n".join(lines) + "\n")
    return str(p)


def run(root, item="BUG-7", branch=BRANCH, log=None, aud=None, base="origin/main", waive=()):
    """evaluate() and the CLI on the same inputs; asserts they agree, returns the result."""
    suite = log if log is not None else gate_log(root)
    aud = aud if aud is not None else audit(root)
    res = evaluate(root, base, branch, item, suite, aud, waive)
    args = ["--repo", str(root), "--base", base, "--branch", branch, "--item", item,
            "--audit", aud, "--record", logs_dir(root)]
    for w in waive:
        args += ["--waive", w]
    c = cli(args, suite)
    lines = [x for x in c.stdout.splitlines() if not x.startswith("# ")]
    assert lines and lines[0].split(" ")[0] == res.verdict, c.stdout + c.stderr
    assert lines == [x for x in res.to_text().splitlines() if not x.startswith("# ")]
    assert c.returncode == (0 if res.verdict == "MERGE" else 1)
    return res


def letters(res):
    return sorted(set(r[1:r.index("]")] for r in res.reasons))


def assert_hold(res, letter):
    assert res.verdict == "HOLD"
    assert letters(res) == [letter], res.reasons
    if letter != "pre":
        assert res.checked[letter] is False
        assert all(res.checked[c] for c in gate.CONDITIONS if c != letter)


# --- the happy path ---------------------------------------------------------
def test_all_valid_merges(repo):
    res = run(repo)
    assert res.verdict == "MERGE" and res.reasons == []
    assert res.tip == tip(repo) and all(res.checked.values())


def test_short_sha_prefix_matches(repo):
    short = tip(repo)[:7]
    assert run(repo, aud=audit(repo, sha=short)).verdict == "MERGE"


# --- (a) suite gate: the gate runs the suite itself (FEAT-51) ---------------
def _worktrees(root):
    return [ln for ln in _git(root, "worktree", "list", "--porcelain").splitlines()
            if ln.startswith("worktree ")]


def test_a_runs_the_suite_on_the_tip_in_a_fresh_worktree(repo):
    suite = FakeSuite()
    res = evaluate(repo, "origin/main", BRANCH, "BUG-7", suite, audit(repo))
    assert res.verdict == "MERGE", res.reasons
    (call,) = suite.suites()
    assert call.head == tip(repo)                       # the TIP, checked out detached
    assert not call.cwd.startswith(str(repo))           # not the caller's working tree
    assert call.cmd == ["/usr/bin/flock" if os.path.exists("/usr/bin/flock") else "/bin/flock",
                        os.environ[gate.SUITE_LOCK_ENV],
                        os.path.join(ROOT, "tools", "night_shift_nonet.sh"),  # the GATE's own copy
                        "./run_tests.sh", "--full"]
    assert call.timeout == gate.SUITE_TIMEOUT_S
    assert not os.path.exists(call.cwd) and len(_worktrees(repo)) == 1  # removed afterwards
    assert res.suite_exit == 0 and res.suite_log == call.log
    assert open(call.log).read().splitlines()[0] == "HEAD=%s" % tip(repo)


def test_a_exit_nonzero(repo):
    suite = gate_log(repo, exit_code=1)
    res = run(repo, log=suite)
    assert_hold(res, "a")
    assert res.reasons[0].startswith("[a] suite exit 1 on %s (expected 0); log " % tip(repo)[:12])
    assert res.suite_exit == 1 and open(res.suite_log).read().rstrip().endswith("EXIT=1")
    assert all(not os.path.exists(c.cwd) for c in suite.calls) and len(_worktrees(repo)) == 1


def test_a_timeout_holds(repo):
    res = run(repo, log=gate_log(repo, exit_code=None))
    assert_hold(res, "a")
    assert "[a] suite timed out after 3600 s" in res.reasons[0] and res.suite_exit is None


def test_a_runner_exception_holds_and_removes_the_worktree(repo):
    suite = FakeSuite(raise_on="suite")
    res = evaluate(repo, "origin/main", BRANCH, "BUG-7", suite, audit(repo))
    assert res.verdict == "HOLD" and letters(res) == ["a"]
    assert res.reasons[0].startswith("[a] gate error: RuntimeError: runner blew up")
    assert not os.path.exists(suite.suites()[0].cwd) and len(_worktrees(repo)) == 1


def test_a_missing_fence_holds(repo, monkeypatch):
    monkeypatch.setattr(gate, "NONET", "no_such_fence.sh")
    suite = FakeSuite()
    res = evaluate(repo, "origin/main", BRANCH, "BUG-7", suite, audit(repo))
    assert res.verdict == "HOLD" and "a" in letters(res) and "i" in letters(res)
    assert any("network fence" in r and "missing" in r for r in res.reasons)
    assert suite.calls == []  # nothing ran unfenced


def test_gate_log_flag_is_rejected(repo):
    """The flag is gone: argparse refuses it, so no log can be handed in (real subprocess)."""
    old = repo.parent / "gate.log"
    old.write_text("HEAD=%s\nBRANCH=%s\nEXIT=0\n" % (tip(repo), BRANCH))
    r = subprocess.run([sys.executable, TOOL, "--repo", str(repo), "--base", "origin/main",
                        "--branch", BRANCH, "--item", "BUG-7", "--gate-log", str(old),
                        "--audit", audit(repo)], capture_output=True, text=True)
    assert r.returncode == 1 and r.stdout.splitlines() == ["HOLD", "[pre] usage error (see stderr)"]
    assert "--gate-log" in r.stderr


def test_no_env_or_file_substitutes_the_result(repo, monkeypatch):
    """A fabricated old-style log where the gate writes its own, beside the repo and in the
    record dir, and the variables that would swap the interpreter or the pytest options:
    a red suite still holds, and the suite never sees those variables."""
    green = "HEAD=%s\nBRANCH=%s\n....  [100%%]\nEXIT=0\n" % (tip(repo), BRANCH)
    (repo.parent / "gate.log").write_text(green)
    os.makedirs(logs_dir(repo), exist_ok=True)
    with open(os.path.join(logs_dir(repo), "gate-%s.log" % tip(repo)[:12]), "w") as fh:
        fh.write(green)
    monkeypatch.setenv("PDJ_VENV_PY", "/bin/true")
    monkeypatch.setenv("PYTEST_ADDOPTS", "-k nothing")
    monkeypatch.setenv("BASH_ENV", str(repo.parent / "evil.sh"))
    monkeypatch.setenv("PYTHONPATH", "%s:/opt/ros/foxy/lib/python3.8/site-packages" % repo.parent)
    monkeypatch.setenv("NIGHT_SHIFT_GATE_LOG", str(repo.parent / "gate.log"))
    suite = gate_log(repo, exit_code=1)
    res = run(repo, log=suite)
    assert_hold(res, "a")
    for call in suite.calls:
        assert not ({"PDJ_VENV_PY", "PYTEST_ADDOPTS", "BASH_ENV"} & set(call.env))
        assert call.env["PYTHONPATH"] == "/opt/ros/foxy/lib/python3.8/site-packages"
        assert call.env["HOME"] == pwd.getpwuid(os.getuid()).pw_dir  # not the caller's HOME
    # the gate overwrote the planted log with its own run
    assert open(res.suite_log).read().rstrip().endswith("EXIT=1")
    src = open(TOOL, encoding="utf-8").read()
    assert "os.environ.get(" in src and src.count("os.environ.get(") == 2  # the lock, the record dir


def test_suite_env_strips_what_would_change_the_run():
    env = gate.suite_env({"PATH": "/usr/bin", "HOME": "/elsewhere", "PDJ_VENV_PY": "/bin/true",
                          "PYTEST_ADDOPTS": "-x", "PYTEST_PLUGINS": "p", "BASH_ENV": "/x",
                          "BASH_FUNC_flock%%": "() { true; }", "LD_PRELOAD": "/x.so",
                          "PYTHONPATH": "/x:/opt/ros/foxy/lib", "ROS_DISTRO": "foxy"})
    assert env == {"PATH": "/usr/bin", "HOME": pwd.getpwuid(os.getuid()).pw_dir,
                   "PYTHONPATH": "/opt/ros/foxy/lib", "ROS_DISTRO": "foxy"}
    assert "PYTHONPATH" not in gate.suite_env({"PYTHONPATH": "/tmp/evil"})


def test_run_command_closes_stdin_logs_and_returns_the_exit_code(tmp_path):
    log = str(tmp_path / "x.log")
    rc = gate.run_command(["sh", "-c", "echo out; echo err >&2; read x; echo got=[$x]; exit 3"],
                          str(tmp_path), {"PATH": "/usr/bin:/bin"}, log, 30)
    assert rc == 3
    assert open(log).read().split() == ["out", "err", "got=[]"]


def test_run_command_times_out_and_kills_the_group(tmp_path):
    log = str(tmp_path / "x.log")
    t0 = time.monotonic()
    rc = gate.run_command(["sh", "-c", "sleep 30 & sleep 30; wait"], str(tmp_path),
                          {"PATH": "/usr/bin:/bin"}, log, 0.5)
    assert rc is None and time.monotonic() - t0 < 15


# --- (i) the collected-test floor -------------------------------------------
def test_i_equal_and_greater_counts_merge(repo):
    base = _git(repo, "rev-parse", "origin/main")
    assert run(repo, log=FakeSuite(counts={base: 100, tip(repo): 100})).verdict == "MERGE"
    res = run(repo, log=FakeSuite(counts={base: 100, tip(repo): 101}))
    assert res.verdict == "MERGE" and (res.collected_base, res.collected_tip) == (100, 101)


def test_i_fewer_tests_holds_with_both_numbers(repo):
    base = _git(repo, "rev-parse", "origin/main")
    suite = FakeSuite(counts={base: 100, tip(repo): 99})
    res = run(repo, log=suite)
    assert_hold(res, "i")
    assert res.reasons == ["[i] the tip collects 99 tests, fewer than the base's 100: a branch "
                           "that removes tests is merged by a day Session, never at night"]
    heads = sorted(c.head for c in suite.collections())
    assert set(heads) == {base, tip(repo)}
    for c in suite.collections():
        assert c.cmd[2] == os.path.join(ROOT, "tools", "night_shift_nonet.sh")
        assert c.cmd[3:] == [sys.executable, "-m", "pytest", "tests/", "--collect-only", "-q",
                             "-p", "no:cacheprovider"]
        assert not os.path.exists(c.cwd)
    assert len(_worktrees(repo)) == 1


def test_i_cannot_be_waived(repo):
    base = _git(repo, "rev-parse", "origin/main")
    res = run(repo, log=FakeSuite(counts={base: 100, tip(repo): 99}), waive=("c",))
    assert res.verdict == "HOLD" and letters(res) == ["i"]


@pytest.mark.parametrize("kw", [dict(collect_rc=2), dict(collect_rc=None),
                                dict(collect_text="tests/x.py: ERROR\n"),
                                dict(collect_text="100 tests collected, 1 error in 3.1s\n")])
def test_i_collection_error_holds(repo, kw):
    res = run(repo, log=FakeSuite(**kw))
    assert_hold(res, "i")
    assert res.reasons[0].startswith("[i] cannot count the base's tests")


def test_i_base_count_is_cached_per_base_sha(repo):
    """Two branches on one base: the base is collected once, the tips once each."""
    suite = FakeSuite()
    assert evaluate(repo, "origin/main", BRANCH, "BUG-7", suite, audit(repo)).verdict == "MERGE"
    other = "night-shift/2026-10-31-BUG-7-b"
    _git(repo, "checkout", "-q", "-b", other, "origin/main")
    (repo / "src" / "thing.py").write_text("X = 3\n")
    commit(repo, "fix(thing): X is three")
    res = evaluate(repo, "origin/main", other, "BUG-7", suite, audit(repo, sha=tip_of(repo, other)))
    assert res.verdict == "MERGE", res.reasons
    base = _git(repo, "rev-parse", "origin/main")
    assert [c.head for c in suite.collections()].count(base) == 1
    assert json.load(open(os.path.join(logs_dir(repo), "collected-%s.json" % base))) == \
        {"sha": base, "count": 100}


def tip_of(root, branch):
    return _git(root, "rev-parse", branch)


def test_i_a_corrupt_cache_is_recounted(repo):
    base = _git(repo, "rev-parse", "origin/main")
    os.makedirs(logs_dir(repo), exist_ok=True)
    with open(os.path.join(logs_dir(repo), "collected-%s.json" % base), "w") as fh:
        fh.write('{"sha": "%s", "count": "lots"}' % base)
    suite = FakeSuite(counts={base: 100, tip(repo): 99})
    assert_hold(evaluate(repo, "origin/main", BRANCH, "BUG-7", suite, audit(repo)), "i")


# --- (b) audit --------------------------------------------------------------
def test_b_missing_audit(repo):
    assert_hold(run(repo, aud=str(repo.parent / "nope.md")), "b")


def test_b_stale_audited_head(repo):
    base = _git(repo, "rev-parse", "origin/main")
    assert_hold(run(repo, aud=audit(repo, sha=base)), "b")


def test_b_open_finding(repo):
    assert_hold(run(repo, aud=audit(repo, open_findings=1)), "b")


def test_b_missing_count_line(repo):
    assert_hold(run(repo, aud=audit(repo, with_count=False)), "b")


# --- (c) hardware, (d) radius: read from the BRANCH ------------------------
def test_c_hardware_robot_on_branch_tree_says_none(repo):
    write_bug(repo, hardware="robot")
    commit(repo, "map: rate BUG-7 Wobble")
    write_bug(repo, hardware="none")  # the working tree lies; the branch is what merges
    res = run(repo)
    assert_hold(res, "c")
    assert res.reasons == ["[c] hardware: robot (expected none)"]


def test_c_hardware_not_rated(repo):
    set_base(repo, hardware=None)
    res = run(repo)
    assert_hold(res, "c")
    assert res.reasons == ["[c] hardware not rated"]


def test_d_radius_normative(repo):
    write_bug(repo, radius="normative")
    commit(repo)
    assert_hold(run(repo), "d")


def test_d_radius_not_rated(repo):
    set_base(repo, radius=None)
    res = run(repo)
    assert_hold(res, "d")
    assert res.reasons == ["[d] radius not rated"]


# --- (e) open question ------------------------------------------------------
def _with_ticket(repo, **kw):
    write_ticket(repo, "which-way", 8, **kw)
    commit(repo, "map: file ticket TKT-8")
    return run(repo)


def test_e_open_question_by_stem_holds(repo):
    res = _with_ticket(repo)
    assert_hold(res, "e")
    assert "TKT-8" in res.reasons[0]


def test_e_open_question_by_id_holds(repo):
    assert_hold(_with_ticket(repo, parent="BUG-7"), "e")


def test_e_answered_question_merges(repo):
    assert _with_ticket(repo, status="complete").verdict == "MERGE"


def test_e_open_task_ticket_merges(repo):
    assert _with_ticket(repo, type_="task").verdict == "MERGE"


def test_e_question_for_another_item_merges(repo):
    assert _with_ticket(repo, parent="pathway-steady").verdict == "MERGE"


def test_e_unparseable_map_file_holds(repo):
    (repo / "map" / "ticket-broken.md").write_text("---\nkind: [ticket\n")
    commit(repo)
    assert_hold(run(repo), "e")


# --- (f) forbidden paths ----------------------------------------------------
@pytest.mark.parametrize("path", [
    "ros_ws/src/jugglebot/Teensy_code_canbridge/x.ino",
    "ros_ws/src/jugglebot/Teensy_code_platform/x.ino",
    "ros_ws/src/jugglebot/CatchingCone_code/x.cpp",
    "ros_ws/src/jugglebot/WaveCamera_ESP_code/src/x.cpp",
    "ros_ws/src/jugglebot/WaveCamera_ESP_code/include/x.h",
    "ros_ws/src/jugglebot/WaveCamera_ESP_code/platformio.ini",
    "tools/probes/imu_bench/firmware/src/x.cpp", "tools/probes/imu_bench/firmware/platformio.ini",
    "ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md",
    "ros_ws/src/jugglebot_interfaces/srv/InstallSegment.srv",
    "config/hardware_config.yaml", "config/generate_udp_protocol.py",
    "config/generated/udp_protocol.py", "config/sub/deep.yaml", "GLOSSARY.md", "CLAUDE.md", "DOCUMENTATION_GUIDE.md",
    "tools/night_shift.sh", "tools/night_shift_gate.py", "tools/night_shift_preflight.py",
    "tools/night_shift_candidate.sh", "tools/night_shift_settings.json",
    "tools/lib/live_session.sh", "tools/systemd/jugglebot-night-shift.timer",
    "tools/nightly_suite.sh", ".claude/commands/night-shift.md", ".claude/settings.json",
    "tools/night_shift_deploy.py", "tools/night_shift_nonet.sh",
    "tools/night_shift_app_suite.sh", "tests/sim/test_night_shift_gate.py",
    "tests/conftest.py", "tests/sim/conftest.py", "conftest.py", "pytest.ini", "tox.ini", "setup.cfg",
    "tests/sim/test_live_session_guard.py", "tests/sim/test_question_tickets.py",
    "tests/sim/test_sitting_debrief.py", "tools/sitting_merge.sh", "tests/sim/test_sitting_merge.py",
    "run_tests.sh", "pyproject.toml",
])
def test_f_forbidden_path_holds(repo, path):
    p = repo / path
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text("changed\n")
    commit(repo)
    res = run(repo)
    assert_hold(res, "f")
    assert res.reasons == ["[f] touches forbidden path %s" % path]


def test_f_rename_away_from_forbidden_path_holds(repo):
    _git(repo, "mv", "GLOSSARY.md", "docs/glossary-moved.md")
    commit(repo)
    res = run(repo)
    assert_hold(res, "f")
    assert "[f] touches forbidden path GLOSSARY.md" in res.reasons


def test_f_lookalike_paths_merge(repo):
    for path in ("docs/CLAUDE.md", "docs/config/notes.md", "tools/nightly.py", "tools/other.py",
                 "tools/probes/imu_bench/viewer/index.html", "tools/probes/imu_bench/README.md",
                 "tests/sim/test_mapkit_items_extra.py", "docs/run_tests.sh",
                 "src/mapkit/x.py", "ros_ws/src/jugglebot/WaveCamera_ESP_code/test/x.cpp",
                 "ros_ws/src/jugglebot/jugglebot/skill_node.py", "tests/fixtures/wave.json"):
        p = repo / path
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("edited\n")
    commit(repo)
    assert run(repo).verdict == "MERGE"


def test_forbidden_paths_pinned():
    assert gate.FORBIDDEN_PATHS == (
        "ros_ws/src/jugglebot/Teensy_code_canbridge/*",
        "ros_ws/src/jugglebot/Teensy_code_platform/*",
        "ros_ws/src/jugglebot/CatchingCone_code/*",
        "ros_ws/src/jugglebot/WaveCamera_ESP_code/src/*",
        "ros_ws/src/jugglebot/WaveCamera_ESP_code/include/*",
        "ros_ws/src/jugglebot/WaveCamera_ESP_code/platformio.ini",
        "tools/probes/imu_bench/firmware/*",
        "ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md",
        "ros_ws/src/jugglebot_interfaces/*", "config/*", "tools/night_shift*", "tools/lib/*",
        "tools/systemd/*", "tools/nightly_*", "tools/sitting_merge.sh", ".claude/*",
        "tests/sim/test_night_shift_*",
        "tests/sim/test_live_session_guard.py", "tests/sim/test_question_tickets.py",
        "tests/sim/test_sitting_*", "run_tests.sh", "pyproject.toml", "GLOSSARY.md",
        "CLAUDE.md", "DOCUMENTATION_GUIDE.md", "tests/conftest.py", "tests/*/conftest.py",
        "conftest.py", "pytest.ini", "tox.ini", "setup.cfg")
    assert gate.WAIVABLE == ("c",)


def test_every_forbidden_path_matches_a_tracked_file():
    """A pattern matching nothing freezes nothing, silently. Until 2026-11-07 the firmware
    entry was ``Teensy_code_canbridge/*``, 0 of the 71 firmware files, and the pin above
    held the same wrong string, so only a check against the real tree can catch it."""
    import fnmatch
    tracked = subprocess.run(["git", "-C", ROOT, "ls-files"], check=True, capture_output=True,
                             text=True).stdout.splitlines()
    assert len(tracked) > 1000
    dead = [e for e in gate.FORBIDDEN_PATHS if not any(fnmatch.fnmatchcase(p, e) for p in tracked)]
    # These catch a file being CREATED: a test-collection config that could drop the frozen
    # gate tests from condition (a). Absent today is what they are for.
    assert set(dead) <= {"conftest.py", "pytest.ini", "tox.ini", "setup.cfg"}


def test_every_firmware_tree_is_frozen():
    """A new firmware tree (anything with a platformio.ini) must be added to the freeze."""
    import fnmatch
    tracked = subprocess.run(["git", "-C", ROOT, "ls-files"], check=True, capture_output=True,
                             text=True).stdout.splitlines()
    inis = [p for p in tracked if os.path.basename(p) == "platformio.ini"]
    assert inis
    for ini in inis:
        assert any(fnmatch.fnmatchcase(ini, e) for e in gate.FORBIDDEN_PATHS), ini


# --- the gate running must be the base's own --------------------------------
def test_mapkit_edit_alone_passes_f_but_needs_the_app_log(repo):
    (repo / "mapkit" / "extra.py").write_text("X = 1\n")
    commit(repo)
    res = run(repo)
    assert_hold(res, "h")
    assert res.checked["f"] is True


def _app_log(root, sha=None, exit_code=0, with_head=True):
    p = root.parent / "app.log"
    lines = ["HEAD=abc", "BRANCH=main"]
    if with_head:
        lines.append("MAPKIT_HEAD=%s" % (sha or tip(root)))
    lines += ["ok", "EXIT=%d" % exit_code]
    p.write_text("\n".join(lines) + "\n")
    return str(p)


def _mapkit_branch(repo):
    (repo / "mapkit" / "extra.py").write_text("X = 1\n")
    commit(repo)


def _eval(repo, app_log):
    return evaluate(repo, "origin/main", BRANCH, "BUG-7", gate_log(repo), audit(repo),
                         (), app_log)


def test_h_mapkit_branch_merges_with_a_green_app_log(repo):
    _mapkit_branch(repo)
    assert _eval(repo, _app_log(repo)).verdict == "MERGE"


@pytest.mark.parametrize("kw", [dict(exit_code=1), dict(sha="0" * 40), dict(with_head=False)])
def test_h_bad_app_log_holds(repo, kw):
    _mapkit_branch(repo)
    assert_hold(_eval(repo, _app_log(repo, **kw)), "h")


def test_h_cannot_be_waived(repo):
    _mapkit_branch(repo)
    res = evaluate(repo, "origin/main", BRANCH, "BUG-7", gate_log(repo), audit(repo), ("c",))
    assert res.verdict == "HOLD" and "h" in letters(res)


def _move_base(repo, change):
    _git(repo, "checkout", "-q", "main")
    change()
    commit(repo, "base moved")
    _git(repo, "push", "-q", "origin", "main")
    _git(repo, "checkout", "-q", BRANCH)
    _git(repo, "merge", "-q", "--no-edit", "main")


def test_gate_refuses_a_mapkit_that_is_not_the_bases(repo):
    """What running with a branch's mapkit on the path looks like: other bytes at the base."""
    _move_base(repo, lambda: (repo / "mapkit" / "items.py").write_text("# the base's items\n"))
    res = run(repo)
    assert res.verdict == "HOLD"
    assert any("gate is not the base's copy" in r for r in res.reasons)


def test_gate_refuses_itself_when_not_the_bases(repo):
    _move_base(repo, lambda: (repo / "tools" / "night_shift_gate.py").write_text("# an older gate\n"))
    res = run(repo)
    assert res.verdict == "HOLD"
    assert any("gate is not the base's copy" in r for r in res.reasons)


def test_gate_refuses_a_file_the_base_does_not_have(repo):
    _move_base(repo, lambda: _git(repo, "rm", "-q", "mapkit/items.py"))
    res = run(repo)
    assert res.verdict == "HOLD"
    assert any("the base has no mapkit/items.py" in r for r in res.reasons)


def _extract(repo, dest):
    dest.mkdir()
    tar = subprocess.run(["git", "-C", str(repo), "archive", "origin/main",
                          "tools/night_shift_gate.py", "tools/night_shift_nonet.sh", "mapkit"],
                         check=True, capture_output=True)
    subprocess.run(["tar", "-x", "-C", str(dest)], input=tar.stdout, check=True)
    return dest / "tools" / "night_shift_gate.py"


def _from_extract(repo, tool, cwd, *extra):
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    args = [sys.executable, str(tool), "--repo", str(repo), "--base", "origin/main", "--branch", BRANCH,
            "--item", "BUG-7", "--audit", audit(repo), "--record", logs_dir(repo)] + list(extra)
    return subprocess.run(args, capture_output=True, text=True, cwd=str(cwd), env=env)


def test_the_protocols_spelling_reaches_a_real_merge(repo, tmp_path):
    """The 2g command itself: a git archive of the base, from any cwd, no PYTHONPATH, all the
    way through the byte comparison to MERGE. The protocol's earlier command did not import."""
    tool = _extract(repo, tmp_path / "gate-base")
    r = _from_extract(repo, tool, tmp_path)
    assert r.returncode == 0 and r.stdout.splitlines()[0] == "MERGE", r.stdout + r.stderr
    # ... and it ran the suite itself (the stub's own output is in the gate's log)
    log = os.path.join(logs_dir(repo), "gate-%s.log" % tip(repo)[:12])
    text = open(log).read()
    assert "stub suite --full" in text and text.rstrip().endswith("EXIT=0")
    assert "# suite: exit 0 in " in r.stdout and log in r.stdout
    assert "# collected: base 1, tip 1" in r.stdout


def test_a_branch_cannot_win_through_its_own_mapkit(repo, tmp_path):
    """A branch whose mapkit would print MERGE: the extract never loads it."""
    (repo / "mapkit" / "ids.py").write_text('print("MERGE")\nimport sys\nsys.exit(0)\n')
    commit(repo)
    tool = _extract(repo, tmp_path / "gate-base")
    r = _from_extract(repo, tool, repo)   # cwd inside the branch's tree, too
    assert r.returncode == 1 and r.stdout.startswith("HOLD"), r.stdout + r.stderr
    assert "[h]" in r.stdout


def test_a_branch_cannot_switch_off_the_frozen_tests(repo):
    (repo / "tests" / "sim").mkdir(parents=True)
    (repo / "tests" / "sim" / "conftest.py").write_text('collect_ignore = ["test_night_shift_gate.py"]\n')
    commit(repo)
    assert_hold(run(repo), "f")


@pytest.mark.parametrize("rel", ["tests/ros/conftest.py", "tests/teensy_link/conftest.py"])
def test_f_editing_an_existing_subdir_conftest_holds(repo, rel):
    """Every conftest under tests/ decides what is collected, not only tests/sim's: an edited
    tests/ros or tests/teensy_link conftest could drop or skip a frozen test as well."""
    def seed():
        (repo / rel).parent.mkdir(parents=True, exist_ok=True)
        (repo / rel).write_text("# base conftest\n")
    _move_base(repo, seed)
    (repo / rel).write_text('collect_ignore = ["test_x.py"]\n')
    commit(repo)
    res = run(repo)
    assert_hold(res, "f")
    assert res.reasons == ["[f] touches forbidden path %s" % rel]


def test_f_adding_a_new_subdir_conftest_holds(repo):
    """A conftest that does not exist at the base (tests/motion has none today) is frozen too."""
    p = repo / "tests" / "motion" / "conftest.py"
    p.parent.mkdir(parents=True)
    p.write_text("collect_ignore_glob = ['*']\n")
    commit(repo)
    res = run(repo)
    assert_hold(res, "f")
    assert res.reasons == ["[f] touches forbidden path tests/motion/conftest.py"]


def test_f_an_ordinary_test_file_beside_a_conftest_merges(repo):
    p = repo / "tests" / "motion" / "test_x.py"
    p.parent.mkdir(parents=True)
    p.write_text("def test_x():\n    pass\n")
    commit(repo)
    assert run(repo).verdict == "MERGE"


def test_the_debriefs_update_then_gate_reaches_a_merge(repo):
    """/sitting-debrief section 3: a robot Item whose base moved. The night's results are for
    the old tip and no longer count; new results on the updated tip, against the base it was
    updated to, merge with the hardware condition waived and nothing else."""
    write_bug(repo, hardware="robot")
    commit(repo, "map: rate BUG-7")
    night_tip = tip(repo)
    night_log, night_audit = gate_log(repo), audit(repo, sha=night_tip)
    _git(repo, "checkout", "-q", "main")
    (repo / "src" / "other.py").write_text("Y = 1\n")
    commit(repo, "a day's work")
    _git(repo, "push", "-q", "origin", "main")
    base = _git(repo, "rev-parse", "main")
    _git(repo, "checkout", "-q", BRANCH)

    stale = evaluate(repo, base, BRANCH, "BUG-7", night_log, night_audit, ("c",))
    assert stale.verdict == "HOLD" and "does not contain base" in stale.reasons[0]

    _git(repo, "merge", "-q", "--no-edit", base)
    still = evaluate(repo, base, BRANCH, "BUG-7", night_log, night_audit, ("c",))
    # (a) is no longer the night's log: the gate runs the suite on the updated tip itself.
    assert still.verdict == "HOLD" and letters(still) == ["b", "c"]
    assert [c.head for c in night_log.suites()] == [tip(repo)]

    fresh = evaluate(repo, base, BRANCH, "BUG-7", gate_log(repo), audit(repo), ("c",))
    assert fresh.verdict == "MERGE" and fresh.waived == ["c"], fresh.reasons
    assert "MERGE (c waived)" in fresh.to_text()


def test_the_debriefs_remote_branch_name_matches_the_nights_log(repo):
    """/sitting-debrief names the branch origin/<name>; the night's log has the local name."""
    _git(repo, "push", "-q", "origin", BRANCH)
    _git(repo, "fetch", "-q", "origin")
    res = evaluate(repo, "origin/main", "origin/" + BRANCH, "BUG-7",
                        gate_log(repo), audit(repo))
    assert res.verdict == "MERGE", res.reasons


# --- a branch of the map app ------------------------------------------------
APP_BRANCH = "night-shift/2026-11-07-BUG-7"


def _app_repo(tmp_path):
    app = tmp_path / "app"
    app.mkdir()
    _git(app, "init", "-q")
    _git(app, "checkout", "-q", "-b", "main")
    for rel in ("deploy/x.sh", "deploy-jetson.sh", ".github/ci.yml", "scripts/x.sh",
                "server/gitwriter.py", "server/nightshift.py", "server/store.py", "pyproject.toml",
                "CLAUDE.md", "frontend/package.json", "frontend/package-lock.json", "frontend/src/a.tsx"):
        p = app / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("x\n")
    commit(app, "seed")
    _git(tmp_path, "clone", "-q", "--bare", str(app), str(tmp_path / "app-origin.git"))
    _git(app, "remote", "add", "origin", str(tmp_path / "app-origin.git"))
    _git(app, "fetch", "-q", "origin")
    _git(app, "checkout", "-q", "-b", APP_BRANCH)
    return app


def _app_run(repo, app, branch=APP_BRANCH, item="BUG-7", exit_code=0, open_findings=0):
    sha = _git(app, "rev-parse", branch)
    log = app.parent / "app-gate.log"
    log.write_text("HEAD=%s\nBRANCH=%s\nok\nEXIT=%d\n" % (sha, branch, exit_code))
    aud = app.parent / "app-audit.md"
    aud.write_text("Audited HEAD: %s\nBehaviour-affecting findings open: %d\n" % (sha, open_findings))
    args = ["--repo", str(repo), "--app-repo", str(app), "--base", "origin/main", "--branch", branch,
            "--item", item, "--app-log", str(log), "--audit", str(aud)]
    res = gate.evaluate_app(str(repo), str(app), "origin/main", branch, item, str(log), str(aud))
    cli = subprocess.run([sys.executable, TOOL] + args, capture_output=True, text=True)
    assert cli.stdout.splitlines()[0] == res.verdict, cli.stdout + cli.stderr
    return res


def _app_edit(app, rel="frontend/src/a.tsx"):
    p = app / rel
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text("changed\n")
    commit(app, "fix(app): thing (BUG-7)")


def test_app_branch_merges(repo, tmp_path):
    app = _app_repo(tmp_path)
    _app_edit(app)
    res = _app_run(repo, app)
    assert res.verdict == "MERGE" and res.reasons == []


@pytest.mark.parametrize("rel", ["deploy/x.sh", "deploy-jetson.sh", ".github/ci.yml", "scripts/x.sh",
                                 "server/gitwriter.py", "server/nightshift.py", "pyproject.toml",
                                 "CLAUDE.md", "frontend/package.json", "frontend/package-lock.json"])
def test_app_forbidden_path_holds(repo, tmp_path, rel):
    app = _app_repo(tmp_path)
    _app_edit(app, rel)
    res = _app_run(repo, app)
    assert res.verdict == "HOLD" and res.reasons == ["[f] touches forbidden app path %s" % rel]


def test_app_dead_forbidden_pattern_holds(repo, tmp_path):
    app = _app_repo(tmp_path)
    _git(app, "rm", "-q", "server/gitwriter.py")
    commit(app, "drop on base")
    _git(app, "push", "-q", "origin", "HEAD:main")
    _git(app, "fetch", "-q", "origin")
    _app_edit(app)
    res = _app_run(repo, app)
    assert res.verdict == "HOLD"
    assert "[f] app forbidden pattern server/gitwriter.py matches nothing at the base" in res.reasons


def test_app_red_suite_and_open_findings_hold(repo, tmp_path):
    app = _app_repo(tmp_path)
    _app_edit(app)
    res = _app_run(repo, app, exit_code=1, open_findings=2)
    assert res.verdict == "HOLD" and letters(res) == ["a", "b"]


def _main_rating(repo, **over):
    _git(repo, "checkout", "-q", "main")
    write_bug(repo, **over)
    commit(repo, "map: rate BUG-7")
    _git(repo, "push", "-q", "origin", "main")
    _git(repo, "fetch", "-q", "origin")


@pytest.mark.parametrize("over,letter", [(dict(hardware="robot"), "c"), (dict(radius="normative"), "d")])
def test_app_item_rating_on_main_holds(repo, tmp_path, over, letter):
    app = _app_repo(tmp_path)
    _app_edit(app)
    _main_rating(repo, **over)
    res = _app_run(repo, app)
    assert res.verdict == "HOLD" and letter in letters(res)


def test_app_open_question_holds(repo, tmp_path):
    app = _app_repo(tmp_path)
    _app_edit(app)
    _git(repo, "checkout", "-q", "main")
    write_item(str(repo / "map" / "ticket-q.md"),
               _meta("ticket", "Which?", "open", 9, type="question", parent="BUG-7"), "Q\n")
    commit(repo, "map: file ticket")
    _git(repo, "push", "-q", "origin", "main")
    _git(repo, "fetch", "-q", "origin")
    res = _app_run(repo, app)
    assert res.verdict == "HOLD" and letters(res) == ["e"]


def test_app_branch_not_ahead_holds(repo, tmp_path):
    app = _app_repo(tmp_path)
    res = _app_run(repo, app)
    assert res.verdict == "HOLD" and res.reasons[0].startswith("[pre]")


def test_app_cli_refuses_a_waiver(repo, tmp_path):
    app = _app_repo(tmp_path)
    _app_edit(app)
    r = subprocess.run([sys.executable, TOOL, "--repo", str(repo), "--app-repo", str(app), "--base",
                        "origin/main", "--branch", APP_BRANCH, "--item", "BUG-7", "--app-log", "x",
                        "--audit", "y", "--waive", "c"], capture_output=True, text=True)
    assert r.returncode == 1 and "HOLD" in r.stdout


# --- ratings the branch lowered (base vs branch) ----------------------------
def _rerate(repo, **over):
    write_bug(repo, **over)
    commit(repo, "map: rate BUG-7 Wobble")
    return run(repo)


def test_c_branch_lowers_hardware(repo):
    set_base(repo, hardware="robot")
    res = _rerate(repo, hardware="none")
    assert_hold(res, "c")
    assert res.reasons == ["[c] rating changed on branch: hardware base=robot branch=none"]


def test_c_branch_removes_hardware(repo):
    res = _rerate(repo, hardware=None)
    assert_hold(res, "c")
    assert "[c] rating changed on branch: hardware base=none branch=unrated" in res.reasons


@pytest.mark.parametrize("old,new", [("none", "decision"), ("decision", "none"), ("hands", None)])
def test_g_branch_changes_owner(repo, old, new):
    set_base(repo, owner=old)
    res = _rerate(repo, owner=new)
    assert_hold(res, "g")
    assert res.checked["c"] is True   # the owner check is not part of (c)
    assert res.reasons == ["[g] rating changed on branch: owner base=%s branch=%s"
                           % (old, new or "unrated")]


def test_waive_c_does_not_rescue_g(repo):
    """A Sitting pass clears the hardware condition only: a branch that also moved
    the Item's owner (decision -> none) stays HOLD under --waive c (owner, 2026-10-31)."""
    set_base(repo, hardware="robot", owner="decision")
    res = _rerate(repo, hardware="none", owner="none")
    assert res.verdict == "HOLD" and letters(res) == ["c", "g"], res.reasons
    res = run(repo, waive=("c",))
    assert res.verdict == "HOLD" and res.waived == ["c"]
    assert res.checked["c"] is False and res.checked["g"] is False
    assert [r for r in res.reasons if r.startswith("[g]")] == [
        "[g] rating changed on branch: owner base=decision branch=none"]


def test_d_branch_lowers_radius(repo):
    set_base(repo, radius="subsystem")
    res = _rerate(repo, radius="local")
    assert_hold(res, "d")
    assert res.reasons == ["[d] rating changed on branch: radius base=subsystem branch=local"]


def test_d_branch_lowers_normative(repo):
    set_base(repo, radius="normative")
    res = _rerate(repo, radius="cross-cutting")
    assert_hold(res, "d")
    assert res.reasons == ["[d] rating changed on branch: radius base=normative branch=cross-cutting"]


def test_rating_first_appearing_on_branch_merges(repo):
    set_base(repo, hardware=None, radius=None, owner=None)
    assert _rerate(repo, hardware="none", radius="local", owner="none").verdict == "MERGE"


def test_raising_radius_on_branch_merges(repo):
    assert _rerate(repo, radius="subsystem").verdict == "MERGE"


def test_pre_item_not_on_base(repo):
    write_item(str(repo / "map" / "bug-fresh.md"),
               _meta("bug", "Fresh", "inbox", 9, hardware="none", radius="local"), "B\n")
    commit(repo)
    res = run(repo, item="BUG-9")
    assert_hold(res, "pre")
    assert res.reasons[0].startswith("[pre] Item not on base")


# --- --waive c (the Sitting debrief's hardware pass) ------------------------
def test_waive_c_merges_where_unwaived_holds(repo):
    write_bug(repo, hardware="robot")
    commit(repo, "map: rate BUG-7 Wobble")
    assert_hold(run(repo), "c")
    res = run(repo, waive=("c",))
    assert res.verdict == "MERGE" and res.waived == ["c"] and res.checked["c"] is False
    assert res.to_text().splitlines()[0] == "MERGE (c waived)"
    args = ["--repo", str(repo), "--base", "origin/main", "--branch", BRANCH, "--item", "BUG-7",
            "--audit", audit(repo), "--waive", "c", "--json"]
    c = cli(args)
    doc = json.loads(c.stdout)
    assert c.returncode == 0 and doc["verdict"] == "MERGE" and doc["waived"] == ["c"]


def test_waive_c_does_not_rescue_a(repo):
    write_bug(repo, hardware="robot")
    commit(repo)
    res = run(repo, log=gate_log(repo, exit_code=1), waive=("c",))
    assert res.verdict == "HOLD" and letters(res) == ["a", "c"]


def test_waive_c_does_not_rescue_f(repo):
    (repo / "GLOSSARY.md").write_text("changed\n")
    commit(repo)
    res = run(repo, waive=("c",))
    assert res.verdict == "HOLD" and letters(res) == ["f"]


# --- (c) a Sitting section: a fix to flown behaviour never merges at night ----
SITTING_BODY = "Body\n\n## Sitting\n\n- rig: robot\n- branch: %s\n" % BRANCH
FLOWN = "[c] changes flown behaviour (has a Sitting section)"


def test_c_sitting_section_holds_even_rated_none(repo):
    """Mutant: drop the Sitting rule from _check_c and this Item (hardware: none, a
    Sitting section, everything else valid) would MERGE at night (closeout C7)."""
    write_bug(repo, body=SITTING_BODY)
    commit(repo, "map: BUG-7 gets a Sitting section")
    res = run(repo)
    assert_hold(res, "c")
    assert res.reasons == [FLOWN]
    waived = run(repo, waive=("c",))  # the debrief, after the Sitting passed
    assert waived.verdict == "MERGE" and waived.waived == ["c"], waived.reasons


def test_c_sitting_section_at_base_cannot_be_deleted_on_branch(repo):
    set_base(repo, body=SITTING_BODY)
    write_bug(repo)  # the branch drops the section
    commit(repo, "map: BUG-7 loses its Sitting section")
    res = run(repo)
    assert_hold(res, "c")
    assert res.reasons == [FLOWN]


@pytest.mark.parametrize("letter", ["d", "a", "x"])
def test_waive_other_letter_refused(repo, letter):
    write_bug(repo, radius="normative")
    commit(repo)
    res = evaluate(repo, "origin/main", BRANCH, "BUG-7", gate_log(repo), audit(repo),
                        (letter,))
    assert res.verdict == "HOLD" and letters(res) == ["pre"]
    args = ["--repo", str(repo), "--base", "origin/main", "--branch", BRANCH, "--item", "BUG-7",
            "--audit", audit(repo), "--waive", letter]
    suite = FakeSuite()
    c = cli(args, suite)
    assert c.returncode == 1 and c.stdout.splitlines()[0] == "HOLD"
    assert c.stdout.splitlines()[1].startswith("[pre]")
    assert suite.calls == []  # a usage HOLD runs nothing


# --- several at once, and the preconditions ---------------------------------
def test_two_conditions_both_listed(repo):
    (repo / "CLAUDE.md").write_text("changed\n")
    commit(repo)
    res = run(repo, log=gate_log(repo, exit_code=2))
    assert res.verdict == "HOLD" and letters(res) == ["a", "f"]


def test_pre_branch_missing(repo):
    assert_hold(run(repo, branch="night-shift/2026-10-31-BUG-404"), "pre")


def test_pre_branch_not_ahead(repo):
    _git(repo, "branch", "night-shift/empty", "origin/main")
    assert_hold(run(repo, branch="night-shift/empty"), "pre")


def test_pre_branch_does_not_contain_base(repo):
    _git(repo, "checkout", "-q", "main")
    (repo / "src" / "other.py").write_text("Y = 1\n")
    commit(repo)
    _git(repo, "push", "-q", "origin", "main")
    _git(repo, "fetch", "-q", "origin")
    assert_hold(run(repo), "pre")


def test_pre_base_unresolvable(repo):
    assert_hold(run(repo, base="origin/nope"), "pre")


def test_pre_not_a_repo(tmp_path):
    d = tmp_path / "plain"
    d.mkdir()
    res = evaluate(d, "origin/main", BRANCH, "BUG-7", "x", "y")
    assert res.verdict == "HOLD" and letters(res) == ["pre"]


def test_pre_item_not_on_branch(repo):
    assert_hold(run(repo, item="BUG-99"), "pre")


def test_pre_item_not_an_id(repo):
    assert_hold(run(repo, item="bug-wobble"), "pre")


def test_pre_duplicate_id(repo):
    write_item(str(repo / "map" / "bug-other.md"), _meta("bug", "Other", "inbox", 7), "B\n")
    commit(repo)
    assert_hold(run(repo), "pre")


def test_usage_error_is_hold(repo):
    cli = subprocess.run([sys.executable, TOOL, "--repo", str(repo)], capture_output=True, text=True)
    assert cli.returncode == 1 and cli.stdout.splitlines()[0] == "HOLD"


def test_json_shape(repo):
    args = ["--repo", str(repo), "--base", "origin/main", "--branch", BRANCH, "--item", "BUG-7",
            "--audit", audit(repo), "--json", "--record", logs_dir(repo)]
    c = cli(args, gate_log(repo, exit_code=1))
    doc = json.loads(c.stdout)
    assert set(doc) == {"verdict", "item", "branch", "tip", "reasons", "checked", "waived",
                        "suite_log", "suite_exit", "suite_seconds", "collected_base",
                        "collected_tip"}
    assert doc["waived"] == []
    assert doc["verdict"] == "HOLD" and doc["tip"] == tip(repo) and doc["item"] == "BUG-7"
    assert doc["checked"] == {"a": False, "b": True, "c": True, "d": True, "e": True, "f": True,
                              "g": True, "h": True, "i": True}
    assert doc["suite_exit"] == 1 and doc["collected_base"] == doc["collected_tip"] == 100
    assert c.returncode == 1


# --- the tests above bite: a gate with a check removed would MERGE ----------
def _neutered(tmp_path, monkeypatch, old, new):
    """A COPY of the gate with one check replaced, loaded from tmp (the real file is untouched)."""
    src = open(TOOL, encoding="utf-8").read()
    assert src.count(old) == 1, old
    p = tmp_path / "neutered_gate.py"
    p.write_text(src.replace(old, new))
    shutil.copy(os.path.join(ROOT, "tools", "night_shift_nonet.sh"), str(tmp_path))  # its fence
    spec = importlib.util.spec_from_file_location("neutered_gate", str(p))
    mod = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, "neutered_gate", mod)  # dataclasses look it up
    spec.loader.exec_module(mod)
    mod._check_gate_is_base = lambda repo, base_sha: None   # it is, on purpose, not the base's
    return mod


def test_neutered_exit_check_would_merge(repo, tmp_path, monkeypatch):
    mod = _neutered(tmp_path, monkeypatch, "    if rc != 0:\n        return [\"[a] suite exit",
                    "    if False:\n        return [\"[a] suite exit")
    log, aud = gate_log(repo, exit_code=1), audit(repo)
    assert evaluate(repo, "origin/main", BRANCH, "BUG-7", log, aud, module=mod).verdict == "MERGE"
    assert evaluate(repo, "origin/main", BRANCH, "BUG-7", log, aud).verdict == "HOLD"


def test_neutered_floor_would_merge(repo, tmp_path, monkeypatch):
    mod = _neutered(tmp_path, monkeypatch,
                    "if not out and res.collected_tip < res.collected_base:", "if False:")
    base = _git(repo, "rev-parse", "origin/main")
    log = FakeSuite(counts={base: 100, tip(repo): 99})
    aud = audit(repo)
    assert evaluate(repo, "origin/main", BRANCH, "BUG-7", log, aud, module=mod).verdict == "MERGE"
    assert evaluate(repo, "origin/main", BRANCH, "BUG-7", log, aud).verdict == "HOLD"


def test_neutered_path_check_would_merge(repo, tmp_path, monkeypatch):
    (repo / "GLOSSARY.md").write_text("changed\n")
    commit(repo)
    mod = _neutered(tmp_path, monkeypatch, "if p and _forbidden(p)]", "if False]")
    log, aud = gate_log(repo), audit(repo)
    assert evaluate(repo, "origin/main", BRANCH, "BUG-7", log, aud, module=mod).verdict == "MERGE"
    assert evaluate(repo, "origin/main", BRANCH, "BUG-7", log, aud).verdict == "HOLD"


# --- item-id-per-kind P1 (plan section 5, T-U6): both ID forms, prefix must match kind ---

def _write_with_id(root, sid, kind="bug", rel=BUG, title="Wobble", **extra):
    fields = dict(hardware="none", radius="local", effort="s", owner="none")
    if kind == "bug":
        fields["diagnosis"] = "confirmed"
    fields.update(extra)
    meta = dict([("id", sid)] + list(_meta(kind, title, "in_progress", None, **fields).items()))
    write_item(str(root / rel), meta, "Body\n")


def _rebase_on(root, *writes):
    """Write ``(sid, kind, rel)`` Items on main, push, and rebuild BRANCH one src commit ahead."""
    _git(root, "checkout", "-q", "main")
    for sid, kind, rel in writes:
        _write_with_id(root, sid, kind, rel)
    _git(root, "add", "-A")
    _git(root, "commit", "-q", "--allow-empty", "-m", "map: per-kind ids on main")
    _git(root, "push", "-q", "origin", "main")
    _git(root, "fetch", "-q", "origin")
    _git(root, "checkout", "-q", "-B", BRANCH, "main")
    (root / "src" / "thing.py").write_text("X = 2\n")
    commit(root, "fix(thing): X is two")


FEAT = "map/feature-tilt.md"


def test_per_kind_item_merges(repo):
    """T-U6: a per-kind Item passes the gate, typed loosely or canonically (sites _find_item, evaluate)."""
    _rebase_on(repo, ("BUG-7", "bug", BUG))
    assert run(repo, item="BUG-7").verdict == "MERGE"
    assert run(repo, item="bug7").verdict == "MERGE"


def test_same_number_other_kind_is_another_item(repo):
    """T-U6: BUG-7 and FEAT-7 are two Items: each --item finds its own file, never the other's."""
    _rebase_on(repo, ("BUG-7", "bug", BUG), ("FEAT-7", "feature", FEAT))
    res = run(repo, item="FEAT-7")
    assert res.verdict == "MERGE", res.reasons
    assert run(repo, item="BUG-7").verdict == "MERGE"
    res = run(repo, item="BUG-3")
    assert_hold(res, "pre")
    assert "no map/ file on the branch carries id BUG-3" in res.reasons[0]


def test_prefix_contradicting_kind_holds_naming_both(repo):
    """T-U6: an ID whose prefix names another kind than its Item's is a [pre] HOLD naming both."""
    _rebase_on(repo, ("FEAT-7", "bug", BUG))
    res = run(repo, item="FEAT-7")
    assert_hold(res, "pre")
    assert "FEAT-7 names a feature, but map/bug-wobble.md is a bug" in res.reasons[0], res.reasons


def test_e_question_parent_compares_whole_ids(repo):
    """T-U6 (the :501 site): an open question for FEAT-7 does not hold BUG-7; one for BUG-7 does."""
    _rebase_on(repo, ("BUG-7", "bug", BUG), ("FEAT-7", "feature", FEAT))
    meta = dict([("id", "TKT-8")] + list(_meta("ticket", "Which way?", "open", None,
                                                 parent="FEAT-7", type="question").items()))
    write_item(str(repo / "map" / "ticket-which-way.md"), meta, "Question?\n")
    commit(repo, "map: file ticket TKT-8")
    assert run(repo, item="BUG-7").verdict == "MERGE"
    assert_hold(run(repo, item="FEAT-7"), "e")
    meta["parent"] = "bug7"
    write_item(str(repo / "map" / "ticket-which-way.md"), meta, "Question?\n")
    commit(repo, "map: re-parent TKT-8")
    assert_hold(run(repo, item="BUG-7"), "e")


def test_old_form_item_is_a_pre_hold(repo):
    """T-U6: an old-form ``--item`` (any spelling; fictional prefix here) is no Item ID: [pre] HOLD."""
    _rebase_on(repo, ("BUG-7", "bug", BUG))
    for text in ("OLD-7", "old-07", "old7"):
        res = run(repo, item=text)
        assert_hold(res, "pre")
        assert "is not an Item ID" in res.reasons[0], res.reasons


def test_app_gate_takes_per_kind_ids_and_refuses_a_mismatch(repo, tmp_path):
    """T-U6 (the evaluate_app site): the app gate reads the Item in either form, and a
    prefix that contradicts the kind is a [pre] HOLD."""
    _rebase_on(repo, ("BUG-7", "bug", BUG), ("BUG-9", "feature", FEAT))
    app = _app_repo(tmp_path)
    _app_edit(app)
    res = _app_run(repo, app, item="BUG-7")
    assert res.verdict == "MERGE", res.reasons
    res = _app_run(repo, app, item="BUG-9")
    assert res.verdict == "HOLD"
    assert "BUG-9 names a bug, but map/feature-tilt.md is a feature" in res.reasons[0], res.reasons


# --- the verdict record (--record, closeout C1) ------------------------------
RECORD_FIELDS = {"schema", "verdict", "item", "branch", "tip", "base_ref", "base_sha", "waived",
                 "reasons", "gate_sha256", "gate_log_sha256", "audit_sha256", "written_at",
                 "suite_log", "suite_exit", "suite_seconds", "collected_base", "collected_tip"}


def _sha256(path):
    import hashlib
    with open(str(path), "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def _cli(repo, *extra, log=None, aud=None, env=None):
    args = ["--repo", str(repo), "--base", "origin/main", "--branch", BRANCH,
            "--item", "BUG-7", "--audit", aud or audit(repo)]
    return cli(args + list(extra), log or gate_log(repo), env=env)


def test_record_written_for_a_merge_with_every_field(repo, tmp_path):
    d = tmp_path / "verdicts"
    log, aud = gate_log(repo), audit(repo)
    r = _cli(repo, "--record", str(d), log=log, aud=aud)
    assert r.returncode == 0 and r.stdout.splitlines()[0] == "MERGE", r.stdout + r.stderr
    rec = json.loads((d / (tip(repo) + ".MERGE.json")).read_text())
    assert set(rec) == RECORD_FIELDS
    assert rec["schema"] == 1 and rec["verdict"] == "MERGE" and rec["item"] == "BUG-7"
    assert rec["branch"] == BRANCH and rec["tip"] == tip(repo) and rec["reasons"] == []
    assert rec["base_ref"] == "origin/main" and rec["base_sha"] == _git(repo, "rev-parse", "origin/main")
    assert rec["waived"] == [] and rec["written_at"].endswith("+00:00")
    assert rec["gate_sha256"] == _sha256(TOOL)
    suite_log = str(d / ("gate-%s.log" % tip(repo)[:12]))
    assert rec["suite_log"] == suite_log and rec["suite_exit"] == 0
    assert isinstance(rec["suite_seconds"], float) and rec["suite_seconds"] >= 0
    assert rec["collected_base"] == 100 and rec["collected_tip"] == 100
    assert rec["gate_log_sha256"] == _sha256(suite_log) and rec["audit_sha256"] == _sha256(aud)
    assert suite_log in r.stdout  # named in the output too
    # atomic: the temp files are renamed into place; besides the record only the gate's own
    # logs and the base-count cache are left in the directory
    base = _git(repo, "rev-parse", "origin/main")
    assert sorted(os.listdir(str(d))) == sorted([
        tip(repo) + ".MERGE.json", "gate-%s.log" % tip(repo)[:12], "collect-%s.log" % tip(repo)[:12],
        "collect-%s.log" % base[:12], "collected-%s.json" % base])


def test_record_written_for_a_hold_with_its_reasons(repo, tmp_path):
    d = tmp_path / "verdicts"
    r = _cli(repo, "--record", str(d), log=gate_log(repo, exit_code=1))
    assert r.returncode == 1 and r.stdout.startswith("HOLD")
    rec = json.loads((d / (tip(repo) + ".HOLD.json")).read_text())
    assert rec["verdict"] == "HOLD" and [x[:3] for x in rec["reasons"]] == ["[a]"]


def test_record_dir_defaults_to_the_env_then_home(repo, tmp_path):
    env = dict(os.environ)
    env["NIGHT_SHIFT_VERDICTS"] = str(tmp_path / "from-env")
    assert _cli(repo, "--record", env=env).returncode == 0
    assert (tmp_path / "from-env" / (tip(repo) + ".MERGE.json")).exists()
    env.pop("NIGHT_SHIFT_VERDICTS")
    assert _cli(repo, "--record", "", env=env).returncode == 0   # HOME is tmp_path (autouse)
    assert (tmp_path / ".local" / "state" / "jugglebot" / "verdicts" / (tip(repo) + ".MERGE.json")).exists()


def test_a_merge_whose_record_cannot_be_written_is_a_hold(repo, tmp_path):
    blocker = tmp_path / "not-a-dir"
    blocker.write_text("x\n")
    r = _cli(repo, "--record", str(blocker / "verdicts"))
    assert r.returncode == 1 and r.stdout.startswith("HOLD"), r.stdout
    assert "could not write the verdict record" in r.stdout


def test_record_changes_no_verdict(repo, tmp_path):
    """The same inputs with and without --record print the same verdict and reasons."""
    for kw in ({}, {"exit_code": 1}):
        log = gate_log(repo, **kw)
        plain = _cli(repo, log=log)
        recorded = _cli(repo, "--record", str(tmp_path / "v"), log=log)
        # the reasons name the suite log, which lives in the record directory
        verdict = lambda r: [x.split("; log ")[0] for x in r.stdout.splitlines()  # noqa: E731
                             if not x.startswith("# ")]
        assert (plain.returncode, verdict(plain)) == (recorded.returncode, verdict(recorded))


def test_a_later_hold_never_replaces_the_merge_record(repo, tmp_path):
    """A resume or re-check that HOLDs an already-merged tip keeps its MERGE record."""
    d = tmp_path / "verdicts"
    assert _cli(repo, "--record", str(d)).returncode == 0
    assert _cli(repo, "--record", str(d), log=gate_log(repo, exit_code=1)).returncode == 1
    records = sorted(n for n in os.listdir(str(d)) if n.startswith(tip(repo)))
    assert records == [tip(repo) + ".HOLD.json", tip(repo) + ".MERGE.json"]
    assert json.loads((d / (tip(repo) + ".MERGE.json")).read_text())["verdict"] == "MERGE"
