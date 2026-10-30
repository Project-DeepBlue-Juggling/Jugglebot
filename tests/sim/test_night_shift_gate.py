"""tools/night_shift_gate.py: the Night Shift merge gate (plan night-shift.md § 2, T-U1).

The gate is the single enforcement point between unattended work and ``main``,
so every test makes ONE condition fail with everything else valid and asserts
HOLD with only that condition's letter. Each scenario runs both ``evaluate``
and the CLI (subprocess) and asserts they agree. Repositories are built under
tmp_path with a bare ``origin``; no ports, no fixed paths, safe under xdist.
"""
from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys

import pytest

from mapkit.items import write_item

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TOOL = os.path.join(ROOT, "tools", "night_shift_gate.py")
sys.path.insert(0, os.path.join(ROOT, "tools"))
import night_shift_gate as gate  # noqa: E402

BRANCH = "night-shift/2026-10-31-JB-7"
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


def _git(cwd, *args):
    return subprocess.run(["git"] + list(args), cwd=str(cwd), check=True,
                          capture_output=True, text=True).stdout.strip()


def _meta(kind, title, status, n=None, **extra):
    meta = {"kind": kind, "title": title, "status": status, "created": "2026-10-01",
            "updated": "2026-10-01T00:00:00Z", "priority": "medium"}
    if n is not None:
        meta = dict([("id", "JB-%d" % n)] + list(meta.items()))
    meta.update(extra)
    return dict((k, v) for k, v in meta.items() if v is not None)


def write_bug(root, **over):
    fields = dict(hardware="none", radius="local", effort="s", owner="none",
                  diagnosis="confirmed")
    fields.update(over)
    write_item(str(root / BUG), _meta("bug", "Wobble", "in_progress", 7, **fields), "Body\n")


def write_ticket(root, slug, n, parent="bug-wobble", type_="question", status="open"):
    meta = _meta("ticket", "Which way?", status, n, parent=parent, type=type_)
    write_item(str(root / "map" / ("ticket-%s.md" % slug)), meta, "Question?\n")


def set_base(root, **over):
    """Re-rate the Item on main (the base) and rebuild BRANCH on it with one src commit."""
    _git(root, "checkout", "-q", "main")
    write_bug(root, **over)
    _git(root, "add", "-A")
    _git(root, "commit", "-q", "--allow-empty", "-m", "map: rate JB-7 on main")
    _git(root, "push", "-q", "origin", "main")
    _git(root, "fetch", "-q", "origin")
    _git(root, "checkout", "-q", "-B", BRANCH, "main")
    (root / "src" / "thing.py").write_text("X = 2\n")
    commit(root, "fix(thing): X is two (JB-7)")


def commit(root, msg="edit"):
    _git(root, "add", "-A")
    _git(root, "commit", "-q", "-m", msg)
    return _git(root, "rev-parse", "HEAD")


@pytest.fixture
def repo(tmp_path):
    """A clone with origin/main and BRANCH one commit ahead (src/thing.py edited)."""
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
    (root / "Teensy_code_canbridge").mkdir()
    (root / "Teensy_code_canbridge" / "x.ino").write_text("// x\n")
    (root / "docs").mkdir()
    (root / "docs" / "CLAUDE.md").write_text("# docs copy\n")
    commit(root, "seed")
    _git(tmp_path, "clone", "-q", "--bare", str(root), str(tmp_path / "origin.git"))
    _git(root, "remote", "add", "origin", str(tmp_path / "origin.git"))
    _git(root, "fetch", "-q", "origin")
    _git(root, "push", "-q", "origin", "main")
    _git(root, "checkout", "-q", "-b", BRANCH)
    (root / "src" / "thing.py").write_text("X = 2\n")
    commit(root, "fix(thing): X is two (JB-7)")
    return root


def tip(root):
    return _git(root, "rev-parse", BRANCH)


def gate_log(root, sha=None, exit_code=0, branch=BRANCH, with_exit=True):
    p = root.parent / "gate.log"
    lines = ["HEAD=%s" % (sha or tip(root)), "BRANCH=%s" % branch, "....  [100%]"]
    if with_exit:
        lines.append("EXIT=%d" % exit_code)
    p.write_text("\n".join(lines) + "\n")
    return str(p)


def audit(root, sha=None, open_findings=0, with_count=True):
    p = root.parent / "audit.md"
    lines = ["# Audit JB-7", "", "- comment wording: narrative applied", "",
             "Audited HEAD: %s" % (sha or tip(root))]
    if with_count:
        lines.append("Behaviour-affecting findings open: %d" % open_findings)
    p.write_text("\n".join(lines) + "\n")
    return str(p)


def run(root, item="JB-7", branch=BRANCH, log=None, aud=None, base="origin/main", waive=()):
    """evaluate() and the CLI on the same inputs; asserts they agree, returns the result."""
    log = log if log is not None else gate_log(root)
    aud = aud if aud is not None else audit(root)
    res = gate.evaluate(str(root), base, branch, item, log, aud, tuple(waive))
    args = ["--repo", str(root), "--base", base, "--branch", branch, "--item", item,
            "--gate-log", log, "--audit", aud]
    for w in waive:
        args += ["--waive", w]
    cli = subprocess.run([sys.executable, TOOL] + args, capture_output=True, text=True)
    lines = cli.stdout.splitlines()
    assert lines and lines[0].split(" ")[0] == res.verdict, cli.stdout + cli.stderr
    assert lines == res.to_text().splitlines()
    assert cli.returncode == (0 if res.verdict == "MERGE" else 1)
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
    assert run(repo, log=gate_log(repo, sha=short), aud=audit(repo, sha=short)).verdict == "MERGE"


# --- (a) suite gate ---------------------------------------------------------
def test_a_missing_log(repo):
    assert_hold(run(repo, log=str(repo.parent / "nope.log")), "a")


def test_a_stale_head(repo):
    base = _git(repo, "rev-parse", "origin/main")
    assert_hold(run(repo, log=gate_log(repo, sha=base)), "a")


def test_a_too_short_prefix(repo):
    assert_hold(run(repo, log=gate_log(repo, sha=tip(repo)[:6])), "a")


def test_a_exit_nonzero(repo):
    res = run(repo, log=gate_log(repo, exit_code=1))
    assert_hold(res, "a")
    assert "[a] gate log EXIT=1 (expected 0)" in res.reasons


def test_a_no_exit_line(repo):
    assert_hold(run(repo, log=gate_log(repo, with_exit=False)), "a")


def test_a_wrong_branch_line(repo):
    assert_hold(run(repo, log=gate_log(repo, branch="night-shift/2026-10-30-JB-7")), "a")


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
    commit(repo, "map: rate JB-7 Wobble")
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
    commit(repo, "map: file ticket JB-8")
    return run(repo)


def test_e_open_question_by_stem_holds(repo):
    res = _with_ticket(repo)
    assert_hold(res, "e")
    assert "JB-8" in res.reasons[0]


def test_e_open_question_by_id_holds(repo):
    assert_hold(_with_ticket(repo, parent="JB-7"), "e")


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
    "Teensy_code_canbridge/x.ino", "config/hardware_config.yaml",
    "config/sub/deep.yaml", "GLOSSARY.md", "CLAUDE.md", "DOCUMENTATION_GUIDE.md",
    "tools/night_shift.sh", "tools/night_shift_gate.py", "tools/night_shift_preflight.py",
    "tools/night_shift_candidate.sh", "tools/night_shift_settings.json",
    "tools/lib/live_session.sh", "tools/systemd/jugglebot-night-shift.timer",
    "tools/nightly_suite.sh", ".claude/commands/night-shift.md", ".claude/settings.json",
    "mapkit/items.py", "tests/sim/test_night_shift_gate.py",
    "tests/sim/test_live_session_guard.py", "tests/sim/test_question_tickets.py",
    "tests/sim/test_sitting_wrapup.py", "run_tests.sh", "pyproject.toml",
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
    for path in ("docs/CLAUDE.md", "config/notes.md", "tools/nightly.py", "tools/other.py",
                 "tests/sim/test_mapkit_items_extra.py", "docs/run_tests.sh",
                 "src/mapkit/x.py"):
        p = repo / path
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("edited\n")
    commit(repo)
    assert run(repo).verdict == "MERGE"


def test_forbidden_paths_pinned():
    assert gate.FORBIDDEN_PATHS == (
        "Teensy_code_canbridge/*", "tools/night_shift*", "tools/lib/*", "tools/systemd/*",
        "tools/nightly_*", ".claude/*", "mapkit/*", "tests/sim/test_night_shift_*",
        "tests/sim/test_live_session_guard.py", "tests/sim/test_question_tickets.py",
        "tests/sim/test_sitting_*", "run_tests.sh", "pyproject.toml", "GLOSSARY.md",
        "CLAUDE.md", "DOCUMENTATION_GUIDE.md", "config/*.yaml")
    assert gate.WAIVABLE == ("c",)


# --- ratings the branch lowered (base vs branch) ----------------------------
def _rerate(repo, **over):
    write_bug(repo, **over)
    commit(repo, "map: rate JB-7 Wobble")
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
def test_c_branch_changes_owner(repo, old, new):
    set_base(repo, owner=old)
    res = _rerate(repo, owner=new)
    assert_hold(res, "c")
    assert res.reasons == ["[c] rating changed on branch: owner base=%s branch=%s"
                           % (old, new or "unrated")]


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
    res = run(repo, item="JB-9")
    assert_hold(res, "pre")
    assert res.reasons[0].startswith("[pre] Item not on base")


# --- --waive c (the Sitting wrap-up's hardware pass) ------------------------
def test_waive_c_merges_where_unwaived_holds(repo):
    write_bug(repo, hardware="robot")
    commit(repo, "map: rate JB-7 Wobble")
    assert_hold(run(repo), "c")
    res = run(repo, waive=("c",))
    assert res.verdict == "MERGE" and res.waived == ["c"] and res.checked["c"] is False
    assert res.to_text().splitlines()[0] == "MERGE (c waived)"
    args = ["--repo", str(repo), "--base", "origin/main", "--branch", BRANCH, "--item", "JB-7",
            "--gate-log", gate_log(repo), "--audit", audit(repo), "--waive", "c", "--json"]
    cli = subprocess.run([sys.executable, TOOL] + args, capture_output=True, text=True)
    doc = json.loads(cli.stdout)
    assert cli.returncode == 0 and doc["verdict"] == "MERGE" and doc["waived"] == ["c"]


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


@pytest.mark.parametrize("letter", ["d", "a", "x"])
def test_waive_other_letter_refused(repo, letter):
    write_bug(repo, radius="normative")
    commit(repo)
    res = gate.evaluate(str(repo), "origin/main", BRANCH, "JB-7", gate_log(repo), audit(repo),
                        (letter,))
    assert res.verdict == "HOLD" and letters(res) == ["pre"]
    args = ["--repo", str(repo), "--base", "origin/main", "--branch", BRANCH, "--item", "JB-7",
            "--gate-log", gate_log(repo), "--audit", audit(repo), "--waive", letter]
    cli = subprocess.run([sys.executable, TOOL] + args, capture_output=True, text=True)
    assert cli.returncode == 1 and cli.stdout.splitlines()[0] == "HOLD"
    assert cli.stdout.splitlines()[1].startswith("[pre]")


# --- several at once, and the preconditions ---------------------------------
def test_two_conditions_both_listed(repo):
    (repo / "CLAUDE.md").write_text("changed\n")
    commit(repo)
    res = run(repo, log=gate_log(repo, exit_code=2))
    assert res.verdict == "HOLD" and letters(res) == ["a", "f"]


def test_pre_branch_missing(repo):
    assert_hold(run(repo, branch="night-shift/2026-10-31-JB-404"), "pre")


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
    res = gate.evaluate(str(d), "origin/main", BRANCH, "JB-7", "x", "y")
    assert res.verdict == "HOLD" and letters(res) == ["pre"]


def test_pre_item_not_on_branch(repo):
    assert_hold(run(repo, item="JB-99"), "pre")


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
    args = ["--repo", str(repo), "--base", "origin/main", "--branch", BRANCH, "--item", "JB-7",
            "--gate-log", gate_log(repo, exit_code=1), "--audit", audit(repo), "--json"]
    cli = subprocess.run([sys.executable, TOOL] + args, capture_output=True, text=True)
    doc = json.loads(cli.stdout)
    assert set(doc) == {"verdict", "item", "branch", "tip", "reasons", "checked", "waived"}
    assert doc["waived"] == []
    assert doc["verdict"] == "HOLD" and doc["tip"] == tip(repo) and doc["item"] == "JB-7"
    assert doc["checked"] == {"a": False, "b": True, "c": True, "d": True, "e": True, "f": True}
    assert cli.returncode == 1


# --- the tests above bite: a gate with a check removed would MERGE ----------
def _neutered(tmp_path, monkeypatch, old, new):
    """A COPY of the gate with one check replaced, loaded from tmp (the real file is untouched)."""
    src = open(TOOL, encoding="utf-8").read()
    assert src.count(old) == 1, old
    p = tmp_path / "neutered_gate.py"
    p.write_text(src.replace(old, new))
    spec = importlib.util.spec_from_file_location("neutered_gate", str(p))
    mod = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, "neutered_gate", mod)  # dataclasses look it up
    spec.loader.exec_module(mod)
    return mod


def test_neutered_exit_check_would_merge(repo, tmp_path, monkeypatch):
    mod = _neutered(tmp_path, monkeypatch, 'elif exits[-1] != "0":', "elif False:")
    log, aud = gate_log(repo, exit_code=1), audit(repo)
    assert mod.evaluate(str(repo), "origin/main", BRANCH, "JB-7", log, aud).verdict == "MERGE"
    assert gate.evaluate(str(repo), "origin/main", BRANCH, "JB-7", log, aud).verdict == "HOLD"


def test_neutered_path_check_would_merge(repo, tmp_path, monkeypatch):
    (repo / "GLOSSARY.md").write_text("changed\n")
    commit(repo)
    mod = _neutered(tmp_path, monkeypatch, "if p and _forbidden(p)]", "if False]")
    log, aud = gate_log(repo), audit(repo)
    assert mod.evaluate(str(repo), "origin/main", BRANCH, "JB-7", log, aud).verdict == "MERGE"
    assert gate.evaluate(str(repo), "origin/main", BRANCH, "JB-7", log, aud).verdict == "HOLD"
