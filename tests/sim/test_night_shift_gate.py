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
import shutil
import subprocess
import sys

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
    fw = root / "ros_ws" / "src" / "jugglebot" / "Teensy_code_canbridge"
    fw.mkdir(parents=True)
    (fw / "x.ino").write_text("// x\n")
    (root / "docs").mkdir()
    (root / "docs" / "CLAUDE.md").write_text("# docs copy\n")
    # The gate running must be the base's own, so the base carries the real one and its mapkit.
    (root / "tools").mkdir()
    shutil.copy(TOOL, str(root / "tools" / "night_shift_gate.py"))
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
    lines = ["# Audit BUG-7", "", "- comment wording: narrative applied", "",
             "Audited HEAD: %s" % (sha or tip(root))]
    if with_count:
        lines.append("Behaviour-affecting findings open: %d" % open_findings)
    p.write_text("\n".join(lines) + "\n")
    return str(p)


def run(root, item="BUG-7", branch=BRANCH, log=None, aud=None, base="origin/main", waive=()):
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
    assert_hold(run(repo, log=gate_log(repo, branch="night-shift/2026-10-30-BUG-7")), "a")


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
    "tests/sim/test_sitting_debrief.py", "run_tests.sh", "pyproject.toml",
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
        "ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md",
        "ros_ws/src/jugglebot_interfaces/*", "config/*", "tools/night_shift*", "tools/lib/*",
        "tools/systemd/*", "tools/nightly_*", ".claude/*", "tests/sim/test_night_shift_*",
        "tests/sim/test_live_session_guard.py", "tests/sim/test_question_tickets.py",
        "tests/sim/test_sitting_*", "run_tests.sh", "pyproject.toml", "GLOSSARY.md",
        "CLAUDE.md", "DOCUMENTATION_GUIDE.md", "tests/conftest.py", "tests/sim/conftest.py",
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
    return gate.evaluate(str(repo), "origin/main", BRANCH, "BUG-7", gate_log(repo), audit(repo),
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
    res = gate.evaluate(str(repo), "origin/main", BRANCH, "BUG-7", gate_log(repo), audit(repo), ("c",))
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
                          "tools/night_shift_gate.py", "mapkit"], check=True, capture_output=True)
    subprocess.run(["tar", "-x", "-C", str(dest)], input=tar.stdout, check=True)
    return dest / "tools" / "night_shift_gate.py"


def _from_extract(repo, tool, cwd, *extra):
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    args = [sys.executable, str(tool), "--repo", str(repo), "--base", "origin/main", "--branch", BRANCH,
            "--item", "BUG-7", "--gate-log", gate_log(repo), "--audit", audit(repo)] + list(extra)
    return subprocess.run(args, capture_output=True, text=True, cwd=str(cwd), env=env)


def test_the_protocols_spelling_reaches_a_real_merge(repo, tmp_path):
    """The 2g command itself: a git archive of the base, from any cwd, no PYTHONPATH, all the
    way through the byte comparison to MERGE. The protocol's earlier command did not import."""
    tool = _extract(repo, tmp_path / "gate-base")
    r = _from_extract(repo, tool, tmp_path)
    assert r.returncode == 0 and r.stdout.strip() == "MERGE", r.stdout + r.stderr


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


def test_the_debriefs_update_then_gate_reaches_a_merge(repo):
    """/sitting-debrief section 3: a robot Item whose base moved. The night's results are for
    the old tip and no longer count; new results on the updated tip, against the base it was
    updated to, merge with the hardware condition waived and nothing else."""
    write_bug(repo, hardware="robot")
    commit(repo, "map: rate BUG-7")
    night_tip = tip(repo)
    night_log, night_audit = gate_log(repo, sha=night_tip), audit(repo, sha=night_tip)
    _git(repo, "checkout", "-q", "main")
    (repo / "src" / "other.py").write_text("Y = 1\n")
    commit(repo, "a day's work")
    _git(repo, "push", "-q", "origin", "main")
    base = _git(repo, "rev-parse", "main")
    _git(repo, "checkout", "-q", BRANCH)

    stale = gate.evaluate(str(repo), base, BRANCH, "BUG-7", night_log, night_audit, ("c",))
    assert stale.verdict == "HOLD" and "does not contain base" in stale.reasons[0]

    _git(repo, "merge", "-q", "--no-edit", base)
    still = gate.evaluate(str(repo), base, BRANCH, "BUG-7", night_log, night_audit, ("c",))
    assert still.verdict == "HOLD" and letters(still) == ["a", "b", "c"]

    fresh = gate.evaluate(str(repo), base, BRANCH, "BUG-7", gate_log(repo), audit(repo), ("c",))
    assert fresh.verdict == "MERGE" and fresh.waived == ["c"], fresh.reasons
    assert "MERGE (c waived)" in fresh.to_text()


def test_the_debriefs_remote_branch_name_matches_the_nights_log(repo):
    """/sitting-debrief names the branch origin/<name>; the night's log has the local name."""
    _git(repo, "push", "-q", "origin", BRANCH)
    _git(repo, "fetch", "-q", "origin")
    res = gate.evaluate(str(repo), "origin/main", "origin/" + BRANCH, "BUG-7",
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
            "--item", item, "--gate-log", str(log), "--audit", str(aud)]
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
                        "origin/main", "--branch", APP_BRANCH, "--item", "BUG-7", "--gate-log", "x",
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
    res = gate.evaluate(str(repo), "origin/main", BRANCH, "BUG-7", gate_log(repo), audit(repo),
                        (letter,))
    assert res.verdict == "HOLD" and letters(res) == ["pre"]
    args = ["--repo", str(repo), "--base", "origin/main", "--branch", BRANCH, "--item", "BUG-7",
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
    res = gate.evaluate(str(d), "origin/main", BRANCH, "BUG-7", "x", "y")
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
            "--gate-log", gate_log(repo, exit_code=1), "--audit", audit(repo), "--json"]
    cli = subprocess.run([sys.executable, TOOL] + args, capture_output=True, text=True)
    doc = json.loads(cli.stdout)
    assert set(doc) == {"verdict", "item", "branch", "tip", "reasons", "checked", "waived"}
    assert doc["waived"] == []
    assert doc["verdict"] == "HOLD" and doc["tip"] == tip(repo) and doc["item"] == "BUG-7"
    assert doc["checked"] == {"a": False, "b": True, "c": True, "d": True, "e": True, "f": True,
                              "g": True, "h": True}
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
    mod._check_gate_is_base = lambda repo, base_sha: None   # it is, on purpose, not the base's
    return mod


def test_neutered_exit_check_would_merge(repo, tmp_path, monkeypatch):
    mod = _neutered(tmp_path, monkeypatch, 'elif exits[-1] != "0":', "elif False:")
    log, aud = gate_log(repo, exit_code=1), audit(repo)
    assert mod.evaluate(str(repo), "origin/main", BRANCH, "BUG-7", log, aud).verdict == "MERGE"
    assert gate.evaluate(str(repo), "origin/main", BRANCH, "BUG-7", log, aud).verdict == "HOLD"


def test_neutered_path_check_would_merge(repo, tmp_path, monkeypatch):
    (repo / "GLOSSARY.md").write_text("changed\n")
    commit(repo)
    mod = _neutered(tmp_path, monkeypatch, "if p and _forbidden(p)]", "if False]")
    log, aud = gate_log(repo), audit(repo)
    assert mod.evaluate(str(repo), "origin/main", BRANCH, "BUG-7", log, aud).verdict == "MERGE"
    assert gate.evaluate(str(repo), "origin/main", BRANCH, "BUG-7", log, aud).verdict == "HOLD"


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
                 "reasons", "gate_sha256", "gate_log_sha256", "audit_sha256", "written_at"}


def _sha256(path):
    import hashlib
    with open(str(path), "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def _cli(repo, *extra, log=None, aud=None, env=None):
    args = [sys.executable, TOOL, "--repo", str(repo), "--base", "origin/main", "--branch", BRANCH,
            "--item", "BUG-7", "--gate-log", log or gate_log(repo), "--audit", aud or audit(repo)]
    return subprocess.run(args + list(extra), capture_output=True, text=True, env=env)


def test_record_written_for_a_merge_with_every_field(repo, tmp_path):
    d = tmp_path / "verdicts"
    log, aud = gate_log(repo), audit(repo)
    r = _cli(repo, "--record", str(d), log=log, aud=aud)
    assert r.returncode == 0 and r.stdout.strip() == "MERGE", r.stdout + r.stderr
    rec = json.loads((d / (tip(repo) + ".json")).read_text())
    assert set(rec) == RECORD_FIELDS
    assert rec["schema"] == 1 and rec["verdict"] == "MERGE" and rec["item"] == "BUG-7"
    assert rec["branch"] == BRANCH and rec["tip"] == tip(repo) and rec["reasons"] == []
    assert rec["base_ref"] == "origin/main" and rec["base_sha"] == _git(repo, "rev-parse", "origin/main")
    assert rec["waived"] == [] and rec["written_at"].endswith("+00:00")
    assert rec["gate_sha256"] == _sha256(TOOL)
    assert rec["gate_log_sha256"] == _sha256(log) and rec["audit_sha256"] == _sha256(aud)
    # atomic: the temp file is renamed into place, nothing else is left in the directory
    assert os.listdir(str(d)) == [tip(repo) + ".json"]


def test_record_written_for_a_hold_with_its_reasons(repo, tmp_path):
    d = tmp_path / "verdicts"
    r = _cli(repo, "--record", str(d), log=gate_log(repo, exit_code=1))
    assert r.returncode == 1 and r.stdout.startswith("HOLD")
    rec = json.loads((d / (tip(repo) + ".json")).read_text())
    assert rec["verdict"] == "HOLD" and [x[:3] for x in rec["reasons"]] == ["[a]"]


def test_record_dir_defaults_to_the_env_then_home(repo, tmp_path):
    env = dict(os.environ)
    env["NIGHT_SHIFT_VERDICTS"] = str(tmp_path / "from-env")
    assert _cli(repo, "--record", env=env).returncode == 0
    assert (tmp_path / "from-env" / (tip(repo) + ".json")).exists()
    env.pop("NIGHT_SHIFT_VERDICTS")
    assert _cli(repo, "--record", "", env=env).returncode == 0   # HOME is tmp_path (autouse)
    assert (tmp_path / ".local" / "state" / "jugglebot" / "verdicts" / (tip(repo) + ".json")).exists()


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
        assert (plain.returncode, plain.stdout) == (recorded.returncode, recorded.stdout)
