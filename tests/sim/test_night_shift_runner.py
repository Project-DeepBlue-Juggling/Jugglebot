"""Night Shift runner guards (plans/active/night-shift.md T-U2 runner part, T-U3).

`tools/night_shift.sh` is the safety layer between an unattended `claude -p` and a
box whose CAN bridge is powered overnight, so every guard is pinned here: it must
stop the run with an explicit status and must not invoke Claude. Every test drives
the real script against temporary git repositories (a bare `origin`, a `live`
checkout stand-in holding the hold file and the nightly report dir) and a fake
`claude` first on PATH that records its argv and working directory. The real
`claude` is filtered off PATH entirely, so no test can ever reach it.
"""
from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import sys
import time
import uuid
from datetime import date, timedelta
from pathlib import Path

import pytest

from mapkit.items import load_item, write_item

REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "tools" / "night_shift.sh"
SETTINGS = REPO / "tools" / "night_shift_settings.json"

_TOOL_FILES = ("tools/night_shift.sh", "tools/night_shift_settings.json",
               "tools/night_shift_preflight.py", "tools/nightly_ticker.sh",
               "tools/lib/live_session.sh", ".gitignore")
_GITCONFIG = "[user]\n\tname = t\n\temail = t@t\n"
_GREEN = "GREEN 10/10 passed, 0 failed, 0 errored, 0 xfailed, 0 skipped %sT04:20:00+11:00"

FAKE_CLAUDE = r"""#!/usr/bin/env bash
# Fake claude for the Night Shift runner tests: records argv + PWD, can sleep,
# fail, commit on main, and writes the dry-run report the protocol would.
{ for a in "$@"; do printf 'ARG %s\n' "$a"; done; printf 'PWD %s\nEND\n' "$PWD"; } >>"$FAKE_CLAUDE_LOG"
prompt=""
while (( $# )); do [[ "$1" == "-p" ]] && prompt="$2"; shift; done
[[ -n "${FAKE_CLAUDE_SLEEP:-}" ]] && sleep "$FAKE_CLAUDE_SLEEP"
if [[ "$prompt" == *--dry-run* && "${FAKE_CLAUDE_NO_REPORT:-0}" != 1 ]]; then
  [[ "$prompt" =~ ^/night-shift\ ([0-9-]+) ]] && date="${BASH_REMATCH[1]}"
  [[ "$prompt" =~ --reports\ ([^ ]+) ]] && reports="${BASH_REMATCH[1]}"
  echo "# dry run" >"$reports/dry-$date.md"
fi
[[ "${FAKE_CLAUDE_COMMIT_MAIN:-0}" == 1 ]] && git -C "$PWD" commit -q --allow-empty -m x
if [[ "${FAKE_CLAUDE_REPORT:-0}" == 1 && "$prompt" != *--dry-run* ]]; then
  [[ "$prompt" =~ ^/night-shift\ ([0-9-]+) ]] && date="${BASH_REMATCH[1]}"
  mkdir -p docs/night-shift
  printf '# Night Shift %s\n\n## Nightly\nGREEN\n\n## Cost\nunits: 0\n' "$date" >"docs/night-shift/$date.md"
  git add docs/night-shift && git commit -q -m "docs(night-shift): report $date" && git push -q origin main
fi
if [[ -n "${FAKE_CLAUDE_CLOSE_ITEM:-}" ]]; then
  sed -i 's/^status: inbox$/status: complete/' "map/$FAKE_CLAUDE_CLOSE_ITEM.md"
  git commit -q -am "close $FAKE_CLAUDE_CLOSE_ITEM" && git push -q origin main
fi
[[ "${FAKE_CLAUDE_LEAVE_BRANCH:-0}" == 1 ]] && git checkout -q -b night-shift/left-behind
[[ "${FAKE_CLAUDE_LEAVE_FILE:-0}" == 1 ]] && echo x >stray.txt
echo '{"result":"ok","total_cost_usd":0.42,"is_error":false,"num_turns":3,"permission_denials":[],"subagent_stats":{"spawned":1}}'
exit "${FAKE_CLAUDE_EXIT:-0}"
"""


def _git(*args, cwd=None, env=None):
    return subprocess.run(["git"] + list(args), cwd=cwd, env=env, check=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          universal_newlines=True).stdout.strip()


def _bug(root: Path, n: int, updated: str = "2026-10-20T00:00:00Z") -> None:
    write_item(str(root / "map" / ("bug-b%d.md" % n)),
               {"id": "JB-%d" % n, "kind": "bug", "title": "Bug %d" % n,
                "status": "inbox", "created": "2026-10-20", "updated": updated,
                "priority": "medium"}, "Body\n")


def _base_path() -> str:
    """System PATH minus every directory that holds a real `claude`."""
    keep = [d for d in os.environ.get("PATH", "").split(os.pathsep)
            if d and not os.path.exists(os.path.join(d, "claude"))]
    return os.pathsep.join(keep)


@pytest.fixture(scope="module")
def template(tmp_path_factory):
    """A bare origin with one commit on main; copied per test."""
    root = tmp_path_factory.mktemp("ns_template")
    gitcfg = root / ".gitconfig"   # git 2.25 ignores GIT_CONFIG_GLOBAL: HOME is what counts
    gitcfg.write_text(_GITCONFIG)
    env = {"PATH": _base_path(), "HOME": str(root), "GIT_CONFIG_GLOBAL": str(gitcfg),
           "GIT_CONFIG_NOSYSTEM": "1"}
    src = root / "src"
    shutil.copytree(str(REPO / "mapkit"), str(src / "mapkit"),
                    ignore=shutil.ignore_patterns("__pycache__"))
    for rel in _TOOL_FILES:
        (src / rel).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(str(REPO / rel), str(src / rel))
    (src / "map").mkdir()
    write_item(str(src / "map" / "pathway-p.md"),
               {"id": "JB-3", "kind": "pathway", "title": "P", "status": "active",
                "created": "2026-10-20", "destination": "d"}, "Body\n")
    _bug(src, 1)
    _bug(src, 2)
    for d in ("docs", "plans/active", "plans/parked", "plans/archived"):
        (src / d).mkdir(parents=True)
        (src / d / ".gitkeep").write_text("")
    (src / "GLOSSARY.md").write_text("# G\n\n## Language\n\n### Robot\n\n**Hand**: The cup.\n")
    _git("init", "-q", cwd=src, env=env)   # git 2.25 on this box: no `init -b`
    _git("symbolic-ref", "HEAD", "refs/heads/main", cwd=src, env=env)
    _git("add", "-A", cwd=src, env=env)
    _git("commit", "-q", "-m", "init", cwd=src, env=env)
    _git("clone", "-q", "--bare", str(src), str(root / "origin.git"), env=env)
    return root / "origin.git"


class NS:
    """One test's world: origin, live stand-in, fake claude, env, runner."""

    def __init__(self, tmp: Path, template: Path):
        self.tmp = tmp
        self.origin = tmp / "origin.git"
        shutil.copytree(str(template), str(self.origin))
        self.clone = tmp / "clone"
        self.reports = tmp / "reports"
        self.live = tmp / "live"
        self.nightly = self.live / "temp" / "reports" / "nightly"
        self.nightly.mkdir(parents=True)
        self.today = date.today().isoformat()
        self.green = _GREEN % self.today
        (self.nightly / "status").write_text(self.green + "\n")
        (self.nightly / "2026-10-31.md").write_text("# Nightly suite\n")
        os.symlink("2026-10-31.md", str(self.nightly / "latest.md"))
        (self.nightly / "ticker.raised").write_text(
            "%sT04:20:00+11:00 %s\n" % (self.today, self.green))
        self.bin = tmp / "bin"
        self.bin.mkdir()
        fake = self.bin / "claude"
        fake.write_text(FAKE_CLAUDE)
        fake.chmod(0o755)
        self.log = tmp / "claude.log"
        gitcfg = tmp / ".gitconfig"
        gitcfg.write_text(_GITCONFIG)
        self.env = {
            "PATH": str(self.bin) + os.pathsep + _base_path(),
            "HOME": str(tmp), "LANG": "C.UTF-8",
            "GIT_CONFIG_GLOBAL": str(gitcfg), "GIT_CONFIG_NOSYSTEM": "1",
            "NIGHT_SHIFT_CLONE": str(self.clone),
            "NIGHT_SHIFT_LIVE_REPO": str(self.live),
            "NIGHT_SHIFT_ORIGIN": str(self.origin),
            "NIGHT_SHIFT_REPORTS": str(self.reports),
            "NIGHT_SHIFT_SKIP_ENV": "1",
            "NIGHT_SHIFT_PYTHON": sys.executable,
            "ANTHROPIC_API_KEY": "test",
            "LIVE_SESSION_RE": "nsguard_absent_" + uuid.uuid4().hex,
            "FAKE_CLAUDE_LOG": str(self.log),
            # Safety net: a run that straddles midnight must not wait an hour.
            "NIGHT_SHIFT_WAIT_NIGHTLY_S": "2",
            "NIGHT_SHIFT_WAIT_NIGHTLY_POLL_S": "1",
        }

    def run(self, background=False, new_session=False, **overrides):
        env = dict(self.env)
        for k, v in overrides.items():
            if v is None:
                env.pop(k, None)
            else:
                env[k] = str(v)
        cmd = ["bash", str(RUNNER)]
        if background:
            return subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, universal_newlines=True,
                                    start_new_session=new_session)
        return subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              universal_newlines=True, timeout=60)

    def last(self, reports=None):
        return json.loads(((reports or self.reports) / "last_run.json").read_text())

    def calls(self):
        """[(argv, pwd)] per fake claude invocation."""
        if not self.log.exists():
            return []
        out, argv, pwd = [], [], None
        for line in self.log.read_text().splitlines():
            if line.startswith("ARG "):
                argv.append(line[4:])
            elif line.startswith("PWD "):
                pwd = line[4:]
            elif line == "END":
                out.append((argv, pwd))
                argv, pwd = [], None
        return out

    def push_change(self):
        """Edit bug 1 and bump its `updated` on origin's main via a throwaway clone
        (write_item skips a save whose only change is `updated`)."""
        work = self.tmp / ("work_" + uuid.uuid4().hex[:6])
        _git("clone", "-q", str(self.origin), str(work), env=self.env)
        p = work / "map" / "bug-b1.md"
        item = load_item(str(p))
        meta = dict(item.meta)
        meta["updated"] = "2026-10-30T12:00:00Z"
        write_item(str(p), meta, item.body + "More.\n")
        _git("commit", "-q", "-am", "bump", cwd=work, env=self.env)
        _git("push", "-q", "origin", "main", cwd=work, env=self.env)


@pytest.fixture
def ns(tmp_path, template):
    return NS(tmp_path, template)


@pytest.fixture
def sentinel():
    """A live process whose argv carries a unique marker for LIVE_SESSION_RE."""
    marker = "nsguard_live_" + uuid.uuid4().hex
    # Its own session: live_session() ignores processes in the runner's process
    # group, and pytest's group is the runner's too.
    proc = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(120)", marker],
                            start_new_session=True)
    try:
        yield marker
    finally:
        proc.kill()
        proc.wait()


def _check(proc, status, rc):
    assert proc.returncode == rc, (proc.stdout, proc.stderr)
    assert proc.stdout.strip().splitlines()[-1].startswith("night_shift: %s" % status), proc.stdout


def test_runner_parses():
    subprocess.run(["bash", "-n", str(RUNNER)], check=True)


def test_first_run_then_nothing_new_then_change(ns):
    # First (real) run: clones, claims the ticker, runs claude once with the deny model.
    r = ns.run()
    _check(r, "OK", 0)
    last = ns.last()
    assert last["status"] == "OK" and last["dry_run"] is False
    date = last["date"]
    calls = ns.calls()
    assert len(calls) == 1
    argv, pwd = calls[0]
    assert argv[0] == "-p" and argv[1].startswith("/night-shift %s --candidates" % date)
    joined = " ".join(argv)
    for flag in ("--permission-mode acceptEdits", "--permission-prompts none",
                 "--model fable", "--output-format json"):
        assert flag in joined
    assert "--dangerously-skip-permissions" not in joined
    # No budget fuse unless NIGHT_SHIFT_MAX_USD is set (owner, 2026-10-31).
    assert "--max-budget-usd" not in joined
    assert last["cost_recorded"] == "no-report"
    assert "--dry-run" not in argv[1]
    assert Path(pwd).resolve() == ns.clone.resolve()
    installed = ns.clone / ".claude" / "settings.local.json"
    assert installed.read_bytes() == SETTINGS.read_bytes()
    assert oct(installed.stat().st_mode & 0o777) == "0o600"
    lowered = ns.nightly / "ticker.lowered"
    assert lowered.exists() and "night-shift" in lowered.read_text()
    assert last["nightly_line"].startswith("CLAIMED GREEN")
    assert last["ticker_reraised"] is False and not (ns.nightly / "ticker.raised").exists()
    assert (ns.reports / "candidates-last.json").exists()
    assert (ns.reports / ("nightly-%s.md" % date)).read_text() == "# Nightly suite\n"
    assert last["cost_usd"] == 0.42 and last["num_turns"] == 3
    assert last["permission_denials"] == 0 and last["subagents_spawned"] == 1
    assert last["claude_exit"] == 0 and last["unpushed"] == []
    assert last["clone_branch_after"] == "main"
    assert last["report"] == str(ns.clone / "docs" / "night-shift" / ("%s.md" % date))
    head1 = last["head_after"]

    # Second run, nothing changed: no claude.
    r = ns.run()
    _check(r, "NOTHING_NEW", 0)
    assert ns.last()["status"] == "NOTHING_NEW"
    assert len(ns.calls()) == 1

    # Third run after an Item changed on origin: claude again, clone advanced.
    ns.push_change()
    r = ns.run()
    _check(r, "OK", 0)
    last = ns.last()
    assert len(ns.calls()) == 2
    assert last["head_before"] != head1 and last["head_after"] == last["head_before"]
    assert last["preflight"].startswith("NEW ")

    history = (ns.reports / "history.log").read_text().splitlines()
    assert [h.split()[1] for h in history] == ["OK", "NOTHING_NEW", "OK"]


def test_hold_file_stops_before_clone(ns):
    (ns.live / "temp" / "night-shift.hold").write_text("owner: bench work\n")
    r = ns.run()
    _check(r, "HELD", 0)
    assert ns.last()["status"] == "HELD"
    assert ns.last()["detail"] == "owner: bench work"
    assert not ns.clone.exists()
    assert ns.calls() == []


def test_live_session_defers(ns, sentinel):
    r = ns.run(LIVE_SESSION_RE=sentinel, NIGHT_SHIFT_DEFER_BUDGET_S=2,
               NIGHT_SHIFT_DEFER_POLL_S=1)
    _check(r, "DEFERRED", 0)
    last = ns.last()
    assert last["status"] == "DEFERRED" and "2s" in last["detail"]
    assert ns.calls() == [] and not ns.clone.exists()


def test_ignore_session_proceeds(ns, sentinel):
    r = ns.run(LIVE_SESSION_RE=sentinel, NIGHT_SHIFT_IGNORE_SESSION=1)
    _check(r, "OK", 0)
    assert len(ns.calls()) == 1


def test_overlapping_run_refused(ns):
    first = ns.run(background=True, FAKE_CLAUDE_SLEEP=3)
    try:
        deadline = time.monotonic() + 20
        while not ns.log.exists() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert ns.log.exists(), "first run never reached claude"
        second = ns.run()
        _check(second, "OVERLAP", 0)
    finally:
        out, err = first.communicate(timeout=60)
    assert first.returncode == 0, (out, err)
    assert len(ns.calls()) == 1
    assert ns.last()["status"] == "OK"   # the overlap never clobbers the real run's summary
    history = (ns.reports / "history.log").read_text().splitlines()
    assert sorted(h.split()[1] for h in history) == ["OK", "OVERLAP"]


def test_clone_on_a_branch_is_dirty(ns):
    _git("clone", "-q", str(ns.origin), str(ns.clone), env=ns.env)
    _git("checkout", "-q", "-b", "x", cwd=ns.clone, env=ns.env)
    r = ns.run()
    _check(r, "CLONE_DIRTY", 1)
    assert "branch=x" in ns.last()["detail"]
    assert _git("symbolic-ref", "--short", "HEAD", cwd=ns.clone, env=ns.env) == "x"
    assert ns.calls() == []


def test_clone_with_a_dirty_tree_is_dirty(ns):
    _git("clone", "-q", str(ns.origin), str(ns.clone), env=ns.env)
    (ns.clone / "stray.txt").write_text("x")
    r = ns.run()
    _check(r, "CLONE_DIRTY", 1)
    assert "stray.txt" in ns.last()["detail"]
    assert (ns.clone / "stray.txt").exists()   # never cleaned by the runner
    assert ns.calls() == []


def test_settings_reinstalled_over_a_tampered_copy(ns):
    _git("clone", "-q", str(ns.origin), str(ns.clone), env=ns.env)
    (ns.clone / ".claude").mkdir()
    (ns.clone / ".claude" / "settings.local.json").write_text('{"permissions": {}}')
    r = ns.run()
    _check(r, "OK", 0)
    assert (ns.clone / ".claude" / "settings.local.json").read_bytes() == SETTINGS.read_bytes()


def test_missing_settings_template_never_runs_claude(ns):
    work = ns.tmp / "work"
    _git("clone", "-q", str(ns.origin), str(work), env=ns.env)
    _git("rm", "-q", "tools/night_shift_settings.json", cwd=work, env=ns.env)
    _git("commit", "-q", "-m", "drop settings", cwd=work, env=ns.env)
    _git("push", "-q", "origin", "main", cwd=work, env=ns.env)
    r = ns.run()
    _check(r, "NO_SETTINGS", 1)
    assert ns.calls() == []


def test_claude_failure_is_reported(ns):
    r = ns.run(FAKE_CLAUDE_EXIT=7)
    _check(r, "CLAUDE_FAILED", 1)
    last = ns.last()
    assert last["claude_exit"] == 7
    # The claim is handed back: the morning Session must still see the nightly.
    assert last["ticker_reraised"] is True
    assert (ns.nightly / "ticker.raised").exists()
    assert not (ns.nightly / "ticker.lowered").exists()
    assert not (ns.reports / "candidates-last.json").exists()


def test_claude_missing(ns):
    r = ns.run(PATH=_base_path())
    _check(r, "CLAUDE_MISSING", 1)


def test_claude_timeout(ns):
    r = ns.run(NIGHT_SHIFT_TIMEOUT_S=1, FAKE_CLAUDE_SLEEP=3)
    _check(r, "CLAUDE_TIMEOUT", 1)
    assert ns.last()["claude_exit"] == 124


def test_unpushed_commit_on_main_is_push_rejected(ns):
    r = ns.run(FAKE_CLAUDE_COMMIT_MAIN=1)
    _check(r, "PUSH_REJECTED", 1)
    last = ns.last()
    assert len(last["unpushed"]) == 1 and len(last["unpushed"][0]) == 40
    assert not (ns.reports / "candidates-last.json").exists()


def test_dry_run(ns):
    r = ns.run(NIGHT_SHIFT_DRY_RUN=1)
    _check(r, "DRY_OK", 0)
    last = ns.last()
    date = last["date"]
    argv, _ = ns.calls()[0]
    assert argv[1].endswith("--dry-run")
    assert "--nightly \"UNCLAIMED GREEN" in argv[1]
    assert not (ns.nightly / "ticker.lowered").exists()
    assert (ns.nightly / "ticker.raised").exists()
    assert not (ns.reports / "candidates-last.json").exists()
    assert last["report"] == str(ns.reports / ("dry-%s.md" % date))
    assert last["dry_run"] is True
    assert last["cost_recorded"] == "file"
    assert "total API cost: USD 0.42" in Path(last["report"]).read_text().splitlines()[-1]

    # A dry run always exercises claude, even when nothing is new.
    r = ns.run(NIGHT_SHIFT_DRY_RUN=1, FAKE_CLAUDE_NO_REPORT=1)
    _check(r, "DRY_NO_REPORT", 1)
    assert len(ns.calls()) == 2


def test_dry_run_that_commits_is_caught(ns):
    r = ns.run(NIGHT_SHIFT_DRY_RUN=1, FAKE_CLAUDE_COMMIT_MAIN=1)
    _check(r, "PUSH_REJECTED", 1)


def test_no_api_key_stops_before_clone(ns):
    r = ns.run(ANTHROPIC_API_KEY=None)
    _check(r, "NO_API_KEY", 1)
    assert not ns.clone.exists() and ns.calls() == []


def test_default_reports_dir_inside_the_clone(ns):
    """Production layout: the reports dir lives in the clone, so a first night that
    was HELD leaves clone/temp behind and the next run must still clone."""
    reports = ns.clone / "temp" / "reports" / "night-shift"
    hold = ns.live / "temp" / "night-shift.hold"
    hold.write_text("")
    r = ns.run(NIGHT_SHIFT_REPORTS=None)
    _check(r, "HELD", 0)
    assert ns.last(reports)["detail"] == str(hold)
    assert not (ns.clone / ".git").exists()
    hold.unlink()
    r = ns.run(NIGHT_SHIFT_REPORTS=None)
    _check(r, "OK", 0)
    assert ns.last(reports)["status"] == "OK"
    assert _git("status", "--porcelain", cwd=ns.clone, env=ns.env) == ""
    assert len((reports / "history.log").read_text().splitlines()) == 2


def test_sigterm_mid_run_still_writes_a_status(ns):
    """systemd's TimeoutStartSec kill signals the whole cgroup: the run must end
    with an explicit failure status, never a missing or stale summary."""
    proc = ns.run(background=True, new_session=True, FAKE_CLAUDE_SLEEP=30)
    try:
        deadline = time.monotonic() + 20
        while not ns.log.exists() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert ns.log.exists(), "runner never reached claude"
        os.killpg(proc.pid, signal.SIGTERM)
        out, err = proc.communicate(timeout=30)
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    assert proc.returncode == 1, (out, err)
    last = ns.last()
    assert last["status"] == "INTERNAL_ERROR" and "signal" in last["detail"]


@pytest.mark.parametrize("knob, needle", [("FAKE_CLAUDE_LEAVE_BRANCH", "branch=night-shift/left-behind"),
                                          ("FAKE_CLAUDE_LEAVE_FILE", "stray.txt")])
def test_run_that_leaves_the_clone_off_main_or_dirty(ns, knob, needle):
    r = ns.run(**{knob: 1})
    _check(r, "RUN_LEFT_DIRTY", 1)
    last = ns.last()
    assert needle in last["detail"]
    assert last["ticker_reraised"] is True
    assert not (ns.reports / "candidates-last.json").exists()


def test_post_run_snapshot_is_tomorrows_baseline(ns):
    r = ns.run(FAKE_CLAUDE_CLOSE_ITEM="bug-b1")
    _check(r, "OK", 0)
    last = ns.last()
    assert len(last["main_commits"]) == 1
    assert last["main_commits"][0].endswith(" close bug-b1")
    snapshot = json.loads((ns.reports / "candidates-last.json").read_text())
    assert [i["id"] for i in snapshot["items"]] == ["JB-2"]
    # The Item closed tonight is not news tomorrow.
    r = ns.run()
    _check(r, "NOTHING_NEW", 0)
    assert len(ns.calls()) == 1


def test_stale_nightly_waits_then_proceeds(ns):
    old = (date.today() - timedelta(days=1)).isoformat()
    stale = _GREEN % old
    (ns.nightly / "status").write_text(stale + "\n")
    (ns.nightly / "ticker.raised").write_text("%sT04:20:00+11:00 %s\n" % (old, stale))
    r = ns.run()
    _check(r, "OK", 0)
    last = ns.last()
    assert last["nightly_line"].startswith("NIGHTLY_STALE CLAIMED")
    assert last["duration_s"] >= 2
    assert len(ns.calls()) == 1


def test_todays_nightly_is_not_waited_for(ns):
    r = ns.run(NIGHT_SHIFT_WAIT_NIGHTLY_S=30)
    _check(r, "OK", 0)
    last = ns.last()
    assert "NIGHTLY_STALE" not in last["nightly_line"]
    assert last["duration_s"] < 15


def test_service_tolerates_a_missing_env_file():
    unit = (REPO / "tools" / "systemd" / "jugglebot-night-shift.service").read_text()
    assert "EnvironmentFile=-%h/.config/jugglebot/night-shift.env" in unit


def test_budget_fuse_is_opt_in(ns):
    r = ns.run(NIGHT_SHIFT_MAX_USD=90)
    _check(r, "OK", 0)
    argv, _ = ns.calls()[0]
    i = argv.index("--max-budget-usd")
    assert argv[i + 1] == "90"
    r = ns.run(NIGHT_SHIFT_MAX_USD=0, NIGHT_SHIFT_DRY_RUN=1)
    _check(r, "DRY_OK", 0)
    assert "--max-budget-usd" not in ns.calls()[1][0]


def test_cost_line_is_pushed_into_the_report(ns):
    """The orchestrator cannot see total_cost_usd; the runner appends it to the
    night's report as its own docs commit on main, and the clone stays clean."""
    r = ns.run(FAKE_CLAUDE_REPORT=1)
    _check(r, "OK", 0)
    last = ns.last()
    date = last["date"]
    assert last["cost_recorded"] == "pushed"
    assert last["head_after"] == _git("rev-parse", "main", cwd=ns.origin, env=ns.env)
    report = _git("show", "main:docs/night-shift/%s.md" % date, cwd=ns.origin, env=ns.env)
    lines = report.splitlines()
    assert lines[-1] == "total API cost: USD 0.42 (runner, total_cost_usd)"
    assert lines.index("## Cost") < len(lines) - 1
    subject = _git("log", "-1", "--format=%s%n%b", "main", cwd=ns.origin, env=ns.env)
    assert subject.startswith("docs(night-shift): total API cost %s" % date)
    assert "Logbook-Entry: %s-night-shift" % date in subject
    assert _git("status", "--porcelain", cwd=ns.clone, env=ns.env) == ""
    assert _git("rev-parse", "--abbrev-ref", "HEAD", cwd=ns.clone, env=ns.env) == "main"


def test_cost_line_rejected_push_leaves_main_clean(ns):
    """If main moved between the orchestrator's push and the cost commit, the cost
    commit is dropped (never rebased, never forced) and the run is still OK."""
    hook = ns.origin / "hooks" / "pre-receive"
    hook.write_text("#!/bin/sh\nwhile read o n r; do git cat-file -p \"$n\" | "
                    "grep -q '^docs(night-shift): total API cost' && exit 1; done; exit 0\n")
    hook.chmod(0o755)
    r = ns.run(FAKE_CLAUDE_REPORT=1)
    _check(r, "OK", 0)
    last = ns.last()
    assert last["cost_recorded"] == "rejected"
    origin_main = _git("rev-parse", "main", cwd=ns.origin, env=ns.env)
    assert _git("rev-parse", "HEAD", cwd=ns.clone, env=ns.env) == origin_main
    assert _git("status", "--porcelain", cwd=ns.clone, env=ns.env) == ""
    report = _git("show", "main:docs/night-shift/%s.md" % last["date"], cwd=ns.origin, env=ns.env)
    assert "total API cost" not in report
    # tomorrow's run must not find CLONE_DIRTY
    r = ns.run(NIGHT_SHIFT_DRY_RUN=1)
    _check(r, "DRY_OK", 0)


def test_service_working_directory_exists_before_the_pre_step():
    """WorkingDirectory is entered before ExecStartPre, so the directory the
    pre-step creates cannot be it (first dry fire 2026-10-31: status=200/CHDIR)."""
    unit = (REPO / "tools" / "systemd" / "jugglebot-night-shift.service").read_text()
    assert "WorkingDirectory=%h\n" in unit
    assert "WorkingDirectory=%h/night-shift" not in unit
