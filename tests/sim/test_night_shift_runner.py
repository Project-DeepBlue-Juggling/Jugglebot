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
               "tools/night_shift_stub_report.py", "tools/night_shift_landed.py",
               "tools/night_shift_preflight.py", "tools/nightly_ticker.sh",
               "tools/lib/live_session.sh", ".gitignore")
_GITCONFIG = "[user]\n\tname = t\n\temail = t@t\n"
_GREEN = "GREEN 10/10 passed, 0 failed, 0 errored, 0 xfailed, 0 skipped %sT04:20:00+11:00"

FAKE_CLAUDE = r"""#!/usr/bin/env bash
# Fake claude for the Night Shift runner tests: records argv + PWD, can sleep,
# fail, commit on main, and writes the dry-run report the protocol would.
{ for a in "$@"; do printf 'ARG %s\n' "$a"; done
  printf 'AUTH key=%s token=%s\n' "${ANTHROPIC_API_KEY:+set}" "${CLAUDE_CODE_OAUTH_TOKEN:+set}"
  printf 'BGWAIT %s\n' "${CLAUDE_CODE_PRINT_BG_WAIT_CEILING_MS:-unset}"
  printf 'PWD %s\nEND\n' "$PWD"; } >>"$FAKE_CLAUDE_LOG"
if [[ " $* " == *" --resume "* ]]; then
  if [[ "${FAKE_CLAUDE_FIX_ON_RESUME:-0}" == 1 ]]; then
    git -C "$PWD" checkout -q -f main; git -C "$PWD" clean -qfd
    FAKE_CLAUDE_REPORT=1; FAKE_CLAUDE_LEAVE_BRANCH=0; FAKE_CLAUDE_LEAVE_FILE=0
  fi
  [[ -n "${FAKE_CLAUDE_RESUME_EXIT:-}" ]] && FAKE_CLAUDE_EXIT="$FAKE_CLAUDE_RESUME_EXIT"
fi
if [[ "${FAKE_CLAUDE_CREDIT_OUT:-0}" == 1 && -n "${ANTHROPIC_API_KEY:-}" ]]; then
  echo '{"type":"result","is_error":true,"api_error_status":400,"result":"Credit balance is too low","session_id":"sess-123","total_cost_usd":1.5,"num_turns":9}'
  echo 'warning: claude.ai connectors are disabled' >&2
  exit 1
fi
prompt=""
while (( $# )); do [[ "$1" == "-p" ]] && prompt="$2"; shift; done
[[ -n "${FAKE_CLAUDE_SLEEP:-}" ]] && sleep "$FAKE_CLAUDE_SLEEP"
if [[ "$prompt" == *--dry-run* && "${FAKE_CLAUDE_NO_REPORT:-0}" != 1 ]]; then
  [[ "$prompt" =~ ^/night-shift\ ([0-9-]+) ]] && date="${BASH_REMATCH[1]}"
  [[ "$prompt" =~ --reports\ ([^ ]+) ]] && reports="${BASH_REMATCH[1]}"
  echo "# dry run" >"$reports/dry-$date.md"
fi
[[ "${FAKE_CLAUDE_COMMIT_MAIN:-0}" == 1 ]] && git -C "$PWD" commit -q --allow-empty -m x
[[ -n "${FAKE_CLAUDE_START:-}" ]] && { nohup "$FAKE_CLAUDE_START" >/dev/null 2>&1 & disown; sleep 0.3; }
# Landed-check shapes (closeout C1): an ungated code push, a gated merge with its
# MERGE record in the runner's $NIGHT_SHIFT_VERDICTS, and an evil merge of a gated tip.
if [[ "${FAKE_CLAUDE_PUSH_CODE:-0}" == 1 ]]; then
  echo "Y = 1" >code.py && git add code.py && git commit -q -m "fix: ungated" && git push -q origin main
fi
if [[ "${FAKE_CLAUDE_GATED_MERGE:-0}" == 1 || "${FAKE_CLAUDE_EVIL_MERGE:-0}" == 1 ]]; then
  git checkout -q -b night-shift/gated && echo "X = 1" >gated.py && git add gated.py && git commit -q -m "fix: gated"
  tip="$(git rev-parse HEAD)"; mkdir -p "$NIGHT_SHIFT_VERDICTS"
  printf '{"schema": 1, "verdict": "MERGE", "tip": "%s"}\n' "$tip" >"$NIGHT_SHIFT_VERDICTS/$tip.MERGE.json"
  git checkout -q main
  if [[ "${FAKE_CLAUDE_EVIL_MERGE:-0}" == 1 ]]; then
    git merge -q --no-ff --no-commit night-shift/gated && echo "EVIL = 1" >evil.py && git add evil.py \
      && git commit -q -m "merge(night-shift): gated"
  else
    git merge -q --ff-only night-shift/gated
  fi
  git push -q origin main
fi
# The never-list scan (closeout C7): a held branch pushed tonight that edits run_tests.sh.
if [[ "${FAKE_CLAUDE_SELF_EDIT:-0}" == 1 ]]; then
  git checkout -q -b night-shift/self-edit && echo "# skip" >run_tests.sh && git add run_tests.sh \
    && git commit -q -m "held: edit the gate" && git push -q origin night-shift/self-edit && git checkout -q main
fi
if [[ "${FAKE_CLAUDE_REPORT:-0}" == 1 && "$prompt" != *--dry-run* ]]; then
  date="$(date +%F)"   # a resumed leg's prompt carries no date
  [[ "$prompt" =~ ^/night-shift\ ([0-9-]+) ]] && date="${BASH_REMATCH[1]}"
  mkdir -p docs/night-shift
  summary=""; [[ "${FAKE_CLAUDE_SUMMARY:-0}" == 1 ]] && summary=$'**Summary.** merged 0\n\n'
  printf '# Night Shift %s\n\n%s## Nightly\nGREEN\n\n## Cost\nunits: 0\n' "$date" "$summary" >"docs/night-shift/$date.md"
  git add docs/night-shift && git commit -q -m "docs(night-shift): report $date" && git push -q origin main
fi
if [[ -n "${FAKE_CLAUDE_CLOSE_ITEM:-}" ]]; then
  sed -i 's/^status: inbox$/status: complete/' "map/$FAKE_CLAUDE_CLOSE_ITEM.md"
  git commit -q -am "close $FAKE_CLAUDE_CLOSE_ITEM" && git push -q origin main
fi
[[ "${FAKE_CLAUDE_LEAVE_BRANCH:-0}" == 1 ]] && git checkout -q -b night-shift/left-behind
[[ "${FAKE_CLAUDE_LEAVE_FILE:-0}" == 1 ]] && echo x >stray.txt
echo '{"result":"ok","session_id":"sess-456","total_cost_usd":0.42,"is_error":false,"num_turns":3,"permission_denials":[],"subagent_stats":{"spawned":1}}'
exit "${FAKE_CLAUDE_EXIT:-0}"
"""


def _git(*args, cwd=None, env=None):
    return subprocess.run(["git"] + list(args), cwd=cwd, env=env, check=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          universal_newlines=True).stdout.strip()


def _bug(root: Path, n: int, updated: str = "2026-10-20T00:00:00Z") -> None:
    write_item(str(root / "map" / ("bug-b%d.md" % n)),
               {"id": "BUG-%d" % n, "kind": "bug", "title": "Bug %d" % n,
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
               {"id": "PWAY-1", "kind": "pathway", "title": "P", "status": "active",
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
            "NIGHT_SHIFT_APP": "0",   # no test reaches the real app; its own tests turn it on
            "ANTHROPIC_API_KEY": "test",
            "LIVE_SESSION_RE": "nsguard_absent_" + uuid.uuid4().hex,
            "FAKE_CLAUDE_LOG": str(self.log),
            # A healthy night writes its report; a test of an early end says 0.
            "FAKE_CLAUDE_REPORT": "1",
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

    def auths(self):
        """The credentials each fake claude invocation saw, e.g. 'key=set token='."""
        if not self.log.exists():
            return []
        return [l[5:] for l in self.log.read_text().splitlines() if l.startswith("AUTH ")]

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
                 "--model opus", "--output-format json"):
        assert flag in joined
    assert "--dangerously-skip-permissions" not in joined
    # No budget fuse unless NIGHT_SHIFT_MAX_USD is set (owner, 2026-10-31).
    assert "--max-budget-usd" not in joined
    assert last["cost_recorded"] == "pushed"
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
    assert last["head_before"] != head1
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
    r = ns.run(FAKE_CLAUDE_COMMIT_MAIN=1, FAKE_CLAUDE_REPORT=0, NIGHT_SHIFT_TIMEOUT_S=120)
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


@pytest.mark.parametrize("knob", ["NIGHT_SHIFT_MODEL", "NIGHT_SHIFT_FALLBACK_MODEL"])
@pytest.mark.parametrize("value", ["fable", "claude-fable-5-1", "Fable"])
def test_a_fable_model_is_refused_before_anything_runs(ns, knob, value):
    """Owner, 2026-11-02: Fable is never invoked by any part of the Night Shift."""
    r = ns.run(**{knob: value})
    _check(r, "FORBIDDEN_MODEL", 1)
    assert not ns.clone.exists() and ns.calls() == []


def test_no_api_key_stops_before_clone(ns):
    r = ns.run(ANTHROPIC_API_KEY=None)
    _check(r, "NO_API_KEY", 1)
    assert not ns.clone.exists() and ns.calls() == []


def _argval(argv, flag):
    return argv[argv.index(flag) + 1] if flag in argv else None


def test_api_key_run_never_sees_the_token(ns):
    """Both credentials set: the API leg gets the key alone (the key silently
    wins over a token, so each leg must carry exactly one)."""
    r = ns.run(CLAUDE_CODE_OAUTH_TOKEN="tok", FAKE_CLAUDE_REPORT=1)
    _check(r, "OK", 0)
    assert ns.auths() == ["key=set token="]
    assert _argval(ns.calls()[0][0], "--model") == "opus"
    last = ns.last()
    assert last["auth"] == "api" and last["cost_api_usd"] is None
    report = ns.clone / "docs" / "night-shift" / ("%s.md" % last["date"])
    assert report.read_text().splitlines()[-1].startswith("total API cost: USD 0.42")


def test_credit_out_resumes_the_session_on_the_subscription(ns):
    """The 2026-11-02 failure: the API leg ends on 'Credit balance is too low';
    with a token set the runner resumes the SAME session on the subscription,
    on the fallback model, without the key, and the night ends OK."""
    r = ns.run(CLAUDE_CODE_OAUTH_TOKEN="tok", FAKE_CLAUDE_CREDIT_OUT=1, FAKE_CLAUDE_REPORT=1)
    _check(r, "OK", 0)
    assert ns.auths() == ["key=set token=", "key= token=set"]
    (first, _), (second, _) = ns.calls()
    assert _argval(first, "--model") == "opus"
    assert _argval(second, "--resume") == "sess-123"
    assert _argval(second, "--model") == "opus"
    assert "credit balance ran out" in _argval(second, "-p")
    last = ns.last()
    assert last["auth"] == "api-then-subscription"
    assert last["cost_api_usd"] == 1.5 and last["cost_usd"] == 0.42
    assert (ns.reports / ("%s.claude-api.json" % last["date"])).exists()
    report = ns.clone / "docs" / "night-shift" / ("%s.md" % last["date"])
    line = report.read_text().splitlines()[-1]
    assert "USD 1.50 until the credit balance ran out" in line and "USD 0.42 notional" in line


def test_fallback_model_is_a_knob(ns):
    r = ns.run(CLAUDE_CODE_OAUTH_TOKEN="tok", FAKE_CLAUDE_CREDIT_OUT=1, FAKE_CLAUDE_REPORT=1,
               NIGHT_SHIFT_FALLBACK_MODEL="sonnet")
    _check(r, "OK", 0)
    assert _argval(ns.calls()[1][0], "--model") == "sonnet"


def test_credit_out_without_a_token_names_the_real_cause(ns):
    r = ns.run(FAKE_CLAUDE_CREDIT_OUT=1)
    _check(r, "CLAUDE_FAILED", 1)
    assert len(ns.calls()) == 1
    detail = ns.last()["detail"]
    assert "Credit balance is too low" in detail and "connectors" not in detail


def test_other_failures_do_not_fall_back(ns):
    r = ns.run(CLAUDE_CODE_OAUTH_TOKEN="tok", FAKE_CLAUDE_EXIT=7)
    _check(r, "CLAUDE_FAILED", 1)
    assert len(ns.calls()) == 1 and ns.last()["auth"] == "api"


def test_token_only_runs_on_the_subscription(ns):
    r = ns.run(ANTHROPIC_API_KEY=None, CLAUDE_CODE_OAUTH_TOKEN="tok", FAKE_CLAUDE_REPORT=1)
    _check(r, "OK", 0)
    assert ns.auths() == ["key= token=set"]
    argv = ns.calls()[0][0]
    assert _argval(argv, "--model") == "opus" and "--resume" not in argv
    last = ns.last()
    assert last["auth"] == "subscription"
    report = ns.clone / "docs" / "night-shift" / ("%s.md" % last["date"])
    assert "notional, on the Claude subscription" in report.read_text().splitlines()[-1]


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


def _on_origin(ns, ref):
    return subprocess.run(["git", "-C", str(ns.origin), "rev-parse", "-q", "--verify", ref],
                          env=ns.env, stdout=subprocess.PIPE, universal_newlines=True).stdout.strip()


def _origin_file(ns, ref, path):
    return subprocess.run(["git", "-C", str(ns.origin), "show", "%s:%s" % (ref, path)],
                          env=ns.env, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                          universal_newlines=True)


def _clone_is_home(ns):
    assert _git("symbolic-ref", "--short", "HEAD", cwd=ns.clone, env=ns.env) == "main"
    assert _git("status", "--porcelain", cwd=ns.clone, env=ns.env) == ""


def test_every_leg_runs_with_no_background_task_limit(ns):
    """2026-11-09: `claude -p` killed a unit 10 minutes after the orchestrator's last
    turn and exited 0. Every leg must switch that limit off, the resumed one too."""
    r = ns.run(FAKE_CLAUDE_REPORT=0, FAKE_CLAUDE_FIX_ON_RESUME=1)
    _check(r, "OK", 0)
    waits = [l[7:] for l in ns.log.read_text().splitlines() if l.startswith("BGWAIT ")]
    assert waits == ["0", "0"]


def test_an_early_end_is_resumed_once_and_can_finish_ok(ns):
    r = ns.run(FAKE_CLAUDE_REPORT=0, FAKE_CLAUDE_LEAVE_BRANCH=1, FAKE_CLAUDE_FIX_ON_RESUME=1)
    _check(r, "OK", 0)
    (first, _), (second, _) = ns.calls()
    assert "--resume" not in first
    assert _argval(second, "--resume") == "sess-456"
    last = ns.last()
    assert "no report" in last["resumed"]
    assert last["salvaged"] is None
    assert (ns.reports / ("%s.claude-first.json" % last["date"])).exists()
    assert last["cost_usd"] == 0.42    # the resumed leg's total is cumulative
    _clone_is_home(ns)


def test_a_complete_night_is_not_resumed(ns):
    _check(ns.run(), "OK", 0)
    assert len(ns.calls()) == 1
    assert ns.last()["resumed"] is None


@pytest.mark.parametrize("knob, needle", [("FAKE_CLAUDE_LEAVE_BRANCH", "night-shift/left-behind"),
                                          ("FAKE_CLAUDE_LEAVE_FILE", "salvage")])
def test_work_left_behind_is_saved_and_the_next_night_can_run(ns, knob, needle):
    """The other half of 2026-11-09: a dirty clone is CLONE_DIRTY the next night."""
    r = ns.run(FAKE_CLAUDE_REPORT=0, **{knob: 1})
    _check(r, "RUN_SALVAGED", 1)
    last = ns.last()
    assert len(ns.calls()) == 2   # resumed once, and no more
    target, _sha, pushed = last["salvaged"].split()
    assert needle in target and pushed == "pushed"
    assert _on_origin(ns, target)
    if knob == "FAKE_CLAUDE_LEAVE_FILE":
        assert _origin_file(ns, target, "stray.txt").returncode == 0
    assert last["ticker_reraised"] is True
    assert not (ns.reports / "candidates-last.json").exists()
    _clone_is_home(ns)
    assert "stray.txt" not in _git("ls-tree", "-r", "--name-only", "origin/main",
                                   cwd=ns.clone, env=ns.env)
    (ns.nightly / "ticker.raised").write_text("%sT04:20:00+11:00 %s\n" % (ns.today, ns.green))
    r = ns.run(FAKE_CLAUDE_REPORT=0, FAKE_CLAUDE_FIX_ON_RESUME=1)
    assert "CLONE_DIRTY" not in r.stdout


def test_uncommitted_work_on_its_own_branch_stays_on_that_branch(ns):
    r = ns.run(FAKE_CLAUDE_REPORT=0, FAKE_CLAUDE_LEAVE_BRANCH=1, FAKE_CLAUDE_LEAVE_FILE=1)
    _check(r, "RUN_SALVAGED", 1)
    assert ns.last()["salvaged"].startswith("night-shift/left-behind ")
    assert _origin_file(ns, "night-shift/left-behind", "stray.txt").returncode == 0


def test_a_night_with_no_report_gets_a_valid_stub_and_is_not_ok(ns):
    from mapkit.night_shift_report import report_errors
    r = ns.run(FAKE_CLAUDE_REPORT=0)
    _check(r, "NO_REPORT", 1)
    last = ns.last()
    assert len(ns.calls()) == 2
    assert last["stub_report"] == "pushed" and last["ticker_reraised"] is True
    stub = _origin_file(ns, "main", "docs/night-shift/%s.md" % last["date"]).stdout
    assert report_errors(stub) == []
    assert "NO_REPORT" in stub and "none recorded" in stub
    _clone_is_home(ns)


def test_a_salvaged_night_names_the_branch_in_its_stub(ns):
    r = ns.run(FAKE_CLAUDE_REPORT=0, FAKE_CLAUDE_LEAVE_BRANCH=1)
    _check(r, "RUN_SALVAGED", 1)
    last = ns.last()
    stub = _origin_file(ns, "main", "docs/night-shift/%s.md" % last["date"]).stdout
    assert "night-shift/left-behind" in stub and "not gated" in stub


def test_a_failed_leg_is_saved_but_not_resumed(ns):
    r = ns.run(FAKE_CLAUDE_REPORT=0, FAKE_CLAUDE_EXIT=1, FAKE_CLAUDE_LEAVE_FILE=1)
    _check(r, "CLAUDE_FAILED", 1)
    assert len(ns.calls()) == 1
    assert ns.last()["salvaged"]
    _clone_is_home(ns)


def test_no_resume_without_enough_time_left(ns):
    r = ns.run(FAKE_CLAUDE_REPORT=0, NIGHT_SHIFT_TIMEOUT_S=120)
    _check(r, "NO_REPORT", 1)
    assert len(ns.calls()) == 1


def test_a_dry_run_is_neither_resumed_nor_saved(ns):
    r = ns.run(NIGHT_SHIFT_DRY_RUN=1, FAKE_CLAUDE_NO_REPORT=1, FAKE_CLAUDE_LEAVE_FILE=1)
    assert len(ns.calls()) == 1
    last = ns.last()
    assert last["salvaged"] is None and last["stub_report"] is None
    assert (ns.clone / "stray.txt").exists()


def test_a_multi_line_detail_cannot_break_the_stub_layout(tmp_path):
    """A CLAUDE_FAILED detail can carry a stderr tail, and a line of it starting
    ``## `` would otherwise become a section of its own."""
    from mapkit.night_shift_report import report_errors
    out = tmp_path / "docs" / "stub.md"
    subprocess.run([sys.executable, str(REPO / "tools" / "night_shift_stub_report.py"),
                    "--date", "2026-11-09", "--status", "CLAUDE_FAILED", "--clone", str(tmp_path),
                    "--out", str(out), "--started", "a", "--finished", "b",
                    "--detail", "foo\n## Cost\n## Held\n| BUG-1 | x"], check=True)
    assert report_errors(out.read_text()) == []


def test_stub_never_lands_over_a_report_that_exists(ns):
    _check(ns.run(FAKE_CLAUDE_EXIT=1), "CLAUDE_FAILED", 1)
    assert ns.last()["stub_report"] is None
    text = _origin_file(ns, "main", "docs/night-shift/%s.md" % ns.last()["date"]).stdout
    assert text.startswith("# Night Shift") and "units: 0" in text


def test_post_run_snapshot_is_tomorrows_baseline(ns):
    r = ns.run(FAKE_CLAUDE_CLOSE_ITEM="bug-b1")
    _check(r, "OK", 0)
    last = ns.last()
    assert sum(c.endswith(" close bug-b1") for c in last["main_commits"]) == 1
    snapshot = json.loads((ns.reports / "candidates-last.json").read_text())
    assert [i["id"] for i in snapshot["items"]] == ["BUG-2"]
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


# --- the map app: the clone, the prompt, the deploy step ---------------------
FAKE_DEPLOY = r"""#!/usr/bin/env python3
import json, os, sys
args = sys.argv[1:]
result = args[args.index("--result") + 1]
with open(os.environ["FAKE_DEPLOY_LOG"], "a") as fh:
    fh.write(" ".join(args) + "\n")
status = os.environ.get("FAKE_DEPLOY_STATUS", "DEPLOYED")
if status != "NO_RESULT":
    json.dump({"status": status, "line": "app deploy: %s; fake" % status}, open(result, "w"))
"""


def _app_on(ns, tmp, app_origin=True):
    env = {}
    fake = tmp / "fake_deploy.py"
    fake.write_text(FAKE_DEPLOY)
    env["NIGHT_SHIFT_DEPLOY_TOOL"] = str(fake)
    env["FAKE_DEPLOY_LOG"] = str(tmp / "deploy.log")
    env["NIGHT_SHIFT_APP"] = "1"
    env["NIGHT_SHIFT_APP_CLONE"] = str(tmp / "app-clone")
    env["NIGHT_SHIFT_APP_LIVE"] = str(tmp / "app-live")
    env["NIGHT_SHIFT_DATA_LIVE"] = str(tmp / "data-live")
    env["NIGHT_SHIFT_APP_URL"] = "http://127.0.0.1:1"   # nothing listens: unhealthy before the night
    bare = tmp / "app-origin.git"
    if app_origin:
        seed = tmp / "app-seed"
        seed.mkdir()
        _git("init", "-q", cwd=seed, env=ns.env)
        _git("checkout", "-q", "-b", "main", cwd=seed, env=ns.env)
        (seed / "a.txt").write_text("a\n")
        _git("add", "-A", cwd=seed, env=ns.env)
        _git("commit", "-q", "-m", "seed", cwd=seed, env=ns.env)
        _git("clone", "-q", "--bare", str(seed), str(bare), env=ns.env)
    env["NIGHT_SHIFT_APP_ORIGIN"] = str(bare)
    return env


def _prompt(ns):
    return next(l for l in ns.log.read_text().splitlines() if l.startswith("ARG /night-shift"))


def test_app_off_passes_none_and_never_deploys(ns):
    r = ns.run(FAKE_CLAUDE_REPORT=1)
    _check(r, "OK", 0)
    assert "--app-clone none" in _prompt(ns)
    doc = json.loads((ns.reports / "last_run.json").read_text())
    assert doc["app_clone"] == "off" and doc["deploy_status"] is None


def test_app_clone_prepared_and_named_in_the_prompt(ns, tmp_path):
    env = _app_on(ns, tmp_path)
    r = ns.run(FAKE_CLAUDE_REPORT=1, **env)
    _check(r, "OK", 0)
    assert "--app-clone %s" % (tmp_path / "app-clone") in _prompt(ns)
    doc = json.loads((ns.reports / "last_run.json").read_text())
    assert doc["app_clone"] == "ready" and len(doc["app_night_start"]) == 40


def test_app_clone_failure_is_not_fatal(ns, tmp_path):
    env = _app_on(ns, tmp_path, app_origin=False)
    r = ns.run(FAKE_CLAUDE_REPORT=1, **env)
    _check(r, "OK", 0)
    assert "--app-clone none" in _prompt(ns)
    assert json.loads((ns.reports / "last_run.json").read_text())["app_clone"] == "clone failed"


def test_deploy_line_lands_before_the_cost_line(ns, tmp_path):
    env = _app_on(ns, tmp_path)
    r = ns.run(FAKE_CLAUDE_REPORT=1, FAKE_DEPLOY_STATUS="DEPLOYED", **env)
    _check(r, "OK", 0)
    doc = json.loads((ns.reports / "last_run.json").read_text())
    assert doc["deploy_status"] == "DEPLOYED" and "deploy DEPLOYED" in doc["detail"]
    report = (ns.clone / "docs" / "night-shift" / ("%s.md" % date.today().isoformat())).read_text().splitlines()
    assert report[-2] == "app deploy: DEPLOYED; fake"
    assert report[-1].startswith("total API cost:")
    args = (tmp_path / "deploy.log").read_text()
    assert "--main-clone %s" % ns.clone in args and "--app-live %s" % (tmp_path / "app-live") in args
    assert "--pre-night-healthy 0" in args and doc["app_healthy_before"] == "0"


def test_a_dry_run_leaves_the_app_clone_alone(ns, tmp_path):
    env = _app_on(ns, tmp_path)
    r = ns.run(NIGHT_SHIFT_DRY_RUN=1, **env)
    _check(r, "DRY_OK", 0)
    assert not (tmp_path / "app-clone").exists()
    assert "--app-clone none" in _prompt(ns)
    assert json.loads((ns.reports / "last_run.json").read_text())["app_clone"] == "dry"


def test_failed_deploy_is_flagged_in_the_summary(ns, tmp_path):
    env = _app_on(ns, tmp_path)
    r = ns.run(FAKE_CLAUDE_REPORT=1, FAKE_CLAUDE_SUMMARY=1, FAKE_DEPLOY_STATUS="ROLLED_BACK_APP", **env)
    _check(r, "OK", 0)
    text = (ns.clone / "docs" / "night-shift" / ("%s.md" % date.today().isoformat())).read_text()
    assert "**Summary.** merged 0 **App deploy: ROLLED_BACK_APP, see the last lines of Cost.**" in text


def test_deploy_runs_even_when_claude_fails(ns, tmp_path):
    env = _app_on(ns, tmp_path)
    r = ns.run(FAKE_CLAUDE_EXIT=3, **env)
    assert "CLAUDE_FAILED" in r.stdout
    assert (tmp_path / "deploy.log").exists()
    assert json.loads((ns.reports / "last_run.json").read_text())["deploy_status"] == "DEPLOYED"


def test_deploy_without_a_result_is_named(ns, tmp_path):
    env = _app_on(ns, tmp_path)
    r = ns.run(FAKE_CLAUDE_REPORT=1, FAKE_DEPLOY_STATUS="NO_RESULT", **env)
    _check(r, "OK", 0)
    assert json.loads((ns.reports / "last_run.json").read_text())["deploy_status"] == "FAILED_NEEDS_SESSION"


def test_deploy_skipped_during_a_sitting_and_on_a_dry_run(ns, tmp_path):
    env = _app_on(ns, tmp_path)
    r = ns.run(NIGHT_SHIFT_DRY_RUN=1, **env)
    _check(r, "DRY_OK", 0)
    assert not (tmp_path / "deploy.log").exists()
    # A Sitting that starts while claude runs: the guard matches only after claude exits.
    sentinel = "nsguard_deploy_" + uuid.uuid4().hex
    fake_guard = tmp_path / "sitting.sh"
    fake_guard.write_text("#!/usr/bin/env bash\nexec -a %s sleep 120\n" % sentinel)
    fake_guard.chmod(0o755)
    r = ns.run(FAKE_CLAUDE_REPORT=1, FAKE_CLAUDE_START=str(fake_guard), LIVE_SESSION_RE=sentinel, **env)
    subprocess.run(["pkill", "-f", sentinel])
    _check(r, "OK", 0)
    assert not (tmp_path / "deploy.log").exists()
    assert json.loads((ns.reports / "last_run.json").read_text())["deploy_status"] == "SKIPPED_LIVE_SESSION"


# --- the landed check (closeout C1) -------------------------------------------
def test_an_ungated_code_push_ends_ungated_merge_and_holds_the_next_night(ns):
    r = ns.run(FAKE_CLAUDE_PUSH_CODE=1, FAKE_CLAUDE_SUMMARY=1)
    _check(r, "UNGATED_MERGE", 1)
    last = ns.last()
    code = _git("log", "-1", "--format=%H", "--", "code.py", cwd=ns.clone, env=ns.env)
    assert last["landed"] == "landed: gated=0 exempt=1 ungated=1"
    assert "UNGATED %s t <t@t> no MERGE record covers it: code.py" % code[:12] in last["detail"]
    assert last["ticker_reraised"] is True and (ns.nightly / "ticker.raised").exists()
    hold = (ns.reports / "ungated.hold").read_text()
    assert hold.startswith("UNGATED_MERGE %s: landed: gated=0" % last["date"]) and code[:12] in hold
    # The report and its cost commit still land, flagged; nothing is reverted.
    assert last["cost_recorded"] == "pushed"
    report = _git("show", "main:docs/night-shift/%s.md" % last["date"], cwd=ns.origin, env=ns.env)
    assert "**UNGATED_MERGE: a commit reached main with no MERGE record" in report
    flag = report.splitlines()[-2]
    assert flag.startswith("landed check: UNGATED_MERGE; landed: gated=0")
    assert "then delete %s" % (ns.reports / "ungated.hold") in flag and "flagged by design" in flag
    assert report.splitlines()[-1].startswith("total API cost:")
    assert _git("cat-file", "-e", "main:code.py", cwd=ns.origin, env=ns.env) == ""
    assert _git("status", "--porcelain", cwd=ns.clone, env=ns.env) == ""
    # The next start ends UNGATED_HOLD, without claude, until the owner deletes the file.
    ns.push_change()
    r = ns.run()
    _check(r, "UNGATED_HOLD", 1)
    detail = ns.last()["detail"]
    assert "UNGATED_MERGE %s" % last["date"] in detail and code[:12] in detail
    assert "then delete %s" % (ns.reports / "ungated.hold") in detail
    assert len(ns.calls()) == 1
    (ns.reports / "ungated.hold").unlink()
    _check(ns.run(), "OK", 0)


def test_a_recorded_merge_passes_the_landed_check(ns):
    r = ns.run(FAKE_CLAUDE_GATED_MERGE=1)
    _check(r, "OK", 0)
    last = ns.last()
    assert last["landed"] == "landed: gated=1 exempt=1 ungated=0"
    assert last["origin_after_claude"] == _git("rev-parse", "main~1", cwd=ns.origin, env=ns.env)
    assert not (ns.reports / "ungated.hold").exists()
    records = list((ns.tmp / ".local" / "state" / "jugglebot" / "verdicts").iterdir())
    assert len(records) == 1   # the runner's default directory, exported to claude


def test_exempt_only_commits_pass_the_landed_check(ns):
    r = ns.run(FAKE_CLAUDE_CLOSE_ITEM="bug-b1")
    _check(r, "OK", 0)
    assert ns.last()["landed"] == "landed: gated=0 exempt=2 ungated=0"


def test_an_evil_merge_is_flagged_by_the_runner(ns):
    r = ns.run(FAKE_CLAUDE_EVIL_MERGE=1)
    _check(r, "UNGATED_MERGE", 1)
    last = ns.last()
    assert "merge carries content of its own (evil merge): evil.py" in last["detail"]
    assert last["landed"] == "landed: gated=1 exempt=1 ungated=1"


def test_an_ungated_push_on_a_failed_night_still_holds(ns):
    r = ns.run(FAKE_CLAUDE_PUSH_CODE=1, FAKE_CLAUDE_EXIT=3)
    _check(r, "CLAUDE_FAILED", 1)
    last = ns.last()
    assert "; UNGATED_MERGE landed: gated=0" in last["detail"] and "code.py" in last["detail"]
    assert (ns.reports / "ungated.hold").exists()


def test_every_night_held_by_ungated_hold_pushes_a_stub_report(ns):
    """Not silent: the stub names the file, the flagged commits and how to clear it."""
    ns.reports.mkdir(parents=True)
    (ns.reports / "ungated.hold").write_text(
        "UNGATED_MERGE 2026-11-12: landed: gated=0 exempt=1 ungated=1\n"
        "UNGATED abc123def456 t <t@t> no MERGE record covers it: code.py\n")
    r = ns.run()
    _check(r, "UNGATED_HOLD", 1)
    last = ns.last()
    assert last["stub_report"] == "pushed" and ns.calls() == []
    report = _git("show", "main:docs/night-shift/%s.md" % last["date"], cwd=ns.origin, env=ns.env)
    assert "UNGATED_HOLD" in report and "abc123def456" in report
    assert "then delete %s" % (ns.reports / "ungated.hold") in report
    # The owner's own off switch stays quiet, as before.
    (ns.reports / "ungated.hold").unlink()
    (ns.live / "temp" / "night-shift.hold").write_text("owner: away\n")
    _check(ns.run(), "HELD", 0)


def test_no_checker_at_head_before_skips_the_landed_check(ns):
    """Bootstrap or rollback: the night's start has no checker; recorded, no hold, no copy."""
    work = ns.tmp / "rm_checker"
    _git("clone", "-q", str(ns.origin), str(work), env=ns.env)
    _git("rm", "-q", "tools/night_shift_landed.py", cwd=work, env=ns.env)
    _git("commit", "-q", "-m", "rollback", cwd=work, env=ns.env)
    _git("push", "-q", "origin", "main", cwd=work, env=ns.env)
    r = ns.run(FAKE_CLAUDE_PUSH_CODE=1)
    _check(r, "OK", 0)
    assert ns.last()["landed"] == "landed: skipped (no checker at head_before)"
    assert not (ns.reports / "ungated.hold").exists()


# --- the never-list scan (closeout C7) -----------------------------------------
def test_a_branch_pushed_tonight_touching_the_never_list_is_self_edit_branch(ns):
    """An older branch that touches the never-list is ignored (it is in the pre-claude
    snapshot); the one pushed tonight is named with its path, the report is flagged,
    the ticker re-raised, and no hold file stops the next night."""
    work = ns.tmp / "old_branch"
    _git("clone", "-q", str(ns.origin), str(work), env=ns.env)
    _git("checkout", "-q", "-b", "night-shift/2026-11-01-old", cwd=work, env=ns.env)
    (work / "tools" / "night_shift_old.py").write_text("x = 1\n")
    _git("add", "-A", cwd=work, env=ns.env)
    _git("commit", "-q", "-m", "old", cwd=work, env=ns.env)
    _git("push", "-q", "origin", "night-shift/2026-11-01-old", cwd=work, env=ns.env)
    r = ns.run(FAKE_CLAUDE_SELF_EDIT=1, FAKE_CLAUDE_REPORT=1, FAKE_CLAUDE_SUMMARY=1)
    _check(r, "SELF_EDIT_BRANCH", 1)
    last = ns.last()
    assert last["branches"] == "branches: pushed=1 self_edit=1"
    assert "SELF_EDIT night-shift/self-edit" in last["detail"] and "run_tests.sh" in last["detail"]
    assert "2026-11-01-old" not in last["detail"]
    assert last["ticker_reraised"] is True
    report = _git("show", "main:docs/night-shift/%s.md" % last["date"], cwd=ns.origin, env=ns.env)
    assert "**SELF_EDIT_BRANCH: a branch pushed tonight touches the Night Shift never-list" in report
    assert report.splitlines()[-2].startswith("never-list scan: SELF_EDIT_BRANCH; branches: pushed=1")
    assert not (ns.reports / "ungated.hold").exists()
    # Nothing moved since: the next night is not flagged again.
    ns.push_change()
    _check(ns.run(), "OK", 0)
    assert ns.last()["branches"] == "branches: pushed=0 self_edit=0"
