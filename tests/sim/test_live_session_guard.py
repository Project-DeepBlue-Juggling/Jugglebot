"""Live-session guard library (tools/lib/live_session.sh) and the nightly's use of it.

The guard was factored out of tools/nightly_suite.sh so tools/night_shift.sh shares
the identical regex and wait loop. Pins: the nightly no longer carries its own copy
(T-U3c), the library's default regex is byte-identical to the original, the guard
and wait loop behave (sentinel process stands in for a live Sitting), and the
nightly's status/report/ticker output is unchanged in form (T-R1).
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
import time
import uuid
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
NIGHTLY = REPO / "tools" / "nightly_suite.sh"
LIB = REPO / "tools" / "lib" / "live_session.sh"
TICKER = REPO / "tools" / "nightly_ticker.sh"

DEFAULT_RE = (r"(ros2 launch jugglebot|jugglebot_launch\.py|teensy_bridge_node"
              r"|trajectory_node|orchestrator_node)")


def check_nightly_text(text: str) -> None:
    assert "LIVE_SESSION_RE=" not in text
    assert "live_session()" not in text
    assert re.search(r'^source "\$REPO_ROOT/tools/lib/live_session\.sh"', text, re.M)


@pytest.fixture
def sentinel_name():
    return "jb_sentinel_" + uuid.uuid4().hex


class Sentinel:
    def __init__(self, name):
        self.proc = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(20)", name],
            start_new_session=True)  # own pgid: the guard skips its own group
        self.name = name
        time.sleep(0.3)

    def stop(self):
        self.proc.kill()
        self.proc.wait()


@pytest.fixture
def live(sentinel_name):
    s = Sentinel(sentinel_name)
    try:
        yield s
    finally:
        s.stop()


def run_lib(snippet, sentinel_name):
    env = dict(os.environ, LIVE_SESSION_RE=sentinel_name)
    # Sentinel regex is in this bash's argv too; the guard excludes own pgid/PID/PPID.
    return subprocess.run(["bash", "-c", "source tools/lib/live_session.sh; " + snippet],
                          cwd=str(REPO), env=env, capture_output=True, text=True,
                          timeout=30)


def test_nightly_uses_library_not_inline_copy():
    check_nightly_text(NIGHTLY.read_text())


def test_grep_check_fails_on_pre_change_script():
    old = subprocess.run(["git", "show", "e640a863:tools/nightly_suite.sh"], cwd=str(REPO),
                         capture_output=True, text=True)
    if old.returncode != 0:
        pytest.skip("pre-change revision not available")
    with pytest.raises(AssertionError):
        check_nightly_text(old.stdout)


def test_default_regex_is_unchanged():
    out = subprocess.run(
        ["bash", "-c", "unset LIVE_SESSION_RE; source tools/lib/live_session.sh; "
                       'printf %s "$LIVE_SESSION_RE"'],
        cwd=str(REPO), capture_output=True, text=True, timeout=30)
    assert out.stdout == DEFAULT_RE


def test_live_session_clear_returns_1(sentinel_name):
    assert run_lib("live_session", sentinel_name).returncode == 1


def test_live_session_detects_sentinel(live):
    assert run_lib("live_session", live.name).returncode == 0


def test_wait_returns_1_while_live(live):
    r = run_lib('wait_for_live_session_to_clear 2 1 t; rc=$?; '
                'echo "WAITED=$LIVE_SESSION_WAITED_S"; exit $rc', live.name)
    assert r.returncode == 1
    m = re.search(r"WAITED=(\d+)", r.stdout)
    assert m and int(m.group(1)) >= 2
    assert "waiting (budget 2s)" in r.stderr


def test_wait_returns_0_at_once_when_clear(sentinel_name):
    t0 = time.monotonic()
    r = run_lib('wait_for_live_session_to_clear 5 3; rc=$?; '
                'echo "WAITED=$LIVE_SESSION_WAITED_S"; exit $rc', sentinel_name)
    assert r.returncode == 0
    assert "WAITED=0" in r.stdout
    assert time.monotonic() - t0 < 3


def build_tree(tmp_path):
    root = tmp_path / "repo"
    (root / "tools" / "lib").mkdir(parents=True)
    shutil.copy(NIGHTLY, root / "tools" / "nightly_suite.sh")
    shutil.copy(LIB, root / "tools" / "lib" / "live_session.sh")
    shutil.copy(TICKER, root / "tools" / "nightly_ticker.sh")
    marker = tmp_path / "run_tests_invoked"
    xml = ('<testsuite tests="{n}" failures="0" errors="0" skipped="0">{cases}'
           '</testsuite>')
    p1 = xml.format(n=3, cases="".join('<testcase classname="a" name="t%d"/>' % i
                                       for i in range(3)))
    p2 = xml.format(n=1, cases='<testcase classname="a" name="s"/>')
    rt = root / "run_tests.sh"
    rt.write_text("#!/usr/bin/env bash\ntouch '%s'\n"
                  "cat >\"$JB_JUNIT_DIR/phase1.xml\" <<'X'\n%s\nX\n"
                  "cat >\"$JB_JUNIT_DIR/phase2.xml\" <<'X'\n%s\nX\nexit 0\n"
                  % (marker, p1, p2))
    rt.chmod(0o755)
    for s in ("tools/nightly_suite.sh", "tools/nightly_ticker.sh"):
        (root / s).chmod(0o755)
    bindir = tmp_path / "bin"
    bindir.mkdir()
    (bindir / "python").symlink_to(sys.executable)
    return root, marker, bindir


def run_nightly(root, bindir, tmp_path, extra):
    env = dict(os.environ, HOME=str(tmp_path), PDJ_VENV=str(tmp_path / "nonexistent"),
               PATH=str(bindir) + os.pathsep + os.environ["PATH"])
    env.pop("JB_NIGHTLY_IGNORE_SESSION", None)
    env.pop("LIVE_SESSION_RE", None)
    env.update(extra)
    return subprocess.run(["bash", str(root / "tools" / "nightly_suite.sh")], env=env,
                          capture_output=True, text=True, timeout=120)


def test_nightly_green_form_regression(tmp_path):
    root, marker, bindir = build_tree(tmp_path)
    r = run_nightly(root, bindir, tmp_path, {"JB_NIGHTLY_IGNORE_SESSION": "1"})
    assert r.returncode == 0, r.stderr
    rep = root / "temp" / "reports" / "nightly"
    status = (rep / "status").read_text()
    assert re.match(r"^GREEN 4/4 passed, 0 failed, 0 errored, 0 xfailed, 0 skipped "
                    r"\d{4}-\d{2}-\d{2}T", status), status
    latest = rep / "latest.md"
    assert latest.is_symlink()
    assert re.fullmatch(r"\d{4}-\d{2}-\d{2}\.md", os.readlink(str(latest)))
    assert (rep / "ticker.raised").exists()
    assert "./run_tests.sh --full --hypothesis-profile=ci-deep" in latest.read_text()
    assert marker.exists()


def test_nightly_deferred_when_sentinel_live(tmp_path, live):
    root, marker, bindir = build_tree(tmp_path)
    r = run_nightly(root, bindir, tmp_path, {
        "LIVE_SESSION_RE": live.name,
        "JB_NIGHTLY_DEFER_BUDGET_S": "2", "JB_NIGHTLY_DEFER_POLL_S": "1"})
    assert r.returncode == 0, r.stderr
    status = (root / "temp" / "reports" / "nightly" / "status").read_text().strip()
    assert status.startswith("DEFERRED live robot session held the box for "), status
    assert re.search(r"suite NOT run \d{4}-\d{2}-\d{2}T\S+$", status), status
    assert not marker.exists()
