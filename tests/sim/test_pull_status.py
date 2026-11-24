"""tools/systemd/jugglebot-pull.sh writes a status line every run, and
tools/nightly_ticker.sh check prints a PULL line when a stall is older than 10 min.

Failure prevented: the live checkout silently sitting dozens of commits behind main
because a tracked modification made the auto-pull exit without a word (2026-11-13).
All repos are tmp_path clones; locks and status paths are env-overridden (parallel-safe).
"""
from __future__ import annotations

import datetime as dt
import os
import re
import shutil
import subprocess
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
PULL = REPO / "tools" / "systemd" / "jugglebot-pull.sh"
TICKER = REPO / "tools" / "nightly_ticker.sh"

GIT_ENV = {"GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t", "GIT_COMMITTER_NAME": "t",
           "GIT_COMMITTER_EMAIL": "t@t"}


def git(cwd, *args):
    env = {**os.environ, **GIT_ENV}
    return subprocess.run(["git", *args], cwd=cwd, env=env, check=True,
                          capture_output=True, text=True).stdout.strip()


@pytest.fixture
def world(tmp_path):
    origin = tmp_path / "origin.git"
    git(tmp_path, "init", "-q", "--bare", str(origin))
    git(origin, "symbolic-ref", "HEAD", "refs/heads/main")
    seed = tmp_path / "seed"
    git(tmp_path, "clone", "-q", str(origin), str(seed))
    for n in "abcd":
        (seed / f"{n}.txt").write_text("1\n")
    git(seed, "add", "."); git(seed, "commit", "-qm", "c0"); git(seed, "branch", "-M", "main"); git(seed, "push", "-q", "origin", "HEAD:main")
    live = tmp_path / "live"
    git(tmp_path, "clone", "-q", str(origin), str(live))
    return tmp_path, seed, live


def advance(seed, n=2):
    for i in range(n):
        (seed / "new.txt").write_text(f"{i}{os.urandom(2).hex()}\n")
        git(seed, "add", "."); git(seed, "commit", "-qm", f"n{i}")
    git(seed, "push", "-q", "origin", "HEAD:main")


def run_pull(tmp, live):
    status = live / "temp" / "reports" / "pull" / "status"
    env = {**os.environ, "JUGGLEBOT_PULL_REPO": str(live),
           "JUGGLEBOT_PULL_STATUS": str(status),
           "JUGGLEBOT_SUITE_LOCK": str(tmp / "suite.lock"),
           "JUGGLEBOT_PULL_LOCK": str(tmp / "pull.lock")}
    subprocess.run(["bash", str(PULL)], env=env, capture_output=True, text=True)
    return status.read_text().strip().split(" ")


def kv(fields, key):
    return next(f.split("=", 1)[1] for f in fields if f.startswith(key + "="))


def test_dirty_tracked_with_origin_ahead_is_skipped_dirty(world):
    tmp, seed, live = world
    advance(seed, 3)
    for n in "abcd":
        (live / f"{n}.txt").write_text("dirty\n")
    f = run_pull(tmp, live)
    assert f[1] == "SKIPPED_DIRTY"
    assert kv(f, "behind") == "3"
    assert kv(f, "files") == "a.txt,b.txt,c.txt"      # capped at 3
    assert git(live, "rev-list", "--count", "HEAD..origin/main") == "3"   # not merged


def test_clean_fast_forwards_and_writes_ok(world):
    tmp, seed, live = world
    advance(seed, 2)
    f = run_pull(tmp, live)
    assert f[1] == "OK" and kv(f, "behind") == "0" and kv(f, "since") == "-"
    assert git(live, "rev-parse", "HEAD") == git(seed, "rev-parse", "HEAD")


def test_since_persists_across_non_ok_and_resets_on_ok(world):
    tmp, seed, live = world
    advance(seed, 1)
    (live / "a.txt").write_text("dirty\n")
    first = run_pull(tmp, live)
    second = run_pull(tmp, live)
    assert first[1] == second[1] == "SKIPPED_DIRTY"
    assert kv(first, "since") == kv(second, "since") == first[0]
    git(live, "checkout", "--", "a.txt")
    ok = run_pull(tmp, live)
    assert ok[1] == "OK" and kv(ok, "since") == "-"
    advance(seed, 1)
    (live / "a.txt").write_text("dirty\n")
    again = run_pull(tmp, live)
    assert again[1] == "SKIPPED_DIRTY" and kv(again, "since") == again[0]


def test_detached_and_diverged_words(world):
    tmp, seed, live = world
    (live / "x.txt").write_text("x\n"); git(live, "add", "."); git(live, "commit", "-qm", "local")
    advance(seed, 1)
    assert run_pull(tmp, live)[1] == "DIVERGED"
    git(live, "checkout", "-q", "--detach")
    assert run_pull(tmp, live)[1] == "DETACHED"


def test_stalled_fetch_times_out_as_fetch_failed(world):
    """A fetch that never returns (stalled TCP) must not hang the oneshot: an active oneshot
    cannot be re-triggered by its timer, so the status file would never be rewritten and the
    checkout would go stale in silence. The stub git sleeps on fetch; the bound is 1 s here."""
    tmp, seed, live = world
    real_git = shutil.which("git")
    stub_dir = tmp / "stub"
    stub_dir.mkdir()
    stub = stub_dir / "git"
    stub.write_text('#!/usr/bin/env bash\n'
                    'if [ "$1" = fetch ]; then exec sleep 60; fi\n'
                    'exec %s "$@"\n' % real_git)
    stub.chmod(0o755)
    status = live / "temp" / "reports" / "pull" / "status"
    env = {**os.environ, "PATH": "%s:%s" % (stub_dir, os.environ["PATH"]),
           "JUGGLEBOT_PULL_REPO": str(live), "JUGGLEBOT_PULL_STATUS": str(status),
           "JUGGLEBOT_SUITE_LOCK": str(tmp / "suite.lock"),
           "JUGGLEBOT_PULL_LOCK": str(tmp / "pull.lock"),
           "JUGGLEBOT_PULL_FETCH_TIMEOUT_S": "1"}
    t0 = time.monotonic()
    r = subprocess.run(["bash", str(PULL)], env=env, capture_output=True, text=True, timeout=30)
    elapsed = time.monotonic() - t0
    assert elapsed < 15, elapsed
    assert r.returncode == 1
    assert "timed out" in r.stderr
    assert status.read_text().split(" ")[1] == "FETCH_FAILED"


def ticker(tmp_path, status_line):
    pull = tmp_path / "pullstatus"
    if status_line is not None:
        pull.write_text(status_line + "\n")
    rep = tmp_path / "nightly"
    rep.mkdir(exist_ok=True)
    env = {**os.environ, "PULL_STATUS_FILE": str(pull), "NIGHTLY_REPORTS_DIR": str(rep)}
    r = subprocess.run(["bash", str(TICKER), "check", "--who", "t"], env=env,
                       capture_output=True, text=True)
    assert r.returncode == 0
    return r.stdout.strip().splitlines()


def iso(minutes_ago):
    return (dt.datetime.now().astimezone() - dt.timedelta(minutes=minutes_ago)).isoformat(timespec="seconds")


def test_ticker_prints_pull_only_past_ten_minutes(tmp_path):
    old = f"{iso(0)} SKIPPED_DIRTY behind=43 since={iso(30)} files=a.txt"
    lines = ticker(tmp_path, old)
    assert lines[0].startswith("NEVER")                       # first line unchanged
    assert re.fullmatch(r"PULL SKIPPED_DIRTY behind=43 since=\S+", lines[1])
    fresh = f"{iso(0)} SKIPPED_DIRTY behind=43 since={iso(3)} files=a.txt"
    assert len(ticker(tmp_path, fresh)) == 1
    assert len(ticker(tmp_path, f"{iso(0)} OK behind=0 since=- files=-")) == 1
    assert len(ticker(tmp_path, None)) == 1


# --- dead ticker claims expire by age (BUG-100) ---------------------------------------

def _ticker_dir(tmp_path):
    rep = tmp_path / "nightly"
    rep.mkdir()
    return rep


def _check(rep, tmp_path):
    env = {**os.environ, "PULL_STATUS_FILE": str(tmp_path / "nopull"),
           "NIGHTLY_REPORTS_DIR": str(rep), "NIGHTLY_TICKER_CLAIM_MAX_AGE_S": "300"}
    r = subprocess.run(["bash", str(TICKER), "check", "--who", "t"], env=env,
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return r.stdout.strip()


#: The nightly stamp the claim tests use. It must be RECENT: the ticker calls a status older than
#: two days STALE, so the fixed 2026-11-17 literal these tests first carried made
#: test_dead_claim_is_taken_over_and_read_once fail from 2026-11-19 15:00 AEDT on (a date bomb).
def _claim_ts():
    """A status stamp 6 h old, but never older than the latest 02:00 slot.

    BUG-100 (2026-11-24): the ticker reports STALE for a status from before the latest
    scheduled 02:00 slot (30 min grace), so a plain "now - 6 h" made these tests fail
    between 02:30 and 08:00 local. The claim-expiry behaviour under test does not depend
    on the stamp's age, only on it being current.
    """
    now = dt.datetime.now().astimezone()
    slot = (now - dt.timedelta(minutes=30)).replace(hour=2, minute=0, second=0, microsecond=0)
    if slot > now - dt.timedelta(minutes=30):
        slot -= dt.timedelta(days=1)
    ts = max(now - dt.timedelta(hours=6), slot + dt.timedelta(minutes=1))
    return ts.astimezone(dt.timezone.utc).isoformat(timespec="seconds")


_CLAIM_TS = _claim_ts()


def _claim(rep, age_s, status=None, pid=99999):
    status = status if status is not None else "RED 3/9 %s" % _CLAIM_TS
    f = rep / f"ticker.claim.{pid}"
    f.write_text(f"{_CLAIM_TS} {status}\n")
    t = time.time() - age_s
    os.utime(f, (t, t))
    return f


def test_dead_claim_is_taken_over_and_read_once(tmp_path):
    rep = _ticker_dir(tmp_path)
    _claim(rep, 3600)
    out = _check(rep, tmp_path)
    assert out == "CLAIMED RED 3/9 %s" % _CLAIM_TS
    assert (rep / "ticker.lowered").read_text().startswith("%s handled " % _CLAIM_TS)
    assert not list(rep.glob("ticker.claim.*"))
    assert _check(rep, tmp_path).startswith("LOWERED ")          # read exactly once


def test_young_claim_still_reports_in_progress(tmp_path):
    rep = _ticker_dir(tmp_path)
    f = _claim(rep, 10)
    assert _check(rep, tmp_path) == "LOWERED claim in progress by another session"
    assert f.exists() and not (rep / "ticker.lowered").exists()


def test_dead_claim_plus_new_raise_claims_the_raised_status(tmp_path):
    rep = _ticker_dir(tmp_path)
    _claim(rep, 3600, status="RED 1/9 old")
    (rep / "ticker.raised").write_text("2026-11-18T04:00:00+00:00 GREEN 9/9 2026-11-18T04:00:00+00:00\n")
    assert _check(rep, tmp_path) == "CLAIMED GREEN 9/9 2026-11-18T04:00:00+00:00"
    assert not list(rep.glob("ticker.claim.*"))
