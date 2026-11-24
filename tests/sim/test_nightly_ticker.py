"""tools/nightly_ticker.sh: the missed-slot STALE verdict (BUG-100) and the old verdicts.

Drives the real script with NIGHTLY_REPORTS_DIR in tmp_path, a fake clock
(NIGHTLY_TICKER_NOW) and TZ=UTC, so the 02:00 slot arithmetic is deterministic.
"""
from __future__ import annotations

import datetime as dt
import os
import subprocess
from pathlib import Path

import pytest

TICKER = Path(__file__).resolve().parents[2] / "tools" / "nightly_ticker.sh"
UTC = dt.timezone.utc


def at(day, hhmm):
    d = dt.datetime.strptime("2026-11-%02d %s" % (day, hhmm), "%Y-%m-%d %H:%M")
    return d.replace(tzinfo=UTC)


def check(rep, now, who="t"):
    env = {**os.environ, "NIGHTLY_REPORTS_DIR": str(rep), "TZ": "UTC",
           "PULL_STATUS_FILE": str(rep / "nopull"),
           "NIGHTLY_TICKER_NOW": str(int(now.timestamp()))}
    r = subprocess.run(["bash", str(TICKER), "check", "--who", who], env=env,
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return r.stdout.strip().splitlines()


def lowered(rep, run):
    (rep / "ticker.lowered").write_text(
        "%s handled %s by x\n" % (run.isoformat(), run.isoformat()))


def raised(rep, run, word="GREEN"):
    line = "%s 4/4 passed %s" % (word, run.isoformat())
    (rep / "status").write_text(line + "\n")
    (rep / "ticker.raised").write_text("%s %s\n" % (run.isoformat(), line))


@pytest.fixture
def rep(tmp_path):
    d = tmp_path / "nightly"
    d.mkdir()
    return d


def test_todays_run_is_lowered(rep):
    lowered(rep, at(24, "02:11"))
    assert check(rep, at(24, "09:00"))[0].startswith("LOWERED ")


def test_run_in_progress_at_0210_is_not_stale(rep):
    lowered(rep, at(23, "02:11"))                 # yesterday's run; today's still running
    assert check(rep, at(24, "02:10"))[0].startswith("LOWERED ")


def test_missed_slot_is_stale_after_grace(rep):
    lowered(rep, at(23, "02:11"))                 # no run on the 24th
    out = check(rep, at(24, "02:45"))
    assert len(out) == 1 and out[0].startswith("STALE ")


def test_missed_slot_is_stale_next_morning_not_after_two_days(rep):
    lowered(rep, at(22, "02:11"))
    assert check(rep, at(24, "07:00"))[0].startswith("STALE ")


def test_late_catchup_run_is_not_stale(rep):
    lowered(rep, at(24, "06:30"))                 # Persistent=true catch-up after boot
    assert check(rep, at(24, "09:00"))[0].startswith("LOWERED ")


def test_deferred_status_counts_as_a_run_of_the_slot(rep):
    raised(rep, at(24, "02:40"), word="DEFERRED")
    out = check(rep, at(24, "09:00"))
    assert out == ["CLAIMED DEFERRED 4/4 passed %s" % at(24, "02:40").isoformat()]


def test_claimed_old_result_then_next_check_is_stale(rep):
    raised(rep, at(22, "02:11"))                  # never claimed, and the 23rd/24th missed
    out = check(rep, at(24, "09:00"))
    assert len(out) == 1 and out[0].startswith("CLAIMED GREEN")
    assert check(rep, at(24, "09:01"))[0].startswith("STALE ")


def test_claimed_fresh_result_is_one_line(rep):
    raised(rep, at(24, "02:11"))
    assert len(check(rep, at(24, "09:00"))) == 1


def test_unraised_and_never_unchanged(rep):
    assert check(rep, at(24, "09:00"))[0].startswith("NEVER ")
    (rep / "status").write_text("GREEN 4/4 passed %s\n" % at(24, "02:11").isoformat())
    out = check(rep, at(24, "09:00"))
    assert len(out) == 1 and out[0].startswith("UNRAISED GREEN")


def test_dead_claim_takeover_still_works(rep):
    claim = rep / "ticker.claim.1"
    claim.write_text("%s GREEN x %s\n" % (at(24, "02:11").isoformat(), at(24, "02:11").isoformat()))
    os.utime(claim, (1, 1))                       # ancient mtime: dead
    out = check(rep, at(24, "09:00"))
    assert out[0].startswith("CLAIMED GREEN")
