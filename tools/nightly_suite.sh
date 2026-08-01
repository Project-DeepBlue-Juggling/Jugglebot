#!/usr/bin/env bash
# Nightly full-tier test run — the thing that makes `-m nightly` a real tier
# rather than a delete button.
#
#   ./run_tests.sh          per-commit gate, deselects `nightly`  (200 s measured)
#   this script (04:00)     ./run_tests.sh --full at ci-deep depth (1259 s measured)
#
# Demoting a test is only honest if something still runs it. Land any demotion
# and this runner in the SAME commit (plans/active/refactor-2026-07.md Phase 2).
#
# Armed by systemd --user (jugglebot-nightly.timer, OnCalendar=04:00,
# Persistent=true so a night the Jetson is off fires on the next boot). Linger is
# enabled for jetson, so the user manager runs without an active login session.
# The units are checked in at tools/systemd/ with install + verify commands —
# re-arming is part of any Jetson rebuild.
#
# Outputs, all under temp/reports/nightly/ (gitignored):
#   YYYY-MM-DD.md         per-phase counts, duration, trimmed failure tracebacks
#   YYYY-MM-DD-junit/     phase1.xml + phase2.xml
#   YYYY-MM-DD.log        the raw run_tests.sh output
#   latest.md             symlink -> the newest report
#   status                ONE line: "GREEN|RED|DEFERRED <counts> <iso-date>"
#
# DEFERRED = a live robot session held the box past the wait budget (see the
# live-session guard below); the suite did not run and tomorrow's 04:00 picks
# it up. It is written over any older GREEN precisely so nobody reads a stale
# verdict as today's.
#
# `status` is the session-start channel (owner choice: no email, no GUI). Every
# Claude session reads it first and surfaces a RED with the failure list.
#
# Manual invocation is safe and idempotent (it overwrites today's report):
#   tools/nightly_suite.sh
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${PDJ_VENV:-$HOME/Desktop/PDJ_venv/venv}"
REPORTS="$REPO_ROOT/temp/reports/nightly"
RETENTION_DAYS="${JB_NIGHTLY_RETENTION_DAYS:-30}"

DATE="$(date +%F)"
REPORT="$REPORTS/$DATE.md"
JUNIT_DIR="$REPORTS/$DATE-junit"
LOG="$REPORTS/$DATE.log"

# ── Live-session guard: never run the suite alongside a powered robot ────────
# The timer's Persistent=true means a night the Jetson was off fires the missed
# 04:00 run at the NEXT BOOT — which on this box is often the same minute an
# operator powers up for a sitting. Nice=10 + IOSchedulingClass=idle in the unit
# bound CPU and IO but NOT memory, and run_tests.sh's own header records a
# ~417 MB available-memory floor at 4 workers on this 7.3 GB box with zram swap:
# over-commit degrades hard rather than gracefully. An OOM-killed pytest worker
# is a confusing test failure; an OOM-killed ROS node mid-sitting is a robot
# event. So the suite WAITS for the box rather than competing for it.
#
# This is a wait, not a skip: a skip is the delete button `nightly` exists to
# avoid. If the wait budget expires the status file says DEFERRED — never GREEN,
# never silence — and CLAUDE.md's session-start rule surfaces anything that is
# not a fresh GREEN. Tomorrow's 04:00 run picks it up.
#
# JB_NIGHTLY_IGNORE_SESSION=1 skips the guard (deliberate hand-run during a
# sitting). JB_NIGHTLY_DEFER_BUDGET_S caps the wait; the default 2 h leaves
# comfortable headroom under the unit's TimeoutStartSec=4h.
LIVE_SESSION_RE='(ros2 launch jugglebot|jugglebot_launch\.py|teensy_bridge_node|trajectory_node|orchestrator_node|run_mpc\.py)'
DEFER_BUDGET_S="${JB_NIGHTLY_DEFER_BUDGET_S:-7200}"
DEFER_POLL_S="${JB_NIGHTLY_DEFER_POLL_S:-120}"

live_session() {
  # `pgrep -f` matches full command lines, so anything whose ARGV contains one of
  # these names matches — including our own process group when this script is
  # pasted into a `bash -c` (the pattern is then literally in the command line;
  # harmless in production, where the process is the script PATH). Drop our own
  # process group, PID and parent so the guard can never trip on itself.
  local mypgid p pg
  mypgid="$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ')"
  for p in $(pgrep -f "$LIVE_SESSION_RE" 2>/dev/null); do
    [[ "$p" == "$$" || "$p" == "$PPID" ]] && continue
    pg="$(ps -o pgid= -p "$p" 2>/dev/null | tr -d ' ')"
    # Empty pgid = the process exited between pgrep and ps (typically one of our
    # own transient subshells). Gone is not live — only defer for a process we
    # can positively confirm is still there.
    [[ -z "$pg" ]] && continue
    [[ -n "$mypgid" && "$pg" == "$mypgid" ]] && continue
    return 0
  done
  return 1
}

if [[ "${JB_NIGHTLY_IGNORE_SESSION:-0}" != "1" ]] && live_session; then
  echo "nightly_suite: live robot session detected — waiting (budget ${DEFER_BUDGET_S}s)" >&2
  waited=0
  while live_session && (( waited < DEFER_BUDGET_S )); do
    sleep "$DEFER_POLL_S"
    waited=$((waited + DEFER_POLL_S))
  done
  if live_session; then
    mkdir -p "$REPORTS"
    echo "DEFERRED live robot session held the box for ${waited}s; suite NOT run" \
      "$(date -Iseconds)" >"$REPORTS/status"
    echo "nightly_suite: still live after ${waited}s — deferred to the next run" >&2
    exit 0
  fi
  echo "nightly_suite: session cleared after ${waited}s — proceeding" >&2
fi

mkdir -p "$JUNIT_DIR"

# The venv, not the system python (3.8.10, no mujoco/hypothesis/casadi). Sourcing
# activate rather than calling the venv python directly so run_tests.sh's own
# subprocesses (config codegen, the native g++ pre-build) see the same env.
if [[ -f "$VENV_DIR/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "$VENV_DIR/bin/activate"
else
  echo "nightly_suite: venv not found at $VENV_DIR" >&2
fi

cd "$REPO_ROOT" || exit 1

started="$(date -Iseconds)"
start=$SECONDS
# run_tests.sh takes the suite flock itself, so an interactive gate that is still
# running at 04:00 makes this QUEUE (up to 30 min) rather than collide.
JB_JUNIT_DIR="$JUNIT_DIR" ./run_tests.sh --full --hypothesis-profile=ci-deep \
  >"$LOG" 2>&1
rc=$?
duration=$((SECONDS - start))

# ── Render the report from the junit XMLs (counts + trimmed tracebacks) ───────
# Parsing the XML rather than scraping the pytest summary line: xdist reorders
# and interleaves output, and a crashed worker can eat the summary entirely,
# whereas the XML is written per phase by pytest itself.
python - "$REPORT" "$JUNIT_DIR" "$LOG" "$rc" "$duration" "$started" <<'PY'
import os, sys, xml.etree.ElementTree as ET

report, junit_dir, log_path, rc, duration, started = sys.argv[1:7]
rc = int(rc); duration = int(duration)

MAX_FAILURES = 25          # keep the report readable when a whole subsystem reds
TRACEBACK_LINES = 30       # trimmed tail — the full text is in the .log + XML

def parse(path):
    """-> (counts dict, [(classname, name, kind, text), ...])

    pytest reports xfail as <skipped type="pytest.xfail">, so a naive read calls
    an expected failure a skip. They mean opposite things to someone triaging a
    report — a skip is coverage that did NOT run, an xfail is a characterised
    defect that ran and behaved as documented — so they are counted separately.
    """
    counts = {"tests": 0, "failures": 0, "errors": 0, "skipped": 0}
    failures = []
    if not os.path.exists(path):
        return None, failures
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        return {"parse_error": str(exc)}, failures
    suites = root.iter("testsuite") if root.tag != "testsuite" else [root]
    xfailed = 0
    for ts in suites:
        for k in counts:
            counts[k] += int(ts.get(k, 0) or 0)
        for tc in ts.iter("testcase"):
            for kind in ("failure", "error"):
                el = tc.find(kind)
                if el is not None:
                    failures.append((tc.get("classname", ""), tc.get("name", ""),
                                     kind, (el.text or el.get("message", "") or "")))
            sk = tc.find("skipped")
            if sk is not None and (sk.get("type") or "").endswith("xfail"):
                xfailed += 1
    counts["xfailed"] = xfailed
    counts["skipped"] -= xfailed
    counts["passed"] = (counts["tests"] - counts["failures"]
                        - counts["errors"] - counts["skipped"] - xfailed)
    return counts, failures

phases = [("Phase 1 — parallel (-m 'not serial')", "phase1.xml"),
          ("Phase 2 — serial (-m serial)", "phase2.xml")]

lines = [f"# Nightly suite — {os.path.basename(report)[:-3]}", ""]
lines.append(f"* started: `{started}`")
lines.append(f"* duration: **{duration // 60}m {duration % 60}s**")
lines.append("* command: `./run_tests.sh --full --hypothesis-profile=ci-deep`")
lines.append(f"* exit code: `{rc}` — **{'GREEN' if rc == 0 else 'RED'}**")
lines.append("")

total = {"tests": 0, "passed": 0, "failures": 0, "errors": 0, "skipped": 0,
         "xfailed": 0}
all_failures = []
lines.append("| phase | tests | passed | failed | errored | xfailed | skipped |")
lines.append("|---|---|---|---|---|---|---|")
for title, fname in phases:
    counts, failures = parse(os.path.join(junit_dir, fname))
    if counts is None:
        lines.append(f"| {title} | — | — | — | — | — | (no XML: phase did not run) |")
        continue
    if "parse_error" in counts:
        lines.append(f"| {title} | — | — | — | — | — | (unparsable XML) |")
        continue
    for k in total:
        total[k] += counts.get(k, 0)
    lines.append("| {} | {tests} | {passed} | {failures} | {errors} | {xfailed} "
                 "| {skipped} |".format(title, **counts))
    all_failures += [(title, *f) for f in failures]
lines.append("| **total** | {tests} | {passed} | {failures} | {errors} | {xfailed} "
             "| {skipped} |".format(**total))
lines.append("")

if all_failures:
    lines.append(f"## Failures ({len(all_failures)})")
    lines.append("")
    for phase, cls, name, kind, text in all_failures[:MAX_FAILURES]:
        lines.append(f"### `{cls}::{name}` ({kind}, {phase})")
        lines.append("")
        tail = [ln for ln in text.strip().splitlines()][-TRACEBACK_LINES:]
        lines.append("```")
        lines += tail or ["(no traceback captured)"]
        lines.append("```")
        lines.append("")
    if len(all_failures) > MAX_FAILURES:
        lines.append(f"_...{len(all_failures) - MAX_FAILURES} more — see the junit XML._")
        lines.append("")
elif rc != 0:
    # rc != 0 with zero XML failures: run_tests.sh's own guards (zero serial
    # tests collected, missing xdist, lock timeout) or a crashed worker.
    lines.append("## Non-zero exit with no test-level failure")
    lines.append("")
    lines.append("```")
    with open(log_path, errors="replace") as fh:
        lines += fh.read().splitlines()[-40:]
    lines.append("```")
    lines.append("")

lines.append(f"Raw output: `{os.path.basename(log_path)}` · junit: "
             f"`{os.path.basename(junit_dir)}/`")
lines.append("")

with open(report, "w") as fh:
    fh.write("\n".join(lines))

# One-line machine-readable status for the session-start check.
verdict = "GREEN" if rc == 0 else "RED"
summary = ("{passed}/{tests} passed, {failures} failed, {errors} errored, "
           "{xfailed} xfailed, {skipped} skipped".format(**total))
with open(os.path.join(os.path.dirname(report), "status"), "w") as fh:
    fh.write(f"{verdict} {summary} {started}\n")
PY
render_rc=$?

# The renderer is the only writer of `status`, and `status` is the ONLY channel
# a session sees. If it dies (no interpreter, unreadable XML), a stale GREEN from
# days ago would be read as today's verdict — worse than no runner at all. Write
# an honest RED instead.
if [[ $render_rc -ne 0 ]]; then
  echo "nightly_suite: report renderer failed (rc=$render_rc)" >&2
  echo "RED report-renderer-failed (suite rc=$rc); see $LOG $started" \
    >"$REPORTS/status"
fi

# `latest.md` — relative target so the symlink survives a repo move.
ln -sfn "$DATE.md" "$REPORTS/latest.md"

# Retention: date-named artifacts only. `latest.md` and `status` are never
# matched by the 20* glob, so a long gap in runs cannot delete the status file.
find "$REPORTS" -maxdepth 1 -name '20*' -mtime "+$RETENTION_DAYS" \
  -exec rm -rf {} + 2>/dev/null

echo "nightly_suite: rc=$rc, ${duration}s -> $REPORT"
exit "$rc"
