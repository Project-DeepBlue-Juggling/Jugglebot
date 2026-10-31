#!/usr/bin/env bash
# tools/night_shift.sh: the Night Shift runner (plans/active/night-shift.md P1).
#
# Night Shift is unattended overnight work on the board: a headless `claude -p`
# runs the `/night-shift` orchestrator protocol in its OWN clone of the repo,
# works the open Items one unit at a time, and leaves a report for the morning.
# This script is everything around that one call: the guards that decide
# whether it may run at all, the clone it runs in, the candidate preflight that
# skips a night with nothing new, and an explicit status for every outcome.
#
# Armed by systemd --user (jugglebot-night-shift.timer, 04:45, after the 04:00
# nightly; units and install block in tools/systemd/). The clone is
# $NIGHT_SHIFT_CLONE (~/night-shift/Jugglebot), on `main` only, fast-forwarded
# from origin each run; the runner never switches its branch.
#
# Guards, in order. Each one that stops the run writes a status (never silence):
#   1. flock on $REPORTS/run.lock         two runs never overlap        -> OVERLAP
#   2. $LIVE/temp/night-shift.hold        the owner's off switch        -> HELD
#   3. live robot session (tools/lib/live_session.sh, the nightly's guard):
#      wait up to the budget, then give up                              -> DEFERRED
#   4. ANTHROPIC_API_KEY set (the unit's EnvironmentFile)               -> NO_API_KEY
#   5. clone on `main` and clean; never checkout/stash/reset, the owner
#      sorts a dirty clone out by hand                                  -> CLONE_DIRTY
#   6. ff-only pull from origin                                         -> PULL_FAILED
#   7. the deny-list settings installed into the clone on EVERY run;
#      claude never runs without them                                   -> NO_SETTINGS
#   8. preflight: nothing new since the last real run = no claude call  -> NOTHING_NEW
#
# Permission model: `--permission-mode acceptEdits --permission-prompts none`
# with tools/night_shift_settings.json installed as the clone's
# .claude/settings.local.json (allow every tool, deny flash/launch/hardware/
# probes/system services/the live checkout; deny wins over every allow). The
# `--dangerously-skip-permissions` flag is NOT used: the settings file's comment
# records why the deny list is the safety layer, and the bridge is powered at
# night with a single-owner UDP link.
#
# Outputs, under $NIGHT_SHIFT_REPORTS ($CLONE/temp/reports/night-shift, gitignored):
#   last_run.json           the run's summary, written by EVERY exit path
#   history.log             one line per run: "<iso> <STATUS> <detail>"
#   candidates-<date>.json  the preflight's candidate set; candidates-last.json
#                           is the last REAL run's (a dry run never updates it)
#   nightly-<date>.md       copy of the nightly's latest.md for the orchestrator
#   <date>.claude.json/.err the claude result and stderr
#   dry-<date>.md           the dry-run report (dry runs only)
# stdout gets one line, "night_shift: <STATUS> <detail>".
#
# Status words. Exit 0: OK, DRY_OK, HELD, DEFERRED, OVERLAP, NOTHING_NEW.
# Exit 1: NO_GUARD, NO_API_KEY, NO_CLONE, CLONE_DIRTY, PULL_FAILED, NO_SETTINGS,
# PREFLIGHT_FAILED, CLAUDE_MISSING, CLAUDE_TIMEOUT, CLAUDE_FAILED,
# POST_FETCH_FAILED, PUSH_REJECTED, RUN_LEFT_DIRTY, DRY_CHANGED, DRY_NO_REPORT,
# INTERNAL_ERROR. A real run that claimed the nightly ticker and ends in any
# status but OK re-raises it (ticker_reraised in last_run.json), so a failed
# night never swallows a RED nightly.
#
# Single-writer rule: this script never runs a git command that writes in
# $NIGHT_SHIFT_LIVE_REPO (~/Desktop/Jugglebot, where a live Session may be
# working); its one git read there is `remote get-url origin`, and only when the
# clone must be created and NIGHT_SHIFT_ORIGIN is unset. Otherwise it reads only
# the hold file and the nightly report dir, and its one write is the ticker
# claim on a real run (`nightly_ticker.sh check` renames ticker.raised to
# ticker.lowered, exactly as a morning Session's check would).
#
# Manual use (safe; a dry run changes nothing and pushes nothing):
#   tools/night_shift.sh --dry-run
# Env knobs (defaults): NIGHT_SHIFT_CLONE, NIGHT_SHIFT_LIVE_REPO, NIGHT_SHIFT_ORIGIN,
# NIGHT_SHIFT_REPORTS, NIGHT_SHIFT_DRY_RUN (0), NIGHT_SHIFT_MAX_USD (unset = no fuse),
# NIGHT_SHIFT_MODEL (fable), NIGHT_SHIFT_TIMEOUT_S (14400), NIGHT_SHIFT_DEADLINE
# (08:00), NIGHT_SHIFT_DEFER_BUDGET_S (7200), NIGHT_SHIFT_DEFER_POLL_S (120),
# NIGHT_SHIFT_WAIT_NIGHTLY_S (3600), NIGHT_SHIFT_WAIT_NIGHTLY_POLL_S (60),
# NIGHT_SHIFT_IGNORE_SESSION (0; hand runs only), NIGHT_SHIFT_SKIP_ENV (0; tests),
# NIGHT_SHIFT_PYTHON (the venv python if present, else python3), NIGHT_SHIFT_CLAUDE
# (the claude binary; default: PATH, then the newest ~/.nvm install).
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for arg in "$@"; do
  case "$arg" in
    --dry-run) NIGHT_SHIFT_DRY_RUN=1 ;;
    *) echo "usage: $0 [--dry-run]" >&2; exit 2 ;;
  esac
done

CLONE="${NIGHT_SHIFT_CLONE:-$HOME/night-shift/Jugglebot}"
LIVE="${NIGHT_SHIFT_LIVE_REPO:-$HOME/Desktop/Jugglebot}"
REPORTS="${NIGHT_SHIFT_REPORTS:-$CLONE/temp/reports/night-shift}"
DRY_RUN="${NIGHT_SHIFT_DRY_RUN:-0}"
MAX_USD="${NIGHT_SHIFT_MAX_USD:-}"   # empty or 0 = no --max-budget-usd (owner, 2026-10-31)
MODEL="${NIGHT_SHIFT_MODEL:-fable}"
TIMEOUT_S="${NIGHT_SHIFT_TIMEOUT_S:-14400}"
DEADLINE="${NIGHT_SHIFT_DEADLINE:-08:00}"
DEFER_BUDGET_S="${NIGHT_SHIFT_DEFER_BUDGET_S:-7200}"
DEFER_POLL_S="${NIGHT_SHIFT_DEFER_POLL_S:-120}"
WAIT_NIGHTLY_S="${NIGHT_SHIFT_WAIT_NIGHTLY_S:-3600}"
WAIT_NIGHTLY_POLL_S="${NIGHT_SHIFT_WAIT_NIGHTLY_POLL_S:-60}"
IGNORE_SESSION="${NIGHT_SHIFT_IGNORE_SESSION:-0}"
SKIP_ENV="${NIGHT_SHIFT_SKIP_ENV:-0}"
VENV_DIR="$HOME/Desktop/PDJ_venv/venv"
if [[ -n "${NIGHT_SHIFT_PYTHON:-}" ]]; then
  PYTHON="$NIGHT_SHIFT_PYTHON"
elif [[ -x "$VENV_DIR/bin/python" ]]; then
  PYTHON="$VENV_DIR/bin/python"
else
  PYTHON="python3"
fi
HOLD_FILE="$LIVE/temp/night-shift.hold"
NIGHTLY_DIR="$LIVE/temp/reports/nightly"

DATE="$(date +%F)"
STARTED="$(date -Iseconds)"
START_S=$SECONDS
DRY=""
[[ "$DRY_RUN" == "1" ]] && DRY=1

# Summary fields: unset means "not reached" and becomes null in last_run.json.
unset NS_COST NS_TURNS NS_DENIALS NS_SPAWNED NS_CLAUDE_EXIT NS_HEAD_BEFORE \
  NS_HEAD_AFTER NS_BRANCH_AFTER NS_UNPUSHED NS_REPORT NS_CANDIDATES \
  NS_NIGHTLY_LINE NS_PREFLIGHT NS_RESULT NS_MAIN_COMMITS NS_TICKER_RERAISED NS_COST_RECORDED
STATUS_WRITTEN=0
TICKER_CLAIMED=0

# ── Exit paths ────────────────────────────────────────────────────────────────
one_line() { tr '\n\r\t' '   ' <<<"$1" | sed 's/  */ /g; s/ *$//'; }

# The orchestrator cannot see its own total_cost_usd (the JSON result arrives
# after it exits), so the runner writes the figure into the night's report:
# one line at the end of `## Cost` (the last section). A dry run's report is a
# plain file; a real run's is on main, so the line is its own docs commit,
# pushed, and dropped again if main moved meanwhile (nothing may be left on the
# clone's main unpushed). NS_COST_RECORDED says what happened.
cost_line() { printf 'total API cost: USD %s (runner, total_cost_usd)\n' "$NS_COST"; }
record_cost_dry() {
  [[ -n "${NS_COST:-}" && -s "$NS_REPORT" ]] || { export NS_COST_RECORDED=no-report; return 0; }
  cost_line >>"$NS_REPORT" && export NS_COST_RECORDED=file || export NS_COST_RECORDED=write-failed
}
record_cost_main() {
  if [[ -z "${NS_COST:-}" ]]; then export NS_COST_RECORDED=no-cost; return 0; fi
  if [[ ! -s "$NS_REPORT" ]]; then export NS_COST_RECORDED=no-report; return 0; fi
  local rel="docs/night-shift/$DATE.md"
  if cost_line >>"$NS_REPORT" \
     && git -C "$CLONE" add -- "$rel" \
     && git -C "$CLONE" commit -q -m "docs(night-shift): total API cost $DATE" \
          -m "Logbook-Entry: $DATE-night-shift" -- "$rel" \
     && git -C "$CLONE" push -q origin HEAD:main 2>>"$REPORTS/$DATE.cost.err"; then
    export NS_COST_RECORDED=pushed
  else
    export NS_COST_RECORDED=rejected
    git -C "$CLONE" fetch -q origin 2>/dev/null
    git -C "$CLONE" reset -q --hard origin/main 2>/dev/null
  fi
  export_posthoc_git
}

write_status() {   # write_status STATUS DETAIL -> last_run.json + history.log
  local status="$1" detail; detail="$(one_line "$2")"
  STATUS_WRITTEN=1
  mkdir -p "$REPORTS" 2>/dev/null
  local finished; finished="$(date -Iseconds)"
  if ! NS_STATUS="$status" NS_DETAIL="$detail" NS_DATE="$DATE" NS_DRY="$DRY_RUN" \
       NS_STARTED="$STARTED" NS_FINISHED="$finished" \
       NS_DURATION="$((SECONDS - START_S))" \
       "$PYTHON" - "$REPORTS/last_run.json" <<'PY'
import json, os, sys

e = os.environ.get

def num(key, cast):
    v = e(key)
    if v in (None, ""):
        return None
    try:
        return cast(v)
    except ValueError:
        return None

unpushed = e("NS_UNPUSHED")
out = {
    "date": e("NS_DATE"),
    "status": e("NS_STATUS"),
    "detail": e("NS_DETAIL"),
    "dry_run": e("NS_DRY") == "1",
    "started": e("NS_STARTED"),
    "finished": e("NS_FINISHED"),
    "duration_s": num("NS_DURATION", int),
    "cost_usd": num("NS_COST", float),
    "num_turns": num("NS_TURNS", int),
    "permission_denials": num("NS_DENIALS", int),
    "subagents_spawned": num("NS_SPAWNED", int),
    "claude_exit": num("NS_CLAUDE_EXIT", int),
    "head_before": e("NS_HEAD_BEFORE"),
    "head_after": e("NS_HEAD_AFTER"),
    "clone_branch_after": e("NS_BRANCH_AFTER"),
    "unpushed": None if unpushed is None else unpushed.split(),
    "report": e("NS_REPORT"),
    "candidates": e("NS_CANDIDATES"),
    "nightly_line": e("NS_NIGHTLY_LINE"),
    "preflight": e("NS_PREFLIGHT"),
    "result": e("NS_RESULT"),
    "main_commits": (None if e("NS_MAIN_COMMITS") is None
                     else [c for c in e("NS_MAIN_COMMITS").splitlines() if c]),
    "ticker_reraised": e("NS_TICKER_RERAISED") == "1",
    "cost_recorded": e("NS_COST_RECORDED"),
}
tmp = sys.argv[1] + ".tmp"
with open(tmp, "w") as fh:
    json.dump(out, fh, indent=2)
    fh.write("\n")
os.replace(tmp, sys.argv[1])
PY
  then
    # The interpreter itself is gone: still leave a parseable minimum.
    printf '{"date": "%s", "status": "%s", "detail": "summary writer failed"}\n' \
      "$DATE" "$status" >"$REPORTS/last_run.json" 2>/dev/null
  fi
  printf '%s %s %s\n' "$finished" "$status" "$detail" >>"$REPORTS/history.log" 2>/dev/null
  echo "night_shift: $status $detail"
}

finish() {         # finish STATUS DETAIL: the only way out after argument parsing
  local status="$1" detail="${2:-}"
  trap - EXIT
  # A real run that claimed the nightly ticker and then did not finish OK must
  # hand the claim back: the morning Session reads "LOWERED by night-shift" as
  # "the report covers it", and there is no report, so a RED would be swallowed.
  # `raise` rewrites ticker.raised from `status` and removes ticker.lowered.
  if [[ "$TICKER_CLAIMED" == 1 && "$status" != OK ]]; then
    if NIGHTLY_REPORTS_DIR="$NIGHTLY_DIR" "$CLONE/tools/nightly_ticker.sh" raise >/dev/null 2>&1; then
      export NS_TICKER_RERAISED=1
    else
      detail="$detail (ticker re-raise FAILED)"
    fi
  fi
  write_status "$status" "$detail"
  case "$status" in
    OK|DRY_OK|HELD|DEFERRED|OVERLAP|NOTHING_NEW) exit 0 ;;
    *) exit 1 ;;
  esac
}

# Anything that ends the script without finish() (an unbound variable under
# `set -u`, a signal) still gets an explicit failure status.
trap '[[ $STATUS_WRITTEN == 1 ]] || finish INTERNAL_ERROR "runner exited without a status (line $LINENO)"' EXIT
# A signal (systemd's TimeoutStartSec kill, a Ctrl-C on a hand run) is passed on
# to claude first: `timeout` runs it in its own process group, which a signal to
# OUR group would never reach, and a claude left running after its runner
# reported a status is exactly what must not happen.
CHILD_PID=""
on_signal() {
  if [[ -n "$CHILD_PID" ]]; then
    kill -TERM "$CHILD_PID" 2>/dev/null
    wait "$CHILD_PID" 2>/dev/null
    export_posthoc_git
  fi
  finish INTERNAL_ERROR "terminated by signal"
}
trap on_signal TERM INT HUP

export_posthoc_git() {   # head/branch/unpushed after claude ran; best-effort
  NS_HEAD_AFTER="$(git -C "$CLONE" rev-parse -q --verify main 2>/dev/null)"
  NS_BRANCH_AFTER="$(git -C "$CLONE" symbolic-ref -q --short HEAD 2>/dev/null || echo DETACHED)"
  NS_UNPUSHED="$(git -C "$CLONE" log --format=%H origin/main..main 2>/dev/null | tr '\n' ' ')"
  NS_UNPUSHED="${NS_UNPUSHED% }"
  export NS_HEAD_AFTER NS_BRANCH_AFTER NS_UNPUSHED
}

# ── 1. Overlap ────────────────────────────────────────────────────────────────
mkdir -p "$REPORTS" || { echo "night_shift: INTERNAL_ERROR cannot create $REPORTS"; STATUS_WRITTEN=1; exit 1; }
exec 8>"$REPORTS/run.lock"
if ! flock -n 8; then
  # Do not overwrite the running night's last_run.json: say so on stdout and in
  # history.log only.
  STATUS_WRITTEN=1
  trap - EXIT
  printf '%s %s %s\n' "$(date -Iseconds)" OVERLAP "another run holds $REPORTS/run.lock" \
    >>"$REPORTS/history.log"
  echo "night_shift: OVERLAP another run holds $REPORTS/run.lock"
  exit 0
fi

# ── 2. Hold file ──────────────────────────────────────────────────────────────
if [[ -e "$HOLD_FILE" ]]; then
  hold_line="$(head -n1 "$HOLD_FILE" 2>/dev/null)"
  finish HELD "${hold_line:-$HOLD_FILE}"
fi

# ── 3. Live robot session ─────────────────────────────────────────────────────
# The same guard and semantics as the nightly: wait for the box, then give up
# with an explicit DEFERRED. Missing guard library = fail closed.
if [[ "$IGNORE_SESSION" != "1" ]]; then
  # shellcheck disable=SC1091
  source "$SCRIPT_DIR/lib/live_session.sh" 2>/dev/null \
    || finish NO_GUARD "cannot source $SCRIPT_DIR/lib/live_session.sh"
  declare -F live_session >/dev/null && declare -F wait_for_live_session_to_clear >/dev/null \
    || finish NO_GUARD "live_session.sh does not define the guard functions"
  if live_session; then
    LIVE_SESSION_WAITED_S=0
    if ! wait_for_live_session_to_clear "$DEFER_BUDGET_S" "$DEFER_POLL_S" night_shift; then
      finish DEFERRED "live robot session held the box for ${LIVE_SESSION_WAITED_S}s; claude NOT run"
    fi
  fi
fi

# ── 4. API key (before anything is cloned or fetched) ─────────────────────────
[[ -n "${ANTHROPIC_API_KEY:-}" ]] \
  || finish NO_API_KEY "ANTHROPIC_API_KEY empty (EnvironmentFile ~/.config/jugglebot/night-shift.env)"
export ANTHROPIC_API_KEY

# ── 5. The clone: main, clean, ff from origin ─────────────────────────────────
if [[ ! -e "$CLONE/.git" ]]; then
  ORIGIN="${NIGHT_SHIFT_ORIGIN:-}"
  if [[ -z "$ORIGIN" ]]; then
    ORIGIN="$(git -C "$LIVE" remote get-url origin 2>/dev/null)" \
      || ORIGIN="https://github.com/Project-DeepBlue-Juggling/Jugglebot.git"
  fi
  if [[ ! -e "$CLONE" ]]; then
    mkdir -p "$(dirname "$CLONE")" && git clone -q --branch main "$ORIGIN" "$CLONE" \
      || finish NO_CLONE "git clone of $ORIGIN into $CLONE failed"
  else
    # The default reports dir lives inside the clone, so a first run that was
    # HELD or DEFERRED leaves $CLONE/temp behind. That and nothing else may be
    # there; clone beside it and copy in (temp/ is gitignored).
    extra="$(ls -A "$CLONE" | grep -vx temp | head -n5 | tr '\n' ' ')"
    [[ -z "$extra" ]] || finish NO_CLONE "$CLONE exists without .git and holds: $extra"
    staging="$(dirname "$CLONE")/.$(basename "$CLONE").cloning.$$"
    git clone -q --branch main "$ORIGIN" "$staging" && cp -a "$staging/." "$CLONE/" \
      || { rm -rf "$staging"; finish NO_CLONE "git clone of $ORIGIN via $staging failed"; }
    rm -rf "$staging"
  fi
fi
branch="$(git -C "$CLONE" symbolic-ref -q --short HEAD 2>/dev/null || echo DETACHED)"
porcelain="$(git -C "$CLONE" status --porcelain 2>&1)" \
  || finish CLONE_DIRTY "git status failed in $CLONE: $porcelain"
if [[ "$branch" != "main" || -n "$porcelain" ]]; then
  finish CLONE_DIRTY "branch=$branch; $(head -n5 <<<"$porcelain" | tr '\n' ';')"
fi
fetch_err="$( { git -C "$CLONE" fetch -q origin && git -C "$CLONE" merge -q --ff-only origin/main; } 2>&1)" \
  || finish PULL_FAILED "$fetch_err"
NS_HEAD_BEFORE="$(git -C "$CLONE" rev-parse HEAD)"
export NS_HEAD_BEFORE

# ── 6. Deny-list settings, reinstalled every run ──────────────────────────────
SETTINGS_SRC="$CLONE/tools/night_shift_settings.json"
SETTINGS_DST="$CLONE/.claude/settings.local.json"
[[ -f "$SETTINGS_SRC" ]] || finish NO_SETTINGS "missing $SETTINGS_SRC"
mkdir -p "$CLONE/.claude" && install -m 600 "$SETTINGS_SRC" "$SETTINGS_DST" \
  && cmp -s "$SETTINGS_SRC" "$SETTINGS_DST" \
  || finish NO_SETTINGS "could not install $SETTINGS_DST"
"$PYTHON" - "$SETTINGS_DST" <<'PY' || finish NO_SETTINGS "$SETTINGS_DST has no deny list"
import json, sys
deny = json.load(open(sys.argv[1])).get("permissions", {}).get("deny")
sys.exit(0 if isinstance(deny, list) and deny else 1)
PY

# ── 7. Environment (the gate the orchestrator runs needs ROS + the venv) ──────
# The suite imports the pure-Python ROS packages for real, so source Foxy as the
# nightly does. The clone has NO ros_ws/install: source only what exists. ROS
# setup scripts reference unset vars, so relax nounset around them. The venv
# goes last so its python stays first on PATH.
if [[ "$SKIP_ENV" != "1" ]]; then
  set +u
  # shellcheck disable=SC1091
  [[ -f /opt/ros/foxy/setup.bash ]] && source /opt/ros/foxy/setup.bash
  # shellcheck disable=SC1091
  [[ -f "$CLONE/ros_ws/install/setup.bash" ]] && source "$CLONE/ros_ws/install/setup.bash"
  # shellcheck disable=SC1091
  [[ -f "$VENV_DIR/bin/activate" ]] && source "$VENV_DIR/bin/activate"
  set -u
fi
[[ -n "${ANTHROPIC_API_KEY:-}" ]] || finish NO_API_KEY "ANTHROPIC_API_KEY lost after sourcing the environment"

# ── 8. Preflight: anything new since the last real run? ───────────────────────
CANDIDATES="$REPORTS/candidates-$DATE.json"
NS_PREFLIGHT="$(cd "$CLONE" && "$PYTHON" "$CLONE/tools/night_shift_preflight.py" \
  --repo "$CLONE" --out "$CANDIDATES" --previous "$REPORTS/candidates-last.json" 2>"$REPORTS/preflight.err")"
pf_rc=$?
NS_PREFLIGHT="$(one_line "$NS_PREFLIGHT")"
export NS_PREFLIGHT
case "$pf_rc" in
  0) ;;
  3) [[ -n "$DRY" ]] || { export NS_CANDIDATES="$CANDIDATES"; finish NOTHING_NEW "${NS_PREFLIGHT:-preflight rc 3}"; } ;;
  *) finish PREFLIGHT_FAILED "rc=$pf_rc ${NS_PREFLIGHT:-} $(tail -n3 "$REPORTS/preflight.err" 2>/dev/null)" ;;
esac
[[ -s "$CANDIDATES" ]] || finish PREFLIGHT_FAILED "rc=$pf_rc but $CANDIDATES not written"
export NS_CANDIDATES="$CANDIDATES"

# ── 9. The nightly line (a real run claims the ticker; a dry run never does) ──
# The 04:00 nightly can still be running at 04:45 (it waits up to 2 h on a live
# Sitting; ci-deep takes ~26 min). Claiming yesterday's ticker would hand the
# morning a stale verdict, so a real run first waits for today's run to land:
# ticker.raised or ticker.lowered whose run date is today, or a `status` whose
# trailing iso-date is today. The night is never blocked on the nightly: when
# the budget runs out the line is prefixed NIGHTLY_STALE and the run goes on.
nightly_is_today() {
  local tok
  tok="$(head -n1 "$NIGHTLY_DIR/ticker.raised" 2>/dev/null | cut -d' ' -f1)"
  [[ "${tok:0:10}" == "$DATE" ]] && return 0
  tok="$(head -n1 "$NIGHTLY_DIR/ticker.lowered" 2>/dev/null | cut -d' ' -f1)"
  [[ "${tok:0:10}" == "$DATE" ]] && return 0
  tok="$(head -n1 "$NIGHTLY_DIR/status" 2>/dev/null)"
  tok="${tok##* }"
  [[ "${tok:0:10}" == "$DATE" ]]
}
if [[ -z "$DRY" ]]; then
  nightly_waited=0
  while ! nightly_is_today && (( nightly_waited < WAIT_NIGHTLY_S )); do
    sleep "$WAIT_NIGHTLY_POLL_S"
    nightly_waited=$((nightly_waited + WAIT_NIGHTLY_POLL_S))
  done
  stale=""
  nightly_is_today || stale="NIGHTLY_STALE "
  NS_NIGHTLY_LINE="$(NIGHTLY_REPORTS_DIR="$NIGHTLY_DIR" "$CLONE/tools/nightly_ticker.sh" \
    check --who "night-shift $DATE" 2>&1)" || NS_NIGHTLY_LINE="TICKER_FAILED $NS_NIGHTLY_LINE"
  [[ "$NS_NIGHTLY_LINE" == CLAIMED* ]] && TICKER_CLAIMED=1
  NS_NIGHTLY_LINE="$stale$NS_NIGHTLY_LINE"
elif [[ -s "$NIGHTLY_DIR/status" ]]; then
  NS_NIGHTLY_LINE="UNCLAIMED $(head -n1 "$NIGHTLY_DIR/status")"
else
  NS_NIGHTLY_LINE="UNCLAIMED NEVER"
fi
NS_NIGHTLY_LINE="$(one_line "$NS_NIGHTLY_LINE" | tr -d '"')"
export NS_NIGHTLY_LINE
NIGHTLY_COPY="$REPORTS/nightly-$DATE.md"
rm -f "$NIGHTLY_COPY"
[[ -e "$NIGHTLY_DIR/latest.md" ]] && cp -L "$NIGHTLY_DIR/latest.md" "$NIGHTLY_COPY" 2>/dev/null

# ── 10. claude -p ─────────────────────────────────────────────────────────────
# Which claude: $NIGHT_SHIFT_CLAUDE, else the first on PATH, else the newest nvm
# install. Under systemd --user PATH is the bare system one (no nvm bin dir), so
# the fallback is what the timer actually uses; its bin dir is prepended to PATH
# because the `claude` entry point is a node script that needs `node` beside it.
# /usr/local/lib/nodejs/node-v20*/bin/claude on this box is a broken install
# ("native binary not installed") and is deliberately never picked by the glob.
CLAUDE_BIN="${NIGHT_SHIFT_CLAUDE:-}"
if [[ -z "$CLAUDE_BIN" ]]; then
  CLAUDE_BIN="$(command -v claude 2>/dev/null || true)"
fi
if [[ -z "$CLAUDE_BIN" ]]; then
  CLAUDE_BIN="$(ls -d "$HOME"/.nvm/versions/node/*/bin/claude 2>/dev/null | sort -V | tail -n1)"
fi
[[ -n "$CLAUDE_BIN" && -x "$CLAUDE_BIN" ]] || finish CLAUDE_MISSING "no claude on PATH, \$NIGHT_SHIFT_CLAUDE or under ~/.nvm"
export PATH="$(dirname "$CLAUDE_BIN"):$PATH"
if [[ -n "$DRY" ]]; then
  export NS_REPORT="$REPORTS/dry-$DATE.md"
  rm -f "$NS_REPORT"   # a stale report must not pass for tonight's
else
  export NS_REPORT="$CLONE/docs/night-shift/$DATE.md"
fi
PROMPT="/night-shift $DATE --candidates $CANDIDATES --reports $REPORTS --nightly \"$NS_NIGHTLY_LINE\" --nightly-report $NIGHTLY_COPY --deadline $DEADLINE${DRY:+ --dry-run}"
CLAUDE_JSON="$REPORTS/$DATE.claude.json"
CLAUDE_ERR="$REPORTS/$DATE.claude.err"
cd "$CLONE" || finish INTERNAL_ERROR "cannot cd $CLONE"
# timeout puts claude in its own process group and signals the whole group, so
# agents and suite runs under it die with it. fd 8 (the run lock) is closed for
# the child: a straggler must not keep the next night locked out. Started in the
# background and waited on so on_signal can pass a signal on (a foreground
# child would delay our trap until it exited by itself).
claude_start=$SECONDS
# The budget fuse is opt-in: the owner wants the first nights unlimited and
# will set NIGHT_SHIFT_MAX_USD once the cost line in the reports shows a norm.
budget_args=()
if [[ -n "$MAX_USD" && "$MAX_USD" != 0 ]]; then budget_args=(--max-budget-usd "$MAX_USD"); fi
timeout --kill-after=120 "$TIMEOUT_S" "$CLAUDE_BIN" -p "$PROMPT" \
  --model "$MODEL" "${budget_args[@]}" \
  --permission-mode acceptEdits --permission-prompts none \
  --output-format json </dev/null >"$CLAUDE_JSON" 2>"$CLAUDE_ERR" 8>&- &
CHILD_PID=$!
wait "$CHILD_PID"
claude_rc=$?
CHILD_PID=""
export NS_CLAUDE_EXIT="$claude_rc"
export_posthoc_git

if (( claude_rc == 124 )) || { (( claude_rc == 137 )) && (( SECONDS - claude_start >= TIMEOUT_S )); }; then
  finish CLAUDE_TIMEOUT "killed after ${TIMEOUT_S}s; unpushed: ${NS_UNPUSHED:-none}"
fi
err_tail="$(tail -n3 "$CLAUDE_ERR" 2>/dev/null)"
(( claude_rc == 0 )) || finish CLAUDE_FAILED "rc=$claude_rc $err_tail"

# Parse the result (stdlib only). Tab-separated key/value lines on stdout.
parsed="$("$PYTHON" - "$CLAUDE_JSON" <<'PY'
import json, sys
try:
    data = json.load(open(sys.argv[1]))
except (OSError, ValueError):
    sys.exit(2)
if isinstance(data, list):   # stream-shaped output: take the result message
    data = next((m for m in reversed(data)
                 if isinstance(m, dict) and m.get("type") == "result"), None)
if not isinstance(data, dict):
    sys.exit(2)
def clean(v):
    return str(v).replace("\t", " ").replace("\n", " ").replace("\r", " ")
stats = data.get("subagent_stats") or {}
fields = {
    "NS_COST": data.get("total_cost_usd"),
    "NS_TURNS": data.get("num_turns"),
    "NS_DENIALS": len(data.get("permission_denials") or []),
    "NS_SPAWNED": stats.get("spawned") if isinstance(stats, dict) else None,
    "NS_RESULT": str(data.get("result") or "")[:400],
}
for k, v in fields.items():
    if v is not None:
        print("%s\t%s" % (k, clean(v)))
sys.exit(3 if data.get("is_error") else 0)
PY
)"
parse_rc=$?
while IFS=$'\t' read -r key value; do
  [[ -n "$key" ]] && export "$key=$value"
done <<<"$parsed"
case "$parse_rc" in
  0) ;;
  3) finish CLAUDE_FAILED "is_error: ${NS_RESULT:-}" ;;
  *) finish CLAUDE_FAILED "unparsable result JSON; $err_tail" ;;
esac

# ── 11. After the run: nothing may be left on main unpushed ───────────────────
git -C "$CLONE" fetch -q origin 2>/dev/null \
  || finish POST_FETCH_FAILED "could not fetch origin to check for unpushed commits"
export_posthoc_git
# What reached origin/main tonight: recorded for the report, not judged here.
NS_MAIN_COMMITS="$(git -C "$CLONE" log --format='%H %s' "$NS_HEAD_BEFORE..origin/main" 2>/dev/null)"
export NS_MAIN_COMMITS
[[ -z "$NS_UNPUSHED" ]] || finish PUSH_REJECTED "unpushed on main: $NS_UNPUSHED"

if [[ -n "$DRY" ]]; then
  # A dry run changes nothing: main where it was, the clone on main and clean.
  dry_porcelain="$(git -C "$CLONE" status --porcelain 2>&1 | head -n5 | tr '\n' ';')"
  if [[ "$NS_HEAD_AFTER" != "$NS_HEAD_BEFORE" || "$NS_BRANCH_AFTER" != "main" || -n "$dry_porcelain" ]]; then
    finish DRY_CHANGED "main $NS_HEAD_BEFORE -> $NS_HEAD_AFTER, branch=$NS_BRANCH_AFTER; $dry_porcelain"
  fi
  [[ -s "$NS_REPORT" ]] || finish DRY_NO_REPORT "no $NS_REPORT"
  record_cost_dry
  finish DRY_OK "report $NS_REPORT, cost ${NS_COST:-?} USD"
fi

# A real run must hand the clone back the way it found it: on main and clean.
# Anything else would be CLONE_DIRTY tomorrow; say so tonight, with the evidence.
run_porcelain="$(git -C "$CLONE" status --porcelain 2>&1 | head -n5 | tr '\n' ';')"
if [[ "$NS_BRANCH_AFTER" != "main" || -n "$run_porcelain" ]]; then
  finish RUN_LEFT_DIRTY "branch=$NS_BRANCH_AFTER; $run_porcelain"
fi

# The baseline for tomorrow's "anything new?" is the board AFTER tonight's work
# (Items closed or edited tonight are not news), so snapshot the post-run clone
# rather than keep the pre-run candidates. A snapshot: its exit code is moot.
(cd "$CLONE" && "$PYTHON" "$CLONE/tools/night_shift_preflight.py" --repo "$CLONE" \
  --out "$REPORTS/candidates-last.json") >/dev/null 2>>"$REPORTS/preflight.err"
record_cost_main
finish OK "cost ${NS_COST:-?} USD (${NS_COST_RECORDED:-?}), turns ${NS_TURNS:-?}, main ${NS_HEAD_BEFORE:0:8} -> ${NS_HEAD_AFTER:0:8}"
