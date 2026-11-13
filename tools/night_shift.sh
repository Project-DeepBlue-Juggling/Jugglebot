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
#   2b. no Fable model: NIGHT_SHIFT_MODEL / _FALLBACK_MODEL naming Fable stop
#      the run at once, before the live-session wait, any credential read or
#      the clone (owner, 2026-11-02)                                    -> FORBIDDEN_MODEL
#   3. live robot session (tools/lib/live_session.sh, the nightly's guard):
#      wait up to the budget, then give up                              -> DEFERRED
#   4. a credential (the unit's EnvironmentFile): ANTHROPIC_API_KEY and/or
#      CLAUDE_CODE_OAUTH_TOKEN (the Claude subscription, `claude setup-token`)
#                                                                       -> NO_API_KEY
#   5. clone on `main` and clean; never checkout/stash/reset here. The end of
#      the previous run saved its work (see "A night that stops early"), so
#      this only fires when that failed: the owner sorts it out by hand -> CLONE_DIRTY
#   6. ff-only pull from origin                                         -> PULL_FAILED
#   6a. $REPORTS/ungated.hold: an earlier night ended UNGATED_MERGE. Loud, not
#      HELD: a stub report naming the file, the flagged commits and the clearing
#      step is pushed every such night, exit 1 (the ticker is not claimed yet,
#      so it stays raised for the morning Session)                       -> UNGATED_HOLD
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
#   landed-<date>.txt/.py   the landed check's output, and the checker as extracted
#                           from the night's starting commit
#   ungated.hold            written on an ungated landing; blocks the next nights
# stdout gets one line, "night_shift: <STATUS> <detail>".
#
# Credentials (owner, 2026-11-02: "default to API, but if there are not enough
# funds available there, swap to the subscription"). With an API key the night
# runs on it, on $NIGHT_SHIFT_MODEL. If that run ends on an exhausted credit
# balance (HTTP 400 "Credit balance is too low", the 2026-11-02 night) and a
# subscription token is set, the runner RESUMES the same session (`--resume`,
# probed 2026-11-02: a session begun on the key resumes on the token) with the
# key removed from claude's environment, so the night carries on where it
# stopped. The resumed leg runs on $NIGHT_SHIFT_FALLBACK_MODEL (opus; the
# probe showed Fable is not in the subscription, "out of usage credits", Opus
# is). With no key at all the night runs on the subscription from the start.
#
# Models (owner, 2026-11-02): the orchestrator is Opus ("more appropriate given
# the size of the work chunks that we want to be happening overnight") and Fable
# is never invoked by any part of the Night Shift. The runner refuses a Fable
# model knob (FORBIDDEN_MODEL); subagents are Sonnet, or Opus with a reason, by
# the protocol (.claude/commands/night-shift.md section 0 rule 5), and one spawned
# without a model inherits the orchestrator's, i.e. Opus.
# Each leg gets exactly one credential (`env -u` the other): when both are set
# the key silently wins, which is what made the subscription unreachable.
# The first leg's output is kept as <date>.claude-api.json/.err; the resumed
# leg's is <date>.claude.json/.err, which everything downstream reads.
#
# Status words. Exit 0: OK, DRY_OK, HELD, DEFERRED, OVERLAP, NOTHING_NEW.
# Exit 1: NO_GUARD, FORBIDDEN_MODEL, NO_API_KEY, NO_CLONE, CLONE_DIRTY, PULL_FAILED, NO_SETTINGS,
# PREFLIGHT_FAILED, CLAUDE_MISSING, CLAUDE_TIMEOUT, CLAUDE_FAILED,
# POST_FETCH_FAILED, PUSH_REJECTED, RUN_LEFT_DIRTY, RUN_SALVAGED, NO_REPORT,
# UNGATED_MERGE, UNGATED_HOLD, DRY_CHANGED, DRY_NO_REPORT, INTERNAL_ERROR. A real run that claimed the nightly
# ticker and ends in any status but OK re-raises it (ticker_reraised in
# last_run.json), so a failed night never swallows a RED nightly.
#
# The merge is bound to the gate (closeout C1). Every MERGE the orchestrator acts
# on is recorded by `night_shift_gate.py --record` in $NIGHT_SHIFT_VERDICTS
# (exported to claude; default ~/.local/state/jugglebot/verdicts). Once claude has
# exited, and BEFORE the app deploy (its reverts are this runner's commits), the
# runner fetches, captures origin/main and runs tools/night_shift_landed.py, as it
# stood at the night's start, over head_before..that capture: every non-exempt
# commit must be covered by a MERGE record. A failure (or a check that cannot run)
# is UNGATED_MERGE: the commits, authors and paths go in the detail, the report's
# Summary and Cost get a flag (the report and cost commit still land),
# $REPORTS/ungated.hold is written and the next start is HELD until the owner
# deletes it. Nothing is reverted. On another failure status the same evidence is
# appended to its detail and the hold file is written too. last_run.json carries
# `landed: gated=k exempt=m ungated=n` and `origin_after_claude`.
#   To clear: review the commits listed (last_run.json, landed-<date>.txt, the hold
#   file itself), then delete $REPORTS/ungated.hold. Until then every night ends
#   UNGATED_HOLD with a stub report saying so.
#   Flagged by design: a non-exempt push to main by the owner or a day Session
#   inside the night window has no MERGE record (a debrief merge will, once it
#   records). Merges into the map-app repo are outside this check: it reads
#   Jugglebot's main only.
#   Bootstrap and rollback: when head_before has no tools/night_shift_landed.py,
#   the check is skipped (`landed: skipped (no checker at head_before)`) and the
#   night goes on; a checker that exists there but cannot run is fail-closed.
#
# A night that stops early (2026-11-09: `claude -p` gives background tasks 10
# minutes after the orchestrator's last turn, then kills them and exits 0, so a
# unit outliving the orchestrator's other work lost the unit and the report).
# Three layers, each for what the one before cannot catch:
#   1. CLAUDE_CODE_PRINT_BG_WAIT_CEILING_MS=0 on every leg removes that limit;
#      the leg's `timeout` is still the bound. Probed 2026-11-09: at 20 s the
#      subagent is killed with exit 0, at 0 it finishes and wakes the orchestrator.
#   2. A real run that exits 0 with no report, off main, or dirty gets one
#      resume for it (`--resume`, same credential; on top of the credit-out
#      resume if that ran, all inside the one time budget) to finish. total_cost_usd on a resumed
#      session is cumulative (same probe), so no cost is added up here.
#   3. Whatever still stands is saved: uncommitted work is committed to the
#      Item's branch (a new night-shift/<date>-salvage-* one if it was on main),
#      pushed, and the clone goes back to main so the next night can run. main
#      is never written with it. Status RUN_SALVAGED; RUN_LEFT_DIRTY now means
#      the save itself failed. A non-OK night that reached Claude and has no
#      report gets a stub one (tools/night_shift_stub_report.py), provided the
#      clone is back on a clean main with nothing unpushed.
#
# The map app: the night works on its own clone of the app ($NIGHT_SHIFT_APP_CLONE,
# never ~/map-live), prepared here (a failure there is not fatal, the prompt says
# `--app-clone none` and the app's Items are skipped). After claude has exited, on
# any outcome, a real run calls tools/night_shift_deploy.py, which puts the app's
# main live and takes back what breaks it (owner, 2026-11-07; the tool's docstring
# has the decision). Never during a Sitting: SKIPPED_LIVE_SESSION, and the next
# night picks it up. Its result is one line at the end of the report (before the
# cost line) and `deploy_status` / `deploy_line` in last_run.json.
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
# NIGHT_SHIFT_MODEL (opus), NIGHT_SHIFT_FALLBACK_MODEL (opus; the subscription
# leg; neither may name Fable), NIGHT_SHIFT_TIMEOUT_S (14400; both legs together), NIGHT_SHIFT_DEADLINE
# (08:00), NIGHT_SHIFT_DEFER_BUDGET_S (7200), NIGHT_SHIFT_DEFER_POLL_S (120),
# NIGHT_SHIFT_WAIT_NIGHTLY_S (3600), NIGHT_SHIFT_WAIT_NIGHTLY_POLL_S (60),
# NIGHT_SHIFT_IGNORE_SESSION (0; hand runs only), NIGHT_SHIFT_SKIP_ENV (0; tests),
# NIGHT_SHIFT_PYTHON (the venv python if present, else python3), NIGHT_SHIFT_CLAUDE
# (the claude binary; default: PATH, then the newest ~/.nvm install),
# NIGHT_SHIFT_APP (1; 0 = no app clone, no deploy), NIGHT_SHIFT_APP_DEPLOY (1; 0 =
# the clone but no deploy), NIGHT_SHIFT_APP_CLONE, NIGHT_SHIFT_APP_ORIGIN,
# NIGHT_SHIFT_APP_LIVE, NIGHT_SHIFT_DATA_LIVE, NIGHT_SHIFT_DEPLOY_TOOL (tests).
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
MODEL="${NIGHT_SHIFT_MODEL:-opus}"
FALLBACK_MODEL="${NIGHT_SHIFT_FALLBACK_MODEL:-opus}"
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
APP_ENABLED="${NIGHT_SHIFT_APP:-1}"
APP_DEPLOY="${NIGHT_SHIFT_APP_DEPLOY:-1}"
APP_CLONE="${NIGHT_SHIFT_APP_CLONE:-$HOME/night-shift/jugglebot-map}"
APP_ORIGIN="${NIGHT_SHIFT_APP_ORIGIN:-https://github.com/Project-DeepBlue-Juggling/jugglebot-map.git}"
APP_LIVE="${NIGHT_SHIFT_APP_LIVE:-$HOME/map-live/jugglebot-map}"
DATA_LIVE="${NIGHT_SHIFT_DATA_LIVE:-$HOME/map-live/Jugglebot}"
DEPLOY_TOOL="${NIGHT_SHIFT_DEPLOY_TOOL:-$SCRIPT_DIR/night_shift_deploy.py}"
DEPLOY_TIMEOUT_S="${NIGHT_SHIFT_DEPLOY_TIMEOUT_S:-2400}"
APP_URL="${NIGHT_SHIFT_APP_URL:-http://127.0.0.1:8765}"
UNGATED_HOLD="$REPORTS/ungated.hold"
# Where the gate records its verdicts (--record) and the landed check reads them.
NIGHT_SHIFT_VERDICTS="${NIGHT_SHIFT_VERDICTS:-$HOME/.local/state/jugglebot/verdicts}"
export NIGHT_SHIFT_VERDICTS

DATE="$(date +%F)"
STARTED="$(date -Iseconds)"
START_S=$SECONDS
DRY=""
[[ "$DRY_RUN" == "1" ]] && DRY=1

# Summary fields: unset means "not reached" and becomes null in last_run.json.
unset NS_COST NS_TURNS NS_DENIALS NS_SPAWNED NS_CLAUDE_EXIT NS_HEAD_BEFORE \
  NS_HEAD_AFTER NS_BRANCH_AFTER NS_UNPUSHED NS_REPORT NS_CANDIDATES \
  NS_NIGHTLY_LINE NS_PREFLIGHT NS_RESULT NS_MAIN_COMMITS NS_TICKER_RERAISED NS_COST_RECORDED \
  NS_AUTH NS_COST_API NS_APP_CLONE_STATE NS_APP_NIGHT_START NS_DEPLOY_STATUS NS_DEPLOY_LINE \
  NS_APP_HEALTHY_BEFORE NS_RESUMED NS_SALVAGED NS_STUB_REPORT \
  NS_ORIGIN_AFTER_CLAUDE NS_LANDED NS_LANDED_UNGATED
STATUS_WRITTEN=0
TICKER_CLAIMED=0
CLAUDE_RAN=0

# ── Exit paths ────────────────────────────────────────────────────────────────
one_line() { tr '\n\r\t' '   ' <<<"$1" | sed 's/  */ /g; s/ *$//'; }

# The orchestrator cannot see its own total_cost_usd (the JSON result arrives
# after it exits), so the runner writes the figure into the night's report:
# one line at the end of `## Cost` (the last section). A dry run's report is a
# plain file; a real run's is on main, so the line is its own docs commit,
# pushed, and dropped again if main moved meanwhile (nothing may be left on the
# clone's main unpushed). NS_COST_RECORDED says what happened.
cost_line() {
  case "${NS_AUTH:-api}" in
    subscription)
      printf 'total cost: USD %.2f notional, on the Claude subscription (%s; not billed) (runner, total_cost_usd)\n' \
        "$NS_COST" "$FALLBACK_MODEL" ;;
    api-then-subscription)
      printf 'total API cost: USD %.2f until the credit balance ran out, then USD %.2f notional on the Claude subscription (%s; not billed) (runner, total_cost_usd)\n' \
        "${NS_COST_API:-0}" "$NS_COST" "$FALLBACK_MODEL" ;;
    *) printf 'total API cost: USD %.2f (runner, total_cost_usd)\n' "$NS_COST" ;;
  esac
}
record_cost_dry() {
  [[ -n "${NS_COST:-}" && -s "$NS_REPORT" ]] || { export NS_COST_RECORDED=no-report; return 0; }
  cost_line >>"$NS_REPORT" && export NS_COST_RECORDED=file || export NS_COST_RECORDED=write-failed
}
# The deploy's line goes before the cost line, so the cost stays last. A deploy that
# ended badly also says so in the Summary line, where the owner reads first.
append_deploy_line() {
  [[ -n "${NS_DEPLOY_LINE:-}" ]] || return 0
  printf '%s\n' "$NS_DEPLOY_LINE" >>"$NS_REPORT" || return 1
  case "${NS_DEPLOY_STATUS:-}" in
    NOOP|DEPLOYED) ;;
    *) sed -i "0,/^\*\*Summary\.\*\*.*/s//& **App deploy: ${NS_DEPLOY_STATUS:-unknown}, see the last lines of Cost.**/" "$NS_REPORT" ;;
  esac
}
# An ungated landing (closeout C1) is flagged in the report the owner reads first:
# a line before the cost line, and the Summary.
append_landed_flag() {
  [[ -n "${NS_LANDED_UNGATED:-}" ]] || return 0
  printf 'landed check: UNGATED_MERGE; %s; %s. To clear: review those commits on main, then delete %s. A non-exempt push by the owner or a day Session during the night is flagged by design; map-app merges are outside this check (runner, tools/night_shift_landed.py)\n' \
    "${NS_LANDED:-}" "$NS_LANDED_UNGATED" "$UNGATED_HOLD" >>"$NS_REPORT" || return 1
  sed -i "0,/^\*\*Summary\.\*\*.*/s//& **UNGATED_MERGE: a commit reached main with no MERGE record, see the last lines of Cost.**/" "$NS_REPORT"
}
record_cost_main() {
  if [[ -z "${NS_COST:-}" && -z "${NS_DEPLOY_LINE:-}" && -z "${NS_LANDED_UNGATED:-}" ]]; then
    export NS_COST_RECORDED=no-cost; return 0
  fi
  if [[ ! -s "$NS_REPORT" ]]; then export NS_COST_RECORDED=no-report; return 0; fi
  local rel="docs/night-shift/$DATE.md"
  if append_deploy_line \
     && append_landed_flag \
     && { [[ -z "${NS_COST:-}" ]] || cost_line >>"$NS_REPORT"; } \
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
    "auth": e("NS_AUTH"),
    "resumed": e("NS_RESUMED"),
    "salvaged": e("NS_SALVAGED"),
    "stub_report": e("NS_STUB_REPORT"),
    "cost_api_usd": num("NS_COST_API", float),
    "app_clone": e("NS_APP_CLONE_STATE"),
    "app_night_start": e("NS_APP_NIGHT_START"),
    "app_healthy_before": e("NS_APP_HEALTHY_BEFORE"),
    "deploy_status": e("NS_DEPLOY_STATUS"),
    "deploy_line": e("NS_DEPLOY_LINE"),
    "origin_after_claude": e("NS_ORIGIN_AFTER_CLAUDE"),
    "landed": e("NS_LANDED"),
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

# A non-OK night that reached Claude and has no report still gets a page in the
# usual place. Needs the clone on main, clean and with nothing unpushed: the
# reset below may only ever drop this one commit.
write_stub_report() {   # write_stub_report STATUS DETAIL
  local status="$1" detail="$2" rel="docs/night-shift/$DATE.md"
  export NS_STUB_REPORT=skipped
  [[ -z "$DRY" && ( "$CLAUDE_RAN" == 1 || "$status" == UNGATED_HOLD ) && "$status" != OK && ! -s "$CLONE/$rel" ]] \
    || { unset NS_STUB_REPORT; return 0; }
  [[ "$(git -C "$CLONE" symbolic-ref -q --short HEAD 2>/dev/null)" == main ]] || return 0
  [[ -z "$(git -C "$CLONE" status --porcelain 2>/dev/null)" ]] || return 0
  timeout --kill-after=10 60 git -C "$CLONE" fetch -q origin 2>/dev/null 8>&- || return 0
  [[ -z "$(git -C "$CLONE" log --format=%H origin/main..main 2>/dev/null)" ]] || return 0
  git -C "$CLONE" merge -q --ff-only origin/main 2>/dev/null || return 0
  [[ ! -s "$CLONE/$rel" ]] || { unset NS_STUB_REPORT; return 0; }
  local cost=""
  [[ -n "${NS_COST:-}" ]] && cost="$(cost_line)"
  if ! "$PYTHON" "$CLONE/tools/night_shift_stub_report.py" --date "$DATE" --status "$status" \
       --clone "$CLONE" --out "$CLONE/$rel" --started "$STARTED" --finished "$(date -Iseconds)" \
       --detail "$detail" --nightly "${NS_NIGHTLY_LINE:-}" --head-before "${NS_HEAD_BEFORE:-}" \
       --salvaged "${NS_SALVAGED:-}" --deploy-line "${NS_DEPLOY_LINE:-}" --cost-line "$cost" \
       2>>"$REPORTS/$DATE.stub.err"; then
    rm -f "$CLONE/$rel"
    export NS_STUB_REPORT=write-failed
    return 0
  fi
  if git -C "$CLONE" add -- "$rel" \
     && git -C "$CLONE" commit -q -m "docs(night-shift): stub report $DATE ($status)" -- "$rel" \
     && timeout --kill-after=10 120 git -C "$CLONE" push -q origin HEAD:main 2>>"$REPORTS/$DATE.stub.err" 8>&-; then
    export NS_STUB_REPORT=pushed
  else
    export NS_STUB_REPORT=rejected
    git -C "$CLONE" fetch -q origin 2>/dev/null 8>&-
    git -C "$CLONE" reset -q --hard origin/main 2>/dev/null
  fi
}

finish() {         # finish STATUS DETAIL: the only way out after argument parsing
  local status="$1" detail="${2:-}"
  trap - EXIT
  # An ungated landing outlives any status: the hold file stops the next nights, and a
  # failure status carries the evidence too (UNGATED_MERGE's own detail already does).
  if [[ -n "${NS_LANDED_UNGATED:-}" ]]; then
    printf 'UNGATED_MERGE %s: %s\n%s\n' "$DATE" "${NS_LANDED:-}" "$NS_LANDED_UNGATED" \
      >"$UNGATED_HOLD" 2>/dev/null || detail="$detail (ungated.hold write FAILED)"
    [[ "$status" == UNGATED_MERGE ]] || detail="$detail; UNGATED_MERGE ${NS_LANDED:-} $NS_LANDED_UNGATED"
  fi
  write_stub_report "$status" "$detail"
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
  [[ -n "${NS_DEPLOY_STATUS:-}" ]] && detail="$detail; deploy $NS_DEPLOY_STATUS"
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
  trap '' TERM INT HUP   # a second signal must not interrupt the save
  if [[ -n "$CHILD_PID" ]]; then
    kill -TERM "$CHILD_PID" 2>/dev/null
    wait "$CHILD_PID" 2>/dev/null
    export_posthoc_git
    salvage_clone
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

# Puts a clone Claude left off main or dirty back on main, its work saved on a
# branch and pushed, so one bad night cannot also cost the next (CLONE_DIRTY).
# Every failure just returns, losing nothing and leaving main alone, though a
# failed commit can leave the clone on the new salvage branch with its changes
# staged. The status then says RUN_LEFT_DIRTY, the old by-hand path. Never on a dry run, which must
# prove it changed nothing (DRY_CHANGED). `git add -A` honours .gitignore, so
# temp/ and the installed settings file stay out.
salvage_clone() {
  [[ -z "$DRY" && "$CLAUDE_RAN" == 1 ]] || return 0
  local branch porcelain target sha pushed=pushed
  branch="$(git -C "$CLONE" symbolic-ref -q --short HEAD 2>/dev/null || echo DETACHED)"
  porcelain="$(git -C "$CLONE" status --porcelain 2>/dev/null)" || return 0
  [[ "$branch" != main || -n "$porcelain" ]] || return 0
  target="$branch"
  if [[ "$branch" == main || "$branch" == DETACHED ]]; then
    target="night-shift/$DATE-salvage-$(date +%H%M%S)"
    git -C "$CLONE" checkout -q -b "$target" 2>/dev/null || return 0
  fi
  if [[ -n "$porcelain" ]]; then
    git -C "$CLONE" add -A 2>/dev/null \
      && git -C "$CLONE" commit -q -m "wip(night-shift): salvage $DATE from $branch" \
           -m "Saved by tools/night_shift.sh when the run ended with this work uncommitted. Not reviewed and not gated: a Session must check it before any merge." \
           2>/dev/null \
      || return 0
  fi
  sha="$(git -C "$CLONE" rev-parse --short HEAD 2>/dev/null)"
  timeout --kill-after=10 120 git -C "$CLONE" push -q -u origin "$target" 2>>"$REPORTS/$DATE.salvage.err" 8>&- \
    || pushed=NOT-PUSHED
  git -C "$CLONE" checkout -q main 2>/dev/null || return 0
  export NS_SALVAGED="$target $sha $pushed"
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

# ── 2b. No Fable (owner, 2026-11-02): before the live-session wait ──────────
for m in "$MODEL" "$FALLBACK_MODEL"; do
  [[ "${m,,}" == *fable* ]] \
    && finish FORBIDDEN_MODEL "model '$m' names Fable; the Night Shift never invokes Fable (NIGHT_SHIFT_MODEL / NIGHT_SHIFT_FALLBACK_MODEL)"
done

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

# ── 4. A credential (before anything is cloned or fetched) ────────────────────
have_credential() { [[ -n "${ANTHROPIC_API_KEY:-}" || -n "${CLAUDE_CODE_OAUTH_TOKEN:-}" ]]; }
have_credential \
  || finish NO_API_KEY "neither ANTHROPIC_API_KEY nor CLAUDE_CODE_OAUTH_TOKEN set (EnvironmentFile ~/.config/jugglebot/night-shift.env)"
[[ -n "${ANTHROPIC_API_KEY:-}" ]] && export ANTHROPIC_API_KEY
[[ -n "${CLAUDE_CODE_OAUTH_TOKEN:-}" ]] && export CLAUDE_CODE_OAUTH_TOKEN

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

# ── 6a. An earlier ungated landing holds every night, loudly (closeout C1) ─────
# After the clone is ready, so the stub report (write_stub_report) can say so on
# main each night; the owner's own night-shift.hold (guard 2) stays quiet.
if [[ -e "$UNGATED_HOLD" ]]; then
  finish UNGATED_HOLD "$UNGATED_HOLD from $(one_line "$(head -n12 "$UNGATED_HOLD" 2>/dev/null)"); to clear: review those commits on main, then delete $UNGATED_HOLD; claude NOT run"
fi

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
have_credential || finish NO_API_KEY "credentials lost after sourcing the environment"

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

# The app's own clone (node is on PATH from here). Never fatal: the board has plenty
# else to work, so a failure only takes the app's Items off tonight's list.
prepare_app_clone() {
  [[ -d "$APP_CLONE/.git" ]] || git clone -q --branch main "$APP_ORIGIN" "$APP_CLONE" 2>/dev/null \
    || { NS_APP_CLONE_STATE="clone failed"; return 1; }
  local b p
  b="$(git -C "$APP_CLONE" symbolic-ref -q --short HEAD 2>/dev/null || echo DETACHED)"
  p="$(git -C "$APP_CLONE" status --porcelain 2>&1)"
  if [[ "$b" != main || -n "$p" ]]; then
    NS_APP_CLONE_STATE="dirty: branch=$b $(head -n3 <<<"$p" | tr '\n' ';')"; return 1
  fi
  git -C "$APP_CLONE" fetch -q origin 2>/dev/null && git -C "$APP_CLONE" merge -q --ff-only origin/main 2>/dev/null \
    || { NS_APP_CLONE_STATE="pull failed"; return 1; }
  NS_APP_NIGHT_START="$(git -C "$APP_CLONE" rev-parse origin/main)"
  local lock="$APP_CLONE/frontend/package-lock.json" stamp="$APP_CLONE/frontend/node_modules/.night-shift-lock"
  local want; want="$(sha256sum "$lock" 2>/dev/null | cut -d' ' -f1)"
  if [[ -n "$want" && "$(cat "$stamp" 2>/dev/null)" != "$want" ]]; then
    (cd "$APP_CLONE/frontend" && timeout --kill-after=30 900 npm ci >>"$REPORTS/app-npm-$DATE.log" 2>&1) \
      || { NS_APP_CLONE_STATE="npm ci failed"; return 1; }
    printf '%s\n' "$want" >"$stamp"
  fi
  NS_APP_CLONE_STATE=ready
}
APP_CLONE_ARG=none
if [[ "$APP_ENABLED" != 1 ]]; then
  NS_APP_CLONE_STATE=off
elif [[ -n "$DRY" ]]; then
  # A dry run changes nothing: it reads the clone if there is one.
  NS_APP_CLONE_STATE=dry
  [[ -d "$APP_CLONE/.git" ]] && APP_CLONE_ARG="$APP_CLONE"
else
  prepare_app_clone 8>&- && APP_CLONE_ARG="$APP_CLONE"
  # The deploy may only revert what broke the app tonight, so it needs to know how the
  # app was doing before tonight. A service the owner stopped is not the night's doing.
  if curl -fsS -m 10 "$APP_URL/api/v1/health" >/dev/null 2>&1; then
    NS_APP_HEALTHY_BEFORE=1
  else
    NS_APP_HEALTHY_BEFORE=0
  fi
fi
export NS_APP_CLONE_STATE NS_APP_NIGHT_START NS_APP_HEALTHY_BEFORE

if [[ -n "$DRY" ]]; then
  export NS_REPORT="$REPORTS/dry-$DATE.md"
  rm -f "$NS_REPORT"   # a stale report must not pass for tonight's
else
  export NS_REPORT="$CLONE/docs/night-shift/$DATE.md"
fi
PROMPT="/night-shift $DATE --candidates $CANDIDATES --reports $REPORTS --nightly \"$NS_NIGHTLY_LINE\" --nightly-report $NIGHTLY_COPY --deadline $DEADLINE --app-clone $APP_CLONE_ARG${DRY:+ --dry-run}"
CLAUDE_JSON="$REPORTS/$DATE.claude.json"
CLAUDE_ERR="$REPORTS/$DATE.claude.err"
cd "$CLONE" || finish INTERNAL_ERROR "cannot cd $CLONE"
# timeout puts claude in its own process group and signals the whole group, so
# agents and suite runs under it die with it. fd 8 (the run lock) is closed for
# the child: a straggler must not keep the next night locked out. Started in the
# background and waited on so on_signal can pass a signal on (a foreground
# child would delay our trap until it exited by itself).
claude_start=$SECONDS
CLAUDE_RAN=1
# The budget fuse is opt-in: the owner wants the first nights unlimited and
# will set NIGHT_SHIFT_MAX_USD once the cost line in the reports shows a norm.
# It caps real money only, so it applies to the API leg alone.
budget_args=()
if [[ -n "$MAX_USD" && "$MAX_USD" != 0 ]]; then budget_args=(--max-budget-usd "$MAX_USD"); fi

# run_leg AUTH MODEL TIMEOUT_S [claude args...]: one claude -p call on exactly
# one credential (api | subscription), output to $CLAUDE_JSON / $CLAUDE_ERR,
# exit code in claude_rc. Waited on in the background so on_signal can pass a
# signal on (see above).
run_leg() {
  local auth="$1" model="$2" leg_timeout="$3"; shift 3
  local -a envx
  if [[ "$auth" == api ]]; then envx=(env -u CLAUDE_CODE_OAUTH_TOKEN); else envx=(env -u ANTHROPIC_API_KEY); fi
  envx+=(CLAUDE_CODE_PRINT_BG_WAIT_CEILING_MS=0)   # see "A night that stops early"
  timeout --kill-after=120 "$leg_timeout" "${envx[@]}" "$CLAUDE_BIN" "$@" \
    --model "$model" \
    --permission-mode acceptEdits --permission-prompts none \
    --output-format json </dev/null >"$CLAUDE_JSON" 2>"$CLAUDE_ERR" 8>&- &
  CHILD_PID=$!
  wait "$CHILD_PID"
  claude_rc=$?
  CHILD_PID=""
}

# credit_out FILE: prints the session id and exits 0 when FILE is a result that
# ended on an exhausted API credit balance; exits 1 otherwise.
credit_out() {
  "$PYTHON" - "$1" <<'PY'
import json, sys
try:
    data = json.load(open(sys.argv[1]))
except (OSError, ValueError):
    sys.exit(1)
if isinstance(data, list):
    data = next((m for m in reversed(data)
                 if isinstance(m, dict) and m.get("type") == "result"), None)
if not isinstance(data, dict) or not data.get("is_error"):
    sys.exit(1)
if "credit balance is too low" not in str(data.get("result") or "").lower():
    sys.exit(1)
print(data.get("session_id") or "")
sys.exit(0)
PY
}

leg_timed_out() {   # leg_timed_out RC LEG_START LEG_TIMEOUT
  (( $1 == 124 )) || { (( $1 == 137 )) && (( SECONDS - $2 >= $3 )); }
}

if [[ -n "${ANTHROPIC_API_KEY:-}" ]]; then
  export NS_AUTH=api
  run_leg api "$MODEL" "$TIMEOUT_S" -p "$PROMPT" "${budget_args[@]}"
  if ! leg_timed_out "$claude_rc" "$claude_start" "$TIMEOUT_S" \
     && [[ -n "${CLAUDE_CODE_OAUTH_TOKEN:-}" ]] \
     && sid="$(credit_out "$CLAUDE_JSON")"; then
    # The API ran dry mid-night: keep its record, resume the session on the
    # subscription with what is left of the time budget.
    NS_COST_API="$("$PYTHON" -c 'import json,sys
try: print(float(json.load(open(sys.argv[1])).get("total_cost_usd") or 0))
except Exception: print(0)' "$CLAUDE_JSON")"
    export NS_COST_API NS_AUTH=api-then-subscription
    mv -f "$CLAUDE_JSON" "$REPORTS/$DATE.claude-api.json"
    mv -f "$CLAUDE_ERR" "$REPORTS/$DATE.claude-api.err"
    left=$(( TIMEOUT_S - (SECONDS - claude_start) ))
    (( left > 60 )) || left=60
    resume_prompt="The API credit balance ran out, so the Night Shift runner has resumed this session on the Claude subscription (model $FALLBACK_MODEL). Continue the /night-shift protocol from exactly where you stopped, with the same arguments and deadline: first run git status and git branch --show-current in the clone and git log origin/main..HEAD, finish or hold the Item that was in flight (its units may have been cut off mid-way: check their output before trusting it), then carry on to the report. Under ## Cost, add the line: auth: API until the credit balance ran out, then the Claude subscription ($FALLBACK_MODEL)."
    resume_args=(-p "$resume_prompt")
    [[ -n "$sid" ]] && resume_args+=(--resume "$sid")
    leg2_start=$SECONDS
    run_leg subscription "$FALLBACK_MODEL" "$left" "${resume_args[@]}"
    if leg_timed_out "$claude_rc" "$leg2_start" "$left"; then claude_rc=124; fi
  fi
else
  export NS_AUTH=subscription
  run_leg subscription "$FALLBACK_MODEL" "$TIMEOUT_S" -p "$PROMPT"
fi

# Why a real run that says it finished has not (empty = it has).
run_incomplete() {
  local branch
  branch="$(git -C "$CLONE" symbolic-ref -q --short HEAD 2>/dev/null || echo DETACHED)"
  [[ -s "$CLONE/docs/night-shift/$DATE.md" ]] || { echo "no report at docs/night-shift/$DATE.md"; return 0; }
  [[ "$branch" == main ]] || { echo "the clone is on $branch, not main"; return 0; }
  [[ -z "$(git -C "$CLONE" status --porcelain 2>/dev/null)" ]] || echo "the clone has uncommitted work"
}
session_of() {   # session_of FILE: the result's session id, or nothing
  "$PYTHON" -c 'import json,sys
d=json.load(open(sys.argv[1]))
if isinstance(d,list): d=next(m for m in reversed(d) if isinstance(m,dict) and m.get("type")=="result")
if not d.get("is_error"): print(d.get("session_id") or "")' "$1" 2>/dev/null
}
RESUME_MIN_S=300
if [[ -z "$DRY" && "$claude_rc" == 0 ]]; then
  why="$(run_incomplete)"
  left=$(( TIMEOUT_S - (SECONDS - claude_start) ))
  sid="$(session_of "$CLAUDE_JSON")"
  if [[ -n "$why" && -n "$sid" ]] && (( left >= RESUME_MIN_S )); then
    export NS_RESUMED="$why"
    mv -f "$CLAUDE_JSON" "$REPORTS/$DATE.claude-first.json"
    mv -f "$CLAUDE_ERR" "$REPORTS/$DATE.claude-first.err"
    resume_prompt="Your session ended before the Night Shift run was complete: $why. The runner has resumed it to finish. A unit that was still running may have been stopped, so check its output and the tree before trusting either. Continue the /night-shift protocol from where you stopped, with the same arguments and deadline: first run git status and git branch --show-current in the clone and git log origin/main..HEAD, finish or hold the Item in flight, then write the report (section 3) and end with the clone on main and clean."
    leg_start=$SECONDS
    if [[ "$NS_AUTH" == api ]]; then
      run_leg api "$MODEL" "$left" -p "$resume_prompt" --resume "$sid" "${budget_args[@]}"
    else
      run_leg subscription "$FALLBACK_MODEL" "$left" -p "$resume_prompt" --resume "$sid"
    fi
    if leg_timed_out "$claude_rc" "$leg_start" "$left"; then claude_rc=124; fi
  fi
fi
export NS_CLAUDE_EXIT="$claude_rc"
export_posthoc_git
salvage_clone
export_posthoc_git

# ── 10a. The landed check: what reached origin/main while claude ran (C1) ─────
# Captured here, BEFORE the deploy, whose reverts and reopens are this runner's own
# commits. The checker is the copy at the night's start, so a push tonight cannot
# rewrite it; any failure to run it counts as ungated (fail closed).
landed_step() {
  [[ -z "$DRY" ]] || return 0
  timeout --kill-after=10 120 git -C "$CLONE" fetch -q origin 2>/dev/null 8>&-
  NS_ORIGIN_AFTER_CLAUDE="$(git -C "$CLONE" rev-parse -q --verify refs/remotes/origin/main 2>/dev/null)"
  export NS_ORIGIN_AFTER_CLAUDE
  local checker="$REPORTS/landed-$DATE.py" out rc
  # Bootstrap or rollback: no checker in the night's starting commit, so nothing to
  # bind the night to. Recorded, never a hold, never a copy from elsewhere.
  if ! git -C "$CLONE" cat-file -e "$NS_HEAD_BEFORE:tools/night_shift_landed.py" 2>/dev/null; then
    export NS_LANDED="landed: skipped (no checker at head_before)" NS_LANDED_UNGATED=""
    return 0
  fi
  git -C "$CLONE" show "$NS_HEAD_BEFORE:tools/night_shift_landed.py" >"$checker" 2>/dev/null \
    || rm -f "$checker"
  out="$(timeout --kill-after=10 300 "$PYTHON" -I "$checker" --repo "$CLONE" --from "$NS_HEAD_BEFORE" \
         --to "${NS_ORIGIN_AFTER_CLAUDE:-refs/remotes/origin/main}" --verdicts "$NIGHT_SHIFT_VERDICTS" 2>&1 8>&-)"
  rc=$?
  printf '%s\n' "$out" >"$REPORTS/landed-$DATE.txt" 2>/dev/null
  NS_LANDED="$(head -n1 <<<"$out")"
  case "$rc" in
    0) NS_LANDED_UNGATED="" ;;
    1) NS_LANDED_UNGATED="$(tail -n +2 <<<"$out" | head -n10 | tr '\n' ';')"
       NS_LANDED_UNGATED="${NS_LANDED_UNGATED%;}"
       NS_LANDED_UNGATED="${NS_LANDED_UNGATED:-ungated commits, none named (rc=1)}" ;;
    *) NS_LANDED_UNGATED="the landed check could not run (rc=$rc): $(one_line "$(head -n3 <<<"$out")")" ;;
  esac
  export NS_LANDED NS_LANDED_UNGATED
}
landed_step

# ── 10b. The app deploy: the last step of the night, whatever claude's outcome ──
# Before the outcome is judged, because a night that dies after merging still has
# commits to put live or take back. A Sitting holding the box outranks all of it:
# a frontend build is minutes of CPU on the box running the control loops.
deploy_step() {
  [[ -z "$DRY" && "$APP_ENABLED" == 1 && "$APP_DEPLOY" == 1 ]] || return 0
  if [[ "$IGNORE_SESSION" != 1 ]] && declare -F live_session >/dev/null && live_session; then
    export NS_DEPLOY_STATUS=SKIPPED_LIVE_SESSION
    export NS_DEPLOY_LINE="app deploy: SKIPPED_LIVE_SESSION; a Sitting holds the box, the next night picks it up"
    return 0
  fi
  local result="$REPORTS/deploy-$DATE.json"
  rm -f "$result"
  timeout --kill-after=60 "$DEPLOY_TIMEOUT_S" "$PYTHON" "$DEPLOY_TOOL" --date "$DATE" --result "$result" \
    --app-night-start "${NS_APP_NIGHT_START:-}" --main-night-start "$NS_HEAD_BEFORE" \
    --pre-night-healthy "${NS_APP_HEALTHY_BEFORE:-unknown}" --base-url "$APP_URL" \
    --app-live "$APP_LIVE" --data-live "$DATA_LIVE" --app-clone "$APP_CLONE" --main-clone "$CLONE" \
    >>"$REPORTS/deploy-$DATE.log" 2>&1 8>&-
  local rc=$? parsed
  parsed="$("$PYTHON" -c 'import json,sys
d=json.load(open(sys.argv[1]));print(d["status"]);print(str(d.get("line","")).replace("\n"," ")[:900])' "$result" 2>/dev/null)"
  if [[ -z "$parsed" ]]; then
    export NS_DEPLOY_STATUS=FAILED_NEEDS_SESSION
    export NS_DEPLOY_LINE="app deploy: FAILED_NEEDS_SESSION; the deploy tool wrote no result (rc=$rc), see deploy-$DATE.log"
    return 0
  fi
  NS_DEPLOY_STATUS="$(head -n1 <<<"$parsed")"
  NS_DEPLOY_LINE="$(tail -n1 <<<"$parsed")"
  export NS_DEPLOY_STATUS NS_DEPLOY_LINE
  # A revert or reopen was pushed from a worktree: the clone's main must follow, or
  # the cost commit is rejected and the clone starts tomorrow behind.
  case "$NS_DEPLOY_STATUS" in
    ROLLED_BACK*|DOWN|FAILED_NEEDS_SESSION)
      if [[ "$(git -C "$CLONE" symbolic-ref -q --short HEAD 2>/dev/null)" == main ]]; then
        git -C "$CLONE" fetch -q origin 2>/dev/null && git -C "$CLONE" merge -q --ff-only origin/main 2>/dev/null
      fi ;;
  esac
  export_posthoc_git
}
deploy_step

if leg_timed_out "$claude_rc" "$claude_start" "$TIMEOUT_S"; then
  finish CLAUDE_TIMEOUT "killed after ${TIMEOUT_S}s; unpushed: ${NS_UNPUSHED:-none}"
fi
err_tail="$(tail -n3 "$CLAUDE_ERR" 2>/dev/null)"

# Parse the result (stdlib only). Tab-separated key/value lines on stdout.
# Parsed before the exit code is judged, so a failure names its real cause
# (the result) rather than the first stderr line (on 2026-11-02 that was the
# connectors warning, not "Credit balance is too low").
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
(( claude_rc == 0 )) || finish CLAUDE_FAILED "rc=$claude_rc auth=$NS_AUTH ${NS_RESULT:-$err_tail}"
case "$parse_rc" in
  0) ;;
  3) finish CLAUDE_FAILED "is_error: auth=$NS_AUTH ${NS_RESULT:-}" ;;
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
  finish RUN_LEFT_DIRTY "the save failed; branch=$NS_BRANCH_AFTER; $run_porcelain"
fi
[[ -z "${NS_SALVAGED:-}" ]] || finish RUN_SALVAGED "saved on ${NS_SALVAGED}; ${NS_RESUMED:+resumed once ($NS_RESUMED); }${NS_RESULT:-}"
[[ -s "$NS_REPORT" ]] || finish NO_REPORT "claude exited 0 without a report; ${NS_RESUMED:+resumed once ($NS_RESUMED); }${NS_RESULT:-}"

# The baseline for tomorrow's "anything new?" is the board AFTER tonight's work
# (Items closed or edited tonight are not news), so snapshot the post-run clone
# rather than keep the pre-run candidates. A snapshot: its exit code is moot.
(cd "$CLONE" && "$PYTHON" "$CLONE/tools/night_shift_preflight.py" --repo "$CLONE" \
  --out "$REPORTS/candidates-last.json") >/dev/null 2>>"$REPORTS/preflight.err"
record_cost_main
[[ -z "${NS_LANDED_UNGATED:-}" ]] || finish UNGATED_MERGE "${NS_LANDED:-}; ${NS_LANDED_UNGATED}"
finish OK "cost ${NS_COST:-?} USD (${NS_COST_RECORDED:-?}), turns ${NS_TURNS:-?}, main ${NS_HEAD_BEFORE:0:8} -> ${NS_HEAD_AFTER:0:8}"
