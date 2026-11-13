#!/usr/bin/env bash
# tools/nightly_ticker.sh — a once-per-day claim on the nightly result (owner ask,
# 2026-09-08).
#
# Problem it solves: every Claude session on this box used to read
# temp/reports/nightly/status at start. With several sessions a day that is the
# same RED read and re-diagnosed N times — often a RED that was only one session's
# unfinished work-in-progress at 04:00 (the nightly runs against the WORKING TREE).
#
# Mechanism: the nightly RAISES the ticker right after it writes `status`. The
# FIRST session to run `check` afterwards CLAIMS it with an atomic rename(2) —
# exactly one claimant can win, the rest see ENOENT — surfaces the result once,
# and the ticker stays LOWERED for every later session that day, which prints one
# line and reads nothing else. A new run re-raises it.
#
#   tools/nightly_ticker.sh raise            # nightly_suite.sh only, after `status`
#   tools/nightly_ticker.sh check [--who L]  # session start; prints ONE line:
#     CLAIMED  <status line>     you are first today: read latest.md, surface it
#     LOWERED  <run-iso> handled <ack-iso> by <who>   skip; nothing to read
#     STALE    <run-iso> ...     last run > 2 days ago: the runner may have stopped
#     UNRAISED <status line>     status exists but no ticker: read it once (a
#                                pre-ticker runner, or `raise` failed)
#     NEVER    no nightly artefacts: never armed on this box, or temp/ was cleaned
#   Exit code is 0 for every verdict; only a usage error or a failed `raise` is non-zero.
#
# Files, all under temp/reports/nightly/ (gitignored, like `status`):
#   ticker.raised   "<run-iso> <status line>"                 written by `raise`
#   ticker.lowered  "<run-iso> handled <ack-iso> by <who>"    written by the claimant
# `raise` removes any old `ticker.lowered`, so each run starts a fresh cycle.
# NIGHTLY_REPORTS_DIR overrides the directory (tests).

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPORTS="${NIGHTLY_REPORTS_DIR:-$REPO_ROOT/temp/reports/nightly}"
RAISED="$REPORTS/ticker.raised"
LOWERED="$REPORTS/ticker.lowered"
STATUS="$REPORTS/status"
STALE_AFTER_S=$((2 * 24 * 3600))
# The auto-pull's status line (tools/systemd/jugglebot-pull.sh); PULL_STATUS_FILE overrides (tests).
PULL_STATUS="${PULL_STATUS_FILE:-$REPO_ROOT/temp/reports/pull/status}"
PULL_LOUD_AFTER_S=600

usage() { sed -n '2,30p' "${BASH_SOURCE[0]}" >&2; exit 2; }

raise() {
  [[ -s "$STATUS" ]] || { echo "nightly_ticker: refusing to raise — no status file at $STATUS" >&2; exit 1; }
  local line run_iso
  line="$(head -n1 "$STATUS")"
  run_iso="${line##* }"                      # status ends with the run's iso-date
  local tmp="$RAISED.tmp.$$"
  printf '%s %s\n' "$run_iso" "$line" >"$tmp"
  rm -f "$LOWERED"
  mv -f "$tmp" "$RAISED"                     # atomic: readers see old-or-new, never partial
  echo "nightly_ticker: RAISED for run $run_iso"
}

age_s() {                                    # seconds since an iso-date, or empty if unparsable
  local t; t="$(date -d "$1" +%s 2>/dev/null)" || return 0
  echo $(( $(date +%s) - t ))
}

# One extra line after the verdict when the auto-pull has been stalled (non-OK)
# for more than 10 minutes. Silent when OK, absent, or unparsable.
pull_line() {
  [[ -s "$PULL_STATUS" ]] || return 0
  local line word behind since age
  line="$(head -n1 "$PULL_STATUS")"
  word="$(cut -d' ' -f2 <<<"$line")"
  [[ -n "$word" && "$word" != "OK" ]] || return 0
  behind="$(sed -n 's/.* behind=\([^ ]*\).*/\1/p' <<<"$line")"
  since="$(sed -n 's/.* since=\([^ ]*\).*/\1/p' <<<"$line")"
  age="$(age_s "$since")"
  [[ -n "$age" ]] && (( age > PULL_LOUD_AFTER_S )) || return 0
  echo "PULL $word behind=${behind:-?} since=$since"
}

check() {
  check_verdict "$@"
  pull_line
}

check_verdict() {
  local who="${NIGHTLY_TICKER_WHO:-pid$$/ppid$PPID/$(whoami)}"
  while (( $# )); do
    case "$1" in
      --who) who="${2:?--who needs a label}"; shift 2 ;;
      *) usage ;;
    esac
  done
  local claim="$REPORTS/ticker.claim.$$"
  if mv -f "$RAISED" "$claim" 2>/dev/null; then       # the atomic claim; losers get ENOENT
    local run_iso line
    run_iso="$(cut -d' ' -f1 "$claim")"
    line="$(cut -d' ' -f2- "$claim")"
    printf '%s handled %s by %s\n' "$run_iso" "$(date -Iseconds)" "$who" >"$LOWERED"
    rm -f "$claim"
    echo "CLAIMED $line"
    return 0
  fi
  if [[ -f "$LOWERED" ]]; then
    local run_iso age
    run_iso="$(cut -d' ' -f1 "$LOWERED")"
    age="$(age_s "$run_iso")"
    if [[ -n "$age" ]] && (( age > STALE_AFTER_S )); then
      echo "STALE $(cat "$LOWERED") — last nightly ran >2 days ago; check the timer"
    else
      echo "LOWERED $(cat "$LOWERED")"
    fi
    return 0
  fi
  if compgen -G "$REPORTS/ticker.claim.*" >/dev/null; then
    echo "LOWERED claim in progress by another session"; return 0
  fi
  if [[ -s "$STATUS" ]]; then
    echo "UNRAISED $(head -n1 "$STATUS")"; return 0
  fi
  echo "NEVER no nightly artefacts under $REPORTS"
}

case "${1:-}" in
  raise) shift; raise "$@" ;;
  check) shift; check "$@" ;;
  *) usage ;;
esac
