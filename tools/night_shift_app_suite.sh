#!/usr/bin/env bash
# tools/night_shift_app_suite.sh: the map app's CI, run locally, as a gate log.
#
#   tools/night_shift_app_suite.sh --app ~/night-shift/jugglebot-map \
#       --mapkit ~/night-shift/Jugglebot --log D/app-<ID>.log
#
# The same steps as the app's .github/workflows/ci.yml: server tests, the name
# check, lint, typecheck, vitest, build, then the live contract check, all against
# the mapkit in --mapkit (whichever branch that clone has checked out). The log
# carries what tools/night_shift_gate.py reads: HEAD=, BRANCH= and MAPKIT_HEAD=
# first, then the output, then a last EXIT=. Every step runs even after a failure,
# so one log names everything that is wrong; EXIT is the first failing step's code.
#
# It does not install anything: tools/night_shift.sh runs `npm ci` on the clone
# whenever the lock file changes. Run it under tools/night_shift_nonet.sh.
set -uo pipefail

APP="" MAPKIT="" LOG=""
while (( $# )); do
  case "$1" in
    --app) APP="$2"; shift 2 ;;
    --mapkit) MAPKIT="$2"; shift 2 ;;
    --log) LOG="$2"; shift 2 ;;
    *) echo "usage: $0 --app DIR --mapkit DIR --log FILE" >&2; exit 2 ;;
  esac
done
[[ -n "$APP" && -n "$MAPKIT" && -n "$LOG" ]] || { echo "usage: $0 --app DIR --mapkit DIR --log FILE" >&2; exit 2; }

APP="$(cd "$APP" && pwd)" || exit 2
MAPKIT="$(cd "$MAPKIT" && pwd)" || exit 2
: >"$LOG" || exit 2
LOG="$(cd "$(dirname "$LOG")" && pwd)/$(basename "$LOG")"

{
  printf 'HEAD=%s\n' "$(git -C "$APP" rev-parse HEAD)"
  printf 'BRANCH=%s\n' "$(git -C "$APP" rev-parse --abbrev-ref HEAD)"
  printf 'MAPKIT_HEAD=%s\n' "$(git -C "$MAPKIT" rev-parse HEAD)"
} >>"$LOG"

first_rc=0
step() {   # step DIR COMMAND...: one CI step; output to the log, a line to stdout
  local dir="$1" start=$SECONDS rc; shift
  printf '\n### %s\n' "$*" >>"$LOG"
  (cd "$dir" && MAPKIT_SRC="$MAPKIT" "$@") >>"$LOG" 2>&1
  rc=$?
  echo "app_suite: rc=$rc $((SECONDS - start))s: $*"
  (( rc != 0 && first_rc == 0 )) && first_rc=$rc
}

step "$APP" python -m pytest -q
step "$APP" bash scripts/check_no_upstream_name.sh
step "$APP/frontend" npm run lint
step "$APP/frontend" npm run typecheck
step "$APP/frontend" npm run test -- --run
step "$APP/frontend" npm run build
step "$APP" bash scripts/contract_check.sh

printf 'EXIT=%s\n' "$first_rc" >>"$LOG"
exit "$first_rc"
