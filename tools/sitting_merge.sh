#!/usr/bin/env bash
# tools/sitting_merge.sh: land ONE Item's held branch on main after a Sitting passed it,
# through the merge gate, as the /sitting-debrief section 3 procedure (closeout C10).
#
#   tools/sitting_merge.sh prepare --branch night-shift/<d>-<ID> --item <ID> --work DIR
#   (the debrief's /audit then writes DIR/audit-<ID>.md for the tip prepare printed)
#   tools/sitting_merge.sh land    --branch night-shift/<d>-<ID> --item <ID> --work DIR
#
# Why: at the 2026-11-13 debrief BUG-80 reached main by hand, ungated, with an open
# behavioural audit finding (logbook/2026-11-13-sitting-debrief.md, Discussion). The prose
# procedure this replaces had nine hand-typed steps between "the suite passed" and "push";
# any one of them could merge a tree the gate never saw. This script is the one path from a
# held branch to main after a Sitting, and it pushes only the tip the gate recorded MERGE for.
#
# prepare: refuses a live robot session (tools/lib/live_session.sh) and a frozen-path branch
#   (gate condition (f), never waivable: such a branch is merged by hand in a day Session;
#   there is no flag). Fetches, records origin/main's sha as the base (DIR/base-<ID>), makes
#   its own worktree DIR/wt-<ID> on a local branch of the same name at the remote branch, and
#   merges the base in. A conflict confined to the generated pages (logbook/INDEX.md,
#   plans/*/INDEX.md, docs/project-map.md, docs/glossary.md, docs/runbook.md) and the map
#   file of the --item Item (found by its id through the base's mapkit; main is the authority
#   for map state) takes the base's copy and runs `python -m mapkit gen`; the Item's file is
#   announced `TOOK MAIN's copy of map/<file>`, in the gate log header too, and the branch's
#   copy is kept as DIR/branch-copy-<ID>.md. Any other conflict, another map/ file included,
#   aborts the merge and stops with the paths named. Writes DIR/diff-<ID>.patch (base...HEAD; refuses an
#   empty one), then runs `./run_tests.sh --full` in the worktree under the base's
#   night_shift_nonet.sh and `flock /tmp/jugglebot-tests.lock`, into DIR/gate-<ID>.log:
#   HEAD=/BRANCH= first, the suite output, HEAD=/BRANCH= again, EXIT= last.
# land: refuses unless the gate log's HEAD= lines all equal the worktree's HEAD (nothing was
#   committed after the suite), the worktree is clean, the log's last EXIT= is 0, and
#   DIR/audit-<ID>.md exists; refuses if origin/main is not the base prepare merged (it never
#   improvises a merge: re-run prepare) and refuses a frozen-path branch again. Then runs the
#   gate from an extract of the base (never the branch's copy) with
#   `--waive c --record <verdicts>` (and --app-log DIR/app-<ID>.log when present). MERGE (and
#   the MERGE record on disk): pushes exactly the gated tip to main, as a fast-forward and
#   never forced, deletes the remote branch, removes the worktree and local branch, appends
#   one JSON line to <repo>/temp/reports/sitting-merge.jsonl. HOLD: prints the gate's reasons,
#   keeps everything. Re-running land after a land finishes any cleanup and exits 0.
#
# Why a fast-forward and not --no-ff: main then IS the gated tree, and every commit it gains
# is an ancestor of the MERGE record's tip, which tools/night_shift_landed.py counts as gated
# (a debrief made inside the night window included). The night's 2i tries --ff-only first too.
#
# Options: --repo R (default: the git top level of the current directory), --origin NAME
# (default origin), --verdicts DIR (default $NIGHT_SHIFT_VERDICTS, then
# ~/.local/state/jugglebot/verdicts, the gate's and the landed check's default).
# Test-only environment: SITTING_MERGE_LOCK (the suite lock file), SITTING_MERGE_PYTHON.
#
# Exit codes (every refusal prints one line `REFUSED <code> <reason>` on stderr):
#   0 prepare: suite green, ready for the audit; land: landed, or already landed
#   2 usage                       3 live robot session (prepare)
#   4 frozen path (f)             5 conflict beyond the generated pages and the Item's own map file
#   6 the branch adds nothing     7 suite red (prepare writes the log; land reads it)
#   8 no gate log (land)          9 committed after the suite: HEAD is not the log's HEAD=
#  10 uncommitted changes in the worktree
#  11 no audit file (land)       12 main moved since prepare (or during the push): re-run prepare
#  13 HOLD (land)                14 gate error, or a MERGE with no record on disk
#  15 git or setup failure       16 no prepared worktree and nothing landed (land)
set -euo pipefail

refuse() { echo "REFUSED $1 $2" >&2; exit "$1"; }
info() { echo "sitting_merge: $*" >&2; }

CMD="${1:-}"; shift || true
case "$CMD" in prepare|land) ;; *) refuse 2 "usage: sitting_merge.sh prepare|land --branch B --item ID --work DIR [--repo R] [--origin NAME] [--verdicts DIR]" ;; esac

BRANCH=""; ITEM=""; WORK=""; REPO=""; ORIGIN="origin"; VERDICTS=""
while (( $# )); do
  case "$1" in
    --branch) BRANCH="${2:-}"; shift 2 ;;
    --item) ITEM="${2:-}"; shift 2 ;;
    --work) WORK="${2:-}"; shift 2 ;;
    --repo) REPO="${2:-}"; shift 2 ;;
    --origin) ORIGIN="${2:-}"; shift 2 ;;
    --verdicts) VERDICTS="${2:-}"; shift 2 ;;
    *) refuse 2 "unknown argument: $1" ;;
  esac
done

BRANCH="${BRANCH#refs/remotes/}"; BRANCH="${BRANCH#"$ORIGIN"/}"
[[ "$ITEM" =~ ^[A-Z]+-[1-9][0-9]*$ ]] || refuse 2 "--item must be an Item ID such as BUG-80, not '$ITEM'"
# A held branch is night-shift/<date>-<ID> for THIS Item. The candidate branch
# night-shift/sitting-<date> is a build product and never matches.
[[ "$BRANCH" =~ ^night-shift/[0-9]{4}-[0-9]{2}-[0-9]{2}-([A-Z]+-[1-9][0-9]*)$ ]] \
  || refuse 2 "--branch must be a held branch night-shift/<date>-<ID>, not '$BRANCH' (never the candidate branch)"
[[ "${BASH_REMATCH[1]}" == "$ITEM" ]] || refuse 2 "--branch $BRANCH is not --item $ITEM's branch"
[[ -n "$WORK" ]] || refuse 2 "--work DIR required"
if [[ -z "$REPO" ]]; then REPO="$(git rev-parse --show-toplevel 2>/dev/null)" || refuse 2 "not in a git repo; pass --repo"; fi
REPO="$(cd "$REPO" && pwd)" || refuse 2 "--repo $REPO not found"
mkdir -p "$WORK" && WORK="$(cd "$WORK" && pwd)" || refuse 15 "cannot create --work $WORK"
VERDICTS="${VERDICTS:-${NIGHT_SHIFT_VERDICTS:-$HOME/.local/state/jugglebot/verdicts}}"
LOCK="${SITTING_MERGE_LOCK:-/tmp/jugglebot-tests.lock}"
if [[ -n "${SITTING_MERGE_PYTHON:-}" ]]; then PYTHON="$SITTING_MERGE_PYTHON"
elif [[ -x "$HOME/Desktop/PDJ_venv/venv/bin/python" ]]; then PYTHON="$HOME/Desktop/PDJ_venv/venv/bin/python"
else PYTHON="python3"; fi

WT="$WORK/wt-$ITEM"; EXTRACT="$WORK/gate-base-$ITEM"
GATE_LOG="$WORK/gate-$ITEM.log"; AUDIT="$WORK/audit-$ITEM.md"; APP_LOG="$WORK/app-$ITEM.log"
BASE_FILE="$WORK/base-$ITEM"; DIFF="$WORK/diff-$ITEM.patch"
JSONL="$REPO/temp/reports/sitting-merge.jsonl"
RREF="refs/remotes/$ORIGIN"
G() { git -C "$REPO" "$@"; }
W() { git -C "$WT" "$@"; }

fetch() { G fetch -q --prune "$ORIGIN" || refuse 15 "git fetch $ORIGIN failed"; }

# The base's own machinery (protocol 2a/2g): the gate, its mapkit, the network fence, the
# live-session guard. A branch can never supply the copy that judges or fences it.
extract_base() {
  rm -rf "$EXTRACT"; mkdir -p "$EXTRACT"
  G archive "$1" tools/night_shift_gate.py tools/night_shift_nonet.sh tools/lib/live_session.sh mapkit \
    | tar -x -C "$EXTRACT" || refuse 15 "cannot extract the gate, nonet wrapper and live_session guard from $1"
}

# Paths in base...tip that the base's gate freezes (condition (f)), one per line.
frozen_paths() {
  G diff -z --no-renames --name-only "$1...$2" \
    | "$PYTHON" -I -c '
import fnmatch, sys
sys.path.insert(0, sys.argv[1] + "/tools"); sys.path.insert(0, sys.argv[1])
from night_shift_gate import FORBIDDEN_PATHS
for p in sys.stdin.read().split("\0"):
    if p and any(fnmatch.fnmatchcase(p, f) for f in FORBIDDEN_PATHS):
        print(p)' "$EXTRACT" || refuse 15 "cannot read FORBIDDEN_PATHS from the base's gate"
}

refuse_frozen() {
  local hits; hits="$(frozen_paths "$1" "$2")"
  [[ -z "$hits" ]] || refuse 4 "frozen path(s) $(echo "$hits" | paste -sd' ' -): gate condition (f) cannot be waived; a frozen-path branch is merged by hand in a day Session, never by this script"
}

# The map file of the Item named by --item at the base, found by its id: through the base's
# own mapkit (never guessed from a slug). Prints the path only when exactly one file has it.
own_map_file() {
  "$PYTHON" -I - "$EXTRACT" "$REPO" "$1" "$ITEM" <<'PY' || true
import subprocess, sys
extract, repo, base, item = sys.argv[1:5]
sys.path.insert(0, extract)
from mapkit.frontmatter import split_frontmatter
def git(*a):
    return subprocess.run(["git", "-C", repo] + list(a), stdout=subprocess.PIPE,
                          stderr=subprocess.DEVNULL, universal_newlines=True).stdout
hits = []
for name in git("ls-tree", "--name-only", base, "map/").split():
    if name.endswith(".md"):
        try:
            meta, _ = split_frontmatter(git("show", base + ":" + name))
        except Exception:
            continue
        if str(meta.get("id", "")) == item:
            hits.append(name)
if len(hits) == 1:
    print(hits[0])
PY
}

GENERATED_RE='^(logbook/INDEX\.md|plans/[^/]+/INDEX\.md|docs/project-map\.md|docs/glossary\.md|docs/runbook\.md)$'

prepare() {
  # Anything from an earlier prepare is stale the moment we start again.
  rm -f "$GATE_LOG" "$AUDIT" "$DIFF" "$BASE_FILE" "$APP_LOG" "$WORK/branch-copy-$ITEM.md"
  fetch
  G rev-parse -q --verify "$RREF/$BRANCH^{commit}" >/dev/null || refuse 15 "no remote branch $ORIGIN/$BRANCH"
  local base; base="$(G rev-parse "$RREF/main")"
  extract_base "$base"
  # shellcheck source=lib/live_session.sh
  source "$EXTRACT/tools/lib/live_session.sh" || refuse 15 "cannot source the base's live_session.sh"
  declare -F live_session >/dev/null || refuse 15 "the base's live_session.sh defines no live_session"
  if live_session; then refuse 3 "a live robot session holds the box (launch up); the full suite waits for it to end"; fi
  refuse_frozen "$base" "$RREF/$BRANCH"

  if [[ -d "$WT" ]]; then G worktree remove --force "$WT" >/dev/null 2>&1 || rm -rf "$WT"; fi
  G worktree prune
  G worktree add -q -B "$BRANCH" "$WT" "$RREF/$BRANCH" >/dev/null 2>&1 || refuse 15 "git worktree add $WT failed"
  echo "$base" > "$BASE_FILE"
  local own; own="$(own_map_file "$base")"
  local taken=()

  if ! W merge -q --no-edit -m "merge(sitting): $ORIGIN/main into $BRANCH for the debrief gate" "$base" >/dev/null 2>&1; then
    local conflicts other
    conflicts="$(W diff --name-only --diff-filter=U)"
    # Mechanical answers only: the generated pages, and the Item's OWN map file, whose state
    # (status, Sitting section, skip keys) main writes; both sides touch its updated: line.
    other="$(echo "$conflicts" | grep -Ev "$GENERATED_RE" | grep -vxF "${own:-/}" | grep -v '^$' || true)"
    if [[ -n "$other" || -z "$conflicts" ]]; then
      W merge --abort >/dev/null 2>&1 || true
      G worktree remove --force "$WT" >/dev/null 2>&1 || true
      rm -f "$BASE_FILE"
      refuse 5 "merge conflict with main in ${other:-unknown paths}: the branch stays held; resolve it in a day Session"
    fi
    # Take the base's copy by naming the commit (in this direction --ours is the branch),
    # keep the branch's copy of the Item's file for the debrief to read, then regenerate.
    local p
    while IFS= read -r p; do
      [[ -n "$p" ]] || continue
      if [[ "$p" == "$own" ]]; then
        W show ":2:$p" > "$WORK/branch-copy-$ITEM.md" 2>/dev/null || true  # stage 2 = the branch
        echo "TOOK MAIN's copy of $p"
        taken+=("$p")
      fi
      W checkout "$base" -- "$p"
    done <<<"$conflicts"
    (cd "$WT" && "$PYTHON" -m mapkit gen --repo "$WT" >/dev/null) || refuse 15 "python -m mapkit gen failed in $WT"
    W add -A
    W commit -q --no-edit >/dev/null || refuse 15 "cannot commit the regenerated merge"
    info "regenerated: $(echo "$conflicts" | paste -sd' ' -)"
  fi
  [[ -z "$(W status --porcelain)" ]] || refuse 10 "worktree $WT is not clean after the merge: $(W status --porcelain | head -3 | paste -sd';' -)"

  W diff "$base...HEAD" > "$DIFF"
  [[ -s "$DIFF" ]] || refuse 6 "$BRANCH adds nothing to main (empty diff): nothing to audit or merge"

  local head; head="$(W rev-parse HEAD)"
  printf 'HEAD=%s\nBRANCH=%s\n' "$head" "$(W rev-parse --abbrev-ref HEAD)" > "$GATE_LOG"
  for p in "${taken[@]+"${taken[@]}"}"; do
    echo "TOOK MAIN's copy of $p (merge conflict; the branch's copy is $WORK/branch-copy-$ITEM.md)" >> "$GATE_LOG"
  done
  info "suite on $head (waits for $LOCK); log $GATE_LOG"
  local rc=0
  (
    set +eu
    [[ -n "${ROS_DISTRO:-}" || ! -f /opt/ros/foxy/setup.bash ]] || source /opt/ros/foxy/setup.bash
    [[ -n "${VIRTUAL_ENV:-}" || ! -f "$HOME/Desktop/PDJ_venv/venv/bin/activate" ]] || source "$HOME/Desktop/PDJ_venv/venv/bin/activate"
    cd "$WT" && exec flock "$LOCK" "$EXTRACT/tools/night_shift_nonet.sh" ./run_tests.sh --full
  ) >> "$GATE_LOG" 2>&1 || rc=$?
  printf 'HEAD=%s\nBRANCH=%s\nEXIT=%s\n' "$(W rev-parse HEAD)" "$(W rev-parse --abbrev-ref HEAD)" "$rc" >> "$GATE_LOG"
  (( rc == 0 )) || refuse 7 "suite red (EXIT=$rc) on $head; see $GATE_LOG; the branch stays held"
  echo "PREPARED $ITEM $head"
  echo "next: audit $DIFF, write $AUDIT ending 'Audited HEAD: $head' and 'Behaviour-affecting findings open: <n>', then land"
}

finish_cleanup() {  # idempotent: remote branch, worktree, local branch, the jsonl line
  local tip="$1" base="$2" record="$3"
  if G rev-parse -q --verify "$RREF/$BRANCH" >/dev/null; then
    G push -q "$ORIGIN" --delete "$BRANCH" >/dev/null 2>&1 || info "WARN could not delete $ORIGIN/$BRANCH; re-run land to retry"
  fi
  if [[ -d "$WT" ]]; then G worktree remove --force "$WT" >/dev/null 2>&1 || info "WARN could not remove $WT"; fi
  G worktree prune
  if G rev-parse -q --verify "refs/heads/$BRANCH" >/dev/null && [[ "$(G rev-parse "refs/heads/$BRANCH")" == "$tip" ]]; then
    G branch -q -D "$BRANCH" >/dev/null 2>&1 || true
  fi
  rm -rf "$EXTRACT"
  mkdir -p "$(dirname "$JSONL")"
  if ! grep -qs "\"tip\": \"$tip\"" "$JSONL"; then
    printf '{"ts": "%s", "item": "%s", "branch": "%s", "tip": "%s", "base": "%s", "verdict": "MERGE", "record": "%s", "gate_log": "%s", "audit": "%s"}\n' \
      "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$ITEM" "$BRANCH" "$tip" "$base" "$record" "$GATE_LOG" "$AUDIT" >> "$JSONL"
  fi
}

land() {
  fetch
  local logged=""
  [[ -f "$GATE_LOG" ]] && logged="$(grep -E '^HEAD=' "$GATE_LOG" | head -1 | cut -d= -f2)"
  # Already landed? (a re-run after a land, or after one whose cleanup was cut short)
  if [[ -n "$logged" ]] && G cat-file -e "$logged^{commit}" 2>/dev/null \
      && G merge-base --is-ancestor "$logged" "$RREF/main" \
      && [[ -f "$VERDICTS/$logged.MERGE.json" ]]; then
    finish_cleanup "$logged" "$(cat "$BASE_FILE" 2>/dev/null || echo unknown)" "$VERDICTS/$logged.MERGE.json"
    echo "ALREADY LANDED $ITEM $logged"
    return 0
  fi
  [[ -d "$WT" ]] || refuse 16 "no prepared worktree $WT and $ITEM has not landed: run prepare first"
  [[ -f "$GATE_LOG" ]] || refuse 8 "no gate log $GATE_LOG: run prepare"
  [[ -f "$BASE_FILE" ]] || refuse 8 "no base record $BASE_FILE: run prepare"
  local head base; head="$(W rev-parse HEAD)"; base="$(cat "$BASE_FILE")"
  local h
  for h in $(grep -E '^HEAD=' "$GATE_LOG" | cut -d= -f2); do
    [[ "$h" == "$head" ]] || refuse 9 "the worktree HEAD $head is not the gate log's HEAD=$h: something was committed after the suite; re-run prepare"
  done
  [[ -z "$(W status --porcelain)" ]] || refuse 10 "worktree $WT has uncommitted changes after the suite; re-run prepare"
  local last; last="$(grep -E '^EXIT=' "$GATE_LOG" | tail -1 | cut -d= -f2)"
  [[ "$last" == "0" ]] || refuse 7 "gate log $GATE_LOG ends EXIT=${last:-none}: the suite did not pass"
  [[ -f "$AUDIT" ]] || refuse 11 "no audit $AUDIT: run /audit on $DIFF and write it, ending 'Audited HEAD: $head'"
  local main; main="$(G rev-parse "$RREF/main")"
  [[ "$main" == "$base" ]] || refuse 12 "main moved since prepare ($base -> $main): re-run prepare (a fresh merge, suite and audit); never merged untested"
  G merge-base --is-ancestor "$base" "$head" || refuse 12 "the prepared tip does not contain the base: re-run prepare"

  extract_base "$base"
  refuse_frozen "$base" "$head"
  local args=(--repo "$REPO" --base "$base" --branch "$BRANCH" --item "$ITEM" --gate-log "$GATE_LOG"
              --audit "$AUDIT" --waive c --record "$VERDICTS")
  [[ -f "$APP_LOG" ]] && args+=(--app-log "$APP_LOG")
  local out rc=0
  out="$("$PYTHON" -I "$EXTRACT/tools/night_shift_gate.py" "${args[@]}" 2>&1)" || rc=$?
  echo "$out"
  if (( rc == 1 )); then refuse 13 "HOLD: $ITEM stays on $BRANCH; the worktree and logs in $WORK are kept"; fi
  (( rc == 0 )) || refuse 14 "gate error (exit $rc); nothing pushed"
  [[ "$(echo "$out" | head -1)" == MERGE* ]] || refuse 14 "gate exited 0 without MERGE; nothing pushed"
  local record="$VERDICTS/$head.MERGE.json"
  [[ -f "$record" ]] || refuse 14 "no MERGE record $record on disk; nothing pushed"

  # Exactly the gated tip, as a fast-forward of the base; never forced.
  G push -q "$ORIGIN" "$head:refs/heads/main" >/dev/null 2>&1 \
    || refuse 12 "push of $head to main rejected (main moved during land): re-run prepare"
  finish_cleanup "$head" "$base" "$record"
  echo "LANDED $ITEM $head"
}

"$CMD"
