#!/usr/bin/env bash
# Night Shift candidate branch (plans/active/night-shift.md section 4 P4, T-U6).
#
# What: stack every held Night Shift branch (origin/night-shift/<date>-JB-n) whose Item
# carries a `## Sitting` section into ONE branch, night-shift/sitting-<date>.
# Why: the owner checks out a single branch at a Sitting, not one per Item. A branch
# that conflicts with the stack is reported as a HOLD row and skipped, so one bad
# merge never costs the rest of the night's work.
#
# Rules:
#   - never checks out or modifies main; the stack is built in a temporary worktree
#     under <repo>/temp/candidate-<date>/ which is removed at the end, even on error;
#   - the candidate branch is a BUILD PRODUCT: if it exists locally it is reset to BASE;
#   - a conflicting merge is aborted and the script goes on;
#   - never force-pushes: a rejected push prints PUSH_REJECTED and exits 1.
#
# Usage: night_shift_candidate.sh --date YYYY-MM-DD [--repo DIR] [--base origin/main]
#                                 [--push] [--remote origin]
# Exit: 0 when nothing failed (HOLD and SKIP rows are not failures), 1 otherwise.
# A second --push on the same date is rejected (the local branch is rebuilt from
# BASE, never force-pushed): delete the remote candidate first
# (`git push origin --delete night-shift/sitting-<date>`) or pick a new date.
set -uo pipefail

DATE=""; REPO="."; BASE="origin/main"; PUSH=0; REMOTE="origin"
while (( $# )); do
  case "$1" in
    --date) DATE="${2:-}"; shift 2 ;;
    --repo) REPO="${2:-}"; shift 2 ;;
    --base) BASE="${2:-}"; shift 2 ;;
    --remote) REMOTE="${2:-}"; shift 2 ;;
    --push) PUSH=1; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ "$DATE" =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}$ ]] || { echo "--date YYYY-MM-DD required" >&2; exit 2; }
REPO="$(cd "$REPO" && pwd)" || exit 2

if [[ -n "${NIGHT_SHIFT_PYTHON:-}" ]]; then PYTHON="$NIGHT_SHIFT_PYTHON"
elif [[ -x "$HOME/Desktop/PDJ_venv/venv/bin/python" ]]; then PYTHON="$HOME/Desktop/PDJ_venv/venv/bin/python"
else PYTHON="python3"; fi

CAND="night-shift/sitting-$DATE"
WT="$REPO/temp/candidate-$DATE"
OUT="$REPO/temp/reports/night-shift/candidate-$DATE.txt"
ROWS=()
merged=0; held=0; skipped=0; rc=0

cleanup() {
  git -C "$REPO" worktree remove --force "$WT" >/dev/null 2>&1
  rm -rf "$WT"
  git -C "$REPO" worktree prune >/dev/null 2>&1
}
trap cleanup EXIT

row() { ROWS+=("$1"); }

git -C "$REPO" fetch -q "$REMOTE" || { echo "fetch failed" >&2; exit 1; }
cleanup
mkdir -p "$(dirname "$WT")" "$(dirname "$OUT")"
git -C "$REPO" worktree add -q -B "$CAND" "$WT" "$BASE" || { echo "worktree add failed" >&2; exit 1; }

# Does BASE hold an Item JB-n, and does its body have a ## Sitting section?
# Prints FOUND, NOSECTION or NOITEM.
item_state() {
  REPO="$REPO" BASE="$BASE" JB="$1" "$PYTHON" - <<'PY'
import os, subprocess, sys
repo, base, jb = os.environ["REPO"], os.environ["BASE"], os.environ["JB"]
sys.path.insert(0, repo)
from mapkit.frontmatter import split_frontmatter
from mapkit.sitting import has_sitting_section
def git(*a):
    return subprocess.run(["git", "-C", repo] + list(a), stdout=subprocess.PIPE,
                          stderr=subprocess.DEVNULL, universal_newlines=True).stdout
for name in git("ls-tree", "--name-only", base, "map/").split():
    if not name.endswith(".md"):
        continue
    try:
        meta, body = split_frontmatter(git("show", base + ":" + name))
    except Exception:
        continue
    if str(meta.get("id", "")) == jb:
        print("FOUND" if has_sitting_section(body) else "NOSECTION")
        sys.exit(0)
print("NOITEM")
PY
}

mapfile -t BRANCHES < <(git -C "$REPO" for-each-ref --format='%(refname:short)' \
    "refs/remotes/$REMOTE/night-shift/*-JB-*" | grep -v '/sitting-' \
  | awk '{n=$0; sub(/.*-JB-/,"",n); print n" "$0}' | sort -n | cut -d' ' -f2-)

for br in "${BRANCHES[@]:-}"; do
  [[ -n "$br" ]] || continue
  jb="JB-${br##*-JB-}"
  state="$(item_state "$jb")"
  case "$state" in
    FOUND) ;;
    NOSECTION) row "SKIP $br $jb has no ## Sitting section"; skipped=$((skipped+1)); continue ;;
    *) row "SKIP $br no Item $jb on $BASE"; skipped=$((skipped+1)); continue ;;
  esac
  if git -C "$WT" merge --no-ff -q -m "candidate(sitting-$DATE): $jb from $br" "$br" >/dev/null 2>&1; then
    row "MERGED $br $jb"; merged=$((merged+1))
  else
    files="$(git -C "$WT" diff --name-only --diff-filter=U | paste -sd, - | sed 's/,/, /g')"
    git -C "$WT" merge --abort >/dev/null 2>&1
    row "HOLD $br $jb conflict: ${files:-unknown}"; held=$((held+1))
  fi
done

if (( PUSH )) && (( merged > 0 )); then
  if ! git -C "$WT" push -q -u "$REMOTE" "$CAND" >/dev/null 2>&1; then
    row "PUSH_REJECTED $CAND"; rc=1
  fi
fi

row "candidate: $merged merged, $held held, $skipped skipped -> $CAND"
printf '%s\n' "${ROWS[@]}" | tee "$OUT"
exit "$rc"
