#!/usr/bin/env bash
# Night Shift candidate branch (plans/active/night-shift.md section 4 P4, T-U6).
#
# What: stack every held Night Shift branch (origin/night-shift/<date>-<ID>, the Item
# ID: BUG-n FEAT-n IDEA-n PWAY-n TKT-n) whose Item
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
#   - never force-pushes: a rejected push prints PUSH_REJECTED and exits 1;
#   - branches stack in branch-date order, then by ID prefix and number;
#   - a held branch is never dropped silently: a night-shift/* branch whose name does
#     not parse is an UNPARSED row, and an ID whose prefix names another kind than its
#     Item's is a SKIP row; either makes the exit 1 (the rest still stacks).
#
# Usage: night_shift_candidate.sh --date YYYY-MM-DD [--repo DIR] [--base origin/main]
#                                 [--push] [--remote origin]
# Exit: 0 when nothing failed (HOLD and SKIP rows are not failures, except the
# prefix/kind SKIP), 1 otherwise, UNPARSED included.
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

# Does BASE hold the Item with this ID, and does its body have a ## Sitting section?
# Prints FOUND, NOSECTION, NOITEM or "MISMATCH <prefix kind> <item kind>".
item_state() {
  REPO="$REPO" BASE="$BASE" ITEM_ID="$1" "$PYTHON" - <<'PY'
import os, subprocess, sys
repo, base, item_id = os.environ["REPO"], os.environ["BASE"], os.environ["ITEM_ID"]
sys.path.insert(0, repo)
from mapkit.frontmatter import split_frontmatter
from mapkit.ids import KIND_OF_PREFIX, mismatch, split_ref
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
    if str(meta.get("id", "")) == item_id:
        kind = str(meta.get("kind", ""))
        if mismatch(item_id, kind):
            print("MISMATCH %s %s" % (KIND_OF_PREFIX[split_ref(item_id)[0]], kind))
        else:
            print("FOUND" if has_sitting_section(body) else "NOSECTION")
        sys.exit(0)
print("NOITEM")
PY
}

# The ID prefixes come from mapkit (one source): "BUG|FEAT|IDEA|PWAY|TKT".
PREFIXES="$("$PYTHON" -c 'import sys; sys.path.insert(0, sys.argv[1]); from mapkit.ids import ALL_PREFIXES; print("|".join(ALL_PREFIXES))' "$REPO")"
[[ "$PREFIXES" =~ ^[A-Z|]+$ ]] || { echo "cannot read the ID prefixes from $REPO/mapkit/ids.py" >&2; exit 1; }
NAME_RE="^$REMOTE/night-shift/([0-9]{4}-[0-9]{2}-[0-9]{2})-(($PREFIXES)-([1-9][0-9]*))\$"

# Every night-shift/* branch but the candidates; each one parses or is an UNPARSED row.
mapfile -t ALL < <(git -C "$REPO" for-each-ref --format='%(refname:short)' \
    "refs/remotes/$REMOTE/night-shift/")
KEYED=()
for br in "${ALL[@]:-}"; do
  [[ -n "$br" && "$br" != "$REMOTE/night-shift/sitting-"* ]] || continue
  if [[ "$br" =~ $NAME_RE ]]; then
    KEYED+=("${BASH_REMATCH[1]} ${BASH_REMATCH[3]} ${BASH_REMATCH[4]} ${BASH_REMATCH[2]} $br")
  else
    row "UNPARSED $br is not night-shift/<date>-<ID> (ID prefixes $PREFIXES)"; rc=1
  fi
done
# date, then prefix, then number: same-date <ID> branches keep their numeric order
mapfile -t BRANCHES < <(printf '%s\n' "${KEYED[@]:-}" | grep -v '^$' \
  | sort -k1,1 -k2,2 -k3,3n | cut -d' ' -f4-)

for entry in "${BRANCHES[@]:-}"; do
  [[ -n "$entry" ]] || continue
  item_id="${entry%% *}"; br="${entry#* }"
  state="$(item_state "$item_id")"
  case "$state" in
    FOUND) ;;
    MISMATCH*) read -r _ pkind ikind <<<"$state"
      row "SKIP $br $item_id names a $pkind, but its Item is a $ikind"; skipped=$((skipped+1)); rc=1; continue ;;
    NOSECTION) row "SKIP $br $item_id has no ## Sitting section"; skipped=$((skipped+1)); continue ;;
    NOITEM) row "SKIP $br no Item $item_id on $BASE"; skipped=$((skipped+1)); continue ;;
    *) row "ERROR $br item_state failed for $item_id: ${state:-no output}"; rc=1; continue ;;
  esac
  if git -C "$WT" merge --no-ff -q -m "candidate(sitting-$DATE): $item_id from $br" "$br" >/dev/null 2>&1; then
    row "MERGED $br $item_id"; merged=$((merged+1))
  else
    files="$(git -C "$WT" diff --name-only --diff-filter=U | paste -sd, - | sed 's/,/, /g')"
    git -C "$WT" merge --abort >/dev/null 2>&1
    row "HOLD $br $item_id conflict: ${files:-unknown}"; held=$((held+1))
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
