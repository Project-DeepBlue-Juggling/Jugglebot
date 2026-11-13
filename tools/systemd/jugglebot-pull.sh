#!/usr/bin/env bash
# Poll origin for new commits and fast-forward the Jugglebot working tree.
#
# Source of truth is THIS file (tools/systemd/jugglebot-pull.sh); the copy that
# the user unit jugglebot-pull.service runs is ~/System_Scripts/jugglebot-pull.sh,
# installed with:  install -m 755 tools/systemd/jugglebot-pull.sh ~/System_Scripts/
# It runs from outside the tree on purpose: a script inside the tree could be
# rewritten by the very merge it is executing, and bash reads scripts incrementally.
#
# Replaces the push webhook, which stopped working on 2026-08-02 (CGNAT: inbound
# port-forwarding cannot work, so we poll). Deliberately conservative:
#   - never changes branch
#   - only ever fast-forwards; never creates a merge commit
#   - leaves the tree alone if tracked files are modified
#   - skips the merge while the test suite holds its flock
# A plain fetch always runs, so remote work is visible via origin/* refs.
#
# Every run (except "another run is active") writes ONE line, atomically, to
# <repo>/temp/reports/pull/status:
#   <iso> <WORD> behind=<n> since=<iso of the first consecutive non-OK run, or -> files=<up to 3 dirty tracked paths, comma-joined, or ->
# WORD: OK | SKIPPED_DIRTY | SKIPPED_SUITE | DIVERGED | DETACHED
#       | FETCH_FAILED (fetch error, or no answer within FETCH_TIMEOUT_S)
#       | NO_UPSTREAM (no origin/<branch>) | FF_FAILED (merge --ff-only refused)
# Non-OK also goes to `logger -p user.warning`. tools/nightly_ticker.sh check
# prints a PULL line when a non-OK status is older than 10 minutes, so a stalled
# pull (43 commits behind, 2026-11-13) is told to every Session at start.
#
# Env overrides (tests): JUGGLEBOT_PULL_REPO, JUGGLEBOT_PULL_STATUS,
# JUGGLEBOT_SUITE_LOCK, JUGGLEBOT_PULL_LOCK, JUGGLEBOT_PULL_FETCH_TIMEOUT_S.

set -uo pipefail

REPO_DIR="${JUGGLEBOT_PULL_REPO:-/home/jetson/Desktop/Jugglebot}"
SUITE_LOCK="${JUGGLEBOT_SUITE_LOCK:-/tmp/jugglebot-run_tests.lock}"   # shared with run_tests.sh
SELF_LOCK="${JUGGLEBOT_PULL_LOCK:-/tmp/jugglebot-pull.lock}"
STATUS_FILE="${JUGGLEBOT_PULL_STATUS:-$REPO_DIR/temp/reports/pull/status}"
# Bound on the fetch: a stalled TCP connection otherwise hangs this oneshot forever, and
# an active oneshot cannot be re-triggered by its 2-min timer (the status file then goes
# stale in silence). Well under the period; the unit's TimeoutStartSec is the backstop.
FETCH_TIMEOUT_S="${JUGGLEBOT_PULL_FETCH_TIMEOUT_S:-60}"

# Anonymous fetch (public repo); never block on a prompt under systemd.
export GIT_TERMINAL_PROMPT=0
export GIT_ASKPASS=/bin/false

cd "$REPO_DIR" || { echo "repo not found: $REPO_DIR" >&2; exit 1; }

exec 8>"$SELF_LOCK"
flock -n 8 || { echo "another jugglebot-pull run is active; skipping"; exit 0; }

behind="?"
files="-"

# report WORD: write the status line (atomic) and, if not OK, log a warning.
report() {
    local word="$1" now since prev prev_word prev_since line tmp
    now="$(date -Iseconds)"
    since="-"
    if [ "$word" != "OK" ]; then
        since="$now"
        if [ -s "$STATUS_FILE" ]; then
            prev="$(head -n1 "$STATUS_FILE")"
            prev_word="$(printf '%s' "$prev" | cut -d' ' -f2)"
            prev_since="$(printf '%s' "$prev" | sed -n 's/.* since=\([^ ]*\).*/\1/p')"
            if [ -n "$prev_word" ] && [ "$prev_word" != "OK" ] && [ -n "$prev_since" ] && [ "$prev_since" != "-" ]; then
                since="$prev_since"
            fi
        fi
    fi
    line="$now $word behind=$behind since=$since files=$files"
    if mkdir -p "$(dirname "$STATUS_FILE")" 2>/dev/null; then
        tmp="$STATUS_FILE.tmp.$$"
        printf '%s\n' "$line" >"$tmp" && mv -f "$tmp" "$STATUS_FILE"
    fi
    [ "$word" = "OK" ] || logger -p user.warning -t jugglebot-pull "$line" 2>/dev/null || true
}

# timeout signals its whole process group (git's remote helper included); -k: SIGKILL
# 10 s later if TERM is ignored. 124/137 = timed out; that is a FETCH_FAILED like any other.
git_rc=0
timeout -k 10 "$FETCH_TIMEOUT_S" git fetch --prune --quiet origin || git_rc=$?
if [ "$git_rc" -ne 0 ]; then
    if [ "$git_rc" -eq 124 ] || [ "$git_rc" -eq 137 ]; then
        echo "fetch timed out after ${FETCH_TIMEOUT_S}s" >&2
    else
        echo "fetch failed" >&2
    fi
    report FETCH_FAILED
    exit 1
fi

branch=$(git symbolic-ref --short -q HEAD) || {
    echo "detached HEAD — fetched only"; report DETACHED; exit 0; }

if ! git rev-parse --verify --quiet "refs/remotes/origin/$branch" >/dev/null; then
    echo "no origin/$branch — fetched only"; report NO_UPSTREAM; exit 0
fi

behind="$(git rev-list --count "HEAD..origin/$branch" 2>/dev/null || echo '?')"

local_sha=$(git rev-parse HEAD)
remote_sha=$(git rev-parse "origin/$branch")
if [ "$local_sha" = "$remote_sha" ]; then
    report OK; exit 0   # already current; stay quiet on stdout
fi

# Strictly behind? If HEAD is not an ancestor of origin/<branch> then there are
# local commits (or a rewrite upstream) and fast-forward is the wrong answer.
if ! git merge-base --is-ancestor HEAD "origin/$branch"; then
    echo "$branch has diverged from origin/$branch — leaving it alone"; report DIVERGED; exit 0
fi

# Tracked modifications block the merge. Untracked files deliberately do NOT:
# --ff-only refuses on its own if it would clobber one.
if ! git diff --quiet || ! git diff --cached --quiet; then
    files="$( { git diff --name-only; git diff --cached --name-only; } | sort -u | head -n3 | paste -sd, -)"
    [ -n "$files" ] || files="-"
    echo "uncommitted changes present — fetched only, not merging"; report SKIPPED_DIRTY; exit 0
fi

# Never move the tree under a running test suite.
exec 9>"$SUITE_LOCK"
if ! flock -n 9; then
    echo "test suite holds $SUITE_LOCK — fetched only, not merging"; report SKIPPED_SUITE; exit 0
fi

echo "fast-forwarding $branch: $(git rev-parse --short HEAD) -> $(git rev-parse --short "origin/$branch")"
if git merge --ff-only --quiet "origin/$branch"; then
    behind=0
    report OK
else
    rc=$?
    report FF_FAILED
    exit "$rc"
fi
