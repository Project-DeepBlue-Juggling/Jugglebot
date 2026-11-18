---
title: "The Night Shift history ledger names a failed prune once, and stale tracking refs are pruned before the prune (BUG-109)"
type: bugfix
date: 2026-11-18
status: complete
phase: "night-shift close-out"
subsystem: tooling
files_changed:
  - tools/night_shift.sh
  - tests/sim/test_night_shift_runner.py
  - map/bug-the-night-shift-history-ledger-re-adds-old-form-branch-names.md
  - docs/project-map.md
  - logbook/INDEX.md
---

# The history ledger names a failed prune once (BUG-109)

**What.** Two changes in `tools/night_shift.sh`. (1) `landed_step`'s fetch, the last
fetch before `prune_step`, is now `git fetch -q --prune origin`, so a remote-tracking
ref whose branch is already gone from origin is dropped before the prune reads its
candidates. (2) `history_stage`, which rebuilds the committed
`docs/night-shift/history.jsonl` from the local history every night, now publishes a
`prune_failed` name in a row only if the committed ledger already has it in that same
row, or no other row has it (committed, or earlier in the rebuild). `last_run.json` and
the local `$REPORTS/history.jsonl` keep every night's full list; a failed delete still
never fails the night. Which branches are deleted, the `--force-with-lease` guard and
the gating at the top of `prune_step` are unchanged.

**Why.** The observed cause was stale tracking refs in the Night Shift clone: every
runner fetch was plain `fetch`, so a branch deleted on origin by any other path kept its
`refs/remotes/origin/` ref, `prune_step` saw a merged candidate, the leased delete of a
ref origin no longer had failed, and the rebuilt ledger named it again every night. Six
names rode four rows, four of them old-form branch names, and the pinned per-file count
in `tests/sim/test_no_legacy_item_ids.py` went red on main (re-pinned once to 34 at the
2026-11-18 debrief). Fix (1) removes the cause; fix (2) bounds the ledger for a delete
that fails for any other reason (a refusing hook, a lease that lost, the network).

**Design, and what it rules out.** The once-rule keys on what is already committed, so
the four existing rows keep their lists byte-for-byte and the pinned count stays 34; a
plain "drop any name an earlier row named" rebuild would have rewritten rows 2 to 4
(34 to 22) and turned main red the next night. Failure modes accepted: if the committed
ledger file is missing or unreadable, the rule degrades to "first row wins" over the
local history (still at most once, but it may then rewrite an old row); a branch whose
delete fails, succeeds, and later fails again under the same name is not named the
second time (`last_run.json` still has it). The `--prune` fetch also means a local
`night-shift/*` branch whose upstream was a stale ref is judged by `branch -d` against
`HEAD` instead of that ref, so an unmerged one is kept rather than deleted: stricter,
never looser. The alternative of an `ls-remote` check after a failed delete was not
taken: it needs a network round trip per failure and leaves the stale ref in place for
everything else that reads `refs/remotes/origin/`.

**Verification.** New tests in `tests/sim/test_night_shift_runner.py`: a stale tracking
ref is in no ledger row over two nights and its ref is gone; a delete refused by an
origin hook (the existing deterministic driver) is in the ledger once over two nights
while `last_run.json` keeps it; a pre-fix ledger whose two rows both name a branch keeps
both rows unchanged. All three failed before the change (2026-11-18,
`tools/night_shift_nonet.sh python -m pytest tests/sim/test_night_shift_runner.py -q -p
no:cacheprovider -k "stale_tracking or really_fails or published_row"`: 3 failed), and
the stale-ref test fails again with only `--prune` reverted.

2026-11-18, `flock /tmp/jugglebot-tests.lock tools/night_shift_nonet.sh python -m pytest
tests/sim/test_night_shift_runner.py tests/sim/test_night_shift_landed.py
tests/sim/test_night_shift_protocol.py tests/sim/test_no_legacy_item_ids.py -q -p
no:cacheprovider`: 170 passed in 159.28 s (the legacy-ID pin stays 34, unchanged).
2026-11-18, `tools/night_shift_nonet.sh python -m pytest tests/sim/test_logbook_front_matter.py
tests/sim/test_logbook_index.py tests/sim/test_docs_links.py tests/sim/test_mapkit_docs.py
tests/sim/test_mapkit_itemcli.py tests/sim/test_mapkit_item_ids_cli.py
tests/sim/test_mapkit_metrics.py tests/sim/test_mapkit_steps.py -q -p no:cacheprovider`:
110 passed in 14.82 s. `bash -n tools/night_shift.sh`: clean.
