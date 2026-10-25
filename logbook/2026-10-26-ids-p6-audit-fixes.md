---
title: "Map item IDs P6 audit fixes: item push resolves and checks first, soft-reset undo, no amend of a published commit"
type: bugfix
date: 2026-10-26
status: resolved
phase: "map-item-ids - P6 phase-end audit"
files_changed:
  - mapkit/itemcli.py
  - tests/sim/test_mapkit_item_ids_cli.py
  - .claude/commands/inbox.md
  - docs/agents/map-writes.md
  - docs/agents/issue-tracker.md
  - DOCUMENTATION_GUIDE.md
  - plans/active/map-item-ids.md
  - logbook/2026-10-26-map-item-ids-p6-jugglebot.md
subsystem:
  - docs
tags:
  - tooling
  - project-map
---

## Summary

Phase-end audit of `map-item-ids-p6-jugglebot`, behaviour findings fixed with tests:

- **`item push` resolves its argument first.** `push_item` now goes through `find_item` (ID or stem) before any git call, so `item push JB-64` for an Item that does not exist refuses with the resolver's message instead of rebasing and pushing every unpushed commit and then failing on the load with a false "a local commit for JB-64 is waiting".
- **Undo advice is `git reset --soft HEAD~1`** (then delete the Item file and revert the docs page if unwanted). `--hard` with `--autostash` destroyed the operator's tracked edits. Changed in `_waiting_note`, `map-writes.md` and the test assertion.
- **`_head_adds` amends only an unpublished commit** (HEAD not an ancestor of the upstream, else origin/main). Re-running `item push` after a success used to amend a commit the remote already had; it now takes the separate `map: regenerate` / `map: renumber` commit path.
- **`/inbox --from-issue`** closes the GitHub issue as a new step 6, only after `item push` reports pushed (it used to close before the Avoid-word check and the push). Step 3 no longer claims `item new` pulls under `--commit --no-push`; the operator's preceding `git pull --ff-only` is the pull.
- **Narrative.** `DOCUMENTATION_GUIDE.md` § 2.10: an ID changes only before publication (the unpushed `map: file` commit is amended) or by a `map: renumber` commit repairing a published duplicate. `mapkit ids --check`'s `changed_ids` is still right: it compares against the ref's IDs, an unpushed Item is absent from the ref and is skipped, and a published ID can only change with a `map: renumber` commit. The plan's status is brought to P5 complete (fork `0a4cd9a`, deployed 2026-10-26), P4 complete except its phone-filed-Item check, P6 in progress. The earlier entry's title and headline are narrowed and `issue-tracker.md` now describes `item push`.

No control or hardware path touched.

## Verification

- 2026-10-26, `pytest tests/sim/test_mapkit_item_ids_cli.py -q`: 20 passed; against the pre-fix `mapkit/itemcli.py` the three new tests and the `--soft` assertion fail (4 failed, 16 passed).
- Gate 2026-10-26, `./run_tests.sh -q`: PASS, parallel 369 s rc 0, serial 22 s rc 0, total 391 s; `-q` prints no pass tally, so the counts are `pytest tests/ --collect-only -q -m` collections of the same run set: 7370 parallel + 5 serial; no failures, rc 0 both phases).
