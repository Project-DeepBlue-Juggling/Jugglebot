---
title: "Debrief merge as a tested script: tools/sitting_merge.sh prepare and land (closeout C10)"
type: feature
date: 2026-11-14
status: complete
phase: "night-shift close-out"
subsystem: tools
files_changed:
  - tools/sitting_merge.sh
  - tests/sim/test_sitting_merge.py
  - tools/night_shift_gate.py
  - tests/sim/test_night_shift_gate.py
  - tools/night_shift_landed.py
  - tests/sim/test_night_shift_landed.py
  - .claude/commands/sitting-debrief.md
  - tests/sim/test_sitting_debrief_protocol.py
  - logbook/2026-11-14-sitting-merge-script.md
  - logbook/INDEX.md
tags:
  - night-shift
  - sitting-debrief
  - merge-gate
---

## What

Closeout C10: the `/sitting-debrief` merge of a passed Item's held branch is now
`tools/sitting_merge.sh`, not nine hand-typed steps.

- `prepare --branch night-shift/<d>-<ID> --item <ID> --work DIR` refuses a live robot session
  (3) and a frozen-path branch (4), makes its own worktree `DIR/wt-<ID>`, merges `origin/main`
  in (generated pages and the Item's own map file take main's copy, then `mapkit gen`; any other
  conflict, another Item's `map/` file included, aborts with the paths named, 5), refuses an empty diff (6), and runs `./run_tests.sh --full`
  under the base's `night_shift_nonet.sh` and `flock /tmp/jugglebot-tests.lock` into
  `DIR/gate-<ID>.log` (HEAD=/BRANCH= first and again after the suite, EXIT= last; red is 7).
- `land` refuses a missing log (8), a commit after the suite (9), a dirty worktree (10), a
  red log (7), a missing `DIR/audit-<ID>.md` (11), a `main` that moved since `prepare` (12),
  and a frozen path again (4), then runs the base's gate with `--waive c --record <verdicts>`.
  MERGE: pushes exactly the gated tip to `main` (fast-forward, never forced), deletes the remote
  branch, removes the worktree, appends to `temp/reports/sitting-merge.jsonl`. HOLD is 13 and
  keeps everything; a gate error or a missing MERGE record is 14. A second `land` after a
  land prints `ALREADY LANDED` and exits 0.
- `tools/sitting_merge.sh` joins the gate's `FORBIDDEN_PATHS` and, with its test, the landed
  check's never-list (a held branch that edits it would decide how it lands itself).
- Debrief: section 0 names the scratch worktree and commits the Sitting's records and learner
  Memory first; section 3 is the prepare, audit, land procedure; section 4 follows with
  `git pull --ff-only`. Promises the script took over are pointer rows in
  `test_sitting_debrief_protocol.py` naming their pin in `test_sitting_merge.py`.

## Why

At the 2026-11-13 debrief BUG-80 reached `main` by hand with an open behavioural audit finding
(`logbook/2026-11-13-sitting-debrief.md`, Discussion). The prose procedure could not stop that.
The script is the one path, and it pushes only a tip the gate recorded MERGE for.

## Discussion

**Fast-forward, not `--no-ff`.** `land` pushes `<tip>:main` and nothing else, so `main` becomes
the gated tree byte for byte, and every commit `main` gains is an ancestor of the MERGE record's
tip. `tools/night_shift_landed.py` counts those as gated (the happy-path test runs it and gets
`ungated=0`), including a debrief that runs inside the night window. A `--no-ff` merge commit
would also pass that check only if it added nothing of its own, which holds only when `main` has
not moved, and in that case a fast-forward is available anyway. The night's 2i tries
`--ff-only` first for the same reason. The cost: `main` gets no merge commit naming the Item.
The jsonl line and the MERGE record name it instead.

**When `main` moves, `land` refuses.** It does not merge again. A re-merge is a tree the suite
never saw, so the operator runs `prepare` again (12). The push is not forced, so a race between
the check and the push is refused the same way.

**The Item's own `map/` file takes main's copy; any other `map/` conflict stops `prepare`.**
The plan's first rule stopped on every `map/` conflict, like the night's 2i. But both sides
routinely touch the Item's own file (its `updated:` line at least), so that rule would stop most
real debrief merges. Main is the authority for map state (status, Sitting section, skip keys are
all written there), so the old prose rule comes back as code. The file is found by the Item's
`id:` through the base's `mapkit`, never by slug. The base's copy is taken, then `mapkit gen`. The
script prints `TOOK MAIN's copy of map/<file>`, puts the same line in the gate log header for the
audit, and keeps the branch's copy as `DIR/branch-copy-<ID>.md` for the close. Another Item's file
in conflict is not mechanical and stops with 5.

## Verification

(2026-11-14, `python -m pytest tests/sim/test_sitting_merge.py -q`, mutated script): each of
four mutations (no `--waive c`, no HEAD check, no audit check, every conflict treated as
generated) fails at least one test; the restored script passes.
(2026-11-14, after the rebase onto origin/main b23152f3, `flock /tmp/jugglebot-tests.lock python -m pytest -n 2 --dist loadfile -q tests/sim/test_sitting_merge.py tests/sim/test_sitting_debrief_protocol.py tests/sim/test_night_shift_gate.py tests/sim/test_night_shift_landed.py tests/sim/test_night_shift_candidate.py tests/sim/test_night_shift_protocol.py tests/sim/test_night_shift_settings.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_logbook_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py`): 474 passed in 137.92 s.
