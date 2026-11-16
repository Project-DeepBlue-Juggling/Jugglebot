---
id: BUG-109
kind: bug
title: The Night Shift history ledger re-adds old-form branch names every night, so the legacy ID test fails on main
status: inbox
created: '2026-11-17'
updated: '2026-11-16T17:51:43Z'
owners: []
priority: high
effort: xs
hardware: none
owner: decision
radius: local
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links: []
filed_by: night-shift
---

The 04:00 nightly of 2026-11-17 is RED on one test, `tests/sim/test_no_legacy_item_ids.py::test_the_tree_holds_no_old_item_id`: `docs/night-shift/history.jsonl` holds 26 old-form tokens against the 22 pinned. It fails on `main` too, so every Night Shift gate on 2026-11-17 holds at (a).

## Findings

- BUG-107 (2026-11-15) classed the ledger's branch names as historical on the premise that the first prune removed every old-form branch, so no later row could add one.
- Row 2 (the 2026-11-16 run) carries a `prune_failed` list that names four old-form remote-tracking refs again (the `night-shift/` branches of 2026-11-05, two of 2026-11-06 and 2026-11-07, whose names carry the retired ID form). Those four still have local branches in the Night Shift clone, which is presumably why their prune fails, so the same four tokens will be appended every night.
- Re-pinning the count (22 to 26) only lasts until the next row.

## Options

1. Fix the runner so a ref whose prune failed is recorded once, or not named in the ledger (a `tools/night_shift*` change, day Session).
2. Resolve the four stale local branches in the Night Shift clone (the owner decides whether their unmerged commits are kept, say as `archive/*` tags), so the prune succeeds and no further row names them; then re-pin to 26 once.
3. Let the guard pass the ledger's branch tokens without a per-file count (it is append-only and machine-written), keeping the rule that no other class may appear there.

Recommendation: 2 then a one-time re-pin, with 1 as the durable guard.
