---
id: TKT-79
kind: ticket
title: 'BUG-109: resolve the four stale local branches and re-pin, fix the runner''s ledger, or let the guard pass the ledger''s branch names?'
status: complete
created: '2026-11-17'
updated: '2026-11-17T22:52:53Z'
owners: []
priority: high
effort: xs
hardware: none
owner: decision
radius: local
area: tests-tooling
blocked_by: []
links: []
parent: bug-the-night-shift-history-ledger-re-adds-old-form-branch-names
type: question
---

The 04:00 nightly is RED on `test_no_legacy_item_ids` because the runner's ledger row for 2026-11-16 names four old-form branches in its `prune_failed` list, and it will do so again every night. Until it is fixed, every Night Shift gate holds at (a) and nothing merges.

## Options

1. **Resolve the four stale local branches, then re-pin once:** you decide whether their unmerged commits are kept (an `archive/*` tag, say) or dropped; the prune then succeeds and a one-time re-pin from 22 to 26 makes the test green.
2. **Fix the runner:** record a failed prune once rather than every night, or leave branch names out of the ledger (a `tools/night_shift*` change, so a day Session).
3. **Let the guard pass the ledger's branch names without a count:** the ledger is append-only and machine-written; any other old-form class there still fails.

## What each answer unblocks

- 1 or 3: the night can build the re-pin or the guard rule, and every later gate stops holding at (a).
- 2: a day Session.

Recommendation: 1 now, then 2 so no other prune failure can do the same.

## Answer

If the local branches are fully merged, just delete them. Otherwise, let me know what's outstanding from them and we can take them on a case-by-case basis. Then, I'd like to discuss which of the Option 2 solutions you recommend. Give pros and cons of each
