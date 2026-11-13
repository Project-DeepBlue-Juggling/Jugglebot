---
title: "Per-kind Item IDs P6: auto-memory and the map-app fork rewritten through the crosswalk, the sweeper retired with its classifier kept as the guard's module, FEAT-34 closed"
type: investigation
date: 2026-11-09
status: resolved
phase: "map-improvements"
related_plan: item-id-per-kind.md
files_changed:
  - tests/sim/legacy_id_rules.py
  - tests/sim/test_no_legacy_item_ids.py
  - tests/sim/no_legacy_item_ids.tsv
  - tests/sim/test_id_sweep.py
  - tools/id_sweep.py
  - tools/id_sweep_overrides.tsv
  - tools/probes/id_cutover/README.md
  - map/feature-update-item-naming-convention-based-on-type-eg-pathways-can.md
  - docs/project-map.md
  - plans/active/item-id-per-kind.md
  - logbook/INDEX.md
subsystem:
  - tools
tags:
  - project-map
  - testing
---

## Summary

P6 of `plans/active/item-id-per-kind.md`, the final phase. It clears what the guard cannot see and retires the migration tooling. Auto-memory and the map-app fork no longer cite Items by the old form, `tools/id_sweep.py` is gone with its classifier kept as `tests/sim/legacy_id_rules.py` (the guard is its only reader), and FEAT-34 is closed. The plan is archived in the commit after this one.

## Context

P5 made the tracked tree refuse the old form. Two places sit outside that guard: auto-memory, which every Session reads as current guidance and which is not a git repo, and the fork `jugglebot-map`, a separate repository. The guard also still imported the sweeper, so the sweeper could not go until the classifier had a new home.

The 2026-11-09 Night Shift was the first real write-path night on P4 and P5. It ended `RUN_LEFT_DIRTY` because its unit was killed mid BUG-80 (work saved on `night-shift/2026-11-09-BUG-80`), and it wrote no report. Neither cause is ID-related: the 14 `map:` commits it made use per-kind IDs. The 04:00 nightly was RED on two failures in the map body of IDEA-19 (a heading cap and an Avoid word), also not ID-related.

## Discussion

1. **The memory sweep ran the sweeper on a scratch repo with a synthetic `map/`.** The sweeper takes its known numbers from `id:` lines in `map/`, which memory does not have, so it refused 382 of the 424 tokens. Editing the sweeper to read the crosswalk would have changed a tool that was about to be deleted and had just done the repo's own rewrite. Instead the scratch repo held one stub per crosswalk row, which made the known-number set exactly the frozen crosswalk, and the stubs were never copied back. A hand regex over the old pattern was ruled out: it cannot place the list and range forms, or tell a token that means Jugglebot from an Item ID.

2. **`project_item_id_per_kind.md` was left out of the sweep.** It describes the old scheme, so a sweep would have produced "(FEAT-34, now FEAT-34)". It is tombstoned at archival instead.

3. **Placeholders were judged one by one, not by rule.** The 15 placeholder tokens split into text about the old scheme, kept as written (the 2026-10-26 design notes, the named 2026-11-01 branches), and live guidance, rewritten (the `item rate` command, which would now be refused, the branch pattern and the Runbook heading).

4. **The classifier lives beside the guard, not in `mapkit/` (owner).** `mapkit` was purged of the old form at P5, and a test-side module cannot be imported by robot code. The module was generated from the sweeper's own lines rather than retyped, so the rule table is the same text apart from the two deliberate changes: the `sweeper-self` rule went with its target files, and `SCHEME_FILES` now matches the plan under `plans/archived/` as well as `plans/active/`.

5. **The frontend keeps `JB` in `shortId.ts`'s `PREFIXES`.** It looks like residue but is D-3 behaviour: it is what lets a typed or linked old ID reach the server to be translated. Only comments and fixtures changed around it.

6. **The classifier's tests became one planted token per rule.** Porting the old tests one by one would have kept the coverage they happened to have. A table with a row for every rule, plus a test that the table's rule names equal the rule table's, turns "no rule loses its test" into a failing assertion. Tests of the census, apply, verify and overrides machinery went with the sweeper.

## Fix

**Auto-memory.** 393 edits in 11 files through the crosswalk with a reviewed override set; backup `temp/id-migration/memory-backup-2026-11-09.tgz`.

**Fork (`a7f2e9e`).** Old-form citations, format descriptions, examples and fixtures moved to per-kind IDs in 39 files, and `_legacy_new`'s pre-P5 `typed_ref` fallback dropped. Tests that use an old ID on purpose are kept.

**Classifier.** `tests/sim/legacy_id_rules.py` new, in `SELF`; `tools/id_sweep.py`, `tools/id_sweep_overrides.tsv` and `tests/sim/test_id_sweep.py` deleted; guard rewired and the classifier tests ported; `tools/probes/id_cutover/README.md` says the proofs ran against P4's tree and names the commit to check out (owner: they stay as evidence).

**FEAT-34** closes in this commit.

## Verification

- Memory T-R8 (`grep -rE 'JB-[0-9]'` over the memory directory, 2026-11-09): 381 tokens in 12 files before, 3 tokens in 1 file after, all three in `project_item_id_per_kind.md` explaining the crosswalk. Every one of the 346 citation edits re-checked against `mapkit/legacy_ids.csv`: 0 mismatches.
- Fork T-R8 (`git grep -cE 'JB-[0-9]'`, 2026-11-09): 201 tokens in 33 files before, 24 in 3 files after (`tests/test_ids.py` 18, `frontend/src/lib/shortId.test.ts` 5, `server/store.py` 1), each a deliberate old-ID test or the comment explaining translation. The non-test diff was checked to be comments only: production Python AST-identical apart from docstrings and the intended `store.py` change.
- Fork gate (2026-11-09, against this worktree): `MAPKIT_SRC=<worktree> python -m pytest -q` **118 passed**, rc 0; `bash scripts/contract_check.sh` rc 0; `npx vitest run` **340 passed, 1 skipped**, rc 0.
- Deploy (`tools/night_shift_deploy.py --date 2026-11-09`): DEPLOYED `13e1c0c1 -> a7f2e9ee`, healthy, the one restart the owner approved. `p5-ti2.sh` answers identical to P5's with the uuids masked (health 200, old IDs translate, PATCH of an old ID 422 naming the new one, unknown 404); `p2-capture.sh` before and after: 20 of 20 codes equal.
- Classifier differential (2026-11-09, probe): the old module and the new one over every tracked token: 3888 tokens, 1119 differences, every one in the three deleted files.
- Mutation (2026-11-09, probe): removing each of the 31 classifier rules and 4 guard rules in turn fails the planted-token test every time, 0 survivors.
- Pins (`python tests/sim/test_no_legacy_item_ids.py --pin`, 2026-11-09): the removal of exactly the three deleted files' rows, frozen total 2000 to 881 before this entry's own tokens.
- Full gate (`./run_tests.sh --full`, 2026-11-09, on this tree rebased on `91db1be8`): **9251 passed, 51 skipped**, serial **10 passed**, RESULT: PASS, 625 s (`temp/logs/id-p6-gate.log`).

## Outcome

The per-kind ID work is finished. Old IDs still translate on a read and are refused on a write everywhere, and the old form is now confined to the record of its day and the tests that prove that behaviour. The guard depends on nothing in `tools/`. Scope that was not done: the frozen cutover proofs cannot run on today's tree by design.

## Open Questions

None for this plan. Two children of PWAY-7 stay open and neither was blocking: BUG-17 and FEAT-42.
