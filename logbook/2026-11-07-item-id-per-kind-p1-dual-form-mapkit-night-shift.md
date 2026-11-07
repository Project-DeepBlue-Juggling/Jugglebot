---
title: "Per-kind Item IDs P1: mapkit and the Night Shift tools read and write both ID forms; three silent failures become loud; no Item changes ID"
type: investigation
date: 2026-11-07
status: resolved
phase: "map-improvements"
related_plan: item-id-per-kind.md
files_changed:
  - mapkit/ids.py
  - mapkit/import_issues.py
  - mapkit/itemcli.py
  - mapkit/items.py
  - mapkit/night_shift_report.py
  - mapkit/similar.py
  - tools/night_shift_candidate.sh
  - tools/night_shift_deploy.py
  - tools/night_shift_gate.py
  - tools/night_shift_preflight.py
  - tests/sim/test_mapkit_ids.py
  - tests/sim/test_mapkit_import_issues.py
  - tests/sim/test_mapkit_item_ids_cli.py
  - tests/sim/test_mapkit_similar.py
  - tests/sim/test_night_shift_candidate.py
  - tests/sim/test_night_shift_deploy.py
  - tests/sim/test_night_shift_gate.py
  - tests/sim/test_night_shift_preflight.py
  - tests/sim/test_night_shift_report.py
  - tests/sim/test_question_tickets.py
  - plans/active/item-id-per-kind.md
  - logbook/INDEX.md
  - docs/project-map.md
subsystem:
  - tools
tags:
  - project-map
  - night-shift
  - testing
---

## Summary

P1 of `plans/active/item-id-per-kind.md` is complete. `mapkit` and the four Night Shift tools (`night_shift_gate.py`, `night_shift_preflight.py`, `night_shift_candidate.sh`, `night_shift_deploy.py`) now accept `JB-<n>` and the per-kind forms (`BUG`, `FEAT`, `IDEA`, `PWAY`, `TKT`) as canonical IDs. No Item changes ID here: `git diff -- map/` is empty, `mapkit/legacy_ids.csv` does not exist, and `allocate` still returns `JB-<n>` because the allocator's form follows that file's presence.

The phase had two jobs. The first was the expansion itself: new names in `mapkit/ids.py` (`format_ref`, `canonical_ref`, `split_ref`, `parse_ref`, `typed_ref`, `mismatch`, `per_kind`), a `kind=` keyword on `allocate`, a kind check on every typed ID, and a stem-versus-ID ambiguity error in `resolve`. The names the live map app imports (`parse`, `canonical`, `format_id`, `allocate(repo_root, ref, items)`) keep their signature and meaning until P5. The second job was closing three silent failures that a flip to per-kind IDs would otherwise have hidden from the Night Shift's held-branch ledger.

## Symptoms

None observed in operation. This is a pre-emptive class closure. The Night Shift tools parse `JB-<n>` with a hard-wired pattern, so on the first per-kind ID each would have degraded without an error.

The inventory (a grep of every consumer before any edit) found three silent failures, each reproduced by a test that failed before its fix:

1. **Preflight sort sentinel.** `_id_num` returned `10**9` for an ID it could not read, so every `BUG-n` keyed identically and filing order inside a kind was lost. Failing assertion: `assert ['BUG-3', 'BU...-2', 'FEAT-2'] == ['BUG-3', 'BU...10', 'FEAT-2']  At index 1 diff: 'BUG-10' != 'BUG-2'`, plus `Failed: DID NOT RAISE <class 'mapkit.items.ItemError'>` for four unusable IDs.
2. **Candidate stacker glob.** `night_shift_candidate.sh` listed only `*-JB-*` branches, so a `night-shift/<date>-BUG-3` branch was never stacked, and a lone unparseable branch exited 0 with `0 merged`. Failing assertions: `assert 'MERGED origin/night-shift/2026-11-07-BUG-3 BUG-3' in 'MERGED ...JB-1\nMERGED ...JB-2\n...JB-190 ...candidate: 3 merged, 0 held, 2 skipped'`, and for only `night-shift/garbage`: `assert 0 == 1`.
3. **Deploy rollback regex.** `night_shift_deploy.py` found the Items a rolled-back commit touched with a `JB`-only regex, so a per-kind Item was never reopened. Failing assertions: `assert (None == ['FEAT-12'])` on `items_reopened` (the line said "reverted ... 9ac9ccfd" with no reopen), and `assert ['JB-7'] == ['JB-7', 'BUG...-4', 'TKT-31']` for `items_in`.

The gate had a fourth, loud form: five of six new gate tests failed with `["[pre] --item 'FEAT-7' is not a JB-<n> ID"]`.

## Diagnosis

The plan's § 2.3 consumer snapshot was a starting point, and the fresh inventory corrected it:

- The gate's `parse_id` sites had moved to `:289`, `:338`, `:501`, `:599` and `:662`. The `:501` site is a silent compare (`_names_item`), the worst kind.
- Missed by the snapshot: `mapkit/similar.py` (`_id_num` split on `-` and indexed `[1]`), `mapkit/itemcli.py` (`:301`, `:306`, `:465`, `:488` use `ids.parse` as an is-this-an-ID test), `mapkit/import_issues.py:112` (an `allocate` call), and `mapkit/sitting.py`.
- Wrongly listed: `file_ids` and `ref_ids` were already form-agnostic (they read the raw `^id:` value). `duplicates` and `changed_ids` were already string-keyed. Only `_max_number` (via `canonical`) and `short_id` were JB-bound.
- Fork repo (`~/map-live/jugglebot-map`, separate from this tree): it imports `ids.canonical`, `parse(allow_bare)`, `format_id`, `allocate(repo_root, ref, items)`, `duplicates` and `Item.short_id`, and keys `by_short` on an int. That is P2 scope.

## Discussion

Two of the plan's triggers hold, plus one smaller choice.

**1. `night_shift_candidate.sh` is stricter than the plan.** The plan said the script exits non-zero when `night-shift/*` branches exist and none parses. It now exits 1 on any unparseable `night-shift/*` branch (the `sitting-*` branches are excluded) and on any `item_state` helper output it does not recognise, including a crash. The parseable rest still stacks. The alternative was the plan's wording, which tolerates one garbage branch among good ones. That was rejected because a silent SKIP in the held-branch ledger is exactly the failure class P1 exists to close: a held branch that is skipped without an error is a held Item whose work the next night never sees. The cost is that a stray branch name now fails the candidate step. That cost was checked against today's remote, since a stricter rule that fails on the first night would have been a regression: all five remote `night-shift/*` branches parse (JB-89, JB-163, JB-169, JB-175, JB-190, verified 2026-11-07). The two cases are distinct in the script. An Item that is missing from the base keeps its `SKIP no Item` row, because that is a real, explained state. Any other output is an `ERROR` row with rc=1.

**2. `night_shift_deploy.py` keeps its own prefix tuple instead of importing mapkit.** `ID_PREFIXES` is a local tuple, and a test pins it to `mapkit.ids.ALL_PREFIXES`. Importing `ALL_PREFIXES` would have removed the duplication, and `night_shift_candidate.sh` does read the list from mapkit (it exits 1 if it cannot). The deploy is different: its rollback has to run when the mapkit it is undoing is broken. A rollback that fails to import the module it is reverting fails exactly when it is needed. The duplication is the price, and the pin test makes drift a failing test rather than a silent one.

**3. The preflight key keeps the ID, and an ID-less Item sorts last without raising.** The plan wrote the filing-order key as `created` then stem. The code uses `(created, prefix, number, stem)`, so `JB-<n>` order is preserved exactly (the new key and the old one give the same order over today's 95 candidates). Dropping the ID would have changed which Item the Night Shift takes first. The first draft raised on an Item with no `id:`, which the audit caught (see below): `ids --check` treats a missing ID as allowed.

**Audit.** One `/audit --unstaged` pass ran at the end of the phase (opus). It found no blocking issue, four behaviour findings and seven narrative ones. The orchestrator applied all four behaviour findings, and the narrative ones were applied directly:

- **WARNING, preflight aborted the night on an ID-less Item.** Resolved: an Item with no `id:` keys `(created, "~", 0, stem)` and does not raise. A malformed ID or a prefix that contradicts the kind still exits 2.
- **WARNING, `item new --link` and `item list --pathway` rejected a padded `jb-07`** (`parse_ref` rejects padding, legacy `parse` accepted it). Resolved: `typed_ref` is public in `mapkit/ids.py`, accepts the legacy padded and lower-case forms plus the per-kind forms, and returns None for a non-string. The gate's private copy was deleted and it imports `typed_ref`. Two tests use `jb-02` (`--link`) and `jb-01` (`--pathway`).
- **NOTE, `similar.screen` named a different duplicate** (the live app calls it at `server/store.py:642`). Resolved: `_filing_key` is `(created, split_ref(id) or ("~", 0), stem)`, so the lowest number still wins on a same-day tie and numbers of different kinds are never compared without their prefix.
- **NOTE, `night_shift_candidate.sh` masked a helper crash as a SKIP at exit 0** (pre-existing). Resolved by the stricter rule above.

Four tests failed before the fixes. The narrative findings changed docstrings and messages in `items.py`, `similar.py`, `night_shift_report.py` (the "bare JB-n" string and its three assertions changed together), the gate's `_base_meta` (it now prefixes "Item not on base:" only for "no map/ file"), the stem-versus-ID error wording, the T-U test labels, and one line of the plan (the preflight key).

## Fix

Per unit, tests first:

- **`mapkit/ids.py`**: `PREFIXES`, `KIND_OF_PREFIX`, `ALL_PREFIXES`, `LEGACY_IDS`; `format_ref`, `canonical_ref`, `split_ref`, `parse_ref` (any case, hyphen optional, never bare, never padded), `typed_ref`, `mismatch`, `mismatched`, `per_kind`; `allocate(..., kind=None)` (no csv: global `JB-<n>`, `kind` validated and ignored; csv present: max scoped to the prefix, `kind=None` raises); `resolve` raises on a stem that another Item's typed ID names and, for bare digits on a read, returns the single match or raises listing every holder; `check()` gains a `mismatch` class. `Item.short_id` returns per-kind IDs too.
- **Consumers**: `itemcli.py`, `import_issues.py`, `night_shift_report.py`, `similar.py`; `items.py` needed only text changes.
- **Night Shift**: `night_shift_preflight.py` (`_filing_key`), `night_shift_candidate.sh` (both forms, date then prefix then number, `UNPARSED` and `ERROR` rows), `night_shift_deploy.py` (`ID_PREFIXES`, `_ITEM_ID`), `night_shift_gate.py` (string compares at all five sites, prefix-versus-kind HOLD in `_find_item`, which covers the branch, base and app lookups).

## Deferred

- **D-3 `legacy()` read/refuse (P4).** `legacy("JB-149") -> "BUG-61"` and the refuse-on-write path are not implemented, because the `legacy_ids.csv` column format is undefined. Today the file is only a presence switch. P4 defines the format.
- **Fork allocator and `by_short` (P2).** The fork calls `allocate` without `kind=`, which raises in a per-kind tree, and keys `by_short` on an int. P2 must move it to `kind=` before P4 lands the csv.
- **`.claude/commands/night-shift.md` and `sitting-debrief.md` (P4/P5).** Both still run `ls-remote 'night-shift/*-JB-n'` globs, pinned by `tests/sim/test_sitting_debrief_protocol.py`, and `tools/night_shift_app_suite.sh:5` has JB-only usage text.
- **Gate `--branch` versus `--item`.** The gate never checked that `--branch`'s ID matches `--item` (true for JB before this phase too). Not added: out of scope.
- **Preflight `compare()` at the flip.** It keys on the ID, so the P4 flip reports every candidate added and removed once. Harmless.
- **`parse_ref("BUG-01")` is rejected while legacy `parse("JB-01") == 1` still works.** Frozen on purpose; `typed_ref` bridges it.

## Verification

All runs 2026-11-07, venv active, in the scratch worktree (base 83082eac plus the uncommitted P1 diff).

- **Baseline**, before any edit: `./run_tests.sh` at 83082eac: parallel **9016 passed, 61 skipped in 547.06 s**, serial **7 passed in 26.68 s**, rc=0 (`temp/logs/id-p1-baseline.log`).
- **Gate 1**, pre-fixer: `./run_tests.sh`: parallel **9051 passed, 61 skipped in 531.65 s**, serial **7 passed in 27.31 s**, rc=0 (`temp/logs/id-p1-gate.log`).
- **Gate 2**, final tree after the fixer: `./run_tests.sh` run 2026-11-07 21:31: **9054 passed, 61 skipped in 534.21 s** (parallel) and **7 passed** (serial), rc=0 (`temp/logs/id-p1-gate2.log`).
- **Unit 2** (`mapkit/ids.py`): `pytest tests/sim/test_mapkit*.py tests/sim/test_map_items.py tests/sim/test_night_shift_gate.py tests/sim/test_question_tickets.py tests/sim/test_sitting_sections.py -q`: **344 passed in 125.31 s**.
- **Unit 3** (mapkit consumers): scoped set, **245 passed in 19.80 s**; `python -m mapkit gen` output byte-identical.
- **Unit 4** (Night Shift tools): `pytest tests/sim/test_night_shift*.py tests/sim/test_mapkit*.py -q`: **481 passed in 253.24 s**. Preflight `candidates.json` byte-identical to the base's over today's 95 Items.
- **Fixer**: `pytest tests/sim/test_night_shift*.py tests/sim/test_mapkit*.py tests/sim/test_question_tickets.py tests/sim/test_plans_index.py -q`: **515 passed in 254 s**.
- **Fork T-I0**, run from `~/map-live/jugglebot-map` with `MAPKIT_SRC` pointing at the worktree. Before the fixer: `python -m pytest -q`: **103 passed in 104.96 s** and `bash scripts/contract_check.sh` rc=0 (`temp/logs/id-p1-fork-ti0.log`, `id-p1-contract-check.log`). After the fixer: **103 passed in 89.54 s**, `contract_check` rc=0, vitest `contract.live` 1 passed (`id-p1-fork-ti0-2.log`, `id-p1-contract-check-2.log`).
- **Audit**: one `/audit --unstaged` pass (opus), findings above. It also confirmed that `git diff -- map/` is empty, that `parse` and `canonical` are byte-identical to the base, and that no `JB_OP_*` or UDP-preamble `JB` text changed.

Tier 1 (`mapkit/` and `tools/night_shift*`), so the default gate is the commit gate.

## Outcome

Resolved. The expansion is in and gated, no Item changed ID, and the live app's contract is intact (T-I0, both sides). Next is P2 (the fork). The preconditions it inherits are the three deferred items above that name it.
