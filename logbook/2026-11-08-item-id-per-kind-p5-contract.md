---
title: "Per-kind Item IDs P5: the old global form retired from code; one enforcement point, a classifier-backed guard in the default gate, the format stated in the docs"
type: investigation
date: 2026-11-08
status: resolved
phase: "map-improvements"
related_plan: item-id-per-kind.md
files_changed:
  - CLAUDE.md
  - DOCUMENTATION_GUIDE.md
  - GLOSSARY.md
  - docs/agents/map-writes.md
  - docs/glossary.md
  - mapkit/gen_docs.py
  - mapkit/ids.py
  - mapkit/itemcli.py
  - mapkit/items.py
  - mapkit/night_shift_report.py
  - mapkit/similar.py
  - mapkit/tickets.py
  - tests/sim/no_legacy_item_ids.tsv
  - tests/sim/test_mapkit_ids.py
  - tests/sim/test_mapkit_item_ids_cli.py
  - tests/sim/test_mapkit_similar.py
  - tests/sim/test_night_shift_candidate.py
  - tests/sim/test_night_shift_deploy.py
  - tests/sim/test_night_shift_gate.py
  - tests/sim/test_night_shift_preflight.py
  - tests/sim/test_night_shift_report.py
  - tests/sim/test_night_shift_settings.py
  - tests/sim/test_no_legacy_item_ids.py
  - tools/id_sweep.py
  - tools/night_shift_candidate.sh
  - tools/night_shift_deploy.py
  - tools/night_shift_gate.py
  - tools/night_shift_preflight.py
  - plans/active/item-id-per-kind.md
  - logbook/INDEX.md
subsystem:
  - tools
tags:
  - project-map
  - night-shift
  - testing
---

## Summary

P5 of `plans/active/item-id-per-kind.md`: the contract phase. The old form of Item ID is now refused, not merely unused. `mapkit/ids.py` is the one enforcement point and names no old prefix; the callers (`mapkit/itemcli.py`, `items`, `similar`, `gen_docs`, `tickets`, `night_shift_report.py`) and the Night Shift tools go through it. `tests/sim/test_no_legacy_item_ids.py` is a classifier-backed guard in the default gate, pinned per file by `tests/sim/no_legacy_item_ids.tsv`. `DOCUMENTATION_GUIDE.md` and the GLOSSARY state the format. Nothing under `ros_ws/`, `controller/`, `teensy_link/`, `config/` or firmware changed, so no robot behaviour can change.

## Context

P4 renumbered every Item in one commit (`logbook/2026-11-08-item-id-per-kind-p4-cutover.md`) and left the old form readable behind 28 legacy-marked tests, 81 `code_literal` tokens and `typed_ref`'s old-form branch. P5 removes that dual form, adds the guard the plan promised (a held branch or a habit that reintroduces the old form must fail at commit), and writes the format into the normative docs. Owner answers Q2 (any spelling translates on a read, writes refuse), Q3 (the history layer lives in the guard) and Q4 (restart order for the app) bound the design.

## Discussion

1. **Stop point 5 was replaced by the owner (2026-11-08 about 14:00).** The plan wanted one clean nightly and one clean Night Shift on the new form first, so that P4 stayed one revert away while it was young. The owner had time today, not tomorrow. Same-day checks stood in: `./run_tests.sh --full` on the flip tree 12:29-12:43 (9196 passed, 55 skipped, serial 10, rc 0); `tools/night_shift.sh --dry-run` at 13:15, DRY_OK (99 candidates classified under the new IDs, held fork branches BUG-60 and BUG-83 found); a never-pushed write-path probe (`item new` BUG-88 and TKT-53, set-status, rate, close, old-ID writes refused with exit 2, merge gate HOLD then MERGE); and the owner's phone filing, which became IDEA-19 on main (93fe67ef, deleted by 04b09ca0). Cost accepted: tonight's 04:46 Night Shift is the first real write-path night and runs on P4 and P5 together. Tomorrow's report `docs/night-shift/2026-11-09.md` is the joint check. P5 is separate commits from P4, so a P5 defect reverts alone.

2. **Why a classifier-backed guard and not a grep.** A `JB-\d+` grep misses list tails like "JB-102, 166, 167" and the lower-case and padded forms (jb149, JB-0149, jb-12). A case-insensitive grep hits JB_OP_*, JB_TRAJ_*, jb_lag_fold and the robot's name. The guard reuses `tools/id_sweep.py`'s rule table unchanged (one classifier for sweep, census and guard) and judges every token; an unclassified token fails. A `git grep -I -l -i jb` prefilter plus the sweeper's own skip rule makes the real-tree scan 1.4 s instead of the census's 32 s, so it can be a default-gate test.

3. **Why the old scheme's history is allowlisted by class and pinned count, not ignored (owner Q3).** Owner Q2 left old-scheme placeholders and citations in `logbook/`, `map/` bodies, `plans/archived/` and the plans INDEX files; owner Q1 left the jb21_, jb73 and jb75 test identifiers. A per-directory pass would let a NEW old ID in a logbook entry through silently. So a guard-side rule layer (four rules: blame-ignore subject, glossary Avoid line, Q1 identifiers, Q2 historical-record) names each class, and `tests/sim/no_legacy_item_ids.tsv` pins frozen and historical counts per file: a new file or one more token fails, naming the lines. Re-pinning (`python tests/sim/test_no_legacy_item_ids.py --pin`) is a reviewable diff. The layer sits in the guard, not the sweeper's table (owner Q3): the sweeper and its tests describe P4 and fold into the guard's module at P6. An Item's own frontmatter id line is never historical.

4. **Why `legacy()` outlives the form it reads (D-3), and why it now validates.** Commit messages, bookmarks and old launch logs cite old IDs forever, so the frozen crosswalk is permanent and its old_id column is the only knowledge of the old form left (`ids.py` names no old prefix; `legacy_ref` is generic: letters, optional hyphen, padded number, upper-cased, looked up). Owner Q2: any spelling translates on a read, because an old lower-case link silently resolving to nothing is the failure; writes refuse, naming the new ID. A malformed crosswalk now raises `ItemError` naming file and line (P4 audit B3), and the CLI reports it as one error line (P5 audit finding 1).

5. **`make_crosswalk` and `--make-crosswalk` were deleted rather than kept refusing.** Once `ids.py` forgets the old form it cannot recognise a legacy tree, so the function could only ever refuse. The crosswalk is frozen forever and its generator stays in git at 38058788. Stated to the owner at the start, no objection.

6. **The fork and the two restarts (owner Q4).** The app imports mapkit from its data clone at start, and the fork imported `typed_ref` at startup (`server/config.py`) and used `ids.PREFIX`, so a P5 mapkit would have stopped the app starting. Fork 13e1c0c works on BOTH mapkits (`legacy_ref` when present, else P4's `typed_ref`; the fallback goes at P6). It was deployed FIRST while the data clone was still P4, so a deploy rollback lands on 6ba8ae4 plus P4 (which works). The P5 mapkit arrives at a second, plain restart after the push.

7. **Owner-interactive GLOSSARY round (grilling and domain-modeling, owner present).** Item ID's temporary sentence was deleted and `JB-<n>` (the retired global form) joined its Avoid line. The read-side translation and format rules were kept OUT of the glossary (they live in `DOCUMENTATION_GUIDE.md` 2.10). The Night Shift entry's branch pattern became `night-shift/<date>-<ID>` (Q7).

## Fix

By part of `git show --stat HEAD` (28 files, 1009 insertions, 580 deletions):

- **Enforcement point.** `mapkit/ids.py` (net -254 lines of dual-form code): no old prefix, int-based `parse`, `canonical`, `format_id` and `make_crosswalk` gone; `legacy_ref` and a validating `legacy()` stay.
- **Callers.** `mapkit/itemcli.py`, `items.py`, `similar.py`, `gen_docs.py`, `tickets.py` use the per-kind API. `mapkit/night_shift_report.py` `_BARE_ID` is pinned to `ALL_PREFIXES` by a new test.
- **Night Shift tools.** `tools/night_shift_gate.py`, `night_shift_candidate.sh`, `night_shift_deploy.py`, `night_shift_preflight.py`.
- **Sweeper.** `tools/id_sweep.py` gets a local old-form parser (`OLD_PREFIX`, `_old_number`) now that `ids.py` has none.
- **Guard and pins.** `tests/sim/test_no_legacy_item_ids.py` (13 tests) and `tests/sim/no_legacy_item_ids.tsv`.
- **Tests.** 28 legacy-marked tests deleted or converted; synthetic crosswalks use a FICTIONAL OLD-n prefix; real-data refusals read old IDs from the real crosswalk at run time.
- **Docs.** `CLAUDE.md`, `DOCUMENTATION_GUIDE.md`, `GLOSSARY.md`, `docs/glossary.md`, `docs/agents/map-writes.md`.
- **Also fixed.** `items --pathway <unknown ID>` raised a traceback (pre-existing); fixed, with a regression test confirmed failing first.

## Verification

- Baseline `./run_tests.sh` on origin/main 04b09ca0 (clean worktree), 2026-11-08: 9198 passed, 54 skipped, serial 7 passed, rc 0.
- Scoped `python -m pytest -n 4` over the 15 ID and Night Shift test files, 2026-11-08: 497 passed, 1 skipped (the only failure was the real-tree guard before its pins existed).
- Guard `python -m pytest -q tests/sim/test_no_legacy_item_ids.py`, 2026-11-08: 13 passed in 1.24 s; pins 147 frozen files and 28 historical files (the close-out re-pins after this entry and the plan are written).
- Doc and glossary tests (`test_logbook_front_matter test_plans_index test_docs_links test_mapkit_glossary test_glossary_avoid_words test_night_shift_gate test_sitting_debrief_protocol`), 2026-11-08: 213 passed; `python -m mapkit ids --check` rc 0.
- Audit: one sonnet audit-reporter pass (`temp/id-migration/p5-audit-report.md`): 0 blocking, 1 NOTE (a bad crosswalk gave a raw traceback on write commands), fixed with a test confirmed failing first.
- Fork 13e1c0c, 2026-11-08: `MAPKIT_SRC=<P5 tree> python -m pytest -q` 118 passed; the same against the P4 tree 118 passed; `bash scripts/contract_check.sh` rc 0; `npx vitest run` 340 passed. Deployed by `tools/night_shift_deploy.py` 6ba8ae4 to 13e1c0c, health 200. T-I2 curls on P4 mapkit (`temp/id-migration/p5-ti2.sh`): BUG-14 and bug14 200; the old ID of BUG-14 in four spellings 200 with the same uuid; PATCH with the old ID 422 "... is now BUG-14"; unknown 404.
- Final gate (`./run_tests.sh --full`, run 2026-11-08 on this tree, after rebasing onto a peer's four commits, FEAT-46 FW 42 among them): **9249 passed, 53 skipped, serial 10 passed, rc 0** in 589 s (`temp/logs/id-p5-gate2.log`). The pre-rebase run gave 9229 passed, 54 skipped, serial 10. The rebase also showed the re-pin case live: the peer's FEAT-46 entry lists an old filename with an old number in it (a frozen path), pinned at 1.

## Outcome

The old form is refused, not merely unused. Nothing under `ros_ws/`, `controller/`, `teensy_link/`, `config/` or firmware changed, so no robot behaviour can change. Note for the owner: re-pinning is needed when a NEW doc names a file or stem with `jb` in it (a Night Shift entry that does so holds at its gate until re-pinned).

## Open Questions

- Restart 2 DONE 2026-11-08 15:23: data clone at a467a0f6 (P5), `systemctl --user restart jugglebot-map.service`, health 200 in 4 s, NRestarts 0. T-I2 on the P5 mapkit (`bash temp/id-migration/p5-ti2.sh`): BUG-14 and bug14 200; the old ID of BUG-14 in four spellings (canonical, lower case, unhyphenated, zero-padded) 200 with the same uuid; FEAT-34 and its old ID 200; PATCH with the old ID 422 naming BUG-14; unknown 404; all identical to the run on the P4 mapkit. The p2-capture of 20 endpoints gives the same codes as before the fork deploy.
- Tonight's Night Shift is the joint P4 and P5 check (`docs/night-shift/2026-11-09.md`).
- P6 list: see `plans/active/item-id-per-kind.md` P6, "From P5".
