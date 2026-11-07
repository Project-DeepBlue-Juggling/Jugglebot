---
title: "Per-kind Item IDs P3: tools/id_sweep.py classifies every JB token and refuses an unclassified one; nothing in the tree is rewritten"
type: investigation
date: 2026-11-08
status: resolved
phase: "map-improvements"
related_plan: item-id-per-kind.md
files_changed:
  - tools/id_sweep.py
  - tools/id_sweep_overrides.tsv
  - tests/sim/test_id_sweep.py
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

P3 of `plans/active/item-id-per-kind.md`: a sweeper, `tools/id_sweep.py`, that reads every tracked text file, finds every token matching the broad pattern `(?i)(?<![a-z0-9])jb(?![a-z])`, and gives each exactly one class. It refuses to run on an unclassified token or an unknown number. It has a census mode, an apply mode (not run on the real tree) and a verify mode. New files: the tool, `tests/sim/test_id_sweep.py` and `tools/id_sweep_overrides.tsv`. No Item changed ID, no other file in the tree changed, and `mapkit/legacy_ids.csv` is not created (P4 does that).

## Context

P4 rewrites every Item-ID reference in one commit. "No reference is missed" needs a census whose completeness is a test, not a grep someone ran once. The plan's class table (citation, branch, list, range, placeholder, loose, abbreviation, frozen) was written from a survey at `a058d6d1`; P3 is where it met the whole tree.

## Investigation

Unit 1a built the finder and an ordered declarative rule table (the first matching rule wins and its name goes in every CSV row). Its first census over the worktree at `ac49a644` found 6,953 broad tokens, 6,765 classified and 188 refused. The refusals were:

- 146 in nine synthetic-repo test files outside the plan's four fixture files (`test_night_shift_gate.py` 43, `test_night_shift_candidate.py` 33, `test_mapkit_similar.py` 24, `test_night_shift_deploy.py` 18, `test_night_shift_preflight.py` 14, and four small ones). They build tiny repos with `JB-5`, `JB-404`, `"jb"` directory names: parser behaviour, not citations.
- 27 in Night Shift tool code and 14 in `mapkit/` code: literals the dual-form code reads on purpose (for example `mapkit/itemcli.py:81` and `:713`, confirmed as literals).
- 1 prose line in `.claude/commands/inbox.md`.

Unit 2 added the overrides file and the per-token review. Pre-fixer numbers: 6,953 tokens, 0 unclassified; citation 3,723, abbreviation 1,752, frozen 535, fixture 466, placeholder 214, branch 81, code_literal 74, range 58, loose 45, list 5, option_text 12, per-kind 258 (counted apart). An audit (sonnet audit-reporter, one pass) then found 0 blocking and 5 behaviour findings; the fixer applied all five with failing-first tests plus a path-with-extension placeholder freeze.

## Discussion

Surfaced to the owner at stop point 3 before any write; the owner may veto any of these.

1. **A new class, `code_literal`, rather than widening `frozen` or `abbreviation`.** The 41 code tokens (74 after the fixer) are dual-form-reading literals in `mapkit/` and `tools/night_shift*`. Calling them `abbreviation` would be false (they are Item IDs), and calling them `frozen` would hide that they must change. Both are the "default to leave" the plan forbids. `code_literal` is narrow (tested), left at P4 and retired at P5 when the dual form goes. `fixture` was widened from the 4 named files to 13 synthetic-repo test files (`FIXTURE_FILES`), because the nine extra files assert parser behaviour exactly as the four do and would otherwise need per-line rules.
2. **Whole-file freezes over per-line rules for scheme-history text.** The active plan, `plans/archived/map-item-ids.md`, ADR-0015, the JB-100 Item body, five logbook entries from the two ID arcs and the sweeper's own three files are frozen as files. Per-line rules would have to be maintained against prose that describes the old scheme and is false if rewritten, and would silently go stale on any edit. The cost is that a real live citation inside a frozen file is not rewritten; frontmatter `id:` lines stay citations regardless, so the `id:` cannot be missed.
3. **`--apply` refuses any placeholder or loose token without an override row.** Otherwise such a token stays in the old form and `--verify` would not notice, because verify only checks what it classified as rewritable. The plan's "nothing defaults to leave" is applied to the overrides file too: 245 rows cover 258 tokens (156 rewritten, 91 left on purpose, 6 held for the owner).
4. **Ranges keep range form only when the new numbers are consecutive within one kind.** Otherwise the range becomes a comma list of full IDs, because `BUG-3..5` across two kinds would assert Items that do not exist. A comma list whose tail is not rewritable (`JB-1, 2, 3`) is an error, not a partial rewrite.

## Fix

The tool as built:

- Finder over `git ls-files` text files with the single broad-pattern constant (T-R1 imports it); one source for skipping (`skip_reason`, `scan_files`), counted by reason.
- Classes: citation, branch, list, range, placeholder, loose, abbreviation, frozen, fixture, code_literal, option_text, with per-kind tokens counted apart.
- `--census` writes `temp/id-sweep/census.csv`, `summary.md` and `skipped.csv`: per class by directory, with review lists (JB-90..93, 96..99, 118, 119 citations; readability-cap lines; fixtures; whole-file freezes; list and range expansions; skipped files by reason).
- `--apply --crosswalk <csv> [--overrides tsv] [--dry-run]`: crosswalk columns `old_id,new_id,kind,stem`; full-token, single-pass, idempotent, line-count-preserving. Refuses missing crosswalk numbers, stale or missing overrides, an overrides path or column count that is wrong, uncovered placeholders, non-UTF-8 files and partial comma-list tails. `--dry-run` plans in memory and writes only the report.
- `--verify`.

Control-system check: nothing swept is parsed at runtime. The UDP protocol generator descriptions feed three generated files, so P4 regenerates them and re-runs the xlang tests. `MAGIC = 0x4A42` is abbreviation and does not change.

## Verification

- Scoped: `python -m pytest -q tests/sim/test_id_sweep.py` (2026-11-08): 23 passed, 1 skipped in 14.84 s (2026-11-08 00:32, `python -m pytest -q tests/sim/test_id_sweep.py`, temp/logs/id-p3-recheck.log). Tests: T-U12 (two), T-U13, T-U14, T-R1 on a tmp repo, ordering, line-count, stale override, `id:`-line-in-frozen-file, `code_literal` narrowness, verify-after-apply, and the fixer's six.
- Real-tree T-R1 (`ID_SWEEP_REAL_TREE=1 python -m pytest -q tests/sim/test_id_sweep.py -k real`, 2026-11-08): 2 passed in 56.52 s (2026-11-08 00:32, `ID_SWEEP_REAL_TREE=1 python -m pytest -q tests/sim/test_id_sweep.py -k real`).
- Census (2026-11-08, `python tools/id_sweep.py --census`, after the fixer): 8,010 tokens classified, 0 unclassified, 316 per-kind counted apart, 64 files skipped by suffix (2026-11-08 00:52, `python tools/id_sweep.py --census`, after rebasing onto the Hand-fan commits 9f7267df; the pre-rebase tree at 00:33 gave 6,953 and 258, and the added tokens are all classified without a rewrite: the dry run is unchanged at 4,103 spans).
- Dry run (2026-11-08, provisional crosswalk, `--apply --dry-run`, after the fixer): rc 0, 4,103 spans in 667 files, 0 refused, 0 readability-cap breaks, tree unchanged (2026-11-08 00:36, `python tools/id_sweep.py --apply --dry-run --crosswalk temp/id-sweep/dry-run/provisional-crosswalk.csv --overrides tools/id_sweep_overrides.tsv`); the second-pass no-op was shown on a copy of the tree by unit 2 (`--verify` rc 0). Pre-fixer, on a copy of the tree: 0 refused, 4,098 spans in 666 files, line counts preserved, `--verify` rc 0, 0 readability-cap breaks.
- Audit: 0 blocking, 5 behaviour (unanchored `Logbook-Entry` rule; partial comma-list tail; no dry run; overrides path and columns unchecked; skipped files uncounted), all applied.
- Gate (`./run_tests.sh`, 2026-11-08): 9101 passed, 63 skipped in 551.55 s parallel and 7 passed serial, rc 0 (2026-11-08 01:05, `./run_tests.sh` on the tree rebased onto 9f7267df, temp/logs/id-p3-gate2.log; the pre-rebase tree passed 9077 at 00:40). Tier: the tool's only importer is its test (`git grep -l id_sweep -- ':!temp'`; the other four hits are the substring in `grid_sweep`), but the new test file is collected by the gate and must hold under xdist, so the default gate applies.

## Outcome

The census is complete by test and the sweep refuses what it cannot place. The tree is unchanged; P4 consumes the tool.

## Open Questions

Stop point 3 is open: the owner reads `temp/id-migration/p3-owner-report.md` (the frozen and abbreviation classes, the whole-file freezes, `code_literal`, the 6 held overrides, 8 questions). The 6 held overrides are two GLOSSARY lines with their `docs/glossary.md` mirrors and two dual-form comments in `tools/`.

P4 hand-edits recorded here: `docs/agents/map-writes.md:74` ("avoid JB-68..70"); `test_sitting_debrief_protocol.py`'s 7 regexes move with `sitting-debrief.md`; the two option-text examples get backticks; the branch globs in `.claude/commands/night-shift.md` and `sitting-debrief.md`; regenerate the UDP protocol outputs and re-run the xlang tests.
