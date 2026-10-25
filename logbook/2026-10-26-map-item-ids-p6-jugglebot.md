---
title: "Map item IDs P6 (Jugglebot): inbox checks before the push, ID allocation, repair and assign-missing survive a malformed Item, renumber amends, presence test"
type: bugfix
date: 2026-10-26
status: resolved
phase: "map-item-ids - P6 and build-review fixes"
files_changed:
  - mapkit/ids.py
  - mapkit/items.py
  - mapkit/itemcli.py
  - mapkit/gen_docs.py
  - .claude/commands/inbox.md
  - docs/agents/map-writes.md
  - docs/agents/issue-tracker.md
  - DOCUMENTATION_GUIDE.md
  - plans/active/map-item-ids.md
  - tests/sim/test_map_items.py
  - tests/sim/test_mapkit_ids.py
  - tests/sim/test_mapkit_item_ids_cli.py
  - tests/sim/test_mapkit_docs.py
subsystem:
  - docs
tags:
  - tooling
  - project-map
---

## Summary

The Jugglebot half of the map-item-ids build review (accepted with fixes) and P6.

- **Fix 3, `/inbox` checks before the push.** The Avoid-word check had slipped after the push, so a title with an Avoid word reached `main` and painted the gate RED. `/inbox` now runs `item new ... --commit --no-push`, the scoped pytest, then the new `python -m mapkit item push <stem>`, which is `item new`'s own bounded rebase / renumber / push loop (`itemcli.push_item`), factored out and reusable on any existing local Item commit.
- **Fix 4, ID allocation, repair and assign-missing survive a malformed Item.** `ids.allocate`, `repair` and `assign_missing` load leniently (`items.load_items_lenient`; the skipped file is named on stderr) and `allocate` also scans every `map/*.md` frontmatter `id:` line as text (`ids.file_ids`), so the number of a file the loader rejects is never reused. `ids --check` stays strict (a gate). `item new` removes the file it wrote and says so when `gen_docs` refuses because another Item is malformed. Every failure after the Item's commit exists now ends with a note that a local commit is waiting and how to push (`item push`) or undo it.
- **Reviewer suggestion S1.** A renumber inside the loop amends the unpushed `map: file` commit (new subject, new ID) when it is HEAD, so no subject on the pushed branch cites a wrong ID; when other local commits sit above it, it stays a separate `map: renumber` commit (the generated-page amend follows the same HEAD test).
- **P6 docs.** `docs/project-map.md` gains an ID column (Inbox table), IDs on the attached-item and open-ticket lists and a duplicate-ID warning (`gen_docs`); `DOCUMENTATION_GUIDE.md` § 3 map-item row gains `id` (fix 5); `docs/agents/issue-tracker.md` and `map-writes.md` describe ID-taking `claim`/`resolve`, `item push` and the new failure behaviour; the plan's frontmatter and status are brought to P0-P4 complete, P5 built awaiting deploy, P6 in progress (fix 7). `CLAUDE.md`'s project-map paragraph was already right after c22f8e6e.
- **Presence test** `test_every_item_has_an_id` in `tests/sim/test_map_items.py`. The live app (fork `74826ea`) allocates (62 of 62 tasks carry `short_id`, GET only), so it is safe.

No control or hardware path touched.

## Verification

- 2026-10-26, `python -m mapkit ids --check` (before the presence test landed): no output, exit 0 (nothing duplicate, malformed, changed or missing across 62 Items).
- 2026-10-26, `pytest tests/sim/test_map_items.py tests/sim/test_mapkit_docs.py tests/sim/test_glossary_avoid_words.py tests/sim/test_plans_index.py tests/sim/test_mapkit_item_ids_cli.py tests/sim/test_mapkit_ids.py -q`: 71 passed.
- Gate 2026-10-26, `./run_tests.sh -q`: PASS (parallel 373 s rc 0, serial 22 s rc 0, total 395 s; the TypeError thread warning in the replay overview drain is pre-existing, not a failure). Re-run on the audit-fix tree 2026-10-26, `./run_tests.sh -q`: PASS, parallel 369 s rc 0, serial 22 s rc 0, total 391 s; `-q` prints no pass tally, so the counts are `pytest tests/ --collect-only -q -m` collections of the same run set: 7370 parallel + 5 serial; no failures, rc 0 both phases).
