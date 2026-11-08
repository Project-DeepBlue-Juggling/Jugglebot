---
id: BUG-45
kind: bug
title: 'test_references_resolve refuses a question ticket''s bug parent: main red after the first six question tickets'
status: complete
created: '2026-11-01'
updated: '2026-10-31T19:00:27Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links:
- logbook/2026-11-01-jb129-question-ticket-parent-test.md
filed_by: night-shift
---

`tests/sim/test_map_items.py::test_references_resolve` requires every Item's `parent` to be a pathway slug. The question-ticket design (Night Shift P2, 2026-10-31) hangs a `type: question` ticket off the bug, feature or idea it blocks, and `mapkit/itemcli.py` refuses a pathway or ticket parent there. The first six question tickets filed on 2026-11-01 (TKT-19 to TKT-24) were the first to exercise the rule on `main`, which went red on that test.

## Wanted

- The test accepts a question ticket whose parent is a bug, feature or idea stem (the CLI stores the stem) and still refuses an unresolved or wrong-kind parent.
- Every other Item's parent stays a pathway slug.

## Fix

The test change landed as logbook/2026-11-01-jb129-question-ticket-parent-test.md; nothing in `mapkit/` changed (the CLI already enforces the rule at filing).

## Second rule (2026-11-01)

The same six tickets also tripped `tests/sim/test_glossary_avoid_words.py` on a `parent:` line holding a stem with an Avoid-word in it (`bug-sim-reload-gate-limit-jerk-regression`). Frontmatter reference lines are now exempt, body text is not.
