---
title: "BUG-45: test_references_resolve accepts a question ticket's bug, feature or idea parent (the 2026-10-31 question-ticket design) and the Avoid-word test exempts frontmatter reference lines (a parent stem is an identifier); main went red on both after the Night Shift filed its first six question tickets"
type: bugfix
date: 2026-11-01
status: complete
phase: "night-shift (first live night)"
related_plan:
  - night-shift.md
files_changed:
  - tests/sim/test_map_items.py
  - tests/sim/test_glossary_avoid_words.py
  - logbook/INDEX.md
  - map/bug-test-references-resolve-refuses-a-question-ticket-s-bug.md
  - docs/project-map.md
subsystem:
  - map
tags:
  - night-shift
  - tests
---

# BUG-45: the ticket-parent rule in test_references_resolve

**What.** `tests/sim/test_map_items.py::test_references_resolve` required every Item's `parent` to be a pathway slug. The question-ticket design (Night Shift P2, 2026-10-31) hangs a `type: question` ticket off the bug, feature or idea it blocks, stored as the stem, and `mapkit/itemcli.py` refuses a pathway or ticket parent for that type. The dry run never filed a ticket, so the first six question tickets (TKT-19 to TKT-24, filed 2026-11-01 05:57) were the first to meet the test on `main`, which went red on it until this fix merged through the gate.

**Fix.** The test now branches on the Item: a ticket with `type: question` must name a bug, feature or idea stem (and still fails on an unresolved or wrong-kind parent); every other parent must be a pathway slug. `mapkit/` is untouched (the CLI already enforces the rule at filing; the gate freezes that path anyway).

**Second rule, same event.** `tests/sim/test_glossary_avoid_words.py` scanned frontmatter like body text, so the ticket TKT-24 failed on its own `parent:` line, `bug-sim-reload-gate-limit-jerk-regression`, the bug's stem (a filename-like identifier the test already exempts when it carries a `/` or an extension). Frontmatter reference lines (`parent:`, `promoted_to:`, a bare `- <stem>` under `blocked_by:` / `links:`) are now exempt inside the frontmatter block only; body text is scanned as before, and a stem in body prose still fails (the `--reload` and `reload` ctx mentions in TKT-21 / TKT-24 were backticked on `main`). The self-test `test_frontmatter_reference_lines_are_identifiers` pins the exemption, the single-bare-word case, a body bullet, a body rule and a file without frontmatter.

**Why its own branch.** Night Shift invariant 3 allows only report, logbook and `map:` commits straight on `main`, so the one-test fix went through the merge gate as its own Item instead of riding the `map:` commit that split the tickets' intro paragraphs.

## Verification

- 2026-11-01, `pytest tests/sim/test_glossary_avoid_words.py -q -p no:randomly`: **4 passed in 0.98 s** (the new self-test included); the scanner from `origin/main` run on a two-line probe (a `parent:` stem line, a body `reload`) reports both lines, the new one reports only the body line.
- 2026-11-01, `pytest tests/sim/test_map_items.py tests/sim/test_question_tickets.py tests/sim/test_mapkit_docs.py -q -p no:randomly`: **37 passed in 1.66 s** with the fix; with the fix stashed, `-k references_resolve`: **1 failed, 10 deselected in 0.37 s** (the six tickets listed).
