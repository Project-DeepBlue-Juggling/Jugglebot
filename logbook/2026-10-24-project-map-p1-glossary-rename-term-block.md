---
title: "project-map P1: CONTEXT.md becomes GLOSSARY.md, 23 entries land, the CLAUDE.md term block is generated and the Avoid-word test is live"
type: feature
date: 2026-10-24
status: resolved
phase: "project-map - P1 glossary rename and term block"
related_plan: project-map.md
files_changed:
  - GLOSSARY.md
  - CONTEXT.md
  - CLAUDE.md
  - DOCUMENTATION_GUIDE.md
  - docs/agents/domain.md
  - mapkit/gen_glossary.py
  - tests/sim/test_mapkit_glossary.py
  - tests/sim/test_glossary_avoid_words.py
  - tests/sim/avoid_words_checked.txt
  - plans/active/project-map.md
subsystem:
  - docs
tags:
  - docs
  - planning
  - tooling
---

## What changed

`git mv CONTEXT.md GLOSSARY.md` (ticket #28: the installed `domain-modeling` skill writes `GLOSSARY.md`, so without the rename a skill-driven session would create a second glossary) with a three-line `CONTEXT.md` stub so historical references resolve. The 13 entries of #29 and the nine of #25, plus `Ticket` from #34, land under their clusters; `### Tooling` and `### Project map` are new. `python -m mapkit gen` now also writes the term-name list between `<!-- glossary-terms:start -->` / `<!-- glossary-terms:end -->` in `CLAUDE.md`, under the hand-written rule sentence, pinned by a currency test with the 1,200-character ceiling. `tests/sim/test_glossary_avoid_words.py` checks the phrases in `tests/sim/avoid_words_checked.txt` over `plans/active`, `plans/parked`, logbook entries dated on or after 2026-09-21 and `map/`, exempting backticked and quoted spans and `<!-- avoid-ok -->` lines (GitHub #16's standing ask, closed by #28). The live references in `CLAUDE.md`, `DOCUMENTATION_GUIDE.md` (§§ 1, 2.9, 4, 5, 6) and `docs/agents/domain.md` (single-context wording, no more CONTEXT-MAP) now name `GLOSSARY.md`.

Making the Avoid-word test pass on the tree swept 377 lines in 74 files (71 logbook entries and 3 plans): `runsheet` -> runbook, `session limits` -> Sitting limits, `can-hub` -> can-bridge, `self-toss` -> self-throw, `toss` -> throw, two `guard E-STOP` lines reworded, and `reload` backticked where it names the `reload` pattern, skill or button. The sweep touched logbook entries dated 2026-09-21 onward; #28 set that scope deliberately, so it overrides the CLAUDE.md line about never rewriting historical entries for this window only. Earlier entries are untouched.

## Verification

- 2026-10-24, `pytest tests/sim/test_mapkit_*.py tests/sim/test_glossary_avoid_words.py tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/sim/test_docs_links.py -q -p no:cacheprovider`: **114 passed in 4.97 s**.
- 2026-10-24, `python -m mapkit gen` then `python -m mapkit gen --check`: exit 0 (idempotent); the term block is 963 characters.
- 2026-10-24, `grep -rn CONTEXT.md` outside `logbook/`, `plans/archived/` and the stub: only the two rename sentences (Guide § 2.9, `docs/agents/domain.md`) and the project-map plan's own text, all naming the old file as history.
- 2026-10-24, `/audit --staged` at phase end (audit-reporter, Sonnet): one behaviour-affecting finding (this plan's own notes tripped the new Avoid-word test on "page reload") and seven narrative ones (two frontmatter filenames and two historical `.action` paths damaged by the sweep, nested backticks in `bb/reload`, placeholder lines, sweep counts, a stale "22 entries"); all applied before the gate.
- 2026-10-24 01:58-02:04 AEDT, `./run_tests.sh --full` in the worktree (log `temp/logs/gate_p1_project_map_20261024.log`): **7084 passed, 9 skipped in 317.47 s; serial tail 7 passed in 27.76 s**; exit 0. After this line was written (markdown only), `pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/sim/test_glossary_avoid_words.py -q` re-run: **37 passed in 1.37s**.

## Outcome

P1 complete. Next: P2 (the `map/` seed, `/inbox`, `/work-inbox`, the gated offers, agent and doc edits, the first committed `docs/project-map.md` and `docs/glossary.md`).
