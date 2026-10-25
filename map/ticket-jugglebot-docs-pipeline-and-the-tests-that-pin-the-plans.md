---
id: JB-7
kind: ticket
title: Jugglebot docs pipeline and the tests that pin the plans board and logbook
type: research
parent: project-map
blocked_by: []
owners: []
rank: 2
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/23
created: '2026-10-23'
updated: '2026-10-23T14:37:58Z'
priority: medium
---

Part of #21

## Question

What exactly would the generated INDEX files, the generated mkdocs map page and the new parser package have to satisfy? Read tests/sim/test_plans_index.py, tests/sim/test_logbook_front_matter.py, tests/sim/test_logbook_search.py and sim/analysis/logbook_search.py (the existing frontmatter parser: reusable?), mkdocs.yml, .github/workflows/docs.yml, docs/index.md, DOCUMENTATION_GUIDE.md's layer list, run_tests.sh and the marker definitions in pyproject.toml. Also read the frontmatter of every file in plans/active, plans/parked and five archived plans to list every field in use and every phase-heading style.

Output: (a) each existing assertion in one line and whether a generator-based board keeps, changes or retires it; (b) how a generated page enters the mkdocs nav and the Pages build; (c) the exact set of frontmatter fields and phase-heading variants the parser must accept; (d) whether logbook_search.py's parser can seed the shared package. Findings to research/jugglebot-docs-pipeline.md.

## Answer

Full findings: research/jugglebot-docs-pipeline.md.

- Only tests/sim/test_plans_index.py is affected by INDEX generation. The forward/reverse row tests and the active/parked row-shape tests become "generated output is current" plus frontmatter-field checks. The bare-filename, one-board-per-name, parked-note and archived-date tests keep as-is. The reverse test scans the whole INDEX text, so hand prose naming bare *.md files must go or be path-qualified.
- Frontmatter has no source today for the active "Scope" prose, the parked date / why / unpark gate, or "last touched". Needs new fields. `branch:` already exists on 3 archived HANDOFF files. 21 of 69 plans have no phase headings; phase headings come in about 10 styles (Phase N, P1, R0, W4a, Stage 1a, bold status lines, summary tables), so `phases:` must be optional with one explicit extraction rule.
- logbook_search._parse_frontmatter is hand-rolled (no comments, no maps, all strings) and cannot parse a `phases:` list of maps. Do not seed from it; build a PyYAML-based shared parser (PyYAML is already a repo dependency). Gotchas: dates become datetime.date, 3 archived plans are invalid YAML, related_plan is str or list.
- mkdocs: explicit nav, no plugins. docs.yml runs `mkdocs build --strict` with only mkdocs-material, triggered by docs/** and mkdocs.yml. plans/ and logbook/ are not published. Cleanest: generator writes committed docs/*.md listed in nav, with a current-ness test. Links out of docs/ must be absolute GitHub URLs (tests/sim/test_docs_links.py).
- New tests go in tests/sim/ (or a new tests dir), unmarked, in the default parallel xdist phase; no serial/nightly. Avoid collection-time parametrize over directory listings.
- Discrepancy: CONTEXT.md (245 lines) is already on main, contradicting map.md.

## Answer

## Answer

Full findings: research/jugglebot-docs-pipeline.md.

- Only tests/sim/test_plans_index.py is affected by INDEX generation. The forward/reverse row tests and the active/parked row-shape tests become "generated output is current" plus frontmatter-field checks. The bare-filename, one-board-per-name, parked-note and archived-date tests keep as-is. The reverse test scans the whole INDEX text, so hand prose naming bare *.md files must go or be path-qualified.
- Frontmatter has no source today for the active "Scope" prose, the parked date / why / unpark gate, or "last touched". Needs new fields. `branch:` already exists on 3 archived HANDOFF files. 21 of 69 plans have no phase headings; phase headings come in about 10 styles (Phase N, P1, R0, W4a, Stage 1a, bold status lines, summary tables), so `phases:` must be optional with one explicit extraction rule.
- logbook_search._parse_frontmatter is hand-rolled (no comments, no maps, all strings) and cannot parse a `phases:` list of maps. Do not seed from it; build a PyYAML-based shared parser (PyYAML is already a repo dependency). Gotchas: dates become datetime.date, 3 archived plans are invalid YAML, related_plan is str or list.
- mkdocs: explicit nav, no plugins. docs.yml runs `mkdocs build --strict` with only mkdocs-material, triggered by docs/** and mkdocs.yml. plans/ and logbook/ are not published. Cleanest: generator writes committed docs/*.md listed in nav, with a current-ness test. Links out of docs/ must be absolute GitHub URLs (tests/sim/test_docs_links.py).
- New tests go in tests/sim/ (or a new tests dir), unmarked, in the default parallel xdist phase; no serial/nightly. Avoid collection-time parametrize over directory listings.
- Discrepancy: CONTEXT.md (245 lines) is already on main, contradicting map.md.
