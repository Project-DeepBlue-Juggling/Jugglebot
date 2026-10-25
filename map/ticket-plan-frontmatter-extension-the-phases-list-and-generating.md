---
kind: ticket
title: Plan frontmatter extension, the phases list, and generating the three INDEX files
type: grilling
parent: project-map
blocked_by:
- ticket-jugglebot-docs-pipeline-and-the-tests-that-pin-the-plans
- ticket-map-item-schema-and-the-map-directory-layout
owners: []
rank: 6
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/27
created: '2026-10-23'
updated: '2026-10-23T14:37:58Z'
priority: medium
---

Part of #21

## Question

Exactly how do plans join the map? Decide the pathway:, branch: and phases: fields (phase entry shape: id, title, status vocabulary, dated status, gate result?); the rule that phases: must mirror the phase headings and the test that pins it; whether "Open residue" sections become parseable; what each INDEX.md column is generated from and what test_plans_index.py becomes (assert generated output is current? retire the hand-edit assertions?); how "Last touched" is derived (git log vs frontmatter); the archive flow (/archive-plan) under generation; and whether archived plans are migrated or exempt (see fog). Confirm nothing here breaks the filename-never-changes rule or related_plan: resolution.

## Answer

## Answer (grilling, owner, 2026-10-23, one round)

Built on #23's research (`research/jugglebot-docs-pipeline.md`: 69 plans, 21 without step headings, ~10 heading styles, three invalid-YAML archived plans, `branch:` already on three handoff plans) and #25's schema. One glossary fact shaped it: CONTEXT.md defines **Phase** (closes without a Sitting) and **Rung** (closes on a hardware criterion at a Sitting), with "phase" on Rung's Avoid line, and the oval plan's own summary table has a Rung 2 row between P2 and P3.

### New plan frontmatter

```yaml
pathway: two-ball-oval        # required on active and parked plans (test); archived exempt
branch: main                  # optional, default main; same meaning as the three existing uses
scope: >-                     # one paragraph, the stable scope; becomes the active INDEX Scope cell
  ...
last_updated: 2026-10-23      # required on active plans; the generator never shells out to git
steps:                        # optional (21 plans have no step structure)
  - {id: P1, title: The oval pattern dispatchable, kind: phase, status: complete, date: 2026-10-18}
  - {id: P2, title: Banking leak fix, kind: phase, status: retired, date: 2026-10-23}
  - {id: Rung 2, title: One-ball oval at dwell 0.55 then 0.50, kind: rung, status: not_started}
  - {id: P3, title: Pose-level carry planner, kind: phase, status: not_started}
parked: 2026-10-17            # parked plans only: these three replace the status comment as the row source
parked_why: ...
unpark: ...
kind: reference               # reference documents only (leg-gain-tuning-methodology)
```

- **`steps:`, not `phases:`:** each entry `{id, title, kind: phase|rung, status, date?}`; status `not_started | in_progress | complete | retired`; a flown Rung is `complete`. No gate-result field: the Rung's criterion and outcome live in its section.
- The running "where we are" narrative that today lives in the INDEX Scope cell moves into the plan body; the board stops carrying status prose.

### Mirror rule and its test

- One extraction regex for step headings: `##` to `####`, `Phase|P|R|Rung|Stage` followed by an id (`1`, `1a`, `W4a`, `2`), container headings ("Implementation Phases", "Phase Summary") excluded; ids normalised (`Phase 1` ≡ `P1`).
- When `steps:` exists: every listed id matches exactly one heading and every matched heading is listed; titles mirror the heading title; the heading's status suffix (`— NOT STARTED`) is free text and never read. When `steps:` is absent nothing is asserted.
- Each step's **summary** for the map's cards (#38) is the first paragraph after its heading.
- The test is one looping test in `tests/sim/`, unmarked, listing every offender (the collection-time parametrization pattern is not used).

### Residue

- No parseable residue section; residue stays prose. It becomes inbox items through `/inbox` (#30), and **`/archive-plan` gains the same gated offer** as `/investigate` and `/log`: list the residue rows, offer to file each as a Bug or Feature with `links: [plans/archived/<plan>.md]`.

### The three INDEX files are generated

| Board | Columns | Sources |
|---|---|---|
| active | Plan, Status, Last touched, Scope | `status` (`kind: reference` shows "reference"), `last_updated`, `scope` |
| parked | Plan, Parked, Why parked, What would unpark it | `parked`, `parked_why`, `unpark` (the `status: parked # …` comment may stay for readers) |
| archived | Archived, Plan, Status, Title | `archived`, `status`, `title`, sorted by `archived` (unchanged) |

- The hand-written narrative sections (Orchestration prompts, Recently parked, Recently archived) are **retired**; a short static header remains. Their history is logbook content.
- **Who runs the generator:** `/archive-plan`, `/implementation-report` and the parking flow run `python -m mapkit gen`; the app never does (plans are read-only in the app, #38); no pre-commit hook. The backstop is a currency test. **Closes the map's "who runs the generator" fog.**
- **`test_plans_index.py` becomes:** the currency test (regenerate in memory, byte-compare the committed file, all three boards); one looping field test (`status`, `scope`, `last_updated`, `pathway` on active; `parked`, `parked_why`, `unpark`, `pathway` on parked; `archived` on archived); plus the kept filesystem tests (bare archived filenames, one board per filename, no self-link). The row-shape tests and the parked-word regex are retired with the hand-edited rows.

### Archived plans and the parser

- The three invalid-YAML archived plans (`follower-cadence-and-divergence.md`, `PROMPT-canbridge-rx-drain-throughput.md`, `shaped-planning-efficiency.md`) get their colons quoted, one line each, content unchanged.
- The parser normalises `complete` → `completed`, `related_plan` to a list, and dates to ISO strings; files untouched. No retroactive `pathway:` on archived plans (#25).
- The filename-never-changes rule and `related_plan:` resolution are unaffected: plan ids are bare filenames.

### The package

- A new top-level pure-Python package **`mapkit/`** (beside `controller/` and `teensy_link/`, no ROS, tested by the gate): `split_frontmatter`, the item and plan loaders, the step extractor, the three INDEX generators, `python -m mapkit gen`. `logbook_search._parse_frontmatter` is left alone. The app (#31) imports it via `sys.path` on the `--repo` root. This replaces #31's assumed `tools/mapkit/`.

### Sharpened

- #33 receives: the `/archive-plan` gated offer, `/implementation-report` writing `pathway`, `scope`, `last_updated` and running the generator, the parking flow's three fields.
- #35 receives: the `steps:` backfill for the four active plans and the parked plan, the three YAML fixes, the INDEX rewrite and the test rewrite as one phase.
- #32 receives: the generators' output format is the pattern the mkdocs page follows (committed output + currency test).
