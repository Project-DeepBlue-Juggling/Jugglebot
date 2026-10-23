---
title: Project map and shared dictionary: implementation
created: 2026-10-24
status: active
owner: Harrison
last_updated: 2026-10-24
pathway: project-map
branch: project-map
scope: >-
  Builds the project map in six phases. A pure-Python `mapkit/` package parses plans and `map/` items and generates the three plan INDEX files, the CLAUDE.md term block and the two mkdocs pages. The glossary is renamed `GLOSSARY.md` and extended. Claude-session wiring covers the slash commands, agents and normative docs. A new public repo `jugglebot-map`, seeded from TaskTree's frontend, serves two pages (Front page, Now / Next / Later) from the Raspberry Pi `jeeves`, with write-through to `map/` files and a debounced commit-and-push. Done when a phone edit on the LAN lands on `main` within 10 s.
steps:
  - {id: P0, title: mapkit package and generated plan boards, kind: phase, status: not_started}
  - {id: P1, title: Glossary rename and term block, kind: phase, status: not_started}
  - {id: P2, title: map directory seed and session wiring, kind: phase, status: not_started}
  - {id: P3, title: Fork skeleton and thin server, kind: phase, status: not_started}
  - {id: P4, title: The two pages, kind: phase, status: not_started}
  - {id: P5, title: Pi deployment and publishing, kind: phase, status: not_started}
related_logbook:
  - 2026-10-24-project-map-plan-written.md
related_code:
  - mapkit/::the parser, generators and subcommands package
---

# Project map and shared dictionary: implementation

This plan implements the design settled by the resolved tickets of wayfinder map #21 (#22-#34, #36-#38, all on github.com/Project-DeepBlue-Juggling/Jugglebot). It designs nothing: every decision is in the ticket answers and is indexed in § 1. The Destination is met when the Pi app serves both pages from `main`, a phone edit lands on `main` within 10 s, and a Claude session files, works and closes an item without hand-editing any generated file.

## 1. Decisions this plan implements

- #22: kept views need five resource families; every response is zod-parsed with all fields, so the server emits full objects (pydantic mirrors).
- #23: only `test_plans_index.py` is affected; build a PyYAML parser, not the hand-rolled one; links out of `docs/` are absolute GitHub URLs.
- #24: thin file-backed server holds; 14 endpoints plus `/events`; prototype branch kept as history.
- #25: flat `map/<kind>-<slug>.md`, identity is the filename, uuid5 of the slug, the field table, per-kind status sets, 409 on closing a pathway with live plans.
- #26: five pathways, the closed `pathway-two-ball-skill-stack`, residue as inbox items, three unattached ideas, branch disposition.
- #27: plan fields `pathway`, `branch`, `scope`, `last_updated`, `steps`, `parked*`, `kind: reference`; one step-heading regex; the three INDEX files generated; currency test; `mapkit/` is top-level.
- #28: `CONTEXT.md` becomes `GLOSSARY.md`; generated CLAUDE.md term block (under 1,200 characters); Avoid-word test over a checked-phrases file.
- #29: thirteen glossary entries (UH-n struck) land with the nine #25 entries.
- #30: `/inbox` command, five-field modal at 400 px, inbox status set, #17/#18/#19 migrated, gated follow-up offers.
- #31: repo `jugglebot-map`, strip list, FastAPI on one port, Pi `jeeves`, 300 ms watcher, 5 s `map-bot` commit-and-push, `map/config.yaml`.
- #32: committed `docs/project-map.md` and `docs/glossary.md` (markdown, not HTML), same currency test, nav entries.
- #33: the session-wiring enumeration (normative docs, commands, agents, tests).
- #34: wayfinder maps are Pathway items, `ticket` kind, `frontier | claim | resolve | import-issues`; #21 imported on day one.
- #37: `/work-inbox`; `promoted` and `promoted_to` apply to all three inbox kinds.
- #38: per-kind rendering table, no-page-scroll rule, hover glow with red blocker outline, palette. (#36 is replaced by P4's live-reaction round.)

## 2. Phase summary

| Step | Lands | Gate | Status |
|---|---|---|---|
| P0 | `mapkit/` parser, loaders, generators, subcommands; plan frontmatter backfill; three INDEX files regenerated; `test_plans_index.py` rewritten | `./run_tests.sh` | NOT STARTED |
| P1 | `GLOSSARY.md` rename and stub, 22 entries, CLAUDE.md term block, Avoid-word test | `./run_tests.sh`; `/audit --unstaged` | NOT STARTED |
| P2 | `map/` seed, `/inbox`, `/work-inbox`, gated offers, agent edits, doc edits, nav entries | `./run_tests.sh`; `/audit --unstaged` | NOT STARTED |
| P3 | Fork repo: strip, static auth, FastAPI server, contract suite, CI | fork CI green (`npm ci`, lint, typecheck, vitest, build, pytest, grep) | NOT STARTED |
| P4 | Front page, Now / Next / Later, drawer, hover, palette, modal | fork CI green; owner live-reaction round | NOT STARTED |
| P5 | Pi units, `deploy.sh`, watcher invalidate, debounced commit-and-push, generated docs pages live | fork CI green; Pi acceptance checks; Jugglebot `./run_tests.sh` | NOT STARTED |

Dependencies: P1 needs P0 (the term block generator). P2 needs P0 and P1. P3 needs P0 (the fork imports `mapkit`). P4 needs P3. P5 needs P2 and P4. P3 may start in parallel with P1/P2 in a second checkout.

## 3. Phases

### P0 — mapkit package and generated plan boards — NOT STARTED
**Implements:** #27, #28 (generators), #32 (generators), #34 (subcommands), #25 (loaders), #23
**Files:** `mapkit/__init__.py`, `mapkit/__main__.py`, `mapkit/frontmatter.py` (yaml-based `split_frontmatter`), `mapkit/items.py`, `mapkit/plans.py`, `mapkit/steps.py`, `mapkit/gen_index.py`, `mapkit/gen_glossary.py`, `mapkit/gen_docs.py`, `mapkit/tickets.py` (frontier, claim, resolve), `mapkit/import_issues.py`; `plans/active/*.md` (4 plans) and the parked plan (backfill); the three archived plans with invalid YAML (`follower-cadence-and-divergence.md`, `PROMPT-canbridge-rx-drain-throughput.md`, `shaped-planning-efficiency.md`: quote colons only); `plans/active/INDEX.md`, `plans/parked/INDEX.md`, `plans/archived/INDEX.md` (regenerated, narrative sections retired, short static header kept); `tests/sim/test_plans_index.py` (rewritten); `tests/sim/test_mapkit_*.py`; `plans/active/leg-gain-tuning-methodology.md` (`kind: reference`).
**Approach:** Build `mapkit` as pure Python beside `controller/` and `teensy_link/`, with no ROS imports, leaving `logbook_search._parse_frontmatter` alone. Normalise `complete` to `completed`, `related_plan` to a list and dates to ISO strings in the parser, never in the files. The step extractor uses the single heading regex (`##` to `####`; `Phase|P|R|Rung|Stage` plus id; container headings excluded; ids normalised) and takes the first paragraph after the heading as the step summary. Generators write the three INDEX files in the column layouts of #27. `gen` also exposes the CLAUDE.md term-block, `docs/project-map.md` and `docs/glossary.md` entry points as no-ops until P1/P2 provide their inputs; they are wired but not yet committed outputs. Subcommands `frontier`, `claim`, `resolve` and `import-issues` follow #34; `claim` and `resolve` commit and push with the #30 rule (`map: claim <slug>`).
**Tests:** `tests/sim/`, unmarked, parallel-safe, one looping test per concern (no collection-time parametrize). Parser suite covers the frontmatter edge cases (dates, str-or-list `related_plan`, the three formerly invalid files), the step regex over the real plan corpus, the mirror rule (every listed id has exactly one heading and the reverse), item loading, and the tickets `frontier` ordering by `rank` with owned and blocked tickets excluded. `test_plans_index.py` becomes the currency test (all three boards regenerated in memory and byte-compared), one looping field test (`status`, `scope`, `last_updated`, `pathway` on active; `parked`, `parked_why`, `unpark`, `pathway` on parked; `archived` on archived), and the kept filesystem tests (bare archived filenames, one board per filename, no self-link). Row-shape tests and the parked-word regex are retired.
**Acceptance:**
- [ ] `python -m mapkit gen` rewrites nothing on a second run (idempotent), and the three INDEX files carry only generated rows under a short static header.
- [ ] Every active plan has `pathway`, `scope`, `last_updated` and, where its headings are phases or rungs, a `steps:` list that passes the mirror test; the parked plan has `parked`, `parked_why`, `unpark`.
- [ ] All 69 plans parse with no exception; the three formerly invalid files parse, their content unchanged apart from quoting.
- [ ] `python -m mapkit frontier <pathway>` on a fixture returns the lowest-rank open ticket with no open blocker and no owner.
- [ ] No `mapkit` module imports `rclpy` or any ROS package (checked by a test).
- [ ] Gate: `./run_tests.sh`, with the (date, command, result) triple recorded in the logbook entry.
- [ ] All tests pass

### P1 — Glossary rename and term block — NOT STARTED
**Implements:** #28, #29, #25 (glossary entries), #33 (CLAUDE.md edits)
**Files:** `GLOSSARY.md` (via `git mv CONTEXT.md GLOSSARY.md`), `CONTEXT.md` (new three-line stub pointing at `GLOSSARY.md`), `CLAUDE.md`, `DOCUMENTATION_GUIDE.md` (§§ 2.9 and 6), `docs/agents/domain.md` (incl. its CONTEXT-MAP wording), `mapkit/gen_glossary.py`, `tests/sim/test_glossary_avoid_words.py`, `tests/sim/avoid_words_checked.txt`, `tests/sim/test_mapkit_glossary.py`.
**Approach:** One rename commit: `git mv`, update the 16 live references (grep first and count before and after; the five historical logbook and plan files that name it are never rewritten and resolve through the stub), then add the 13 #29 entries (Lane, Robot frame, Wave, Braking knot, Carry, Capture move, Catch plane, Hop, Oval, Ball 1 / Ball 2, Handoff lead, Fresh-origin, Emit gap) and the 9 #25 entries (Pathway, Destination, Item, Inbox, Idea, Feature, Bug, Step, Reference document) under their `###` clusters. The generator writes term names grouped by cluster, one line each, between `<!-- glossary-terms:start -->` and `<!-- glossary-terms:end -->` in CLAUDE.md, under the hand-written rule sentence, within 1,200 characters (cluster names drop first when over). Fix "runsheet" at CLAUDE.md:128 and leave "UH-3" as history. The Avoid-word test checks only the phrases in `avoid_words_checked.txt` (`leg 6`, `can-hub`, `session limits`, `guard E-STOP`, `runsheet`, `toss` as a word, `reload` as a word), skips backticked or quoted occurrences and lines ending `<!-- avoid-ok -->`, and scans plans in `plans/active` and `plans/parked`, logbook entries dated on or after 2026-09-21, and every `map/` item. (GitHub #16, the standing ask for this test, was closed by #28 on 2026-10-23.)
**Tests:** `test_glossary_avoid_words.py` (one looping test listing every offender, severity fail); a glossary parse test (term, definition, avoid list, cluster) over the real file; a term-block currency test (regenerate, byte-compare, assert under 1,200 characters).
**Acceptance:**
- [ ] `grep -rn "CONTEXT.md"` outside `logbook/`, `plans/archived/` and the stub returns 0 live references (count before: 16).
- [ ] `GLOSSARY.md` holds the 22 new entries in the skill's format and `mapkit` parses every entry with its cluster.
- [ ] The term block is between the markers in `CLAUDE.md`, and the currency test fails if it is hand-edited.
- [ ] The Avoid-word test passes on the tree and fails on a planted `leg 6` in a fixture plan (negative case asserted in the test file).
- [ ] `/audit --unstaged` run once at phase end (normative docs touched); behaviour-affecting findings surfaced, narrative findings fixed.
- [ ] Gate: `./run_tests.sh`, (date, command, result) triple in the logbook entry.
- [ ] All tests pass

### P2 — map directory seed and session wiring — NOT STARTED
**Implements:** #26, #25, #30, #34 (migration), #33, #37, #32 (nav)
**Files:** `map/config.yaml`; `map/pathway-*.md` (six files: the five live pathways of #26, `pathway-project-map` among them as the home of the imported tickets, plus the closed `pathway-two-ball-skill-stack`); `map/bug-*.md`, `map/feature-*.md`, `map/idea-*.md` (residue and the three ideas); `map/ticket-*.md` (the #21 import); `.claude/commands/inbox.md`, `.claude/commands/work-inbox.md`; `.claude/commands/investigate.md`, `log.md`, `archive-plan.md`, `implementation-report.md`, `next-phase-prompt.md`; `.claude/agents/` logbook-updater, plan-writer, plan-reviewer; `CLAUDE.md` ("Project map" paragraph); `DOCUMENTATION_GUIDE.md` (§§ 2.5, 2.6, 2.9, new 2.10); `docs/agents/issue-tracker.md` (wayfinding section rewritten to file operations); `docs/agents/domain.md`; `mkdocs.yml` (nav "Project map", "Glossary"); `docs/project-map.md`, `docs/glossary.md` (first committed generator output); `tests/sim/test_map_items.py`.
**Approach:** Seed `map/` exactly as #26 lists: five pathways, the closed Skill-stack pathway whose Destination is the R5 gate met 2026-10-13, the (b)-row residue and the two columns-empty-cup rows as inbox items attached where #26 says, the int16 offset-truncation and LIMIT_JERK items, and the three unattached ideas. Migrate #17, #18 and #19 to Feature, Bug and Feature items and close each issue with a comment naming its file. Run `python -m mapkit import-issues 21` to create the `project-map` pathway's tickets, leaving the GitHub issues as read-only history. `/inbox` writes, commits (`map: file <kind> <title>`, no `Logbook-Entry` trailer) and pushes; `/work-inbox` follows the #37 dispatch table and writes back `## Findings`, `## Decision` or `## Progress`. `/investigate`, `/log` and `/archive-plan` gain one gated offer (default yes) listing open follow-ups or residue rows and filing each as a Bug or Feature with `links:`; `/archive-plan` and `/implementation-report` run `python -m mapkit gen` instead of hand-editing rows, and parking sets the three parked fields. Agent edits follow #33. The `map/` pathway files carry no plan list (plans point up with `pathway:`).
**Tests:** `test_map_items.py`: one looping test over `map/` for required fields per kind, status in the per-kind set, `parent` and `blocked_by` targets resolving to an item or a plan filename, no filename renamed (slug equals filename); currency test extended to `docs/project-map.md` and `docs/glossary.md`; the Avoid-word test now covers `map/` items for real.
**Acceptance:**
- [ ] `map/` contains the 6 pathways listed, the residue and idea items, and the #21 tickets; `python -m mapkit gen` is idempotent and `docs/project-map.md` renders one section per live pathway.
- [ ] `mkdocs build --strict` passes locally (absolute GitHub URLs enforced by `test_docs_links.py`).
- [ ] GitHub #17/#18/#19 are closed with comments naming their item files.
- [ ] `/inbox bug "x" "y"` on a scratch branch writes, commits (no trailer) and pushes one file; verified once by hand and recorded in the logbook.
- [ ] `docs/agents/issue-tracker.md` no longer describes GitHub issue operations for wayfinding; `grep -rn "gh issue" docs/agents/issue-tracker.md` hits only the public-door section.
- [ ] `/audit --unstaged` run once at phase end (two or more narrative docs plus normative docs touched).
- [ ] Gate: `./run_tests.sh`, (date, command, result) triple in the logbook entry.
- [ ] All tests pass

### P3 — Fork skeleton and thin server — NOT STARTED
**Implements:** #31, #22, #24, #25 (uuid5, 409), #34 (ticket kind in the API)
**Files (fork:):** new public repo `jugglebot-map`; `LICENSE` (MIT), `LICENSE-TaskTree` ("Copyright (c) 2026 Harrison", verbatim), `README.md` (opens "Derived from TaskTree at 6e0baa2 (2026-10-07), MIT"), `CLAUDE.md`, `frontend/` (seeded from TaskTree `frontend/` at 6e0baa2, then stripped), `server/app.py`, `server/models.py` (pydantic mirroring the zod schemas), `server/store.py` (write-through to `map/` files), `server/events.py`, `pyproject.toml`, `tests/` (pytest contract suite with a fixture `items/`), `.github/workflows/ci.yml`, `scripts/check_no_upstream_name.sh`.
**Approach:** Seed a fresh repo (not a GitHub fork) and apply the #31 strip list in the first commit: Timeline, Team Roles, Project settings, My Tasks, BOM/catalogue/stock/builds/units, Brief, Availability, Teams, Entra/MSAL, notifications, comments, undo, roles and membership UI, all three auth providers and `SignInPage`, the five stock views, the 13 Playwright specs and the ~60 vitest files in stripped directories. Add a ~30-line `StaticAuthClient` (two synthetic users, `owner` and `claude`, no `X-Dev-User`, no `?as=`) and remove the production `local-dev` guard. The server is one FastAPI + uvicorn process serving the 14 endpoints, `/events` and `dist/` as static files on one port (prints "run `npm run build`" when `dist/` is missing). It imports `mapkit` by inserting the `--repo` / `JUGGLEBOT_REPO` root on `sys.path`, reads `map/config.yaml` and refuses to start without it. The three 409 shapes the kept client handles are TaskTree's reparent-cycle, dependency-cycle and delete-conflict shapes (#24); closing or dropping a pathway while any active or parked plan points at it reuses the delete-conflict shape (#25). All are asserted by contract tests. Plans are served read-only from `mapkit`'s plan loader. Dev run: `VITE_API_PROXY_TARGET` as in #22/#24.
**Tests:** the ~40 kept vitest files (those in directories retained); pytest contract suite: every response parses against the pydantic models, the three 409 shapes plus the pathway-close conflict, write-through round-trip (POST then file content, PATCH then `updated` bumped), the uuid5 derivation is stable across restarts, startup refusal without `map/config.yaml`.
**Acceptance:**
- [ ] CI workflow (one file, on push and PR) runs `npm ci`, lint, typecheck, vitest, `npm run build`, pytest and the grep, and is green.
- [ ] `scripts/check_no_upstream_name.sh` fails on a planted occurrence in a scratch commit and passes on `main`; `grep -rni` for the upstream company over the repo returns 0 (the 9 hits in `ui/bugReport*` and `ui/EnvironmentBanner*` are gone).
- [ ] Started with `--repo` at the Jugglebot checkout, `GET /tasks` returns the seeded items and plans, and the built bundle loads with no sign-in screen.
- [ ] A POST of a new item through the API writes `map/<kind>-<slug>.md` in the repo; no commit is made yet (that is P5).
- [ ] `grep -rn "local-dev"` in the production auth path returns 0.
- [ ] No `LICENSE` or README line names any upstream organisation or account (only "TaskTree, MIT, Copyright (c) 2026 Harrison").
- [ ] All tests pass (fork CI)

### P4 — The two pages — NOT STARTED
**Implements:** #38, #30 (modal, 400 px), #37 (`promoted` on all inbox kinds), #34 (ticket cards), #31 (pages, hover, palette)
**Files (fork:):** `frontend/src/pages/FrontPage/`, `frontend/src/pages/NowNextLater/`, `frontend/src/features/drawer/` (adapted `TaskDetailPanel`), `frontend/src/features/newItemModal/`, `frontend/src/features/hover/`, `frontend/src/theme/palette.ts`, `frontend/src/lib/derive.ts` (pathway order, progress units, Next), `server/models.py` (`promoted`, `promoted_to` on idea, feature, bug), matching vitest files and pytest cases.
**Approach:** Build the two pages to the #38 per-kind table. Front page: equal fixed-height pathway tiles with no lead tile, a segmented progress bar (dropped phases excluded), the computed Next box, a what-remains list capped at four rows plus "+n more", a branch chip only when the branch is not `main`, "n tickets open" beside the bar, and below it an inbox wall band that scrolls internally while the page itself never scrolls. Now / Next / Later: pathway swimlanes against Now / Next / Later / Done, plan cards with one row of P-chips coloured by status, phase cards with a status glyph and a two-line clipped summary, an inbox rail of unattached items split into Ideas, Bugs and Features, and ticket cards in Now (claimed) / Later (open) / Done. Inbox cards show title, two-line description, priority, age, a pathway pin or "unattached", a link chip per `links:` entry, and an arrow to what a promoted item became. The shared drawer is read-only for plans and phases (blocked-by / blocks editable), editable for pathways and items, with Promote on inbox items. Hovering any element glows every other element with the same item id and outlines the item's open blockers in red, without dimming anything. Dark is the default with the theme toggle kept; hues: bug rose, feature sky, idea amber, one hue per pathway from a fixed cycling set. The new-item modal has kind (bug, feature, idea), title, one line, priority and pathway, and fits a 400 px screen. Next is derived, never stored.
**Tests:** vitest for `derive.ts` (pathway order, progress units with dropped excluded, Next with and without a blocker), the Next box, chip rendering per status, the capped list, the promoted arrow; a Playwright-free DOM test that neither page scrolls at 1500x900 with six pathways (document height does not exceed viewport); modal layout test at 400 px width; pytest for `promoted` / `promoted_to` round-trips on all three inbox kinds.
**Acceptance:**
- [ ] At 1500x900 with the six pathways, neither page produces a document scrollbar; the inbox band scrolls internally.
- [ ] The what-remains list never shows more than four rows plus "+n more".
- [ ] Hovering a card in one page glows its tile, lane header, segment, token and dep tag on the same page, and outlines open blockers in red; asserted in a vitest DOM test.
- [ ] The new-item modal's five fields are fully visible and usable at 400 px width.
- [ ] A feature or bug can be set to `promoted` with `promoted_to`, and a promoted card shows the arrow.
- [ ] Owner live-reaction round held against the running app (replaces #36): reactions logged in the logbook entry, and any change they request filed as a `map/` item rather than absorbed silently.
- [ ] All tests pass (fork CI)

### P5 — Pi deployment and publishing — NOT STARTED
**Implements:** #31 (Pi, watcher, write path), #32 (pages live), #30 (phone path)
**Files (fork:):** `deploy/jugglebot-map.service`, `deploy/jugglebot-map-pull.timer`, `deploy/jugglebot-map-pull.service`, `deploy.sh`, `server/watcher.py`, `server/gitwrite.py`, `server/banner.py`, `frontend/src/features/events/` (ten-line invalidate handler), `README.md` (Pi runbook), pytest cases. **Jugglebot:** `docs/project-map.md`, `docs/glossary.md` (regenerated, already in nav; `docs.yml` unchanged), the Pi URL in the `CLAUDE.md` Project map paragraph.
**Approach:** Install the units as system units in `/etc/systemd/system` with `User=jeeves` (linger is off), copied from Jugglebot's `tools/systemd/jugglebot-gui.service`; the timer fast-forwards the Pi's checkout `~/Jugglebot` on `main` every two minutes. `deploy.sh` installs Node 22 from NodeSource if missing, then `git pull`, `npm ci`, `npm run build`, `pip install -e .`, `systemctl restart jugglebot-map`, run by hand over SSH after a fork change; no CI artifacts and no secrets on the Pi. The server binds `0.0.0.0:8765` on the Pi (`127.0.0.1` on Windows, `--port` to override, browser opens unless `--no-open`). `watchdog` watches `map/`, the three `plans/` directories and `GLOSSARY.md`; 300 ms after the last event it rebuilds the index and sends one `invalidate` SSE event, and the frontend refetches tasks, dependencies and the project. The write path commits all changed map files 5 s after the last edit as author `map-bot` with message `map: <titles>` and no `Logbook-Entry` trailer, runs `pull --rebase`, then pushes to `main`; a failed rebase or push raises a red banner, local edits are never discarded, and the next edit retries. The Pi serves `main` only.
**Tests:** pytest with a temporary bare git remote: the debounce coalesces a burst of edits into one commit; the author is `map-bot`; the message has no trailer; a rebase conflict leaves the edit on disk and sets the banner; a push failure retries on the next edit; the watcher sends exactly one `invalidate` for a burst inside 300 ms; vitest for the invalidate handler.
**Acceptance:**
- [ ] `http://jeeves:8765` serves both pages from a phone on the LAN, and `systemctl is-active jugglebot-map` and the pull timer report active after a Pi reboot.
- [ ] A browser edit on a phone over the LAN lands on `main` (visible via `git log origin/main`) within 10 s of the last edit, authored `map-bot`.
- [ ] A hand-edited file in the Pi checkout appears in the app within 1 s without a page refresh.
- [ ] `/inbox` from the Jetson (or any checkout) pushes an item and the Pi shows it within about two minutes (timer) with no manual step.
- [ ] A forced rebase conflict shows the red banner and keeps the local edit.
- [ ] `docs/project-map.md` and `docs/glossary.md` are in the mkdocs nav and the docs workflow passes unchanged; Jugglebot `./run_tests.sh` passes after the final regenerate.
- [ ] (date, command, result) triples for fork CI, the Pi checks and `./run_tests.sh` recorded in the logbook entry.
- [ ] All tests pass

## 4. Residue

- Conflicts resolved, not questions: #32 (markdown page) overrides #31 (static dark HTML); #37 (`promoted` on all three inbox kinds) overrides #25 (idea only); #34 adds the fifth kind `ticket` to #25; #27 (`mapkit/` top-level) overrides #31 (`tools/mapkit/`).
- Parse-error behaviour on a malformed hand-edited file (server and `mapkit`) is open: #31 left it open under "Conflict behaviour". P3 must pick a minimum (skip the file and surface a banner) and log it; a decision is not made here.
- The backfill of the existing non-resolved logbook entries (458 entries: 79 `in-progress`, 21 `tuned`, 192 with open-question sections) is one Idea item filed in P2, not work in this plan.
- The deferred deletion of `origin/two-ball-oval`, the `archive/<branch>` tags and deletion of the three stale Windows-box branches (`demo/bb-led-two-ball-juggle`, `phase-a-plus-accuracy-cal`, `claude/unruffled-beaver`), and the prototype branch `prototype/24-thin-server`, is a separate housekeeping commit after P5; none gate a phase.
- The wave-camera plan file is committed to `main` as soon as it exists, per the branch rule; until then the Wave pathway card has no plan.
- One smoke e2e against a fixture in the fork is deferred (#31: "comes later").
- Per-item SSE events instead of one `invalidate`, only if the refetch flickers.
- The Windows SSH key to the Pi is not yet set up; P5 assumes the deploy is run from a machine that has one.
