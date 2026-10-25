---
kind: pathway
title: Project map and shared dictionary
status: active
created: '2026-10-24'
updated: '2026-10-24T13:05:00Z'
owners:
- harrison
priority: high
blocked_by: []
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/21
destination: 'A locked design plus a phased implementation plan, handed to a fresh session, for: (1) a project map in git-tracked markdown (`map/` items + extended plan frontmatter) that shows every pathway, what remains in it and where it leads, with an inbox for features, bugs and ideas; (2) a viewer/editor app forked from the TaskTree frontend with a thin Python file-backed server; (3) `GLOSSARY.md`, the project''s shared dictionary, resurrected from the stranded `CONTEXT.md`, parseable, editable in the app, and reliably met by every Claude session via a term-name list in CLAUDE.md and an Avoid-word test. Done when nothing is left to decide before implementation starts and the plan is in `plans/active/`.'
branch: project-map
---

Wayfinder map #21 (github.com/Project-DeepBlue-Juggling/Jugglebot/issues/21), imported 2026-10-24; implementation plan `plans/active/project-map.md`.

## Notes

- Domain: Jugglebot (Stewart-platform juggling robot). Read CLAUDE.md and DOCUMENTATION_GUIDE.md first.
- Skills every session consults: `mattpocock-skills:grilling` + `mattpocock-skills:domain-modeling` for grilling
  tickets; `mattpocock-skills:prototype` for 03; `mattpocock-skills:research` for research tickets.
- Owner token rule (CLAUDE.md): orchestrate from the main thread, delegate grunt work to Sonnet agents
  (Opus only for design-bearing units), cap agents at ~80 tool calls, brief with excerpts.
- Research findings are written to `research/<name>.md` beside this map (not a research branch: the
  tracker is local and gitignored, so a branch would outlive the map it serves).
- Pre-charting research (2026-10-23), copied into `research/`: `tasktree-survey.md` (surveyed BEFORE the
  2026-10-23 pull; core model still valid, feature list behind), `jugglebot-tracking-survey.md`,
  `jugglebot-jargon-survey.md` (67-row term table).
- Repos: Jugglebot = github.com/Project-DeepBlue-Juggling/Jugglebot (gh account Jugleer);
  TaskTree = local clone at E:/PDJ/Github/TaskTree, MIT licence (Copyright 2026 Harrison), pulled 2026-10-23; no upstream organisation is named anywhere in the fork (owner rule, #31).
- Git facts (2026-10-23): origin has only `main` and `two-ball-oval`; `skill-stack` was merged onto `main`
  and retired ("one checkout, one branch"). `plans/active/` on main holds 4 plans; the wave-camera plan
  lives uncommitted on the `2026-10_wave-camera-can3` checkout at E:/PDJ/Github/Jugglebot (14 behind main).
- CORRECTION (ticket 02, 2026-10-23): `CONTEXT.md` (245 lines, `_Avoid_:` lines) IS on `main` (ported from
  mvp-trajectory-bringup in commit 2814c4a0, softened 2026-10-23 in 84d40b27) and is NORMATIVE: CLAUDE.md:36 forbids
  Avoid-words and routes changes through grilling + domain-modeling; DOCUMENTATION_GUIDE § 2.9 is its layer;
  docs/agents/domain.md and the 13 ADRs link to it. The jargon survey ran on the wave-camera checkout (14 behind
  main) and did not see it: its '30 terms reachable only on mvp' claim is void; its undefined-coinage and conflict
  lists still stand and need re-checking against the live file.
- CORRECTION (same cause): the repo DOES have a configured tracker on main: docs/agents/issue-tracker.md (GitHub
  issues, with a Wayfinding operations section using sub-issues and native dependencies) and
  docs/agents/triage-labels.md. This map was charted locally before that was known; see fog.

## Decisions so far

### Charting decisions (grilling rounds 1-3, owner, 2026-10-23)

- Destination is design + plan; implementation happens in a later, fresh session.
- Consumers: the owner and Claude sessions (Windows box and Jetson). Collaborators who pull the repo can
  view the map without running anything.
- Markdown in the Jugglebot repo is the single source of truth for map, plans and glossary.
- The map wraps the three-way plans board and the logbook; it does not replace them. The three
  `INDEX.md` files become generated output.
- Pathway = an arc: a named line of work with a stated destination, grouping plans, usually owning a branch.
  Every pathway has a destination.
- Idea = an intake item type beside Feature and Bug: no pathway, no destination required; promotable to a
  plan under a pathway or to a new pathway once it has a destination.
- Glossary: one project-wide file with a context tag per term. Claude sessions edit it directly; the owner
  reviews via the app and git diff.
- Sessions meet the glossary via a bare term-name list in CLAUDE.md (< ~300 tokens) with the rule "read the
  entry before using or reading one of these words"; full entries on demand; an Avoid-word test on new
  logbook entries and plans is the backstop.
- Rename the live normative `CONTEXT.md` to `GLOSSARY.md` (the name the installed domain-modeling skill v1.3.1
  writes; the repo's docs/agents/domain.md still says CONTEXT.md, so a skill-driven session would create a second
  file beside it), restructured to fixed per-term fields; grill the undefined coinages and conflicts. Decided on
  the stale 'stranded file' premise: ticket 07 RE-CONFIRMS the rename against its real ripple (CLAUDE.md:36/126/216,
  DOCUMENTATION_GUIDE § 2.9, docs/agents/domain.md, 13 ADR links, logbook pointers). _Re-confirmed by #28 with a CONTEXT.md stub for history._
- Intake from both a slash command and the app UI, writing the same record; items land in an inbox and are
  attached to a pathway at triage.
- Fork TaskTree, do not upstream: it is a starting point and will diverge.
- Foundation (Q21): fork the TaskTree FRONTEND; replace the backend with a thin Python file-backed server
  (in-memory index rebuilt from the files, write-through edits, directory watcher, serves only the subset of
  TaskTree's REST contract the kept views need). Pre-registered fallback: fork TaskTree whole (Postgres as a
  disposable write-through cache) if the prototype (ticket 03) shows the frontend is too entangled with
  backend semantics (undo, SSE, audit) to run against a thin server.
- The markdown parser/generator is a Python package inside the Jugglebot repo, tested by the gate; the app
  imports it from the checkout. Backend language is therefore Python.
- Fork lives in its own repo under the Project-DeepBlue-Juggling org; Jugglebot holds data + glossary + a
  config pointing the app at the checkout. The app runs on the Windows box only. _Amended by #31 (owner, 2026-10-23): the single instance runs on the Raspberry Pi `jeeves` and every device opens it; the Windows box runs it only for frontend development._
- A generated mkdocs page renders map + glossary for collaborators on GitHub Pages.
- Plans grow `pathway:`, `branch:` and a ~~`phases:`~~ `steps:` frontmatter list mirroring the step headings, pinned by a
  test that fails when list and headings disagree. Plans are read in place; no shadow items. _Amended by #27: the list
  is `steps:` because CONTEXT.md separates Phase from Rung and the oval plan has a Rung row; it is optional._
- This system becomes the issue tracker that future wayfinder maps live on. _Confirmed and extended by #34 (owner): the current map too, imported on day one._
- This effort works in the `project-map` worktree; the dirty wave-camera checkout is left alone.

## Not yet specified

- ~~Migration of the ~62 archived plans and 1 parked plan~~: closed by #25 (archived plans get no `pathway:`; closed pathways are `complete` items, one created on day one; the parked plan gets `pathway:` like any live plan).
- ~~Item identity across branches~~: closed by #25 (identity = filename, no numeric id; concurrent filing meets as an add/add conflict at push and the loser re-slugs before publishing).
- Conflict behaviour: a UI edit while a Claude session has the same file open; file-watcher vs git pull
  mid-edit; what the thin server does on a parse error in a hand-edited file. _Sharpened by #31: the Pi
  commits and pushes every edit with `pull --rebase`, so git settles the first two at file granularity and a
  banner surfaces failures; the parse-error case is still open._
- ~~Undo and live-sync in the thin server~~: closed by #31 (undo dropped; SSE kept only as the watcher's `invalidate` channel).
- ~~Whether `docs/adr/` and logbook entries become map items~~: closed by #34 (by reference, via `links:`).
- ~~Who runs the INDEX generator~~: closed by #27 (the slash commands run it, a currency test is the backstop, no hook, never the app). The mkdocs generator follows the same pattern; #32 confirms.
- Existing-slash-command changes: `/archive-plan`, `/implementation-report`, `/log`, `/investigate` will
  need to write map state; exact edits are fog until 04/06/12 land. _Sharpened by #30: `/investigate` and `/log` gain a gated
  offer to file open follow-ups as inbox items, and `/inbox` is the new intake command; #27 adds the same offer to `/archive-plan` and has `/implementation-report` write `pathway`/`scope`/`last_updated` and run the generator; exact edits are #33's._
- ~~Hosting future wayfinder maps (including this one) inside the new system~~: closed by #34 (pathway + ticket items; this map imported on day one).
- ~~Whether the term-name list in CLAUDE.md is hand-written or generated~~: closed by #28 (generated between markers by `mapkit gen`).

## Out of scope

- Reconciling the wave-camera checkout (14 behind main, uncommitted work, upstream gone): belongs to the
  session that owns that work.
- Mass renames in code/docs to the glossary's canonical terms (`reload`->feed 161 hits, `toss`->throw 124,
  `session`->sitting 167, `runsheet`->runbook 211): the Avoid-word test applies to NEW artefacts only; each
  rename is its own effort.
- Cleaning the stale auto-memory `MEMORY.md` (215 lines, still describes the deleted MPC chain): hygiene,
  not on this route; worth its own short session.
- Multi-user, auth, running the app on the Jetson, upstreaming anything to TaskTree.
- Deleting the stray `grep.exe.stackdump` at the Jugglebot repo root (left by a research agent 2026-10-23).
