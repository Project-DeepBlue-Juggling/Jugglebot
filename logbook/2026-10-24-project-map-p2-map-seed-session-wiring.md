---
title: "project-map P2: map/ seeded (6 pathways, residue and idea items, the 17 map tickets), /inbox and /work-inbox, the gated offers, generated docs pages and the session wiring"
type: feature
date: 2026-10-24
status: resolved
phase: "project-map - P2 map directory seed and session wiring"
related_plan: project-map.md
files_changed:
  - map/
  - mapkit/derive.py
  - mapkit/gen_docs.py
  - mapkit/import_issues.py
  - docs/project-map.md
  - docs/glossary.md
  - mkdocs.yml
  - .claude/commands/inbox.md
  - .claude/commands/work-inbox.md
  - .claude/commands/investigate.md
  - .claude/commands/log.md
  - .claude/commands/archive-plan.md
  - .claude/commands/implementation-report.md
  - .claude/commands/next-phase-prompt.md
  - .claude/agents/logbook-updater.md
  - .claude/agents/plan-reviewer.md
  - CLAUDE.md
  - DOCUMENTATION_GUIDE.md
  - docs/agents/issue-tracker.md
  - docs/agents/domain.md
  - tests/sim/test_map_items.py
  - tests/sim/test_mapkit_docs.py
  - plans/active/project-map.md
subsystem:
  - docs
tags:
  - docs
  - planning
  - tooling
---

## What changed

`map/` exists: `config.yaml`, the six pathways of ticket #26 (five active, `two-ball-skill-stack` complete with the R5 gate as its met Destination), the nine (b) residue rows of the dress-rehearsal plan plus the columns rows 33/34, the sim `--reload` gate regression and the int16 levelling offset truncation as Bug and Feature items, the three unattached Ideas plus the logbook-backfill Idea, GitHub #17/#18/#19 migrated as items and closed with pointer comments, and the 17 tickets of wayfinder map #21 imported by `python -m mapkit import-issues 21 --pathway project-map` (the GitHub issues stay as read-only history). `mapkit/derive.py` computes pathway order, progress, Next and the Now / Next / Later lanes; `python -m mapkit gen` now also writes `docs/project-map.md` and `docs/glossary.md`, both in the mkdocs nav and byte-pinned by `tests/sim/test_mapkit_docs.py`. The session wiring of ticket #33 landed: `/inbox` and `/work-inbox`, the gated follow-up offer in `/investigate`, `/log` and `/archive-plan`, generator calls instead of hand-edited rows in `/archive-plan` and `/implementation-report`, `steps:`-first phase discovery in `/next-phase-prompt`, the logbook-updater's follow-up list and the plan-reviewer's steps check, a Project map paragraph in `CLAUDE.md`, the new `DOCUMENTATION_GUIDE.md` § 2.10, and `docs/agents/issue-tracker.md` rewritten so GitHub issues are the public door only.

## Verification

- 2026-10-24, `pytest tests/sim/test_map_items.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_*.py tests/sim/test_docs_links.py tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py -q -p no:cacheprovider`: **105 passed in 6.49 s**; `python -m mapkit gen --check` exit 0 after the final regenerate.
- 2026-10-24, `mkdocs build --strict` from a scratch venv with `mkdocs-material` (the project venv has no mkdocs): built in 2.40 s, no warnings; the one INFO line is the pre-existing `teensy-udp-protocol.md` outside the nav.
- 2026-10-24, `grep -n "gh issue" docs/agents/issue-tracker.md`: eight hits, all in the public-door conventions, the PR section and the "fetch the relevant ticket" line; none in the wayfinding section.
- 2026-10-24, `/inbox` verified by hand on a scratch branch: `write_item` -> `map/bug-x.md`, `git commit -m "map: file bug x" -- map/bug-x.md` (one file, no `Logbook-Entry` trailer), `git push origin scratch/inbox-verify`; the remote commit showed the one file and the bare message; the scratch branch and the test item were then deleted.
- 2026-10-24, `/audit --staged` at phase end (audit-reporter, Sonnet): two behaviour-affecting findings, both fixed before the gate (`derive.attached` and the Inbox table read `pathway:` where items carry `parent:`, so every pathway showed no attached items; `/inbox` and `/work-inbox` did not regenerate `docs/project-map.md`, which the currency test would have caught on the next gate), and four narrative ones (the Guide's `blocked_by` target form and `filed_by` / `promoted_to`, placeholders, a docstring note that a parked plan can be Next); the reference test now accepts filename stems only.
- 2026-10-24 03:11-03:19 AEDT, `./run_tests.sh --full` in the worktree (log `temp/logs/gate_p2_project_map_20261024.log`): **2 failed, 7097 passed, 9 skipped in 470.84 s; serial tail 7 passed in 28.74 s**. The two failures are the known load-sensitive pair, neither reading anything this phase touched: `tests/ros/test_skill_node.py::test_a_slow_test_run_cannot_age_the_fixture_sample_past_the_accept_check` (the mocap fixture sample aged past its 1.0 s window; same as the P0 gate and the 2026-10-16 entry) and `tests/ros/test_replay_overview.py::test_a_pass_still_exiting_for_an_older_version_neither_reports_computing_nor_blocks_the_requeue` (its own comment records it as load-flaked on 2026-10-11); the box ran a second agent's pytest in the fork directory alongside the gate (load 7-8). Scoped rerun, 2026-10-24 03:24, `pytest tests/ros/test_replay_overview.py tests/ros/test_skill_node.py -q -p no:cacheprovider`: **368 passed in 126.13 s**.
- Venv note: the P3 fork unit installed `fastapi 0.124.4`, `uvicorn`, `httpx 0.28.1` into `~/Desktop/PDJ_venv/venv`, which moved `pydantic` to 2.10.6 and `typing_extensions` 4.8.0 -> 4.13.2 (reverse deps include cantools, python-can, dash, rerun-sdk). This gate ran on the upgraded venv and is the check on that change.
- After these lines were written (markdown only), `pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py -q` re-run: **45 passed in 2.08s**.

## Outcome

P2 complete; the Jugglebot side of the plan (P0 to P2) is done. Next: P3, the `jugglebot-map` fork. Its server half can be built from the prototype's API record; its frontend seed needs the TaskTree pin at 6e0baa2, which exists only on the Windows box (`E:/PDJ/Github/TaskTree`) and could not be located from the Jetson.
