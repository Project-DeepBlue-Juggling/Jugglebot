---
title: "project-map P3/P5 server half: the jugglebot-map fork's FastAPI server, contract suite, CI, watcher, map-bot git writer and Pi units built and committed locally; frontend seed, GitHub repo and Pi steps wait on the owner"
type: feature
date: 2026-10-24
status: in-progress
phase: "project-map - P3 fork skeleton and thin server, P5 Pi deployment (server side)"
related_plan: project-map.md
files_changed:
  - plans/active/project-map.md
  - docs/project-map.md
subsystem:
  - docs
tags:
  - tooling
  - planning
---

## What changed

Outside this repo: `~/Desktop/jugglebot-map` on the Jetson holds the fork, two local commits (`7a66c08` P3, `c84209e` P5 server side). The server is FastAPI + uvicorn over `mapkit` imported from the `--repo` root: the 14 endpoints and 7 quiet stubs of ticket #24, `/events`, `dist/` static, pydantic models mirroring the kept zod schemas with `stage` renamed `kind`, write-through to `map/` with uuid5 identity and the `if_unchanged_since` token, the three TaskTree 409 shapes plus the pathway-close conflict of #25, parse errors skipped and surfaced on `/health` (the § 4 open point, now decided as the minimum). P5's server side: the 300 ms watchdog watcher with one `invalidate`, the 5 s map-bot commit-and-push with rebase-abort and banner semantics, `deploy/` units and `deploy.sh`, the README Pi runbook. CI workflow and `scripts/check_no_upstream_name.sh` (placeholder pattern; the owner fills the real names). In this repo only the plan's P3/P5 status and notes changed, and `docs/project-map.md` was regenerated from them.

## Verification

- 2026-10-24, in `~/Desktop/jugglebot-map`, `pytest -q -p no:cacheprovider` (venv Python 3.8): **30 passed in 23.96 s**; `scripts/check_no_upstream_name.sh`: clean; README first line is the required attribution sentence; no `.env`, no foreign URL or email in the tree.
- 2026-10-24, in this worktree after the plan edit, `python -m mapkit gen --check` exit 0 and `pytest tests/sim/test_plans_index.py tests/sim/test_mapkit_docs.py tests/sim/test_logbook_front_matter.py tests/sim/test_glossary_avoid_words.py -q -p no:cacheprovider`: **34 passed in 3.20s** (markdown-only change; these are the tests that read `plans/`, `docs/` and `logbook/`).

## Open (owner)

1. Create the public repo and push: `cd ~/Desktop/jugglebot-map && gh repo create Project-DeepBlue-Juggling/jugglebot-map --public --source . --remote origin --push` (the session's permission classifier refused "create public surface"); the first CI run is P3's acceptance for the server half.
2. Provide the TaskTree `frontend/` at 6e0baa2 (tarball from `E:/PDJ/Github/TaskTree`, or the upstream URL) and the upstream name patterns for `scripts/upstream_name_patterns.txt`; then the seed-and-strip commit, the static auth client and P4 can proceed.
3. On the Pi: clone both repos and run `deploy.sh`; the five acceptance checks are in the fork README (SSH from the session is classifier-denied).
