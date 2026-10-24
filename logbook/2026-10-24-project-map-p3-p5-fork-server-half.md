---
title: "project-map P3/P5 server half: the jugglebot-map fork's FastAPI server, contract suite, CI, watcher, map-bot git writer and Pi units built and committed locally; frontend seed, GitHub repo and Pi steps wait on the owner"
type: feature
date: 2026-10-24
status: resolved
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

- 2026-10-24, in `~/Desktop/jugglebot-map`, `pytest -q -p no:cacheprovider` (venv Python 3.8): **30 passed in 23.96 s**; `scripts/check_no_upstream_name.sh`: clean; no `.env`, no foreign URL or email in the tree. Later the same day (owner: no attributive connection to the frontend's origin project at all, overriding #31's attribution clause) fork commit `dde18ba` removed `LICENSE-TaskTree` and the README's attribution line and added the project name to the name-pattern check; `pytest -q`: **30 passed in 22.02 s**, check clean.
- 2026-10-24, in this worktree after the plan edit, `python -m mapkit gen --check` exit 0 and `pytest tests/sim/test_plans_index.py tests/sim/test_mapkit_docs.py tests/sim/test_logbook_front_matter.py tests/sim/test_glossary_avoid_words.py -q -p no:cacheprovider`: **34 passed in 3.20s** (markdown-only change; these are the tests that read `plans/`, `docs/` and `logbook/`).

- 2026-10-24, in this worktree with the libyaml parser change, `./run_tests.sh` (log `temp/logs/gate_p5_project_map_20261024.log`): **PASS - 7144 passed, 9 skipped (parallel, 330.76 s) + 5 passed (serial)**, total 357 s.
- 2026-10-24, fork CI run on `2c5fbb0` (`gh run watch --exit-status`): **success** (python, frontend, no-upstream-name).

- 2026-10-24 23:00, on `jeeves` after `git pull --ff-only && ./deploy.sh` (fork `2c5fbb0`, checkout `3393e4a5`, which contains the libyaml change): `yaml.__with_libyaml__` True; check 3 repeated three times over `/api/v1/events` -> `event: invalidate` after **0.869, 0.673 and 0.716 s** (target 1 s, met); both units active, health ok, checkout clean.
- That restart took 90 s: uvicorn waited on a browser's open `/events` stream until systemd's stop timeout killed it, so every deploy took the map offline for 90 s. Fork `3d86c4d` sets a 3 s graceful-shutdown timeout (browsers reconnect); `tests/test_shutdown.py` sends SIGTERM with a stream open (2026-10-24, `pytest -q`: **34 passed in 28.23 s**; the new test fails without the fix, 14.8 s); fork CI run 40092545606 on `3d86c4d` (`gh run watch --exit-status`, 2026-10-24): **success**. The Pi picks it up at its next `./deploy.sh` (that restart is the last one that stalls).

## Open (owner)

1. Create the public repo and push: `cd ~/Desktop/jugglebot-map && gh repo create Project-DeepBlue-Juggling/jugglebot-map --public --source . --remote origin --push` (the session's permission classifier refused "create public surface"). DONE 2026-10-24 by the owner; CI run 40043460138 failed on the workflow file (job-level `hashFiles` condition), fixed in fork commit "ci: the frontend job's seed check is a step"; run 40043742705 (`gh run watch ... --exit-status`, 2026-10-24): **success**.
2. DONE 2026-10-24: the owner supplied the `frontend/` archive and the names; fork commits `024a412` (seed and strip: 398 files to 104 under `src/`, 29 of 128 vitest files kept, `StaticAuthClient`, placeholders for the two pages) and `4aa2397` (`stage` -> `kind`, zod == `server/models.py` field for field, live contract check, ten-line invalidate hook, health banner). 2026-10-24, `cd frontend && npm run lint && npm run typecheck && npm run test -- --run && npm run build`: **303 passed, 1 skipped (the live contract file), build 4.67 s**; `MAPKIT_SRC=... bash scripts/contract_check.sh`: **1 passed**; `pytest -q`: **30 passed in 23.5 s**; fork CI run 40047027409 (`gh run watch --exit-status`): **success**. P3 is complete; the fork history was squashed to one root commit (`08b1b11`) before these landed so no commit message names the origin project (owner rule).
3. DONE 2026-10-24 by the owner's Claude on `jeeves`: deploy key, both clones, `deploy.sh` (twice: server, then with the frontend at fork `11f2f09`), and the checks, reported as triples:
   - check 1: `getent hosts jeeves.local` -> 192.168.20.38 (the Wi-Fi address; wired is .7), `curl -H 'Accept: text/html' http://jeeves.local:8765/` -> 200; the owner's Windows browser could not resolve the bare name `jeeves` (no DNS entry), docs now give both forms;
   - check 2: browser-path edit landed on `main` as `map-bot` within 10 s (`8603f214`, `c449456f`);
   - check 3: `echo >> map/idea-personal-best-window.md` -> `event: invalidate` after **1.12 s and 1.11 s** (target 1 s): cause = 300 ms debounce + full index rebuild in pure-Python YAML; fix `mapkit/frontmatter.py` -> libyaml `CSafeLoader` (rebuild 0.404 s -> 0.147 s on the Jetson, `python -m mapkit gen --check` unchanged); re-measured 23:00 (bullet in Verification): 0.87 / 0.67 / 0.72 s (the server imports `mapkit` once, so a pulled parser change needs `systemctl restart jugglebot-map`);
   - check 4: six items pushed 19:11:13, Pi fast-forward 19:12:03, item served;
   - check 5: forced conflict 11:31, `git pull --rebase failed` logged, banner, edit kept, resolved (`bcfe6faa`, `6f90b945`);
   - reboot 19:30: back at 19:32, both units active, health ok, Immich and Jellyfin back; `npm audit --omit=dev`: 1 high = zod < 4.6.6 (memory exhaustion on oversized input; only our own server feeds it; the fix is the breaking zod 4 upgrade, deferred).
4. Closed 2026-10-24: P5 complete. Residue: the server imports `mapkit` once, so a parser change pulled onto `main` is served only after a restart (map bug `the-pi-server-keeps-serving-the-old-mapkit-after-the-pull-timer-brings-a-new-one`).
