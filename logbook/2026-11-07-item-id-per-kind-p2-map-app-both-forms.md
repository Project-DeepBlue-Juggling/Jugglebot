---
title: "Per-kind Item IDs P2: the map app (fork repo) reads and links both ID forms; no Item changes ID"
type: refactor
date: 2026-11-07
status: resolved
phase: "map-improvements"
related_plan: item-id-per-kind.md
files_changed:
  - jugglebot-map/server/store.py
  - jugglebot-map/server/nightshift.py
  - jugglebot-map/server/idrepair.py
  - jugglebot-map/server/config.py
  - jugglebot-map/server/models.py
  - jugglebot-map/frontend/src/lib/shortId.ts
  - jugglebot-map/frontend/src/lib/search.ts
  - jugglebot-map/frontend/src/api/schemas.ts
  - jugglebot-map/frontend/src/ui/itemLinks.ts
  - jugglebot-map/frontend/src/ui/MarkdownImpl.tsx
  - jugglebot-map/frontend/src/ui/ShortId.tsx
  - jugglebot-map/frontend/src/pages/NightShift/NightShift.tsx
  - jugglebot-map/frontend/src/features/drawer/ItemDrawer.tsx
  - jugglebot-map/tests/test_ids.py
  - jugglebot-map/tests/test_nightshift.py
  - jugglebot-map/frontend/src/lib/shortId.test.ts
  - jugglebot-map/frontend/src/lib/search.test.ts
  - plans/active/item-id-per-kind.md
  - logbook/INDEX.md
  - docs/project-map.md
subsystem:
  - tools
tags:
  - project-map
  - night-shift
  - testing
---

## Summary

P2 of `plans/active/item-id-per-kind.md`: the map app now resolves, links and searches `JB-<n>` and the per-kind forms (`BUG`, `FEAT`, `IDEA`, `PWAY`, `TKT`). The change is in the FORK repo only (`Project-DeepBlue-Juggling/jugglebot-map`; paths above are prefixed `jugglebot-map/`). No Jugglebot code changed, no Item changed ID, and `mapkit/legacy_ids.csv` is not created. It is inert until P4, and it must be live before P4 because a last-good app older than P2 cannot read per-kind IDs.

What changed: `store.by_short` is keyed on the canonical string; `_node` takes `JB-n`, the five per-kind forms and legacy padded `jb-07`. A bare number on a read lists every Item carrying it (409 with `children` when several); on a write it is refused with 422. The digit match is ASCII-only, so `²` and `٣` still 404. `nightshift._JB_RE` is replaced by `_by_id()` at its 3 call sites, so a Night Shift report in either form fills the answer column. `idrepair.allocate` and both writer modes of `_next_id` call `ids.allocate(kind=)`; the `--no-git` writer no longer counts the index itself. `config.py`'s startup guard imports every mapkit name the fork uses, so a pre-P1 mapkit fails at start. The fork no longer calls `ids.parse`, `ids.canonical` or `ids.format_id`. Frontend: `parseShortId` returns the canonical string for the six prefixes, `SHORT_ID_IN_TEXT` is upper-case only, and a bare number in search lists all kinds, exact matches first (`1` does not find `12`).

Plan correction: the P2 section named `deploy-jetson.sh` for the deploy. That script is provisioning (reinstalls units, pip install, no rollback). The live-correct path is `tools/night_shift_deploy.py`, which saves `~/map-live/deploy-state/dist.last-good`, fast-forwards the live clone from the fork's `origin/main`, builds, restarts `jugglebot-map.service`, health-checks `/api/v1/health` and rolls back on failure. The plan now says so.

## Verification

Tests first (T-U9, T-U10, T-U11), each failing before its code. All runs 2026-11-07.

- Fork gate (orchestrator-run, scratch clone, `MAPKIT_SRC` at the 863d07da worktree): `python -m pytest -q` 110 passed in 76.24 s (2026-11-07 22:58, temp/logs/id-p2-fork-gate-2.log); `cd frontend && npx vitest run` 339 passed, 1 skipped; `npm run build` rc 0; `bash scripts/contract_check.sh` rc 0.
- Deploy: deployed 2026-11-07 23:04 with `python3 tools/night_shift_deploy.py --date 2026-11-07 --result temp/id-migration/p2-deploy-result.json`, result `app deploy: DEPLOYED; 0ea6d9e8 -> 67232aab; healthy`; the live clone moved to the fork commit 67232aa, the unit stayed active with no restart loop, health 200, and a 20-endpoint GET capture before and after (`temp/id-migration/p2-live-before/`, `p2-live-after/`) differed only in the capture timestamp and the built asset hashes; `GET /api/v1/tasks/BUG-118` is 404 on the legacy tree, as expected.
- T-I1 (owner opens an Item by `JB-n` on the phone) is stop point 1.
- Audit (one pass, sonnet audit-reporter): 0 blocking, 3 behaviour, 3 narrative. Applied: ASCII-only digits and the startup guard (both restore today's behaviour or the plan's deploy order), plus the narrative comments. Recorded, not applied: `--no-git` now goes through `git grep` via `allocate` (the alignment the plan asked for); the redundant `canonical_ref` guard in `nightshift._by_id` stays; duplicate-ID error order is now lexicographic.

Tier: no Jugglebot code changed; the Jugglebot-side commit is docs only (logbook, plan), covered by `tests/sim/test_logbook_front_matter.py`, `test_plans_index.py`, `test_docs_links.py`.

## Observed

Baseline `./run_tests.sh` of the untouched 863d07da tree (2026-11-07 22:05, `temp/logs/id-p2-baseline.log`): 9053 passed, 61 skipped, 1 failed, serial 7 passed. The failure, `tests/ros/test_trajectory_node.py::test_guard_latch_descent_falls_back_to_stop_then_descent` (`assert 'hold' == 'move'`), passed alone in 0.51 s while the fork pytest ran concurrently with the gate: load-sensitive on origin/main and outside this plan; recorded, not fixed.

## Outcome

Resolved for the code and tests; the deploy and the phone check follow (above). Next is P3.
