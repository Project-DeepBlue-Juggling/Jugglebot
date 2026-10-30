---
title: "Night Shift P3 (and P4's app half): the map app's /night-shift page (committed reports, question tickets answered in two clicks) and /next-sitting page (Runbook rows with pass/fail and notes), fork commit 1ca8001, deployed on the Jetson"
type: feature
date: 2026-10-31
status: complete
phase: "night-shift P3"
related_plan:
  - night-shift.md
related_entries:
  - 2026-10-31-night-shift-p2-work-loop-gate-and-questions.md
  - 2026-10-31-night-shift-p4-sitting-sections-and-runbook.md
files_changed:
  - plans/active/night-shift.md
subsystem:
  - map-app
tags:
  - night-shift
  - map
---

# Night Shift P3: the map app pages

## What

In the `jugglebot-map` repository (fork commit `1ca8001`, one commit for P3 and P4's app half
because they share the hooks, routes and nav): `server/nightshift.py` lists and reads
`docs/night-shift/<date>.md` from the data clone and lists open question tickets;
`POST /night-shift/questions/{id}/answer` appends `## Answer` and completes the ticket through
the store (token check, `updated`, docs regeneration, git writer, 409 on a second answer);
`GET /runbook` serialises `mapkit.sitting.collect` and `POST /runbook/{id}/result` appends
`## Sitting result (<date>)`. Pages `/night-shift` (report picker, Markdown report, question
cards with an answer box; nav badge = open questions) and `/next-sitting` (Pathway chips over
one list, before-power block, blocks by setup state, pass/fail and notes per row; nav badge =
row count). Parsing stays in the checkout's `mapkit`; the server imports it from the data clone.

## Verification

- 2026-10-31, `MAPKIT_SRC=<this tree> python -m pytest -q` in the fork: **96 passed in 142.83 s**
  (T-U8 `tests/test_nightshift.py` 6, `tests/test_runbook.py` 4); `npm run lint`, `npm run typecheck`
  clean; `npm test` **322 passed, 1 skipped**; `npm run build` green; `scripts/check_no_upstream_name.sh` clean.
- Deployed with `~/map-live/jugglebot-map/deploy-jetson.sh`: 2026-10-31 01:12, `~/map-live/jugglebot-map/deploy-jetson.sh` (fork `main` at `1ca8001`, data clone at `aa247beb`), health `ok`; over HTTP at `http://192.168.20.34:8765`: `/api/v1/night-shift/summary` 200 (`latest_date` null, 0 open questions), `/api/v1/night-shift/reports` 200, `/api/v1/night-shift/questions` 200, `/api/v1/runbook` 200 (0 blocks, 24 Items needing a section, 0 questions), `/night-shift` and `/next-sitting` 200 `text/html` with a browser `Accept` header (the SPA fallback serves `index.html` only for `text/html`, so a bare `curl` sees 404 on every page, `/now` included).
