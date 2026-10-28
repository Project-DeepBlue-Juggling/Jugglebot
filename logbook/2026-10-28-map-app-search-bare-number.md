---
title: "Map app: searching for a bare number ranks that JB-N item first (JB-81)"
type: bugfix
date: 2026-10-28
status: resolved
phase: "map-improvements"
files_changed:
  - map/bug-search-for-jb-n-items-with-just-n.md
  - docs/project-map.md
subsystem:
  - project-map
tags:
  - map-improvements
  - search
---

## What changed

The map app's search (`frontend/src/lib/search.ts` in the `jugglebot-map` fork) derived its
exact-ID tier only from `JB-12` / `jb-12` / `jb12`. A query that is only digits (`12`, trimmed)
now also sets the ID query, so the Item `JB-12` leads the result list exactly as a `JB-12` query
does. The existing rule stands: `12` never ID-matches `JB-120`, and `1` never ID-matches `JB-12`
(title and body hits through the normal tiers are unaffected). `parseShortId` stays strict
because the item-URL resolver depends on it. The search input passes all-digit queries through
unchanged.

Fork commit `a91e7e4` on `map-improvements-3`, fast-forwarded to fork `main` and redeployed on
the Jetson (`deploy-jetson.sh`, 2026-10-28, `:8765` answers 200). Owner filed the bug from the
Robot tab the same day.

## Verification

- 2026-10-28, `cd frontend && npm run lint && npm run typecheck && npx vitest run` (fork):
  lint clean, typecheck clean, **295 passed, 1 skipped**.
- 2026-10-28, `pytest tests/sim/test_plans_index.py tests/sim/test_glossary_avoid_words.py tests/sim/test_map_items.py -q`
  after the map close-out in this repo: see the commit.
