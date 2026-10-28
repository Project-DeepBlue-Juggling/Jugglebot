---
title: "Map app: clicking the active Inbox-wall sort reverses it; the drawer's six metrics are one row of compact selects"
type: feature
date: 2026-10-28
status: resolved
phase: "map-improvements"
files_changed:
  - logbook/INDEX.md
subsystem:
  - project-map
tags:
  - map-improvements
  - metrics
---

## What changed (jugglebot-map fork, deployed on the Jetson)

Owner feedback on the metrics batch, same evening:
- **Sort reverse** (fork `96951b1`): clicking the already-active sort pill on the Inbox wall
  flips its direction (Age newest⇄oldest, Severity high⇄low, Effort xs⇄xl, Type reversed);
  the active pill shows ▼/▲; unrated severity/effort stay last either way. The
  `jugglebot-map-inbox-sort` localStorage slot now holds `{sort, reversed}` and a stored bare
  key still loads in its default direction.
- **Metrics as selects** (fork `f4612b2`): the drawer's six pill rows became one "Metrics"
  row of small labelled native selects (five on non-bugs), "unrated" as the empty option,
  wrapping on narrow widths; about 40 px instead of six rows with every option visible. A
  change goes into the draft and is saved with the drawer's Save button, as the pills did.
  Kind/Priority pills and the new-item modal's effort/hardware pills are unchanged.

Fork `main` fast-forwarded to `f4612b2` and redeployed (`deploy-jetson.sh`, 2026-10-28,
`:8765` answers 200).

## Verification

- 2026-10-28, fork `frontend/`: `npm run lint` exit 0; `npm run typecheck` exit 0;
  `npx vitest run` **311 passed, 1 skipped** (40 files); `npm run build` exit 0.
- 2026-10-28, fork root, `MAPKIT_SRC=~/Desktop/Jugglebot/mapkit pytest -q`: **73 passed** in 55.25 s.
