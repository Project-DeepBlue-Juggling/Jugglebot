---
title: "Map app: live updates survive a dropped stream (30 s pull, self-opening new builds), the Front page scrolls as one, Pathway tiles show each plan and pack by height, new-item Kind and Priority pills"
type: feature
date: 2026-10-27
status: resolved
phase: "map-improvements"
files_changed:
  - map/feature-map-pages-update-without-a-manual-refresh.md
  - map/feature-front-page-scrolls-as-one-page.md
  - map/feature-pathway-tile-shows-each-plan-separately.md
  - map/feature-new-item-form-kind-and-priority-as-pill-buttons.md
subsystem:
  - project-map
tags:
  - project-map
  - jugglebot-map
---

## Summary

Four owner asks from 2026-10-27, filed as JB-68..71 under Map improvements, all built in the `jugglebot-map` fork (branch `map-improvements-2`, fast-forwarded to fork `main` at bb1fc29) and deployed on the Jetson.

| Item | Fork commit | What changed |
|---|---|---|
| JB-69 live updates | c590132 | `/events` opens with `hello` (data: the build, a hash of `index.html`) and pings as a named event. The page refetches on every `hello` and when its tab comes back into view, reopens a stream silent for 45 s, and opens a newly deployed build by itself once no form is open, no field is focused, no drawer edit is unsaved and no write is in flight. `index.html` is served `no-cache`; the pull timer runs every 30 s (was 2 min). |
| JB-68 one scroll | 253b4d9 | The Front page root is the one scroll box. The tile grid lost its two-row cap, and the inbox wall shows every card. This retires the no-scroll layout. |
| JB-70 plan stack | 5713175 | A Pathway with two or more plans shows each open plan as a row: its full title, its own bar, and every remaining phase. Finished plans fold into one "n plans done" line. Tiles are as tall as their content and pack into the three columns: each tile spans its measured height in 4 px grid rows. |
| JB-71 pills | bb1fc29 | Kind (Idea / Feature / Bug) and Priority (None / Low / Medium / High) are pill buttons over native radios. The form opens on the Kind pills. |

**Why the page missed the morning's pulls.** The server side worked: a live probe saw the pull land at 20:18:03 and an `invalidate` at 20:18:04. The page refetched only on `invalidate`. When its stream dropped (sleep, a network blip, a server restart), the browser reconnected by itself and nothing refetched, so anything announced in the gap was lost until a manual refresh. Refetch-on-focus is off, and the comment heartbeat was invisible to page code, so the page could not notice a dead stream either.

The 30 s timer does not add git lock contention with the server's writer: a pull with nothing new takes no lock, and each new upstream commit is pulled once whatever the rate.

## Verification

All of these were run on the Jetson on 2026-10-27.

- **Fork unit tests:** `npx vitest run`: 284 passed, 1 skipped. `npm run lint` and `npm run typecheck`: clean. `MAPKIT_SRC=... pytest -q tests`: 69 passed. CI run 40482760960: success on all three jobs.
- **Real browser, dropped-stream replay:** a Playwright script drove headless Chromium against a throwaway server over a scratch copy of the map. The script stopped the server, retitled an Item, then restarted the server.
  - Branch: the new title showed after the reconnect, and the page was not reopened. A rebuilt `index.html` made the page reopen itself on the same route.
  - Fork `main` before the change: the new title never showed (0), and the new build was never picked up. This reproduces the owner's report.
- **Real browser, layout:** Chromium screenshots of the real map at zoom 1 and 1.3.
  - Two-ball oval and Learner generalisation each show two plan rows.
  - Map improvements packs under the short Ball Butler tile, 12 px below it.
- **Real browser, keyboard:** in the new-item form, `n` focuses Idea, ← wraps to Bug, Tab reaches Title, and two more Tabs land on Priority None; → → then picks Medium.
- **Live, after `deploy-jetson.sh`:** health ok; `/events` sends `hello` with build `7554b3702f6f`; `index.html` is served `no-cache`; the pull timer fires every 30 s.

## Residue

A page opened before this deploy still runs the old code, so it needs one manual refresh. From then on, pages update themselves.
