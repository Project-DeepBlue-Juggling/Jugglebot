---
title: "project-map P4: the two pages built in the jugglebot-map fork - Front page (pathway tiles, segmented bar, Next box, capped what-remains, inbox wall), Now / Next / Later (swimlanes, plan and phase cards, ticket cards, unattached rail), the per-kind drawer, the five-field modal, hover-highlight and the dark palette; audited and screenshot-checked; owner reaction round pending"
type: feature
date: 2026-10-24
status: in-progress
phase: "project-map - P4 the two pages"
related_plan: project-map.md
files_changed:
  - plans/active/project-map.md
  - logbook/INDEX.md
subsystem:
  - docs
tags:
  - tooling
  - planning
---

## What changed

Outside this repo, in the `jugglebot-map` fork (public, `Project-DeepBlue-Juggling/jugglebot-map`): the two pages of decision #31 built to the per-kind table of #38 on top of the P3 shell. Fork commits, in order: `020daa4` (`lib/derive.ts`: pathway order, progress units with dropped and retired excluded, Next, lanes, open blockers; `theme/palette.ts`: bug rose, feature sky, idea amber, an eight-hue pathway cycle; `features/hover/`: one provider, every rendered element carries `data-item-id`, hovering glows every twin and outlines open blockers in red, nothing dims; the Front page: equal fixed-height tiles, segmented bar, "x of y stops done", "n tickets open", Next box, what-remains capped at four rows plus "+n more", branch chip only off `main`, inbox wall as a band that scrolls internally), `2afaa46` (Now / Next / Later: swimlanes in computed order, plan cards with a P1..Pn chip row, phase cards with a two-line summary, ticket cards in Now when claimed, the unattached rail split Ideas / Bugs / Features, the computed-next card ringed wherever it sits), `4df1b3e` (the shared drawer under `features/drawer/`, new rather than adapted: the origin's 1,400-line panel was tied to features the strip removed; editable items and pathways, read-only plans and phases, Promote; the new-item modal cut to kind, title, one line, priority, pathway; pytest for the `promoted` round-trip on idea, feature and bug), `bd403f6` (audit fixes: progress units of finished plans, Next when every ready phase is blocked, the drawer diffs against a baseline so an SSE refetch cannot be overwritten, read-only dependency lists on plans and phases, explicit null priority, conflict-dialog labels, ticket links, Promote saves first), `c734924` (screenshot-driven fixes: Destination clamped to three lines in tiles, plan titles instead of slugs, phase summaries without markdown, whole inbox cards in the band, Done column capped at six rows; 46 dead files deleted; no-scroll and 400 px tests relabelled as the CSS contracts they are; `frontend/scripts/screenshot.sh`), `43a29c6` and `11f2f09` (bullet stripping in phase summaries; the fixture id that `43a29c6` broke typecheck on, pushed red and fixed in the next commit).

Three deviations taken by the orchestrator, for the owner to confirm or reverse in the reaction round: (1) the Destination is clamped to three lines inside a tile (the full text is one hover or click away) because a paragraph-long Destination pushed the bar and the Next box out of the fixed-height tile; #38 said "in full". (2) Plan and phase dependencies are read-only in the drawer with the note "edited in the plan file": the server refuses writes to plan files by design (P3), while #38 listed them as editable. (3) In the lanes an in-progress stop that is also the computed Next sits in Now, so Next is often empty and the amber ring marks the next card wherever it is: this is what the owner-scored prototype did.

Known, left: scrollbars render light on the dark theme (`color-scheme: dark` had no effect in Firefox headless); the kept vitest count is 20 files, not the ~40 of #31, because most of the origin's kept tests covered the panel the drawer replaced.

## Verification

- 2026-10-24, in the fork, `cd frontend && npm run lint && npm run typecheck && npm run test -- --run && npm run build` (exit codes checked): **lint 0, typecheck 0, 133 passed / 1 skipped (the live contract file), build ok**; `bash scripts/check_no_upstream_name.sh`: clean; `MAPKIT_SRC=~/Desktop/Jugglebot-map pytest -q`: **33 passed in 25.17 s**; `bash scripts/contract_check.sh` (live zod-vs-server check): 1 passed.
- 2026-10-24, headless Firefox at 1500x900 against the real map (`frontend/scripts/screenshot.sh`): neither page shows a document scrollbar; six tiles, bar, count, Next box and what-remains visible in every tile; first inbox-card row whole; Done column 6 rows + "+15 more" (PNGs in the session scratchpad; the recipe is in the script).
- 2026-10-24, fork CI run 40051768755 on `11f2f09` (`gh run watch --exit-status`): **success** (python, frontend, no-upstream-name).
- 2026-10-24, in this worktree after the plan and INDEX edits, `python -m mapkit gen --check` exit 0 and the scoped doc tests pass (count in the commit message).

## Reactions (owner, 2026-10-24, against the Pi at `http://jeeves.local:8765`)

Both pages "okay, but quite busy". Verbatim points, each filed as a `map/` item (nothing absorbed silently):

- "The Front Page is showing with about a third of the page blank, while the Inbox wall is truncated to be barely taller than a single box" -> Bug `front-page-leaves-a-third-of-the-screen-blank-while-the-inbox-wall-is-one-card-tall` (the 860 px budget tuned to a 900 px viewport does not fill a taller window).
- "Keyboard shortcuts are incorrect" -> Bug `keyboard-shortcuts-dialog-lists-shortcuts-that-no-longer-exist`.
- "Hovering over most of the panels is glitchy (hover effects flash and often disappear completely until the cursor is moved off and back on to the panel)" -> Bug `hover-highlight-flickers-and-drops-out-until-the-cursor-leaves-and-re-enters` (mouseover/mouseout on child transitions).
- "The edit panel is nice, but it takes a while to load. It would be nice to be able to go back to the previous page, especially with the links set up as cleanly as they are. Relatedly, the Summary section could afford to be longer. As long as the entire information in that panel fits in the window height, nothing should be truncated" -> Feature `drawer-opens-as-a-route-loads-faster-and-shows-the-full-summary-when-it-fits`.
- "I don't like how expanding the edit panel hides the ideas/bugs/features, at least in Now/Next/Later" -> Feature `drawer-must-not-cover-the-inbox-rail-on-now-next-later`.
- "I think it might be worth revisiting the style of the Now/Next/Later page. I think something more graphic/visual would be nice. The current layout makes it tricky to see all the different streams of the project at once, as a single stream with many items in Now/Next/Later will cover a lot of vertical space, requiring scrolling to see the rest" -> ticket `now-next-later-a-more-graphic-layout-that-shows-every-stream-at-once` (type grilling, project-map pathway, rank 17): a prototype round like #31's, not a patch.

Fixed the same day (fork, CI-gated): the three bugs in `7585cac` (wall fills the window at 1500x900 and 1920x1080, shortcuts dialog lists the four real keys `n`, `s`, `Enter`, `Esc` with a test per key, hover on `pointerenter`/`pointerleave` with classes toggled on the DOM so no card re-renders), both drawer features in `5ca6452` (`?item=` URL state so Back closes it and walks in-drawer links, rendered from the list cache with dependencies loading behind, full descriptions, a slot beside the rail on Now / Next / Later), dark scrollbars in `672a1df`; vitest **140 passed, 1 skipped** (2026-10-24, `npm run test -- --run`, exit 0). The five items are `complete`. The three orchestrator deviations above drew no comment; they stand until the ticket's round revisits the layout. The reaction round is held; P4's remaining acceptance is the fixes above, which are tracked on the map, not in this plan.

## Open

- The owner's reaction round was held 2026-10-24 (section above); the Pi was redeployed the same afternoon (fork 11f2f09, both pages served). Still open: the Pi Claude's report for checks 1, 3, 4 and the reboot check.
- `logbook/INDEX.md`: the five project-map rows of 2026-10-24 were dropped by another session's commit `4ecc5780` (its INDEX conflict resolution); restored here.
