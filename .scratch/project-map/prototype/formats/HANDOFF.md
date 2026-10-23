# Handoff: four display formats for the project map (wayfinder #31)

Open `index.html` by double-click (file://, data embedded, no libraries). Switch with the bottom
bar or `?variant=a|b|c|d`. Click any item for the shared detail drawer (Esc closes).
Screenshots: `variant-a.png` .. `variant-d.png` (full-page at 1500 px; the fixed bottom bar shows
mid-page in them, it sits at the bottom of the window in real use). Survey: `survey.md`.
Playwright check 2026-10-23: all four variants render, zero console errors, no horizontal scroll at
1500 px, bar switching updates the caption and URL, the drawer opens from a card.

| | Variant | Privileges | Hides / costs | Survey entries |
|---|---|---|---|---|
| a | **Metro map** | Direction: each Pathway is a coloured line to its Destination terminus; Phases (or Plans with no Phases) are stations; NEXT marks the next stop; striped line = Pathway blocked; attached Inbox items in a siding under the line, unattached ones in the Inbox yard; dependencies as red arcs | Descriptions (drawer only); a Plan with Phases is only a bracket label; ordering of stations is computed, not authored | #8 metro, #11 note wall |
| b | **Briefing outline** | Completeness and reading: the whole tree as foldable text, Destination as section strapline, computed "Next up", filter box, show-notes and hide-done toggles; deps as inline "blocked by / blocks" tags | No spatial or visual sense of progress; a 50-item map becomes a long scroll | #5 outliner, #10 briefing |
| c | **Now / Next / Later** | Commitment: Pathway swimlanes x Now / Next / Later / Done, Inbox rail on the right; the Destination sits in the lane header | Tree depth (a Phase and a Plan look alike apart from the kind label); Later mixes queued Phases, parked Plans and attached Inbox items | #4 Now/Next/Later, #1 swimlanes |
| d | **Front page** | Editorial priority: lead tile for the top Pathway, one tile each for the rest with Destination, segmented progress bar, NEXT box, what remains, owners; Keep-style Inbox wall with Pathway pins | The tree below a Pathway is flattened to a list; nothing is shown for Done stops except the bar | #10 front page, #11 Keep wall |

## Computed rules (same in every variant)
- Pathway order: priority, then status (in progress, blocked, not started), then title.
- Stations / progress units: a Plan's Phases if it has any, else the Plan. Dropped units are not counted in "x of y".
- Next up: first unfinished Phase of the first ready Plan (Plans with an open blocker sort last). If every open Plan waits on a blocker, Next is that blocker (Ball Butler: Next = Feature "Probe: round 2").
- Now = in progress / QA / blocked; Next = the computed next; Later = everything else open; Done = complete / dropped.

## Key information shown poorly or not at all
- **Branch: not in the snapshot.** `data.json` has no branch field, so no variant shows it. The drawer renders a Branch row if an item ever carries `branch`.
- Owners: every variant shows names, but almost everything is unowned in the data, so "unowned" dominates (a, c, d show it on every card or tile).
- The Project map Pathway has no Plans: a renders a dashed line with no stations, c an empty lane, d shows its description in the NEXT box.
- Both dependencies are inside one Pathway, so a never had to draw a cross-line arc; cross-row arcs are drawn but untested on real data.
- a does not show priority except as pills on siding tokens; c shows the Destination text truncated only by lane width (long ones make tall lanes).

## Unfinished / caveats
- No editing, drag, persistence or dark theme; light only. Layout tuned for ~1500 px; not checked on narrow screens.
- The source `data.json` had mojibake em dashes in the Phase descriptions; the embedded copy replaces U+FFFD with an em dash.
- The page was generated from a template (data spliced in); edit `index.html` directly from here on.
