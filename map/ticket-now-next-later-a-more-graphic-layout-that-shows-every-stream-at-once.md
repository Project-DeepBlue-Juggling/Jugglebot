---
kind: ticket
title: 'Now / Next / Later: a more graphic layout that shows every stream at once without scrolling'
type: grilling
parent: project-map
blocked_by: []
owners: []
rank: 17
priority: medium
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map/commit/c21a8156a8f13e8ad70fed53512385977fd8389c
- https://github.com/Project-DeepBlue-Juggling/jugglebot-map/tree/prototype/nnl-relayout
- logbook/2026-10-24-project-map-p4-the-two-pages.md
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/38
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/31
created: '2026-10-24'
updated: '2026-10-25T00:55:00Z'
---

## Question

Owner reaction round, 2026-10-24, on the built pages: both pages are "okay, but quite busy". The Now / Next / Later page in particular should be revisited: "something more graphic/visual would be nice. The current layout makes it tricky to see all the different streams of the project at once, as a single stream with many items in Now/Next/Later will cover a lot of vertical space, requiring scrolling to see the rest."

What replaces or reshapes the swimlane grid so that every Pathway (six today, more later) is visible at once at 1500x900 with no scrolling, the commitment horizon (Now / Next / Later / Done) still reads, and the page is less busy? Candidates to put in front of the owner: (a) one fixed-height strip per Pathway with the horizon as a compact timeline (dots/segments with the ring on Next, counts instead of cards, cards on hover or in the drawer); (b) the metro-map variant from the formats prototype (scored 7/10 "too busy" but liked for its directionality) slimmed down; (c) the Front page tiles gaining a horizon row so the second page merges into the first. Also settle where the drawer goes so it never covers the inbox rail (map item filed). Decide by a prototype round like #31's: static variants, owner scores.

## Round 1 (2026-10-25)

Four variants on the real `/now` route (fork branch `prototype/nnl-relayout`, `?variant=A..D`, Z = the current page). Owner, verbatim:

- A, horizon strips: "reasonably good. The "inbox" and "Later" columns feel redundant (all the "Inbox" entries appear duplicated in "Later"). Feels like this wouldn't be able to support many completed Pathways (it'd be nice to have completed pathways be visible in some way, though perhaps this ought to be a separate page?). The "Later" items could afford to be expanded upon; there's vertical space to fit a one-liner for each."
- B, slim metro: "I like this more than A, but it would be better if the bugs/features were more prominent and easy to scan. I like that this view shows how many 'stages' there are to each pathway, giving a better idea of the amount of work completed and left to do. I also like the separation of ideas/bugs/features."
- C, tiles with a horizon row: "Not a big fan. Too much wasted space and it's not easy to scan the page to see what work remains for each pathway, or the ideas/bugs/features."
- D, count matrix: "okay, but it'd be better to be able to see the specific items within each stage of each pathway."
- "My favourite overall is B, but I think we should continue exploring ideas and refining our palette." Round 2 evolves from B, with a dedicated design-critic agent researching practice and reviewing each batch before the owner sees it.


## Round 2 (2026-10-25)

Two new variants built from the critic's research and passed by it before the owner saw them: G, plan tracks (one short line of stages per plan, one full-height inbox column) and E, metro plus list (B's single line split per plan, a readable list beside it). New palette: amber means only Next, ideas lilac, a neutral hover ring. F, a kind grid, was withheld on the critic's advice. Owner, verbatim:

- G: "I really like this one. I think this is the best we've had yet."
- E: "I like this one's display of Pathway bugs/issues the most."
- B: "My least favourite of this set."
- "Overall my favourite is G, but it'd be good to borrow the Pathway bugs/issues style from E; having these expanded out to a one-liner is very helpful to see what the bugs/issues actually are. Expanding the legent would also be nice, so that I can more readily scan through and know what the symbols mean"

Round 3 builds H: G with each Pathway's bugs and features listed as one-liners in its own row (E's style) and an expanded legend.

## Round 3 and verdict (2026-10-25)

H (G plus each Pathway's bugs, features and ideas as one-liners, an unattached-only rail and a full key) passed the critic after one fix pass. Owner, verbatim: "H is the one, build it into the real page. We could afford to simplify the key a bit and make some other smaller changes, but I like the overall look of this one." The smaller changes, verbatim: "Remove the colour bar, stages, hover, "BSS", finished plans, finished tickets items from the key." / coloured dots instead of the coloured left bracket on no-Pathway items / no-Pathway panels "only as tall as they need to be", an over-share category truncated and scrollable / Pathway one-liners aligned, about 85 characters with an ellipsis.

Settled: plan tracks per plan inside each Pathway block, one-liners beside the tracks, unattached items alone in the rail, a key that shows only the marks drawn. Built into fork main c21a815 with tests; the prototype (variants A to H) stays on fork branch `prototype/nnl-relayout` as the primary source. The drawer's Markdown rendering and full titles are a separate follow-up (design review in progress).
