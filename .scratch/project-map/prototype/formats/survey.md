# Display formats for a ~20-50 item project map: survey

Baseline being challenged: the forked TaskTree frontend, an infinite canvas of small uniform
cards joined by parent edges (plus List/Board views). It privileges spatial freedom and the
parent/child tree; it hides reading order, "what next", and anything longer than a card title.

| # | Format (real products) | Privileges | Hides / costs |
|---|---|---|---|
| 1 | **Roadmap swimlanes** (Linear Roadmap, GitHub Projects roadmap, Productboard) | One lane per line of work, items placed on a time axis; parallelism across lanes at a glance | Needs dates we do not have; destinations and descriptions shrink to tooltips |
| 2 | **Gantt / Advanced Roadmaps** (Jira Plans, MS Project, TeamGantt) | Duration, sequencing, dependency arrows, critical path | Dates are mandatory and fake without them; unattached ideas have no row; dense chrome |
| 3 | **Kanban board** (Trello, Jira board, GitHub Projects board) | Status as the primary axis; moving cards is the affordance | Hierarchy flattens (a Phase and a Pathway look alike); "where it leads" disappears |
| 4 | **Now / Next / Later** (ProdPad, Janna Bastow's lean roadmap, Aha! lanes) | Commitment horizon without dates; honest about uncertainty; lanes per theme work well | Exact sequencing inside a bucket; completed work drops out of view |
| 5 | **Outliner** (Workflowy, Logseq, Dynalist, Org-mode) | Full hierarchy as dense readable text; collapse/zoom; keyboard speed | No spatial picture; dependencies are only inline links; status is a glyph among words |
| 6 | **Graph view** (Obsidian graph, Roam, Kumu) | Links between items, clusters, orphans (an Inbox shows up as loose nodes) | Order, status and destination; reads as a hairball beyond ~30 nodes |
| 7 | **Freeform canvas / mind map** (Obsidian Canvas, Miro, Whimsical, MindNode) | Author-placed spatial grouping, radial tree from a centre | Same as TaskTree: arrangement is manual, "what next" is not computed |
| 8 | **Metro / tube map** (subway-map roadmaps, e.g. GOV.UK service roadmaps, Roadmunk "metro" templates) | Each line of work as a coloured route to a named terminus; stations show progress; interchanges show cross-line dependencies | Detail text; fine-grained hierarchy (plans vs phases collapse into stations) |
| 9 | **Treemap / sunburst** (Jira Portfolio sunburst, WinDirStat-style, Foam) | Proportional size of each branch; whole-tree overview in one rectangle | Order, dependencies, destinations; tiny items become unreadable slivers |
| 10 | **Front page / briefing dashboard** (Basecamp Hill Charts + project cards, Notion home, Linear "Pulse", a newspaper lede) | One tile per line of work with headline, progress and next action; editorial priority | Completeness: the long tail and the tree are behind a click |
| 11 | **Note wall** (Google Keep, Apple Freeform stickies, Padlet) | Loose, unequal-size notes; perfect for an Inbox of unattached ideas | Structure of any kind; not a home for the pathway tree |
| 12 | **Hill chart** (Basecamp Shape Up) | Uncertainty: "figuring it out" vs "making it happen" per scope | Needs a manual position per item; no hierarchy or dependencies |

## Recommendation: build 4

Build **(a) Metro map** (#8, inbox yard borrowed from #11), **(b) Briefing outliner** (#5 with a
#10-style strapline per section), **(c) Now / Next / Later with pathway swimlanes** (#4 crossed
with #1, inbox rail), and **(d) Front page** (#10 with a #11 Keep wall for the Inbox). They differ
on the axis that matters. (a) is *spatial and directional*: each Pathway reads left to right
toward its Destination and "next" is a train on the line. (b) is *textual and hierarchical*: the
whole tree as prose you scan and fold, the densest and most complete. (c) is a *matrix*: Pathway
x commitment horizon, so it answers "what are we doing now vs later" across every line at
once. (d) is *editorial*: it ranks Pathways and shows one summary per Pathway, with the
Inbox as a separate loose wall. None of them is a free canvas of uniform cards. Gantt (#2) and
dated swimlanes (#1) were rejected because the data has no dates; graph view (#6) and treemap
(#9) because with 22 items and 2 dependency edges they would show mostly whitespace or slivers.
