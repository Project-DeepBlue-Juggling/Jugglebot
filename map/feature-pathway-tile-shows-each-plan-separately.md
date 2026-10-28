---
id: JB-70
kind: feature
title: Pathway tile shows each plan separately
status: complete
created: '2026-10-27'
updated: '2026-10-27T09:45:33Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: local
area: map-app
blocked_by: []
links:
- logbook/2026-10-27-map-live-updates-scroll-plan-stack-pills.md
parent: map-improvements
filed_by: claude
---

Owner ask 2026-10-27, for Pathways with two or more plans (e.g. Two-ball oval). On the Front page tile, each open plan gets its own row: the plan's name, its own segmented bar, and all its remaining phases listed below it (no +n more). Finished plans fold into one grey line (n plans done). The Next box and the stops count stay at Pathway level; a one-plan Pathway looks as today. Tiles are only as tall as they need to be and tessellate (a tall tile may sit beside several short ones); they do not stretch to the tallest in a row. The Now / Next / Later page already draws one track per plan and is unchanged.

## Progress

- 2026-10-27: built in jugglebot-map bb1fc29 and deployed on the Jetson
