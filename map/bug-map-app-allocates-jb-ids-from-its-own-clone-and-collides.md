---
id: JB-100
kind: bug
title: Map app allocates JB ids from its own clone and collides with mapkit when both file within minutes
status: inbox
created: '2026-10-28'
updated: '2026-10-28T10:53:35Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: subsystem
diagnosis: confirmed
area: map-app
blocked_by: []
links: []
parent: map-improvements
filed_by: claude
---

On 2026-10-28 the owner filed four items from the GUI quick capture (through the map app at :8765, which numbers from its own clone ~/map-live/Jugglebot) while another session filed four with python -m mapkit item new in ~/Desktop/Jugglebot: both allocated JB-90..93. python -m mapkit ids --repair renumbered the mapkit four to JB-96..99 (612ce236) and the JB-76 close-out message carries the pre-repair IDs.

Cause: two allocators (the app server's _next_id over its clone's index, mapkit over the live checkout) with no shared counter; the app clone pulls on a timer, so a filing within the window reuses a number already taken on origin/main.

Fix candidates: the server fetches origin/main and numbers from max(local, origin) before writing; or it numbers from origin/main only and refuses to file while its clone is behind; mapkit item new does the same (git fetch, then max over origin/main). Add a server test that an item on origin/main not yet pulled still bumps the next id.
