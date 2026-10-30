---
id: JB-15
kind: ticket
title: 'The fork: repo name, runtime shape, what is kept, stubbed or dropped, watcher and config'
type: grilling
parent: project-map
blocked_by:
- ticket-prototype-tasktree-frontend-running-against-a-thin-file
owners: []
rank: 10
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/31
created: '2026-10-23'
updated: '2026-10-23T14:37:58Z'
priority: medium
effort: s
hardware: none
owner: decision
radius: local
area: map-app
---

Part of #21

## Question

Given the prototype verdict (03), decide the fork's concrete shape:
- Repo name under Project-DeepBlue-Juggling (proposal: jugglebot-map).
- A GitHub fork of TaskTree (keeps history) or a fresh repo seeded from frontend/.
- Runtime (docker compose with two services, bare uvicorn plus a vite build, or a single container).
- The strip list (BOM, catalogue, calendar, units, Entra, Teams, notifications, multi-user roles, audit log?) and the stub list from 03.
- The directory watcher and its debounce rule.
- The config file in Jugglebot that points the app at the checkout and names the map/, plans/ and GLOSSARY.md paths.
- How the app loads the shared Python parser package from the checkout (PYTHONPATH vs editable install).
- Which tests the fork repo carries.

## Answer

Amendment from #27 (owner, 2026-10-23): the shared parser is a new top-level pure-Python package `mapkit/` in Jugglebot, not `tools/mapkit/`. The sys.path import rule is unchanged.
