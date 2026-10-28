---
id: JB-14
kind: ticket
title: 'TaskTree frontend: what backend contract do the kept views need?'
type: research
parent: project-map
blocked_by: []
owners: []
rank: 1
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/22
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

Under the Q21 decision (fork the TaskTree frontend, replace the backend with a thin Python file-backed server), which REST endpoints, response shapes and runtime semantics do the views we intend to keep (canvas, list, board, timeline, detail panel, search) actually depend on? Which frontend features are entangled with backend-only semantics (undo via audit log, SSE live sync, notifications, @-mentions, multi-user, Entra auth, project settings) and how hard are they to stub or remove? Which TaskTree features (BOM, catalogue, calendar, units, etc.) are cleanly separable?

Read frontend/src/api/schemas.ts, hooks.ts, queryKeys.ts, App.tsx, layout/AppLayout.tsx, the SSE and undo code, and the backend routers for the endpoints the frontend calls. Output: an endpoint-by-view table, an entanglement list with a stub/remove/keep verdict each, and the minimal contract the prototype (03) must serve. Findings to research/tasktree-frontend-contract.md.

## Answer

Full findings: research/tasktree-frontend-contract.md.

- The kept views need only 5 resource families (users, projects, memberships, tasks, dependencies): 13 endpoints plus quiet stubs. Ctrl+K search has no endpoint: it filters the cached task list client-side.
- Every response is zod-parsed with all fields required, so the stub must emit the full 28-field Task, User, Project and Membership. Server-derived fields to compute: effective_start_date, due_date (from work_days), backlog_rank, updated_at.
- Mutations patch the cache from the HTTP response, so SSE is not needed for a single user (stub or 404). Undo degrades to a 404 toast. BOM, catalogue, availability, brief, team roles, Teams, notifications and comments are removable or stubbable without breaking the kept views.
- Auth: local-dev sends X-Dev-User; needs at least one user from GET /users and a membership row for that user, or editing disables. Project id must be a UUID. A production vite build refuses local-dev, so use npm run dev:local with VITE_API_PROXY_TARGET (proxy, no CORS) or patch the auth client.
- Open: MSW fixtures in src/test/server.ts unread, canvas create defaults, sameTask semantics; nothing was run.

## Answer

## Answer

Full findings: research/tasktree-frontend-contract.md.

- The kept views need only 5 resource families (users, projects, memberships, tasks, dependencies): 13 endpoints plus quiet stubs. Ctrl+K search has no endpoint: it filters the cached task list client-side.
- Every response is zod-parsed with all fields required, so the stub must emit the full 28-field Task, User, Project and Membership. Server-derived fields to compute: effective_start_date, due_date (from work_days), backlog_rank, updated_at.
- Mutations patch the cache from the HTTP response, so SSE is not needed for a single user (stub or 404). Undo degrades to a 404 toast. BOM, catalogue, availability, brief, team roles, Teams, notifications and comments are removable or stubbable without breaking the kept views.
- Auth: local-dev sends X-Dev-User; needs at least one user from GET /users and a membership row for that user, or editing disables. Project id must be a UUID. A production vite build refuses local-dev, so use npm run dev:local with VITE_API_PROXY_TARGET (proxy, no CORS) or patch the auth client.
- Open: MSW fixtures in src/test/server.ts unread, canvas create defaults, sameTask semantics; nothing was run.
