---
kind: ticket
title: 'Prototype: TaskTree frontend running against a thin file-backed server'
type: prototype
parent: project-map
blocked_by:
- ticket-tasktree-frontend-what-backend-contract-do-the-kept-views
owners: []
rank: 3
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/24
created: '2026-10-23'
updated: '2026-10-23T14:37:58Z'
priority: medium
---

Part of #21

## Question

Does the TaskTree frontend run usefully against a thin Python server that serves the minimal contract from ticket 01 out of an in-memory index built from markdown files? Build the cheapest possible stub (throwaway, in a scratch directory or a throwaway branch of a TaskTree clone): serve projects, tasks (tree via parent_id) and dependencies from a handful of hand-written item files, and point the stock frontend at it. Record which views render, which actions work end to end (create, reparent, edit, add dependency), which features crash or hang on missing backend semantics, and roughly how many endpoints and how much glue it took.

FLIP-TO-FALLBACK CRITERION (pre-registered, Q21): if the canvas or detail panel cannot be made to work without the audit-log / SSE / undo machinery in under one focused session, adopt fallback A (fork TaskTree whole, Postgres as a disposable write-through cache). The owner reacts to the prototype live; the answer records the verdict and the list of features to strip or stub.

## Answer

Resolved: thin server holds; answer in the comment above, prototype on branch prototype/24-thin-server.
