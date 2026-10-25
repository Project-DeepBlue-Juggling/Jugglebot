---
id: JB-12
kind: ticket
title: 'Prototype: the inbox page (Keep-style wall of Idea, Feature and Bug items)'
type: prototype
parent: project-map
blocked_by:
- ticket-prototype-tasktree-frontend-running-against-a-thin-file
- ticket-intake-the-slash-command-the-ui-form-inbox-triage-states-and
owners: []
rank: 15
status: complete
links:
- https://github.com/Project-DeepBlue-Juggling/Jugglebot/issues/36
created: '2026-10-23'
updated: '2026-10-23T14:37:58Z'
priority: medium
---

Part of #21

## Question

How should the inbox page look and behave? Owner intent (#26, 2026-10-23): a Keep-style wall showing every Idea, Feature and Bug item at once; scroll; click-to-expand; edit; delete; attach to a pathway at triage. Build a rough take in the forked TaskTree frontend against the thin server from #24, using the inbox triage states #30 settles, and react to it with the owner.

Blocked by: #24, #30

## Answer

## Closed as subsumed (owner, 2026-10-23, by standing agreement with the recommendation)

The question this prototype was to answer (how the inbox page looks and behaves) has been settled by three later answers: #31 fixed the inbox as the Front page's Keep-style wall plus the Now/Next/Later rail, #30 fixed its states (`inbox`, `backlog`, …) and triage model, and #38 fixed the card fields, the fixed-height internally-scrolling band and hover-highlight. The formats prototype on branch `prototype/24-thin-server` (`?variant=d`) already shows the wall.

Building another throwaway would cost more than building the real wall in the fork, so the owner reacts to the real thing: #35's Front page phase ends with a live-reaction round on the wall and rail, and any change is a follow-up item filed through `/inbox`.
