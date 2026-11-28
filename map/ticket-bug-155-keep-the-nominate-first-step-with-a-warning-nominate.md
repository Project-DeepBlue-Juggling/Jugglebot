---
id: TKT-111
kind: ticket
title: 'BUG-155: keep the nominate-first step with a warning, nominate a bounded prefix at the goal accept, or split the flight rungs across processes?'
status: open
created: '2026-11-29'
updated: '2026-11-28T16:26:38Z'
owners: []
priority: low
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-a-full-turn-rotating-columns-goal-can-block-the-goal-accept
type: question
---

BUG-155: the goal accept nominates the whole Attempt inside the goal callback. A 38-Throw rotating Columns nomination took 6 min 48 s on a loaded box, about a minute idle, and one 74-Throw case took 47 min. The accept is instant only when `/skills/nominate` ran first with the same fields and count. JP-11's Sitting section already requires that.

## Options

1. Keep the procedural guard: the Sitting section's nominate-first step stays the fix, and a goal accept with no cached nomination logs a WARN with the expected wait. Small, no change to what flies.
2. Nominate a bounded prefix at the accept (for example the first 8 Throws) and the rest in the background before each is due. This changes the accept path every Attempt flies, so the night would build it and hold it for a Sitting.
3. Split the three flight rungs across processes. Larger (effort m or more), a design Session.

## What each answer unblocks

1: the next night builds the WARN and merges it if the gate says MERGE. 2: the next night builds it with a replay of the 38-Throw case and holds it for a Sitting. 3: the Item waits for your design Session.

Recommendation: 1 until JP-11 has flown, because the nominate-first step already makes the accept instant and a prefix nomination changes the accept every Attempt depends on.
