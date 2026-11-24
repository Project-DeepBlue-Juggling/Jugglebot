---
id: TKT-93
kind: ticket
title: 'BUG-138: split the plant model by release kind, add a kind term, share K and split the laterals, or defer to E7?'
status: open
created: '2026-11-25'
updated: '2026-11-24T16:52:46Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: learner
blocked_by: []
links: []
parent: bug-plant-model-pools-carried-and-from-rest-rows-so-oval-rows
type: question
---

The plant model f0 pools carried and free_cup rows, so 607 oval rows move the Columns offset at the +62.5 mm Site from -3.3 to +4.8 mm (the review's numbers, not re-verified). The Item names two fixes; they differ in what stays shared, and tonight's BUG-137 finding bears on the second.

## Options

1. Split: one PlantModel per release kind, chosen by the Learner. Simplest and leak-free; the oval no longer shares K or s with Columns and cold-starts its own f0.
2. A release-kind term: a lateral offset for carried rows (b_x, b_y), free_cup the reference, like the per-ball offset; K and s stay shared. It inherits the BUG-139 flaw (an unforgotten term confounded with the forgotten b), so it needs that settled first.
3. Share K, split the lateral filters (b_x, s, b_y) by release kind. The review's oval fit (b_x +179 mm, s +1.23 per m) says the oval's lateral law differs in slope too, while one apex gain K has held across Sittings and Sites.
4. Defer to E7: Memories are siloed per plant today, so no live Columns Memory holds oval rows; keep the pool until E7 needs them.

## What each answer unblocks

Answer 1, 2 or 3 lets the next night build it with a leakage test (oval rows added to a Columns Memory must not move the Columns offset) and merge it if the gate says MERGE; 2 waits for BUG-139. Answer 4 parks BUG-138 under E7.

Recommendation: 3, because it keeps the one parameter the corpus showed is global and separates the two the evidence shows differ.
