---
id: BUG-138
kind: bug
title: Plant model pools carried and from-rest rows, so oval rows move the Columns offset
status: in_progress
created: '2026-11-24'
updated: '2026-11-25T17:40:23Z'
owners:
- night-shift
priority: medium
effort: m
hardware: none
owner: none
radius: normative
diagnosis: hypothesis
area: learner
blocked_by: []
links:
- logbook/2026-11-26-plant-model-lateral-law-per-release-kind.md
parent: learner-generalisation
filed_by: claude
---

The plant model's f0 pools carried and from-rest rows across release kinds, so rows from the oval move the offset applied to Columns. Found in the PWAY-2 review, `logbook/2026-11-24-learner-efficacy-review-and-scatter-decomposition.md`. Plan step E2(b) in `learner-efficacy.md`.

## Evidence

- Adding 607 oval rows to the main Memory moves the applied offset at the +62.5 mm Site from -3.3 to +4.8 mm.
- The p4-oval fit gives b_x = +179 mm and s = +1.23 per m, outside the priors. Only the sum means anything.
- The Learner's key matches exactly on release kind, but the plant model underneath does not.

Numbers are from an agent's study and were not independently re-verified.

## Consequence

- The old P5 item 4 criterion ("theta is shared" between Columns and the oval) would contaminate Columns as written.
- Step E7 (the oval on the shared Memory) is conditional on this fix.

## Wanted

- Split the plant model by release kind, or add a release-kind term, so carried and from-rest rows do not pool.
- A test that adds oval rows to a Columns Memory and asserts the Columns offset does not move beyond a tolerance.

## Options (2026-11-25)

Night Shift 2026-11-25 drafted these; TKT-93 carries them and its answer re-opens this Item.

1. Split: one plant model per release kind. Leak-free; the oval cold-starts its own K and s.
2. A carried-row lateral term, K and s shared. Inherits BUG-139's flaw, so it waits on it.
3. Share K, split the lateral filters (b_x, s, b_y) by release kind.
4. Defer to E7: Memories are siloed per plant, so no live Columns Memory holds oval rows yet.

Recommendation: 3, since K held across Sittings and Sites while the oval's lateral fit differs in slope too.

## Progress

- 2026-11-26: Night Shift 2026-11-26: held on night-shift/2026-11-26-BUG-138: TKT-93 option 3 built (K shared, lateral law per release kind, I-LEARN-1 amended); unblocked by a day Session (2 audit findings, full gate, sim gate, merge) then a rehearsal
