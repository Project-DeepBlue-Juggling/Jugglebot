---
id: BUG-100
kind: bug
title: A dead ticker claim or a crashed 04:00 suite hides a RED for up to two days
status: inbox
created: '2026-11-14'
updated: '2026-11-13T23:09:01Z'
owners: []
priority: medium
diagnosis: hypothesis
area: tests-tooling
blocked_by: []
links: []
parent: night-shift
filed_by: claude
---

A Session that dies between claiming the 04:00 suite ticker and lowering it leaves a claim file behind, and a later RED can then go unread. Found by the 2026-11-13 maintainer review; the claim path was read, the full sequence across a new raise was not traced.

## Findings

- `tools/nightly_ticker.sh:95-115`: `check` moves the raised ticker to `ticker.claim.<pid>`, writes the lowered line, then removes the claim. A claimant killed in between leaves the claim file for good.
- Later checks then print the previous lowered line, or "claim in progress by another session", and read nothing.
- Separately, if the suite is killed or crashes it writes no status and raises no ticker. `STALE` only fires after two days, so one missed night is invisible.

## Wanted

- Expire a claim file by its age (a claim older than a few minutes is dead).
- Report `STALE` when the status is not from today's 04:00 slot, not after two days.
- Cases for both in `tests/sim/`.
