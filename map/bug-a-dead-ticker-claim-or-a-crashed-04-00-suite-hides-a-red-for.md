---
id: BUG-100
kind: bug
title: A dead ticker claim or a crashed 04:00 suite hides a RED for up to two days
status: complete
created: '2026-11-14'
updated: '2026-11-24T13:07:16Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: subsystem
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

## Progress

- 2026-11-18: Claim expiry landed (tools/nightly_ticker.sh, 300 s by mtime, takeover by rename). The missed-slot STALE half is still open.
- 2026-11-25: missed-slot STALE landed (latest 02:00 slot, 30 min grace, 5 min skew) on the LOWERED path; tests in tests/sim/test_nightly_ticker.py. An unclaimed old result is CLAIMED once, then STALE. Entry 2026-11-24-tooling1-ticker-candidate-livesession.
