---
id: BUG-95
kind: bug
title: 'A long REST can be refused SPLICE_TOO_LATE: the fresh decision and the Knot choice use different leads'
status: inbox
created: '2026-11-14'
updated: '2026-11-13T23:08:52Z'
owners: []
priority: medium
diagnosis: hypothesis
area: skill-stack
blocked_by: []
links: []
filed_by: claude
---

A long REST can be refused `SPLICE_TOO_LATE` when it should have been installed fresh, which ends the Attempt. Found by the 2026-11-13 maintainer review (Opus read of `executor.py`); the two code sites were confirmed by reading, whether any compiled Schedule reaches the band was not checked.

## Findings

- `executor.py:2289`, `check_segment` (`:2102`) and the node's copy at `trajectory_node.py:4818` decide `fresh = record is None or (t_now_s + lead_s) >= record.end_s` with the per-kind lead from `schedule.install_lead_s`.
- `splice_knot_for` (`executor.py:1796`) then replaces the lead with `splice_lead_s`, which for a REST grows with the window (0.225 s up to roughly 0.5-0.9 s for a 6.5 s REST).
- When the running record ends between the two leads, the REST is not treated as fresh, its Knot clamps to the record's last Knot, and the solve gets about `end_s - t_now - 75 ms` instead of the window-scaled budget.
- `_dispatch` retries only `STALE_STATE`, so the refusal ends the Attempt. Had the REST been fresh, the REBASE rule would have absorbed a slow solve.

## Wanted

- A probe over the compiled Schedules of every dispatchable Pattern: does any REST dispatch inside that band? That decides the priority.
- One function that both the fresh decision and the Knot choice read the lead from, at all three sites.
- A test that pins a long REST dispatched inside the band.
