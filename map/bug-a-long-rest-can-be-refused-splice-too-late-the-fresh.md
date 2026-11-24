---
id: BUG-95
kind: bug
title: 'A long REST can be refused SPLICE_TOO_LATE: the fresh decision and the Knot choice use different leads'
status: in_progress
created: '2026-11-14'
updated: '2026-11-24T17:09:51Z'
owners:
- night-shift
- claude
priority: medium
effort: s
hardware: robot
owner: none
radius: subsystem
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

## Sitting

- setup: level, armed, bb-fed
- why: BUG-95's band REST (a Feed's PRE-TILT REST dispatched while a long bridge REST still has 0.225-2.0 s to run) now installs fresh at the bridge's end and slides at most 8 knots (0.200 s) instead of ending the Attempt SPLICE_TOO_LATE; it is offline-only and the slide moves the Feed Catch's splice, so it is flown before it is trusted.
- do:
  - Launch on main after the rebuild, with the bag recording. Dress rehearsal before arming: `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'columns_1ball_fed,apex_m=..,separation_mm=..,dwell_s=..'}"` at the usual working point; it must answer accepted and moves nothing.
  - Fly Ball Butler-fed Columns Attempts as usual, 10 Feeds. The band is reached only when an Attempt opens with the Hand at least about 4 rev from rest (the log line "opening REST returns the Hand to rest ... over" reads 2.2 s or more); note each such Attempt.
  - Watch each Feed: is the Feed Catch made.
- pass: no Attempt ends SPLICE_TOO_LATE at a PRE-TILT REST unless its message says "slide cap"; every "band REST anchored at the running record's end" install slid at most 8 knots, was followed by "spliced CATCH at knot" with no TILT_PIN, and its Feed was caught; Feed Catches no fewer than on the last Ball Butler-fed Sitting.
- record: launch log and bag paths; for each band install its slide in knots and its "installed ... from it" offset; Feeds caught of fed; every SPLICE_TOO_LATE or TILT_PIN line with its time.
- minutes: 20

## Progress

- 2026-11-16: Night Shift 2026-11-16: held on night-shift/2026-11-16-BUG-95: confirmed offline, a band REST installs fresh at the record end; 4 audit findings open, INVARIANTS.md K1 and I-INSTALL-1 to amend; unblocked by TKT-76, a day Session, then a Sitting
- 2026-11-17: Night Shift 2026-11-17: TKT-76 answered option 1 (capped band REST); held on night-shift/2026-11-16-BUG-95; unblocked by a day Session (INVARIANTS.md K1 and I-INSTALL-1, the rebase-slide cap, the ValueError and MAGAZINE findings, then the Sitting section), then a Sitting
- 2026-11-25: TKT-76 option 1 built: a band REST installs fresh at the running record's end on the knot grid; a late solve slides whole knots, capped at 8 (0.200 s), SPLICE_TOO_LATE past it. INVARIANTS K1 and I-INSTALL-1 amended. Logged rebases: 30 of 185 at-rest slides exceed 0.200 s, all outside the band. Unflown; Sitting section added.
