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

- setup: level, bb-fed
- why: BUG-95's band REST (a Feed's pre-tilt REST dispatched onto a still-running long bridge REST) now installs fresh at the bridge's end and slides at most 8 knots (0.200 s) instead of ending the Attempt SPLICE_TOO_LATE. Offline, the band is reached only for a narrow Ball Butler Feed timing, so this flight is a census plus a regression check, not a forced hit; a raised Hand makes it possible, not certain.
- do:
  - Launch on main after the rebuild with auto_arm:=false and the bag recording. Boot, Home, Level, Activate. Dress rehearsal before arming: ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'columns_1ball_fed,apex_m=..,separation_mm=..,dwell_s=..'}" at the usual working point; it must answer accepted and moves nothing.
  - Read /robot_state: Hand homed, no Hand error flags, no jam hold. Cup empty, no ball. Only then one call: ros2 service call /hand_move_to jugglebot_interfaces/srv/SetFloat "{data: 4.5}". It moves the Hand at 1.0 rev/s (about 4.5 s). Do not repeat it if the reply is anything but arrived; stop and ask a Session.
  - Read the Hand from /robot_state: about 4.5 rev (within 0.05), still there 10 s later. Then arm with /set_setpoint_output and fly 10 Ball Butler-fed columns_1ball_fed Attempts as usual; the opening REST should take about 2.3 s (the DEBUG line reads: opening REST returns the Hand to rest ... over).
  - Then fly 10 more Attempts with the Hand left at rest for the control group. Watch each Feed: is the Feed Catch made.
- pass: No Attempt ends SPLICE_TOO_LATE at a PRE-TILT REST unless its message says "slide cap". Every "band REST anchored at the running record's end" install (it is not guaranteed to occur) slid at most 8 knots, was followed by "spliced CATCH at knot" with no TILT_PIN, and its Feed was caught. The Hand held 4.5 rev (within 0.05) after the move with no error flag. Feed Catches per Attempt no fewer than on the last Ball Butler-fed Sitting.
- record: Launch log and bag paths; the /hand_move_to reply text and the Hand rev before, right after and 10 s later; each Attempt's opening-REST period; the count of band installs and for each its slide in knots and its installed-from offset; Feeds caught of fed per group; every SPLICE_TOO_LATE, REJECTED_RELOAD_UNSCHEDULABLE or TILT_PIN line with its time.
- minutes: 35

## Progress

- 2026-11-16: Night Shift 2026-11-16: held on night-shift/2026-11-16-BUG-95: confirmed offline, a band REST installs fresh at the record end; 4 audit findings open, INVARIANTS.md K1 and I-INSTALL-1 to amend; unblocked by TKT-76, a day Session, then a Sitting
- 2026-11-17: Night Shift 2026-11-17: TKT-76 answered option 1 (capped band REST); held on night-shift/2026-11-16-BUG-95; unblocked by a day Session (INVARIANTS.md K1 and I-INSTALL-1, the rebase-slide cap, the ValueError and MAGAZINE findings, then the Sitting section), then a Sitting
- 2026-11-25: TKT-76 option 1 built: a band REST installs fresh at the running record's end on the knot grid; a late solve slides whole knots, capped at 8 (0.200 s), SPLICE_TOO_LATE past it. INVARIANTS K1 and I-INSTALL-1 amended. Logged rebases: 30 of 185 at-rest slides exceed 0.200 s, all outside the band. Unflown; Sitting section added.
