---
id: BUG-110
kind: bug
title: Emitter catch-up sends the next knot up to 25 ms early and the bridge drops the due knot
status: inbox
created: '2026-11-18'
updated: '2026-11-18T02:07:32Z'
owners: []
priority: high
hardware: robot
owner: decision
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-18-sitting-debrief.md
filed_by: sitting-debrief 2026-11-18
---

Found at the debrief of the Sitting of 2026-11-18 (logbook/2026-11-18-sitting-debrief.md). Related: BUG-89, BUG-58.

- After a late tick the emitter catch-up branch (trajectory_node.py, _emitter_loop overrun branch near line 1486) snaps the next deadline and emits the next knot at once, up to 25 ms before its stamp. The bridge newest-only drain then forwards that early knot and discards the knot that was due, which would have kept the cover. Signature in the bridge log: a setpoint drain dropped line with a 50 ms stamp step and a negative forwarded lag.
- 5 of 126 drop lines on 2026-11-18 have it and all 5 sit right before an Attempt-ending STOP; it ended 3 of the 12 LANE_REFUSED Attempts (launch 1 line 1093, launch 3 line 3085, launch 4 line 1859 of the tee logs) and contributed to a fourth.

- Proposed fix (not made: it changes stream timing, owner decision): after snapping the deadline, wait until it before emitting, so at lead 0 no frame is published before its stamp. Control-cycle walk with launch 4 numbers: tick S is published at S+34 and forwarded alone; the firmware promotes it at S+35 on the same plan as the playing S-25 frame (continuous), cover to S+70;
- S+50 is published at S+50 and arrives about S+56; no STOP, no C1 change; knot S+25 is never sent but frame S already carries it. Test: inject a 35 ms late tick and assert no published stamp is ahead of the publish time. Alternative: the due-aware bridge rule filed under BUG-89 (forward the newest due frame plus the not-yet-due ones).

Not checked: early-emit drops hidden by the 1/s WARN throttle; the 2026-11-13 bags for the same signature.

## Sitting

- setup: level, armed, bb-fed
- why: the fix changes when knots are emitted after a late tick, so the stream has to be seen on the robot.
- do: fly 8 Columns Attempts (Sep. 125, Apex 0.95, Dwell 0.27, Throws 21) and 6 oval Attempts on the fixed build, then grep the launch log for setpoint drain dropped lines.
- pass: no setpoint drain dropped line shows a 50 ms stamp step with a negative forwarded lag, and no Attempt ends LANE_REFUSED after such a line.
- record: the count of drop lines, the count with a negative forwarded lag, each LANE_REFUSED end with its STOP verdict, the launch log path.
- minutes: 20
