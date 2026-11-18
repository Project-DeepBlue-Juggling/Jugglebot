---
title: "trajectory_node waits for the snapped deadline after a late tick, so no knot is emitted before its stamp (BUG-110)"
type: bugfix
date: 2026-11-18
status: complete
phase: "Sitting 2026-11-18 follow-up"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - tests/ros/test_trajectory_node.py
subsystem: [skill-stack, can-bridge]
tags: [emitter, timing, lane-refused]
---

# The emitter waits after a late tick (BUG-110)

**What.** `trajectory_node._emitter_loop`'s overrun branch snapped `next_deadline` up to the next grid knot and looped to emit it at once, up to 25 ms before its stamp (stamp = t_k + L, L = `emit_lead_knots` x 25 ms, currently 0). The branch now waits, interruptibly (`self._emit_stop.wait`), until the snapped deadline, like the on-time branch does. No limit, grace, lead or constant changed; the bridge drain is untouched.

**Why.** The early frame and the late due frame reached `teensy_bridge_node` together. Its newest-only drain (`recv_latest`, which assumes nothing is published before its stamp at lead 0) forwarded the not-yet-due frame and discarded the due one. The playing frame's cover then ran out and the firmware STOPped. This ended 3 of the 12 LANE_REFUSED Attempts at the 2026-11-18 Sitting.

**Cycle walk (lead 0; S = the late tick's grid knot; cover = stamp + 50 ms span + 20 ms grace).**
- 10 ms late: S is published at S+10. The deadline S+25 is still ahead, so the overrun branch is not entered. Identical before and after.
- 35 ms late (launch 4): S is published at S+35 and the deadline snaps to S+50. Before: S+50 went out at S+35, the drain kept it and dropped S, and the S-25 cover ended at S+45 before S+50 was due, so the firmware STOPped. After: S is forwarded alone and promoted at about S+36 inside the S-25 cover, with its own cover to S+70. S+50 is published at S+50 and arrives about S+56. Knot S+25 is never sent, but frame S carries it as u1 on the same plan.
- 60 ms late: the S-25 cover ends at S+45, before anything is published, so this is a STOP before and after. The only difference: the first frame after it is S (stale), then S+75 at its stamp, instead of S+75 early with S dropped. Nothing is made worse.
- At a non-zero lead, the knot t_k is published at or after t_k, which is its stamp minus L. A stall of several periods skips the missed knots onto the same absolute grid and the same plan. Once the late frame's lag passes the cover (above about 45 ms at lead 0), the Attempt STOPs, with or without the fix.

**Verification.** (2026-11-18, `python -m pytest tests/ros/test_trajectory_node.py tests/ros/test_trajectory_node_rt_param.py tests/ros/test_skills_plan_bench.py tests/ros/test_install_segment.py tests/motion/test_trajectory_emitter.py tests/motion/test_solver_worker_lateness.py -q`, **332 passed in 36.24 s**; the full gate was not run). The new tests use a fake clock and run the real `_emitter_loop`. With late wakes of 10, 35, 60 and 110 ms they check that every t_k is published at or after t_k and stays on the grid. They also check that a stop during the post-overrun wait emits nothing more. Before the fix, 35, 60, 110 ms and the stop case failed. The 10 ms case passed, as the walk predicts.

**Unflown.** The fix has not run on the robot. The BUG-110 Sitting section (8 Columns plus 6 oval Attempts; count the drop lines that show a 50 ms stamp step and a negative forwarded lag) decides it. The first tick after `_start_emitter` also emits without waiting, but it runs before seeding, so nothing streams.
