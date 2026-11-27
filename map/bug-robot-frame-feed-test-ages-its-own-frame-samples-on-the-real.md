---
id: BUG-154
kind: bug
title: Robot-frame Feed test ages its own frame samples on the real clock and fails the suite on this box
status: inbox
created: '2026-11-28'
updated: '2026-11-27T17:34:58Z'
owners: []
priority: high
effort: xs
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: skill-stack
blocked_by: []
links: []
filed_by: claude
---

Found by the Night Shift 2026-11-28: the merge gate's `./run_tests.sh --full` for BUG-150 failed `tests/ros/test_skill_node_robot_frame.py::test_moved_frame_announcement_landing_is_the_robot_frame_prior`. The self_toss Feed was refused REJECTED_FRAME_OFFSET: the Platform body sample is stale, 1.10 s old against the 1.0 s window.

## Findings

- It fails on a quiet box on main (469617cd), and at every commit tried back to af0f9175, the tip last night's gate passed. So the trigger is on the box, not in a commit; what changed is unchecked.
- The test builds its node with `frame=False` and keeps the real `perf_counter` frame clock. It stamps its frame samples, then `_start_reload` runs the frame check over a second later.
- The fixture froze this clock for every `frame=True` test on 2026-10-28 (BUG-14, d0928ffc). Freezing it in this test too makes it pass in 0.48 s.

## Wanted

- Freeze the frame clock in this test, as the fixture does.
- Later: name what makes the Feed start path take over a second on this box (Memory size, uptime), in case a flown Feed start waits as long.
