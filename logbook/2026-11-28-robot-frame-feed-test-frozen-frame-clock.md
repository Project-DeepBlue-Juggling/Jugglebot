---
title: "The robot-frame Feed test freezes the frame check's clock, as the fixture does (BUG-154)"
type: bugfix
date: 2026-11-28
status: complete
phase: "night-shift"
subsystem: skill-stack
files_changed:
  - tests/ros/test_skill_node_robot_frame.py
---

# The robot-frame Feed test freezes the frame check's clock (BUG-154)

**What.** `test_moved_frame_announcement_landing_is_the_robot_frame_prior` builds its node with `frame=False`, so it kept the real `perf_counter` frame clock. It stamps the Platform body samples, then `_start_reload` runs the session-start frame check. On this box that check now reads the samples 1.10 s old against the 1.0 s window and refuses `REJECTED_FRAME_OFFSET`, so `./run_tests.sh --full` fails (the BUG-150 merge gate, 2026-11-28, log `gate-5b96d46c7aef.log`). The test now sets `node._frame_clock` to `_FRAME_CLOCK_FROZEN_S`, as the `_node_with_client` fixture has done for every `frame=True` test since BUG-14 (2026-10-28, d0928ffc). Test-only; no production code changes.

**Left open (audit, 2026-11-28).** The `frame=True` fixture freezes the clock for every test it builds; `frame=False` tests that stamp their own samples and then run a Feed start still use the real clock. `test_a_robot_frame_update_mid_attempt_does_not_move_its_landings` in the same file has exactly this shape and gets the same one-line freeze before the branch merges; the other `frame=False` call sites in `tests/ros/test_skill_node.py` are unchecked. The 2.9 to 5.1 s below is the whole test's runtime, not the samples' age.

**Why it started now (unchecked).** It fails on a quiet box at main (469617cd) and at af0f9175, the tip the 2026-11-27 gate passed, so the trigger is on the box, not in a commit. With the frozen clock the test takes 0.48 s; on the real clock it took 2.9 to 5.1 s. What makes the Feed start path take over a second here is left open on the Item, in case a flown Feed start waits as long.

**Verification.**
- Fail-before (2026-11-28, `night_shift_nonet.sh python -m pytest "tests/ros/test_skill_node_robot_frame.py::test_moved_frame_announcement_landing_is_the_robot_frame_prior" -q -p no:cacheprovider` at origin/main 469617cd's tree): **1 failed in 2.84 s** (`self_toss refused: REJECTED_FRAME_OFFSET ... 1.10 s old, window 1.0 s`).
- After (2026-11-28, `night_shift_nonet.sh python -m pytest tests/ros/test_skill_node_robot_frame.py tests/sim/test_logbook_front_matter.py -q -p no:cacheprovider`): **25 passed in 13.95 s**.
