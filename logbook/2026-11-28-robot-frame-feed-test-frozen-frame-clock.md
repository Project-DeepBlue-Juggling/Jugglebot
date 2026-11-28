---
title: "The skill_node test fixture freezes the frame check's clock for frame=False tests too; the Night Shift's 'the box changed' was an unpinned-BLAS reproduction (BUG-154)"
type: bugfix
date: 2026-11-28
status: complete
phase: "day session"
subsystem: skill-stack
files_changed:
  - tests/ros/test_skill_node.py
---

# The fixture freezes the frame clock for frame=False tests too (BUG-154)

**What.** The BUG-150 merge gate (Night Shift 2026-11-28, `./run_tests.sh --full`, log `gate-5b96d46c7aef.log`) failed `tests/ros/test_skill_node_robot_frame.py::test_moved_frame_announcement_landing_is_the_robot_frame_prior`: its self_toss Feed was refused `REJECTED_FRAME_OFFSET` because the Platform body sample was 1.10 s old against the 1.0 s window. `_node_with_client` froze the node's `_frame_clock` only for `frame=True` (BUG-14, d0928ffc). A `frame=False` test stamps its own samples on the real `perf_counter` and then runs a start path whose nomination can take seconds, so the samples age before the frame check reads them. The fixture now freezes the frame clock for every node it builds. `test_frame_check_refuses_when_the_commanded_position_is_moving`, the one test that stamped with `time.perf_counter()` directly, now stamps with `node._frame_clock()`. The night branch's per-test freeze in `test_skill_node_robot_frame.py` is dropped as redundant. Test-only.

## Discussion

**The night's "the trigger is on the box" was a reproduction artefact.** The night found the test failing "on a quiet box" at main and at every commit back to af0f9175, and concluded that something on the box had changed. That reproduction was bare `pytest`, which leaves numpy's OpenBLAS multithreaded. `run_tests.sh` exports `OPENBLAS_NUM_THREADS=1` (lines 122-123), and so does the live launch for the planner (`jugglebot_launch.py:565`). Unpinned, OpenBLAS thrashes on this six-core box under load: a 60×60 `np.linalg.inv` took 420 ms unpinned against 0.2 ms pinned (2026-11-28, load about 4). The Feed start's nomination (`_nominate_at_accept` → `plan_cycle` → `_solve_qp`) then takes tens of seconds. That fails the test at any commit. Pinned, the test passes in about 0.5 s even with another session's `--full` gate running. So main was never red: the 02:00 nightly was GREEN, and the gate's own failure was load pushing a 0.5 s path past a 1.0 s window. It is a load flake with 2× headroom, and it still holds Night Shift gates on loaded nights.

**Why the fixture, not per-test lines.** The night's audit named one more test of the same shape, and the night's own list stopped there. An unpinned isolation probe (each candidate test run alone, so it pays the cold first nomination, 2026-11-28) found two more failing in `tests/ros/test_skill_node.py`: `test_check_lists_the_frame_offset_as_a_refusal_over_limit_with_authority` (the sample was 4.48 s stale at the check) and `test_frame_check_refuses_when_the_commanded_position_is_moving` (its commanded samples aged out of the at-rest window, and the start was accepted). Any `frame=False` test that stamps samples and then starts something is exposed, so the fix belongs in the fixture, where BUG-14 put the `frame=True` freeze. Every `frame=False` stamping path reads the node's own clock (`_frame_ready`, `_on_mocap`, `_on_commanded_position`), so a frozen clock keeps them consistent. A test that needs time to pass sets its own `_frame_clock`. No test outside the three files that use `frame=False` is affected: the other nine `_node_with_client` importers use `frame=True`, which was already frozen.

**Not a flight risk.** BUG-154's open question was whether a flown Feed start could wait as long. The live planner runs pinned, and the pinned cold path is about 0.5 s.

## Verification

- Fail-before (2026-11-28, unpinned isolation probe at origin/main, `python -m pytest "tests/ros/test_skill_node.py::<test>" -q -p no:cacheprovider` per test): the two tests above **1 failed** each. The robot-frame test failed with 31.77 s stale in a file run (`python -m pytest tests/ros/test_skill_node_robot_frame.py -q`, **1 failed, 14 passed**).
- After, the same unpinned isolation runs (2026-11-28): all three **1 passed**.
- After, scoped, gate env (2026-11-28, `flock /tmp/jugglebot-tests.lock env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m pytest` over `tests/ros/test_skill_node.py` and the eleven other `tests/ros/` files that import `_node_with_client`, `-q -n 4 --dist loadfile`): **690 passed in 246.95 s**.
