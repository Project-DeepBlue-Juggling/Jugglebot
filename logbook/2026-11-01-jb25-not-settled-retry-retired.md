---
title: "JB-25: the NOT_SETTLED retry in skill_node is retired; a Ball Butler THROW_ABORTED_NOT_SETTLED fails closed like every other non-OK token"
type: refactor
date: 2026-11-01
status: complete
phase: "night-shift follow-up"
related_plan:
  - skill-stack-dress-rehearsals-and-residue.md
related_entries:
  - 2026-11-01-night-shift.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/ros/test_skill_node.py
  - logbook/INDEX.md
---

# JB-25: the NOT_SETTLED retry is retired

**What.** `skill_node._on_bb_throw_outcome` carried a once-per-attempt retry on Ball Butler's
`THROW_ABORTED_NOT_SETTLED` (R5, 2026-09-30, for the 4-of-19 yaw-settle false positives that
Sitting saw). It only fired while the `reload` ctx was still in `await_announcement`, and in the
field the announcement swap retires that ctx before Ball Butler's terminal verdict arrives
(bb-self-cancel finding F1, 2026-10-13), so the verdict always ended the attempt as
`REJECTED_BB(THROW_ABORTED_NOT_SETTLED)`. The owner chose option 1 of JB-125: delete the branch,
keep the fail-closed end. The hazard was the docstring: 40 lines describing a guarantee the field
never got.

**Change.** The `retry_ctx` branch, the ctx field `not_settled_retried`, the retry log lines and
the R5 docstring are gone (-68 lines); the docstring now says the token fails closed and why.
Five retry tests are deleted; a new test sends `NOT_SETTLED` while the ctx is still in
`await_announcement` (the one window where the retry could have fired) and pins exactly one
`bb/throw_at_target` call and the `REJECTED_BB(THROW_ABORTED_NOT_SETTLED)` end. The post-swap
case was already pinned by `test_c3_a_rejection_after_the_announcement_compiled_the_schedule_ends_it`.
Grep (`grep -rnI 'retry_ctx\|not_settled_retried'`) is at zero outside history; the R5 sitting-2
runbook (`tests/hardware/session_skills_r5_sitting2.md`) keeps its "retrying the throw once" line
as the record of that Sitting.

**Behaviour change.** A `NOT_SETTLED` that beats the announcement swap now ends the attempt
instead of re-firing once. No Sitting since 2026-09-30 has shown that ordering. The source-side
fix (tighten Ball Butler's yaw-settle gate, option 3) is filed as its own Item for when the
Ball Butler firmware is next touched.

**Verification.**
- Scoped, 2026-11-01: `python -m pytest tests/ros/test_skill_node.py tests/ros/test_teensy_bridge_node_bb.py -q` -> 421 passed.
- Gate, 2026-11-01: `./run_tests.sh` -> 2 failed, 7894 passed, 52 skipped in 632.80 s at load average ~20 (two sims, a firmware build and headless Chromium beside it); both failures were telemetry-staleness refusals (`test_self_toss_compiles_a_schedule_at_the_owner_operating_point`, `test_TI1_seven_channel_frames_reach_the_wire_and_the_flags_fall`); serial phase 5 passed. Re-run on a quieter box: `pytest -n 2 --dist loadfile tests/ros/test_skill_node.py tests/ros/test_install_segment.py` -> 456 passed.
