---
title: "Per-cycle toss state becomes an object — the node stops being the cycle"
type: refactor
date: 2026-08-26
status: resolved
phase: "toss-pipelined-preamble — Phase B1"
related_plan: toss-pipelined-preamble.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_toss_continuous_node.py
  - tests/ros/test_toss_calibration.py
  - tests/ros/test_toss_ilc_node.py
  - tests/ros/test_toss_trim_node.py
  - tests/ros/test_toss_integration.py
  - tests/ros/test_toss_record_publisher.py
subsystem:
  - ros
tags:
  - refactor
  - toss
  - cadence
---

# Per-cycle toss state becomes an object — the node stops being the cycle

## Summary

`_build_toss_cycle` installed sixteen fields of **node-global** per-cycle state and
`_clear_toss_cycle_state` tore them down. Phase B4 wants two cycles live at once
(the staged preamble overlapping the committed flight), and two cycles on that
layout would silently share every one of those fields — cycle k+1's build would
overwrite the release state cycle k is still flying on.

This lifts them onto a `TossCycleState` dataclass. `_build_toss_cycle` now returns
`(seq, state)`; the state is also installed as the node's one new attribute,
`self._toss_committed`. Every handler on the cycle path takes the state
explicitly (`state=None` ⇒ the committed slot), so the threading is real while the
three paths that have no state to pass — the record builder on the
REJECTED_BAD_GOAL terminal, `_expected_next_cycle_perf` (which the RELOAD builder
also calls), `_toss_trim_snapshot` — resolve the slot instead.

**Zero behaviour change**, and the two places that could have leaked one were
closed deliberately rather than accepted:

* **Teardown clears IN PLACE, it does not drop the object.** `stroke_seen`,
  `track_confirmed` and `record_announce` deliberately survived the old teardown
  and are read by the record builder on the BAD_GOAL path, which runs *after* a
  teardown and *before* the next build. Dropping the object would have changed
  what that (instrument-only) record declares. B4 can switch to dropping in one
  line, on purpose.
* **Five of the plan's listed fields did NOT move.** The `_announced_ball_id`
  pair (`_announced_ball_id` / `_prev_announced_ball_id`) plus
  `_preexisting_flight_ids` are shared verbatim with the RELOAD path
  (`_update_announced_ball_latch`, `_execute_reload`, `_run_one_reload_attempt`,
  `_request_bb_throw`) **and** are read across the cycle boundary —
  `_build_toss_cycle` rolls `_announced_ball_id` into `_prev_announced_ball_id`
  for census D6. A field read across the teardown is by definition not torn down
  with the cycle. Per-slot latching is B4's problem. The other two are
  `_toss_prev_landing_perf` / `_toss_cycle_landing_perf` — the cross-cycle
  arrival boundary, reset per SESSION, already named in the plan's own exclusion
  list. `_announced_id_untagged`, `_ball_possession`, `_catch_vel_scale`,
  `_toss_mocap_body` and `_platform_pos_mm` are node-scoped for the same reasons
  and were never in the plan's twenty-field list.

Lock discipline is untouched: no acquisition added or removed, and the two writes
that were deliberately lock-free (`prepare_pending`, `pretilt_hold_raised`) still
are. The plan's "subscriber callbacks must route to the committed slot" rule
turned out to be a **no-op**: after D1 (2026-08-26) moved the possession latch out
of `_on_balls`, no subscriber callback writes any moved field.

## Verification

Scoped run, 2026-08-27: `python -m pytest tests/ros/ -q -p no:randomly` →
**2415 passed, 1 skipped in 313.32 s**.

Cross-check, 2026-08-27: `python -m pytest tests/motion/ -q -p no:randomly` →
**1938 passed, 5 failed in 319.93 s**. The five are all
`tests/motion/test_ilc_fit.py` and none of them is this change: that file reads
the mined corpus under `temp/probes/` ("newest mine per bag"), and a parallel
session re-mined it at 00:09–00:17 the same morning. Proven rather than assumed —
`test_ilc_fit.py` never imports `reload_coordinator_node`, transitively or
otherwise (checked by importing it with `sys.modules` watched). Fixed in the
mine-flavor commit that follows this one (`_corpus_files` selects within one mine
FLAVOR, so another probe's re-mine can no longer swap the population):
`pytest tests/motion/test_ilc_fit.py -q`, run 2026-08-27 → **66 passed,
3 skipped in 2.36 s**. The full gate is the orchestrator's.

One structural test added,
`test_per_cycle_state_lives_on_the_cycle_object_not_the_node`: it enumerates the
sixteen moved names and asserts `not hasattr(node, name)` both after a cycle is
built and after it is torn down, asserts the seven deliberate exclusions are still
node attributes, and pins the `(seq, TossCycleState)` return and
`node._toss_committed is state`.

Every test edit is one of two classes and nothing else: a **moved-field reach**
(`node._toss_aim` → `node._toss_committed.aim`, 145 sites, mechanical) or a
**stub-signature widen** (a monkeypatched `lambda seq, now, gh` gaining
`state=None` so it still matches the production signature — 36 of them).
No assertion changed.

## Outcome

B1 done; B2 (the release instant becomes an input) is unblocked and depends on
nothing else. The one residual B1 opened was closed the same day, in the
mine-flavor commit that follows this one: `tools/probes/toss_record_miner.py` and
`tests/ros/test_toss_record_miner.py` carried PROSE references to
`_toss_release_state` / `_toss_release_cmd` (comments, not code) and now name
`TossCycleState.release_state` / `.release_cmd`.
