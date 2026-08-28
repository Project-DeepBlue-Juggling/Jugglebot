---
title: "A busy BallButler is a transient, not a fault — two instant refusals become bounded waits"
type: bugfix
date: 2026-08-29
status: resolved
phase: "toss-pipelined-preamble — 2026-08-28 evening sitting debrief, fix 1 of 3"
related_plan: toss-selftuning.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/reload_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - tests/ros/test_reload_sequencer.py
  - tests/ros/test_toss_continuous_node.py
subsystem:
  - ros
tags:
  - bugfix
  - toss
  - reload
---

# A busy BallButler is a transient, not a fault — two instant refusals become bounded waits

## Summary

Operator debrief after the 2026-08-28 evening sitting (bag
`2026-08-28_23-53-25`), request 1 of 3: *a repeated RELOAD must wait for BB to be
ready instead of failing on BB_BUSY.* Two independent sites read
`/bb/heartbeat`'s state **once** and refused on the sample — the reload FSM's
`_step_checking` (`REJECTED_BB_BUSY`) and rung 2 of the node's interlude gate
(`STOPPED_BB_NOT_READY`) — so a BallButler cycle that ends on its own in a second
or two could terminalise a whole `toss_continuous` session. Both sites now wait,
bounded, for `BB_READY_WAIT_S = RELOAD_TIMEOUT_S` = **10.0 s**.

*Built under the standing orchestration pattern: a read-only Opus diagnosis, a
build agent, and one independent audit over the whole three-fix wave (verdict: no
blocking findings; 5 findings, 3 applied). The owner was AFK and the work was
pre-authorised. Nothing here has been on hardware yet.*

## Problem

The two refusals are a TOCTOU race with a 2–4 s window, and the reload interlude
walks straight through it:

1. the interlude gate reads the freshness-gated heartbeat snapshot and BB is
   **IDLE** — admitted;
2. the recentre runs (`go_home` + the verified-arrival pad), **≥ 2 s**;
3. the reload FSM's own CHECKING re-reads the same heartbeat, BB has meanwhile
   started a cycle of its own, and the single sample is now **not IDLE** →
   `REJECTED_BB_BUSY`.

Nothing was wrong with either read. Both were honest about their instant; the
defect is that a *transient* state was given a *terminal* verdict. `RELOADING`
and `THROWING` both end by themselves, and the price of not waiting them out was
the sitting.

## Fix

`BB_READY_WAIT_S` is **aliased** to `RELOAD_TIMEOUT_S`, not typed again:
that constant already *is* this project's validated bound for BB's
`RELOADING → IDLE` cycle, and two hand-written copies of one number is how a
re-measure lands in only one of them.

**`reload_sequencer._step_checking`** — a non-IDLE BB returns
`ReloadDecision(PHASE_CHECKING, ACTION_NONE, …)` until `_bb_ready_deadline`
(stamped in `start()`), then rejects `BB_BUSY` exactly as before. The rung sits
**before** `ACTION_PRIME_HAND`, so the quiescent-refusal property holds for the
whole wait and the deadline reject is still `ACTION_NONE` — nothing is commanded
and nothing is armed while the patience runs. `bb_ready_wait_s = 0.0` reproduces
the pre-2026-08-29 instant reject bit for bit (the deadline is `now` at
`start()`), which is what
`test_zero_patience_reproduces_the_instant_reject` pins.

**`reload_coordinator_node._wait_for_bb_ready`** — the interlude gate's rung 2,
polling `_build_observations` at the ambient `_TICK_S`. It commands nothing.
Two states stay **immediate**, because patience cannot fix either: `bb_connected`
false is a dead link (waiting on a publisher that is not publishing buys
silence), and `BB_STATE_ERROR` is a fault that needs the operator, so 10 s only
delays the message. A cancel is honoured throughout — nothing is armed, nothing
is airborne, and the deferral discipline exists for a committed throw, not for a
wait. The refusal log line now carries `_bb_state_detail()`, because
`STOPPED_BB_NOT_READY` alone cannot say whether BB is disconnected, faulted, or
merely still busy past the patience: three different fixes behind one code.

Both ceilings charge the patience rather than being outrun by it:
`_reload_interlude_budget_s` gains `BB_READY_WAIT_S` per attempt (**113.4 s →
143.4 s** at the sitting's goal shape, and the session ceiling with it, **270.9 s
→ 300.9 s**), and `_sequence_deadline_s` gains
`getattr(seq, 'bb_ready_wait_s', 0.0)`.

## Discussion

**An attempt may legitimately spend the patience TWICE, and that is deliberate.**
The gate waits BB out, the recentre runs, and then the FSM's CHECKING may wait it
out again — the same race, one rung later. Collapsing the two into a single
budget would mean the FSM trusting the gate's read across a recentre, which is
the very assumption that failed. Both charges are inside the ceilings above, and
the worst case is bounded: **+30 s** on a session at `max_reloads 3`.

**A wedged BB now fails 10 s slower, under the SAME outcome names.** No new code
was minted for the timeout path — `REJECTED_BB_BUSY` and `STOPPED_BB_NOT_READY`
still name it — so no runbook row, no `REJECT_WIRE_MAP` entry and no operator
decoder changes. The tradeoff is accepted knowingly: a genuinely stuck BB is a
sitting-ending fault either way, and 10 s of latency on the *rare* terminal buys
the *common* transient.

**Tripwire for whoever re-measures `RELOAD_TIMEOUT_S`.** The alias makes that
constant count **twice** inside `_sequence_deadline_s` (its own term plus
`bb_ready_wait_s`). With the FSM's defaults and the guard test's `throw_delay 3.0`
the budget is 10 + 10 + 3.0 + 0.5 + 0.56 + 5.0 = **29.06 s** against the 30.0 s
`_MAX_SEQUENCE_S` floor — **0.94 s of headroom**, consumed at 2 s per second of
re-measure, so a `RELOAD_TIMEOUT_S` above **≈ 10.47 s** flips the first assertion
of `test_sequence_deadline_never_lands_inside_the_flight_window`. That is the
test telling the truth (the ceiling stops being the floor), not a test to relax.

**Banked adjacent defects, named rather than fixed.** Both were found while
reading the same path and are recorded so the next session does not re-find them:

* `bb_connected` still refuses **instantly** on a single dropped heartbeat inside
  the 0.5 s (`_HEARTBEAT_STALE_S`) staleness window — the same one-sample shape
  this entry closed for `bb_state`, one field over;
* `_call_reload()`'s `Trigger` return is **discarded**, so a failed RPC does not
  surface as a failed reload: it surfaces 10 s later as `REJECTED_NO_BALL`, with
  the wrong name on the wrong subsystem.

## Verification

* Scoped (`pytest tests/ros/test_reload_sequencer.py
  tests/ros/test_toss_continuous_node.py tests/ros/test_reload_coordinator_node.py
  tests/ros/test_reload_integration.py -q`, run 2026-08-29):
  **309 passed in 20.54 s**.
* THE GATE: full gate (`./run_tests.sh --full`, run 2026-08-29): parallel **6648 passed, 4 skipped, 3 xfailed in 535.28 s**; serial **9 passed in 42.17 s**; total 584 s — RESULT: PASS.
* New regressions, both sites: `test_a_busy_bb_is_waited_out_and_then_the_sequence_proceeds`,
  `test_nothing_arms_while_the_bb_patience_runs`,
  `test_a_bb_busy_through_the_whole_patience_still_rejects`,
  `test_a_bb_error_mid_patience_pre_empts_the_wait`,
  `test_leaving_the_control_mode_mid_patience_pre_empts_the_wait`,
  `test_zero_patience_reproduces_the_instant_reject` (FSM);
  `test_gate_waits_a_busy_bb_out_before_calling_it_not_ready`,
  `test_gate_admits_a_bb_that_becomes_ready_inside_the_patience`,
  `test_gate_refuses_immediately_when_bb_cannot_become_ready`,
  `test_a_cancel_during_the_bb_wait_is_honoured_before_anything_is_committed`,
  `test_an_interlude_entered_while_bb_is_busy_still_reloads` (node), plus the two
  ceiling pins `test_the_interlude_budget_charges_the_bb_patience_per_attempt`
  and `test_the_sequence_ceiling_covers_the_fsm_bb_patience`.

## Outcome

A repeated RELOAD now survives a BallButler that is mid-cycle, which is the
common case on a chained sitting rather than an exotic one. The two immediate
refusals that remain are the two that patience cannot help. **Unflown**: the next
`on_empty_cup: RELOAD` sitting is the first evidence, and the thing to look for
is a reload that *completes* after a visible pause rather than one that never
starts.

⚠ **Deploy**: `cd ros_ws && colcon build --packages-select jugglebot` before the
next sitting — the launch runs the install space, so this and its two siblings
are inert until it is rebuilt (one build covers the whole wave;
`jugglebot_interfaces` is untouched).

Two of the three fixes from the same debrief are [[2026-08-29-position-busy-repoll]]
(the displaced chain's `REJECTED_POSITION(BUSY)`) and
[[2026-08-29-rejection-message-enrichment]] — the latter also touches this file,
giving `REJECTED_CANT_MAKE_LEAD` and `REJECTED_NOT_CENTERED` their numbers.

## Open questions / follow-ups

* the two banked defects above (`bb_connected` single-sample; the discarded
  `_call_reload()` return) — neither is in this fix's scope and both are one-line
  reads away from being real;
* the 10 s patience is the *validated BB cycle bound*, not a measured
  busy-duration distribution. If a sitting's records ever show waits clustering
  near the bound rather than near 1–2 s, the number to re-cut is
  `RELOAD_TIMEOUT_S` — and the tripwire above says what that costs.
