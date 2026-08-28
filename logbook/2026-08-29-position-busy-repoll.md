---
title: "The displaced chain died inside the level-out — POSITIONING met the previous catch's settle hold and took BUSY for a verdict"
type: investigation
date: 2026-08-29
status: resolved
phase: "toss-pipelined-preamble — 2026-08-28 evening sitting debrief, fix 2 of 3"
related_plan: toss-multi-catch-pose.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_record.py
  - plans/active/toss-multi-catch-pose.md
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_toss_continuous_node.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_toss_record_publisher.py
  - tests/motion/test_toss_record.py
  - tests/hardware/toss_trace_recorder.py
subsystem:
  - ros
tags:
  - toss
  - pipeline
  - tier8b
  - cadence
  - performance
---

# The displaced chain died inside the level-out — POSITIONING met the previous catch's settle hold and took BUSY for a verdict

## Summary

Bag `2026-08-28_23-53-25`. **Every displaced `toss_continuous` chain aborted
after cycle 1**, `ABORTED_CYCLE_REJECTED_POSITION(BUSY)`, while the operator
watched the platform visibly levelling out. Three occurrences, deterministic.

The refusal is `trajectory_node`'s and it is **correct**: cycle 1's catch plan
ends in a `JB_TRAJ_CATCH_SETTLE_HOLD_S` = **0.500 s** quiescent hold, during which
`_active_move_in_flight()` is True and any `go_to_pose` is refused `BUSY`. Cycle
2's POSITIONING dispatched at **landing + 0.234 s** — **0.266 s inside** that
hold. The toss FSM asked once and terminalised the answer.

The fix is a **bounded re-poll**, not a widened guard and not a relaxed hold:
`TOSS_POSITION_BUSY_PATIENCE_S = 0.54 s` of patience
(`_absorb_position_busy`), re-emitting the service call every
`TOSS_POSITION_BUSY_REPOLL_S = 0.10 s`, conjunct with the release lead so the
wait is never paid out of the throw window. A BUSY that outlives the patience
rejects with the same code, on the same path, byte-identical.

*Built under the standing orchestration pattern: a read-only Opus diagnosis from
the bag, a build agent, and one independent audit over the whole three-fix wave
(verdict: no blocking findings; 5 findings, 3 applied). The owner was AFK and the
work was pre-authorised. The fix is unflown — see Open questions for the one
record field that will confirm or re-cut it.*

## Symptom

Operator report, from the debrief: displaced sessions never got past the first
cycle. The abort landed *while the platform was still reorienting* — the level-out
after the aimed throw — and the machine said `POSITION(BUSY)`, which reads as "the
platform is busy" and was met with the reasonable objection that of course it is,
so wait.

## Diagnosis

### D1 — the seam, in one line of timing

A chained cycle's POSITIONING must command a move (the displaced case: A ≠ B), and
it dispatches into the tail of the plan the *previous* cycle installed:

| event | t (relative to cycle-1 landing) |
| --- | --- |
| cycle 1's ball lands; the catch plan enters its quiescent settle hold | 0.000 |
| cycle 2's POSITIONING dispatches `go_to_pose` | **+0.234** (0.09–0.23 s dispatch lag across the sitting) |
| `trajectory_node` answers `BUSY` — `_active_move_in_flight()` is True | +0.234 |
| the settle hold expires and the platform is free | **+0.500** |

The overlap is **0.266 s** on the convicted cycle. The FSM's `_step_positioning`
dispatched once, read the refusal, and minted `REJECTED_POSITION(BUSY)`; the
session composed it into `ABORTED_CYCLE_REJECTED_POSITION(BUSY)` and stopped.

### D2 — why `throw_delay_s` could not have protected it, at any value

This is the part worth spelling out, because the operator had already tried the
obvious remedy — 5 s of delay — and it bought nothing. The next cycle's start is

> next cycle START = landing + dwell − throw_delay

so **raising `throw_delay` moves POSITIONING EARLIER**, deeper into the settle
hold. It has the wrong sign. Raising `dwell` alone is the right sign, but raising
*both* is pinned: `required_dwell_s` derives the dwell floor from the same
`throw_delay`, and the pair is held to a **~0.09–0.12 s** handoff gap. There is
no goal the operator could have typed that clears a 0.5 s hold with the shipped
floors — the seam is structural, and the 5 s of delay made it slightly worse.

### D3 — why co-located chains never saw it

Same bag, same session shape, `COMPLETED 5/5` at dwell **0.52–0.69**. A
co-located chain takes the census-B1 positioning **skip**: the platform is already
at the pose, so the FSM never calls `go_to_pose` at all. It is not that the
co-located chain waits better; it never asks the question. That also means the
protection is accidental — under `plans/active/toss-multi-catch-pose.md` M3's
pose rings, where every cycle commands a re-orient, **the seam becomes
universal**.

### D4 — what the evidence did not support

* **not the guard being wrong.** `trajectory_node` refuses because a plan really
  is installed and running its post-arrival hold; superseding it would supersede
  a ball-in-cup settle through the optimistic analytic gate. The guard is the one
  correct actor in the whole trace.
* **not the stale-site class** fixed the same week
  ([[2026-08-28-displaced-chain-stale-site]]): that one threw from the wrong
  site; this one never reaches a throw at all.
* **not a cadence-lead shortfall.** `ABORTED_CANT_MAKE_RELEASE` is the other
  displaced-chain terminal on the board and it is *not* what these three cycles
  minted — which turns out to matter, because a careless fix converts one into
  the other (see the Discussion).

## Discussion

### Why the bound is the settle hold, and not either watchdog already in the file

Two existing timeouts were candidates and both are wrong by an order of
magnitude. `TOSS_POSITIONING_TIMEOUT_S` is **6.0 s** and the session stall
watchdog is looser still; either would "fix" the bug by waiting out anything at
all. The cost is not latency, it is **misattribution**: a BUSY that outlives the
settle hold is a *wedge* — a stray `go_home`, a SpaceMouse nudge, a guard latch —
and laundering it through a 6 s wait renames it `ABORTED_POSITION_TIMEOUT`, which
routes the operator to the "no response" branch of the ladder and, at the node,
fires the zombie-move superseder's `go_home` for a platform that never moved.

So the patience is exactly the interval the machine can *explain*:

> `TOSS_POSITION_BUSY_PATIENCE_S` = `JB_TRAJ_CATCH_SETTLE_HOLD_S` (0.50) +
> `NODE_LOOP_PERIOD_S` (0.040) = **0.54 s**

— the one plan that can legitimately hold the platform at the head of a chained
cycle, plus the tick grid the FSM polls on. It is cross-read from the generated
config by a drift guard
(`test_busy_patience_matches_the_settle_hold_plus_one_loop_period`), because a
YAML change to the hold must move this bound with it: a patience *shorter* than
the hold re-opens the abort class on the very next sitting.

Re-polls are spaced `TOSS_POSITION_BUSY_REPOLL_S = 0.10 s` rather than run on
every tick because `go_to_pose` is a **blocking round trip inside the node
loop** — the bag's `tick_max` on these cycles is 320–337 ms — so re-emitting at
tick rate would multiply the very cost the wait exists to absorb.

### The conjunct that stops the fix from being a worse bug

A wait costs time, and the time it costs is time the release window needs. Without
a guard, this fix converts a **`REJECTED_POSITION(BUSY)` — nothing moved, nothing
armed** — into an **`ABORTED_CANT_MAKE_RELEASE` with the hand committed and a
retract under a seated ball**: a strictly worse terminal for the same fault, and
precisely the standing cadence fence of
[[2026-08-28-displaced-chain-stale-site]] § Open questions (`_step_preparing`'s
release guard has **no slip**). So `_absorb_position_busy` checks, from the
**first** BUSY and not only at the bound:

```python
if (self._t_release - now) <= min_throw_delay_for_release_s(
        self.event_vel_mps, True, float(self.min_event_delay_s)):
    return None          # reject now; never spend the release window
```

That is the **same** derivation `_build_toss_cycle` charges at accept time, on
the mover branch — which is the only branch a BUSY can arise on, since a
census-B1 skip never calls the service. Accept-time and runtime therefore cannot
disagree, and a re-cut of the floor moves both.

### The one-liner that was tempting and is wrong

The cheapest conceivable fix is a session-level floor:
`next_at = max(next_at, landing + 0.5)`. It is one line, it needs no new
constant, and it would have cleared every abort in this bag. It is rejected
because it charges **every** cycle 0.5 s for contention that only the displaced
ones have — and the same bag proves co-located chains run clean at dwell
0.52–0.69 with no absorb at all. Paying the shipped co-located cadence half a
second per cycle to fix a case it does not have is the wrong trade, and it hides
the seam instead of measuring it.

### The audit finding, accepted as instrument-first

The audit observed that the patience deadline is anchored at the **dispatch
tick** (`now + TOSS_POSITION_BUSY_PATIENCE_S`, stamped where
`ACTION_POSITION_PLATFORM` is emitted), so the first blocking round trip is
charged *against* the patience rather than added to it. The absorbable residual
is therefore about `patience − spacing ≈ 0.44 s`, and today that is comfortably
covered: the observed dispatch lag is 0.09–0.23 s, leaving at most
`0.50 − 0.09 = 0.41 s` of hold to absorb. Accepted as-is and **instrumented
rather than pre-corrected** — `position_busy_polls` says whether the assumption
holds. The exact re-anchor (move the deadline to the first BUSY *observation*) is
noted in the audit and is a two-line edit if the corpus says so.

### Relation to the plan: this is the absorb, not the budget

`plans/active/toss-multi-catch-pose.md` § 2.7 was amended this session (M2's
obligation): **the moving leg's dwell floor owes the previous catch's settle
hold**, `+ JB_TRAJ_CATCH_SETTLE_HOLD_S`, anchored at the **committed arrival**.
That is the *budgeted* answer — it makes the cadence honest before the cycle is
built. This entry is the *bounded absorb* for the residual a budget cannot
pre-pay, the same division of labour § 2.8 already draws. Under M3's rings the
plan text now says so explicitly: the seam is universal, not displacement-only.

**Do not "fix" `trajectory_node`'s guard.** It protects a ball-in-cup settle from
being superseded through the optimistic analytic gate. Every version of this
investigation that started with "just let the move through" ends with a
supersede during a settle.

## Fix

**`toss_sequencer.py`**

* `_POSITION_BUSY_CODE = 'BUSY'` — named rather than inlined, because the string
  is a **wire value** shared with a node this pure module deliberately does not
  import;
* `_step_positioning` routes **only** that code into `_absorb_position_busy`
  (`test_only_busy_is_re_polled` parametrises the rest: every other refusal names
  a state that will not clear on its own);
* `_absorb_position_busy(now)` returns `None` ⇒ the caller rejects (release-window
  conjunct, or patience spent); a HOLD decision (`ACTION_NONE`) between polls; or
  a re-emitted `ACTION_POSITION_PLATFORM`. Between polls the BUSY result
  deliberately **stays in place** — clearing it there would drop the tick into
  the "no answer yet" branch and wait out `_positioning_deadline` instead. The
  result is cleared at exactly one instant: the tick that re-emits, in the same
  tick the node re-dispatches in;
* `_position_busy_deadline` is stamped at **both** POSITIONING dispatch sites.

**`reload_coordinator_node.py`** — one INFO line on the **first** re-poll only
(`position_busy_polls == 1`), naming the hold, the spacing and the bound, so an
operator watching the console sees the seam live and a completed session with
absorbs is distinguishable from one that never met them.

**`toss_record.py`** — two **additive** fields, no schema bump:
`position_busy_wait_s` (HOW LONG) and `position_busy_polls` (HOW MANY), the same
split `commit_slip_s` / `commit_slips` makes for the commit gate. Both are
recorded **even when zero**, because a zero is a measurement ("this cycle was
never refused") and a distribution that drops its zeros cannot say how often a
chained cycle meets the hold.

**`tests/hardware/toss_trace_recorder.py`** — `REJECTED_POSITION` joins
`REJECT_WIRE_MAP` with its subcode ladder, and the `BUSY` gloss now says the FSM
already re-polled: a `BUSY` that reaches the operator is a hold that **outlived**
the settle tail, so check the cadence, not the guard.

## Verification

* Scoped (`pytest tests/ros/test_toss_sequencer.py tests/ros/test_toss_coordinator.py
  tests/ros/test_toss_continuous_node.py tests/ros/test_toss_record_publisher.py -q`,
  run 2026-08-29): **550 passed in 104.07 s**.
* THE GATE: full gate (`./run_tests.sh --full`, run 2026-08-29): parallel **6648 passed, 4 skipped, 3 xfailed in 535.28 s**; serial **9 passed in 42.17 s**; total 584 s — RESULT: PASS.
* The regression pair that reproduces the sitting and then clears it:
  `test_a_chained_cycle_absorbs_the_previous_catchs_settle_hold` (the fixed
  behaviour) and `test_without_the_patience_the_same_script_aborts_the_session`
  (the same script with the patience removed still aborts — so the test cannot
  pass vacuously on a script that never met a BUSY).
* FSM-level: `test_positioning_busy_re_polls_and_succeeds`,
  `test_positioning_busy_past_the_patience_bound_rejects`,
  `test_positioning_busy_never_eats_the_release_lead`, `test_only_busy_is_re_polled`,
  `test_busy_re_polls_are_spaced_by_the_repoll_constant`, and the drift guard
  `test_busy_patience_matches_the_settle_hold_plus_one_loop_period`.
* Node + record: `test_position_busy_redispatches_the_go_to_pose`,
  `test_the_busy_absorb_reaches_the_record`.

## Outcome

A displaced chain now absorbs the previous catch's settle hold instead of dying
in it, and it does so with a bound the machine can explain: 0.54 s, the hold plus
a tick. Everything past that bound still rejects by name.

**Operationally, a displaced chain no longer needs the `dwell − throw_delay >
0.55 s` guard** the debrief had converged on as a workaround — that number was
the settle hold plus a margin, hand-held by the operator at goal-composition
time, and it is now the machine's own business.

⚠ **Deploy**: `cd ros_ws && colcon build --packages-select jugglebot` before the
next sitting — the launch runs the install space, so this and its two siblings
are inert until it is rebuilt (one build covers the whole wave;
`jugglebot_interfaces` is untouched).

Sibling fixes from the same debrief: [[2026-08-29-bb-reload-busy-patience]] and
[[2026-08-29-rejection-message-enrichment]] — the latter is why a
`REJECTED_POSITION(BUSY)` that *does* reach the operator now carries
`trajectory_node`'s own sentence after the subcode.

Unflown. The standing fence from [[2026-08-28-displaced-chain-stale-site]]
(no cadence-lead displaced sittings until M2's honest budget) is **unchanged** by
this fix: it is about the *arrival* outrunning `pre_dispatch_budget_s(True)`, a
different seam one rung later.

## Open questions / follow-ups

* **the one number to read off the first displaced sitting**: if the toss records
  show `position_busy_polls` stuck at **1** with `position_busy_wait_s` ≫ 0.10 s,
  the dispatch-tick anchor is being eaten by the blocking round trip and the
  deadline should be re-anchored at the first BUSY observation (the audit records
  the exact edit). Polls of 2–5 with waits under ~0.3 s is the healthy shape.
* **M2 § 2.7's budgeted floor** is still owed; this absorb does not discharge it,
  and under M3's rings the budget is what keeps the absorb from being load-bearing
  every cycle.
* **the blocking `go_to_pose` call** remains the sitting's loop-overrun source
  (banked in [[2026-08-28-displaced-chain-stale-site]]); the re-poll spacing is
  sized around that cost rather than removing it.
