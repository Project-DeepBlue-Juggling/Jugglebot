---
title: "UH-7a steady-chain plumbing — the ring machinery was already written and inert, so the work was a shape change and a release hand-off; the beat stops being a choreography artefact and becomes the planner's own window"
type: feature
date: 2026-09-07
status: in-progress
phase: "unified-7dof-planner — Phase 5 (UH-7a)"
related_plan: unified-7dof-planner.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/toss_session.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - tests/hardware/toss_trace_recorder.py
  - tests/motion/test_unified_cycle.py
  - tests/ros/test_outcome_detail.py
  - tests/ros/test_trajectory_hold_preempts_solve.py
  - tests/ros/test_unified_cycle_integration.py
  - tests/ros/test_unified_ring.py
  - tools/probes/cadence_rung_check.py
  - tests/hardware/session_unified7_cycle_ladder.md
  - plans/active/unified-7dof-planner.md
  - logbook/2026-09-07-unified-7dof-uh7-steady-chain-plumbing.md
  - logbook/INDEX.md
subsystem:
  - ros
  - motion
  - tools
tags:
  - safety
  - performance
  - testing
---

# The ring: one plan, one clock, and a beat the planner owns

## Summary

**UH-7a — a single-pose unified session at a constant beat — is code-complete and NOT FLOWN.**

A unified session now installs `LAUNCH + STEADY` on cycle 1 instead of `LAUNCH + LANDING`. Each cycle
plans the *next* cycle's window inside its own, once the live catch has passed, and the last cycle
chains a `LANDING` so the machine comes to rest. The whole settle-plus-relaunch tail comes off the
beat: the structural floor of the serial choreography was **~2.5–2.7 s at flight 0.8** (flight + 0.6 s
settle + a 0.5–0.606 s hardware solve + a 0.6 s launch window + the FSM preamble), and the ring's floor
is **the beat itself**, floored by what the planner will actually serve — the sim gate records
`LIMIT_JERK` at a 1.0 s window against the 150 k session cap and carries the whole ring at 1.4 s.
Enforced at goal acceptance rather than discovered mid-session: a chain dwell floor of **0.800 s**, a
cycle-1 `throw_delay_s` floor of **0.866 s**, and the legacy dwell floor on top — a **minimum beat of
≈1.81 s at flight 0.80 and ≈1.68 s at the 0.639 s flight of `throw_height_m 0.5`** (§ Fix 8).

Three owner decisions shaped it (2026-09-07): **all three replan-mechanism changes land**, in **one
atomic commit**, and the ring **holds immediately on a MISS**. The **single-pose** ring flies first;
the two-pose ring (UH-7b) sits behind the aim-authority derivation *and* a goal-surface change, because
`TossContinuous` has exactly one catch site.

Four things this change is, in order of how much they cost to get right:

- **A shape change, not new machinery.** The chaining primitives were all written, all commented
  *"Phase 5's UH-7 ring"*, and all inert — for exactly one reason. See § Diagnosis 1.
- **The replan policy kept alive on a ring.** On a chained plan `catches[0]` is spent from the second
  window on, so every tracker landing update would have answered `REPLAN_WINDOW`; and
  `replan_tail`'s mid-plan-release detach bound was skipped on exactly the release-terminal shape
  UH-7a ships, leaving a spent release's detach cone open to a splice (**1.126 m/s²** of off-axis
  specific force delivered to an airborne ball, invisible to `validate_cycle`).
- **Two gates bounded so the beat can be short.** `extend` re-gated the *whole* joined plan and
  `validate_cycle` is ~89 % of the solve: **220 → 1017 ms** across six chained windows. Bounded to the
  seam plus the new window it is **flat at 163 ms** (58 knots at every depth). `replan_tail`'s re-gate
  was **399 ms at 137 knots — already past its own 0.30 s commit lead** — and is now 136–155 ms
  (43 knots at every depth, one wider than the extend's for the reason in § Discussion (h) A3).
- **A hold that cannot queue behind a solve.** The ladder's last rung is `trajectory/hold`, and
  `trajectory_node` spun single-threaded, so a hold arriving during a 307–314 ms solve waited for it.
  `trajectory/hold` now has its own reentrant callback group, `main()` runs a two-thread executor, and
  an install epoch refuses the pre-hold solve `SUPERSEDED_BY_HOLD`. Measured hold latency **0.8 ms
  alone, 1.0–4.4 ms during a live solve**.

Nothing was flown. The next sitting is the first evidence that any of it moves a real machine.

---

## Motivation

The 2026-09-07 re-fly flew UH-6 clean, with one observation:
`dwell_time_s` / `throw_delay_s` below their defaults produced `ABORTED_NO_RELEASE` on the affected
cycles (`logbook/2026-09-07-unified-launch-refused-below-floor-seed.md`, Open Question 5). That is
expected under the serial per-cycle choreography and it is also dishonest: the **accept-time** gates
imply a floor of roughly `dwell ≥ 0.55 s` at flight 0.8, i.e. a beat of ~1.35 s, while the
**structural** floor is 2.5–2.7 s. Nothing refuses the difference, so it surfaces mid-session as an
abort with a ball unthrown.

The plan's own item 6 named the blocker: *"UH-7's constant beat needs a `release_at_perf` hand-off that
does not exist yet."* That premise turned out to be half right, and the half that was wrong changed the
work.

---

## Diagnosis

### 1. The chaining machinery was complete, tested, and inert — for one reason

`_tick_unified_extend`, `_extend_unified_cycle`, `_unified_beat_s`, `_toss_unified_chain` and the
`CHAIN_SKEW` guard were all written and all reachable in tests. They never fired in production because
`_tick_unified_launch` installed a **rest-terminal** `LAUNCH + LANDING` join, and
`_tick_unified_extend` returns immediately on a rest-terminal plan. `trajectory_node`'s "first release
still ahead of now" search — written explicitly so an `EXTEND` does not report a spent release — had
never run either. So had `MODE_EXTEND`, `KIND_STEADY`, `uc.plan_steady`, `latest_supersede_time_s`'s
finite branch, `_check_supersede_deadline`, and `replan_tail`'s release-terminal branch.

The hand-off genuinely was missing: under unified every cycle ran the serial branch with
`release_at_perf = 0.0`, so `_t_release = START_CYCLE_poll_instant + throw_delay_s` — a release derived
from *when the poll happened* rather than handed to the cycle. But "the chaining code does not exist"
was never true. UH-7a was therefore a *shape* change (chain a STEADY, not a LANDING) plus an absolute
release, against an estimate that had assumed new machinery.

### 2. The replan policy would have been dead code on a ring

`_replan_cycle_from_target` always used `meta.catch_site_mm` / `catch_vel_mm_s`, i.e. `catches[0]`, and
passed `catch_frac = 0.0` so `replan_tail` read `meta.catches[0].t_s` too. `extend` pins the joined
plan's `catch_k` to the **first** catch permanently. On a `LAUNCH + STEADY` join that is still the live
catch and everything works; from the **second** chained window on it is spent, the splice knot is past
it, and `replan_tail` refuses `REPLAN_WINDOW` — *"the catch is inside the committed head, nothing is
left to re-aim"*. Loud, never dangerous, and exactly the dead-code shape the 2026-09-05 rest-terminal
generalisation was written to end.

`t_catch_mono` and `arm_lead_s` had the same defect one field over: both read `catches[0]` unguarded,
where the release side already had the still-ahead search.

### 3. The detach bound was skipped on precisely the shape UH-7a ships

`replan_tail` guarded the mid-plan-release detach check with `if not release_terminal`, and the
remaining `k_s <= n_detach` check only covers the plan's *first* release. A `LAUNCH + STEADY` join is
release-terminal **and** carries a spent release in its middle, so neither check protected that
release's detach cone. Measured before the fix, at session limits: a replan at `t_now = 0.00` (k_s 12)
and at `t_now = 0.30` (k_s 24) were **both accepted** and both returned `releases = [2.0]` — the 0.6 s
throw silently deleted from a plan the emitter was streaming, plus the detach-cone rows re-solved away.
After: both refused `REPLAN_WINDOW … release knot 24 + detach knots 26`, while `t_now = 0.40` (k_s 28)
still accepts — so the bound is a window, not a floor.

### 4. Chaining a STEADY re-opens the supersede cliff, on purpose

A release-terminal plan streamed to its last knot **commands a stop at the throw**: the emitter's τ+dt
sample lands on the terminal hold, so the transmitted `v1` collapses from **93.011 rev/s to 0.0** —
0.3445 rev = **10.90 mm** of slider error, inside `MAX_LEAD_HAND_REV` 2.0 and
`MAX_DEVIATION_HAND_REV` 2.5, so no firmware guard fires and the only symptom is a throw that went
somewhere else. Joining a LANDING is what closed that class in Phase 4. A ring re-opens it deliberately
and arms, for the first time in production, the two mitigations that were kept for it:
`supersede_deadline_mono` on the response and `trajectory_node`'s once-per-install alarm.

### 5. The gates were the cost, and one of them was already over its own budget

`extend` re-ran `validate_cycle` over the whole joined plan, which grows 81 → 137 → 193 → 249 → 305 →
361 knots over six windows. Measured whole-plan gate: **219.7 → 1016.8 ms**, still climbing.
`replan_tail` was worse in kind rather than in degree: **399 ms on a 137-knot ring, already past
`_CYCLE_REPLAN_LEAD_S` = 0.30 s**, so from the second chained window on every tracker landing update
would have been solved for a splice knot the emitter had passed and refused `STALE_STATE` at install —
the same dead replan policy as § 2, arriving by a different road.

---

## Discussion

### (a) Single pose first, and why the two-pose ring is three decisions and not one

The brief for the survey assumed UH-7 meant the sim gate's SET 2 ring. It does — SET 2 *is* a two-pose
ring (60 mm of x displacement, alternating), which corrected the framing this work started from. But
nothing in the chaining machinery is aim-dependent: `plan_steady` accepts `throw_site == throw_target`
exactly as `plan_launch` does, the coordinator's chained request already names one site for both, and
the sim's 60 mm displacement is what drives the *worst* mirror residual — so a single-pose ring is
strictly easier on the machine. It simply does not exercise everything SET 2 measured.

A two-pose ring needs **three** things, and only one of them is the one everybody names:

1. the **aim-authority re-derivation** against `tilt_geometry.MAX_TILT_DEG` (12°), before
   `REJECTED_UNIFIED_AIM_UNSUPPORTED` can be lifted and `throw_target_mm = throw_site_mm` unpinned;
2. a **goal-surface change** — `TossContinuous` carries exactly one `catch_position`, and its own field
   comment calls a per-cycle waypoint list *"the obvious v2 and explicitly OUT OF SCOPE here"*. There is
   nowhere on the wire to *say* "two poses";
3. `toss_sequencer`'s `REJECTED_DISPLACEMENT` bound deleted or unified-branched — it measures a
   platform A→B reach the unified plan never performs (the plan owns the whole traverse), and it is what
   refused 2026-09-06's goal 3 for a 100.2 mm displacement against a 66.0 mm bound on a motion that does
   not exist.

Splitting UH-7a from UH-7b is therefore not caution, it is a scope statement: the chaining risk and the
aim risk are independent, and flying them together would leave a refusal ambiguous between them.

### (b) The extend is non-blocking, and that is a safety property rather than a latency one

The coordinator's first cut kept the existing blocking `_call_plan_cycle` (a 5 ms poll bounded by
`_SERVICE_WAIT_S` = 2.0 s). Subscriptions keep ingesting during it, so nothing is *lost*; what stalls
is the **FSM tick**, for the whole solve — measured `MODE_EXTEND` 354–387 ms idle, 1.06 s under load.
Since the extend now fires at the catch, that stall sits exactly between the catch and the verdict.

**A stalled FSM cannot observe a MISS.** And a MISS on a ring is the one state where the correct action
is not to chain but to *stop the plan*, because the standing plan's last act is a throw stroke over a
cup the ball just missed. At a dwell near the floor, a hold issued after a 354–387 ms stall arrives
**after the chained stroke has begun** — which is precisely the empty-cup stroke the whole fail-safe
ladder exists to prevent. So the latency argument ("the stall is charged against the dwell, so the
practical floor is ~0.55 s idle and ~1.2 s loaded") is real but secondary; the safety argument decides.

Shipped: `service_is_ready()` + `call_async`, a `_UnifiedChainCall` record, and a per-tick poll bounded
by the same `_SERVICE_WAIT_S`. There is deliberately **no** `wait_for_service` — a 2 s readiness wait
inside a live cycle is the same defect one layer up. The one place it still blocks is
`_settle_unified_extend`, run right after `_run_toss_cycle` returns: the FSM has terminalised, so there
is nothing left to observe, and leaving the request pending would take the ring into the next cycle
with no window chained and a standing release-terminal plan — a fresh `MODE_NEW` LAUNCH refused
`STALE_STATE` while the old plan threw anyway.

Cost of the conversion, stated because it is the kind of thing that gets lost: two integration tests
stubbed `_call_plan_cycle` with a one-positional-arg lambda and asserted the call count. Both now stub
`node._plan_cycle_cli` with a future that never resolves — which is what "dispatched and unanswered"
actually looks like — and every assertion in them is unchanged.

### (c) A refused STEADY stops the session; it does not degrade

When a mid-ring STEADY is refused, the ladder installs a LANDING instead. Left alone that is
*physically* fine: the ball is caught, the cup settles at the floor, and the session's next cycle plans
a genuine `MODE_NEW` LAUNCH from a stopped machine. It also silently reverts the beat to the
settle-plus-relaunch cadence for the rest of the sitting.

That is the wrong behaviour for a **measurement** rung, and the reason is not safety. The achieved
cadence, the per-cycle catch error and the ILC corpus are all read *per session*, and a corpus with two
cadences in it is a corpus with two machines in it. The operator's correct response to a refused window
is to raise `dwell_time_s` and re-run — and that is only obvious if the session says so by name. So a
fall-back LANDING arms `STOPPED_CHAIN_REFUSED`, carrying the planner's own refusal string, and the
session stops (owner, 2026-09-07).

Exactly **one more cycle runs**, deliberately: its ball is already airborne when the refusal is known,
and the fall-back window is what catches it. The terminal is therefore armed at the *start* of that
cycle and consumed by its own `note_cycle_result`, which is where every other session terminal is
minted.

### (d) The gates are bounded by range, not by trimming the head

The obvious way to stop `extend` re-gating a growing plan is to drop the committed head off the
installed plan and re-base `t0`. That changes **install semantics** — the origin, the τ that
`_install_continuity_ok` compares at, the emitter's clock — on the path whose terminal knot is the
throw. The bounded gate changes nothing that leaves the function: `validate_cycle` itself is untouched
(no new parameter, no changed check), `feasibility.py` was not edited, the head is not trimmed and `t0`
is not re-based. Only *which knots are asked about* moves.

The stencil width is **derived from `validate_cycle`'s four passes, not chosen**: per-sample geometry
and leg vel/acc reach 0 knots (`_locate` is `int(t/dt)`, Hermite over `k`, `k+1` only); leg jerk reaches
one *sub*-sample; hand-span extrema and the per-knot step each reach one knot ⇒ `_VALIDATE_STENCIL_KNOTS
= 1`. It must be > 0 for a reason that only appears on the join: the acceleration **at** the seam knot
genuinely changes when the join happens (before it, that sample sat one ULP inside the old span; after,
`_locate` puts it at `s = 0` of the new one), so the jerk difference reaching into it is new content.

Two review findings turned that derivation from prose into a checked claim. **F3** — the stencil test's
stated motivation was wrong: a range starting *at* the seam does see an injected hand-rate defect
(hand-span `k → k+1`); the real motivation is the leg-jerk difference from the last sub-sample of span
`k_seam − 1` into `s = 0` of the new span, now driven by the smallest bump that discriminates
(+10.0 mm/s on `pose_vel[k_seam]`: from `k_seam` it passes, from `k_seam − 1` it refuses `LIMIT_JERK`).
**F4** — the tripwire is now *executed* rather than asserted: for each pass a defect of its own kind is
injected and the range start walked back until the gate refuses. Measured reaches: leg jerk 1, and 0 for
every other pass. The test asserts `max(reach) == _VALIDATE_STENCIL_KNOTS`, so the constant can be
neither too small nor larger than the code needs.

**One tradeoff was accepted and written down (F5): a joined plan inherits its head's verdict.** Tighten
`set_limits` mid-session and the extend does not retro-judge the head. It cannot — the head is already
on the wire and cannot be un-emitted — and refusing the extend of a release-terminal plan re-opens the
supersede cliff, which is a strictly worse answer than carrying a head that was legal when it was
gated. The new window is judged at the new limits, and the test halves the leg-jerk limit to prove it.

**F1** is the finding that says why the label matters as much as the range: `extend` after a
`replan_tail` merged a *range* report's peaks and labelled the result whole-plan — `peak_leg_jerk`
75 356 against a true 95 101 mm/s³, a **20.8 % under-report wearing a whole-plan label**. Fixed by
returning the union range, contiguous by construction. A replan's own peaks are deliberately **not**
merged: a re-plan replaces the tail, so the source report describes trajectory that no longer exists,
and merging would put a deleted tail's number next to `ok=True`.

### (e) The hold pre-empts the solve on the server, not by giving up earlier on the client

After the ladder was made deadline-driven (§ f) one residual remained: the hold itself could queue
behind an in-flight `plan_cycle` solve, because `trajectory_node.main` was plain `rclpy.spin` with every
service on the default mutually-exclusive group. The client-side alternative — give up earlier, so the
hold is *issued* sooner — was measured against and rejected: it would raise the practical dwell floor to
**~1.15 s** (the give-up would have to precede a whole solve), and it still would not hold under load,
where the solve is 1.06 s.

So the fix is server-side, and it is deliberately minimal:

- `trajectory/hold` gets its own `ReentrantCallbackGroup`; **every other callback keeps the default
  mutually-exclusive group**, so `plan_cycle` still cannot interleave with the timers and subscriptions
  it shares state with, and no invariant that held under `spin` is weakened;
- `main()` builds `MultiThreadedExecutor(num_threads=2)` — one thread for the serialized group, one free
  for the hold;
- an **install epoch**: `_svc_plan_cycle` captures it at entry and each mode refuses
  `SUPERSEDED_BY_HOLD` twice — once early, for an honest message, and once at the install via
  `_install(..., require_epoch=…)`, **under the same lock acquisition that performs the install**,
  which is what closes the race. The hold's *bump* has to be inside that same block too, and in the
  first cut it was not — see § (h) A1: it was a second `with self._plan_lock:` after the install, and
  a solve already past both the early check and `_install_continuity_ok` but blocked on `_plan_lock`
  could take the lock in that window, read the **old** epoch, and put the release-terminal plan back
  on the wire while the hold returned `success=True`. The operator's cancel silently undone, with a
  success response to say otherwise. `_install` now takes `bump_epoch=`, applied in the same block
  that swaps the active plan, and `_svc_hold`'s second acquisition is gone.
- the guard is **not** cycle-only: `go_home`, `go_to_pose`, the timed target and the catch install
  each capture the epoch at handler entry and refuse through their own failure shape. All four were
  serialized behind the hold before the reentrant group existed, and each takes hundreds of ms during
  which a hold can now land.

**Why the epoch and not the continuity check**, verified rather than argued:
`_install_continuity_ok(plan, tau_now)` returns **True** immediately after a hold — the hold ramp and
the plan it replaced agree well inside the 0.06 rev bound — so nothing but the epoch stands between a
stale solve and the wire. **Why the epoch is checked first**: after a hold the continuity check often
refuses too, and `STALE_STATE` reads as *"the machine moved, retry"* — and retrying is exactly what
re-arms the throw the operator just cancelled. **Why only the hold bumps it**: every other install runs
on the serialized group and cannot land during a solve, so bumping there would refuse installs for a
race that cannot happen, and a guard that fires when nothing is wrong is how guards get switched off.

Measured (2026-09-07, this Jetson, three runs, a real concurrent solve): hold **0.8 / 0.8 / 0.8 ms**
alone; **4.4 / 1.0 / 1.0 ms** during a **314.2 / 310.5 / 307.6 ms** solve; all three of those solves
refused `SUPERSEDED_BY_HOLD`. Contention costs at most **~3.6 ms** against the **307–1060 ms** of
waiting it removes. The 40 Hz emitter is unaffected — it is its own thread and never ran on the
executor at all.

### (f) What the 0.160 s margin covers, and what it cannot

`_UNIFIED_HOLD_MARGIN_S = 4 × NODE_LOOP_PERIOD_S = 0.160 s`: one tick to observe (the poll is
level-triggered on that grid), one to dispatch, the hold's own planning work, and one tick of slack for
the server's queue. The planning term is **measured**, not estimated: `planner.build_hold` from a
*moving* seed at session limits 250/3000/150000 — the real case, a hold issued mid-carry — over **300
calls: min 46.7 ms, p50 47.0, p95 47.4, max 49.2 ms**. It is a single C2 quintic plus `_gate`, not a QP,
which is why it is tens of milliseconds. 0.160 s covers the measured worst ~3×.

That margin covers **the hold's own work and nothing else**. It neither does nor can cover a
`plan_cycle` solve still executing when the hold arrives — that is § (e)'s job, and the operator-facing
string says so, because an operator who reads a late hold as a mis-sized margin will "fix" the wrong
number.

The finding that produced it (**H2**) is worth stating plainly: the extend poll was bounded by the
**2.0 s client wait**, not by the deadline it was racing. The carry a ring actually has between its
catch and its supersede deadline is `dwell − dt` — 1.175 s at beat 2.0 — so a wedged `plan_cycle` was
not declared UNACKED until **R_k + 2.8 s against a deadline of R_k + 1.975 s**: the LANDING fall-back
*and* the hold both landing ~0.8 s past the cliff, at **any beat under about 2.8 s** at flight 0.8
(the extend is sent at `R_k + flight`, so the old bound sat past the cliff whenever
`flight + 2.0 > beat − dt`; 2.6 s is the flight-0.6 figure). The bound is now
`min(sent_at + _SERVICE_WAIT_S, deadline − _UNIFIED_HOLD_MARGIN_S)`, and it is the same bound
`_settle_unified_extend` inherits rather than restarting the clock.

There is deliberately **no separate STEADY give-up constant**: the STEADY is waited for to that bound,
so it gets every millisecond that exists, and whether the *middle* rung can still fit is asked at the
moment of failure — `now + _UNIFIED_EXTEND_LEAD_S + _UNIFIED_HOLD_MARGIN_S <= deadline`. If it cannot,
the LANDING is **skipped** and the hold runs: a window that cannot arrive in time is not a fall-back, it
is the hold's budget spent on nothing.

### (g) Every review finding was a predicate that was true of the old shape and false of a ring

That is the common shape, and it is the reason the review lenses earned their cost:

- **H1** — the chain-live predicate read `resp.release_terminal`, i.e. *"the plan's last knot is a
  release"*. That is a different question, and it answers **False for the last two cycles of every
  ring**: the final LANDING window is installed a whole beat before the release it starts at. A MISS
  there, or a cancel / ceiling / stall-watchdog / exception in cycle N before R_N, took the early return
  and issued **zero holds** while a full stroke was ~1.2 s ahead — the plan throwing over an empty cup
  after the session had already reported `STOPPED_ON_MISS`. The predicate is now an **instant**: the
  last release the standing plan carries, computed where the chained window's period is in hand, with
  the test `now < that`.
- **A second H2-class bug** — a lost chain relabelled `cycle_result.outcome` to a `REJECTED_*` string
  while leaving `success = True`, so `note_cycle_result` read a rejection as a successful throw and the
  session reported **COMPLETED after losing its ring**. It now arms a session terminal
  (`STOPPED_CHAIN_LOST`) and the cycle keeps its own CAUGHT verdict — the ring failed, the throw did
  not, and collapsing the two hides a good catch inside a plumbing fault.
- **M3** — the reload interlude go-homes and recentres the platform, over a streaming plan with a
  release still ahead: one interval with two owners *and* a stroke still coming. Fixed as the class
  rather than the instance, by enumerating the session-loop branches that command motion instead of
  sampling them.
- **M4** — the abandoned-extend drain sat below an early return, so it was skipped in exactly the cases
  the predicate called safe. An accepted EXTEND installs a window that *carries* a release, so ignoring
  it re-creates the hazard that had just been correctly called absent.
- **F2** — *"the head is carried bit for bit"* was **false** for `replan_tail` on a tilted plan.
  `cup_realize._knot_derivative` is central inside a series and second-order one-sided at its ends, and
  it feeds the tilt-rate term; a plan built by `extend` therefore carries one-sided velocities at every
  **prior seam knot**, and re-decomposing the joint series made those knots interior. Measured
  **4.41e-2 mm/s** at knots 24 / 80 / 136; now **0.000e+00 on all four arrays at every prior seam knot**,
  because the head's four channels are taken from the live plan verbatim. The invariant the bounded gate
  rests on is now true by construction rather than by accident of magnitude.
- **F6** — `_CYCLE_CATCH_MATCH_S` (0.25 s) must stay strictly **below** `_CYCLE_REPLAN_LEAD_S` (0.30 s),
  or a just-passed catch could be selected by an arrival inside the band. That coupling is now named in
  both constants and pinned by a test that drives the consequence. And two marks inside the band now
  refuse `NO_LIVE_CATCH(AMBIGUOUS …)` naming both candidates, rather than resolving a coin flip by
  nearest-wins.

### (h) The audit: one pass, fourteen findings, all approved, all fixed

The review lenses of § (g) ran on the implementation; a separate audit then ran on the whole diff
**and on these documents**. Fourteen findings, every one owner-approved. Three of them were the kind
that only a second reader finds, and all three are the same failure of imagination as § (g): a rule
correct for the shipped shape, carried onto a ring or onto a newly-concurrent pair.

- **A1 (blocking) — the epoch bump was a second lock acquisition.** § (e). The guard was written to
  close a race and left the race open by one `with` block. Pinned by a test that parks a thread on
  `_plan_lock` across the hold and asserts that whichever order the two threads get the lock in, the
  hold is what ends up streaming and the epoch has moved.
- **A2 (warning) — check-then-unpack, and post-install bookkeeping.** `_plan_cycle_extend`,
  `_plan_cycle_replan` and `_on_dynamic_target` checked `self._cycle` and then unpacked it. A hold
  landing between the two makes that a `TypeError`, and `_svc_plan_cycle`'s except ladder catches
  `CycleInfeasible` / `ValueError` / `TrajectoryInfeasible` — not that. On Foxy the
  `MultiThreadedExecutor` re-raises, `spin()` unwinds into `main()`'s `finally`, **the emitter thread
  stops and the Teensy's setpoint watchdog E-STOPs the machine.** A crash-to-E-STOP reachable from a
  cancel is the worst thing on this list, and it was invisible while the executor was single-threaded
  because the hold could not land mid-callback at all. All three paths now take one snapshot under the
  lock, and the `self._cycle = (...)` writes and `_cycle_replans` resets moved inside `_install`'s own
  lock block.
- **B1 — the runtime release-window guard charged a chained cycle for a stroke it never dispatches.**
  `min_event_delay_for_throw_s` (0.281 s at the nominal flight) fronts a `set_hand_traj_cmd` whose
  windup has to fit before the ball leaves. A chained cycle issues no such RPC — its release is a knot
  — and it cannot start before `catch + max(verdict, extend)`, so **every chained cycle aborted
  `ABORTED_CANT_MAKE_RELEASE` at a dwell the accept gate had just admitted**: the machine refusing,
  mid-ring, a cadence it had promised to fly. It is now one loop period on a chain, branched on the
  same `chained` flag and for the same reason as the park band.

The accept gate itself was the other half of that hole, and closing it is where the floors of § Fix 8
come from: the gate knew about the extend carry and not about **the next cycle's lead**, which is the
larger of the two — so it would have admitted beats the ring could not run. The cycle-1 delay floor
did not exist at all, and its absence is exactly the `ABORTED_NO_RELEASE` the 2026-09-07 re-fly saw.

Three more are worth recording because they change what an operator sees. **B2** — an armed ring
terminal was consumed only on a *successful* cycle, so with `stop_on_miss: false` the ring reverted to
the serial cadence silently after all; it is now consumed on a failed cycle too, below the
`stop_on_miss` clause, so a MISS the operator asked to stop on still wins. **B7d** — with the hand-off
in force, a cycle that merely spun up late produced a `CHAIN_SKEW` refusal quoting `+0.000 s` against a
0.040 s tolerance, a sentence nobody can act on; the fact is different in kind and now has its own
name, `REJECTED_CYCLE_PLAN(CHAIN_PAST: …)`, which tells the operator to look at the *previous* cycle's
verdict and settle times rather than at `dwell_time_s`. **B3** — `STOPPED_CHAIN_LOST` had no
`REJECT_WIRE_MAP` hint, so a trace read of the sitting would have shown the code without its operator
line. The fix worth more than the entry is the new
`test_every_minted_outcome_code_has_an_operator_hint`, which closes the class the 2026-09-07 entry
raised as open item 4 — and **it immediately found three pre-existing unhinted terminals**
(`STOPPED_RELOAD_BUDGET`, `STOPPED_FLOOR_CLEAR_REQUIRED`, `STOPPED_RELOAD_FAILED`), all now hinted.
That is the difference between fixing an instance and landing the contract: the instance was one
missing row, and the enumeration found three more that had been missing for weeks.

**A3** is a correction to § (d)'s own derivation rather than a defect. The replan stencil is one knot
wider than the extend's: at a splice the knot at `k_s` legitimately moves (its position is pinned, its
velocity is not — its forward neighbour is new), so the Hermite acceleration at `s = 0` of span
`[k_s−1, k_s]` changes and the leg-jerk difference across the sub-sample boundary `(k_s−2 | k_s−1)` is
new content that a view starting at `k_s−1` never forms. The replan path gates from
`k_s − _VALIDATE_STENCIL_KNOTS − 1`, **43 knots**, with the asymmetry written out at the code; the
extend path is unchanged, because *its* seam knot is bit-identical on both channels. The non-vacuity
pair moved with it: a leg-acceleration step at `k_s−1` is **missed from `k_s−1` and caught from
`k_s−2`**.

---

## Fix — what shipped

### The ring (coordinator: `reload_coordinator_node.py`, `toss_session.py`, `toss_sequencer.py`)

1. **The shape.** `_tick_unified_launch` chains `KIND_STEADY` with `chain_period_s = _unified_beat_s`
   when the session still owes releases, and `KIND_LANDING` otherwise. `_unified_cycles_after_this()`
   is the one place `num_throws − cycle_index` is written, read by both the launch and the extend. At
   `num_throws == 1` the `PlanCycle` **request** is bit-for-bit the pre-UH-7a one.
2. **The release hand-off.** A chained cycle is built with
   `release_at_perf = chained.t_release_mono` and `chained=True`, so the FSM's `_t_release` *is* the
   plan's knot rather than `poll_instant + throw_delay_s`. `CHAIN_SKEW` therefore becomes a **check**
   rather than a negotiation, and its tolerance drops from `_UNIFIED_EXTEND_LEAD_S` (0.60 s — 15× too
   wide to catch what it exists for) to `_UNIFIED_CHAIN_SKEW_TOL_S` = **one tick**. `throw_delay_s`
   keeps a narrower meaning on a chain: how far before the planned release the next FSM spins up.
3. **The beat, hoisted once.** `TossSessionSequencer.beat_s` is computed in `__post_init__` after the
   dwell's default substitution, and `_unified_beat_s` reads it instead of re-adding `flight + dwell`.
   Two additions of the same two floats is exactly the shape that produces a skew nobody can account
   for.
4. **The trigger.** `now >= t_release_mono + flight` (the live catch, on the plan's own clock) **or**
   `now >= supersede_deadline_mono − _UNIFIED_EXTEND_LEAD_S`, whichever first. After the catch, because
   a replan and an extend both re-install `trajectory_node`'s single `_cycle` record and the replan
   window has already closed — so the two are ordered structurally rather than by luck.
   `t_catch_mono` is deliberately *not* used: on a joined plan it reports `catches[0]`.
5. **The ladder, three rungs downhill.** STEADY → LANDING fall-back → `trajectory/hold` + stop.
   Deadline-driven throughout (§ Discussion f), non-blocking (§ b), with the middle rung skipped when it
   cannot finish in time.
6. **The teardown holds.** `_hold_live_unified_chain(why)` — idempotent, instant-predicated — from four
   places: the FSM's SAFE_ABORT terminal, `_safe_toss_on_early_exit` (**outside** its `seq.prepared`
   gate, because a chained cycle that has not reached PREPARE has armed nothing while the previous
   cycle's plan carries its release), the session `finally`, and
   `_hold_chain_before_session_motion` for every session-loop branch that commands motion. It also
   **discards** the in-flight extend record under the same lock acquisition, and
   `_settle_abandoned_chain` drains that future and **holds again** if it accepted — because rcl walks
   services in registration order and `trajectory/hold` is registered *before* `trajectory/plan_cycle`,
   the adverse order.
7. **Two session terminals**, `STOPPED_CHAIN_REFUSED` (a fall-back window installed, the ball was
   caught, the ring was truncated) and `STOPPED_CHAIN_LOST` (nothing was chained, the machine was held,
   the cycle keeps its own verdict). Both threaded through `outcome_detail`'s round-trip and subcode
   contract and through `REJECT_WIRE_MAP` — `STOPPED_CHAIN_LOST` only after the audit (§ Discussion h).
8. **The beat floors at acceptance**, Layer B, right after the aim gate, both minting
   `REJECTED_BEAT_TOO_SHORT` before anything is armed, lifted or commanded, and each naming the knob
   that moves it:
   - **the chain dwell floor, 0.800 s**, whose binding term is *the next cycle's lead*: cycle k+1
     cannot start before `catch + max(verdict 0.560, extend 0.600)` and then needs
     `_UNIFIED_CHAIN_PREAMBLE_S` 0.160 s of FSM ladder plus one 0.040 s tick to announce and dispatch.
     The other requirement — that the carry `dwell − dt` hold one `_UNIFIED_EXTEND_LEAD_S` — is only
     0.625 s and does **not** bind. The message names both and says which held, because they move
     under different knobs and an operator told only the number cannot tell which to turn;
   - **the cycle-1 `throw_delay_s` floor, 0.866 s** = `preamble 0.160 + solve 0.606 + window 0.600 −
     grace 0.500`. Cycle 1 is the only cycle whose release is *derived* (`now + throw_delay_s`, stamped
     once) while the LAUNCH that throws lands its release at `install + 0.6`, so below this floor the
     ball leaves later than `TOSS_RELEASE_GRACE_S` allows and cycle 1 aborts `ABORTED_NO_RELEASE` with
     the ball unthrown. The 0.606 s is `_UNIFIED_JOINED_SOLVE_S`, the **measured worst** joined install
     on hardware (500–606 ms at load1 3.7–7.4) — deliberately not the 1.20 s `_UNIFIED_PLAN_BUDGET_S`,
     because over-estimating *there* only buys an early release, which is free, while over-estimating
     *here* refuses cadences the machine can fly.

   Both are the **plumbing** floor — the chain has time to *ask* — and explicitly not the planner's
   feasibility floor, which is answered per cycle and is safe to take late because the ladder holds
   the machine before the deadline. Layer B and not the session FSM because the floors are built from
   planner constants, and pushing them into `TossSessionSequencer` would give the beat two owners.
   The legacy `dwell ≥ throw_delay_s + handoff_margin_s` floor still applies on top and is
   flight-dependent — **0.1412 s at flight 0.80, 0.1766 s at the 0.6387 s flight of
   `throw_height_m 0.5`** — so a unified session's real minimum dwell is **1.007 s / 1.043 s** and its
   minimum beat **≈1.81 s / ≈1.68 s**, against the ~2.5–2.7 s the serial choreography needed. (The
   cycle-1 refusal's tail deliberately quotes NO dwell figure: the docs pass caught a hardcoded
   0.120 s handoff that was 20–60 ms optimistic, and restating the sequencer's flight-dependent
   number at the gate is drift by construction — the message points at `REJECTED_DWELL`, whose
   number is the authority at any given flight.)
9. **Three no-ops on a chained cycle**: the per-cycle floor lift (its premise is a hand at rest a
   `MODE_NEW` LAUNCH is about to be seeded from), the positioning move (forced off at the single
   decision seam, because the plan owns the platform for the whole beat), and the FSM's `hand_parked`
   band (which describes a kind-0 stroke no chained cycle dispatches — branched at both the CHECKING and
   the THROWING-entry site together, and deliberately *not* at the pipelined commit gate, which unified
   makes unreachable).

### The planner (`unified_cycle.py`, `trajectory_node.py`)

10. **Live-catch selection (B1).** `_live_catch_mark(meta, t0, arrival_perf)` picks the catch mark
    nearest the tracker's own predicted landing, refusing `NO_LIVE_CATCH` when the gap exceeds
    `_CYCLE_CATCH_MATCH_S` = 0.25 s ("about some other ball") or when the mark is inside the committed
    head. The selected mark's instant is nominated through `catch_frac = mark.t_s / total_duration` — an
    exact round trip through an existing field, so `PlanCycle.srv` is unchanged and
    `catch_coordinator_node.py` is untouched. The tolerance is derived from the throw's own execution
    error: measured release is +5…+17 % of plan, and flight time is proportional to take-off speed, so
    +17 % is +0.136 s at the nominal 0.8 s flight. Empirically found while building it: the knot must
    come from the `CatchMark`, not from `floor(t_s/dt)` on the joined clock — `2.6/0.025` is exactly
    `104.0` while the window's `0.6/0.025` is `23.999999999999996`, one knot of pure representation that
    refused the very replan the fix enables.
11. **The mid-plan-release detach bound made unconditional (B2)** over every non-terminal release.
12. **The catch-side still-ahead search (B3)** for `t_catch_mono` / `arm_lead_s`, mirroring the release
    side; `stroke_clear_s` still comes from `releases[-1]`, which is correct on a release-terminal chain.
13. **The replan budget is per WINDOW, i.e. per BEAT under a chain (B4)** — stated in
    `_plan_cycle_extend`'s docstring and pinned, rather than inherited.
14. **Bounded gates.** `extend_gate_range`, `_gate_view` (numpy views, no copy), `_merged_peaks`,
    `_needs_whole_plan_gate`, `_gate_from_knot`, `_gate_joined`, and
    `CycleMeta.report_range_knots` (`None` = whole plan). Refusals from a bounded gate carry a leading
    note naming the range's clock offset, because without it a seam defect at 0.581 s reported
    `t = 0.006 s`. One fall-back to a whole-plan gate remains: a joined plan whose first knot is parked
    below the homed zero, where `_cycle_stroke_floor` would silently re-derive a stricter floor from the
    view's own first knot. It does not hold on any shape the coordinator installs. The **replan** path
    gates one knot wider (`k_s − _VALIDATE_STENCIL_KNOTS − 1`, **43 knots**) — see § Discussion (h) A3;
    the extend path is unchanged at 58.
15. **The hold pre-emption** — reentrant group, two-thread executor, and an install epoch bumped
    **inside the install's own lock block** (`_install(..., bump_epoch=True)`), extended to `go_home`,
    `go_to_pose`, the timed target and the catch install; `SUPERSEDED_BY_HOLD` (§ Discussion e, h).

### From the audit

16. **`TossSequencer.release_window_floor_s`** — one loop period on a chained cycle, the kind-0
    dispatch budget otherwise; both release-window guards branch on it (B1).
17. **`REJECTED_CYCLE_PLAN(CHAIN_PAST: …)`** for a chained release already in the past, replacing a
    `CHAIN_SKEW` that could only quote `+0.000` (B7d).
18. **`note_cycle_result` consumes an armed ring terminal on a FAILED cycle too**, below the
    `stop_on_miss` clause (B2).
19. **`_install` grew `bump_epoch=`, `cycle=` and `reset_replans=`**, all applied inside the lock block
    that swaps the active plan; the three `self._cycle = (...)` writes and two `_cycle_replans` resets
    after `_install` are gone, and the three cycle-reading paths take one snapshot under the lock
    (A1, A2).
20. **`_UNIFIED_KNOT_DT_S` is read from `hw.JB_TRAJ_KNOT_DT_S`** rather than restated as a literal, and
    `_UNIFIED_CHAIN_PREAMBLE_S` from `pre_dispatch_budget_s(False)` — so a re-cut of the FSM ladder or
    the knot grid moves the accept floors with it (B6).
21. **The drain-timeout branch HOLDS** when nothing had held yet and carries the verdict (B4); the
    stale-owner branch drains instead of dropping, `_toss_unified_chain = resp` moved inside the lock,
    and the direct arming call shares `_arm_chain_stop`'s stale-install ERROR (B7a–c).

### Tests

`tests/ros/test_unified_ring.py` (new, 38 tests: the ring shape, the hand-off, trigger ordering, the
no-command rules, the fail-safe ladder, both beat floors, a realistic 0.40 s solve latency, a MISS
during a pending extend) · `tests/ros/test_trajectory_hold_preempts_solve.py` (new, 9 tests, including
`test_main_runs_a_multi_threaded_executor_not_plain_spin`, which fails if `rclpy.spin` returns — a
reentrant group is inert under it and the cliff would be back, silently;
`test_the_hold_swaps_the_plan_and_bumps_the_epoch_under_ONE_acquisition`;
`test_a_hold_racing_the_cycle_unpack_does_not_raise` and
`…_bookkeeping_leaves_no_stale_record`; and `test_a_move_service_cannot_install_over_a_hold_either`,
which drives `go_to_pose` and then greps all four handlers' source so a new planning service cannot
quietly join without the guard) · `tests/motion/test_unified_cycle.py` (a `ring` fixture, a
module-scoped `long_ring` of LAUNCH + six STEADY windows, the STEADY→STEADY seam, the chained
spent-release cone, the gate-range and stencil tripwires, the tilted-ring head-verbatim pin) ·
`tests/ros/test_unified_cycle_integration.py` (a `ring_stages` fixture making three **real** service
calls) · `tests/ros/test_outcome_detail.py` (round-trip and subcode rows for the new codes, plus
`test_every_minted_outcome_code_has_an_operator_hint`) · `tests/hardware/toss_trace_recorder.py`
(`REJECT_WIRE_MAP` **+11 hints** — 8 on the unified path, including the two UH-7a session terminals and
`REJECTED_CHAIN_LOST`, and **3 pre-existing terminals the new completeness test found unhinted**:
`STOPPED_RELOAD_BUDGET`, `STOPPED_FLOOR_CLEAR_REQUIRED`, `STOPPED_RELOAD_FAILED` — closing the
2026-09-07 entry's open item 4 as a contract rather than as an instance) ·
`tools/probes/cadence_rung_check.py` (its `session_accepts` docstring now says it models the
**legacy** session only — the beat floors are Layer B and out of its scope).

One test-honesty fix (A5): `test_a_hold_preempts_an_in_flight_solve` asserted
`held_at < returned_at`, which was true by construction — the worker cannot return until `release` is
set on the next line — so it would have passed even if the hold had queued behind the solve. It now
asserts `worker.is_alive()` before `release.set()`, which actually proves the solve was in flight when
the hold installed. The 0.100 s latency bound is unchanged.

---

## Verification

All runs on the Jetson, in the project venv, 2026-09-07. Load is noted where it matters, because four
tests in this area are wall-clock sensitive by construction.

**Planner side**

- (2026-09-07 11:58–12:01, `pytest tests/motion/test_unified_cycle.py tests/ros/test_unified_cycle_integration.py tests/ros/test_unified_cycle_levelling.py tests/ros/test_unified_launch_floor.py tests/ros/test_catch_coordinator_node.py tests/ros/test_catch_coordinator.py tests/ros/test_unified_cycle_bench.py -q`, run 3× back to back) — **361/361 pass**, 41.68 / 42.31 / 42.21 s.
- (2026-09-07 13:49, `pytest tests/motion/test_unified_cycle.py -q`, run 3×) — **83/83 pass**, 15.41 / 15.49 / 15.56 s.
- (2026-09-07 13:50:21, `pytest tests/motion/test_unified_cycle_budget.py -q`, alone as the serial tail runs it) — **1/1 pass in 8.42 s**.
- (2026-09-07 13:50:32, `pytest tests/motion/ tests/ros/test_unified_cycle_integration.py tests/ros/test_unified_cycle_levelling.py tests/ros/test_unified_launch_floor.py tests/ros/test_unified_cycle_bench.py tests/ros/test_trajectory_node.py tests/ros/test_catch_coordinator_node.py tests/ros/test_catch_coordinator.py tests/sim/test_unified_gate.py -q`) — **2698 passed, 3 skipped in 397.29 s**.
- (2026-09-07 14:27:38, the F1–F7 scoped set: `pytest tests/motion/ tests/ros/test_unified_cycle_integration.py tests/ros/test_unified_cycle_levelling.py tests/ros/test_unified_launch_floor.py tests/ros/test_unified_cycle_bench.py tests/ros/test_trajectory_node.py tests/ros/test_catch_coordinator_node.py tests/ros/test_catch_coordinator.py tests/sim/test_unified_gate.py -q`) — **2704 passed, 3 skipped in 417.46 s**.
- (2026-09-07 14:41:39, `pytest tests/ros/test_launch_nodes.py tests/ros/test_trajectory_hold_preempts_solve.py -q`) — **20/20 pass in 1.99 s**.
- (2026-09-07 14:41:43, `pytest tests/motion/test_unified_cycle_budget.py -q`, alone) — **1/1 pass in 8.34 s**.
- (2026-09-07 14:42, `pytest tests/motion/ tests/ros/ tests/sim/test_unified_gate.py -q`) — **5171 passed, 4 skipped in 767.24 s**.

**Coordinator side**

- (2026-09-07, `pytest tests/ros/test_unified_ring.py -q -p no:randomly`) — **33 passed in 3.89 s**.
- (2026-09-07, `pytest tests/ros/test_unified_ring.py tests/ros/test_toss_session.py tests/ros/test_outcome_detail.py -q`) — **203 passed in 4.97 s** (35 ring tests).
- (2026-09-07, `pytest tests/ros/test_unified_cycle_integration.py tests/ros/test_unified_ring.py -q`) — **132 passed in 21.03 s**, no deselects.
- (2026-09-07, the 12-file scoped set: ring + integration + toss_coordinator / toss_session / toss_integration / outcome_detail / toss_ilc_node / toss_continuous_node / toss_sequencer / unified_launch_floor / choreography_map + `tests/motion/test_cadence_rung_check.py`) — **959 passed in 132.52 s**.
- (2026-09-07, `pytest tests/ros/ -q`) — **2898 passed, 1 skipped in 373.26 s**.
- (2026-09-07, `pytest tests/ros/test_unified_cycle_integration.py tests/ros/test_toss_continuous_node.py tests/ros/test_toss_coordinator.py tests/ros/test_toss_integration.py -q`) — **483 passed in 101.35 s**.

**After the audit**

- (2026-09-07 20:29:06, `pytest tests/ros/test_trajectory_hold_preempts_solve.py tests/ros/test_unified_cycle_integration.py tests/ros/test_launch_nodes.py tests/motion/test_unified_cycle.py -q`, quiet box at load1 1.14) — **217/217 pass in 44.72 s**.
- (2026-09-07 20:29:53, `pytest tests/motion/test_unified_cycle_budget.py -q`, alone as the serial tail runs it) — **1/1 pass in 8.73 s**.
- (2026-09-07 20:30:48, `pytest tests/ros/ -q`) — **2924 passed, 1 skipped in 383.39 s**.
- (2026-09-07, `pytest tests/ros/test_unified_ring.py -q -p no:randomly`) — **38 passed in 3.99 s**.
- (2026-09-07, `pytest tests/ros/test_unified_ring.py tests/ros/test_toss_session.py tests/ros/test_outcome_detail.py tests/ros/test_toss_coordinator.py tests/ros/test_toss_integration.py tests/ros/test_toss_continuous_node.py tests/ros/test_toss_ilc_node.py tests/motion/test_cadence_rung_check.py tests/ros/test_unified_cycle_integration.py -q`) — **760 passed in 125.45 s**.
- (2026-09-07, `pytest tests/ros/ -q`) — **2925 passed, 1 skipped in 377.51 s**.
- Documents (2026-09-07 20:50:59, `pytest tests/sim/test_plans_index.py tests/ros/test_choreography_map.py tests/sim/test_logbook_search.py tests/sim/test_logbook_front_matter.py -q -p no:randomly`) — **136 passed in 9.04 s**.

**Probes** (uncommitted, cited in the docstrings they justify)

- (2026-09-07, `/tmp/probe_uh7_cost.py`, run twice with identical output) — six extends off a 0.6 s
  launch at session limits: joined 81 → 361 knots, **58 knots gated at every depth**, extend
  **163.2 / 163.2 / 163.0 / 162.9 / 163.5 / 164.3 ms** against a whole-plan gate of
  **219.7 / 378.5 / 536.2 / 694.5 / 857.8 / 1016.8 ms**.
- (2026-09-07, `/tmp/probe_uh7_replan_cost.py`, run three times) — replanning the live catch on a
  growing ring: **a constant gated width at every depth**, 136–155 ms with one 311 ms outlier in the
  first run only, against a whole-plan gate of **222 → 1022 ms**. The probe measured 42 knots against
  the then-shipped `k_s − 1` start; the audit's A3 widened it to `k_s − 2` and the shipped width is
  **43**, which does not move the wall times (one knot of a 43-knot gate).
- (2026-09-07, `/tmp/probe_uh7_planner.py`, run twice identically) — a `replan_tail` on a joined
  LAUNCH(0.6)+STEADY(1.4) spliced at `k_s = 28` preserves the terminal release **exactly** (`t_s`,
  `total_duration`, mark `site_mm` / `vel_mm_s` / `tilt` all Δ = 0.0; cup terminal velocity 8.793e-11
  mm/s).
- (2026-09-07, `/tmp/t_lat.py`, three runs, a real concurrent solve) — hold **0.8 ms** alone;
  **4.4 / 1.0 / 1.0 ms** during a **314.2 / 310.5 / 307.6 ms** solve; all three solves refused
  `SUPERSEDED_BY_HOLD`.
- (2026-09-07, `planner.build_hold` from a moving seed at session limits, 300 calls) — **min 46.7 /
  p50 47.0 / p95 47.4 / max 49.2 ms**, the measurement `_UNIFIED_HOLD_MARGIN_S` is derived from.

**First full gate on the final tree** (2026-09-07, `./run_tests.sh --full`): parallel
**1 failed / 6963 passed / 4 skipped / 2 xfailed in 460.77 s**, serial 4 passed in 26.43 s,
exit 1. The one failure was a test premise, not a defect:
`test_an_extend_after_a_replan_labels_the_UNION_range_not_the_whole_plan` asserted the
union-range jerk peak is *strictly below* the whole plan's, but after A3 widened the replan
gate to `k_s − 2` the plan's global peak sits inside that range, so the two agree to the last
ULP (95100.913270814 vs 95100.913270774) and the strict `<` asserted the sign of a 4e-11
residual — the same class as the sign-flaky replan test fixed on 2026-09-05. The assertion is
now `≤` within 1e-9 with the reason at the site; the label and the range-equality assertions
above it are the real pins and were never in doubt.

**Full gate, after that one-line test fix** (started 2026-09-07 23:53, finished 2026-09-08 00:01,
`./run_tests.sh --full`): parallel **6964 passed / 4 skipped / 2 xfailed in 460.71 s**, serial
**4 passed in 26.39 s**, total 493 s, **exit 0**. The fixed file alone
(2026-09-07 23:53, `pytest tests/motion/test_unified_cycle.py -q`): **87 passed in 23.51 s**.

### One flag investigated and dismissed

`tests/motion/test_unified_cycle.py::test_the_gate_and_not_the_qp_dominates_a_healthy_solve` failed in
two mid-session runs and was reported once as *"reproduces in isolation, so a real regression"*. It is
**load**. Re-checked on a confirmed-quiet box (`ps` showed no other pytest, load1 1.01): three isolated
runs **1 passed** in 0.52 / 0.52 / 0.51 s, and the whole file **83 passed** three times. The
cold-process stage split for the fixture it measures is `qp 14.2, tilt 4.2, dec 3.7, val 163.0,
cont 0.8 ms` — the gate dominates by **11×**, which is what the test asserts. Under BLAS thread-pool
starvation the cold QP inflates ~20× (14 → 295 ms) while the gate inflates ~1.07× (163 → 175 ms), which
is the test's own documented discriminator firing correctly. The bounded replan gate also took that
file from **54.6 s to 15.4 s**, which lowers the module's contribution to box load and makes the flake
*less* likely, not more.

---

## Outcome

**Code-complete, one atomic commit, NOT FLOWN.** Everything above is offline: probes, the mocked-ROS
suite, and the real-service integration fixtures. Nothing has moved a hand.

What the next sitting is the first evidence of, in order: that a chained STEADY installs at all on the
machine; that the achieved extend lead (the log line's *"installed X s before the superseded
deadline"*) sits where the 0.60 s budget says it should; that the beat the operator asked for is the
beat the releases actually land on; and that a hand which never rests between cycles tolerates it. The
runbook's UH-7a rung caps the first sitting at **3 then 5 throws** for exactly that last reason — the
600 s hold row (0.041 mm flat, 2026-09-04) is a *static* hold and says nothing about duty cycle.

**The supersede alarm firing is a plumbing defect report, not an acceptance signal.** It has never
fired in production; UH-7a is the first shape that arms it. If it fires, the beat carried 10.90 mm of
slider error inside every firmware guard — stop the sitting and raise the dwell rather than retuning
around it.

---

## Withdrawn claims

- **"UH-7's constant beat needs a `release_at_perf` hand-off that does not exist yet"** — the plan's
  item 6, read (including by the brief this work started from) as *"the chaining code does not exist"*.
  **WITHDRAWN on the survey:** the hand-off really was missing, but `_tick_unified_extend`,
  `_extend_unified_cycle`, `_unified_beat_s`, `_toss_unified_chain` and the `CHAIN_SKEW` guard were all
  written, commented *"Phase 5's UH-7 ring"* and covered by tests. They were inert because
  `_tick_unified_launch` installed a rest-terminal join. **Superseded by:** § Diagnosis 1. The plan text
  is corrected in this commit.
- **"UH-7 needs two poses."** **WITHDRAWN:** `sim/unified_gate.py` SET 2 *is* a two-pose ring, but
  nothing in the chaining machinery is aim-dependent, and the two-pose ring is blocked by a goal-surface
  limit that has nothing to do with chaining. **Superseded by:** § Discussion (a).
- **"The extend stays blocking."** The coordinator's first cut, with the stall measured and accepted as
  a latency cost. **WITHDRAWN on the safety argument:** a stalled FSM cannot observe a MISS, and at a
  short dwell the hold then arrives after the chained stroke has begun. **Superseded by:**
  § Discussion (b).
- **"`_toss_unified_chain_live`, from `resp.release_terminal`, is the hold predicate."**
  **WITHDRAWN on audit H1:** that flag is False for the last two cycles of every ring, and a probe
  confirmed the old predicate issued **zero** holds with a full stroke ~1.2 s ahead. **Superseded by:**
  the instant-valued `_toss_unified_release_ahead`, § Discussion (g).
- **"`num_throws == 1` is byte-identical to the pre-UH-7a behaviour."** **WITHDRAWN as stated:** it is
  true of the `PlanCycle` **request** (same kind, period and chain fields) and false of the **teardown**
  — a cancel taken before the single release now issues a `trajectory/hold` where it issued none, which
  is stricter and correct: an operator who cancels before the throw gets no throw.
- **"`test_the_gate_and_not_the_qp_dominates_a_healthy_solve` reproduces in isolation, so it is a real
  regression."** **WITHDRAWN on a controlled re-run** (see § Verification): three isolated passes and
  three whole-file passes on a confirmed-quiet box; the earlier "isolation" runs shared the box with
  another agent's suites.

---

## Open Questions

1. **UH-7a has to fly, and the first sitting is capped.** `num_throws` **3, then 5, never more** on the
   first sitting: a ring never lets the hand rest, and nothing in this change models hand-motor thermal
   load. Watch hand `iq_rms` and let the hand cool between rungs. Runbook rung § UH-7a.
2. **UH-7b, the two-pose ring, needs three things and not one** — the aim-authority re-derivation
   against `MAX_TILT_DEG` (12°), a **goal-surface change** (`TossContinuous` has exactly one
   `catch_position` and its own comment calls a waypoint list out of scope), and
   `toss_sequencer`'s `REJECTED_DISPLACEMENT` bound deleted or unified-branched. **Owner items.**
3. **The `main()` executor caveat.** The hold's pre-emption lives in `trajectory_node.main`. Anything
   that spins the node *without* it — a test harness, a future composable-node container, a bench
   driver that builds the node itself — loses the pre-emption **silently**, because a reentrant callback
   group is inert under `rclpy.spin`. `test_main_runs_a_multi_threaded_executor_not_plain_spin` guards
   the shipped entry point and nothing else can.
4. **The joined plan still grows without bound.** The *gate* is now flat, but every `MODE_EXTEND`
   concatenates onto the whole installed plan: 81 → 137 → 193 → 249 → 305 → 361 knots over six windows.
   The concat itself is a vectorised O(n) memcpy whose measured contribution is nil (163 ms flat from 81
   to 361 knots), so this is not the beat's problem today — but a long ring wants either a chain-depth
   cap on the coordinator or a planner that can drop committed head windows.
5. **`validate_cycle` vectorisation is still open, and no longer gates the beat.** It is ~89 % of the
   solve and remains the follow-up the plan names; the bounded ranges took it off the ring's critical
   path, so it now buys headroom under load rather than feasibility.
6. **The `catch_frac` / beat coupling.** The chained window's `chain_catch_frac = flight / beat` is the
   only place the beat enters the *shape* of the plan, and as the dwell falls the catch migrates toward
   the terminal release, squeezing the post-catch carry the QP has to re-load the stroke in. Nothing
   refuses on `catch_frac`; the planner refuses on jerk instead, one cycle at a time, and the ladder
   makes that refusal safe. Two derived bounds ride on the same number and should be re-checked
   together if the beat ever goes below ~1.2 s: `_CYCLE_CATCH_MATCH_S` (0.25 s) must stay strictly under
   `_CYCLE_REPLAN_LEAD_S` (0.30 s) **and** under half the shortest flyable beat.
7. **No chain depth or beat on `/trajectory/status`.** The survey's C3 is served by log lines only. A
   ring is otherwise invisible to a topic reader: `cycle_active` is a type test on the installed plan
   and stays true across the whole chain.
8. **`_replan_cycle_from_target`'s reach-freeze block is dead under unified** — `_catch_arrival_perf` is
   only ever stamped on the legacy `build_catch` path, so the freeze window never engages for a cycle
   replan. Pre-existing; not touched here.
9. **`PlanCycle.srv`'s `catch_frac` comment is slightly behind the code** — the node now always
   nominates the instant on `MODE_REPLAN`, where the comment says `0.0` means "keep the committed
   instant". `0.0` is still accepted and still means that. The `.srv` was deliberately left alone
   (signature freeze, and it lives in `jugglebot_interfaces`, which would make this a two-package
   `colcon build`); the rationale is written into `_plan_cycle_replan` instead.
10. **The one-catch-per-tail backstop is unreachable on today's kinds.** Every kind carrying a catch is
    entered from a release, so the releases interleave the catches and the mid-plan-release bound always
    gets there first. Kept as a structural backstop for a future two-catch kind, driven in test on a
    hand-built meta, with the subsumption asserted on a real one.
11. **Four wall-clock-sensitive tests in this area remain pre-existing load flakes**, each with a
    documented signature: `test_the_gate_and_not_the_qp_dominates_a_healthy_solve` (BLAS pool
    starvation), `test_the_velocity_term_passes_the_same_origin_re_installs`,
    `test_the_launch_lead_makes_the_release_land_LATE_not_early`, and
    `test_a_landing_update_on_a_bare_launch_is_refused_without_a_solve` (no `_refresh` between its
    ~0.5 s solve and the `_on_dynamic_target` call, so a loaded box walks past the 0.5 s freshness
    bound). The gate runs alone by rule, which is the condition every number above was taken under.
    **Do not widen the tolerances.**
