---
title: The tracker was minting the catches, and the cup was right 31 times out of 31
type: investigation
date: 2026-08-26
status: resolved
related_plan: toss-selftuning.md
sessions:
  - 2026-08-26_14-25-16
  - 2026-08-26_08-51-06
  - 2026-08-25_22-07-37
  - 2026-08-25_19-19-57
files_changed:
  - ros_ws/src/jugglebot/jugglebot/ball_possession.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/reload_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/toss_session.py
  - ros_ws/src/jugglebot/jugglebot/toss_record.py
  - ros_ws/src/jugglebot_interfaces/action/TossContinuous.action
  - ros_ws/docs/ball_possession_contract.md
  - tools/probes/possession_replay.py
  - tools/probes/possession_verdict_bag_check.py
  - tools/probes/hand_sensor_verdict_replay.py
  - tools/probes/cadence_rung_check.py
  - tests/ros/toss_verdict_replay_fixtures.py
  - tests/ros/test_possession_replay.py
  - tests/ros/test_ball_possession.py
  - tests/ros/test_reload_coordinator_node.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_toss_continuous_node.py
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_toss_session.py
  - tests/motion/test_cadence_rung_check.py
  - tests/hardware/session_cadence_ladder.md
  - tests/hardware/session_anomaly_fixes.md
  - tools/probes/README.md
  - sim/analysis/log_index.json
subsystem:
  - ros
  - sensor
tags:
  - possession
  - cadence
  - contract
  - measurement
---

# The tracker was minting the catches, and the cup was right 31 times out of 31

## Summary

Bag `2026-08-26_14-25-16` is 31 declared toss cycles over 12 goals, walking the
cadence ladder down from a 5.6 s dwell to 0.65 s. The operator's report was that
the spacing was visibly irregular and that the machine kept calling good catches
misses.

Both are one defect. **The possession verdict was tracker-primary in practice
even though `merge_possession` had been sensor-primary since 2026-08-10**, because
both FSM observation builders only ASKED the possession question inside
`if int(b.status) == _BALL_STATUS_CAUGHT` — a source that gates whether the
question is asked can veto by silence, whatever the merge rule says.

The census, replayed through the production verdict surface. The table adjudicates
the **27** cycles that produced a cup verdict; the 31/31 is the whole bag — those
27 plus **four** rows that never put a ball up (2× `ABORTED_CANT_MAKE_RELEASE`,
2× `REJECTED_NO_BALL`), which the cup also called correctly and which have no
CAUGHT/MISSED column to sit in:

| | CAUGHT | MISSED |
|---|---|---|
| cup sensor (ground truth, operator-confirmed 31/31) | **23** | **4** |
| the shipped FSM verdict | 11 | 16 |

* **15 false MISSED.** All genuine catches, cup edges +0.143…+0.303 s past the
  scheduled landing — every one comfortably inside the 0.560 s confirm window.
  **Twelve had no confirmed tracker track at all** (a mocap coverage hole); three
  had a tracker `CAUGHT` that arrived at +0.615 / +0.622 / +0.830 s, i.e. after
  the deadline.
* **3 false CAUGHT.** The tracker minted over an empty cup. One drove a phantom
  reload — the machine asked BallButler to throw a second ball at a cup it had
  just wrongly recorded as loaded.
* Each false MISSED additionally charged the next cycle
  `DEFAULT_SESSION_MISS_CLEANUP_S`. That is the irregular spacing: run 7's
  releases went 4.310 / 2.303 / 2.335 / 4.319 s where the corrected verdicts give
  a uniform 2.298 s, and run 9's went 3.861 / 3.868 / 3.853 / 3.866 s against a
  uniform 1.448 s.

Three owner decisions landed (2026-08-26):

* **D1** — the cup sensor is the SOLE source of possession verdicts on both FSMs.
  The tracker keeps every other role.
* **D2** — `max_reloads` widens: any failed THROW draws on the budget, not only
  BB's `THROW_ABORTED_NOT_SETTLED`, and every retry re-runs the whole interlude
  ladder so every refusal rung still gates it. A PRECONDITION failure is not a
  throw and does not draw on it — see the audit-fix paragraph in § D2.
* **D3** — the pre-dispatch budget is charged in the FSM loop's measured PERIOD
  instead of its `time.sleep`. This closes the BLOCKING finding
  `tests/hardware/session_cadence_ladder.md` has carried unfixed since
  2026-08-24, and costs 6.8 % of the published frontier.

## Symptom

Two `ABORTED_CANT_MAKE_RELEASE` cycles, 18 mis-adjudicated outcomes out of 27,
and an operator-visible cadence that did not match the requested dwell at any
rung below R2.

## Diagnosis

### The pre-registered hypothesis, and why it is wrong

**H1a (pre-registered): `CATCH_CONFIRM_WINDOW_S` is too short — shrink it.**
The reasoning was that the confirm window is a sensor-band budget
(`= ARRIVAL_BAND_MAX_S = 0.56`) and the machine was minting MISSED on catches, so
the window must be mis-sized.

**REFUTED, and refuted in the opposite direction to the one that was expected.**
The cup edges on the 15 false MISSED sit at +0.143…+0.303 s. The window is
0.560 s. It was not merely adequate, it had **1.85x of margin** — and shrinking
it would have converted more good catches into misses. H1a is the hypothesis this
investigation would have shipped if it had stopped at "which constant is wrong".

What it got right is that a constant was being *spent* wrongly. The confirm
window was budgeted for the SENSOR band and consumed by TRACKER latency: three of
the false MISSED had a perfectly good cup edge at +0.222 / +0.249 / +0.227 s and a
tracker `CAUGHT` that did not arrive until +0.615 / +0.830 / +0.622 s. Two
unrelated quantities were sharing one number, which is the "bound without an
error model" defect C-POSSESS-1 was written to close, one level up.

**So the constant did not change.** The fix is in the CONSUMER, and with the cup
as the consumer the derivation closes on its own terms: the latest cup edge in the
whole bag is +0.303 s against a 0.560 s ceiling. `CATCH_CONFIRM_WINDOW_S` was
correct by luck and is now correct by construction — a distinction recorded at
both copies of the constant, because "it did not change" is exactly the kind of
non-event a future reader will not think to check.

### The root cause, which was not pre-registered

`merge_possession` was already sensor-primary. What was not is the *asking*:

```python
if int(b.status) == _BALL_STATUS_CAUGHT:              # reload_coordinator_node:2060
    if self._possession_confirmed(b, ref_point_mm=landing_ref):
```

and the same shape at `:1633` on the reload FSM, and a third at `_on_balls`'s
possession latch. On the twelve cycles where the tracker never confirmed a track,
the cup was reading a seated ball and nobody asked it. The tracker could not
CONFIRM a catch it had not seen, but it could and did SUPPRESS one.

This is a class, not three sites: **a source that gates the evaluation of a
verdict is primary regardless of what the merge rule says.** The contract
(C-POSSESS-1 § 3.2) had the merge rules right and said nothing about who decides
when to ask, so the enforcement point enforced nothing on the path that mattered.

### D3's second thread — the two aborts

Both `ABORTED_CANT_MAKE_RELEASE` cycles are a separate, independent defect that
the same bag happens to carry:

| cycle | v (m/s) | dispatch budget | `throw_delay_s` | old accept floor | lead at the guard |
|---|---|---|---|---|---|
| run 2 c1 | 2.48 | 0.3344 | 0.440 | 0.4144 | 0.330 → ABORT |
| run 10 c2 | 3.92 | 0.2813 | 0.400 | 0.3613 | 0.272 → ABORT |

Both cleared the accept gate — by 26 ms and 39 ms — and then died at the runtime
guard in PREPARING, with the catch latch raised, the announcement published and a
phantom tracker expectation left behind. That is precisely the failure
`min_throw_delay_for_release_s` exists to make unreachable from static arithmetic.

It stayed reachable because `pre_dispatch_budget_s` counted the tick ladder in
`NODE_TICK_S` — the `time.sleep` at the bottom of `_run_toss_cycle` — and the
pre-dispatch ticks are the expensive ones: the observation build, a blocking
`trajectory/arm_catch` raise-and-confirm, the announcement build and publish.
Measured over **28 cycle starts** in this bag, cycle start → `/throw_ann` is
exactly 3 loop iterations and takes **0.080–0.113 s**, i.e. **0.0267–0.0377 s per
iteration** against a 0.020 s sleep. Ceiled to the next 10 ms:
`NODE_LOOP_PERIOD_S = 0.040`.

## Discussion

### Why sensor-ONLY, and not sensor-OR-tracker

The obvious smaller change is to keep the tracker as a fallback and merely ask the
question every tick. It was rejected, and the reason is not conservatism.

**The fallback is a fallback to the LESS reliable observable, keyed on the more
reliable one being quiet.** The tracker's `CAUGHT` estimate is a dead-reckoned
free-fall extrapolation from the last real sighting — that is the whole content of
C-POSSESS-1 § 1 — and its arrival time is the instant a mocap marker *vanished*,
which is a property of the coverage, not of the ball. Falling back to it on
exactly the ticks where the cup said nothing is selecting for the case where the
cup is blind, which is not correlated with the tracker being right.

The numbers on this sitting settle it without appeal to principle: the tracker
scored 11/16 where the cup scored 23/4, and the cup was the one the operator
confirmed 31/31 by eye. "Strictly less capable than before the sensor landed" was
the argument for the fallback in the 2026-08-10 merge docstring. Capability is not
the metric. Being right is.

**Owner decision, revisitable, and simplicity is part of it.** A tracker phase
that fixes the split-track mis-association may earn a corroboration role back —
that is a decision to re-take with data, not a code path to leave armed in the
meantime. `TrackerArrivalSource` is kept, not deleted: `arrival_err_mm` is the
catch-accuracy number the hardware runbooks score and the cup cannot supply it.

### What replaces the fallback: two UNKNOWNs, told apart

Deleting the fallback exposes a distinction that did not previously matter.
`ARRIVAL_UNKNOWN` covers two states:

* **"I could not look"** — `SENSOR_BLIND`, `SENSOR_NO_LANDING`. A machine fault.
* **"I am still looking"** — `SENSOR_WINDOW_OPEN`, `SENSOR_BAND_CLAMPED`.
  Ordinary; it is what every tick before the ball seats returns.

While the tracker was the fallback both degraded to a second opinion and no caller
had to ask which it had. With the cup as the sole source they demand opposite
handling, so `arrival_blind()` separates them and both FSMs mint
`MISSED_SENSOR_BLIND` on the first — same MISSED family (the terminal action, the
`stop_on_miss` governance and the session accounting are all keyed on the prefix),
a different name, which routes an operator to the sensor instead of to the throw.

**Why a reason-string predicate and not a fourth `ARRIVAL_*` member**: the states
are already distinguished, and a fourth member would force every existing
`arrival == ARRIVAL_UNKNOWN` comparison in the tree to be re-audited for which of
the two it meant. It also must NOT collapse to "any UNKNOWN at the deadline is
blindness": both FSMs terminalise at `landing + CATCH_CONFIRM_WINDOW_S`, which IS
`ARRIVAL_BAND_MAX_S`, so at that instant a still-open window has watched the whole
measured band and "no rise" is a real miss. Mapping the tri-state directly would
have turned every single `Toss` into `MISSED_SENSOR_BLIND`, because the arrival
window runs to `landing + 1.5 s` and the FSM stops watching at `+0.56`.

### D2 — why widening `max_reloads` is safe, stated as the failure it prevents

The old rule retried only `THROW_ABORTED_NOT_SETTLED`, on the argument that "a
blanket retry would swallow the BB fail-open boot bug and every real BB fault, and
would keep asking an unwell machine to throw real balls". That is right about the
hazard and wrong about the remedy: the boot-bug fence is `STOPPED_BB_UNVERIFIED`
and the unwell-machine fence is `STOPPED_BB_NOT_READY`, both rungs of the
interlude GATE — and **the gate ran once, outside the retry loop**. Keying on the
code was doing the fences' job badly instead of running the fences.

So D2 moves rungs 1 and 2 INTO the loop. A retry is now a repeat of the whole
ladder: precondition gate (ball-evidence config, BB ready, BB verified, cup
CONFIRMED-EMPTY through the settled query) and a VERIFIED `go_home` recentre, per
attempt. `STOPPED_CUP_NOT_EMPTY` is the load-bearing one — it is what stops a
retry throwing a second ball at a cup the first one actually reached, and before
D2 no retry could see it.

The operator cost of the old rule was concrete: a DELIVERED reload that missed the
cup stopped the sitting on its first miss, and `max_reloads` fenced one of the two
ways a reload can fail while the other cost a whole sitting.

**AUDIT FIX, same day: "any failed attempt" was one word too wide — it is any
failed THROW.** As first written the loop retried on ANY non-success
`ReloadResult`, which includes every `REJECTED_*` (wrong mode, mocap stale, not
streaming, no ball at BB, BB busy/disconnected) and `ABORTED_BB_ERROR` /
`ABORTED_TIMEOUT` / `ABORTED_SHUTDOWN`. Those are preconditions the operator must
fix, not balls, so retrying them spent the BALL budget on non-ball failures — and
then collapsed every one of them into `STOPPED_RELOAD_BUDGET`, which
`tests/hardware/toss_cal_grid.py` reads as "the node exhausted its reloads, skip
this node". A faulted BallButler would therefore have completed a thin calibration
grid **silently**, which is the one failure shape a capture tool must never have.
They now stop the session by name (`STOPPED_RELOAD_<outcome>`), before any budget
is drawn and before the recentre is re-dispatched. Three ABORTED codes are
deliberately left drawing budget and named in the docstring
(`ABORTED_MODE_CHANGED`, `ABORTED_PRIME_FAILED`, `ABORTED_PREPARE_FAILED`) —
extending the list is an operator-visible behaviour change and waits for the
owner.

### D3 — the tradeoff, accepted with its number

**D3 costs 6.8 % of the published cadence frontier: 54.3 → 50.6 throws/min**, and
re-cuts three published ladder rungs (R4, R5, R5-prime), all of which now sit
INSIDE their delay floors and would be `REJECTED_THROW_DELAY` as published.
R5-prime collapses onto R5 — under the corrected floors there is no tighter legal
pair at that height, which was its entire reason for existing, so **the owner
retired the row on 2026-08-26** and the runbook carries a tombstone.

**The owner also declined the razor edge at R5.** The smallest legal pair at that
height is `0.72 / 0.51`, 2.0 ms clear of the delay floor; the published rung is
`0.76 / 0.55`, restoring the **42.0 ms** of delay clearance the rung carried
before D3 at a cost of **1.6 throws/min** (49.1 → 47.5). The delay axis is the
one that matters: it is what `ABORTED_CANT_MAKE_RELEASE` is measured against.
⚠ The DWELL clearance did NOT follow — `required_dwell_s = throw_delay +
handoff_margin`, so the floor moved up with the delay and 0.76 clears by 1.9 ms.
That is a `REJECTED_DWELL` at the accept gate, not an abort in flight; `dwell
0.78` (46.8 throws/min) restores the pre-D3 21.9 ms if a sitting hits it.

That is a real loss and it is worth stating plainly why it is the right trade:
**the 54.3 was a cadence the gates advertised and the machine could not make.**
This sitting proves it — two cycles cleared the accept floor and aborted with the
hand committed. A floor that is exceeded in practice is not a floor, and the
failure mode it permits is the one the whole 2026-08-23 accept-floor package was
written to close.

**The way back is the LOOP, not the floor.** The skip budget is four loop
iterations, so every millisecond removed from the observation build or from the
PREPARE bundle's blocking service calls is worth four in the delay floor. Those
are real, measurable levers, and `NODE_LOOP_PERIOD_S`'s comment carries the
one-grep recipe for re-measuring it off the next bag. Relaxing the floor instead
just re-buys the abort.

### D3's second half — the cleanup floor, and a defect nobody had named

`DEFAULT_SESSION_MISS_CLEANUP_S` was `CATCH_CONFIRM_WINDOW_S + GO_HOME_DURATION_S
+ 2 x NODE_TICK_S`, which starts the 2.0 s recentre-profile clock at the MISSED
verdict instant. It does not start there: the SAFE_ABORT ladder — catch/armed
False, a telemetry-VERIFIED hand retract, `arm_catch` lower, then `go_home` — is
four blocking dispatches, and all four were charged zero.

The evidence was in the bag and had not been read: `trajectory_node` printed
*"catch latch armed mid-move — installed a graceful stop (move silenced)"* on
**10 of the 16 post-MISS toss cycles**, at `cycle_start + 0.0566…0.0717 s`.
Re-basing that onto the verdict needs one step the first write-up left out — the
**verdict → cycle-start gap**, which is `DEFAULT_SESSION_MISS_CLEANUP_S −
CATCH_CONFIRM_WINDOW_S = 2.60 − 0.56 = 2.040 s` on the build that flew:

    install ≥ 2.040 + [0.0566 … 0.0717] − GO_HOME_DURATION_S (2.0)
            = **+0.0966 … +0.1117 s past the verdict**

(re-derived from the bag's `/rosout` on 2026-08-26, audit fix W8. This supersedes
the *+0.099…+0.129 s* pair the first write-up published, which does not re-derive
from the bag by any reading of it. The constant is unchanged.)

**That is not a tidiness issue.** Arming the catch latch mid-move installs a
graceful stop, so the platform halts wherever the interrupted `go_home` left it and
the cycle then throws from a site A its aim was not solved for. It is the
arrived-before-arming invariant `_step_positioning` enforces WITHIN a cycle,
defeated ACROSS cycles by a floor that ends before the move does.

`SAFE_ABORT_LADDER_S = 0.160 s` bounds the largest of the ten lower bounds
(+0.1117 s) by **1.43x**. It is written as `4 x NODE_LOOP_PERIOD_S` so a loop-cost
re-measure moves both ladders together, and **not** because the ladder spends four
iterations — `_safe_abort` runs all four dispatches inside ONE iteration of
`_run_toss_cycle`, with no sleep between rungs (rungs 1–3 measured at
+0.022…0.025 s TOTAL). It is honestly a bound over lower bounds — the profile may
have run past the arm instant on any of them — and the residual is stated at the
constant along with its acceptance test, which is an ABSENCE: grep the next
sitting's log for that line.

### The framing that had to be reset

The first framing of the aborts was the one the brief carried: *"the cycle
following a SAFE_ABORT ladder must be charged the MOVING pre-dispatch budget,
measured cost 0.338 s"*. It does not survive tracing.

* The 0.338 s is `cycle_start → the outcome LOG line`, and the log line is emitted
  *after* `_step_toss_sequence` has run the whole SAFE_ABORT ladder. The guard
  actually fired at ~+0.132 s, which the arithmetic confirms exactly: the guard
  passed on the announce tick (+0.108, lead 0.292 > 0.2813) and failed on the next
  one.
* **Run 2 cycle 1 was not a continuation past a SAFE_ABORT at all.** It is cycle 1
  of a fresh goal on a machine that had been quiescent for 90 s. A post-abort
  charge explains one of the two aborts and misses the other, and the mechanism it
  names — the platform still traversing — is real (10 cycles show it) but costs no
  lead: run 10's announce landed at +0.108 against run 2's +0.090, both inside the
  0.080–0.113 s spread of all 28 cycle starts.
* Charging the moving budget would also not have fixed the thing that mechanism
  causes. A longer `throw_delay_s` moves `t_release` later; PREPARE still fires
  ~0.06 s after cycle start, still mid-move. The lever for "the platform is still
  traversing" is the CLEANUP FLOOR, not the lead — which is why D3 landed as two
  edits and not one.

Both threads then converge on one constant, which is the part worth keeping: the
loop period is charged four times in the pre-dispatch ladder and four times in the
SAFE_ABORT ladder, and both were previously charged at zero for the same reason.

## Control-system walk — one cycle, end to end

Stated per gate, because the one thing this change must not do is move a motion
command.

| stage | what changes | what cannot change |
|---|---|---|
| **CHECKING** | nothing. `ball_seated` was already the LIVE `evidence(now)` read (C-POSSESS-1 § 3.3 edit 1) and is untouched. `REJECTED_BALL_UNKNOWN` still refuses a blind sensor before anything flies — the dead-sensor-refuses posture is unchanged and is still the FIRST fence. | The accept-time gates now charge more lead (D3): **+0.080 s** on the CHAINED (census-B1 skip) floor and **+0.060 s** on the first-cycle MOVING one — only the `+3` term of `(arrival_ticks + 3) × loop` scales with the period, because the moving path's 0.400 s arrival is wall-clock and its tick count halves as the period doubles. So some goals are REFUSED here that were previously accepted-then-aborted. Refusing commands nothing. |
| **POSITIONING / PREPARING** | nothing. The B1 skip predicate, the arrived-before-arming rule, the PREPARE bundle order and the ≥1-tick announce gap are byte-identical. | — |
| **release** | nothing. `t_release` is still `cycle_start + throw_delay_s`; the dispatch is still single-shot and never retried. | The runtime guard's inequality is unchanged; only the accept floor that fronts for it moved. |
| **flight** | `ball_caught` is now evaluated on EVERY tick instead of only on ticks carrying a tracker `CAUGHT`. It is a pure read of a deque under a lock — no I/O, no allocation in the loop. | The FSM still finishes on the FIRST confirmed tick; no new blocking work enters the tick. |
| **cup edge** | THE change. `ARRIVAL_CONFIRMED` at the empty→held edge, +0.087…+0.555 s past the landing (measured band), against the tracker's +0.29…+0.83 s. **The CAUGHT terminal therefore fires ~0.18 s EARLIER, median.** | On the shipped default (`toss_stay_at_pose_on_caught: true`) the CAUGHT terminal is `ACTION_STAY`, which commands NOTHING — so on the toss path this is a reporting-time change and not an actuation-timing change at all. On `RECENTER` (the reload path, and a toss with that config false) the `go_home` is dispatched up to ~0.2 s earlier. That is dispatched under a STRONGER precondition than before — the ball is provably in the cup, rather than a marker having vanished — and `go_home` is a 2.0 s profiled move. Flagged as the one place a motion instant moves. |
| **verdict vs the window** | `CATCH_CONFIRM_WINDOW_S` unchanged at 0.56 s; it is now spent on the band it was derived from. | — |
| **miss_cleanup** | applied to strictly FEWER cycles (15 of this bag's 16 MISSED were catches), and it grows 2.60 → 2.80 s (D3) when it does apply. | It is a FLOOR: `next_at = max(dwell-derived, landing + miss_cleanup)`. It never shortens a cadence and never moves a command earlier. |
| **`stop_on_miss` fence** | becomes trustworthy. It was firing on catches. | Its semantics are unchanged; `MISSED_SENSOR_BLIND` is in the same MISSED family by prefix, so the fence covers it. |
| **reload entry** | `REJECTED_NO_BALL` → interlude is unchanged, and the phantom-reload trigger is gone — **because the `ball_caught` VERDICT is now the cup's**, so a tracker `CAUGHT` over an empty cup no longer terminalises a cycle as caught. (Audit fix N13, 2026-08-26: this row used to credit the `_ball_possession` latch. It cannot be the latch — since D1 made `ball_seated` a live `evidence(now)` read, that latch has no consumer beyond its own self-clear, and no observation field reads it. The interlude is entered from `REJECTED_NO_BALL`, which is a LIVE cup read.) Retries now re-run the gate + verified recentre per attempt (D2). | The interlude still refuses on cup-not-empty / sensor-unknown / BB-not-ready / BB-unverified / recentre-failed / budget, and every one of those now gates every retry rather than only the first attempt. |

Two further invariants checked by hand and worth recording:

* **The possession LATCH moved from `/balls` to `/hand_telemetry`.** A valid
  SEATED sets it, a valid EMPTY clears it, UNKNOWN touches neither. It still
  survives a SAFE_ABORT retract — but now because the ball really is still seated
  (the cup carries it down at ~3.16 m/s² ≪ g), not because a flag was left alone.
* **No new read can mint a catch out of a reload.** `ball_caught` is read only in
  `_step_in_flight` and `_step_settling`, pinned structurally by a test. This
  matters: the bag's run 12 cycle 2 (`REJECTED_NO_BALL`, minted in CHECKING)
  replays as CAUGHT off an operator hand-reload 0.92 s after a landing that never
  happened. It is unreachable today and the test says why.

## Fix

* **D1**, `ball_possession.py`: `merge_possession(sensor, tracker=None)` takes
  ARRIVAL from the sensor unconditionally; the tracker supplies only the
  report-only `arrival_err_mm` / `plane_drop_mm`. New `arrival_blind()` +
  `BLIND_REASONS`. `SOURCE_MERGED` retired as an author, kept for replaying old
  logs.
* **D1**, `reload_coordinator_node.py`: `_possession_observed(now, ball=None, …)`
  is the tick-driven seam; `ball` is optional and report-only. Both observation
  builders call it once per tick, outside any tracker branch. `_on_balls` carries
  no possession claim; the latch moved to `_on_hand_telemetry`. `now` is threaded
  through so one tick is one instant.
* **D1**, both FSMs: `possession_blind` observation field, `MISSED_SENSOR_BLIND`
  terminal, and the `CATCH_CONFIRM_WINDOW_S` derivation restated at both copies.
* **D2**, `_run_reload_interlude`: the gate + verified recentre run per attempt;
  any failure retries within budget; `_BB_NOT_SETTLED_CODE` demoted to a log
  label. IDL `max_reloads` docs rewritten.
* **D3**, `toss_sequencer.py`: `NODE_LOOP_PERIOD_S = 0.040`;
  `pre_dispatch_budget_s` / `min_throw_delay_for_release_s` take `loop_period_s`.
  `toss_session.py`: `SAFE_ABORT_LADDER_S`, and
  `DEFAULT_SESSION_MISS_CLEANUP_S` 2.60 → 2.80 s.
* Ladder rungs re-cut in `tools/probes/cadence_rung_check.py` and
  `tests/hardware/session_cadence_ladder.md`; the runbook's carried BLOCKING
  finding 1 marked closed (finding 2 still carried).

## Verification

**Acceptance — production-faithful offline replay.**
`tools/probes/possession_replay.py` drives the real `HandBallSensorSource`
(constructed from the generated config as the node constructs it), the real
`merge_possession`, the real cadence clamps and the FSM's own settle deadline,
over the bag's own `/hand_telemetry` stream. Run 2026-08-26:

```
python tools/probes/possession_replay.py --bag ~/Desktop/rosbags/2026-08-26_14-25-16
  -> replayed census: {'CAUGHT': 24, 'MISSED': 7}
     shipped  census: {'CAUGHT': 11, 'MISSED': 16, 'ABORTED_CANT_MAKE_RELEASE': 2,
                       'REJECTED_NO_BALL': 2}
```

Restricted to the 27 cycles the cup adjudicated: **23 CAUGHT / 4 MISSED, matching
the cup label on every single row.** All 15 false MISSED flip to CAUGHT, all 3
false CAUGHT flip to MISSED, and run 5 cycle 3's genuine drop stays MISSED. Pinned
as `tests/ros/test_possession_replay.py` against
`tests/ros/toss_verdict_replay_fixtures.py` (run-length compressed **21,753
samples → 162 segments**; the probe refuses to emit a fixture whose expanded
stream answers differently from the raw one on `(label, blind, catch_dt_s)`, so
the compression is **verdict-equivalent at emit time against the source bag** —
which is what it is checked for, and is a weaker claim than lossless: the
encoding re-lays sample instants on a nominal `step_s` grid, and the gate is
sound for this source only because `HandBallSensorSource` reacts to exactly three
things and all three are segment boundaries).

Schedule arithmetic, driven through the real `TossSessionSequencer`: with every
cycle CAUGHT, run 7's release spacing is a uniform **2.298 s** and run 9's
**1.448 s** (`flight + dwell`), against the 4.310/2.303/2.335/4.319 and
3.861/3.868/3.853/3.866 the bag recorded.

**Gate.** `./run_tests.sh --full` (every tier, `nightly` included — mandatory
here: this precedes a hardware sitting and moves the cadence surface), run
2026-08-26: **6395 passed, 4 skipped, 3 xfailed in 531.15 s** (parallel) +
**9 passed in 41.76 s** (serial); `Summary [full]: parallel 533s (rc=0) | serial
46s (rc=0) | total 579s`, `RESULT: PASS`.

Plus a single-process run under the default RANDOM ordering, because the first
`--full` attempt went RED on three `test_reload_coordinator_node.py` tests that a
scoped `-p no:randomly` run had passed — a real defect in the new test helper, not
a flake: it fed the cup on `time.perf_counter()` while the production path now
takes the tick's instant, which silently disabled every blind/refused assertion
(`ARRIVAL_CONFIRMED` depends on the EDGE, not on `now`, so the positive cases kept
passing). `pytest tests/ros/ tests/motion/test_cadence_rung_check.py -q`, run
2026-08-26: **2384 passed, 1 skipped in 322.01 s**.

**Bag replay**, run 2026-08-26:
`python tools/probes/possession_replay.py --bag ~/Desktop/rosbags/2026-08-26_14-25-16`
→ `replayed census: {'CAUGHT': 24, 'MISSED': 7}` (23/4 over the 27 cup-adjudicated
cycles, plus the 4 rows that never reach a verdict) against
`shipped census: {'CAUGHT': 11, 'MISSED': 16, ...}`.

**The loop-period measurement**, re-derived from the same bag's `/rosout`
(28 cycle starts, cycle start → `/throw_ann`): min 0.080, median 0.0910, max
0.113 s over 3 loop iterations ⇒ **0.0267 / 0.0303 / 0.0377 s per iteration**;
4 iterations at the max is 0.1507 s against the 0.160 s now charged and the
0.080 s charged before.

## Outcome

Possession is the cup's. The confirm window did not move and now means what it
says. `stop_on_miss` is a fence again. Two abort classes are refused at accept
instead of aborting with the hand committed, at a stated cost of 6.8 % of the
cadence frontier and three re-cut ladder rungs.

## Open questions

* **The frontier is a LOOP-cost problem now.** 50.6 → 54.3 throws/min needs
  0.0377 s → 0.020 s per FSM iteration, not a smaller floor. The candidates are
  the PREPARE bundle's blocking service calls and the per-tick observation build.
  R4's published clearances are 3.9 / 8.7 ms and R5's DWELL clearance is 1.9 ms,
  so any upward re-measure of `NODE_LOOP_PERIOD_S` takes both rungs out again.
  R5's DELAY carries 42.0 ms after the owner's margin re-cut.
* ~~**R5-prime coincides with R5.**~~ **RESOLVED 2026-08-26**: the owner retired
  the row; the runbook keeps a one-line tombstone pointing at R5, which is now
  the starred operating point.
* **First bench read of all of this is unvalidated.** Expect: no
  `catch latch armed mid-move` lines (the D3b acceptance), sessions running
  FURTHER before `stop_on_miss` fires, and `catch_error_mm` NaN on a large
  minority of confirmed catches (the tracker did not see them — that is the
  honest reading, not a fault).
* **Phase B–E roadmap** (owner will commission the plan separately): pipelined
  preamble → beat clock with bounded slip → displaced pose → ILC unpark. Phase A
  is the prerequisite for all four — a beat clock cannot be built on a verdict
  that mis-scores 18 cycles in 27, and the loop-cost work above is what the
  pipelined preamble is for.
