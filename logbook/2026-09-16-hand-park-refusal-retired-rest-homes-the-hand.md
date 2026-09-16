---
title: "REJECTED_HAND_NOT_PARKED retired — the opening REST brings the hand home"
type: bugfix
date: 2026-09-16
status: done
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py (delete REJECTED_HAND_NOT_PARKED, the hand_at_seed/hand_at_park Observations fields and the fresh_origin argument)
  - ros_ws/src/jugglebot/jugglebot/skill_node.py (stop building the two predicates; skills/check REPORTS pos_meas + pos_cmd instead of gating)
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py (_SEED_HAND_RECONCILE_TOL_REV; _cycle_start_state reconciles a hand seed against the encoder when at_rest)
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py (_run_activate writes _last_hand_cmd from the park it fired, so /hand_telemetry pos_cmd is not stale after ACTIVATE)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md (§ 8 row rewritten RETIRED, enforcement point moved to the seed)
  - sim/skills_gate.py (drop the two Observations fields)
  - config/hardware_config.yaml (hand_park_band_rev comment — FSM-only reader now)
  - tests/motion/test_skills_executor.py (retire the 3 Unit B tests, add the 3 retirement tests, drop the ladder row)
  - tests/ros/test_skill_node.py (observations + skills/check)
  - tests/ros/test_install_segment.py (5 new seed-reconciliation tests)
  - tests/ros/test_teensy_bridge_node_activate.py (2 echo tests)
  - tests/hardware/session_skills_r3.md (refusal table + the retired sitting-1 recovery)
  - tests/hardware/session_skills_r3_apex_ladder.md (§ 3 premise, row 13 expectation)
  - plans/active/two-ball-skill-stack.md
---

## Symptom

At the 2026-09-16 R3 sitting (log `temp/logs/launch_r2gate_20260916_1416.log`,
bag `~/Desktop/rosbags/2026-09-16_14-16-38`) one attempt ended
`SPLICE_TOO_LATE` and installed a hold that left the hand commanded to
**+0.5639 rev**. From that moment **every** later schedule was refused at
skill 0 — the opening REST —

```
END REJECTED_HAND_NOT_PARKED at skill 0 (REST)
```

**nine times**, across several DEACTIVATE/ACTIVATE cycles, with the hand
MEASURED at **0.0000 rev**. The runsheet's own recovery for that code was
exactly DEACTIVATE/ACTIVATE, and running it changed nothing. The sitting ended
with no further throws.

## Diagnosis

`/hand_telemetry` at each refusal read `pos_cmd = +0.5639`, `pos_meas =
+0.0001`. The ladder (`motion/skills/executor.py::precondition_refusals`)
refused a fresh-origin skill when either half of a two-part predicate was
false (`skill_node._observations`):

* `hand_at_seed` — `|pos_meas − pos_cmd| ≤ 0.5 rev`, a TRACKING-error check;
* `hand_at_park` — `|pos_meas − 0| ≤ 0.5 rev`, an ABSOLUTE-position check.

`pos_cmd` is not the firmware's live hand target. It is the last
`HAND_CMD_ECHO` the bridge firmware **emitted**, and that uplink is
event-driven off the streamed lane:
`telemetry.cpp::hand_cmd_echo_uplink_step` emits only when
`interp_hand_sent()` has changed — "silent while the hand lane is idle". The
ACTIVATE park is not that lane: `leg_activate.cpp`'s COMMAND phase TRAP_TRAJs
axis 6 to `HAND_ACTIVATE_POSITION_REV` straight onto the Jugglebot bus, and
CAN `SRX_DIS` means the bridge never sniffs its own frame. So the echo froze
at the hold's value and reported it for the rest of the session. Nine of the
ten refusals were reading a **phantom** 0.56 rev tracking error on a hand the
firmware had just parked. (Only the very first, at 1789532454.639, was
genuine: hand 0.5605, echo 0.5639 — `hand_at_seed` true, `hand_at_park`
false.)

## Discussion

### The refusal was always a proxy, and this is the bill

The row exists because of latch L2 at the first R3 sitting
([2026-09-14-skill-stack-r3-first-powered-sitting](2026-09-14-skill-stack-r3-first-powered-sitting.md),
Unit B): a REST was planned from a COMMANDED hand pose 0.76 rev off the
encoder, itself 8.67 rev from the park the bridge's recovery slew was about to
hold it to; the plan streamed the hand at ~9 rev/s into that slew and
`MAX_DEVIATION` latched. The failure class is *"the plan's hand seed is not
where the hand actually is / where the firmware will let it go"* — and that is
a statement about the **seed**, not about the hand. The refusal asked a
different question ("is the hand at park?") and got the right answer once and
the wrong answer nine times.

Climbing one level: a seed that disagrees with the machine is a **correctable**
condition, not a refusable one. Every fresh-origin window already has a
mechanism for moving the hand — the schedule's own opening REST exists to lift
the hand onto the site (`schedule.FLOOR_LIFT_S`, 1.5 s) — so the honest fix is
to seed that window from the truth and let it do its job.

### What was ruled out

**Fixing the echo alone.** Tempting, because the echo *is* the bug in the nine
false refusals. Rejected on two grounds. (a) It leaves the tenth refusal — the
genuine 0.5605 rev hand — refusing an attempt for a condition the machine can
simply fix, which is the owner's own objection. (b) The echo is the wrong
reference for a seed in the first place: it is the *Platform/interp command*,
whereas a seed has to be continuous with what the emitter is about to ship and
where the encoder actually is. A gate built on a diagnostic channel is a gate
that fails whenever the diagnostic does, which is what happened. The echo is
still repaired (below), but as diagnostic honesty, with nothing gating on it.

**Widening the park band.** 0.5 rev → something larger would have passed the
nine false refusals and the genuine one, and would also have passed the L2 case
(8.67 rev off park) if widened far enough to be useless. A band wide enough to
never be wrong is a band that protects nothing; a band narrow enough to protect
protects against a fact the encoder already reports exactly. This is the
"just relax this one invariant" move, and the answer is the contract, not the
carve-out.

**Stretching the REST when the seed is far.** Specified in the brief as a
contingency and measured away. Probe (2026-09-16, venv, R3 session limits
300/5000/200000 mm + hand 3500 rev/s², the schedule's real ACTIVATE-park → P1
(−50, 0) move, 1.5 s `FLOOR_LIFT_S`): the REST plans CLEAN from **every** seed
in the hand's stroke — 0.0001 rev at 0.31 rev/s peak, 0.5639 at 0.26, 2.0 at
1.72, 5.0 at 4.77, 8.0 at 7.82, 9.9 at 9.76 — all three orders under the
200 rev/s session ceiling, and the hand's whole travel is 9.959 rev. There is
no seed the existing window cannot absorb, so a duration-stretch mechanism
would be dead code guarding an empty set. Not built.

### Where the tolerance came from, and the tradeoff accepted

The reconciliation fires when the commanded and measured hand disagree by more
than `_SEED_HAND_RECONCILE_TOL_REV` = **0.05 rev**, which is the firmware's own
`SCHED_RESUME_TOL_POS_HAND_REV` (`canbridge_config.h:426`) — what
`sched_apply` allows between a promoted frame's `u0[6]` and the hand it is
already holding. Below it the two readings describe the same machine *by the
firmware's definition* and cannot produce a promotion refusal or a command step
any guard can see; at or above it they describe different machines, and the
encoder is the one that is not a belief. 0.05 rev is 1.63 mm of slider.

Not `_ARM_U0_HAND_TOL_REV` (0.625 rev): that is an *arming* gate, so using it
would leave a 0.6 rev disagreement unreconciled and then hand the arming gate a
seed it refuses. Not `HOMING_HAND_PARK_BAND_REV` (0.5 rev): that was the
retired refusal's own bar and describes where the hand is *allowed* to be, not
when two readings of it disagree.

**The tradeoff.** Reconciling means the installed plan's knot 0 deliberately
differs from what the wire is currently holding — a command step. It is in the
safe direction (the new command matches the encoder, so `MAX_DEVIATION` goes
*down*, not up), and `_install_continuity_ok` is the outer bound on it,
**left untouched**: it compares knot 0 against `_commanded_hand_state` at
1.0 rev (a quarter of the margin-discounted pump gate, half
`MAX_LEAD_HAND_REV`). A reconciliation inside that installs; one beyond it is
refused `STALE_STATE`. That is accepted deliberately — a >1 rev
commanded/measured disagreement on a *stationary* hand is a machine fault, and
stepping the wire that far in one 25 ms knot sits within a factor of 2.5 of the
`MAX_DEVIATION_HAND_REV` E-STOP band. Both real cases are inside it: 0.564 rev
here, 0.761 rev at L2.

### The gate is `at_rest`, not the branch — and that nearly went wrong

The reconciliation must be gated on the machine being at rest: with the hand
moving, source (1) of `_commanded_hand_state` (the active plan's own
`hand_at(tau)`) is exact at every instant and the *measurement* is the channel
that lags — by the whole launch — so reconciling there would fold tracking
error into knot 0, which is the defect that method exists to prevent.

The first implementation put the check inside `_cycle_start_state`'s
`if not post_release:` branch, which reads like the rest branch and is not:
`_KIND_SHAPE[SETTLE]` carries `post_release=True`, so the skill stack's own
REST — the one window the nine refusals were blocking — takes the *other*
branch. The test caught it (`state.hand_rev` came back 0.5639), and the check
now sits directly after `at_rest` is computed, governing both branches. Worth
recording: the branch name and the physical condition do not coincide here.

## Fix

1. **`REJECTED_HAND_NOT_PARKED` is gone** from the skill ladder, with
   `hand_at_seed`, `hand_at_park` and the `fresh_origin` argument that gated
   it. `REJECTED_HAND_STALE` (on `hand_fresh`) is KEPT and is now
   load-bearing, because the replacement reconciles against the encoder. The
   FSM's own identically-named refusal (`toss_sequencer`'s `hand_parked`) is
   untouched — that stack dies at R4.
2. **`trajectory_node._cycle_start_state` reconciles the hand seed.** When the
   machine is at rest and the commanded hand seed disagrees with
   `_latest_hand_rev` by more than 0.05 rev, knot 0 is seeded from the ENCODER
   and ONE warning names both values (`HAND SEED RECONCILED for <kind>:
   commanded X rev vs MEASURED Y rev`).
3. **The opening REST carries the hand home.** No code change needed: a REST is
   a SETTLE aimed at `SETTLE_CUP_Z_MM`, so it already plans the hand from its
   seed to the settle clamp (**0.3071 rev** — not 0.0: the QP's cup box is
   inset 10 mm above the homed zero, so a window asked to settle at the literal
   park is refused `SETTLE_SITE`; 0.3071 rev is inside the 0.5 rev park band
   with 39 % to spare).
4. **The bridge echo follows the park.** `_run_activate` writes
   `_last_hand_cmd` from `JB_OP_HAND_ACTIVATE_POSITION_REV` on a COMPLETED
   activate (ACTIVATE fires AXIS_ALL, so the firmware parks axis 6 regardless
   of which axes the host observes). Diagnostic only. The durable fix is the
   firmware emitting its own echo after the park; this is the half that needs
   no flash.
5. `INVARIANTS.md` § 8's row is rewritten RETIRED, naming the new enforcement
   point and its tests; `hand_park_band_rev`'s comment now says it is an
   FSM-only reader.

## Verification

* Scoped (`pytest tests/motion/test_skills_executor.py tests/ros/test_skill_node.py
  tests/ros/test_install_segment.py tests/ros/test_teensy_bridge_node_activate.py -q`,
  run 2026-09-16): **162/162 pass**.
* Wider (`pytest tests/sim/ tests/motion/test_skills_segments.py
  tests/motion/test_skills_schedule.py tests/motion/test_skills_executor.py
  tests/ros/test_skill_node.py tests/ros/test_install_segment.py
  tests/ros/test_teensy_bridge_node_*.py -q`, run 2026-09-16): **1068 passed,
  4 skipped, 2 xfailed in 516.25 s**.
* The FSM stack's identically-named refusal is untouched
  (`pytest tests/ros/test_toss_session.py tests/ros/test_toss_sequencer.py -q`,
  run 2026-09-16): **308/308 pass**. The same files inside a wider
  fixed-order run alongside `trajectory_node` (`pytest tests/sim/
  tests/motion/test_skills_segments.py tests/motion/test_skills_schedule.py
  tests/ros/test_toss_session.py tests/ros/test_toss_sequencer.py
  tests/ros/test_trajectory_node.py -q -p no:randomly`, run 2026-09-16):
  **1343 passed, 4 skipped, 2 xfailed in 485.04 s**.
* (`pytest tests/ros/test_trajectory_node.py
  tests/ros/test_unified_cycle_integration.py -q`, run 2026-09-16):
  **248 passed, 1 failed** —
  `test_the_velocity_term_passes_the_same_origin_re_installs`, the documented
  load-flake whose own docstring names this exact signature ("failed once
  inside a full `tests/ros/` run on 2026-09-06 and passed in isolation"). It
  passes 96/96 twice when its file is run alone (same date), and it exercises
  the EXTEND chain, which this change does not touch.
* Probe (`/tmp/claude-…/scratchpad/probe_rest2.py`, run 2026-09-16, venv): the
  1.5 s opening REST plans CLEAN from hand seeds 0.0001 / 0.5639 / 2.0 / 5.0 /
  8.0 / 9.9 rev, terminal 0.3071 rev in every case, peak hand rate 0.31 →
  9.76 rev/s.
* ⚠ **Knife-edge found and NOT fixed** (owner ruling wanted): a seed of
  *exactly* 0.0 rev is refused `HAND_STROKE` ("hand position −0.000 rev outside
  [0.000, 9.959]"), because `feasibility._cycle_stroke_floor` grants the
  `_HAND_DIVE_TOL_REV` dive tolerance only to a seed already BELOW the homed
  zero. 0.0001 and −0.002 both pass. No measured encoder value has been exactly
  0.0, and the seed now comes from the encoder rather than from the commanded
  0.0, so the exposure went DOWN with this change — but the asymmetry is real.

(2026-09-16, `./run_tests.sh --full`, log `temp/logs/park_outcome_full2_20260916.log`, the final tree of both same-day units plus the audit follow-ups): **PASS — parallel 6198 passed, 9 skipped, 2 xfailed in 295.88 s; serial 6 passed in 19.40 s.**

## Outcome

The 2026-09-16 failure mode cannot recur: there is no hand-position refusal
left to fire, and the seed the retirement relies on is the encoder rather than
a diagnostic echo. NOT yet flown — the next sitting is the first test, and it
should expect at most one `HAND SEED RECONCILED` line at the opening REST.
