---
title: Hand catch arm gated off a live throw stroke (C-HAND-1 host half) — and five review findings that reshaped what the bench will believe
type: bugfix
date: 2026-07-26
status: resolved
phase: "Self-toss anomaly fixes — hand-command-continuity Phases 1-2"
related_plan: "hand-command-continuity.md"
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/hand_stroke.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/docs/hand_command_continuity.md
  - tools/probes/hand_stroke_timeline.py
  - tools/probes/README.md
  - tests/motion/test_hand_stroke.py
  - tests/ros/test_catch_coordinator_node.py
  - tests/hardware/session_anomaly_fixes.md
  - plans/archived/hand-command-continuity.md
commits:
  - 6179a88
subsystem:
  - ros
  - motion
tags:
  - safety
  - testing
  - docs
---

# Hand catch arm gated off a live throw stroke (C-HAND-1 host half)

## Summary

The Platform Teensy holds **one** packed trajectory queue and rebuilds it from
scratch on any kind-0/1/2 command, seeding the replacement prelude from
`current_hand_position` with `v = 0` — the live velocity is available and never
read. During a self-toss the kind-1 catch arm was landing 8-18 ms after ball
release, inside the throw's 65 ms deceleration ramp, so the queue was cleared
while the hand was travelling at ~120 rev/s. This phase adds a stroke-busy window
derived from the toss's own announcement and enforces it at exactly one point, so
the arm is **deferred** to the first tick after the stroke completes rather than
dropped. It also lands the contract document (`ros_ws/docs/hand_command_continuity.md`)
and the shared host-side stroke model that the bench verdict probe and the
shipped window now both read.

Status is `in-progress`, not `resolved`: this is the host half of C-HAND-1, and
**nothing here is hardware-validated yet**. The firmware half (a prelude
continuous with the live velocity) is plan Phase 4.

## Problem

Measured 2026-07-25 across seven self-tosses in three sessions:

* the hand **overshot to 10.17-10.33 rev**, leaving as little as 0.775 rev
  (24.5 mm) of headroom against the 11.1 rev overextension guard;
* it was then **yanked 0.34-1.75 rev below the stroke end** (10.7-55.3 mm, up to
  20.5 % of the usable stroke) at −17.9 to −42.4 rev/s, recovering over ~300 ms;
* the throw's own deceleration ramp was discarded, replaced by the position
  loop's reaction to a frozen setpoint.

## Root Cause

`Teensy_code.ino:539` calls `packedMsgs.clear()` on any kind-0/1/2 command;
`Trajectory.h:242-301` (`makeSmoothMove`) then seeds the replacement prelude from
`current_hand_position` with `v = 0, a = 0`. `current_hand_velocity` is declared
`extern` at `Trajectory.h:47` and is never read.

That is harmless with the hand at rest, which is why it survived a year of reload
operation — a reload's catch arm lands with the hand parked at the top. It is not
harmless mid-stroke: the replacement is a rest-to-rest quintic computed from a
position the hand is *travelling through*.

The defect is not "the arm was late". It is that **a single-queue,
last-writer-wins actuator was being written to while it was moving**, and nothing
forbade it.

## Discussion

### Why a derived window rather than a fixed delay

`t_dec` spans 94.5 ms at the 0.55 s flight to 47.4 ms at 1.10 s — a 2× range.
Sized for the short end, a fixed delay wastes half the window at the long end;
sized for the long end, it lands back inside the ramp at the short end, where the
momentum is ~1.9× and the measured end-stop headroom is smallest. Both inputs the
derivation needs are already on the wire: `throw_time` is the kind-0 event
instant, which *is* ball release (`makeThrow`'s `shiftTime(-t2)` puts t = 0 at the
end of the velocity hold), and `|initial_velocity|/1000` *is* the commanded
`event_vel`. No new field, no new topic.

### Why the model is shared rather than duplicated-and-pinned

The probe `tools/probes/hand_stroke_timeline.py` is the Phase-5 **verdict**
instrument. If it and the shipped suppression window disagreed about `t_dec`, the
bench would score the fix against a different model than the one that shipped —
and a pin only catches the quantities it happens to assert. So the probe's local
`StrokeModel` was **moved** into `jugglebot/motion/trajectory/hand_stroke.py` and
imported back, rather than pinned against a second copy. The precedent is not
hypothetical: `JB_OP_HAND_CATCH_PRIME_REV` is a hand-maintained 9.858 against a
stroke top of 9.9594 rev, so "catch from rest at the top" has never actually
started from rest at the top. A second host copy of the *timing* would fail the
same way and much more quietly.

`sim/hand/trajectory.py` — the sim-side copy — was deliberately **not** touched.
It is plan Phase 4's territory, and Phase 0 recorded two catch-timeline
divergences there that must not be repaired in passing.

### Why the gate sits inside `_arm_hand_catch`, not at its call site

A second caller added later (a re-arm timer, a recovery path) would bypass a
call-site check silently, and the defect it re-introduces is invisible in ROS
because the Teensy prints nothing when it clears its queue. Putting the predicate
at the dispatch also makes Phase 2 free: the ack-failure retry re-enters the same
gate, so "further repacks just churn" — which was an *assumption* in that
callback's docstring, true for a reload and false for a toss — becomes an
enforced precondition.

### The fork that mattered: what to do when the window would close

`Teensy_code.ino:533` refuses the whole command when it will not fit and prints to
serial **only** (`:534`). So an arm deferred past that point is not a late catch —
it is a silently missing one, with no ROS-visible signal and the ball on the
floor. The gate therefore logs loudly and **dispatches immediately** rather than
deferring. A dip is ugly and recoverable; a silently-lost catch is neither.

One reviewer proposed hardening this branch: budget the worst-case *mid-stroke*
prelude and drop the arm with an explicit ERROR when even that will not fit.
**Rejected**, on the asymmetry: a drop guarantees no catch, whereas a dispatch is
refused only if the Teensy's own clock agrees it will not fit. Reading the
firmware settled it — `:533`'s `return` sits **before** `packedMsgs.clear()` at
`:539`, so a refused command leaves the live throw stroke **intact**. The forced
branch's worst case is a lost catch, never a clobbered stroke, and it reproduces
the pre-fix arithmetic exactly rather than trading it for something worse.

### The kind-3 exemption is deliberate and must stay

A kind-3 replacing whatever is queued is the **only un-arm mechanism the Teensy
offers**, and a pre-release `SAFE_ABORT`'s retract depends on it clobbering an
armed kind-0. Gating it would leave the abort path unable to stop a queued throw.
The toss's own prime-during-stroke hazard is owned by a separate, already-enforced
gate (`catch/prime_hold`, raised for the whole PREPARE→terminal span). Pinned by
test.

### Two Phase-0 hedges that did not survive re-derivation

Both were optimistic, and both are corrected in place in the plan:

1. **"Arming inside the window makes the prelude exactly empty"** is an
   idealisation. `makeSmoothMove`'s dead-band is 1e-6 rev = 3.16e-5 mm — 0.03
   microns, unreachable against a live float encoder — and `Trajectory.h:260`
   floors every non-empty duration at 0.05 s. The post-fix prelude is a 50-76 ms
   micro-move, never nothing. A fit check assuming zero would hand the Teensy a
   command it refuses at `:533`, which is the exact silent-no-catch failure this
   phase exists to remove.
2. **The Phase-0 window table omitted `_MIN_EVENT_DELAY_S = 0.3`**, which the
   caller uses to drop the arm outright and which binds *ahead* of the Teensy
   budget at every nominal velocity. With both carried, the real windows are
   **395 ms** at 0.80 s and **115 ms** at 0.55 s, not 546/208, and the closure
   velocity is 1.26 m/s rather than 1.02.

### Review adjudication — what the three lenses changed

Twelve findings; nine verified-and-fixed, three verified-but-deferred. The five
that changed what ships are recorded in the plan's Outcome. Two are worth
restating here because each is a way a bench session would have drawn the wrong
conclusion:

* **`catch/vel_scale` closes the window on its own.** The docstrings blamed only
  a 40 % tracker under-read. Swept against the production velocities, a scale of
  **0.45 closes the window at the 0.55-0.56 s flight** (−15 ms; 0.50 gives
  +18 ms, the 0.8 default +116 ms) with a perfectly healthy tracker, inside the
  shipped `[0.3, 1.5]` range. That corner is exactly the optional short-flight
  check, so an operator running it with a reduced knob would have seen every toss
  take the closed branch and routed a knob setting to a timing defect.
* **The flight-time error is probably not this phase's to fix.** The pre-fix
  0.887/1.091 s against a commanded 0.800 s had been attributed to the truncated
  decel ramp "setting the release conditions". But by design the ball separates
  at the decel **onset** (`x2`), and all seven measured truncations sit *past* the
  commanded `x2` crossing (6.1965-7.7825 rev against `x2` = 5.9138 rev) — so the
  ball had most likely already left the cup before the queue was cleared. A null
  result on flight time is expected and is **not** a Phase-1 failure. Without
  that correction the next session would have re-opened the queue-clobber
  question instead of the release-model question that `single-ball-toss.md` Phase
  5 T0 actually owns.

Deferred with reason rather than fixed: the deferral's **tick-dependency** (the
gate is reached only from `_on_balls` and nothing re-enters it on a timer, so a
track dropout spanning the whole remaining window, or a landing revision pushing
`event_delay` under the 0.3 s floor, bypasses the gate entirely and even the
closure branch cannot fire). Both reviewers reached it independently and both
scored reachability NOT-PROVEN; the node's own probed note argues against it
(announced-vs-tracked landing agreeing to 0.000 s at the arm moment, n = 6105, in
early life — which is exactly the suppression window). Making the deferral
self-driving with a one-shot timer is a behaviour change on the strength of an
unproven trigger, so it is **instrumented instead**: new runbook row H1.7 counts
withheld lines with no matching dispatch, and a non-zero count is the signal to
build the timer.

Also deferred: instrumenting the measured hand velocity at the window's opening.
The premise that the hand is at rest there is exact for the *commanded* profile
and assumed-within-tolerance for the *measured* one, with 16.6 ms of settle
allowance at the worst measured dispatch shift and no post-fix settle measured
anywhere. A velocity column in the probe would instrument it directly — but the
probe's gate fixture is byte-compared, and this phase's evidence that moving the
model did not perturb the instrument rests on that byte-identity. H1.1's
`dip_below_x3 <= 0.100 rev` already measures the harm the premise protects
against: a hand still moving at the window's opening seeds the quintic from a
position it is travelling through, which is precisely what that row reads.

### Why the contract got a document

C-HAND-1's normative statement lived only in the plan and a source comment. Plans
get archived; the sibling contract landed in the same run
(`ros_ws/docs/levelling_frame.md`, C-LEVEL-1) set the opposite precedent the same
week. The repo's own rule is that a contract is three parts — a normative
document, one canonical enforcement point, a test that fails on violation — and
parts two and three were already here. The concrete cost of skipping part one:
the next person asked to "just gate the kind-3 too" or "just relax the window for
this one flight" would have had no document to be pointed at, only a comment
inside the file they were editing.

## Fix

* **`ros_ws/src/jugglebot/jugglebot/motion/trajectory/hand_stroke.py`** (new) —
  THE host-side copy of `Trajectory.h`'s closed form. `HandStrokeModel`,
  `throw_decel_s`, `catch_lead_s`, `smooth_move_duration_s`, `stroke_clear_time`,
  `required_arm_lead_s`, plus the policy constants `ARM_SUPPRESS_MARGIN_S = 0.040`
  and `HAND_SETTLE_BAND_REV = 0.10`. Constants come from `jugglebot.hardware_config`,
  generated from the same YAML that generates the firmware header.
* **`catch_coordinator_node`** — `_latch_throw_stroke_window` (from our own
  announcement only, keyed on `thrower_name`; a BB reload leaves it inert) and
  `_throw_stroke_gate_ok`, consulted from `_arm_hand_catch`. Withholding leaves
  the one-shot latch open and the dispatch counter untouched, so the arm is
  deferred, never dropped.
* **`ros_ws/docs/hand_command_continuity.md`** (new) — the normative contract, its
  two obligations, the enforcement point, the deliberate exemptions, and the four
  known limits of the host half.
* **The margin is 40 ms** — 1.7× the worst measured announcement-to-release shift
  (+12.8 to +23.4 ms, which nothing compensates because
  `JB_OP_TOSS_RELEASE_LATENCY_MS` ships 0.0). Zero margin expires mid-ramp and
  reproduces the defect exactly.

## Verification

**Full suite** (`pytest tests/ -q`, run 2026-07-26 on the Jetson under
`~/Desktop/PDJ_venv/venv`): **3517 passed, 3 xfailed in 1351.82 s (22:31)**. Baseline at
HEAD `2395244` was 3484 passed, 3 xfailed in 1371.51 s — **+33 passed**, exactly
the 33 cases this phase adds (15 in `tests/motion/test_hand_stroke.py`, 18 in
`tests/ros/test_catch_coordinator_node.py`). The **xfail count is unchanged at
3**: no test was weakened, skipped, xfailed or deleted.

**Mutation-verified** rather than assumed (each mutation applied, suite run,
mutation reverted):

| mutation | tests that fail |
|---|---|
| delete the one-line gate call in `_arm_hand_catch` | 8 |
| `ARM_SUPPRESS_MARGIN_S = 0.040 → 0.0` | 9 |
| remove the `thrower_name` discriminator | 14 |
| delete the arm-edge window clear | 1 — **after** this session extended the test; it was **0** before |

That last row is a finding, not a flourish: `test_stroke_window_cleared_on_both_latch_edges`
as first written exercised only the disarm edge (nothing is latched by the time it
re-arms, so the closing assertion passed on the disarm clear alone). The test now
drives both edges independently and the mutation fails it.

**Instrument re-verified after the model move**:
`python tools/probes/hand_stroke_timeline.py --gate` → `GATE PASS — 25/25 rows`
(the truncated capture it must FLAG) **and** `GATE PASS — fixed-shape branch`
(four clean shapes it must ACCEPT: clean, overshoot, short-flight,
braking-prelude), exit 0. `--emit-gate-fixture` regenerates the committed fixture
byte-identically.

## Outcome

Host-side only. **Deployment: `colcon build --packages-select jugglebot` +
relaunch** — the launch runs the *installed* copy, so a relaunch alone keeps the
old code. No codegen run (no YAML changed) and **no firmware flash**.

**Not yet validated on hardware.** The bench checks are
`tests/hardware/session_anomaly_fixes.md` § Section HAND, rows H1.1-H1.7 (+
optional HAND-1b) and H2.1-H2.4, all scored off a single capture. Stated plainly
there and worth repeating: mocked-ROS tests prove the arm is not *dispatched*
during the stroke, but they cannot see the Teensy's queue semantics, which is
where the failure lives — H1.1's `trunc`/`seeds` rows are the only evidence that
no queue was cleared.

## Follow-ups

* **Phase 4 (firmware)** closes the class: a prelude continuous with the live
  velocity. Until then, every other command that can land mid-motion — a prime, a
  retract ladder rung, a `SAFE_ABORT` — still re-preludes from `v = 0`.
* **An armed stroke produces no observable until its event time**, so a kind-1 arm
  cannot be telemetry-verified the way the hand ladders were (`4e33b53`) — which
  is why the retry path exists at all, against a 40-60 % `ERR_TIMEOUT` ack failure
  rate. A Teensy-side "armed stroke" field in `hand_telemetry` or `link_status`
  would make it verifiable. Protocol change, out of scope.
* **`Teensy_code.ino:533`'s refusal is serial-only** and invisible to ROS. Phase 1
  predicts it host-side and avoids provoking it; surfacing it over the wire (a
  nack code on the `SetHandTrajCmd` response, or a `link_status` counter) would
  close the observability gap. Runbook row H1.6 reads the serial console as the
  interim.
* **`JB_OP_TOSS_RELEASE_LATENCY_MS` ships 0.0** against a measured +12.8 to
  +23.4 ms shift. This phase *covers* that latency with a margin rather than
  compensating it. Compensating it is `single-ball-toss.md` Phase 5 T0's
  measurand.
* **Unrelated latent defect, untouched and reported as required:**
  `Teensy_code.ino:472-475` returns *before* `packedMsgs.clear()` at `:479` when
  `makeSmoothMove` comes back empty, so a kind-3 retract whose target is within
  1e-6 rev of the live position would not clear an armed stroke. Latent (1e-6 rev
  = 0.03 microns) and it is the only un-arm mechanism the Teensy offers, so moving
  that clear is an operator decision.
* **Operator decision still open from Phase 0:** whether kind-1 should centre the
  catch velocity hold on the predicted arrival. `calcCatch`'s
  `t5 = t2 + airT − 0.5·t_vel` makes the *arrival* the hold's centre while
  `sim/hand/trajectory.py` centres its own t = 0 on the hold — a 4.9-9.7 ms /
  0.498 rev (15.75 mm) divergence, and a candidate contributor to the known
  sim-catch fidelity gap.
## Close-out — 2026-08-21

**Status `in-progress` → `resolved`.** The entry was left `in-progress` because
its own Outcome said "DEFERRED TO THE OPERATOR — this phase is NOT validated
until it runs". It ran, on 2026-07-27, and it passed:

* `H1.2`–`H1.7` PASS — **17 latched = 17 tosses, 17 withheld, 0 CLOSED**, min
  slack **0.124 s** = 2.5× the floor;
* `H2.1`–`H2.3` PASS;
* the headline mechanism is *verified, not inferred*: on **all 17** tosses
  `pos_cmd` reached `x3` and held it before any new command landed, with
  commanded velocity never negative and commanded position never below `x3`
  between release and the arm.

Verdicts: `logbook/2026-07-28-anomaly-fixes-validation-sitting.md`.

**Two later corrections that touch this entry's numbers**, recorded rather than
edited in:

1. **The arm window at the band floor is 50 ms, not 115 ms.** Contract C-HAND-3
   (2026-08-18, `ros_ws/docs/hand_throw_envelope.md`) replaced the hand-picked
   `FLIGHT_TIME_MIN_S = 0.55` with a DERIVED **0.4949 s** floor, defined as the
   flight at which this very window reaches `arm_window_margin_s = 0.050 s`. So
   the floor window is now an identity, not incidental slack, and runbook row
   **H1.5** (`slack > 0.050 s`) is a boundary rather than a margin. The window
   closure velocity moves with it: `v_armed < 1.5907` m/s, not 1.26.
2. **The end-stop anchor.** Every "0.775 rev of headroom" figure here is against
   the *declared* 11.1 rev stop; against the measured 10.8 rev it is 0.475 rev
   (`logbook/2026-08-18-hand-end-stop-corrected.md`).

The `Teensy_code.ino:472-475` latent defect noted above is unchanged and still
an operator decision; it lives at `Teensy_code_platform.ino:581-583` (clear at
`:588`) in the current tree.
