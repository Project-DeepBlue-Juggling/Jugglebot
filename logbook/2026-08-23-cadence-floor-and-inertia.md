---
title: The accept-time delay floor learns the sequence it is measured against, and the B1 positioning skip learns to read an orientation
type: bugfix
date: 2026-08-23
status: resolved
files_changed:
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/toss_session.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/levelling.py
  - ros_ws/src/jugglebot_interfaces/action/TossContinuous.action
  - tools/probes/cadence_rung_check.py
  - tools/probes/toss_record_miner.py
  - tools/probes/ilc_speed_band.py
  - tools/probes/README.md
  - tools/toss_cal_analyse.py
  - tests/hardware/ilc_fit.py
  - tests/hardware/toss_cal_fit.py
  - tests/hardware/session_cadence_ladder.md
  - tests/motion/test_cadence_rung_check.py
  - tests/motion/test_levelling.py
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_toss_session.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_trajectory_node.py
  - plans/active/critical-point-ilc.md
subsystem:
  - ros
  - motion
  - plans
tags:
  - toss
  - cadence
  - ilc
  - contracts
  - levelling
related_plan: critical-point-ilc.md
---

# The accept-time delay floor learns the sequence it is measured against, and the B1 positioning skip learns to read an orientation

## Summary

Follow-up package **cadence-floor** of the 2026-08-23 arc, closing the two open
questions the ILC-primary fold-in's Phase-L audit carried out
(`logbook/2026-08-21-ilc-primary-foldin.md` § "Open questions carried out of
Phase L", items 1 and 2). Operator-approved 2026-08-23.

Both accept-time `throw_delay` gates charged the kind-0 dispatch budget alone,
while the runtime guard applies that same budget to the lead REMAINING after the
whole pre-dispatch sequence — so a goal could clear accept by construction and
then abort `ABORTED_CANT_MAKE_RELEASE` on every cycle, with the catch armed and
the hand retracting under a seated ball. Both gates now import ONE derivation
that charges the dispatch budget **plus** the sequence, computed at the slowest
release layer 3's apply seam could command; a committed probe asserts
accept-implies-flies over the whole `(T, dwell, delay, aim)` grid, and it is
pinned as a test. Separately, `trajectory/commanded_pose` publishes the commanded
ORIENTATION in the intent frame, so the census-B1 positioning skip can verify a
pre-tilt pose and fire on an aimed chain — worth 0.38 s of throw delay on every
cycle of every ILC sitting, and the aimed cadence frontier moves **40.4 → 54.3
throws/min**. Operator decision 3 is re-taken and accepted at R5-prime
`dwell 0.66 / delay 0.44`, 51.6 throws/min with or without an armed aim.

## Symptom

The 2026-08-22 audit found that three published rungs of
`tests/hardware/session_cadence_ladder.md` — R4, R5 and R5-prime — could not
throw a ball. Each cycle died `ABORTED_CANT_MAKE_RELEASE` at
`cycle_start + 0.06 s`, which is *after* the catch latch is up, the announcement
is out and the hand is committed, so the cleanup retracts the hand under a seated
ball. The audit republished slower rungs and carried two items as open:

1. **BLOCKING** — the accept-time `throw_delay` floor did not model the sequence
   it is measured against.
2. the census-B1 positioning skip could not fire on an aimed chain, costing
   0.38 s of throw delay on every cycle of every ILC sitting.

The ladder's aimed frontier was **40.4 throws/min** against a level chain's 54.3,
and the operator's 2026-08-21 cadence decision (61 throws/min) was unreachable in
either.

## Diagnosis

**One arithmetic error, in two places, both of which called themselves a mirror
of the other.**

`toss_sequencer._step_preparing` guards the release window with

```python
if self._t_release - now < self.min_event_delay_for_throw_s:
    return self._abort('CANT_MAKE_RELEASE')
```

`min_event_delay_for_throw_s` is `hand_stroke.min_throw_event_delay_s(v)` — the
Teensy's own `:642` budget for the kind-0 dispatch, prelude + gap + windup. That
is the correct budget for the dispatch, and the guard applies it to the lead
**remaining** after the whole pre-dispatch sequence has already run.

Both accept-time gates charged that same number against the lead **at accept**:

* `toss_sequencer._step_checking`'s `REJECTED_CANT_MAKE_LEAD`, and
* `toss_session.min_throw_delay_s`, its session-scope mirror.

So both were systematically loose by the entire pre-dispatch cost. That cost is
not a mystery — it is the node's own tick ladder, and it is exactly determined:

| | |
|---|---|
| tick 0 | CHECKING passes → `ACTION_POSITION_PLATFORM`; the node answers SYNCHRONOUSLY inside the tick |
| first tick at/after arrival | `_step_positioning` → `_enter_preparing` → `ACTION_PREPARE_CATCH` |
| +1 tick | the node's DEFERRED PREPARE bundle answers |
| +1 tick | release-window guard, then `ACTION_ANNOUNCE` |
| +1 tick | release-window guard, then DISPATCH |

`arrival` is 0 when POSITIONING takes the census-B1 no-op skip and
`min_move_duration_s + TOSS_POSITION_SETTLE_PAD_S` = 0.40 s when it commands a
move. At the shipped 0.02 s tick that is **4 ticks = 0.080 s** with the skip and
**23 ticks = 0.460 s** without it.

**And the second half of the BLOCKING finding: the session was computing its
floors from the wrong speed.** `toss_session` used
`vertical_event_vel_mps(T)` while the cycle FSM receives `v · (1 + ilc_vel_trim)`
from layer 3. Every derived floor in that class *rises as the release slows* (a
slower throw has a longer windup), so a NEGATIVE speed trim raised the cycle's
floor **after** the session had already accepted the goal — a
`REJECTED_CANT_MAKE_LEAD` on cycle 1 of a session the operator was told was
legal. The measured corpus asks for `event_vel_trim = −0.1076`, so this is not
hypothetical.

**Why the B1 skip refused every tilted release.** `_toss_already_positioned` had
three conditions and the first was *"the release must be LEVEL"*:
`trajectory/commanded_position` publishes `_current_state()[0][:3]`, three of the
six pose components, so a pre-tilt pose carried an orientation the coordinator
could not verify and a wrong "yes" would fire the throw from a pose the aim was
not solved for. Correct, and expensive: an armed aim — ILC layer 3, the
calibration map, Tier 8b — makes every release tilted, so the skip was off for
the whole of every ILC sitting and each cycle paid 0.40 s of positioning to
traverse zero millimetres and re-command a tilt it was already holding.

## Discussion

### The framing that had to be abandoned: "the session mirrors the cycle"

`toss_session.min_throw_delay_s`'s docstring said it was
*"the cycle FSM's own delay floor, restated at session scope … so a session can
never be looser than the cycle it repeats nor stricter than it"*. That framing is
what made the fix look like a one-line change, and it does not survive the
pre-dispatch term.

The reason: **the pre-dispatch cost is a property of a CYCLE, not of a SESSION,
and the two cycles of a sitting that matter have different ones.** The first
cycle commands the positioning move (the platform is not yet at the pre-tilt
pose) and pays 0.460 s. Every chained cycle takes the B1 skip — a CAUGHT toss
ends in `ACTION_STAY` and the emitter's terminal hold leaves the platform exactly
where it threw and caught from, and the next cycle recomputes a bit-identical
pre-tilt from the identical `(catch_pose, flight, aim)` — and pays 0.080 s. There
is no single number for the session to mirror.

Three ways out, and the choice between them is the whole design:

**(a) charge the moving budget everywhere.** Fail-closed, one number, no new
plumbing. It also forbids every cadence above ~40 throws/min for a cost that only
the first cycle of a sitting ever pays. Rejected: it would have made the fix
*cost* the operator the entire cadence gain of the same day's other change.

**(b) charge the chained budget everywhere, and let the first cycle abort.**
Cheapest arithmetic, and it re-opens the exact hole being closed: a goal that
passes accept dies mid-sequence, just on cycle 1 instead of on all of them.

**(c) charge the REAL per-cycle predicate, and pay for the first cycle
explicitly.** What shipped. The cycle's gate charges what that cycle will
actually spend, keyed on a boolean the node evaluates once at cycle build; the
session's gate charges the chained steady state, because that is what
`throw_delay_s` MEANS at session scope (`required_dwell_s` is
`throw_delay + handoff_margin`, and a cadence is a steady state); and the node
grants the first cycle the extra lead it needs, once, with one WARN line naming
the raise.

### Why granting lead is not "silently stretching a cadence"

The doctrine in `toss_session` is explicit: *"a cadence the machine quietly
ignores is a lie about what it did"*, and `REJECTED_DWELL` exists rather than a
silent stretch. Granting a cycle more `throw_delay` looks like a violation of
that, and it is not, for a reason worth writing down:

**`throw_delay_s` is not a cadence. It is plumbing.** The session schedules
`cycle_start(N+1) = landing(N) + dwell − throw_delay`, precisely so the RELEASE
lands one dwell past the landing. The delay exists to give a cycle room to run
CHECKING → POSITIONING → PREPARE before its release; the operator's cadence
parameter is `dwell_time_s`, and the grant does not touch it. Cycle 1 releases
~0.38 s later than the metronome implies, the schedule re-anchors on cycle 1's
real landing, and every subsequent period is exactly what was asked for.

The grant is also deliberately **not** applied to a single `Toss`. There the
delay is an appointment the operator set for a one-shot event, nothing consumes
it as a schedule, and stretching it answers a request nobody made. That path
keeps its loud `REJECTED_CANT_MAKE_LEAD`. One flag,
`_build_toss_cycle(delay_is_cadence=...)`, and the docstring says which caller
means which.

### Why the orientation is published in the INTENT frame, and why that is a contract question

The obvious implementation — publish `_current_state()[0]` as a `Pose` — is
wrong, and wrong in a way that would have been invisible until an ILC sitting
mis-aimed.

`_current_state()` samples the ACTIVE PLAN, and the plan was built to the
C-LEVEL-1 **corrected** target: `R_gravity @ R_request`. The consumer
(`_toss_already_positioned`) is asking *"is the platform already at the pose I am
about to REQUEST?"* — a question in the INTENT frame, because its request will be
corrected on the way in. Comparing a corrected rotation against an intent one is
a frame error of exactly the magnitude of the levelling offset (0.78° measured =
13.6 mrad), which is 5.0× the angular tolerance and would have made the skip
refuse forever — a silent cadence loss, not a visible fault.

The consumer's two ways out were: re-derive the correction on its own side, or be
handed the intent frame. The first is precisely the cross-node duplication
`motion/levelling.py` exists to delete — its module docstring names the
non-commutative composition and the 1.268e-3 rad silent error a re-derived copy
produces. So the inversion happens **once, in the node that owns the correction**,
through a new `levelling.uncorrect_pose` that is the exact transpose inverse of
`correct_pose`. Position is frame-invariant here (`correct_pose` never touches
it), so the new topic's position half is bit-identical to the existing `Point`.

### Why a new topic rather than changing the existing one

`trajectory/commanded_position` keeps its name, its type and every consumer. A
type change on a live ROS 2 topic is **silent**: a mismatched subscriber simply
never connects. That read sites the displaced throw (`throw_site_known`), and its
failure mode would be `REJECTED_POSE_UNKNOWN` on every 8b goal until somebody
noticed. The new `trajectory/commanded_pose` is additive, published from the same
sample in the same statement under the same `streaming` guard, and read by
exactly one thing.

**One message, not two topics**, because the B1 decision needs position and
orientation from the SAME plan sample. Two topics would let a mid-move platform
match a target on a stale half — at 5 Hz against a 0.4 s move that is a real
window, and the wrong "yes" fires a throw from a pose nobody solved for.

### Why the check got STRICTER, not looser

Retiring the "release must be LEVEL" condition reads like a relaxation. It is the
opposite: a tilted release must now **prove** the platform holds its tilt, where
before it could not be asked. And the LEVEL branch tightened too — it used to
skip on position alone, so a platform holding a residual tilt from some other
commander (a reload catch's pre-tilt at the same position) would have passed.

The angular tolerance is derived, not chosen: a residual tilt δθ tilts the
release velocity by δθ and displaces the landing by `4·h·sin δθ` — the identical
expression the CHECKING levelling gate is sized on. Setting that equal to the
position tolerance (`GEOM_HAND_RADIUS_MM / 2` = 17.5 mm) at the tallest admitted
throw gives **2.71 mrad (0.155°)**, which is 1/6.4 of the ILC's ±1.0° aim
authority — so an armed aim can never be mistaken for a level platform, which is
the wrong "yes" that would fly an ILC throw with the correction not applied.

### The tradeoff accepted on the ILC speed trim, and the one deliberately not taken

The session cannot know the goal's actual ILC trim at accept time without calling
`_toss_aim_for_goal`, and that is **the** single per-goal aim lookup —
`tests/motion/test_toss_cal.py` pins its call site structurally, it seeds the
goal-local common mode, and it owns a once-per-goal WARN dedup. Calling it twice
was not worth the blast radius.

So the session charges the **slowest release the apply seam could command**:
`_ilc_vel_trim_refusal` refuses any trim whose speed breaks
`throw_envelope.evaluate`, so the bound is the smallest `v · (1 + t)`,
`t ∈ [−0.15, 0]`, that the envelope admits — bisected, because the envelope's
negative-side bound is strongly flight-dependent and has no closed form. It costs
**exactly nothing at the band floor** (the envelope refuses the negative side
there outright, which is where the frontier lives) and ~1.5 throws/min away from
it, and only when an artifact is actually loaded.

The tighter alternative — hoist `_toss_aim_for_goal` to per-GOAL, which is what
its own docstring already claims it is, and pass the block down — is real and is
carried as an open question rather than done here.

### The one floor deliberately left on the untrimmed speed

`hand_floor_dwell_s` stays at `vertical_event_vel_mps(T)` while the two floors
either side of it moved to the fail-closed speed. It is THE named C-HAND-1
geometry number — the census, the runbook's § 0 table and
`ros_ws/docs/hand_decel_feedforward.md` all quote it per flight time — and
re-basing it onto a speed that depends on whether an artifact happens to be
loaded would make "the hand floor at T = 0.5029 s" two different numbers on one
machine.

That is safe only because it is **not the binding term at any admitted flight**:
since the delay floor grew the pre-dispatch sequence, `throw_delay +
handoff_margin` exceeds it by **0.1230 s at its worst** across the whole band,
which covers this term's entire **0.0715 s** worst-case trim sensitivity with
1.7× margin. Left as an argument that could rot, it would rot; so
`test_the_hand_floor_is_dominated_by_the_plumbing_term` pins the dominance and
its minimum margin, and reds if the plumbing term ever shrinks back under it.

### A knife edge the model exposed, and what it is honest to do about it

With the floor set to exactly `dispatch + sequence`, the runtime guard's
inequality becomes `(dispatch + budget) − budget < dispatch` — an identity in
real arithmetic and a coin flip in binary floating point, because `(a + b) − b`
is not `a`. Driven at the exact floor against the tree, R5 passed and R4 aborted,
for no reason but the bit pattern.

The tempting fix is a tick of jitter allowance. That would be dishonest: bounding
scheduling jitter is the RUNTIME guard's job and no static floor can do it (the
node's `time.sleep(_TICK_S)` is a lower bound, and the cadence runbook aborts a
sitting on one `ABORTED_CANT_MAKE_RELEASE` precisely because it is real signal).
What shipped is `FLOOR_REPRESENTATION_SLACK_S = 1e-6` — named for what it is,
1/20000 of a tick, invisible in every published number — so the floor is strictly
sufficient rather than exactly-equal-and-lucky.

## Fix

**One derivation, imported by both gates** (`toss_sequencer`):

* `pre_dispatch_budget_s(positioning_move)` — the tick ladder above, in closed
  form, with the `ceil` epsilon that stops `0.40/0.02 = 20.000000000000004` from
  charging a 21st tick;
* `min_throw_delay_for_release_s(v, positioning_move, …)` —
  `max(debounce, dispatch budget + pre-dispatch budget + representation slack)`;
* `NODE_TICK_S` moved here from `toss_session` (which re-exports it) because it
  stopped being a latency note and became arithmetic.

**The cycle gate**: `TossSequencer.positioning_move_expected` (default **True**,
fail-closed) feeds `min_throw_delay_for_cycle_s`, which `_step_checking` refuses
against. The reject message now names the dispatch budget, its three terms, the
sequence cost and which sequence it charged.

**The session gate**: `min_throw_delay_s` charges the chained steady state
through the same function, at `floor_event_vel_mps` — the new fail-closed speed
property. `handoff_margin_s` moved to that speed too (the catch is armed for the
ball the throw put up); `hand_floor_dwell_s` deliberately did not.

**The node**: `_build_toss_cycle` evaluates the B1 predicate ONCE, caches it as
`_toss_positioning_move`, feeds it to the FSM as the budget input, and
`_position_platform_for_toss` consumes the cached answer — so the branch taken and
the budget charged cannot disagree. `delay_is_cadence=True` (the session path
only) grants a moving cycle the lead it needs, never touching a 0.0 sentinel or a
negative sign-typo.

**The orientation surface**: `levelling.uncorrect_pose`;
`trajectory_node._intent_orientation` + the `trajectory/commanded_pose` publisher;
the coordinator's `_on_commanded_pose` / `_live_commanded_pose` and the
orientation-aware `_toss_already_positioned` with the derived
`_TOSS_ALREADY_THERE_TOL_RAD`.

**The probe** (`tools/probes/cadence_rung_check.py`): one pair per rung again,
checked four ways (session accept; chained cycle; first cycle at the granted
delay; each with the ILC trim possible and not), a rewritten `--frontier` keyed
on the session floors, and a new `--grid` that sweeps the whole
`(T, dwell, delay, aim)` space for accept-implies-flies violations. Pinned by
`tests/motion/test_cadence_rung_check.py`, which imports it.

**The republished ladder**: the LEVEL/AIMED split is gone; R5-prime is
`dwell 0.66 / delay 0.44`, period 1.163 s, **51.6 throws/min with or without an
armed aim** (was 53.0 level / 39.7 aimed). A new § 2.9 gives the full
`ros2 action send_goal` template, every field's default-when-omitted, its
validated range, the code that refuses it, and one worked example per rung.

**Small items**: `argparse` parsers that passed a multi-line `__doc__` without
`RawDescriptionHelpFormatter` (or truncated it to one line) now render the whole
module docstring — `cadence_rung_check`, `toss_record_miner`, `ilc_speed_band`,
`ilc_fit`, `toss_cal_fit`, `toss_cal_analyse`.

## Outcome

| | before (2026-08-22) | after (2026-08-23) |
|---|---|---|
| frontier, aim disarmed | 54.3 throws/min | 54.3 throws/min |
| frontier, ILC armed | **40.4** throws/min | **54.3** throws/min |
| published operating point | 53.0 level / 39.7 aimed | **51.6**, either way |
| a goal that passes accept can abort mid-sequence | yes | **no** (grid-asserted) |

The accept-floor redesign bought no cadence and was never going to. The 34 %
aimed improvement is entirely the B1 skip becoming reachable on a tilted release.
What the redesign bought is that the failure it was chasing is now a pre-throw
refusal with nothing armed — run against the pre-audit ladder, not one of those
rungs reaches `ABORTED_CANT_MAKE_RELEASE` any more.

## Verification

- `./run_tests.sh --full` (every tier, `nightly` included), run 2026-08-23 as the
  package-closure gate: **RESULT: PASS** — parallel **6144 passed, 3 skipped,
  3 xfailed in 516.44 s**, serial **9 passed in 41.46 s**, total 564 s. Re-run
  immediately before the commit, after this entry itself was written:
  **PASS again, identical counts** (parallel 6144/3/3 in 522.55 s, serial 9 in
  41.64 s, total 570 s). Both runs quoted rather than one, because the entry is
  inside the test surface — `tests/sim/test_logbook_search.py` parses the real
  `logbook/` directory — and "the docs edit cannot have changed anything" is the
  claim `logbook/README.md` § "What the logbook tests actually check" exists to
  refuse. Traced as well as run: this entry loads through
  `logbook_search.load_entries` with `title` / `type` / `date` / `status` /
  `related_plan` / `subsystem` / `tags` all populated and its `Summary` / `Fix` /
  `Outcome` sections all extracted (checked 2026-08-23), which is precisely what
  that loader silently drops an entry for lacking.
- Collected-test delta, measured against a detached `HEAD` worktree at 24c7551
  with `pytest tests/ -q --collect-only`, both run 2026-08-23: **6141 → 6159,
  +18** (net of two tests this package replaced — `test_toss_session.py`'s
  crossover test became the dominance test, and `test_toss_coordinator.py`'s
  "a tilted release ALWAYS commands the move" became the skip test plus the
  tolerance-derivation test).
- Ladder-rung probe, run 2026-08-23 against this tree
  (`python tools/probes/cadence_rung_check.py --grid`): `PUBLISHED LADDER: all
  rungs FLY` (7 rungs × {ILC off, ILC on} × {chained, first} = 28 checks) and
  **0 violation(s)** over the accept-implies-flies grid; exit code 0.
- Frontier sweep, run 2026-08-23 (`--frontier`): monotone across the whole
  C-HAND-3 band, fastest at the floor — **1.1050 s = 54.3 throws/min at
  T 0.4949 (apex 0.300 m), throw_delay 0.4168, dwell 0.6101**, and the aim-armed
  column reports the identical numbers there.
- Per-rung clearances against the ILC-loaded (binding) floors, probe 2026-08-23:
  R4 delay +38.7 ms / dwell +13.9 ms; R5 +42.0 / +21.9; R5-prime +12.0 / +11.9.
- Mutation check, run 2026-08-23 — each of the four tests in
  `tests/motion/test_cadence_rung_check.py` verified RED against the pre-fix
  arithmetic (`min_throw_delay_for_release_s` reverted to
  `max(debounce, dispatch_s)`): **4 failed**, restored, **4 passed**.
- New tests, all landed this package:
  `tests/motion/test_cadence_rung_check.py` (4);
  `tests/ros/test_toss_sequencer.py::test_the_pre_dispatch_budget_is_the_node_tick_ladder_not_a_literal`,
  `…::test_the_planner_min_move_floor_matches_the_generated_config`;
  `tests/ros/test_toss_session.py::test_the_hand_floor_is_dominated_by_the_plumbing_term`,
  `…::test_an_armed_ilc_raises_every_derived_floor_but_the_hand_geometry`;
  `tests/ros/test_toss_coordinator.py::test_the_positioning_decision_is_taken_once_and_reused`,
  `…::test_a_cycle_that_must_move_charges_the_moving_budget_at_CHECKING`,
  `…::test_a_session_cycle_that_must_move_is_GRANTED_the_lead_a_single_toss_is_refused`,
  `…::test_the_lead_grant_never_launders_an_unset_or_negative_delay` (2 params),
  `…::test_the_angular_tolerance_is_derived_from_the_position_one`, and the
  rewritten `…::test_a_tilted_release_skips_only_when_the_platform_HOLDS_that_tilt`;
  `tests/ros/test_trajectory_node.py::test_commanded_pose_carries_the_same_sample_with_its_orientation`,
  `…::test_commanded_pose_orientation_is_the_INTENT_frame_not_the_corrected_one`;
  `tests/motion/test_levelling.py::test_uncorrect_pose_inverts_correct_pose_exactly`,
  `…::test_uncorrect_pose_leaves_position_untouched_and_does_not_mutate`,
  `…::test_uncorrect_pose_is_the_transpose_not_the_negated_offset`.
- `--help` re-rendered for all six swept CLIs, 2026-08-23: each now prints its
  full module docstring with line breaks preserved.

### Open questions carried out of this package

1. **The session's ILC speed bound is the envelope frontier, not the goal's
   actual trim.** Tighter would be to hoist `_toss_aim_for_goal` to per-GOAL —
   which is what its own docstring already claims it is; it currently runs per
   CYCLE — and pass the block into `_build_toss_cycle`. Worth ~1.5 throws/min at
   flights away from the band floor. `needs-design`: the aim path is the most
   test-pinned surface of this arc and `tests/motion/test_toss_cal.py` pins the
   call site structurally.
2. **The first cycle's lead grant is untested on hardware.** The modelled cost is
   one WARN line and a ~0.38 s later first release; the ladder's R4 box tells an
   operator what to expect and what a raise on a LATER cycle would mean.
3. **`ABORTED_CANT_MAKE_RELEASE` is now unreachable from static arithmetic but
   not from scheduling jitter**, and no static floor can change that. If a bench
   sitting produces one, it is a real finding about tick overshoot on a loaded
   Jetson and belongs in an investigation, not in the floor.
