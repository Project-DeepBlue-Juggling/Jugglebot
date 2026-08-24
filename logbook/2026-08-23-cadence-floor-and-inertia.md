---
title: The accept-time delay floor learns the sequence it is measured against, and the B1 positioning skip learns to read an orientation
type: bugfix
date: 2026-08-23
status: resolved-with-open-items
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
  - tools/probes/hand_decel_authority.py
  - tools/probes/hand_stroke_timeline.py
  - tests/motion/test_hand_decel_authority_probe.py
  - tests/motion/test_hand_stroke_timeline_probe.py
  - ros_ws/docs/hand_decel_feedforward.md
  - ros_ws/docs/ball_possession_contract.md
  - ros_ws/src/jugglebot/jugglebot/ball_possession.py
  - ros_ws/src/jugglebot/jugglebot/toss_record.py
  - tests/motion/test_toss_record.py
  - tests/ros/test_ball_possession.py
  - logbook/2026-08-21-ilc-primary-foldin.md
  - tests/hardware/session_anomaly_fixes.md
  - plans/active/catch-robustness.md
  - plans/active/INDEX.md
  - plans/active/toss-selftuning.md
  - tests/sim/test_hand_throw_decel_ff.py
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
   ⛔ **Item 3 is DEFEATED on the lead-GRANT path, and item 2 is worse than
   "untested" — see § Audit fixes (2026-08-24).** The 2026-08-24 audit found that
   the grant lands *exactly* on the modelled floor, and that the model charges
   nothing for the synchronous service round trips or the per-tick loop body, so
   the granted cycle aborts deterministically rather than sometimes. Carried
   `needs-design`; R4/R5/R5-prime are not bookable until it closes, R0–R3 are
   unaffected.

---

## Package 2 — the C-HAND-2 inertia re-derivation on the unclamped drive

**Verdict: the premise was wrong, and no constant was landed.** The package went
looking for evidence to *raise* `teensy_trajectory.throw_decel_reflected_inertia_kgm2`
above 9.5e-6. The ladder says the opposite — and it says it through two
instrument defects that had to be fixed before any of it could be read.

### The capture, and its plant identity

Operator flew the § CHECK HAND-7 R0–R5 ladder on 2026-08-23:
`~/Desktop/rosbags/2026-08-23_19-14-54`, **15 self-tosses, 5 tiers × 3** at
2.706 / 3.440 / 3.920 / 4.436 / 4.858 m/s, every throw caught. Verified before
scoring anything: can-bridge FW **15** (proto 5), Platform FW **3**,
`torque_ff_enabled = 1`, `torque_clamp_mask = 0`, bridge uptime 116.71 → 116.79 h.

The uptime is 4.9 days and the standing rule is to reboot before a session, so
the S2 telemetry-freeze mechanism was checked rather than assumed: over 27 415
frames the longest bit-identical `pos+vel+iq` run is **3 samples / 21 ms, at
rest**, and the longest while moving is **2 samples / 13 ms**. No ~100–150 ms
freeze anywhere in the hand stream. Median telemetry `dt` 10.0 ms. Dispatch shift
(`hand_stroke_timeline`, fit − announcement) **+8.3 / +11.6 / +19.5 ms**
min/median/max, so FW 15 holds FW 14's 10–20 ms.

### The discriminator answered, in both channels

The 2026-08-10 sitting is the true A/B: **same corrected feedforward** (Platform
FW 2, `tor_hold` 0.150 N·m), **clamp still live**, same 4.436 m/s tier, 31 tosses.

| | 2026-08-10 clamp LIVE | 2026-08-23 clamp GONE |
|---|---|---|
| commanded decel FF | 27.2 A | 27.2 A |
| worst braking `iq` per toss | min **−10.52**, median −7.05 A | min **−14.40**, median −13.95 A |
| whole-session `iq` floor | **−11.42 A** | **−17.77 A** |
| coast peak vs `x3` | **+0.719 rev** (31/31) | **−0.279 rev** (3/3) |

Braking current now exceeds the old −10 A ceiling on every toss of the top three
tiers — structurally impossible before — and scales monotonically with the
command across the ladder (9.1 A → −3.5, 16.3 → −7.3, 20.0 → −9.2, 27.2 → −14.0,
30.8 → −17.7). **`b084f98` is confirmed on the plant.**

**But the discriminator question — *does achieved decel become tier-independent?*
— answers NO.** η reads **1.018 / 1.044 / 1.053 / 1.074 / 1.076** up the ladder:
monotone, and *above* 1 at every tier, which means the hand **stopped short of
`x3` on 15 of 15 throws**. Its coast topped out 0.058–0.292 rev under the stroke
top and sagged a further 0.034–0.183 rev under the latched terminal torque,
finishing **0.113–0.468 rev below `x3`** — over row H7.4's 0.100 rev band on
every throw, by up to 4.7× (4.8× on the timeline probe's own row).

### Why that took two instrument fixes to see

Both shipped instruments reported `dip_below_x3 = 0.000  [OK]` on all 15.

**Defect 1 — the window.** `peak` was the maximum `pos_meas` over the ~400 ms
after the commanded stroke end, and `over_x3` / `eta` / the decel-side inertia
bound all derived from it. C-HAND-1's gated catch arm dispatches its prelude
37–183 ms into that window, re-seeds `pos_cmd` at the live encoder position and
climbs back past `x3`, overshooting by a measured 0.046–0.222 rev. Whenever the
throw's own coast tops out lower, **the reported peak is the arm's excursion,
~200 ms after the throw ended.** Contamination measured by re-scoring the
recordings: **15 of 15** on 2026-08-23, and **10 of the 17 tosses of the
2026-07-27 sitting every empirical number in C-HAND-2 is drawn from** — both low
tiers on every toss, where the contract's table reads `over_x3` +0.074 / +0.063
rev while those throws actually finished **−0.051 / −0.092 rev, below `x3`**. The
contract's "the plant already tracks to η = 0.98 at the bottom of the band",
which it leans on twice, was never measured.

**Defect 2 — the torque.** The per-tier bound multiplied `tor_legacy` —
`accelToTorque`, the *pre*-C-HAND-2 feedforward — long after Platform FW 2 put
`throwDecelToTorque` aboard, understating the shipped braking torque by
**1.289×** and the bound with it. The capture carries the answer directly:
`buildSegment` stops one 500 Hz sample short of `t3`, so the last commanded frame
latches the full decel feedforward across the coast. Read off the wire, the
ladder gives 0.050 / 0.090 / 0.110 / 0.150 / 0.170 N·m against a corrected model
of 0.054 / 0.087 / 0.113 / 0.145 / 0.174 and a legacy model of 0.042 / 0.068 /
0.088 / 0.113 / 0.135 — never confusable, and the probe now prints which
generation it found.

### Discussion — why refuse to land, when the sign is settled

Re-run with the coast window and the wire torque, **every tier's coast finished
below `x3`**, so the hand never caught `pos_cmd`, `τ_loop` pushed *up* through
the whole excursion, and C-HAND-2's own expression is an **upper** bound rather
than a lower one: `J_true ≤ 1.004e-5 / 1.025e-5 / 9.41e-6 / 9.63e-6 / 9.04e-6`
kg·m² by tier. The regression identification agrees in kind (slope 0.9415,
R² 0.9999 → 1.009e-5; on the measured wire torque 1.027e-5, R² 0.9915, intercept
0.84 A against the 1.50 A ± 50 % gravity hold).

Both are computed at the **commanded** release velocity, and that is where the
capture stops being decisive. The hand's encoder puts its peak velocity −5.5 % to
+2.3 % of commanded, with a `a·2.5 ms` sampling bias of −4.1 % at the top tier —
de-biased, *on target*. The **ball** disagrees: a ballistic fit `z = z₀ + v·t −
½g·t²` over the rise (mocap, static reflectors filtered, n = 3/tier, within-tier
spread < 2 points) gives **+15.5 / +11.8 / +11.3 / +9.9 / +10.6 %**, with a
fitted `z₀` of 794–832 mm against the announced 802.3 — so the fit does not rest
on the announced release height. Independently corroborated by the ILC corpus's
**+11 % fast**.

**The two channels disagree on magnitude and agree on sign, and the sign is what
the contract turns on.** `J ∝ v⁻²`, so the encoder channel bounds `J_true ≤
9.04e-6` and the ball channel `≤ ~7.5e-6` — **both below the declared 9.5e-6.**
With the direct kinematic observation (short on 15/15), the one-sided-safety
clause reads violated *in the over-braking direction*, and the contract's own
documented response to that is to **lower** the constant, never raise it.

So why not land a lower value? Because the sign is settled and the **magnitude is
not** — the two bounds differ by 20 % — and closing that by guessing costs a
Platform Teensy flash (Arduino IDE; `pio` is CAN-mute and suspended) plus a
re-validation ladder *each time*. Landing 9.5e-6 → 9.0e-6 → 7.5e-6 in sequence
would be three flashes to converge on a number that one measurement settles. The
measurement is named rather than deferred: **the rev→mm gain, measured
statically**. The two channels reconcile exactly if one motor revolution moves
~10 % more hand than the assumed 31.628 mm — the encoder is right in *rev* space
and every m/s in the hand path is 10 % low — and that also re-bases `J`, which
scales as gain⁻². It was attempted from this bag (regressing the seated ball's
mocap `z` on `pos_meas` through the ascent) and is **inconclusive**: the ball is
occluded in the cup for most of the climb, only 1 of 15 throws yielded ≥ 15
paired samples, and that fit read −21 % at R² 0.81. Recorded as not-evidence
rather than quietly dropped.

**What was NOT concluded, deliberately.** That the hand over-brakes because the
declared inertia is too large is *one* reading; that it releases 10 % fast and
therefore under-shoots a correctly-sized ramp is another, and this capture cannot
separate them. Both are consistent with every number above. The refusal is not
caution about the direction — it is that a flash sized from the wrong one of the
two is a flash in the wrong direction.

### Why `peak` was NOT narrowed, and the gated dip row NOT re-armed

Two deliberate non-changes, both about not trading a measurement bug for a worse
one. `peak` is the **end-stop** column: the question it answers is "what is the
largest excursion of any cause", and on a healthy capture the arm's own overshoot
is routinely the answer — bounding it to the coast would hide exactly the
excursions that approach the 10.8 rev stop. It keeps the wide window; a `*`
marks the tosses where it and the coast differ (15 of 15 here). And
`dip_below_x3` is an operator **ABORT threshold**: re-arming it on this evidence
would newly abort on the machine's shipped behaviour, which is a stakeholder
decision, not an instrument fix. It keeps its definition and its verdict, and
`coast_below_x3` is reported beside it with a `<<< BLIND SPOT` marker when the
two disagree across the band — so the instrument can no longer pass quietly,
without anyone's abort moving behind their back.

### Bonus measurements from the same bag (report-only, nothing applied)

1. **Sensor arrival band, the pre-R3 re-measure the constants ask for.** Raw
   `empty→held` edge vs announced landing: **+46.5 … +267.5 ms, median +184.7**,
   n = 15. The band collapsed to about a third of the shipped
   `[0.137, 0.80]` — and the **floor is now the binding half**: the earliest edge
   is 91 ms *below* `ARRIVAL_BAND_MIN_S`, so a floor left there refuses a real
   arrival (fail-closed, but it costs a false MISS on the fastest catches,
   exactly the rungs the census is trying to reach). Not applied: 15 self-tosses
   at one dwell is a thinner corpus than the 35 announcements the 0.80 was cut
   from, and both consumers size *refusals* from it. Annotated at the constants,
   with D7's derived `CATCH_CONFIRM_WINDOW_S` to move in the same commit.
2. **Hand-sensor fresh-sample cadence — the 71 ms figure is stale.** 11 462 fresh
   samples, every frame `ball_held_valid`: interval between distinct
   `ball_held_stamp` values is **median 20.0 ms, exactly the configured
   cadence** (mean 24.0, p10 20.0, p90 30.0, max 160.0). The 3.5× gap is gone and
   the distribution is one-sided — a tail to 160 ms, not a uniform stretch. The
   asymmetric-debounce numbers it was cited beside (232/241/295 ms fall, 0 ms
   rise) are **not** re-measured and still stand; the cadence was one candidate
   explanation for them and is now excluded rather than confirmed.
3. **Catch rate and seat proxy, first baseline on the restored plant.** 15/15
   throws produced an arrival edge. Raw-bit flicker in the 400 ms after the seat
   edge: **17 of 20 rises show zero transitions**, three show 2–4 — consistent
   with the operator's "every throw caught, some messy".

### Verification

- `python tools/probes/hand_decel_authority.py --self-check`, run 2026-08-23:
  **SELF-CHECK: PASS**, exit 0 — and **mutation-verified to bite**: reverting the
  coast bound to `w[stop:stop+40]` fails it with `max |recovered − built| =
  0.1458 rev`.
- `python tools/probes/hand_stroke_timeline.py --gate`, run 2026-08-23:
  **GATE PASS** on both branches (the pre-fix fixture is a truncation, where the
  coast row is correctly blank).
- `pytest tests/motion/test_hand_decel_authority_probe.py
  tests/motion/test_hand_stroke_timeline_probe.py -q`, run 2026-08-23:
  **29/29 pass in 3.17 s** — 12 of them new (7 in the new file, and the
  timeline probe's file goes 17 -> 22). Mutation-verified:
  reverting the window fails 2 of the 7; reverting the wire torque to
  `tor_legacy` fails 2 of the 7.

---

## Package 3 — the C-POSSESS-1 § 3.4 arrival clamp stops measuring the schedule

Closes the third open question the Phase-L audit carried out
(`logbook/2026-08-21-ilc-primary-foldin.md` § "Open questions carried out of
Phase L", item 3, MEDIUM): *the arrival clamp closes inside the measured arrival
band at the target cadence — clamped close at landing+0.7929 vs a band ceiling of
+0.798 — dropping `catch_event_dt_s` for the tail and letting a valid sensor
`ARRIVAL_REJECTED` veto a tracker CAUGHT; the abutment argument also assumes exact
schedule adherence.* Operator-approved 2026-08-23.

### Symptom

C-POSSESS-1.C (landed 2026-08-21, census D2) closed each cycle's ARRIVAL window
at `next_landing_t − arrival_lead_s`, the same instant the next cycle's window
opens. Below a cycle period of `ARRIVAL_BAND_MAX_S + arrival_lead_s` = **1.000 s**
that instant falls *inside the current ball's own measured arrival band*
(+137…+798 ms, 35 announcements, three 2026-08-10 bags). A catch seating in the
amputated tail then produces:

- `arrival_time` → NaN, so `catch_event_dt_s` — the ILC catch-timing measurand,
  and the only quantity this machine has for *when* the ball entered the cup — is
  silently dropped;
- a closed, empty window → `ARRIVAL_REJECTED`, which is a **positive claim of
  non-arrival**, and which `merge_possession` lets **veto a tracker CAUGHT**
  (§ 3.2 rule 2). A schedule number thereby manufactures a refusal about a ball.

Offline the same geometry mints a false `MISSED` in `toss_record.label_from_sensor`
— the label the aim fit weights most heavily.

### Diagnosis — two terms that are not the same kind of quantity

The root cause is not the arithmetic, it is the *pairing*:

| term | value | property of |
|---|---|---|
| `ARRIVAL_BAND_MAX_S` | 0.80 s | the **ball** — the latest empty→held edge ever observed on a real catch |
| `arrival_lead_s` | 0.200 s | the **schedule** — how early the NEXT window starts looking, in case its landing prediction runs late |

`next_landing_t − arrival_lead_s` charges the *next* window's guard to the
*current* window's evidence. Nothing in C-POSSESS-1.C's derivation noticed,
because at the dwell it was written against (1.50 s, R3) the two never met.

Measured over the tree's own constants across every published rung
(`/tmp/probe_arrival_clamp.py`, run 2026-08-23; offsets from this cycle's landing,
band ceiling +0.800):

| rung | flight | period | window closed at | tail lost |
|---|---|---|---|---|
| R0–R3 | 0.7977 | 2.2977–6.3977 | +1.5000 (fixed window binds) | — |
| R4 | 0.6059 | 1.2559 | +1.0559 | — |
| R5 | 0.5029 | 1.2029 | +1.0029 | — |
| **R5-prime (accepted)** | 0.5029 | 1.1629 | +0.9629 | — |
| R5′ clamp pin (0.49 / 0.4949) | 0.4949 | 0.9849 | +0.8000 | — |
| R6 (deferred fork, dwell 0.25) | 0.5029 | 0.7529 | +0.7529 | **47.1 ms** |

> **The last two rows were published wrong on 2026-08-23 and are corrected
> here (2026-08-24 audit, MEDIUM — see § Audit fixes).** They read
> `+0.7849 / 15.1 ms` and `+0.5529 / 247.1 ms`, which is the **superseded**
> `b − lead` rule, not the `arrival_boundary_t` that shipped in this very
> package. The § Discussion two subsections down always carried the correct
> 47.1 ms, so the entry contradicted itself; the probe was rewritten to CALL
> `arrival_boundary_t` rather than restate its formula, which is what made the
> two agree. Same correction landed in `ros_ws/docs/ball_possession_contract.md`
> § 3.4 (the normative copy) and the runbook's § 3.1 fifth-consumer box.

**No published rung amputates today** — the accepted operating point clears the
ceiling by 163 ms. That is why this was a MEDIUM and not a live defect, and it is
stated in the contract rather than left for a reader to derive.

### Discussion — why this shape of fix, and what was rejected

**The abutment principle stays.** A window must never outlast the machine's next
scheduled event of the kind it is looking for; two adjacent windows must neither
claim one edge twice nor leave a gap. Every alternative below was measured against
those two, plus a third the finding itself adds: *a clamp that amputates the
measured physical band is measuring the schedule, not the ball.*

- **Just drop the clamp / widen the window.** Re-opens census D2 outright: at any
  period under `arrival_window_s` the next cycle's seat edge falls inside this
  cycle's search and both rows claim it. Rejected — that is the fault C-POSSESS-1.C
  exists to close, and the fix for one inversion must not restore the other.
- **Only downgrade the refusal (clause C.2 alone).** Kills the veto but keeps the
  measurand dropped — `arrival_time` still returns NaN for the tail, and the
  sensor-primary verdict degrades to tracker-fallback on exactly the late catches
  the ILC timing channel most wants. It answers the safety half of the finding and
  ignores the measurement half. Rejected as the whole fix, **kept as the belt**.
- **A tolerance term added to the clamp.** § 3.4 already argued this down for the
  retention clamp: a tolerance has to be re-tuned at every rung of the ladder,
  which is why the guard was written as an instant. The same argument applies here
  and it has not weakened.
- **A period-dependent lead, with no new argument.** Shrink `arrival_lead_s` as
  the period tightens and the closing survives — but the *next* window computes
  its opening from `(its own landing, ITS next landing)` and does not know this
  period, so the two ends stop agreeing the moment the period varies between
  cycles. Abutment is the property being protected; a rule that only abuts at
  constant cadence is not a fix. Rejected — and it is what forced the extra
  argument below.

**What landed instead — the guard comes out of the NEXT window's OPENING.** One
boundary instant per pair of adjacent landings, evaluated by both neighbours from
the same two numbers (`ball_possession.arrival_boundary_t`, imported by
`toss_record` rather than re-derived — the discipline `DEPARTURE_LEAD_S`/
`RELEASE_GUARD_S` already carries):

```
arrival_boundary_t(a, b) = max(b − arrival_lead_s, min(a + ARRIVAL_BAND_MAX_S, b))
```

The boundary belongs to the earlier ball for as long as its measured band runs,
never past the next scheduled landing, and never earlier than the lead-based
instant — so no window that works today gets narrower. That is why the source and
the labeller now take `prev_landing_t` / `prev_landing_time`: the closing of
window `L` and the opening of window `N` are the same call on the same pair.

**The cost of relocating the guard is measured to be zero.** Across the same 35
announcements the band was cut from, *nothing arrived before its announced landing
at all* (earliest +137 ms; +46.5 ms on the 2026-08-23 FW-15 capture). The
pre-landing lead has never once been the term that caught an edge; the band's tail
demonstrably has. Surrendering the first to protect the second is not a trade
between two risks, it is a trade between a hypothetical and a measurement.

**The schedule-adherence sub-item dissolves rather than being patched.** The old
boundary was a function of `next_landing_t` — a prediction made a cycle early,
while the next window opens at the *actual* landing, so a late release pulled the
two ends apart. Under the new rule the boundary is pinned to
`L + ARRIVAL_BAND_MAX_S` and is **independent of `N`** for every period between
0.800 and 1.000 s, i.e. for exactly the cadences at which it sits anywhere near
an edge. (Under 0.800 s it is pinned to `N`, but that is the C.2 region below —
the band cannot be watched out there at all and no verdict rests on where the
boundary landed.)
Above 1.000 s it tracks `N` again, but there it is `period − 1.000 s` past this
ball's band ceiling (163 ms at the accepted operating point) and never less than
`ARRIVAL_BAND_MIN_S + arrival_lead_s` = 337 ms before the next ball's earliest
possible edge. No tolerance was added and no new instant was invented; the
dependence was moved out of the region where it could reach an edge — which is a
weaker claim than "removed" and is the one the measurements support.

**Where the band cannot be preserved, the refusal is refused (clause C.2).**
Below a 0.800 s period the next ball lands before this one's band closes and *no*
boundary rule can give both balls their whole band — at the deferred R6 fork the
window reaches +0.7529 and 47.1 ms of band goes unwatched. A window shorter than
the evidence it is judging has not observed non-arrival, so `REJECTED` (live) and
`MISSED` (corpus) become unavailable to it: it answers UNKNOWN with the cause
named (`SENSOR_BAND_CLAMPED` / `band clamped`). UNKNOWN never vetoes, so the
tracker survives — **and the loss is surfaced instead of silent**, which was the
finding's own stated requirement.

**A consequence that was not designed for, and is kept.** C.2 is written as an
invariant over the window rather than over the cadence, so it also fires when
`arrival_window_s` is configured *shorter than the band*. That relabels one
pre-existing test: § 3.3's draft 0.70 s window used to turn the measured +798 ms
catch into `MISSED`, and now turns it into `UNKNOWN` with the cause on the record.
This is the D7 derivation (`CATCH_CONFIRM_WINDOW_S` from `ARRIVAL_BAND_MAX_S`)
becoming enforceable in the labeller: a window sized under the band can no longer
produce a verdict at all. The row is lost to the fit either way; it is now lost
honestly. The test's narrative was rewritten rather than its assertion relaxed.

**Why now rather than at R6, when no rung reaches it today.** `ARRIVAL_BAND_MAX_S`
is the constant the pending post-FW14 re-measure will move (ladder § 3.1), and
moving a constant is only safe once every consumer reads it the same way. The
boundary is now a fifth consumer of that ceiling, which is recorded in the ladder:
on the 2026-08-23 capture's +267.5 ms ceiling the amputation threshold would fall
from **0.800 s to about 0.27 s**, moving R6's 0.7529 s period from "the band
cannot be watched out" to clear. *(Corrected 2026-08-24 with the table above: the
`1.000 s → 0.47 s` originally written here is `BAND_MAX + lead`, the superseded
rule's threshold. The shipped rule's threshold is `BAND_MAX` alone.)*

**One residual, named.** `prev_landing_t` is latched per SESSION and rolled per
cycle (`_reset_toss_arrival_boundary`, `_set_toss_next_cycle_perf`), so a reload
and a session's FIRST cycle pass `None` and open at the shipped
`landing − arrival_lead_s`. That is correct rather than tolerated — nothing landed
before them — but it does mean the boundary is a property of a SESSION, not of
the machine, and a future interleaving of reloads with session cycles would need
the reload's landing rolled in too.

A single `Toss` is the interesting corner: it never calls
`_set_toss_next_cycle_perf`, so it neither rolls nor clears the latch and inherits
a previous session's last landing. Left deliberately, and `arrival_boundary_t` is
what makes it safe — the boundary is a `max` against `landing − arrival_lead_s`,
so a landing older than `ARRIVAL_BAND_MAX_S + arrival_lead_s` = 1.000 s
degenerates to exactly the shipped opening, while one newer than that is a *real*
neighbour whose band genuinely overlaps, which is the case the boundary exists to
arbitrate. Clearing it at a `Toss` accept would be less correct, not more. The
same `max` is why the argument holds without a lifecycle audit: every path that
does not know its predecessor fails to the shipped window rather than to a
narrower one.

### Fix

1. **`ball_possession.arrival_boundary_t`** — the boundary, with the root cause in
   its docstring. `HandBallSensorSource._window` applies it at both ends;
   `observe` / `arrival_time` take `prev_landing_t`.
2. **`HandBallSensorSource._band_watched_out`** + the C.2 branch in
   `_arrival_state`: `ARRIVAL_UNKNOWN` / `SENSOR_BAND_CLAMPED` instead of
   `ARRIVAL_REJECTED` whenever the window closed before
   `landing + ARRIVAL_BAND_MAX_S`.
3. **`toss_record.label_from_sensor`** — the same boundary at both ends of the
   arrival search (importing the one function), `prev_landing_time`, and gate 3
   answering `LABEL_UNKNOWN` with the shortfall in the reason rather than `MISSED`.
4. **`reload_coordinator_node`** — `_toss_prev_landing_perf` / `_toss_cycle_landing_perf`
   rolled in `_set_toss_next_cycle_perf` and cleared by the named
   `_reset_toss_arrival_boundary` at session start; both consumers
   (`_possession_confirmed`, the `catch_event_dt_s` read) pass it through. The
   value handed on is `seq.t_release + seq.flight_time_s` — bit-identical to the
   `landing_t` that cycle was judged against, not a second derivation of it.
5. **`tools/probes/toss_record_miner.py`** — `prev_landing_time` from `anns[i-1]`,
   plus two rows in the built-in self-check.
6. **`ros_ws/docs/ball_possession_contract.md`** — clauses **C.1** and **C.2**,
   the two-term table, the reachability table, and the enforcement-point note.
   **`tests/hardware/session_cadence_ladder.md`** § 3.1 — the fifth consumer.

### Verification

- Rung probe, **re-run 2026-08-24** (`python /tmp/probe_arrival_clamp.py`,
  rewritten to CALL `ball_possession.arrival_boundary_t` instead of restating its
  formula): the corrected reachability table above, computed from
  `ARRIVAL_BAND_MAX_S`, `JB_BD_ARRIVAL_LEAD_S` and `JB_BD_ARRIVAL_WINDOW_S` as
  the tree holds them — shipped-rule amputation threshold **0.800 s period**
  (`ARRIVAL_BAND_MAX_S` alone; the superseded `b − lead` rule's was 1.000 s), and
  **no published rung affected — R5′'s clamp pin now loses nothing and R6 loses
  47.1 ms.** The 2026-08-23 run of this probe reported the superseded rule's
  numbers; see § Audit fixes.
- `python -m pytest tests/motion/test_toss_record.py tests/ros/test_ball_possession.py
  tests/ros/test_toss_coordinator.py tests/ros/test_toss_record_miner.py
  tests/ros/test_hand_sensor_replay.py -q -p no:randomly`, run 2026-08-23:
  **352 passed in 108.17 s** — 9 of them new (4 in `test_ball_possession.py`,
  4 in `test_toss_record.py`, 1 in `test_toss_coordinator.py`).
- Miner self-check, run 2026-08-23 (`toss_record_miner.self_check()`):
  **63/63 self-check cases pass**, up from 61 — the two new rows are the band
  tail and the band-clamped UNKNOWN.
- `./run_tests.sh --full`, run 2026-08-23: **RESULT: PASS** — parallel 533 s
  (**6165 passed, 3 skipped, 3 xfailed**) + serial 46 s (**9 passed**),
  total 579 s.
- **Mutation-verified, both halves separately** (run 2026-08-23): with the three
  production files stashed, 5 of `tests/motion/test_toss_record.py` go RED
  (`…share_ONE_arrival_boundary`, `…band_TAIL_is_not_labelled_missed`,
  `…reach_back_for_the_PREVIOUS_balls_seat_edge`,
  `…band_clamped_search_declares_unknown_rather_than_missed`, and the rewritten
  draft-window test). With only the BEHAVIOUR reverted in `ball_possession.py`
  and `arrival_boundary_t` left defined — so the check isolates the fix from the
  new API surface — 3 of `tests/ros/test_ball_possession.py` go RED; reverting
  the `prev_landing_t` clamp alone reds the fourth.

---

## Audit fixes (2026-08-24)

The independent audit of the three packages returned **NOT CLEAN — 1 BLOCKING,
1 HIGH, 3 MEDIUM, 3 LOW**. Everything it verified clean stayed clean: no codegen
drift, py3.8 compiles, the new tests are parallel-safe (no fixed paths or ports,
no new markers), the runbook's § 2.9 operator command reference matches the
`.action` IDL field-for-field, `arrival_boundary_t` / `_band_watched_out` are
correct and consistently implemented across the live source and the offline
corpus labeller, and `levelling.uncorrect_pose` is an exact transpose inverse
evaluated at the frame-invariant position key.

**Five landed; four are carried.** The split follows the house rule — fix every
BLOCKING/HIGH whose diagnosis is `clear`, carry `needs-design` and
`needs-operator` untouched, take MEDIUM/LOW only when trivially safe. Both of the
severe findings are `needs-design`, so **nothing in this package's production
code changed in this pass**; what changed is that the machine now says so where
an operator will read it.

### Landed

1. **MEDIUM — `ros_ws/docs/ball_possession_contract.md`'s C-POSSESS-1.C.1
   reachability table published the SUPERSEDED boundary rule in its last two
   rows,** so the normative contract contradicted its own prose two paragraphs
   later. Re-derived against the shipped `arrival_boundary_t`: the R5′ clamp pin
   loses **nothing** (the table said 15.1 ms) and R6 loses **47.1 ms** (the table
   said 247.1 ms). The root cause is worth naming because it is the reason the
   discipline exists: the 2026-08-23 probe **restated the boundary formula**
   instead of calling `arrival_boundary_t`, and it restated the version the
   clause replaces. `/tmp/probe_arrival_clamp.py` now imports the function — the
   same "one home, never a second derivation" rule the fix itself landed for
   `toss_record`. Corrected in all three places that carried the number: the
   contract, this entry's Package 3 table, and the runbook's § 3.1
   fifth-consumer box (which quoted the superseded 1.000 s / 0.47 s thresholds;
   the shipped rule amputates below `ARRIVAL_BAND_MAX_S` **alone** — 0.800 s
   today, ~0.27 s on the 2026-08-23 capture's ceiling).
2. **MEDIUM — stale LEVEL/AIMED residue survived in the operator-facing body of
   the runbook whose headline says the split is GONE.** The R4 and R5 section
   headings still published two dwell/delay pairs, R5's measurable gate still
   demanded "sustained 49.9 (level) / 37.9 (aimed) throws/min", and the
   0.31-vs-0.301 cliff box still quoted the retired "0.620 s level / 1.000 s
   aimed" floor with a 9.7 ms clearance against a superseded 0.63 dwell. All
   collapsed to one column and **re-derived rather than re-typed**
   (`/tmp/probe_cliff_box.py`, which calls the probe's own `fastest_at`, run
   2026-08-24): R5 is 49.9 throws/min either way; choosing `0.31` over `0.301`
   costs **0.12 throws/min** with the aim disarmed and **1.48** with an ILC
   artifact loaded — the binding column, and **4× what the retired pair
   reported**, which is exactly the kind of drift a collapsed-column edit hides
   if the numbers are carried across by hand.
3. **LOW — `plans/active/critical-point-ilc.md` decision-3's "Why" cell asserted
   that the ACCEPTED operating point is refused.** The Decision column was
   rewritten to R5-prime `dwell 0.66 / delay 0.44`; the Why column still carried
   the arithmetic for the retired 0.49 s definition, ending "R5-prime as decided
   is now 43 ms SHORT and is REJECTED_DWELL". Both statements are true of
   *different* R5-primes, so the cell now names which is which — the 0.49 s
   original is what is 43 ms short, and that is *why* the row was re-taken.
4. **Consistency, found while closing out rather than by the audit: the runbook's
   § 3.2 pre-R3 gate was still open against a number this very package
   measured away.** It told an operator to open a can-bridge investigation into a
   "~71 ms poll cadence vs the configured 20 ms, 3.5× gap, no diagnosis" — while
   § "Bonus measurements" above reports the same quantity at **median 20.0 ms,
   exactly the configured cadence**, on the FW-15 capture. Marked done in the
   runbook and struck in `plans/active/toss-selftuning.md` § 11.4's pre-R3 list,
   both carrying the explicit warning that the **asymmetric-debounce** numbers it
   was cited beside (232/241/295 ms fall, 0 ms rise) are NOT re-measured: the
   poll cadence was one candidate explanation for them and is now *excluded*,
   which makes them more in need of a mechanism, not less. § 3.1 — the arrival
   band re-measure — is untouched and is still the gate on R3.
5. **LOW (documentation half only) —
   `tests/sim/test_hand_throw_decel_ff.py::test_declared_inertia_cannot_over_brake`
   read as live confirmation of one-sided safety that no longer holds.** It pins
   the declared 9.5e-6 under `_MEASURED_REFLECTED_INERTIA_MIN_KGM2 = 1.0126e-5`,
   a bound `hand_decel_feedforward.md` now marks SUPERSEDED — and Package 2 above
   puts the tightest measured bounds at 9.04e-6 (encoder) and ~7.5e-6 (ball),
   **both below the declared value**. The assertion is unchanged (see the carried
   list); the module docstring and the test docstring now say plainly that a PASS
   here is a **regression fence against re-raising the constant**, not evidence
   the failure mode is closed on the current plant, and that lowering the
   constant without landing the firmware value in the same commit reds the suite
   on a machine nobody changed.

### Carried untouched — and what each one costs

**BLOCKING (`needs-design`) — the first-cycle lead GRANT sets `throw_delay`
exactly on an idealised tick-grid floor, so the cycle it grants aborts.**
`reload_coordinator_node._build_toss_cycle` sets `cycle_delay =
min_throw_delay_for_release_s(...)` — equal to the floor, whose only headroom
over the runtime guard is `FLOOR_REPRESENTATION_SLACK_S = 1e-6 s`, a value named
for beating a floating-point identity and never intended to buy wall-clock time.
`pre_dispatch_budget_s` charges 23 ticks × 20 ms and charges **zero** for the
`go_to_pose` service round trip the node makes synchronously inside tick 0,
**zero** for the PREPARE bundle's synchronous service calls, and **zero** for any
tick's loop body (`time.sleep(_TICK_S)` is a lower bound on a tick). Every one of
those costs is strictly positive, so the release-window guard at the DISPATCH
tick mints `ABORTED_CANT_MAKE_RELEASE` — with the catch armed, the announcement
out and the hand retracting under a seated ball. **This is the package's own
central contract, defeated on the one path the package itself added.**

*Scope, exactly.* The grant only fires when the asked delay is under the MOVING
floor, so R0–R3 (asking 0.90–5.00 s against 0.7414 s) never take it and are
unaffected; **R4, R5 and R5-prime ask 0.45 / 0.47 / 0.44 and every one takes
it.** There is no operator-side workaround at those rungs: raising the delay past
the moving floor by hand raises `required_dwell_s = delay + handoff_margin` to
~0.96–1.00 s and the session then answers `REJECTED_DWELL` at the published
dwells, so honouring it lands near 40 throws/min — slower than R4.

*Why the committed probe greens it, and why that is not a probe bug.*
`cadence_rung_check` drives the real FSMs at **exact** tick boundaries; its own
docstring says the PREPARE bundle's round-trips "are charged as zero". So `first
… FLIES` means *flies on an ideal clock*, and this finding is the gap between
that clock and the node's. It is emphatically **not** a jitter question — jitter
is the runtime guard's job by design (§ "A knife edge the model exposed" above
argues against a static jitter allowance, and that argument stands); this is a
static term omitted from a static floor. Which is also why it is `needs-design`:
the honest fix is to make the budget charge the sequence's real synchronous
costs — measured, not guessed — and nothing in this tree measures them yet.

**HIGH (`needs-design`) — on the SHIPPED Tier 8b the B1 skip cannot fire on a
chain, so the 0.38 s / 34 % claim is demonstrated on Tier 8a only.**
`JB_OP_TOSS_TIER` ships `8b`. `toss_sequencer._reach_action_if_due` emits
`ACTION_REACH_CATCH` unconditionally for 8b at `t_release`, and
`_publish_toss_reach` publishes a `catch/dynamic_target` whose `target_quat`
comes from `catch_coordinator.predicted_catch_command(announced landing)` →
`compute_catch_orientation(landing_velocity)`: a **receive tilt for the incoming
ball**, essentially level for a self-toss, never the throw's aim pre-tilt. The
held pose at cycle end therefore has the aim taken *out* of its orientation, the
next cycle's `_toss_already_positioned` fails the 2.71 mrad test, and POSITIONING
commands the move again. **Compounded with the BLOCKING**: every cycle then takes
the grant, so on 8b the exposure is every cycle, not only the first.

*What is verified and what is not.* The orientation mechanism is traced end to
end in the source. The finding bites when an aim is actually armed (an ILC
artifact, a calibration map, or a displaced 8b site); with no artifact loaded the
aim composes to exactly zero, the pre-tilt is level, and a level reach and a
level pre-tilt agree, so a zero-aim 8b chain may still skip. Whether the reach's
*position* (`landing − hand_catch_offset · platform_z`) also breaks the 17.5 mm
test is **not** measured. The honest summary is the one now in the runbook:
**every published cadence number in this arc was produced on the 8a-equivalent
path.** `needs-design` because the fix is a real fork — teach the reach the throw
pre-tilt, gate it at zero displacement, or teach B1 that an 8b cycle ends at the
catch pose — and each answers a different question about what an 8b cycle *is*.

**MEDIUM (`needs-operator`) — the published per-rung clearances are measured
against the same idealised budget.** R5-prime's starred 12.0 ms delay / 11.9 ms
dwell and R4's 13.9 ms dwell all come from the probe, and the runbook's own § 2.0
says the PREPARE bundle's synchronous round-trips "are charged as free". Those
omitted costs are of the same order as the clearance, so the starred operating
point may sit inside the noise of its own floor. It resolves with the BLOCKING —
measure the sequence's real cost once and every number here re-bases — so it is
deliberately not patched with a guessed margin.

**LOW (`clear`, not taken — a production change on the shipped 8b path) — a torn
read across two independently-stamped caches.** `_build_toss_cycle` sets the
Tier-8b throw site A from `_live_commanded_position` (Point) and backs the B1
skip with `_live_commanded_pose` (Pose). The design explicitly avoids a torn read
*inside* the Pose (§ "One message, not two topics") and reintroduces one *across*
the two topics, so A can come from sample N while "already positioned" is judged
against N+1. Not taken here for two reasons. First, changing A's source changes
which topic must be alive for an 8b goal to be accepted at all, and
`REJECTED_POSE_UNKNOWN` is pinned to the existing one — that is not a trivially
safe edit on the shipped tier. Second, the only consequence of the tear is
whether the B1 skip fires, and the HIGH above says it cannot fire on 8b anyway:
this belongs in the same redesign, not ahead of it. The existing code comment's
argument survives untouched — A is NOMINATED, not observed, so a stale read
yields a *self-consistent* throw site.

**LOW (`needs-operator`) — the C-HAND-2 inertia constant itself.** Documented
above (landed item 5); the value is not changed. See § "why refuse to land, when
the sign is settled": the sign is settled and the magnitude is not, the two
channels differ by 20 %, and each candidate costs a Platform Teensy flash
(Arduino IDE — `pio` is CAN-mute and suspended) plus a re-validation ladder. The
reconciling measurement is named and cheap: **the rev→mm gain, taken statically.**

### Verification

- `./run_tests.sh --full` (every tier, `nightly` included), run 2026-08-24 as the
  first close-out gate: **RESULT: PASS** — parallel **6165 passed, 3 skipped,
  3 xfailed in 518.63 s**, serial **9 passed in 41.69 s**, total 567 s. Counts
  identical to the package's own 2026-08-23 closure run, as they must be: this
  pass changed one Python docstring and seven markdown files, and added no test.
- `./run_tests.sh --full`, re-run 2026-08-24 as the COMMIT gate (the close-out
  session died to an API outage between the run above and the commit, so the
  gate was re-run by the salvaging session on the final tree): **RESULT: PASS**
  — parallel **6181 passed, 3 skipped, 3 xfailed in 523.41 s**, serial **9
  passed in 41.92 s**, total 571 s. The +16 over the run above is the
  interleaved observability-session commits (61aea25/2995855/145484c), which
  added tests after the first gate ran. Caveat, stated per the load-flake
  doctrine: this run overlapped a foreign scoped pytest for part of its window;
  it finished all-green, which the doctrine holds valid (only load-flake
  FAILURES under shared CPU are suspect).
- `/tmp/probe_arrival_clamp.py`, rewritten to CALL `ball_possession.arrival_boundary_t`,
  run 2026-08-24: the corrected reachability table — R5′ clamp pin closes at
  **+0.8000** (loses nothing), R6 at **+0.7529** (loses **47.1 ms**),
  amputation threshold **0.800 s** of cycle period.
- `/tmp/probe_cliff_box.py`, run 2026-08-24 (calls `cadence_rung_check.fastest_at`):
  `h 0.301` → 54.29/min aim-disarmed, 54.16/min ILC-loaded; `h 0.31` → 54.17 and
  **52.68**; `h 0.300` refuses (T 0.494720, **0.16 ms** under
  `MIN_FLIGHT_TIME_S`). R5-prime's `required_dwell_s` at its own 0.44 s delay is
  **0.6303** (aim disarmed) / **0.6481** (ILC loaded) — clearance 29.7 / **11.9**
  ms; delay floor **0.4144 / 0.4280** — clearance 25.6 / **12.0** ms.
- `python tools/probes/cadence_rung_check.py --solve`, run 2026-08-24 against
  this tree: **PUBLISHED LADDER: all rungs FLY** (7 rungs × {ILC off, ON} ×
  {chained, first}) — reproduced here precisely so the BLOCKING's "the probe
  cannot see it" claim is on the record beside the probe's own green.
