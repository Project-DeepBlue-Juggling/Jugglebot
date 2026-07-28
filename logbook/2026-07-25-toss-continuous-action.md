---
title: TossContinuous — repeated toss-catch cycles as a sequencer, not a capability
type: feature
date: 2026-07-25
status: in-progress
phase: "Self-toss anomaly fixes — Phase F (TossContinuous, the programme finale)"
related_plan: "single-ball-toss.md"
files_changed:
  - ros_ws/src/jugglebot_interfaces/action/TossContinuous.action
  - ros_ws/src/jugglebot_interfaces/CMakeLists.txt
  - ros_ws/src/jugglebot/jugglebot/toss_session.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/toss_release.py
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/docs/ball_possession_contract.md
  - tests/ros/conftest.py
  - tests/ros/test_toss_session.py
  - tests/ros/test_toss_continuous_node.py
  - tests/ros/test_toss_coordinator.py
  - tests/hardware/toss_trace_recorder.py
  - tests/hardware/session_anomaly_fixes.md
  - tools/probes/toss_trace_synth.py
  - tools/probes/README.md
  - plans/active/single-ball-toss.md
commits:
  - <pending>
subsystem:
  - ros
  - motion
  - config
tags:
  - safety
  - testing
  - docs
---

# TossContinuous — repeated toss-catch cycles as a sequencer, not a capability

## Summary

`TossContinuous` runs `num_throws` toss-catch cycles from one goal with a
configurable dwell between them — the operator-requested bridge from the
validated single `Toss` to two-ball juggling. It ships as a **sequencer**: every
cycle is an ordinary `Toss` built by `_build_toss_cycle` and ticked by
`_run_toss_cycle`, the same two methods the single `Toss` now uses after those
were extracted, so a session cannot drift from the toss the hardware ladder
validated. A pure-Python outer FSM (`toss_session.py`) owns only *when* the next
cycle starts, *whether* it starts, and the per-cycle accounting. `stop_on_miss`
defaults TRUE on the wire. **Nothing in this phase has run on hardware.**

Finalize adjudicated fourteen reviewer findings and fixed ten. Two of them were
about the same thing from two directions and mattered most: the CS-1..CS-6 trace
checker this phase ships as the bench gate had only ever been validated against
an all-CAUGHT session, while the runbook makes an all-MISSED dry trace the
*mandatory* capture before any ball flies — so the instrument was one-sided, and
building the missing case immediately proved it would have FAILed a hard-STOP row
on the first correct capture of the sitting.

## Motivation

Phases A–E made a single self-toss work: throw from a nominated site, reach to
the catch point mid-flight, mint a CAUGHT verdict, hold. The operator's ask
(decision (c), 2026-07-28) was repetition — a cadence, from one goal, that can be
watched and scored.

The failure this phase is built against is not "repetition is hard". It is that
**a repeated-cycle capability can silently diverge from the single toss the
hardware ladder validated, and the divergence is only discoverable at a
sitting.** Every arming order, precondition, abort ladder and terminal in the
toss path is load-bearing and was paid for in powered time. A second copy of the
cycle construction — a coordinator goal-loop, a duplicated tick loop — would
start identical and drift, and the drift would surface as a ball on the floor.

## Design

**The session is a sequencer.** `_build_toss_cycle` (resolve tier → throw site →
release state, construct and start the `TossSequencer`, install per-goal node
state) and `_run_toss_cycle` (tick to a terminal, safe on every node-level exit,
emit the one authoritative outcome line) were extracted from `_execute_toss` and
are now called by exactly two callers: the single `Toss` and each session cycle.
The extraction is behaviour-preserving on the existing surface — 345 pre-existing
toss/reload tests pass with only the one deliberate assertion added below.

**Five invariants live in the module docstring with one enforcement point each**
(S1 at most one live cycle; S2 the session commands no motion of its own; S3
`stop_on_miss` stops at the cycle boundary and introduces no new abort point; S4
cancellation obeys the per-cycle phase rules verbatim; S5 the dwell is a
quiescent wait). No new normative contract document: a contract is for a class of
failures with multiple enforcement sites, and this is composition with one FSM
and one consumer.

**The dwell is a quiescent wait, not a stretched `throw_delay`.** Both satisfy
the same arithmetic. The rejected one leaves `catch/armed` RAISED for the whole
dwell with a ball resting in the cup, which keeps `catch_coordinator`'s reactive
catch path live — verified in the mechanism, not the prose: both
`_on_throw_announcement` and `_prime_retry_tick` are gated on `_catch_armed`, and
`_toss_stay` lowers that latch first.

**The dwell floor is derived, and the brief's 2.0 s figure was refuted, not
adopted.** `dwell_floor = throw_delay + dwell_margin_s`. A 2.0 s dwell would need
`throw_delay <= 1.458 s`, which is 2.042 s below the toss FSM's own
`MIN_TOSS_THROW_DELAY_S = 3.5` (verified against the real FSM: 3.49 rejects, 3.50
dispatches). Lowering that floor changes the arming/release timing of a
hardware-validated path — a safety fork, deliberately not taken. Absolute floor
**4.10 s**, **5.60 s** at the default delay, config default 6.0 s.

**Phase E's known limitation became a pre-throw refusal.**
`REJECTED_CHAIN_UNREACHABLE` runs the same `predicted_catch_command` policy the
deferred A→B reach publishes from, so a near-cap session is refused before a ball
flies instead of throwing one, catching it, and then rejecting cycle 2
`WORKSPACE` with the platform parked off-box. Measured frontier: `|B| <= 146.5`
chains, `>= 147.0` does not; a fixed-B chain converges, so cycle 2 is the only
one at risk.

## Discussion

### The finding that mattered most: an instrument validated on one side only

The panel's strongest signal was convergence — two reviewers, from a contract
lens and a regression lens, independently said the CS-1..CS-6 verification matrix
contains no MISSED-cycle continuous trace. `gen_continuous` emitted `Toss CAUGHT`
unconditionally, modelled only the `ACTION_STAY` teardown, and held `pos_cmd` at
0.0 through every dwell **by construction**. Every `viol_cs*` case was a
single-field mutation of that same all-CAUGHT trace.

Meanwhile runbook `CONT-STEP-1` — the capture that must pass *before any ball
flies* — is a 3-cycle no-ball session in which every cycle ends MISSED through
its own `SAFE_ABORT`. That ladder publishes `catch/armed False` **first** and only
then retracts the hand, so the cycle's own mandated retract lands inside the dwell
window CS-3 measures, and on a dry trace (no catch stroke to re-park the hand)
that retract is a ~10 rev descent.

This is the two-sided acceptance criterion an instrument owes: it must produce the
right reading on a capture of the shape it must FLAG *and* on a capture of the
shape it must ACCEPT. Building `cont_dry` and running it found two real defects
in the same check, in sequence:

1. **CS-3's hand bound was inherited from the wrong window.** `CMD_POS_TOL_REV =
   0.05 rev` sizes DT-5's *pre-announcement* window, where the hand is idle and
   nothing has been dispatched. Applied to the post-disarm window it is 1.12×
   the real residual: replaying the 16 real post-disarm → next-prime_hold windows
   of `temp/logs/toss_trace_2026-07-27_15-39-50.jsonl` that follow a toss (the
   larger excursions all contain an intervening Reload and are not
   dwell-shaped) gives a worst `|delta pos_cmd|` of **0.0446 rev**. A coin-flip
   false STOP on a healthy machine, on a row the runbook marks a hard STOP.
2. **"Once parked, stays parked" was still wrong, and the instrument said so.**
   The first repair anchored the baseline on the first sample inside the ±0.5 rev
   park band — and `cont_dry` promptly FAILed with `0.250 -> 0.000 rev`, because
   the park band is wide enough that the *tail of the retract ramp* is already
   inside it, so the baseline landed mid-motion. That is the same defect class
   one layer down, and it was caught only because the new case existed.

The landed formulation is **no-ascent**: every hazard CS-3's hand half exists to
catch is upward (an auto-prime kind-3 rising to ~9.96 rev with a ball in the cup,
or a new throw stroke); every legitimate command in the window is downward toward
the 0 rev retract target; and the next cycle's own prime cannot appear before
`prime_hold True`, which is where the window ends. So the check is a running
minimum with a measured tolerance of 0.15 rev (3.4× the 0.0440 rev worst real
ascent, 10× below `ASCENT_POS_REV`, 66× below a real ascent), plus a new failure
for a window in which the hand never reaches park at all. Validated three ways:
`cont_dry` PASSes 6/6, the new `viol_cs3_hand` case FAILs CS-3 with zero
collateral, and all 16 real windows pass with 3.4× margin.

**What was ruled out.** Simply raising `CMD_POS_TOL_REV` globally — DT-5 depends
on the tight value and it is a different window with a different error model.
Excluding the retract by moving the window start to `prime_hold False` — the
retract's `pos_cmd` ramp outlives that edge (it is dispatched on an ack, not on
motion completion), so the exclusion would have been approximate and would have
blinded the check to anything between the two edges.

### The extraction regressed the single Toss's shutdown terminal

A lone regression-lens finding, PROVEN on trace. At HEAD the `rclpy` shutdown
branch of `_execute_toss` returned its result with **no** goal-handle transition.
After the extraction, `_run_toss_cycle` returns `(result, 'shutdown')`,
`_execute_toss` discarded the exit kind, and `r.success is False` fell through to
the common `goal_handle.abort()`.

This is exactly the class of drift the phase exists to prevent, running in the
opposite direction to the one anticipated: the *session* handles shutdown
correctly and explicitly, and the session's own test asserted parity with "the
single Toss does the same" — which the same change had made false. The
pre-existing `test_rclpy_shutdown_aborts_and_safes` asserted the outcome string
and the safing but never the handle, so it passed both ways.

The consequence is not cosmetic: a status transition on a dying executor can
itself raise, and `_execute_toss`'s `except` would then overwrite the
`ABORTED_SHUTDOWN` line already in the log with `ABORTED_EXCEPTION` and re-raise,
turning a clean Ctrl-C into a fault trace. Fixed by short-circuiting on
`exit_kind == 'shutdown'`, and the invariant is now pinned on the single Toss
rather than only claimed by the session.

### The dwell floor was sized for the handoff that commands nothing

The physics lens proved, from landed constants alone, that `dwell_margin_s = 0.6`
sizes only the **CAUGHT** handoff — a verdict lands, two ticks pass, nothing is
commanded. A MISSED cycle the session continues past hands over through a whole
`SAFE_ABORT` ladder, and **every rung of it returns on a service ack**:
`_go_home()` returns when `trajectory_node` has *installed* a 2.0 s recentre
profile, `_retract_hand_with_retries()` on the first successful ack. So
`_run_toss_cycle` returns while the machine is still moving.

At the shipped defaults the naive arithmetic is already inside that ladder:
`dwell 6.0 − delay 5.0` starts cycle N+1 1.0 s after the landing, 1.7 s before the
recentre lands. The module docstring's claim that a cycle "whose cleanup ran long
simply reports a longer achieved dwell" was true of the FSM and false of the
session, and the test that pinned it is pure-FSM and structurally blind to the
difference — the mocked-ROS-blind-to-choreography class this project has a memory
note about.

**Two candidate fixes, and why the floor won.** Making the *dwell floor*
`stop_on_miss`-dependent (refuse a short dwell on a miss-continued session) is
consistent with the module's refuse-don't-stretch doctrine, but that doctrine is
about the requested cadence at CHECKING; it would also make the all-defaults goal
with `stop_on_miss: false` REJECTED, i.e. it would refuse the exact configuration
the mandatory dry trace uses. The landed fix instead applies
`DEFAULT_SESSION_MISS_CLEANUP_S = 2.80 s` (`CATCH_CONFIRM_WINDOW_S 0.7` +
`go_home` 2.0 + two node ticks) as a **floor on landing → next cycle start after a
non-success cycle**. It can only lengthen a gap, never shorten one, so any session
that already dwells long enough is bit-unchanged — and it makes the docstring's
existing claim true by construction instead of accidentally false.

**The tradeoff accepted**: three of the constants it is derived from live in
other files (`trajectory_node`'s `go_home_duration_s` parameter default,
`reload_coordinator_node._TICK_S`, `toss_sequencer.CATCH_CONFIRM_WINDOW_S`). Only
the last is importable without a ROS dependency, so the other two are named module
constants pinned by a regex drift-guard against their source files rather than
imported. A drift guard is weaker than an import; a ROS import into a pure-Python
module is worse.

**Neither consequence is a hazard, and that is why this is a MEDIUM and not a
stop.** A cycle built mid-traverse reads a transient `commanded_position` as its
throw site and the resulting toss is self-consistent (the `go_to_pose` replaces
the in-flight `go_home` as a profiled plan, which is the normal path); a hand
still descending yields `REJECTED_HAND_NOT_PARKED`. Both are loud refusals. What
made it worth fixing is that the second one is a *machine-fault verdict for a
cadence fault*, and it would route the operator to the wrong subsystem.

### The advertised 4.10 s floor was not a floor

Session CHECKING validated `dwell >= throw_delay + margin` but never bounded
`throw_delay` itself, so `throw_delay 2.0 / dwell 3.0` satisfied the inequality,
was ACCEPTED, built a whole cycle's per-goal state, and then died
`REJECTED_CANT_MAKE_LEAD` inside the cycle FSM — naming a field the operator did
not think was in play. `REJECTED_THROW_DELAY` now gates it first, so the number
four artefacts advertise is real and the refusal names the field that is wrong.
Nothing moves either way; this is a spec-versus-code inconsistency, fixed because
an advertised floor that is only an arithmetic identity is a trap for the next
reader.

### `catch/reach_center` was in a silence set it must not be in

Two lenses converged again, one rating it PROVEN and one NOT-PROVEN. The
coordinator publishes `prime_hold True` and `reach_center` as adjacent statements
in one FSM tick on two different topics — measured as close as 131 µs apart on
the 2026-07-27 bag — and CS-3's silence window *ends* at that `prime_hold True`.
This file's own RF-3 banner says same-wait-set arrivals can be observed in either
order, so an inversion puts a correct declaration inside the window and FAILs a
STOP row. Removed from `DWELL_SILENT_TOPICS` and the remaining scan backed off by
the observation epsilon. **No coverage is lost**: CS-4 requires exactly one
`reach_center` in `[disarm_i, arm_{i+1}]`, which spans the whole gap and is
strictly stronger than "none in its leading part".

### Two documents that promised more than the code delivers

`CS-6`'s docstring claimed it scores "a cadence that is never EARLY of the
requested dwell". It cannot: the requested dwell is not in the trace, and the code
compares only against the previous scheduled landing — so a collapse from 8 s to
2 s would PASS. Reworded to what it proves, with a pointer to the rows that do
score the request (`CONT-1.8` / `CONT-2.5`, off the action result's
`per_cycle_dwell_s`) and an explicit note that the value CS-6 prints is the
*independent wire-side measurement* of the same quantity — which is what makes
those rows a cross-check rather than the coordinator marking its own homework.

`ball_possession_contract.md` § 7.1 claimed the dwell-retention gap closes when
the hand sensor becomes `_possession_source`, "with no edit to the session FSM,
the action, or the coordinator". Traced and refuted: cycle N+1's `ball_seated` is
a **plain read of the sticky `_ball_possession` latch**; the only in-observation
call to `_possession_confirmed` is gated on `announced_id is not None`, which
`_build_toss_cycle` resets to `None` every cycle; `_on_balls` can only ever SET
the latch True; and `PossessionSource.judge` is reachable only from a `/balls`
CAUGHT message, while the tracker prunes the terminal track ~2 s after CAUGHT
against a 4.10 s dwell floor. § 7.1 now names the two coordinator edits retention
actually needs, so the sensor phase does not inherit a belief that the work is
already done.

### What was deliberately NOT fixed

**The unprotected pre-`try` region of `_execute_toss_continuous`** (LOW,
NOT-PROVEN by both reviewers who raised it). The structural claim is true —
`_predicted_chain_site_mm`, the throwaway budget sequencer and the deadline
computation run before the `try` whose `finally` releases `_goal_claimed`, so an
exception there would wedge all three ball-op actions until relaunch. But neither
reviewer could construct a reachable input, the same shape pre-exists in
`_execute_toss` at HEAD, and the safe restructure is not the one-line move it
looks like: the `except` and `finally` both reference `session`, so hoisting the
`try` above its construction needs `session is None` guards on two handlers. A
late structural change to an exception path, to close a window nobody could reach,
is the wrong trade at finalize. Recorded as an open question with the trace, so
the next session can do it deliberately.

**The last-cycle `stop_on_miss` ordering test** the implementer flagged. The
ordering (STOPPED_ON_MISS adjudicated before the completion check) is deliberate
and reasoned; the gap is only that no test pins the *final*-cycle case
specifically. Left open rather than added, because it is a coverage gap in a
behaviour that is otherwise pinned, not a defect.

**`tools/probes/toss_trace_synth.py`'s `viol_dt7` repair is a separately-diagnosed
pre-existing defect** and by the repo's own rule would earn its own commit. It
does not get one, deliberately: it lives in the same `CASES` table and the same
generator this phase rewrote, so the two are not independently revertible, and the
matrix being clean is a *prerequisite* for trusting the CS cases rather than an
unrelated fix. Called out here and in the commit body instead.

## Implementation

- **`ros_ws/src/jugglebot_interfaces/action/TossContinuous.action`** — new action.
  `stop_on_miss` carries `true` as an IDL field default (verified honoured by
  Foxy's `rosidl_adapter` against the real generated message, not only the mock).
  Result carries per-cycle arrays plus counts and no scalar means: a mean over a
  mixed NaN/finite array has a defined-vs-undefined edge that reads as zero when
  it means unknown, and every consumer has the array.
- **`toss_session.py`** — the pure-Python outer FSM. Takes no observations at all;
  reasons about time and cycle results only. `REJECTED_THROW_DELAY` →
  `REJECTED_NUM_THROWS` → `REJECTED_DWELL` → `REJECTED_CHAIN_UNREACHABLE`,
  strictest first. `success` requires the `COMPLETED` terminal as well as the
  counts (closing the vacuous-success class the implementer found, where
  `num_throws = 0` satisfied `0 == 0` and the node would have called `succeed()`
  on a rejected goal). `DEFAULT_SESSION_MISS_CLEANUP_S` floors the post-miss gap.
- **`reload_coordinator_node.py`** — `_build_toss_cycle` / `_run_toss_cycle`
  extracted; third action server on the same one-ball-op claim; the chain
  pre-check; the `exit_kind == 'shutdown'` short-circuit; one `SESSION_CHECKING`
  feedback at accept so the documented phase is observable; the busy warning now
  names all three actions.
- **`tests/hardware/toss_trace_recorder.py`** — CS-1..CS-6, with CS-3 reformulated
  as above and `DWELL_CMD_POS_TOL_REV` derived from measurement.
- **`tools/probes/toss_trace_synth.py`** — `cont_dry` and `viol_cs3_hand` added;
  `viol_dt7` repaired.
- **Config** — `jugglebot_operational.toss_session_dwell_default_s` (6.0),
  `_dwell_margin_s` (0.6), `_max_throws` (20). Regeneration verified deterministic
  (a second run leaves the tree byte-identical).

## Verification

**Synthetic matrix, both directions** —
`python tools/probes/toss_trace_synth.py --all --verify`, run 2026-07-29:
**30/30 cases OK, matrix CLEAN**. Includes both happy shapes (`cont_happy` 6/6
PASS exit 0; `cont_dry` 6/6 PASS exit 0) and one violation trace per invariant,
each FAILing exactly its target with zero collateral. The matrix was RED at HEAD
on `viol_dt7`.

**Real-data acceptance of the changed check** — the new CS-3 rule applied to the
16 real post-disarm dwell windows of
`temp/logs/toss_trace_2026-07-27_15-39-50.jsonl`: worst ascent **0.0440 rev**
against the 0.15 rev bound, **16/16 pass**, all reach the park band. Under the
pre-fix rule the same windows sat at 1.12× margin.

**Full suite** — `pytest tests/ -q`, run 2026-07-29 on the Jetson in the project venv (07:47:07 -> 08:11:09): **4254 passed, 3 xfailed, 198 warnings in 1435.48 s (0:23:55)**, exit 0. That is (+114 on the `3332bc6` baseline of 4140 — accounted EXACTLY: `--collect-only` reports 70 tests in `test_toss_session.py` and 44 in `test_toss_continuous_node.py`, both new files; xfail unchanged at 3, so no test was weakened to reach green).
No hot-loop allocation-budget flake occurred.

**Not verified, and this is the whole risk**: no part of this phase has run on
hardware. `tests/hardware/session_anomaly_fixes.md` § SECTION CONT is the
authority and runs LAST, after § SECTION POSS (whose verdicts it consumes) and
§ SECTION DISP (whose STAY terminal makes chaining possible).

## Deployment

**`colcon build --packages-select jugglebot_interfaces jugglebot` +
`source install/setup.bash` + RELAUNCH `jugglebot_launch.py`. BOTH packages.**
`reload_coordinator_node` imports `TossContinuous` at module scope, so a stale
interfaces package raises `ImportError` before the node is constructed — and that
node hosts *all three* ball-op actions, so `Reload` and `Toss` go down with the
session.

This falsified the runbook far more widely than the review found. Four
per-section blocks affirmatively said "no `jugglebot_interfaces` rebuild", and the
§ Build gate enumerated the exceptions — but underneath those, **every one of the
runbook's build instructions was the single-package command**. All of them were
swept: 20 sites now read
`colcon build --packages-select jugglebot_interfaces jugglebot`, and
`grep -c 'packages-select jugglebot\b'` returns **0**. The two-package build is
mandatory in every section, including the ones that change no interface.

**No firmware flash for this phase.** Config regeneration added three `constexpr`
to the delivered `hardware_config.h` in all three sketch trees; no sketch reads
them, so they compile out. (The Platform Teensy flash owed by Phase D / § CHECK
HAND-7 is separate and still outstanding, and § SECTION CONT gates its >0.6 m
rungs on `platform_fw_version = 2`.)

## Open questions

1. **The unprotected pre-`try` region of `_execute_toss_continuous`** — see
   Discussion. Structurally real, reachability unproven, pre-existing in
   `_execute_toss`. Needs `session is None` guards on the `except`/`finally`, not
   a one-line move.
2. **No test pins `stop_on_miss` on the FINAL cycle** — the ordering is
   deliberate; the coverage gap is real but narrow.
3. **`tests/ros/conftest.py`'s `_TossGoal` still carries `flight_time_s`**, which
   the real `Toss.action` renamed to `throw_height_m` on 2026-07-25. Currently
   harmless (`Toss.Goal()` has zero call sites in the test tree) but latent.
4. **Session feedback during a dwell publishes at the full 20 Hz** (~60 messages
   per 8 s dwell). Matches the single Toss's rate; throttling only the DWELL-phase
   publish is a one-line change with no behavioural coupling if the operator finds
   the stream noisy.
5. **Three pre-existing ragged markdown table rows in `tools/probes/README.md`**
   (`levelling_tilt_bag_check.py`, `catch_reach_replay.py`, `traj_stream_probe.py`)
   — unescaped `|` inside cells, present at HEAD, outside this phase's scope.
