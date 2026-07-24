---
title: "Single-ball toss Phase 1: Toss.action + toss_sequencer FSM + ball-ops coordinator (Tier 8a)"
type: feature
date: 2026-07-25
status: resolved
phase: "MVP trajectory bringup — Phase 8 / single-ball toss Phase 1 (action + sequencer + coordinator)"
related_plan: single-ball-toss.md
subsystem: ros
tags: [feature, control, testing]
commits:
  - PENDING
files_changed:
  - ros_ws/src/jugglebot_interfaces/action/Toss.action
  - ros_ws/src/jugglebot_interfaces/CMakeLists.txt
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/toss_release.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - config/hardware_config.yaml (+ regenerated artifacts)
  - tests/ros/conftest.py
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_toss_integration.py
  - tests/ros/test_catch_coordinator_node.py
  - tests/motion/test_toss_release.py
---

# Single-ball toss Phase 1: Toss.action + toss_sequencer + ball-ops coordinator

> **ros_ws changes** — running any of this on the robot requires
> `colcon build --packages-select jugglebot_interfaces jugglebot` and a
> relaunch (the launch runs the installed copy; the new `Toss.action` needs
> the interfaces package rebuilt).

## Summary

Phase 1 of `plans/active/single-ball-toss.md` lands software-complete: the
`Toss.action` interface (goal/result/feedback exactly as the plan's locked
spec), the pure-Python `toss_sequencer` FSM (7 phases, 65 tests), the
release-state math module `motion/trajectory/toss_release.py` (the single
STOW→global conversion point + the full ballistic inverse), and the
coordinator — `reload_coordinator_node` extended into a two-action ball-ops
node (measured shared surface 76.5% > the plan's 70% threshold) with a
cross-action busy gate, mocap-augmented positioning, an ordering-hardened
PREPARE bundle, a single-shot tri-state telemetry-verified throw dispatch,
and the new `catch/prime_hold` suppression gate in `catch_coordinator_node`.
Tier 8b returns `REJECTED_TIER` (config-selected; see Discussion). The
adversarial review panel caught one BLOCKING frame bug the unit tests had
enshrined (details below) plus three convergent WARNING races; all were
fixed-and-landed pre-commit. Full `tests/ros/` scope: 1157 passed.

Built during the 2026-07-25 unattended workflow-orchestrated run
(operator-authorized 2026-07-24). Design artifacts (spine maps, control
analysis, FSM spec, hosting measurement, decision record) live in the run's
scratchpad; their load-bearing conclusions are recorded here.

## Changes

- **`Toss.action`** (`jugglebot_interfaces`): goal `catch_position`
  (STOW-relative platform pose, `TimedTarget.pose` convention),
  `flight_time_s`, `throw_delay_s`, `catch_vel_scale`; result `success`,
  `outcome`, `catch_error_mm`, `achieved_flight_s`; feedback `phase`
  (CHECKING | POSITIONING | PREPARING | THROWING | BALL_IN_FLIGHT |
  CATCHING | SETTLING). Field lines byte-verified against the plan block;
  registered in the interfaces CMake stanza next to `Reload.action`.
- **`motion/trajectory/toss_release.py`**: `stow_to_global_mm` — THE one
  conversion point (docstring cites `DynamicTargetCommand.msg` as the
  normative frame declaration and the 2026-07-23 z double-add lesson);
  `HAND_THROW_OFFSET_MM = 58.044` (named, provenance:
  `GEOM_HAND_AXIS_BOTTOM_OFFSET_MM` −129.0 + `HAND_THROW_POS_M` 0.187044);
  `compute_release_state`; full release-plane→catch-plane launch velocity
  via `ballistics_bc` (GRAVITY_MMS2 9806.0; T=0.8 s ⇒ 3930.82 mm/s, apex
  787.85 mm; idealized g·T/2 = 3922.4 identity pinned);
  `build_announcement_fields` (`target_id='jugglebot'` mandate). 18 pin
  tests incl. `compute_catch_point_mm` parity at 809.08 and the
  9806-vs-9810 gravity guard.
- **`toss_sequencer.py`**: pure dataclass FSM mirroring
  `reload_sequencer`'s structure; CHECKING rejects incl.
  `REJECTED_HAND_NOT_PARKED`, `REJECTED_HAND_STALE`,
  `REJECTED_TRACK_ACTIVE`, `REJECTED_TIER` (the
  `REJECTED_BAD_GOAL(<field>)` finiteness/negativity guards are
  node-level, pre-FSM, in the coordinator); positioning with timed arrival + verify
  window; PREPARE → gap → ANNOUNCE → single-shot tri-state dispatch;
  hand-parked re-verified at THROWING entry; in-flight cancel deferred to
  phase boundaries; settle anchored on landing time. 65 tests in the
  per-phase abort-enumeration style.
- **`reload_coordinator_node.py`** (kept filename; now the ball-ops node):
  second ActionServer `jugglebot/toss`; goal claim under `_lock` in the
  shared accept callback (cross-action `REJECTED_BUSY`, closes the
  accept→install race for Reload too); toss observation builder (hand
  freshness/park band, sticky ball-possession latch, phantom-track
  exclusion snapshotted at goal start, release-evidence latches gated on
  the throw dispatch, ROS→perf crossings); `go_to_pose` positioning with
  `[wire DISARMED` mapping and best-effort `go_home` on ack-unknown
  terminals; PREPARE ordering: `prime_hold` raised ALONE one tick before
  the bundle → soft catch gains → `arm_catch` confirm → `vel_scale` →
  `prime_dispatched` belt stamp → `catch/armed` → next-tick
  self-`ThrowAnnouncement` (`thrower_name='jugglebot'`,
  `target_id='jugglebot'`); throw dispatch = one `SetHandTrajCmd`
  `traj_type=0` with `event_delay` recomputed from absolute release time,
  outcome read from hand telemetry (never re-dispatched); terminals reuse
  reload's teardown with `prime_hold` released last; execute loop
  exception-safed (`ABORTED_EXCEPTION` after `_safe_toss_on_early_exit`);
  trace-only ball-evidence waiver as a node parameter (waives possession
  only; park-band and freshness stay hard).
- **`catch_coordinator_node.py`**: `catch/prime_hold` Bool subscription
  gating ONLY the two hand-prime dispatch paths (armed-edge prime, retry
  tick); catch arm untouched; absent-topic behaviour bit-identical
  (44 pre-existing tests pass unmodified; 5 new).
- **Config**: `toss_tier` '8a' → `JB_OP_TOSS_TIER`;
  `toss_flight_time_default_s` 0.8 → `JB_OP_TOSS_FLIGHT_TIME_DEFAULT_S`
  (passed into the FSM; module literal is the documented no-config
  fallback); `toss_release_latency_ms` 0.0 → reserved T0 slot (wired into
  `event_delay`; pinned at 0.0 by test). Artifacts regenerated via
  `config/generate_config.py`.
- **`tests/ros/conftest.py`**: Toss action mock registered exactly as
  Reload's; pre-existing drift fixed (`_ReloadGoal` lacked
  `catch_vel_scale`).

## Discussion

**Control-system analysis (mandated walk-through) — Tier 8a needs ZERO
`trajectory_node` changes.** The full-cycle walk (accept → CHECKING →
POSITIONING → PREPARING → THROWING → flight → CATCHING → SETTLING →
RECENTER/SAFE_ABORT, plus every abort seam) confirmed every surface the
toss crosses already exists: `go_to_pose` (first live client), the
`arm_catch` latch (raise deferred until verified arrival because the raise
edge C2-stops in-flight moves and captures the 80 mm reach-envelope center
from the *current* commanded pose), latch-gated `dynamic_target`, and
`go_home`. The platform command sources stay temporally disjoint
(positioning before the latch; tracker-driven catch targets only while
armed; open-loop hold through flight). Firmware ground truth dissolved two
feared seams: the throw stroke commands absolute positions 0 → release
5.914 → end 9.959 rev, i.e. it *starts* where a completed catch leaves the
hand (bottom — no pre-move needed) and *ends* at the catch-prime position
(no post-release prime ascent needed; the 0.7–1.05 s prime vs 0.55–1.1 s
flight tension the maps flagged does not exist).

**The one genuinely new hazard was in `catch_coordinator`, not
`trajectory_node`.** All three design agents independently converged on it:
the hardware-proven catch path primes the hand to top on the `catch/armed`
rising edge and re-primes from a 0.5 s retry tick (the ack lies ~59%, so
`_hand_primed` often stays unlatched). During a toss PREPARE the ball sits
in the cup at stroke *bottom* — an auto-prime would smooth-move the
ball-laden hand, and a post-dispatch re-prime would clobber the armed
kind-0 throw stroke on the last-writer-wins Teensy queue. Two mitigations
were designed: (A1) an explicit `catch/prime_hold` gate in
`catch_coordinator_node`; (A2) repurposing the `catch/prime_dispatched`
stamp as a ≤1.0 s heartbeat riding the existing 1.2 s in-flight window.
A1 was chosen *by failure mode*: a heartbeat gap >1.2 s (blocking service
waits, executor stall) primes the ball-laden hand silently and
load-dependently, and couples the toss to an internal window constant that
can be retuned independently; A1 is additive, default-off (reload path
bit-identical), fails safe when stale (no auto-prime; reload primes
proactively itself), and is pinned by tests. The review panel then added
the raise-side ordering hardening: `prime_hold` is published one FSM tick
*before* the bundle (same-wait-set reordering at the subscriber could
otherwise run the armed edge first), with a single `prime_dispatched` belt
stamp before `catch/armed` — reload's proven suppressor — covering a
lost/reordered hold. No periodic re-stamping: A2's failure mode is not
re-imported.

**Hosting: extend, not a new node.** Measured shared surface 13/17 items =
76.5% (>70% ⇒ the plan's own threshold says extend). The deeper reason:
Reload and Toss must be mutually exclusive (both own the latch, the hand,
and the platform), and cross-node exclusion has TOCTOU races whose
physical cost is a hand-queue clobber; in one node the busy gate is a
`_lock`-claimed flag in the shared accept callback — which also closed a
pre-existing accept→install race on the reload side. The telemetry-ladder
thresholds (bag-probed) stay single-copy. Cost accepted: one node crash
kills both actions; two servers share the executor (the RCN executor
discipline comment is preserved and both servers share the one reentrant
group).

**Thrower-agnosticism — verified, with a sharper invariant than the plan
asked for.** `thrower_name` appears nowhere in the catch path; the actual
gates are `target_id == 'jugglebot'` (announcement pre-tilt and ball
destination filters). A self-announcement with an empty `target_id` would
be *silently* uncatchable. The integration test drives the node's real
published announcement through the real `BallTracker` + `CatchCoordinator`
to an accepted catch command — pinning both the agnosticism and the
`target_id` mandate.

**Tiering is config-only — a plan tension, documented not papered over.**
The plan says "tier parameter (config default + per-goal override)", but
the locked `Toss.action` goal has no tier field and altering the goal is a
run stop-condition. `JB_OP_TOSS_TIER='8a'` selects capability;
`REJECTED_TIER` fires iff the config selects an unimplemented tier. Every
locked goal is Tier-8a-serviceable (8a pre-positions to any catch (x,y)),
so nothing is lost in Phase 1; Phase 4 implements 8b behind the same key.
The per-goal override needs either a goal field (operator decision) or a
plan-text amendment.

**What the adversarial review caught (and the lesson).** The BLOCKING
finding: the mocap arrival cross-check was written against a QTM rigid
body named `'jugglebot'` that does not exist in the stream (known bodies:
Base, Ball_Butler, Catching_Cone), and compared a global-converted target
against a measurement that `mocap_interface` publishes z-shifted into the
platform_start frame — the z double-add class again, re-introduced in the
*verification* path that existed to prevent it, and the unit tests
fabricated the measured pose in the same wrong frame so everything passed.
Every hardware toss would have died `ABORTED_POSITION_FAILED`. The fix
re-scopes D7: the cross-check is config-keyed (`toss_mocap_body`, default
disabled — arrival is then ACK + planned-duration + wire-disarmed
mapping), compares in platform_start frame when enabled, and the real
platform body name/frame must be resolved on the bench (Phase-3 dry trace
runbook item) before enabling. Lesson recorded: a verification path is
itself code that can carry the exact bug class it guards against, and
tests written by the same hand enshrine it — the independent-lens panel is
what caught it. Convergent WARNINGs also fixed: phantom-track poisoning
(pre-existing untagged CONFIRMED tracks could destroy the possession latch
or pre-arm release evidence — snapshot at goal start + dispatch-gated
latches), non-finite goal numerics detonating *after* arming (finiteness
rejects + exception-safed execute loop → `ABORTED_EXCEPTION`), and the
stroke-signature threshold raised 15→40 rev/s (the kind-1 smooth-move
prelude peaks ~31.4 rev/s and would have read as release evidence).

**Deliberate deviations from the reload mirror** (each pinned by a test
saying why): announcement/feasibility notes gate on PREPARED, not
`_throw_sent` (the pre-tilt acceptance precedes the throw in a toss;
reload's gating would mint false `MISSED_INFEASIBLE` verdicts); in-flight
cancel defers to the next phase boundary (aborting a catch mid-flight
drops a ball on the robot); `catch_error_mm` is NaN on all non-CAUGHT
terminals.

**Known limitations / follow-ups (deliberate, documented):**
- **Tracker liveness is not a CHECKING precondition** — a dead
  `ball_tracker_node` passes every gate and guarantees an uncaught throw
  (mocap freshness comes from a different process). Session runbooks must
  verify tracker liveness; a tracker heartbeat is the structural fix.
- **`reload_coordinator`'s own immediate-cancel honours a cancel
  mid-flight** (retract under an airborne ball) — pre-existing hazard now
  thrown into relief by the toss's deferred-cancel design; left unchanged
  this run (operator-facing semantics of a hardware-proven path); named
  follow-up.
- **Firmware kind-1 time-budget race at short flights**: the catch arm
  arriving mid-throw-decel is silently dropped by the Teensy windup budget
  for flight ≤~0.6 s at realistic pipeline latencies — hardware T1 floor
  of 0.7 s recommended (plan risk register updated); the sim gate keeps
  the plan's 0.55 s sweep floor (sim does not model the firmware budget)
  and Phase 2's report will annotate the hardware-marginal band.
- `_THROW_STROKE_VEL_RPS=40.0` and `MIN_THROW_EVENT_DELAY_S=1.0` are
  derived, not probe-measured — T0 re-tunes both;
  `toss_release_latency_ms` ships 0.0 (T0 measures);
  `achieved_flight_s` therefore contains the unmeasured JB
  command→release latency until then.
- The nominated catch z shifts the real catch plane while the tracker
  predicts at its fixed 809.08 plane — ~10 ms-class arm-timing skew at
  ±30 mm, absorbed by the 0.75 s arm window; Phase 2 quantifies.
- The possession latch can read a bounced-out ball as still-seated
  (no ball-held sensor yet); accepted until the sensor era.
- The integration test's announcement-field mapping mirrors
  `ball_tracker_node._on_announcement` rather than driving it (reload
  precedent; drift risk accepted); the corner-pose `build_catch`
  wire-crossing test lands with Phase 2's gate harness.

## Verification

- Scoped (2026-07-25, all on the Jetson venv, post-fix):
  `pytest tests/ros/ -q` → **1157 passed in 56.16 s** (includes 65
  toss_sequencer + 72 toss_coordinator + 5 toss_integration + 5
  prime_hold tests); `pytest tests/motion/ -q` → **737 passed** (18 new
  toss_release pins); reload/catch suites green untouched except the two
  documented shared-surface changes (busy-gate claim, conftest drift fix).
- The D8 reject-classification tests drive the REAL
  `teensy_bridge_node._svc_set_hand_traj` validation path (the
  empirical-probe rule applied in-test; no restated strings).
- Full suite (phase gate) — `pytest tests/ -q`, run 2026-07-25 against the
  final post-audit tree: **3318 passed, 3 xfailed, 198 warnings in
  1291.03 s** (+165 tests over the 3153 Phase-0 baseline; same 3 xfails).
  An earlier full run of the pre-audit-fix tree also passed 3318/3
  (1471.46 s) — the gate was re-run because the audit fixes touched
  `toss_sequencer.py` after it.
- `/audit` (pre-commit, 2026-07-25): no BLOCKING; 3 WARNINGs — stale
  per-file test counts in this entry (63/53/4 → 65/72/5, collected), the
  INDEX row pointing at then-unfilled placeholders, and the `Toss.action`
  header still claiming unconditional mocap-verified arrival after the
  D7 re-scope — plus 2 NOTEs: `REJECTED_BAD_GOAL` mis-attributed to the
  FSM (it is node-level), and a real bounded code defect — a pre-dispatch
  phantom ball's landing-crossing estimate could contaminate the
  `achieved_flight_s` diagnostic (`_last_time_at_land` now refreshes only
  once our throw is dispatched). All five fixed-and-landed pre-commit per
  the autonomous-runner policy. The audit independently re-verified the
  cross-document numbers (58.044, 3930.82, 787.85, 76.5%, 809.08, the
  31.4 rev/s prelude peak) and the safety invariants (single-shot
  dispatch, prime_hold never gates the catch arm, busy-gate release on
  every exit path, no second +574.3 outside `toss_release`).
