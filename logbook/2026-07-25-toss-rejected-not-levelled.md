---
title: The toss refuses an un-levelled launch — by observing the node that applies the correction, not the Teensy flag that outlives it
type: feature
date: 2026-07-25
status: in-progress
phase: "Self-toss anomaly fixes — levelling-frame-contract Phase 3"
related_plan: "levelling-frame-contract.md"
files_changed:
  - ros_ws/src/jugglebot_interfaces/msg/TrajectoryStatus.msg
  - ros_ws/src/jugglebot_interfaces/action/Toss.action
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/docs/levelling_frame.md
  - tests/ros/conftest.py
  - tests/ros/test_trajectory_node.py
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_toss_coordinator.py
  - tests/hardware/toss_trace_recorder.py
  - tests/hardware/session_anomaly_fixes.md
  - tests/hardware/session_phase8_toss_hardware.md
  - tests/hardware/session_phase8_toss_trace.md
  - plans/parked/levelling-frame-contract.md
commits:
  - e36d60d
subsystem:
  - ros
  - tools
tags:
  - safety
  - testing
  - docs
---

# The toss refuses an un-levelled launch — by observing the node that applies the correction, not the Teensy flag that outlives it

## Summary

The toss's CHECKING phase now refuses with `REJECTED_NOT_LEVELLED` unless
`trajectory_node` is *currently saying* it holds a gravity-levelling correction.
That is a deliberate departure from the plan's own step 3, which specified
gating on `RobotState.levelling_complete`: that flag is persisted on the Teensy
**per boot**, so it still reads `true` after a relaunch has emptied the ROS
node's in-memory correction — the gate would have passed in exactly the state it
exists to refuse. `TrajectoryStatus` therefore gains
`bool gravity_correction_loaded`, published at 5 Hz by the node that applies the
correction; the coordinator stamps every status arrival and feeds
`platform_levelled = status_fresh AND loaded` into `TossObservations`. Contract
**C-LEVEL-1** gains an observability half, **C-LEVEL-1.O**, in
`ros_ws/docs/levelling_frame.md`.

## Motivation

Phase 0's Table C found that the levelling correction has no observability at
all. Concretely, all three legs of its delivery are hostile:

| Leg | Reality |
|---|---|
| publish | `/gravity_offset` is **VOLATILE**, not transient-local |
| frequency | once per `level`, plus **one** latched auto-push on the first IDLE after orchestrator boot |
| storage | per-process, in memory, **no re-request path** |

So a `trajectory_node` that restarts after a `level` — a crash, or precisely the
`colcon build` + relaunch that any change to this package requires — silently
reverts to the identity correction while every nearby proxy keeps saying the
machine is levelled. Nothing downstream could tell the two apart.

The refusal itself is justified **geometrically, not procedurally**. Un-levelled,
the launch leaves the cup 0.78° off gravity (the 2026-07-25 measured offset). A
vertical toss drifts `v·sin θ·T`, which with `T = 2v/g` and `v = sqrt(2gh)` is
exactly `4·h·sin θ`: **43 mm** at the ~0.79 m config-default apex, against a
`GEOM_HAND_RADIUS_MM = 35.0` cup. The catch is impossible before the ball leaves
the hand, so the throw refuses rather than putting a ball on the floor — the same
class of loud-early reject as `REJECTED_HAND_NOT_PARKED`.

## Discussion

### Why the plan's step 3 was overridden

The plan said to gate on `RobotState.levelling_complete`. That premise was
checked against ground truth rather than trusted:
`orchestrator_node._on_robot_state` assigns `ctx.levelling_complete =
msg.levelling_complete` straight from a Teensy-persisted per-boot flag. It says
nothing about any ROS process's memory. Wiring the gate to it would have
produced **false assurance**, which is strictly worse than the no-gate status
quo: the operator would read an all-clear and throw a 0.78°-off ball.

Observing the *applier* is the only reading that cannot lie, and it is correct
regardless of *how* the correction went missing — never published, discovery
race, malformed message, node restart, orchestrator never reaching IDLE. It also
handles a case the plan's wiring gets backwards: if LEVELLING FAULTs at
`level_mocap_check`, which runs *after* `level_send_correction`, the correction
**is** loaded but `levelling_complete` is never persisted. The shipped gate
passes that machine; the plan-as-written gate would have refused one that is
genuinely in the gravity frame.

### Three predicates that look right and are not

Each is ruled out in the contract and pinned by a test, because each fails in a
direction that gets the gate bypassed or gets a ball dropped:

- **`levelling_complete`** — passes after a relaunch. False assurance.
- **`R != I`** — a genuinely level machine measures a zero tilt whose correction
  *is* the identity. This predicate refuses a correct machine forever, and a
  gate that cries wolf gets bypassed.
- **A sticky cached `True`** — between a `trajectory_node` dying and its
  replacement's first status there is no message to flip a consumer's cache, so
  a consumer without an expiry answers with the dead process's frame. Hence the
  freshness half, and hence a 5 Hz publication rather than a latch.

A fourth was added at finalize (see *Non-finite offsets*, below).

### Transient-local QoS: considered, rejected

Candidate (a) in the plan was to make `/gravity_offset` transient-local so a
restarting subscriber gets the last value redelivered. Three failure modes
killed it, in order of weight:

1. **It does not help the case that motivates the hazard.** A transient-local
   latch lives in the **publisher**. The relaunch this plan's own deployment
   mandates restarts `orchestrator_node` too, so there is nothing latched to
   redeliver. It would only help a manual partial restart of `trajectory_node`
   while the orchestrator survives, and the launch has no `respawn`.
2. **It creates a silent DDS-level failure of exactly the class this contract
   exists to close.** A TRANSIENT_LOCAL subscriber is incompatible with a
   VOLATILE publisher (requested durability exceeds offered) — the connection is
   simply never made, with no error and no warning. Any hand-run
   `ros2 topic pub /gravity_offset …` without `--qos-durability
   transient_local`, or a second publisher added later, would be silently
   ignored.
3. **It closes nothing on its own.** It lowers the frequency of the blind state
   without making it observable, so a gate built on it is still guessing.

### The freshness window is measured, not copied

Every sibling window in `reload_coordinator_node` (`_MOCAP_STALE_S`,
`_HAND_STATE_STALE_S`, `_STATUS_STALE_S`) is 0.5 s, sized for 100–160 Hz
sources. `trajectory/status` is **5 Hz**, so 0.5 s is 2.5 periods. Measured
inter-arrival over the two 2026-07-25 reference bags (`_15-17-48` 290.6 s,
`_15-22-50` 129.8 s; probe run 2026-07-26, independently re-measured at
finalize): **median 200.0 ms, p99 210.3 / 204.0 ms, max 508.5 ms**, with one gap
already past 0.5 s and four past 0.3 s (two per bag: 384.5 / 508.5 ms and
334.5 / 426.2 ms), and **zero** past 1.0 s. A copied 0.5 s would therefore have
minted occasional spurious refusals on a healthy machine.
`_TRAJ_STATUS_STALE_S = 1.0` is five periods and ~2× the worst observed gap.
The *discipline* is matched exactly to `hand_fresh` (perf_counter stamp in the
callback, `stamp > 0.0 and now - stamp < window` outside the lock); only the
value differs, for a measured reason.

### Naming: `gravity_correction_loaded`, not `levelled`

Calling the wire field `levelled` is one step from a future reader wiring it to
— or from — `RobotState.levelling_complete`, which is precisely the conflation
of the third pair of meanings of "level" that this phase exists to break. The
field name states what is actually observed: the applier holds a correction.

### A hypothesis that was withdrawn at finalize

The implementer's account, the plan Outcome and a production comment all stated
that a half-rebuild (`jugglebot` without `jugglebot_interfaces`) makes
`trajectory_node`'s status publish "raise at 5 Hz, the topic go silent, and
every toss refuse". Two reviewers independently disputed the mechanism and both
were right. Traced: `_publish_status` has no `try/except`; rclpy's
`SingleThreadedExecutor.spin_once` ends `handler(); if handler.exception() is
not None: raise handler.exception()`
(`/opt/ros/foxy/lib/python3.8/site-packages/rclpy/executors.py:708-719`); and
`trajectory_node.main` wraps `rclpy.spin(node)` in `except KeyboardInterrupt`
only. The **process exits** ~200 ms after launch. The operator therefore never
sees a refused toss at all — they see no `trajectory_node` in `ros2 node list`,
no 40 Hz emitter, and `activate` failing at the A2 arm for want of an mpccmd
frame. Documenting the softer symptom would have sent the operator hunting the
levelling gate on a signature the gate cannot produce. All three sites are
corrected, and LG-0 now checks `ros2 node list` first.

The *design* decision that comment justified — read `msg.gravity_correction_loaded`
as a plain attribute, never `getattr(…, False)` — survives, but its stated
reason had to change too: the same-install case never reaches the coordinator,
so the reachable justification is a genuine build split between two publishers,
which must fail loudly rather than be papered over into a permanent unexplained
`NOT_LEVELLED`.

### Non-finite offsets (added at finalize)

`levelling.correction_from_offset` does no finiteness validation — it negates and
builds a rotation. A `[nan, nan]` offset would therefore be *stored*, the flag
would read `True`, CHECKING would pass, and the NaN would surface only
downstream as a POSITIONING feasibility rejection — **after** the goal had
claimed the platform. The `len(msg.data) < 2` guard was extended to non-finite
values, with an error log, so `test_malformed_offset_does_not_count_as_loaded`
means what its name says. This is the fourth wrong predicate: *affirming a frame
you cannot use*.

### Deliberately not fixed: a dead `trajectory_node` reports the wrong subsystem

`streaming` is a sticky last-value with no freshness stamp, so when
`trajectory_node` dies outright `REJECTED_NOT_STREAMING` cannot fire and the
fault surfaces here instead, as `REJECTED_NOT_LEVELLED`. Before this phase the
same fault surfaced later as `REJECTED_POSITION(NO_RESPONSE)`, which named the
right node — so this is a genuine diagnostic regression, raised by a reviewer,
verified, and deferred **for a reason that is not scope-protection**.

The proposed fix — a toss-only `traj_status_fresh` observation with its own
reject code — correctly avoids touching `streaming`. But it would give one node
**two independent representations of "is `trajectory_node` alive"**: `streaming`
(consumed by the reload FSM, un-expiring) and a parallel toss-only stamp, which
answer differently for the same fault. That is the one-enforcement-point
violation this contract pattern exists to prevent, and it is how a later correct
fix to `streaming` leaves a stale duplicate behind. The single-enforcement-point
fix is one freshness stamp on `streaming` itself, which changes **when a reload
refuses** — a safety-relevant behaviour change, and its own small phase. The
consequential half is identical under either code (no motion, no arming, nothing
dropped); only the routing differs, and the routing is now documented at three
levels: operator pre-brief item 4 (with the two commands that distinguish the
causes), `REJECT_WIRE_MAP`'s entry naming both causes, and the contract's
Enforcement paragraph.

## Design

- **Producer.** `trajectory_node._on_gravity_offset` is the sole writer of both
  `_gravity_correction` and `_gravity_correction_loaded`, and sets the flag
  *after* the store, behind the malformed-message guard. `_publish_status`
  publishes the **boolean, never the matrix** — a consumer handed the matrix
  would be one step from re-applying it, the mirror bug C-LEVEL-1 forbids.
- **Consumer.** `reload_coordinator_node._on_traj_status` caches the flag and a
  `time.perf_counter()` arrival stamp under the existing lock;
  `_build_toss_observations` computes `traj_status_fresh and correction_loaded`
  outside it. `_traj_status_mono` initialises to `0.0` and the builder requires
  `stamp > 0.0`, so silence fails **closed**.
- **Observation shape.** A plain `platform_levelled: bool` on
  `TossObservations`, not a staleness-aware struct. The staleness lives where the
  stamp lives, exactly like `hand_fresh`/`hand_parked`; pushing a struct into the
  pure-Python FSM would give it a second opinion about ROS timing it cannot
  verify, and its default-`False` is the fail-closed answer either way.
- **Gate placement.** `toss_sequencer._step_checking`, after
  `mocap_fresh`/`streaming` (a stale graph makes the flag *unknowable*, not
  false) and before the hand-evidence chain (those gates describe *how* the
  throw is dispatched; this one says the throw cannot be caught wherever it is
  dispatched from).

## Implementation

`obs.platform_levelled` is read at **exactly one** site in production code —
`toss_sequencer.py`'s `_step_checking`, which the FSM enters exactly once. A
status hiccup mid-sequence therefore cannot abort a flight, cannot terminate
CATCHING early, and cannot trigger a retract into an incoming ball (the
2026-07-23 hardware hazard).

**No commanded-motion magnitude, feedforward term or loop timing changed.** On a
refusal `_terminal_action` returns `ACTION_NONE`, because `_positioned` and
`_prepare_dispatched` are both `False` — no `go_to_pose`, no `arm_catch` latch,
no announcement, no `SetHandTrajCmd`, no SAFE_ABORT/kind-3 retract. The net
effect on the machine is strictly **less** motion than before. Hot-loop cost is
one bool assignment in the existing 5 Hz `_publish_status`; the 40 Hz emitter and
the 500 Hz `motor_guard` are untouched.

`TossObservations` has exactly two construction sites, both all-keyword, so
inserting the field mid-dataclass is safe.

## Verification

Full suite, run 2026-07-26 on this Jetson in the project venv:
`pytest tests/ -q` → **3569 passed, 3 xfailed, 198 warnings in 1399.52s
(0:23:19)**, exit 0.
Baseline at HEAD `b9fd45e` was 3543 passed, 3 xfailed in 1376.13s, so the delta
is **+26 passed**, accounted for exactly by the new cases: `+8` in
`test_toss_sequencer.py` (1 parametrize row on `test_precondition_rejects`, 6
ordering cases, 1 proceeds-when-levelled), `+7` in `test_trajectory_node.py`
(no-correction, zero-offset, a 4-row malformed parametrization, restart), and
`+11` in `test_toss_coordinator.py` (2 rows on
`test_toss_goal_rejections_via_execute`, a 5-row freshness table, and 4
single-case tests). Nothing else moved, and neither known order/load-flaky
allocation-budget test failed in this run.
**xfail count unchanged at 3** — no test was weakened, skipped or deleted; the
only pre-existing tests edited are two shared healthy-state fixtures (`_obs`,
`_toss_ready_node`/`_stamp_fresh`) which gained the new precondition so that
"everything satisfied" still means everything, plus docstring counts three → four.

Scoped confirmation after the last code edit, same day and venv:
`pytest tests/ros/test_trajectory_node.py tests/ros/test_toss_coordinator.py
tests/ros/test_toss_sequencer.py tests/ros/test_levelling_frame.py
tests/motion/test_levelling.py -q` → **367 passed in 21.67s**.

The new cases cover: the gate fires and sits between the graph gates and the
hand chain (6 ordering cases); a levelled machine proceeds to POSITIONING; the
applier reports `False` until an offset arrives, `True` for a **zero** offset,
`False` for a short **or non-finite** one (4 rows), and `False` after a restart
even with `robot_state` fed in; the coordinator's 5-row freshness table (the
0.5 s row is the measured 508.5 ms worst gap turned into a test); fail-closed on
never-heard; the orchestrator's persisted auto-push alone satisfying the gate,
driven through the **real producer's** `_publish_status` output into the real
consumer callback; and the restart flipping the gate end-to-end through
`_execute_toss`.

**The LG-5 analysis command was validated in both directions** before it shipped
(2026-07-26): four synthetic captures in the recorder's own JSONL schema — the
clean `False → REJECTED_NOT_LEVELLED → flip → MISSED` shape it must ACCEPT
(separations `+7.200 s` / `+12.800 s`), plus never-flips, `True`-before-`level`
and a 2 ms flip/goal separation, each of which it must FLAG. Two decoy `/rosout`
lines (the coordinator's own `Toss early exit while prepared …`, and a `Toss …`
line from another node) are correctly excluded. That validation changed the
shipped reader: it now uses the recorder's own `OUTCOME_RE` plus the
`reload_coordinator_node` filter instead of a bare `startswith('Toss ')`, and
prints the two separations as **numbers** so the PASS/AMBIGUOUS boundary is
measured rather than eyeballed.

## Outcome

Software-complete; **hardware validation deferred to the operator** —
`tests/hardware/session_anomaly_fixes.md` § Section LVLGATE, checks LG-0…LG-5.
LG-3 is the check the phase turns on (`levelling_complete: true` **and**
`gravity_correction_loaded: false` **and** `REJECTED_NOT_LEVELLED`, together);
LG-5 is the only check that exercises the cross-process ordering that every test
here is blind to by construction.

**Deployment: `colcon build --packages-select jugglebot_interfaces jugglebot`
+ relaunch.** This is the one phase of the anomaly-fix run that needs the
interfaces package. No firmware flash, no config regeneration. Building only
`jugglebot` kills `trajectory_node` outright (see Discussion).

Falsified prose in three sibling runbooks was swept in the same commit: both
Phase-8 toss runbooks gained the now-mandatory `level` step (without it every
toss in either file refuses, and `session_phase8_toss_trace.md`'s Capture R
would fail checker RJ-1 with the wrong code), and RJ-1's own gate enumeration
gained "levelling". `/trajectory/status` was added to the shared § Recording
list, because LG-4's decisive diagnostic cannot be reconstructed after the fact
if the topic was never in the bag.

### Follow-ups (out of scope, recorded)

- **`streaming` has no freshness stamp.** See Discussion. One stamp plus a
  decision about the reload FSM's semantics; its own small phase.
- **`REJECT_WIRE_MAP` is missing five codes** the FSM can mint
  (`REJECTED_NO_BALL`, `REJECTED_DISPLACEMENT`, `REJECTED_TILT_CLAMP`,
  `REJECTED_BAD_GOAL`, `REJECTED_POSITION`), with no drift guard because nothing
  in `tests/` imports the recorder. The module imports pure stdlib at top level,
  so a `test_every_toss_reject_code_has_an_operator_cause` guard is cheap.
- **No drift guard** between `tests/ros/conftest.py`'s mock message dataclasses
  and the real `.msg` files (general, ~40 messages). Both were updated by hand
  and the real generated message's field list verified. The risk is
  self-revealing for an *added* field; a removed or renamed one could drift
  silently.
- **`mpc_bridge_node`** holds a second copy of the correction with no
  observability. It is dropped from the launch and reaches `planner` zero times,
  so nothing gates on it; noted in the contract's observability row.
