---
title: Throw director node + trimmed cone-test launch for live BB→cone integration
type: feature
date: 2026-05-23
status: tuned
phase: catching-cone-bringup
related_plan: ""
files_changed:
  # New
  - ros_ws/src/jugglebot/jugglebot/throw_director_node.py
  - ros_ws/src/jugglebot/launch/catching_cone_test.launch.py
  - ros_ws/src/jugglebot_interfaces/srv/ThrowAtTarget.srv
  - tests/ros/test_throw_ballistics.py
  - tests/ros/test_throw_director_node.py
  # Extended
  - ros_ws/src/jugglebot/jugglebot/can/throw_ballistics.py
  - ros_ws/src/jugglebot/jugglebot/can_node.py
  - ros_ws/src/jugglebot/setup.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot_interfaces/CMakeLists.txt
  - ros_ws/src/jugglebot_interfaces/srv/SendBallButlerCommand.srv
  - ros_ws/gui/index.html
  - ros_ws/gui/js/panels.js
  - ros_ws/gui/css/panels.css
  - tests/ros/conftest.py
  - tests/ros/test_can_node.py
  # Logbook
  - logbook/2026-05-23-throw-director-and-cone-live-integration.md
  - logbook/INDEX.md
commits:
  - 1a3e429  # feat(bb): add throw_director_node for aim-and-throw at named QTM targets
  - 455cf06  # fix(bb): solver release-z (pitch_z_offset) + correct ThrowAnnouncement for off-plane targets
subsystem:
  - ros
  - gui
  - can
tags:
  - feature
  - timing
  - testing
  - kinematics
  - hardware
---

# Throw director node + trimmed cone-test launch for live BB→cone integration

## Summary

Wired up the host-side aim-and-throw path required to drive the live
Ball-Butler→catching-cone timing experiment described in the "Live Jetson
integration" open question of
`2026-05-23-catching-cone-hardware-bringup.md`.  Brought
the inverse-ballistics solver back from `archived/ball_butler_node.py` as a
pure-Python module, added a new `throw_director_node` exposing
`bb/throw_at_target(target_name)`, gave the GUI a target dropdown + Throw
button, and added a trimmed `catching_cone_test.launch.py` that runs only
the nodes needed when Jugglebot itself is offline (the cone shares
Jugglebot's CAN cable).  First live session surfaced two real bugs — both
fixed in the second commit — plus a setup gotcha (mocap_node's "Base
alignment" gate silently blocks rigid-body publish if QTM frames are
unaligned).  Time-signal audit confirms the catch/throw correlation
pipeline already lives in a single Unix-epoch domain.

## Motivation

The cone hardware bringup landed a complete catch-detection pipeline
(cone → CAN → `can_node` → `cone/catch_event` → `catch_correlation_node`
→ `cone/timing_result` + GUI panel).  The "Live Jetson integration" open
question in that entry was "does the full pipeline work end-to-end with
a real throw?" — which requires a way for the operator to issue a
*targeted* throw and have the host publish a `ThrowAnnouncement` for the
correlation node to match against.

The operator's expected workflow was a `throw_to_target` command that
aims BB at a given landing state and aborts if infeasible.  That logic
existed in `archived/ball_butler_node.py` (`compute_command_for_target`,
`aim_and_throw`) but was retired with the rest of that node and never
landed on `refactor`.  Without it, the only way to fire a throw was the
low-level `bb/send_throw_command(yaw, pitch, speed, delay)` — manual
trigonometry every time, no feasibility check.

A secondary motivation: the existing `jugglebot_launch.py` starts the
full platform stack (motor_guard, mpc_bridge_node, catch_coordinator_node,
motion_bridge_node).  For sessions where the cone is plugged into
Jugglebot's CAN cable while the platform itself is powered down, those
nodes either error-loop or block waiting for hardware that's not on the
bus.  A trimmed launch makes the cone-only test setup a single command.

## Design

**New node, not an extension.**  The natural place to put inverse-ballistics
+ target-resolution is *not* `can_node`.  `can_node`'s remit is "pure CAN
transport" (see `controller/REFERENCE_LAYER_CONTRACT.md`'s peer convention
for the same idea on the reference layer), and the existing
`_publish_throw_announcement` already centralises ThrowAnnouncement
publication on every `bb/send_throw_command` call — so the director can
delegate the CAN-send + announce to that path and stay strictly host-side.
Concrete failure modes a separate node prevents: solver iteration inside
the 1 kHz CAN-poll executor's callback group; target-resolution logic
coupled to CAN transport when it should be coupled to mocap; and the
"add a second target type" extension going into a transport file instead
of a director.

**Target resolution: subscribe to `/rigid_body_poses`, not tf2.**  Mocap
node already publishes `RigidBodyPoses` at ~200 Hz with every tracked
body's name + pose.  The director caches the latest position per name
and looks up by name on each service call.  tf2 would add a transform
listener with its own buffer / async timing — overkill for one rigid
body, easy to extend (just add another lookup key) when more targets
appear.

**Service interface, not a topic.**  `bb/throw_at_target` is request/
response so the GUI gets immediate feedback on feasibility — yaw out of
range, target inside the s-offset circle, no pitch satisfies speed limits,
BB calibration not yet received, can_node down.  The response includes
the resolved yaw/pitch/speed/ToF for display, plus the target's global
and BB-local positions for debugging when a throw goes somewhere
unexpected.

**Inverse-ballistics solver: lifted from archived, parameterised.**
`solve_throw_local` takes BB geometry + operational limits as keyword
arguments, defaulting to the `hardware_config` constants
(`BB_GEOM_YAW_S_OFFSET_MM`, `BB_GEOM_PITCH_D_OFFSET_MM`,
`BB_GEOM_RELEASE_L_POSITION_MM`, `BB_GEOM_PITCH_MIN/MAX_DEG`,
`BB_GEOM_YAW_MIN/MAX_DEG`, `BB_OP_MAX_THROW_SPEED_MPS`,
`BB_OP_MAX_THROW_HEIGHT_M`).  This keeps `hardware_config` as the single
source of truth while still being unit-testable without a config-codegen
pass.  Algorithm: 2-D yaw geometry (independent of pitch / l / d), then
a 0.5°-step pitch search minimising horizontal landing velocity subject
to speed + peak-height limits.  Same logic as the archived node, just
pure-Python.  Raises `ValueError` with the named binding constraint on
infeasibility so the director can surface the actual reason in the
service response.

**No new clocks.**  The director does not stamp anything itself — every
timestamp originates in `can_node._publish_throw_announcement` (via
`self.get_clock().now()`) or in the cone's TimeSync-locked microsecond
counter.  The director is "stateless across time": each call resolves the
*current* target position and computes a fresh throw.

## Implementation

* **`ros_ws/src/jugglebot/jugglebot/can/throw_ballistics.py`** extended with:
  * `_wrap_pi(angle)` — angle wrap helper.
  * `yaw_solve_thetas(x, y, s)` — 2-D yaw geometry; returns
    `(t1, t2, chosen)` with NaN sentinel for no-solution.
  * `global_to_bb_local(x, y, z, bb_position_mm, yaw_offset_rad)` —
    world-frame → BB-local transform using the BB's calibrated origin
    and yaw offset.
  * `@dataclass ThrowSolution(yaw_rad, pitch_rad, speed_mps, tof_s,
    peak_height_mm)`.
  * `solve_throw_local(x, y, z, **limits)` — the solver itself.
* **`ros_ws/src/jugglebot_interfaces/srv/ThrowAtTarget.srv`** —
  `string target_name, float64 throw_delay_s` →
  `bool success, string message`, plus the resolved
  yaw/pitch/speed/ToF/delay and both global and BB-local target
  positions.  Registered in `CMakeLists.txt`.
* **`ros_ws/src/jugglebot/jugglebot/throw_director_node.py`** — new node.
  Subscriptions: `/rigid_body_poses` (cache by name),
  `/bb/calibration_result` (latched, transient-local QoS to match
  `mocap_node`).  Service server: `bb/throw_at_target`.  Service client:
  `bb/send_throw_command` (existing on can_node).  Handler order:
  calibration-present → target-known → world→BB-local →
  `solve_throw_local` → `bb/send_throw_command` async call → fill
  response.  Entry point registered in `setup.py`.
* **`ros_ws/src/jugglebot/launch/catching_cone_test.launch.py`** — new.
  Starts `can_node`, `mocap_node`, `catch_correlation_node`,
  `throw_director_node`, rosbridge, and a cone-aware `ros2 bag record`.
  Skips motor_guard, mpc_bridge, motion_bridge, catch_coordinator,
  spacemouse_handler, ball_tracker, orchestrator.
* **`ros_ws/src/jugglebot/launch/jugglebot_launch.py`** — added
  `/cone/catch_event`, `/cone/heartbeat`, `/cone/timing_result` to the
  production launch's bag list so cone topics are captured in normal
  sessions too.
* **GUI** — new "Throw Director" panel in `index.html`,
  `initThrowDirectorPanel()` in `panels.js` (calls `bb/throw_at_target`
  via `callService`), styles in `panels.css`.  Target dropdown is a
  static list — currently `['Catching_Cone']`; future targets get added
  to that array.
* **Tests** — `test_throw_ballistics.py` (18 tests: wrap-pi, yaw solver,
  global→local transform, end-to-end roundtrip via release-point
  projectile, infeasibility surfacing, model-mismatch documentation).
  `test_throw_director_node.py` (11 tests: pre-flight gates, typical
  success path with delegation to `bb/send_throw_command`,
  solver-aware ThrowAnnouncement publication, caller-specified delay,
  yaw-offset transform, infeasibility / can-node-down branches, cache
  update semantics).  `conftest.py` extended with
  `RigidBodyPoses`, `RigidBodyPose`, `ThrowAtTarget`, and
  `ReliabilityPolicy` mocks.

## Discussion

This entry's non-obvious decisions are concentrated below.  Each one was
either a refuted hypothesis or a tradeoff worth recording before it
fossilises into "the way it is."

### "I thought you had wired this up" — refuted hypothesis

The session opened with the operator recalling a previously-wired
`throw_to_target` command.  First check: `grep` across the working tree
for the symbol — nothing.  Wider check: `git log --all --oneline | grep
-iE "(throw_to_target|aim_and_throw|throw_at_target|target_throw|...)"`
— zero matches.  The logic was in `archived/ball_butler_node.py` but
never made it onto any branch under a callable name.  Stale recollection.

Why this matters as a discipline note: the operator's confidence in
"this exists" is not evidence.  Cheap to verify (one `git log`), and the
verification reframes the work from "find what's already there" to
"build it" — which changes the design space a lot.

### New node vs extension on can_node — concrete failure modes

The default move would have been to add the service handler directly to
`can_node`: it already has the BB calibration state (`_bb_position_mm`,
`_bb_yaw_offset_rad`), already publishes ThrowAnnouncement, already owns
the `bb/send_throw_command` service handler.  Concrete failure modes a
separate node prevents (the framing
`feedback_root_cause_not_authority.md` calls for):

* **Solver inside the 1 kHz CAN-poll executor.**  The solver is
  analytical so it's fast (~146 pitch iterations × ~30 floats =
  microseconds), but as a discipline the CAN-poll callback group should
  not host non-trivial computation.  A separate node uses a separate
  executor.
* **Coupling target-resolution to CAN transport.**  Adding a
  `/rigid_body_poses` subscription to `can_node` makes the file have two
  reasons to change (CAN protocol churn and mocap-pipeline changes).
  Splitting them means each file has one reason to change.
* **Future target sources.**  If the second target ever comes from
  estimated ball position (not mocap), or from a configured fixture
  position, the director is the natural place to add a source dispatch.
  In can_node it would be one if-branch among many CAN-id dispatches.

The cost was small — one new entry-point in `setup.py`, one extra
process in the launch.  The alternative (one if-branch in can_node) was
genuinely smaller in lines-of-code today; the win comes from where the
*next* change lands.

### Inverse solver vs predict_throw — model mismatch accepted, with a tripwire

The solver models the full BB serial chain: yaw axis → pitch axis
(offset `d`) → linear axis → release point (offset `s` lateral and
`l*cos(pitch)` forward + `l*sin(pitch)` up).  The forward predictor
(`predict_throw`, called by `can_node._publish_throw_announcement`) is a
point launch from `bb_position_mm` — it doesn't account for the serial
chain at all.

For typical throws, feeding solver output into `predict_throw` produces
a landing point ~100 mm short of the actual target and a ToF that's
correspondingly off by ~25 ms at 4 m/s.  This is not a bug; it's a
deliberately-tolerated model mismatch the production pipeline
compensates via `BB_OP_LANDING_TIME_OFFSET_MS = -120 ms` (a constant
offset applied to the BB throw delay so the *real* landing matches the
*predicted* landing time).

Two options were on the table:
1. Unify the models so predict_throw accounts for the serial chain.
2. Accept the asymmetry and document it.

Chose (2) for this commit because (a) the production pipeline already
absorbs the bias and the operator's existing tuning is calibrated against
it, (b) unifying the models is a separate refactor that should land
across simulator + tests + hardware in one go, and (c) the time-signal
audit (below) is unaffected — both models live in the same Unix epoch.

To keep (1) on the radar, `test_solve_throw_local_vs_predict_throw_has_serial_chain_bias`
asserts the bias is in `(50 mm, 200 mm)` — if a future change unifies
the models and the bias drops to near zero, the test fails and points
the next session at `BB_OP_LANDING_TIME_OFFSET_MS` as obsoleted.

### Time-signal pipeline — audited, no fix needed

Traced end-to-end:

| Stage | Source | Epoch |
|-------|--------|-------|
| TimeSync broadcast (host→Teensys) | `bus.py:112` `t = time.time()` | Unix |
| Cone catch_time CAN encoding | cone Teensy locks to TimeSync master | Unix |
| `_handle_catch_event` reconstruction | `can_node.py:1299` `host_now_us = int(time.time() * 1e6)` | Unix |
| `CatchEvent.catch_time` stamp | reconstructed → `rclpy.time.Time(nanoseconds=catch_us * 1000)` | Unix ns |
| `ThrowAnnouncement.landing_time` | `self.get_clock().now() + Duration(delay) + Duration(tof)` | Unix ns (CLOCK_REALTIME in rclpy Foxy, unless `use_sim_time` is set) |
| Correlation arithmetic | `catch_ns - landing_ns` in `catch_correlation_node._on_catch` | Unix ns |

All three timestamp producers and the consumer share the Unix epoch.
The director introduces no new clocks — it delegates timestamp
production to can_node which delegates to `bus.broadcast_time` /
`get_clock().now()`.

The one watch-point worth recording: `get_clock().now()` is CLOCK_REALTIME
*by default* in rclpy Foxy.  If a future change sets `use_sim_time:=true`
on any of these nodes, the announcement clock diverges from the cone
clock and timing_error_ms becomes meaningless until you set it back.
The trimmed launch file does not set `use_sim_time`.

### Jugglebot-offline workflow

The cone uses Jugglebot's CAN cable; while the cone is plugged in, the
ODrives are powered (their heartbeats are visible on candump) but
Jugglebot itself is offline.  Running `jugglebot_launch.py` in this
state brings up motor_guard, mpc_bridge_node, motion_bridge_node, and
catch_coordinator_node, all of which either error or sit blocked
waiting for hardware that's not on the bus.

The trimmed launch (`catching_cone_test.launch.py`) starts only the
nodes that actually do work in this scenario.  Naming convention: the
filename is the scenario, not the subsystem, because the cone-only
scenario is the actual unit of "things you might launch."

### mocap_node's Base-alignment gate is silent — first-throw blocker

First live attempt: `bb/throw_at_target` returned "Target 'Catching_Cone'
not in latest rigid_body_poses. Known: <none>".  Topic echo on
`/rigid_body_poses` showed *nothing* publishing despite mocap_node being
alive and QTM connected.

Root cause: `mocap_node.py:148` gates `pub_rigid_bodies.publish` on
`is_aligned`, set in `mocap_interface.py:405-421` based on the
**`Base`** rigid body being within position + rotation tolerance of the
world origin.  If `Base` is misaligned (platform moved after QTM
calibration, drift, etc.), *zero* rigid bodies publish — including
unrelated ones like the cone.  The cone-only scenario doesn't logically
need Base aligned (the cone has its own world-frame position), but the
gate is global to all bodies.

Operator fix: realign the world frame in QTM so `Base` returns to
origin.  Solved the symptom; no code change.  The gate's design is
*correct* for the main robot's safety (BB calibration is in the world
frame and must stay coherent), but the failure mode is silent — worth
considering a `WARN`-throttled log when bodies arrive from QTM but the
gate suppresses publish (no work this commit; flagged as Open Question).

### Two bugs surfaced by the first run, both fixed in commit 455cf06

**Bug 1 — solver missed `BB_GEOM_PITCH_Z_OFFSET_MM`.**  The BB pitch
axis sits 17.5 mm above the yaw axis (per
`hardware_config.yaml:434`), so the release point z is
`pitch_z_offset_mm + l*sin(pitch)`, not just `l*sin(pitch)`.  The
solver was using the latter — every throw was aimed 17.5 mm too low
relative to its actual launch point.  Small fixed error in one
direction, consistent with a constant z-bias on every throw.  Whether
this alone explains the "skewed off" miss direction the operator
observed is unverified (we lack ball-tracking data to measure it
directly — see "spatial calibration" follow-up below), but it's a
real hardware-model bug worth fixing on its own merits.

**Bug 2 — `predict_throw` used the wrong catch height for cone
targets.**  `can_node._publish_throw_announcement` (the path that
publishes `ThrowAnnouncement` after `bb/send_throw_command`) calls
`predict_throw` without `catch_height_mm`, so it defaults to the
platform catch plane (`_DEFAULT_CATCH_HEIGHT_MM =
GEOM_INITIAL_HEIGHT_MM + JB_OP_DEFAULT_ACTIVE_Z_MM +
HAND_CATCH_OFFSET_MM ≈ 809 mm`, per `throw_ballistics.py:22-24`).  For
cone targets at z≈1143 mm that's ~334 mm of vertical projection
error → ~73 ms ToF error.  `cone/timing_result` was therefore reading
"user reaction lag minus 73 ms" instead of true throw-timing error;
the +74…+218 ms session-2 spread is mostly reaction lag, not pipeline
error.

The fix uses an *opt-out*: a `suppress_announcement` field added to
`SendBallButlerCommand.srv`, default False (backwards-compatible for
all existing callers — platform throws still get the platform-plane
projection, which is correct for them).  `throw_director_node` sets
it True and publishes its own `ThrowAnnouncement` with the solver's
actual target z + serial-chain-aware ToF, so the cone's correlation
reads the true expected arrival time at the cone.

Why opt-out over opt-in: the existing platform-catch behaviour is
*correct* for the dominant use case (catching at the platform's hand
position).  Adding a `catch_height_mm` field to the service request
would force every caller to pass the right z, while opt-out keeps
existing callers right by default and lets the director own its more-
specific case.  Future callers with custom predictions (e.g. a
ball-tracking-based catch_coordinator) follow the same pattern.

### What live integration didn't tell us — the spatial gap

The bag captured `/throw_announcements`, `/rigid_body_poses`,
`/cone/*`, `/bb/*` — but **not** `/mocap_data` (unlabelled marker
positions) and there is no `Ball` rigid body defined in QTM.  So even
with 5 real throws bagged, *we have no ground truth on where each ball
actually landed*.  The "skewed off in one direction" observation is
qualitative; converting that into a quantitative offset matrix
requires either (a) defining a Ball rigid body in QTM, or (b) bagging
`/mocap_data` and finding the ball as motion-classified unlabelled
markers offline.  The cone-test launch's bag list should be extended
when the next calibration session is planned.

## Verification

* **Solver tests:** 18 in `tests/ros/test_throw_ballistics.py` covering
  `_wrap_pi`, `yaw_solve_thetas`, `global_to_bb_local`,
  `solve_throw_local` success + roundtrip + infeasibility branches +
  model-mismatch documentation.  All pass.
* **Director tests:** 11 in `tests/ros/test_throw_director_node.py`
  covering preflight gates (no calibration, unknown target, empty
  target list), success path with delegation to
  `bb/send_throw_command`, solver-aware ThrowAnnouncement publication,
  caller-specified delay override, yaw-offset transform,
  infeasible-target + can-node-down branches, cache update semantics.
  All pass.
* **Full suite:** `pytest tests/ -q`, run 2026-05-23 on `refactor`:
  **1498 passed, 1 xfailed in 437.96 s.**  +28 from the baseline 1470
  on this branch.
* **GUI JS syntax:** `node --check` passes on `panels.js` (as `.mjs`).
* **colcon build:** `colcon build --packages-select jugglebot
  jugglebot_interfaces --symlink-install` succeeds; new launch file
  installed under `share/jugglebot/launch/`, new entry point installed
  under `lib/jugglebot/throw_director_node`, new srv installed under
  `share/jugglebot_interfaces/srv/ThrowAtTarget.srv`.
* **Live integration:** ran two cone-test sessions on 2026-05-24, bags
  in `~/Desktop/rosbags/cone_test_2026-05-23_23-58-37/` (5 real throws,
  all missed — no catches) and `cone_test_2026-05-24_00-02-34/` (4 real
  throws + 5 manual piezo taps to validate the timing pipeline end-to-
  end).  Manual-tap timing-error spread +74 to +218 ms — that's
  reaction lag plus the +73 ms `predict_throw`-catch-height bias the
  fix-commit removes; with the bias gone, future cone-tap experiments
  should read just human reaction lag.  The full pipeline (cone CAN
  frames → can_node decode → `cone/catch_event` → `catch_correlation_node`
  → `cone/timing_result` → GUI sound bar) ticks correctly end-to-end.
  Real throws missed "skewed off in one direction" — spatial calibration
  is the open follow-up (see Open Questions).
* **Full suite after fix commit:** `pytest tests/ -q`, run 2026-05-24
  on `refactor`: **1501 passed, 1 xfailed in 445.83 s** (+3 from 1498:
  director's own-announcement test, can_node suppress on/off tests).

## Outcome

The aim-and-throw path is live and validated end-to-end: the GUI Throw
button → `bb/throw_at_target` → solver → BB CAN throw → ball flies →
(when caught) cone piezo → `cone/catch_event` → correlation →
`cone/timing_result` → GUI sound bar.  Two bugs surfaced by the first
live session were fixed in `455cf06`; the integration itself is
solid.

Status is `tuned` because the subject of this entry (live-integrate
the throw-director feature) is addressed, but there is one
intentionally-open sibling investigation: the spatial-accuracy /
calibration-grid work surfaced by the same session.  The
cone-bringup entry's "Live Jetson integration" open question closes
with this entry; the spatial-accuracy follow-up gets its own entry
when the calibration session lands.

## Open Questions

* **Spatial accuracy / calibration grid.**  Real throws missed in a
  consistent direction.  The 17.5 mm release-z fix from this commit
  *might* account for the bias entirely (operator's intuition: "it's
  possible the z-offset is the whole explanation, I've seen that
  before"), but without ball-tracking data we can't verify.  Next
  session: define a Ball rigid body in QTM (or bag `/mocap_data` and
  identify the ball offline), throw a grid of targets, post-process
  to derive a position-dependent offset matrix.  Store the result as
  a committed artifact (joblib + reproducibility script) so it's
  durable — the previous calibration result the operator remembered
  was never committed and is gone.  Worth its own entry when the
  session lands.
* **Silent alignment-gate.**  `mocap_node` blocks
  `pub_rigid_bodies.publish` on `Base`-aligned but doesn't log when
  bodies arrive from QTM while the gate is closed.  Cost the first
  live session ~5 minutes of "Known: <none>" confusion before the
  gate was traced.  A throttled-recurring WARN ("got N bodies from
  QTM but Base misaligned — not publishing") would surface this in
  future cone-only sessions without changing semantics.  Small,
  low-risk follow-up.
* **Bag `/mocap_data` in cone-test launch.**  The trimmed launch
  bags rigid bodies but not unlabelled markers — meaning even if the
  ball had markers, we wouldn't capture them.  Add `/mocap_data` to
  the bag list before the next calibration session.
* **Model unification.**  Should `predict_throw` learn about the BB
  serial chain so the solver and predictor agree, and
  `BB_OP_LANDING_TIME_OFFSET_MS` become unnecessary?  Worth the
  refactor if the calibration session shows the constant offset is
  well-modelled by the serial chain (and not by other unaccounted-for
  effects).  Tracked by the model-mismatch tripwire test
  (`test_solve_throw_local_vs_predict_throw_has_serial_chain_bias`).
* **Target sources beyond mocap.**  The current resolution is QTM
  rigid-body-name → world position.  A future ball-juggling routine
  might want target = estimated ball-landing-position from the tracker.
  The director's `_target_positions_mm` cache is the natural extension
  point; a second subscription that writes into the same cache (or a
  source-dispatching variant) covers it cleanly.
* **GUI target list synchronisation.**  The dropdown is hardcoded in
  `panels.js` (`TD_TARGETS = ['Catching_Cone']`).  Could be populated
  dynamically from the latest `/rigid_body_poses`, but that introduces
  flicker when QTM drops a body briefly; the static list is more stable
  and aligns with which targets the *system* actually knows how to
  throw at.  Worth revisiting if the target set grows past ~3.
