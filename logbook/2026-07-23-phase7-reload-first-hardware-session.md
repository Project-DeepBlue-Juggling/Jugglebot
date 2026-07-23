---
title: Phase-7 reload first hardware session — z frame double-add rejected every catch reach; prime raced the flight; abort retract was a silent no-op
type: investigation
date: 2026-07-23
status: resolved
phase: "MVP trajectory bringup — Phase 7 reload: first hardware validation of the action-driven catch"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/reload_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot_interfaces/msg/DynamicTargetCommand.msg
  - config/hardware_config.yaml
  - tests/ros/test_trajectory_node.py
  - tests/ros/test_reload_integration.py
  - tests/ros/test_reload_sequencer.py
  - tests/ros/test_reload_coordinator_node.py
  - tests/ros/test_catch_coordinator_node.py
  - ros_ws/src/jugglebot_interfaces/action/Reload.action
  - tests/hardware/session_phase7_reload.md
  - plans/active/mvp-trajectory-bringup.md
  - logbook/INDEX.md
commits:
  - 8dbd3a0
  - bc6f78d
subsystem:
  - ros
  - motion
tags:
  - trajectory
  - reload
  - catch
  - frame-convention
  - hand
  - ball-butler
---

# Phase-7 reload: what the first hardware session found, and the fixes

## Summary

The operator ran `tests/hardware/session_phase7_reload.md` (bag
`~/Desktop/rosbags/2026-07-23_09-13-51`; four manual 7b throws + two full 7c reload
actions). 7a passed. 7b was hit-or-miss: the hand prime raced the ball. 7c never
tilted the platform — `trajectory_node` rejected **every** catch reach with
`leg 3 out of stroke (275.2 mm) on the follower segment` — yet BB threw anyway and
twice the ball landed in the primed, parked cup.

Three root causes, all confirmed against the rosbag and the repo's own kinematics,
plus two latent bugs found en route. All fixed this session.

## Root cause 1 — the catch z was double-added (the pre-registered Phase-7 open question)

`catch_coordinator` publishes `catch/dynamic_target` **stow-relative**
(`z = world_landing − GEOM_INITIAL_HEIGHT` ≈ 171.4 mm at the 12° tilt clamp; the bag
shows all 310 wire messages at z = 171.0–171.3). `trajectory_node._catch_target_from_msg`
believed the wire was **active-relative** ("the MPC target convention") and added
`JB_OP_DEFAULT_ACTIVE_Z_MM` = 170 → commanded catch z ≈ **341 mm** stow-relative.
Required leg extensions for that pose are 272.7–340.7 mm against a 275.0 mm hard max
(280 stroke − 5 margin) → `build_catch`'s gate rejected every reach `WORKSPACE`.
Replaying the reach through `validate_follow`'s 300-sample scan reproduces the exact
observed string — first out-of-bounds sample "leg 3 … 275.19 mm" for ball-arrival
azimuths in the 15–75° band. The **intended** pose (z ≈ 171, 12° tilt) needs at most
192.1 mm of extension and passes the gate comfortably.

The consumer's premise was factually wrong about the MPC: the MPC frame is itself
stow-relative (`run_mpc.py:80-82`, `controller/zmq_target.py:90`,
`controller/target.py:761`), and the historical `mpc_bridge` forwarded the wire
verbatim. The mislabel that seeded it: a comment in `catch_coordinator.py` calling
`(0, 0, initial_height)` "the active pose" (it is the stow plane). Notably the
Phase-5 author pre-registered exactly this doubt (`trajectory_node.py`: "Frame
convention to be RE-VERIFIED on hardware in Phase 7 before any ball flies") — the
feasibility gate did precisely its job, loudly, before any bad motion.

**Fix**: drop the `+ active_z` lift; the wire is declared **STOW-relative,
normatively, in `DynamicTargetCommand.msg`**; the mislabeled comment corrected; the
plan text corrected. Chosen over flipping the coordinator because (a) every recorded
session bag carries the stow-relative wire — flipping the producer would silently
frame-shift all replays by 170 mm; (b) a future MPC re-bridge consumes stow-relative
verbatim (zero conversion points — it was the conversion point that failed); (c) a
consumer that wrongly assumes active-relative on a stow-relative wire UNDER-commands
(harmlessly low), whereas the reverse OVER-commands out of stroke — the convention
fails in the benign direction.

**Class guard**: a new reach-envelope check in `trajectory_node._on_dynamic_target`
(config `catch_reach_envelope_mm` = 80, the envelope `build_catch`'s docstring always
assigned to the caller but nothing enforced). Any catch target > 80 mm (3D) from the
pose held at arm-latch raise is rejected `WORKSPACE` before planning. A future
z-frame regression now rejects loudly at "~170 mm from the armed hold pose", not as a
cryptic leg-stroke graze — and it bounds root-cause-adjacent finding 4 (below).

## Root cause 2 — the hand prime raced the flight (7b), and was still throw-gated (7c)

Empirics from the bag's 100 Hz hand telemetry: bottom→top smooth-move = 0.68–0.73 s
measured (0.754 s from the firmware constants: `T = √(Δrev·5.77/100)`), vs BB flight
time 0.878 s. In 7b the only prime trigger was the first coordinator command — which
fires when the ball is already IN FLIGHT — so the race was decided by < 0.1 s: throw 1
lost by 0.06 s, throw 4 won by 0.09 s, throw 3 never primed at all (stale
`_hand_primed` one-shot). And a hand still mid-prime at fire time is worse than late:
the Platform Teensy's catch-command prelude has a time-budget check that **silently
drops the whole catch stroke** (Serial-only message) when the smooth-move can't finish
before the first trajectory frame — the observed "hand at bottom never tried to catch".

**Fix** (operator requirement): the hand primes at **command time**. The FSM emits a
new `ACTION_PRIME_HAND` the moment CHECKING's preconditions pass (before the BB reload
wait and the aim), and `catch_coordinator_node` now also primes on the `catch/armed`
**rising edge** — covering the manual 7b recipe — instead of waiting for a ball. Once
primed, ANY abort retracts (terminal action covers `_primed`, not just `_prepared`).

## Root cause 3 — arming failures were undetectable and BB throws unabortable

`_prepare_catch` ignored the return values of both the hand prime and the latch raise,
and BB has **no throw-abort surface at any layer**: the ~3 s countdown lives in the BB
Teensy firmware (`StateMachine` enters THROWING immediately; the bridge's throw action
explicitly rejects cancel — "a throw in flight has no firmware abort path"; protocol
opcodes are THROW/RELOAD/RESET/CALIBRATE only, and RESET is ERROR-state-only).

**Fix** (operator requirement, by ordering rather than by new protocol): every
Jugglebot-side arming step now happens **before** `bb/throw_at_target` is sent —
CHECKING primes, a new PREPARING phase raises the latch and waits for the
node-confirmed result, and only then does AIMING commit the throw. A prime/latch
failure aborts (`ABORTED_PRIME_FAILED` / `ABORTED_PREPARE_FAILED`) while there is
still nothing to abort on the BB side. The unabortable residue shrinks to faults
*during* the countdown (mode change / BB error), which SAFE_ABORT safes on the
Jugglebot side. A true firmware abort (new CAN opcode + both-Teensy flash) was
scoped and deferred — see Discussion.

## Latent bugs found en route (both fixed)

1. **SAFE_ABORT's retract was a silent no-op**: it targeted
   `HOMING_HAND_ABS_POS_REV` = −0.1 rev, which the bridge's `smooth_move_hand`
   validation range [0, 11.1] rejects — and the return value was ignored. Every
   aborted reload left the hand parked at top (visible in the session's
   between-throw hand positions). New `hand_retract_rev: 0.0` config constant, used
   by SAFE_ABORT; all safing-path failures now log loudly; a class-guard test pins
   both dispatch targets inside the service range.
2. **Mid-flight teardown**: a standing catch-target rejection finished the FSM
   immediately → the terminal SAFE_ABORT ran while the ball was airborne, tearing
   down the hand's armed schedule and commanding a retract INTO the incoming ball
   (masked only by bug 1!). Now a standing infeasibility is latched but resolves at
   settle: the platform holds, the hand keeps its schedule, and a tracker-confirmed
   CAUGHT (the parked cup caught twice this session) wins over MISSED_INFEASIBLE.
3. (Minor) `reload_sequencer`'s BB-state constants had `RELOADING = 2`, colliding
   with the protocol's `TRACKING = 2`. Unused by current logic, but now corrected
   and pinned by a test against `protocol_config.BallButlerStates`.

## Open finding — BB-ball landing-estimate drift (NOT fixed here)

The tracker's Kalman landing estimates for BB-thrown balls drift 435–605 mm/s toward
BB during the flight (ball 5: x from −6 to −704 mm in 1.26 s), while human-thrown
balls drift 6–22 mm/s. The drift signature is consistent with ~0.24 s of measurement
lag × ball speed, but the attribution (and whether it relates to the OPEN can-bridge
Teensy uptime-lag issue — mocap does not cross that Teensy) is unestablished. With
the z fix alone this drift would have dragged the platform sideways chasing the
estimate; the 80 mm reach envelope bounds that failure while the tracker issue is
investigated. **This needs its own session before catches are judged on accuracy.**

## Verification

- Scoped: `pytest tests/ros/test_reload_sequencer.py tests/ros/test_reload_coordinator_node.py tests/ros/test_catch_coordinator_node.py tests/ros/test_trajectory_node.py tests/ros/test_reload_integration.py -q` (2026-07-23): all pass.
- Broader: `pytest tests/ros tests/motion -q` (2026-07-23): **1655 passed in 340.08 s**.
- Full pre-commit gate, run twice on 2026-07-23: `pytest tests/ -q` = **2901 passed,
  1 xfailed, 0 failed in 921.71 s** (pre-audit-fix tree), then on the final tree
  **2901 passed, 1 failed, 1 xfailed in 760.28 s** — the one failure being the
  KNOWN load/order-flaky `TestT3bH4PostSolveAllocation::test_t3b_h4_on_post_solve_allocates_within_budget`
  (passed isolated 1/1 in 7.23 s immediately after; per
  `project_hot_loop_alloc_test_flaky` this does not block commits, and this diff
  touches nothing near MPC allocation).
- New regression tests: the stow-relative wire (no lift) at node level; the
  cross-wire choreography test (real tracker → real coordinator → real msg packing →
  real `trajectory_node` → `build_catch` ACCEPTED at z ≈ 171) — closing the exact
  mocked-ROS blindness that hid the frame mismatch; envelope reject + lifecycle;
  prime-at-command ordering; prepare-before-throw gating; prime/prepare-failure
  aborts with `_throw_sent is False`; infeasible-resolves-at-settle (both outcomes);
  hand-target range guard; BB-enum drift guard.

## Discussion

**Why the graze theory was wrong, and how it was killed.** The first forensic read of
the bag concluded the rejections were a genuine stroke-ceiling graze (readings
275.1–276.2 vs the 275.0 limit, on a dead-centre ball) and that the wire z ≈ 171 was
"contract-correct, not the bug". Both halves were half-right and the conclusion wrong:
the wire IS correct, but the consumer added 170 on top, and the tight clustering just
above the limit is exactly what a 300-sample scan of a reach produces at its first
out-of-bounds sample regardless of how far past the limit the target lies. The
adversarial verification pass settled it numerically with the repo's own IK: intended
pose max ext 192.1 mm (feasible — so no graze existed to explain), double-added pose
272.7–340.7 mm, first-OOB reading 275.0–275.9 across azimuths — "leg 3 … 275.2"
verbatim. Lesson repeated from this project's history: a plausible mechanism that
explains the error string is not yet the mechanism; compute the counterfactual.

**Why fix the consumer, not the producer** — covered under Root cause 1; the deciding
failure modes were bag-compat, the dormant MPC's verbatim-forward, and the benign
failure direction. The rejected alternative (make the wire active-relative) is
recorded here so a future session doesn't relitigate it without new evidence.

**Why ordering instead of a BB abort opcode.** A true abort needs a new CAN opcode +
handlers in BOTH Teensy firmwares (operator-only flashes), buys only the ~2.3 s
pre-windup window, and the main failure class it would cover — arming fails after
throw-accept — is *eliminated structurally* by arming first. The residue (operator
mode-change or BB fault during the 3 s countdown) is rare, operator-initiated, and
already safed Jugglebot-side. The opcode remains worth adding when the firmware is
next open for changes; scoped in this entry's forensics (new 0x7D6, streamer disarm
gated on frames-not-yet-sent, CMD_RESULT ABORTED relay).

**What the mode→latch refactor's review missed.** Three of the five findings here
(prime timing, retract range, mid-flight teardown) are *choreography-with-hardware*
bugs: each unit behaved as its mocked tests specified, and each spec was wrong about
the physical world (flight times, service validation ranges, a ball that keeps
falling after the FSM "finishes"). The cross-wire choreography test added here is the
template: drive the REAL producer output through the REAL message into the REAL
consumer and assert the physical quantity (a commanded pose), not the call pattern.

## Related

- `logbook/2026-07-20-reload-action-catch-latch.md` — the refactor this session validated
- `plans/active/mvp-trajectory-bringup.md` — Phase-7 plan text corrected by this entry
- `tests/hardware/session_phase7_reload.md` — updated re-test runbook
- Rosbag: `~/Desktop/rosbags/2026-07-23_09-13-51` (310 dynamic targets, 7 WORKSPACE rejects, 6 throws)
- OPEN: BB-ball landing-estimate drift (435–605 mm/s) — needs its own investigation
- OPEN: can-bridge Teensy uptime lag (`logbook/2026-07-18-teensy-uptime-tracking-degradation.md`) — reboot before the re-test session
