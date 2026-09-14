---
title: "Hand C2 continuity + torque feedforward plumbing (PROTOCOL_VERSION 8 / can-bridge FW 22)"
type: feature
date: 2026-09-15
status: in-progress
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - config/generate_udp_protocol.py, config/generated/udp_protocol.{h,py} (v2, hand_ff_gain, HAS_V2/HAS_SCHED, HeartbeatT2J diagnostics, GET_HAND_TORQUE_SCALE)
  - config/hardware_config.yaml, config/generated/hardware_config.{h,py} (stream_stop_*, stream_hand_torque_ff_*, emit_lead_knots)
  - config/protocol_config.yaml, config/generated/protocol_config.{h,py} (hand_tor scale 100→1000, can_input_torque_scale endpoint 283)
  - "config/ODrive config Files/odrive_pro_hand_config.json (input_torque_scale 100→1000)"
  - ros_ws/src/jugglebot/{CatchingCone_code,Teensy_code_canbridge,Teensy_code_platform}/{hardware_config.h,protocol_config.h} (regenerated consumer copies)
  - ros_ws/src/jugglebot/jugglebot/{hardware_config.py,protocol_config.py} (regenerated consumer copies)
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/{leg_interp.cpp,leg_interp.h,canbridge_config.h,Teensy_code_canbridge.ino,rpc.cpp,rpc.h,can_buses.cpp,udp_protocol.h} (FW 22 — scheduled lanes, C2 stop, hand torque path, GET_HAND_TORQUE_SCALE readback)
  - teensy_link/{setpoint_pump.py,protocol.py,rpc.py,rpc_args.py}
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py (hand_torque_ff_gain param, verified-readback gate, /link_status rows)
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py (absolute knot grid, emit lead, epoch stamping, fresh-install t0 snap)
  - ros_ws/src/jugglebot/jugglebot/motion/ipc.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/{emitter.py,cycle_plan.py,feasibility.py} (hand_accel_at, HAND_LIMIT_C2 gate)
  - tools/probes/teensy_link_profiling/hermite_xref/teensy_interp.py (twin — scheduled path + torque model, line-for-line mirror)
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py (regenerated mirror copy)
  - tools/probes/hand_cascade_ff_model.py (new — offline cascade A/B model)
  - tools/probes/README.md
  - tests/firmware/native/{test_leg_interp.cpp,test_rpc_dispatch.cpp,odrive_protocol_golden.json}
  - tests/firmware/{test_udp_protocol_xlang.py,test_sched_c2_twin.py,test_hand_torque_ff_twin.py} (latter two new)
  - tests/teensy_link/{test_setpoint_pump.py,test_protocol_codec.py,test_v5_wire_regression.py,test_rpc.py,test_rpc_args.py}
  - tests/motion/{test_trajectory_emitter.py,test_cycle_plan.py,test_validate_cycle.py,test_validate_cycle_vectorised.py,test_skills_executor.py}
  - tests/ros/{conftest.py,test_teensy_bridge_node_hand.py,test_teensy_bridge_node_hand_torque_ff.py,test_unified_cycle_integration.py} (test_teensy_bridge_node_hand_torque_ff.py new)
subsystem:
  - can
  - motion
  - ros
  - tools
  - config
tags:
  - safety
  - performance
  - IPC
  - testing
---

# Hand C2 continuity + torque feedforward plumbing (PROTOCOL_VERSION 8 / can-bridge FW 22)

## Summary

The owner asked for hand torque feedforward before flying the R3 apex ladder
(`logbook/2026-09-14-skill-stack-r3-apex-ladder-prep.md`), with a hard
requirement that the actuator commands stay C2 (continuous acceleration)
including between knots, not just within the planner's own curve. That
turned into a wire bump (PROTOCOL_VERSION 7→8) and a can-bridge firmware
bump (FW 21→22): stamped, phase-locked frames played on their own clock
instead of on UDP-arrival time, firmware-computed hand torque
`τ = K·J·2π·a_cmd` tied to the curve the firmware actually emits, a
torque-scale readback gate, and a `trajectory_node` absolute knot grid so the
emitter never re-phases. Four units (U2a/U2b firmware, U3a/U3b host) landed
2026-09-14/15, built and scoped-tested, nothing flashed or committed yet.

## Motivation

The plan itself is already C2 to float64 noise: a real 4-throw self-toss
chain built through `install_segment` measures `|Δa| ≤ 8.5e-9 rev/s²` across
every interior knot and every splice seam, at both 0.9 m and 0.5 m apex
(`tools/probes/hand_cascade_ff_model.py`'s sibling analytic probe, folded
into `tests/motion/test_skills_executor.py`). The breaks were entirely in
the *stream*: the emitter sampled the plan at an arbitrary sub-knot phase
(299–679 rev/s² steps at each 25 ms frame boundary), the firmware started
each frame's clock at UDP arrival rather than the stamped play time (1 ms of
jitter ≈ 122 rev/s²), a late frame (>25 ms gap) fell back to a zeroed
acceleration, and measured emit gaps in a real bag
(`2026-09-13_22-57-18`) ran p50 26.1 ms / p99 30.1 ms / max 40.2 ms per
200 ms window — well past a single frame period. On top of that the hand
ODrive received `torque_ff = 0` on every frame; the retired Platform-Teensy
stroke engine sent roughly 70% of `J·α` and its absence is the leading
suspect for the R3 apex ladder's ~25% throw overspeed.

## Design

Owner-approved decisions (2026-09-14), each closing a design fork the two
survey agents and the probe opened:

1. **Phase-locked, stamped frames played on the stamp**, not a per-frame
   quintic re-fit. A quintic with `(p, v, a)` per frame still misses the
   plan by 53–250 rev/s² mid-frame (measured); a knot-aligned cubic played
   on its own clock is exact.
2. **All 7 lanes ride the scheduled clock**, leg interpolation math
   unchanged — so the emitter now sends exact leg `v1`/`v2` on every frame,
   holds included, not just throws.
3. **Firmware computes torque**, `τ = K·J·2π·a_cmd`, from the curve it
   actually emits (span Hermite, C2 stop, slew, or 0 at hold) rather than
   from a Jetson-supplied torque knot. `K` is per-frame from the host and
   defaults to 0 (arm A of the A/B).
4. **Hand wire torque scale 100 → 1000** (yaml, ODrive JSON, firmware),
   gated by a host readback of the drive's actual `input_torque_scale`: `K`
   is forced to 0 until the readback matches.

Leg torque FF is explicitly out of scope; the wire is 7-wide so it can land
later without another bump. No dedicated plan file was opened for this unit
(owner) — it is recorded here and in the four unit handoffs referenced
below.

## Discussion

**Both survey agents opened by claiming the planned hand curve was only
C1 — refuted by the empirical probe.** The firmware-side survey read
`CyclePlan.hand_at`'s cubic Hermite basis and concluded acceleration steps
"at every knot in the plan itself," because nothing in the QP enforces the
spline continuity condition. That is true of the *general* curve family —
but the probe that actually walked a real R3-shaped chain through
`install_segment` (the code path `trajectory_node` uses) found the QP's
chosen knot velocities satisfy the spline condition to float64 noise on
every knot it produces at this operating point, splices included. The
class of curve the planner *can* emit is not C2 by construction; the curve
it *does* emit, here, is. This mattered for the design fork directly: it
meant the fix could be "stop breaking the plan's own continuity in the
stream" rather than "re-derive the plan to force continuity," which would
have meant solving a global tridiagonal spline condition inside the QP.

**The orchestrator's own pre-registered FF magnitude was wrong.** Before
the firmware survey returned hard numbers, the working assumption in this
arc was "expect roughly 0.3–0.5 N·m of feedforward." The actual figure,
from `J = 1.05e-5 kg·m²` (measured reflected inertia) and the plan's own
peak acceleration at 0.9 m apex (3340.6 rev/s²), is `τ ≈ 0.2204 N·m` — about
40 A at the hand drive's 50 A soft current limit (0.2757 N·m ceiling). A
naive 0.3–0.5 N·m target would have left the position loop with no current
headroom at all; the 0.234 N·m clamp (0.85× the drive ceiling) exists
because of this correction, not the original guess.

**The pre-registered A/B model prediction was NOT SUPPORTED.** The plan for
validating K before flashing was to build an offline cascade model
(`tools/probes/hand_cascade_ff_model.py`) predicting peak
measured/commanded velocity ratio at K = 0, 0.7 and 1.0, with a
pre-registered pass criterion of K = 0.7 → ≤ 1.00 ± 0.02. The model instead
gave, at 0.9 m apex: K=0 → 1.14×, K=0.7 → 1.05×, K=1.0 → 1.02× (0.5 m apex:
1.11× / 1.04× / 1.015×). None of the three K values clears the criterion —
the verdict the probe's own output recorded is literally "NOT SUPPORTED"
(`temp/probes/hand_cascade_ff/hand_cascade_ff_20260914T122549Z.md`). Two
candidate reasons were named, neither investigated further in this unit:
the model's cascade doesn't carry the ball's own +23% inertia contribution
during the ascent, and it doesn't model the current ceiling at ~49 A that
the streamed data shows binding near the model's own K=0 case. This is why
the operator flies the K=0 vs K=0.7 A/B on hardware rather than trusting
the model's own recommended K — the model narrowed the search, it didn't
answer it.

**The emit-lead decision was reversed mid-unit.** The spec (`c2ff_spec.md`)
originally called for an emit lead of 1 knot ahead of the current tick, to
give the firmware a frame before it needs it. U3b's own seam survey and the
scheduled-lane design (U2a) found this widens the exact window during which
a "seed at now" install (a hold or a descent, which install at the tick's
*current* knot rather than one ahead) races the still-playing old plan, and
it eats into the splice margin `WIRE_READ_KNOTS` already assumes. The lead
was set to 0 instead; U2a's 20 ms grace window (below) absorbs the ordinary
case the lead was meant to buy slack for, without widening that race.

## Implementation

Four units landed in sequence, each build-only / test-only (nothing flashed,
nothing committed):

- **U2a — wire v8 + can-bridge FW 22 scheduled lanes.** `SetpointPayload`
  gains `v2 f32[7]` (exact knot velocity at u2) and `hand_ff_gain f32`
  (208→240 B); `HeartbeatT2J` gains promotion-continuity diagnostics and
  scheduler counters (73→121 B). Firmware holds a 4-slot queue ordered by
  `t_start` (derived from `t_origin_us` on the shared wall clock, not UDP
  arrival), promotes the newest due frame before advancing the interpolator
  (advancing first was found, via the twin, to start a stop whenever a
  cover boundary falls between ISR ticks), and runs a closed-form C2 stop
  polynomial — matched acceleration at both ends, exact minimal duration via
  bracket-then-bisect — when no newer frame arrives within a 2-span + 20 ms
  grace window. The grace exists because at zero emit lead every emitter
  overrun drops a knot and the next frame lands 1–15 ms late; without it the
  twin hit 37 spurious stop-and-latch refusals before the fix. Resume out of
  a stop is a latched hold that only accepts a promotion within tight
  position/velocity/acceleration tolerances — anything else stays refused
  and loud until the guard disarms, because accepting an out-of-tolerance
  resume is exactly the step the stop exists to prevent.
- **U2b — hand torque path, drain, readback.** `τ = fade · sat(Ks·J·2π·a_cmd
  + bias)`, computed every 500 Hz tick, saturated on the *sum* (not the
  raw K·J·α term alone) at 0.234 N·m — a deliberate deviation from the
  spec's literal formula, because saturating only the K-term could still
  let a nonzero bias push the total past the drive's 0.2757 N·m ceiling.
  `Ks` slews toward the host's target at ≤5/s; `fade` ramps to 0 at 50/s
  whenever the lead clamp, stroke clip, recovery slew or NaN backstop is
  active, so none of those existing nets gets a torque step layered on it.
  A 3-tick (6 ms) drain re-sends the last frame with τ forced to 0, bit-
  identical position/vel_ff otherwise, whenever output disables while the
  last transmitted torque was nonzero — the ODrive's CAN watchdog is off
  (`enable_watchdog: false`) and holds its last command indefinitely, so
  without the drain a disarm mid-throw would leave feedforward torque
  latched on the axis. The readback (`GET_HAND_TORQUE_SCALE`, endpoint 283,
  from ODrive Pro's own `flat_endpoints.json`) lets the host confirm the
  drive is actually running scale 1000 before it ever sends K > 0; an FW≤21
  board answers `ERR_UNKNOWN_METHOD`, read as unverified.
- **U3a — pump + bridge node wire fill.** `SetpointPump` gains
  `hand_ff_gain`/`hand_torque_scale_verified`; the new v2/HAS_SCHED fields
  degrade cleanly rather than joining the existing hand all-or-nothing
  reject gate (a producer that doesn't yet know about the new optional
  fields — every bench source, forever — is not in an incoherent state).
  `hand_torque_ff_gain` is a new validated ROS param with the first
  `add_on_set_parameters_callback` this node has ever used (grepped: zero
  prior uses anywhere in `ros_ws/src/jugglebot/`), wired to refuse a bad
  live `ros2 param set` rather than silently swallowing it.
- **U3b — trajectory_node emitter, t0 snapping, `validate_cycle` gate.** The
  emitter's clock moved from "now, whenever the thread wakes" to a fixed
  absolute grid `t_k = k·dt`; an overrun skips forward on the same grid and
  never re-phases. `install_segment`'s fresh branch (seeded from REST) now
  snaps its origin onto that grid; splices inherit the already-snapped
  chain origin unchanged. `feasibility.py` gained a fifth vectorised
  `validate_cycle` pass, `HAND_LIMIT_C2`, comparing left/right Hermite
  acceleration at every interior hand knot against `1e-3 × hand_acc_limit`
  (3.5 rev/s² at the current 3500 rev/s² operating point) — a genuine
  regression detector five orders of magnitude above the probe's own noise
  floor. One real, unrelated production path was found to violate it: a
  stroke-saturated knot (the slider clamped to its physical travel) forces
  a hard velocity override that is not QP-smoothed and produces ~5200 rev/s²
  steps — an order of magnitude over the acceleration limit itself, not
  just the C2 tolerance. No R2/R3 sizing reaches a saturated knot today
  (the probe found zero across the full 4-throw chain at both apexes), so
  the gate has no live trigger, but if a future site does reach one it will
  now be refused loudly rather than shipping a torque-step curve silently.

## Verification

Each unit ran scoped tests only (build-only firmware; the orchestrator owns
`./run_tests.sh --full`):

- U2a (2026-09-14): `pio run -e teensy41` SUCCESS; `pytest tests/firmware
  tests/teensy_link -q --deselect tests/firmware/test_native_firmware.py` →
  **596 passed, 1 skipped**; `pytest tests/firmware/test_native_firmware.py
  -q` → **17 passed** (185.8 s, 5 new FW-22 cases in `test_leg_interp`);
  `pytest tests/sim/test_skills_gate.py tests/motion/test_skills_executor.py
  tests/ros/test_unified_cycle_bench.py -q` → **150 passed, 1 xfailed**;
  `pytest tests/ros/test_teensy_bridge_node*.py -q` → **406 passed**.
- U2b (2026-09-15): `pio run -e teensy41` SUCCESS; `pytest tests/firmware
  tests/teensy_link tests/motion/test_endpoint_id_contract.py -q --deselect
  tests/firmware/test_native_firmware.py` → **635 passed, 1 skipped**;
  `pytest tests/firmware/test_native_firmware.py -q` → **17 passed**
  (190.7 s, `test_leg_interp` now 47 cases, `test_rpc_dispatch` +3).
- U3a (2026-09-14): `pytest tests/teensy_link/test_setpoint_pump.py -q` →
  **89 passed**; `pytest tests/ros/test_teensy_bridge_node_setpoint.py
  tests/ros/test_teensy_bridge_node_hand.py
  tests/ros/test_teensy_bridge_node_hand_torque_ff.py
  tests/motion/test_trajectory_emitter.py -q` → **160 passed**. A wider
  `tests/teensy_link tests/ros/test_teensy_bridge_node*.py
  tests/motion/test_trajectory_emitter.py -q` run showed **822 passed, 2
  failed** — one is U2b's own in-flight RPC test (fixed by U2b landing
  after), one is a pre-existing load-flake in
  `test_teensy_bridge_node_udp_diag.py` (passes in isolation), neither a
  regression from this unit.
- U3b (2026-09-14): `pytest tests/motion/test_trajectory_emitter.py
  tests/motion/test_cycle_plan.py tests/motion/test_validate_cycle.py
  tests/motion/test_validate_cycle_vectorised.py -q` → **79 passed**; a
  wider 15-file run across `tests/ros` and `tests/motion` → **667 passed, 1
  failed** (a documented CPU-contention load-flake in
  `test_unified_cycle.py`, passes in isolation: 1 passed, 0.29 s);
  `tests/sim/test_skills_gate.py -q` → **11 passed, 1 xfailed**.

- **Final gate** (2026-09-15, `./run_tests.sh --full`, on the committed tree):
  **PASS** — parallel **6097 passed, 9 skipped, 2 xfailed** in 297.57 s;
  serial **6 passed**. Two earlier full runs the same day failed and were
  fixed before this one: stale admissible boxes (the new C2 gate changes
  `gate_hash`) plus two test expectations (sim flags gained HAS_V2; a fresh
  install's t0 now snaps to the grid), then generator drift in
  `udp_protocol.{py,h}` after doc-string fixes.
- **Admissible boxes re-swept** (2026-09-15, `python tools/admissible_sweep.py
  --site-pairs both --single-apex 0.5 0.6 0.7 0.8 0.9 --leg-jerk 150000`):
  every box identical to the previous file; only `swept_at` and `gate_hash`
  (`8fbee14b1090` → `d38f76690bc1`) changed.
- **Firmware build** (2026-09-15, `pio run -e teensy41`, BUILD ONLY): SUCCESS.
  Nothing flashed.
- **Hardware:** NOT flown. The K=0 vs K=0.7 A/B is
  `tests/hardware/session_skills_r3_apex_ladder.md`.

## Outcome

The plan is confirmed C2 to measurement noise; the stream-side breaks that
defeated it (off-knot resampling, arrival-time phase, zeroed late-frame
acceleration) are fixed in FW 22 behind a phase-locked scheduled clock with
a C2 stop on cover exhaustion and a latched, fail-loud resume discipline.
Hand torque feedforward is plumbed end-to-end — firmware computation from
the emitted curve, a drive-limit-respecting clamp, a fade that covers every
existing safety net (lead clamp, stroke clip, recovery slew, NaN backstop),
a drain that prevents latched feedforward torque surviving a disarm, and a
host-side readback gate that keeps K forced to 0 until the drive's wire
scale is verified. Nothing is flashed or committed; the offline cascade
model's own pre-registered criterion was not met at any tested K, so the
next step is empirical (the hardware apex ladder), not a model-derived
gain choice.

## Withdrawn claims

- [2026-09-14] Both the firmware-side and Jetson-side survey agents opened
  by stating the planned hand curve is C1, not C2.
  WITHDRAWN: true of the general Hermite curve family the QP *could*
  produce, but not of the curve it actually produces at this operating
  point. `probe_hand_c2.py`'s real `install_segment` chain measured
  `|Δa| ≤ 8.544e-9 rev/s²` across every interior knot and splice seam at
  both 0.9 m and 0.5 m apex — float64 noise floor.
  Superseded by: Discussion, "Both survey agents opened by claiming..."
- [2026-09-14] Orchestrator's working assumption going into the firmware
  survey: expected hand feedforward torque in the 0.3–0.5 N·m range.
  WITHDRAWN: the real figure at the plan's own peak acceleration is
  ≈0.2204 N·m (40 A against a 50 A / 0.2757 N·m drive ceiling); a
  0.3–0.5 N·m target would have left no current headroom for the position
  loop at all.
  Superseded by: Discussion, "The orchestrator's own pre-registered FF
  magnitude was wrong."
- [2026-09-14] Pre-registered pass criterion for the cascade A/B model:
  K=0.7 → peak measured/commanded velocity ratio ≤ 1.00 ± 0.02.
  WITHDRAWN: the model itself returned 1.05× at K=0.7 (0.9 m apex),
  outside the criterion at every tested K including K=1.0 (1.02×). Filed
  as "NOT SUPPORTED" in the probe's own output, not silently dropped.
  Superseded by: Discussion, "The pre-registered A/B model prediction was
  NOT SUPPORTED" — the operator flies K=0 vs K=0.7 on hardware instead of
  trusting a model-recommended gain.
- [2026-09-14] Spec's original decision: emit frames 1 knot ahead of the
  current tick (`emit_lead_knots: 1`).
  WITHDRAWN: widens the window in which a "seed at now" install (a hold or
  descent) races the still-playing old plan, and eats into the
  `WIRE_READ_KNOTS` splice margin.
  Superseded by: Design point 1 is unaffected, but `emit_lead_knots` ships
  at 0; U2a's 20 ms grace absorbs ordinary overrun instead.

## Open Questions

- **Operator steps before any K > 0 sitting**: re-apply the hand ODrive's
  `input_torque_scale` to 1000 (USB/odrivetool or the regenerated ODrive
  JSON), flash can-bridge FW 22 in lockstep (`pio run -e teensy41 -t
  upload`), and verify `RpcClient.read_hand_input_torque_scale() == 1000`
  on the live bridge before the pump is allowed to send K > 0.
- Recovery slew, lead clamp and stroke clip torque paths are NOT C2 — fade
  covers the step, but the underlying pos/vel_ff commands through those
  paths still step, same as before this unit.
- Grace-window extrapolation can add up to ≈2800 rev/s² of acceleration
  (hand) before a stop engages, at the worst-case 20 ms of open-loop
  continuation.
- `vel_ff` persists on the hand ODrive after TX stops (pre-existing since
  FW 17, not touched by the new drain, which deliberately leaves it
  bit-identical); whether the drain should also zero it is an explicit
  owner decision, flagged for before high-speed throws.
- The FSM `PlanCycle` path (`_svc_plan_cycle`) is not grid-snapped or
  seam-surveyed — deliberate, since it is deleted at R4, but its hand-accel
  continuity under a torque-FF-enabled firmware is unverified if it is ever
  exercised concurrently.
- Several judgment constants need a bench check before they're trusted:
  leg stop limits (250 rev/s² / 25000 rev/s³), the promotion/resume
  tolerances, and the 20 ms grace window itself.
- ISR jitter (`interp_max_jitter_us`) was not measured on target — the
  torque block adds a few float ops per 500 Hz tick; measure at the first
  sitting.
- The ball's own inertia contribution (+23% during the ascent) is not in
  the cascade FF model — a candidate reason the model's own K=0.7
  prediction missed its criterion.
- BallButler's generated config copies are now stale (U2b's regeneration
  ran with `--no-external`); the next plain `generate_config.py` run
  delivers them. Normal generator behaviour, not a defect.
