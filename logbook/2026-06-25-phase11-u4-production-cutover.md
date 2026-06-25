---
title: Phase 11 U4 — production α→β setpoint switch + friction-FF drop + /teensy rename
type: feature
date: 2026-06-25
status: complete
phase: "11"
related_plan: teensy-can-offload.md
files_changed:
  - controller/teensy_link/setpoint_pump.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot/launch/teensy_bridge_launch.py
  - controller/teensy_link/fault_logic.py
  - tests/teensy_link/test_setpoint_pump.py
  - tests/firmware/test_hermite_xref.py
  - tests/ros/test_teensy_bridge_node_setpoint.py
  - tests/ros/test_teensy_bridge_node_read.py
  - tests/ros/test_teensy_bridge_node_watchdog.py
  - tests/ros/conftest.py
  - tests/hardware/teensy_encoder_search_bench.py
  - tests/hardware/teensy_home_bench.py
commits:
  - cb0d158
  - 50fc8fe
  - aa74fdf
  - 123b386
subsystem:
  - can
  - controller
tags:
  - safety
  - testing
---

# Phase 11 U4 — production α→β setpoint switch + friction-FF drop + /teensy rename

**Summary.** U4 is the *production* half of the teensy-can-offload leg path: the
desk-side code switch that finally delivers the migration's headline benefit —
moving the 500 Hz interpolation off the Jetson onto the Teensy. The deployed
bridge had been forwarding `motor_guard`'s *already-interpolated* 500 Hz output
(`:5556`, `flags=0`, the **α relay**), so the Teensy ran in pass-through and
`motor_guard`'s Hermite still ran on the Jetson. U4 re-points the bridge at the
**40 Hz MPC command stream** (`:5557`, `TOPIC_MPC_CMD`) and rewrites
`SetpointPump` to emit **β knots** (`u0/u1/u2` + `v0`, `flags` carrying
`HAS_U1`/`HAS_U2`, `torque_ff = 0`) so the Teensy's 500 Hz cubic-Hermite
interpolator (`leg_interp.cpp` Mode 1) does the interpolation. `motor_guard`
leaves the leg path entirely; leg safety becomes **MPC (coupled workspace) + the
Teensy fault machine** (per-leg stroke / deviation / MPC-staleness-E-STOP /
deferred-stow — all U3-validated). The friction-FF is dropped (decision **D9**,
green-lit by the U3-iv measurements: float32 residual 5.5e-7 rev + null
motion-onset penalty — see `2026-06-24-phase11-bench-cutover.md`). U4 also retires
the side-by-side `/teensy/*` namespace, promoting the leg/hand topics + services
to their production names. **U4 ARMS NOTHING** — it stays gated behind
`enable_setpoint_output=false` / `mpc_active=0`; the powered six-leg validation of
the β path is **U5**. This entry is the production-code companion to the bench/
hardware-validation record in
[[2026-06-24-phase11-bench-cutover]] (U1–U3 + D9).

## Design

**The α→β switch and the friction-FF drop (D9) are ONE diff.** The α source
(`:5556`) carries `motor_guard`'s Stribeck friction-FF in `leg_torques`; the β
source (the MPC `:5557` stream) carries `torque_Nm = zeros`. So re-pointing the
source *is* the friction-FF-drop edit — they cannot be split, and the decision to
accept the loss (D9) had to come from a *powered* measurement, which is exactly
what U3-iv produced. With D9 decided (accept the loss), U4 was unblocked.

**THE load-bearing invariant: the switch must be BUMPLESS.** The new pump must
reproduce `MotorGuard._on_mpc_command`'s *exact* knot latch
(`motor_guard.py:541–603`), which mixes conventions. Verified empirically before
writing the rewrite (a throwaway probe fed a representative `mpc_cmd` dict through
both `motor_guard`'s derivation and the new pump and asserted `u0/u1/u2/v0` match
bit-for-bit, in both the `motor_rev`-present and ext-fallback paths):

- **u0** = `msg['motor_rev']` (ODrive convention, 0 = STOW, **includes** any stow
  offset) if present, else `ext_mm × mm_to_rev` (the unit-test/sim fallback, no
  offset). Using `motor_rev` *verbatim* when present is the bumpless-critical
  choice — falling back to `ext × mm_to_rev` would jump the leg by the stow offset
  the instant the source flips. (In this codebase's `HardwarePlant` the offset
  happens to be 0 — `motor_rev = ext × mm_to_rev`, `hardware_plant.py:413` — but
  the pump uses `motor_rev` regardless, to match `motor_guard` and survive any
  future offset.)
- **u1** = `cmd_next_mm × mm_to_rev`; **u2** = `cmd_next2_mm × mm_to_rev`
  (extension convention, no offset). Absent / non-finite / wrong-length ⇒ **clear**
  the corresponding `HAS_U1`/`HAS_U2` flag — never a NaN sentinel (firmware
  handoff D4). `u2` is only meaningful with `u1` (it sets the Hermite endpoint
  velocity `v1 = (u2 − u1)/T`), so `HAS_U2` is gated on `HAS_U1`.
- **v0** = `vel_mm_s × mm_to_rev` (= `conversions.leg_velocities_to_motor_velocities`,
  a pure elementwise scale).
- **accel** = 0 — the firmware Mode-1 Hermite does NOT read `accel`
  (`teensy_interp.py:122–131`); only the Mode-2 Taylor fallback does, and
  production always supplies `cmd_next_mm` so Mode 1 always runs.
- **torque_ff** = 0 — the friction-FF drop (D9).

The bumpless chain closes in two links: (1) **pump knots == motor_guard knots**
(this entry's parity test), and (2) **firmware Hermite == motor_guard interp for
the same knots** (the pre-existing `hermite_xref` cross-check, < 1e-6 rev). Same
knots → same interpolated trajectory the α relay produced → no position jump.

**Geometry injection.** The pump was pure (imported only `math`/`typing`/
`protocol`). It now needs `mm_to_rev`; injected as a **constructor arg**
(`hw.GEOM_MM_TO_REV`) rather than a per-build param or a `jugglebot.motion`
import — `leg_velocities_to_motor_velocities` reduces to `× mm_to_rev`, so only
the vector is needed and the module stays pure.

**Operator decisions (2026-06-25):**
1. **Bypass motor_guard for the legs** (not "remove this session"). The bypass
   leaves `motor_guard`'s `:5556` output unconsumed; ZMQ PUB/SUB tolerates the
   idle subscriber, so removing `motor_guard` from `jugglebot_launch.py` is a
   deployment follow-up (U5/cleanup), not required for the U4 code switch.
2. **Do the `/teensy` rename in U4.** Promote the leg/hand `/teensy/*` topics +
   services to production names — they were only namespaced to coexist with
   `can_node`, which is out of the production launch.

## Implementation

- **`controller/teensy_link/setpoint_pump.py`** — `SetpointPump.build()` rewritten
  to derive β knots from the `:5557` `mpc_cmd` dict per the convention above;
  `mm_to_rev` added to the constructor; per-step `JB_OP_MAX_POSITION_STEP_REV` gate
  (now on `u0`) + NaN/Inf reject preserved. A missing/non-finite `vel_mm_s` is a
  SAFETY reject (the pump has no per-tick ext history to finite-difference, and a
  wrong feedforward velocity is unsafe); a non-finite lookahead clears the flag
  (not a reject).
- **`teensy_bridge_node.py`** — `_MotorGuardSetpointSource` → `_MpcCommandSetpointSource`
  (SUB `:5557`, `b'mpccmd'` topic filter — the port also carries the
  HardwarePlant fallback-enable message, which the filter excludes; zmq/msgpack
  stay lazily imported). Pump constructed with `hw.GEOM_MM_TO_REV`. The
  `mpc_active` / `enable_setpoint_output` (default false) gate is untouched.
- **`/teensy` rename** — 4 publishers (`robot_state`, `hand_telemetry`,
  `link_status`, `profile`), 5 services (`clear_errors`, `reboot_odrives`,
  `encoder_search`, `home`, `odrive_command`), 1 subscription
  (`set_motor_vel_curr_limits`) dropped the prefix. Stale docstrings/comments
  updated repo-wide; `teensy_bridge_launch.py` gained a dual-launch caveat.

## Verification

- **Bumpless parity (the safety invariant):** `tests/firmware/test_hermite_xref.py::
  test_beta_pump_knots_match_motor_guard` asserts the pump's `u0/u1/u2/v0` equal a
  live `MotorGuard`'s latched base (motor_rev-present with a synthetic +0.5 rev
  offset, ext-fallback, `cmd_next2`-absent, and over-long-lookahead paths),
  bit-exact (`atol=1e-12`). The interp cross-check `xref.py` still reports
  **0.000e+00 rev** (threshold 1e-6), so the firmware Hermite still matches
  `motor_guard`'s interp for the β knots. *(Hardening from the holistic U4 audit,
  `123b386`: the pump's length gates were tightened `>= n` → `== n` to mirror
  `motor_guard`'s `shape == (6,)` contract exactly — a malformed >6-element
  lookahead now clears the flag (Taylor fallback) rather than silently taking the
  first 6 and emitting a divergent-but-accepted β frame; the parity test's
  7-element path pins that both the pump and a live `MotorGuard` clear it.
  Unreachable on the real wire today — `make_mpc_command` sends exactly-6 — but it
  closes a latent bumplessness-divergence class.)*
- **Pump unit tests** (`tests/teensy_link/test_setpoint_pump.py`, rewritten for the
  MPC-dict input): field mapping, `motor_rev`-preferred-over-ext (no stow jump),
  flag clearing, `torque_ff` always zero (D9), per-step gate on `u0`, the safety
  rejects.
- **Node wiring** (`tests/ros/test_teensy_bridge_node_setpoint.py`): the
  `_MpcCommandSetpointSource` decodes the real `:5557` wire format and the
  `b'mpccmd'` filter excludes a non-mpccmd frame; the default-disabled / link-down
  / step-gate safety gates hold.
- **Rename:** namespace-discipline test inverted (`test_no_publisher_under_teensy_
  namespace`); new `test_leg_hand_services_use_production_names` pins the 5 services
  + subscription (the conftest mock now records service/subscription names) so a
  service-name typo can't silently disconnect the orchestrator. `grep` confirms no
  functional `/teensy/*` topic/service string remains (count → 0).
- **`enable_setpoint_output` stays default-false** — U4 arms nothing.
- **Full suite:** `pytest tests/ -q` (run 2026-06-25): **1817 passed, 1 xfailed in
  435.78s**. (A prior run flagged `test_hot_loop_allocation_contract` — a
  `tracemalloc`-based per-tick allocation-rate threshold on the MPC 40 Hz hot loop,
  GC/order-sensitive; it passed in isolation and on the clean re-run. U4 touches no
  sim/MPC hot-loop file, so this is pre-existing flakiness, not a regression.)
- **Audit:** two independent adversarial audits (audit-reporter) — one on the α→β
  diff, one on the rename diff — both **CLEAN**. The α→β audit verified the
  convention mapping against `motor_guard` line-by-line and `accel=0` against
  `teensy_interp.py`. The rename audit confirmed completeness against can_node's
  historical names and flagged a pre-existing service-name test gap (addressed) and
  a pre-existing `home`-service/`home_motors`-action mismatch (out of scope, filed).

## Discussion

*Why this approach beat the alternatives — the parts a future reader can't infer
from the diff.*

**Why bypass `motor_guard` entirely, rather than keep it as a Jetson-side safety
pass-through.** The tempting middle path was to keep `motor_guard` in the loop as a
read-only safety filter (workspace/deviation/E-STOP) while the Teensy does the
interpolation. We rejected it because it re-introduces the exact coupling U4
exists to remove: `motor_guard`'s value *is* its 500 Hz Hermite + friction-FF, and
once the Teensy owns the interpolation, a Jetson-side `motor_guard` would either
(a) re-interpolate (defeating the offload) or (b) run its safety checks at 40 Hz on
the raw MPC command — which the **MPC itself already enforces** (the coupled
workspace is an MPC constraint, not a `motor_guard` invention). Keeping a degraded
`motor_guard` would be a second authority computing a weaker version of a check the
MPC already owns, adding a process, a hop of latency, and a second place for the
two to disagree. The clean split is: **the MPC owns the coupled-workspace
feasibility; the Teensy owns the per-leg hardware-protection envelope** (stroke,
deviation, MPC-staleness E-STOP, deferred-stow). Both were validated under powered
motion in U3.

**The tradeoff accepted: the lost Jetson-side coupled-workspace backstop.** Under
the α relay, `motor_guard` ran a *second*, independent coupled-workspace check on
the Jetson — a backstop if the MPC ever commanded outside the workspace. The β path
removes that backstop: workspace feasibility now rests solely on the MPC. This is a
real reduction in defence-in-depth, accepted deliberately because (a) the MPC's
workspace constraint is the *authoritative* model (motor_guard's was a copy), (b)
the per-leg stroke clamp on the Teensy still catches any single-leg overrun
(hardware-protective, the binding safety bound), and (c) the bench driver's
per-step gate (ported into the pump) still catches command-stream discontinuity.
What is genuinely gone is the *cross-leg coupled* check at 40 Hz; U5's powered
six-leg validation is where that loss must be confirmed harmless under real
multi-leg motion.

**Why `motor_rev` verbatim for u0, even though the offset is 0 today.** The audit
and the probe both confirmed `motor_rev == ext × mm_to_rev` in the current
`HardwarePlant` (no stow offset), so `ext × mm_to_rev` would work *today*. We use
`motor_rev` anyway because (a) it matches `motor_guard` exactly — the bumpless
guarantee is "reproduce motor_guard's latch," not "reproduce it under today's
config" — and (b) it is robust to a future config that re-introduces a stow offset,
where `ext × mm_to_rev` would silently jump the leg by the offset at the switch.
The cost is nil; the failure it forecloses is a silent stow-offset discontinuity.

**Why the rename rides in U4 and is corrective, not cosmetic.** The `/teensy/*`
namespace existed only to let the bridge run *side-by-side* with `can_node` without
dual-publisher conflicts (handoff D1). With `can_node` out of the production launch,
that risk is moot — and the side effect is that the GUI (`robot_state`,
`hand_telemetry`) and the orchestrator (`encoder_search`, `odrive_command`) were
subscribing to the bare production names the bridge *wasn't publishing*, so they
were silently disconnected from the bridge. Dropping the prefix reconnects them. We
landed it as a separate commit (`50fc8fe`) from the α→β switch (`cb0d158`) for clean
rollback granularity — different concerns, no shared hunks.

## Open Questions / Next

- **U5 — powered six-leg validation (the next operator sitting).** Flip
  `enable_setpoint_output=true` on the rewired six-leg rig and validate the β
  production path under real multi-leg motion: confirm the lost Jetson-side
  coupled-workspace backstop is harmless (the tradeoff above), measure the
  disarm-to-output-gate latency directly (the ~100–200 ms estimate from the
  rollback note was never characterised), then decommission. Removing `motor_guard`
  from `jugglebot_launch.py` is the U5/cleanup deployment step.
- **Pre-existing, filed (not U4 scope):** the `home` Trigger service has no
  production caller — the orchestrator homes via a `home_motors` *action*
  (`orchestrator_node.py`), so end-to-end homing needs a `home_motors` ActionServer
  on the bridge, not the `home` service. Unchanged by this rename (equally true at
  `/teensy/home`).
- **Carried from U3 (open):** Finding A (the rare feedback freeze) root cause;
  Finding C's deferred deterministic pre-arm sole-authority guard.
