---
title: Phase 11 bench cutover — present-axis scoping + synthetic β-knot interp driver (software prep)
type: feature
date: 2026-06-24
status: in-progress
phase: "11"
related_plan: teensy-can-offload.md
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/axis_state.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - controller/teensy_link/synthetic_setpoint.py
  - tests/hardware/teensy_setpoint_bench.py
  - tests/firmware/test_fault_logic.py
  - tests/teensy_link/test_synthetic_setpoint.py
commits:
  - 4d69ac7
  - 54aa928
  - d90e365
subsystem:
  - can
  - controller
  - motion
tags:
  - safety
  - testing
---

# Phase 11 bench cutover — present-axis scoping + synthetic β-knot interp driver

**Summary.** Phase 11 is the convergence point of the teensy-can-offload leg path:
the first *continuous powered* MPC-style setpoint stream to a leg over the
can-bridge (the plan's Phases 5/7/8 powered tails collapse into one armed bench
sitting). This entry covers the **desk-side software prep** that makes that sitting
safe and reproducible — landed and flashed 2026-06-24, software-complete — and is
the live record we extend as the hardware validation proceeds. Two units landed:
**U1**, a firmware *present-axis* contract that lets the can-bridge run correctly on
a subset-populated CAN3 (the single-leg bench rig has only odrv0 present); and
**U2**, a synthetic β-knot setpoint source + armed bench driver that exercises the
Teensy's 40 Hz-knot Hermite interpolator on hardware with a known, bounded command,
decoupled from `motor_guard` / the MPC / friction-FF. **U3 armed validation
stages (i) approach+hold and (ii) bounded sinusoid now PASS on hardware** (see
*Hardware validation*) — the leg tracks the Teensy 40 Hz-knot Hermite cleanly; the
fault-replay + real-MPC tails remain.

## Motivation

Starting Phase 11 surfaced that the migration's headline benefit — moving the
500 Hz interpolation off the Jetson onto the Teensy — is **not realised by the
deployed code**, by deliberate design, and that two things had to be settled before
arming a powered leg:

1. **α relay vs β knots.** The deployed bridge forwards `motor_guard`'s
   *already-interpolated* 500 Hz output (`:5556`, `flags=0`), so the Teensy interp
   runs in pass-through (Mode-2 Taylor over a ~2 ms gap) and `motor_guard`'s 500 Hz
   Hermite still runs on the Jetson — the **α relay**. The plan's "40 Hz knots →
   Teensy Hermite" design is the **β path** (the firmware already implements it:
   `leg_interp.cpp` Mode-1 Hermite fires whenever `flags` carries `has_u1`). α was
   chosen at 10b to (a) preserve `motor_guard`'s per-leg Stribeck friction-FF and
   (b) stay byte-comparable to legacy `can_node`. So the interp-offload is
   *deferred*, not delivered.
2. **Single-leg-rig hazards.** The firmware looped all six legs unconditionally,
   but the bench rig has only odrv0 on CAN3 — which would stream phantom frames to
   absent nodes, dead-lock the deferred-stow reconnect, and (with the production
   MPC's six leg targets) false-trip the `MAX_DEVIATION` E-STOP.

## Design

**Couple the software, stage the hardware (U1..U5).** The hardware validation (arm
one leg → six legs → decommission) is irreducibly serial behind a powered operator
sitting; the software offload is one coupled desk unit. The remaining cutover was
restructured accordingly (full table in `plans/active/teensy-can-offload.md`,
"Software-offload vs hardware-cutover split"): **U1** present-axis firmware scoping,
**U2** synthetic bench driver — both desk-side, landed here; **U3** the operator
armed run that *measures* the float32 residual + the D9 motion-onset penalty; **U4**
the production α→β switch + the D9 friction-FF decision, gated on U3's data; **U5**
six-leg rewire + decommission.

**U1 — one present-axis predicate.** `leg_present(i) == axes[i].heartbeat_seen`
(latched-once by the CAN3 RX decode, never cleared → monotonic) is the single
enforcement point. It gates the two *streaming* per-leg CAN3 fan-outs (the 500 Hz
interp setpoint TX and the stow-descent TX), scopes the deferred-stow reconnect
predicate (`all_legs_heartbeats_fresh` → `all_present_legs_fresh`), and skips absent
legs in the `MAX_DEVIATION` loop. Low-rate operator/fault one-shots (CLEAR_ERRORS,
REBOOT, stow-complete IDLE) are deliberately *left ungated* — they must still reach
a configured-but-silent or wedged leg, and odrv0 ACKs them bus-wide anyway. Every
change is a strict no-op on the full six-leg robot.

**U2 — synthetic β-knot source + armed driver.** A pure generator
(`synthetic_setpoint.py`, the `homing.py` pattern) emits `flags=0x3` knots
(`u0/u1/u2` + `v0`) for a single axis: a smooth raised-cosine approach off the
hardstop into the workspace, then hold or bounded sinusoid; bounds-validated
(envelope ⊂ workspace, peak per-frame step ≤ lead clamp); absent axes packed 0.0.
The armed driver (`teensy_setpoint_bench.py`, the `teensy_home_bench.py` pattern)
keeps `mpc_active=0` until an explicit operator ARM gate, is the sole wire
authority, commands the live encoder position in its first frame (no step at arm),
disarms instantly on fault/deviation/Ctrl-C, and always disarms in a `finally`.

## Implementation

- `axis_state.h`: `leg_present(i)` inline predicate.
- `leg_interp.cpp`: present-gate the interp setpoint TX (`:275`) and stow-descent
  TX (`:185`).
- `fault_machine.cpp`: `all_present_legs_fresh()` (skip absent legs), and
  `if (!leg_present(i)) continue;` in the `MAX_DEVIATION` loop.
- `controller/teensy_link/synthetic_setpoint.py`: the pure generator.
- `tests/hardware/teensy_setpoint_bench.py`: the armed single-leg driver (out of
  pytest collection — needs hardware).
- `tests/firmware/test_fault_logic.py` (+6 cases), `tests/teensy_link/test_synthetic_setpoint.py`
  (16 cases): the off-hardware spec.

## Discussion

*Why this approach beat the alternatives — the parts a future reader can't infer
from the diff.*

**Heartbeat-derived `leg_present`, not a compile-time mask.** A scope-assessment
pass recommended a compile-time/bench mask for determinism. We chose
`heartbeat_seen` instead because it is the *correct abstraction for the actual
failure class*: "never stream to a node we haven't heard from" auto-prevents
phantom TX to **any** absent or dead node — the single-leg bench rig *and* a
full robot with one dead leg — with no separate build that could ship to the robot
by accident, and no protocol surface. The feared non-determinism is only a
power-on transient (a present leg before its first heartbeat), and arming always
happens post-telemetry, so at arm time every present leg already reads true. The
predicate is monotonic (latched-once, never cleared), so it is stable mid-run — a
momentary heartbeat gap does *not* drop a leg from "present" (that is the
deferred-stow's job, not the TX gate's).

**The "phantom TX → bus-off" risk was overstated — corrected here.** The scope
pass flagged the 5 phantom frames/tick to absent nodes 1–5 as a CAN3 bus-off
hazard. That is wrong while odrv0 is powered: CAN ACK is *bus-level, not
address-filtered*, so odrv0 ACKs every well-formed frame on the bus, including
those addressed to absent nodes. The 100 Hz 0x7DD time-sync broadcast already
proved this — it ran on CAN3 through the entire 9a/9b bring-up with odrv0 the sole
node and the bus stayed at fault-conf 0 (21.3 V, `tec_max` flat). So U1's real
value is **removing 2500 frames/s of waste + fixing the stow-recovery dead-lock +
production dead-leg robustness** — not preventing a bus-off. Recording this so the
next reader doesn't re-derive a non-existent hazard. (Bus-off *would* occur only if
odrv0 itself dropped — and then the leg isn't being driven anyway.)

**The deferred-stow reconnect dead-lock is the load-bearing single-leg fix.** The
phantom TX is cosmetic; the dead-lock is not. `all_legs_heartbeats_fresh` required
*all six* legs fresh to confirm a CAN-loss reconnect, which is never true with five
absent legs — so a stow armed at CAN-loss *detection* could never *execute*. That
makes the Phase-8 CAN-loss safety-inversion replay (a U3 step) unobservable on the
single-leg rig. Scoping the predicate to present legs is what makes that test
possible, and it is a strict no-op on the full robot (all six present → still
requires all six fresh). The boolean inversion is the subtle part: old `!seen ||
stale → false` becomes new `present && stale → false`, so an absent leg is now
*skipped* rather than forcing the whole predicate false.

**α→β and friction-FF (D9) are one diff, gated on a hardware measurement.** The α
source (`:5556`) carries friction-FF in `leg_torques`; the β source (the MPC
`:5557` stream) carries `torque_Nm = zeros`. So re-pointing the source *is* the
friction-FF-drop edit — they cannot be split. And whether the loss is acceptable
can only be judged from a *powered* measurement (motion-onset penalty + the float32
interp residual under armed motion), which is exactly what U3 produces. This is why
the production switch (U4) is the one piece of the "coupled software" that genuinely
cannot be written ahead of the hardware: not because testing-after is less careful,
but because the diff's central decision is data-gated. **Pre-registered D9
criterion (set 2026-06-23):** accept the friction-FF loss as a logged interim if the
motion-onset penalty is **≤ 40 ms** (tunable) and the float32 residual is within
tracking noise; otherwise port friction-FF to the 500 Hz ISR (it is a function of
the *interpolated* velocity, so a once-per-MPC-step Jetson compute would be wrong).

**The homed leg starts *below* its workspace — there is no zero-motion hold.**
Homing leaves the leg at the retracted hardstop (≈ −0.10 rev), below the stroke
floor 0.0709 rev. Any armed setpoint is stroke-clamped up to ≥ 0.0709, so the first
armed motion is necessarily a controlled *extension* off the hardstop into the
workspace — not the "armed hold (zero net motion)" first stage originally
envisaged. The driver does this as a smooth approach ramp, and the first frame
still commands the live encoder position (≈0 deviation at arm), so there is no step
or false E-STOP at the moment of arming. This reframing matters for the U3 staging
and is why stage (i) is "approach + hold," not "hold in place."

**Isolation via worktree.** A parallel session was concurrently editing the same
firmware tree (the BB command-outcome channel). Per the project's
parallel-session/worktree discipline, all of this work was done in an isolated
`git worktree` on a separate branch, then cherry-picked onto
`phase-a-plus-accuracy-cal` once the parallel work was committed — no clobbering,
conflict-free (zero file overlap). The merged firmware was rebuilt and the
interlock my code depends on (the `.ino` `mpc_active` heartbeat decode, the
unchanged `Setpoint` frame) re-verified before flashing.

## Verification

- **Firmware build (merged tree):** `pio run` green, dec **354560** (= the BB
  command-outcome work + this change; the present-axis scoping is itself
  dec-neutral). My isolated-branch build was dec 353344; the +1216 is the BB work.
  Changed objects confirmed recompiled (not stale).
- **Off-hardware tests:** `pytest tests/ -q` (merged tree, run 2026-06-24):
  **1769 passed, 1 xfailed in 443.34 s** — no failures. The 22 tests new in this
  change (6 fault-logic present-axis cases, incl. a single-leg stow-recovery
  end-to-end pinning the dead-lock fix; + 16 synthetic-source cases, incl. analytic
  velocity = finite-difference, bounds rejection, wire round-trip) sit on top of the
  BB-merged baseline.
- **Audit:** independent adversarial audit (audit-reporter) of the full change set
  returned **CLEAN** — no BLOCKING/WARNING findings; two benign NOTEs (sibling
  all-leg loops verified no-op for absent legs; a pre-existing non-atomic tx-seq in
  `client.py`, diagnostic-only).
- **Flash:** dec-354560 firmware flashed to the can-bridge Teensy 2026-06-24
  (HalfKay → programmed → booting); bridge healthy on the link post-flash
  (3/3 ping, ~2.4 ms RTT). Boots `mpc_active=0`, output gated off — no motion on
  flash.

## Hardware validation (U3 stages i–ii, 2026-06-24)

**Stages (i) approach+hold and (ii) bounded sinusoid PASS on the standalone leg
(odrv0)** — the first continuous powered MPC-style leg motion through the
can-bridge. Stage (i): the leg tracked a smooth approach to 0.15 rev (err ≤ 0.031
rev the whole descent), settled to +0.0007 rev, clean disarm. Stage (ii):
approached 0.30 rev then oscillated 0.20↔0.40 at 0.25 Hz for 20 s, err mostly
≤ 0.03 rev (small lag at the velocity peaks, as expected), `fault_state=NONE`
throughout. The `[guard]` diagnostic confirmed the full interlock live:
`mpc_active=1 guard_mode=ENABLED output=1`, `sp_age≈13 ms`, `u0` tracking the
trajectory. **The Teensy 40 Hz-knot Hermite (Mode 1) ran on hardware** — the
interp-offload, exercised for real. Getting here surfaced three things.

**Finding B — the feedback-staleness guard was defined but never wired.** The
*first* armed attempt (pre-guard) aborted at t≈0.6 s: the encoder + iq froze at
identical values while the command kept descending — a frozen feedback loop, not a
stuck leg (a stuck leg builds current; iq sat dead). The 0.30-rev deviation belt
caught it (the command was moving *away* from the frozen encoder), which exposed
the gap: `canbridge_config.h` *defined* `MOTOR_FB_STALENESS_US = 0.15 s` (a copy of
motor_guard's `MOTOR_FB_STALENESS_S`) yet **nothing used it** — and a freeze during
a *hold* (cmd ≈ frozen enc, no growing deviation) would have gone unnoticed.
`evaluate_guard` now suppresses output (recoverable, not a latched E-STOP) and
reports `FaultState.MOTOR_FB_STALE` when a present leg's
`now − pos_timestamp_us > 0.15 s`, mirroring motor_guard. Commit `155a06e`.

**Finding A — the freeze itself is rare, now mitigated; root cause open.** That
t≈0.6 s feedback freeze did **not** recur across the two clean stage runs, so it is
intermittent. CAN3 was healthy at the time (`tec=0 rec=0 synced=1`), ruling out
bus-off; the TX-in-ISR race is already IRQ-serialised, so a corrupted setpoint is
not it. The live candidates are an RX-task starvation (axis cache froze) or a
UDP-uplink stall. Left open — but the Finding-B guard now catches it
deterministically at 0.15 s if it recurs, so the system is safe regardless.

**The "leg won't arm" gotcha — a competing heartbeat authority.** Attempts 2–4
(post-guard) showed the leg in CLOSED_LOOP but motionless with `iq≈0` and live
telemetry. A `[guard]` firmware diagnostic line (`mpc_active / guard_mode / output
/ sp_age / u0`) made it instant: `mpc_active=0` throughout — the firmware never
armed, so the output gate stayed shut even though the driver's setpoints were
arriving (`sp_age≈1 ms`, `u0` updating). Root cause: a running ROS2 launch — its
`teensy_bridge_node` sends J→T heartbeats with `flags=0` (mpc_active pinned off)
*unconditionally*, and the firmware applies the **last** heartbeat it sees, so the
bridge's `flags=0` kept overwriting the bench driver's `flags=1`. The bench driver
must be the **sole wire authority**; the fix is operational (stop the ROS2 launch),
after which the leg tracked perfectly. To close the footgun, the firmware now
**uplinks `mpc_active` in the T2J heartbeat (bit3)** and the bench driver verifies
its arm took ~0.7 s after ARM, aborting with an explicit *"another heartbeat
authority is overriding — stop the ROS2 launch"* instead of a cryptic
tracking-deviation timeout.

**Separate bug — `motor_guard` crashes on a bad config path.** Surfaced while the
launch was up: `motor_guard` dies at startup with `FileNotFoundError` for
`.../install/jugglebot/config/hardware_config.yaml` (`friction_ff_params.py:90`
resolves the path under the *install* tree, where `config/` isn't installed). The
bench driver doesn't use motor_guard, so it didn't block stages i–ii — but it
**will** block the real-MPC path (U3 stage iii). Fix before stage iii.

## Open Questions / Next (U3 tail — operator-gated, powered)

Stages (i) + (ii) done (above). Remaining, each gated by explicit operator
final-say + e-stop:

3. **Powered fault-replay** — induced J→T link drop + CAN3 drop with the leg armed:
   confirm the MPC-staleness E-STOP and the deferred-stow safety inversion (the
   reconnect path U1 unblocked). Validating the deferred-stow on the single-leg rig
   relies on U1's present-scoped `all_present_legs_fresh`.
4. **Real-MPC trajectory** — needs the `motor_guard` config-path bug fixed first.
   Produces the **float32 interp residual** and the **D9 motion-onset penalty** that
   decide U4 (the production α→β switch + friction-FF disposition) against the
   pre-registered ≤ 40 ms criterion; compare-to-legacy "within tracking noise."

Also open: **Finding A** (the rare feedback freeze) root cause; the **`motor_guard`
config-path bug**. **Rollback note (corrected):** `can_node` is now reference-only
(the Jetson no longer sees CAN directly), so the cutover abort is **e-stop +
`mpc_active=0` disarm + power-down**, *not* a `<10 min` socketcan swap-back — the
bench driver's instant disarm (Ctrl-C / fault) gates firmware output off in one
heartbeat.
