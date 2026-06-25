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
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - controller/teensy_link/synthetic_setpoint.py
  - controller/teensy_link/replay_setpoint.py
  - controller/teensy_link/protocol.py
  - config/generate_udp_protocol.py
  - ros_ws/src/jugglebot/jugglebot/motion/friction_ff_params.py
  - ros_ws/src/jugglebot/setup.py
  - tests/hardware/teensy_setpoint_bench.py
  - tests/firmware/test_fault_logic.py
  - tests/teensy_link/test_synthetic_setpoint.py
  - tests/teensy_link/test_replay_setpoint.py
  - tests/motion/test_friction_ff_params.py
  - tests/motion/test_motor_guard_friction_ff.py
commits:
  - 4d69ac7
  - 54aa928
  - d90e365
  - 155a06e
  - 33b4ad5
  - ada4a1b
  - 844e68e
  - a19b68c
  - c584767
  - b06699f
  - a277274
  - 2443ff6
  - a37bc05
  - ac66acf
  - 930f70e
  - 489b495
  - 7f88721
  - b6d6392
  - a5d5b7e
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
*Hardware validation*) — the leg tracks the Teensy 40 Hz-knot Hermite cleanly. The
fault-replay **link-drop sub-test (A)** then confirmed the firmware MPC-staleness
gating, the `motor_guard` config-path crash was fixed (`c584767`), and the U3-iv
real-MPC residual tooling — a `LegCmd` firmware uplink (`b06699f`) + a recorded-throw
replay source (`a277274`) — was built desk-side. The **CAN3 sub-test (B) then PASSED
on hardware** (2026-06-25): the Phase-8 deferred-stow safety inversion fired on
confirmed reconnect with a measured descent peak of 2.472 rev/s ≤ the 2.5 rev/s cap
(see *sub-test B + recovery*). Finally **U3-iv** measured the two D9 quantities on a
time-stretched replay (the full-speed throw is current-limited at the onset on the
bench leg): **float32 residual 5.5e-7 rev** and a **null motion-onset penalty** from
dropping friction-FF (the smooth gate suppresses FF at v≈0, so its loss is free at
breakaway). Both within criteria → **D9 = accept the friction-FF loss**, green-lighting
**U4** (the production α→β switch). U3 is complete (see *U3-iv real-MPC residual + D9*);
U4/U5 remain, so this entry stays `in-progress` as the live Phase-11 record.

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
tracking-deviation timeout. (Later corrected — see *Finding C* under *U3 tail —
continued*: the bridge does **not** win "unconditionally"; it is a non-deterministic
phase race, so this check only guards the driver-loses side.)

**Separate bug — `motor_guard` crashes on a bad config path.** Surfaced while the
launch was up: `motor_guard` dies at startup with `FileNotFoundError` for
`.../install/jugglebot/config/hardware_config.yaml` (`friction_ff_params.py:90`
resolves the path under the *install* tree, where `config/` isn't installed). The
bench driver doesn't use motor_guard, so it didn't block stages i–ii — but it
**will** block the real-MPC path (U3 stage iii). Fix before stage iii.

## U3 tail — continued (2026-06-24): motor_guard fix, fault-replay sub-test A, U3-iv desk build

Same day, continued: the `motor_guard` config-path bug was fixed, U3 stage (iii)
fault-replay began (the J→T link-drop **sub-test A** — firmware gating confirmed),
and the U3-iv real-MPC residual tooling was built desk-side. The CAN3 drop
(sub-test B), the recovery, the undervoltage observe, and the U3-iv hardware run
are **blocked** on a Jetson `eth0` kernel hang that the link-drop method itself
triggered (below) — the operator is recovering it in a parallel session.

**`motor_guard` config-path fix (Fix A — ament data file).** The bug logged above:
`friction_ff_params.py` resolved `hardware_config.yaml` by a rigid five-level
`__file__` walk that only works from the source tree; under the colcon install
tree it pointed at the never-installed `install/jugglebot/config/` →
`FileNotFoundError` at `MotorGuard.__init__`, crashing the production launch (and
blocking the U3-iv real-MPC path). Fixed by installing the YAML into the ament
*share* dir (`setup.py` `data_files`, a package-relative source — colcon rejects
absolute `data_files`) and resolving via env-override → `get_package_share_directory`
→ source-tree fallback → fail-fast. Verified on the install tree: the resolver
returns the share copy and `load_params()` succeeds. **Contract change, recorded:**
because `ament_python` *copies* `data_files` at build time, an in-place friction
re-tune now takes effect only after a `colcon build` — OR via the
`JUGGLEBOT_HW_CONFIG` env override pointed at the source file (the supported
tune-without-rebuild path). The module docstring was updated to state this. *Why
Fix A over source-tree resolution:* the operator chose the canonical ament idiom;
the env-override escape hatch preserves in-place tuning without re-introducing the
original silent-stale-config hazard (an explicit override path always wins). Commit
`c584767` (5 resolver tests; `pytest tests/ -q`, run 2026-06-24: 1779 passed,
1 xfailed).

**Finding C — the competing-authority is a non-deterministic heartbeat race, not
"the bridge always wins."** The verify-armed check (added with Finding B's
mpc_active uplink) was *expected* to fire: run the bench driver *with* the ROS2
launch up and watch it abort with "another heartbeat authority is overriding." It
did **not** — the leg **armed and tracked** (the driver's `flags=1` won the race),
`fault=NONE`, ran to completion. This **contradicts this entry's own earlier claim**
(the "leg won't arm" gotcha above) that the bridge `flags=0` overwrote the driver's
"*unconditionally*." Corrected: `teensy_bridge_node` and the bench driver **both**
send J→T heartbeats at 10 Hz and the firmware applies the *last one received*, so
which wins is a **phase race** — non-deterministic. The earlier runs likely had the
bridge winning partly *because* `motor_guard` was crashing then (the Fix-A bug),
perturbing the bridge's heartbeat cadence; with `motor_guard` running cleanly the
driver won. *Safety implication:* the verify-armed check only guards the
*driver-loses* side (a clear error when the firmware won't arm); when the driver
*wins*, the leg arms with the launch still live — "sole wire authority" is enforced
only by operator discipline (stop the launch), not by the check. This run was
harmless only because `center ≈ current` (≈0.002 rev of motion) and the bridge
sends no setpoints with output gated. The robust fix — a **deterministic pre-arm
guard** that refuses to arm unless the firmware reads `LINK_LOST` (no other J→T
authority) before the driver sends its first heartbeat — was **deferred** by the
operator (open item). (`git log` confirms `teensy_bridge_node` *is* in the default
`jugglebot_launch.py`; the `setup.py` comment claiming otherwise is stale.)

**U3-iii sub-test A (J→T link drop) — firmware gating confirmed; one hypothesis
withdrawn.** Launch stopped (driver sole authority — confirmed by the firmware
reading `LINK_LOST` before arm), leg armed to a clean hold at 0.30 rev, then the
operator physically unplugged the J↔T Ethernet. Captured on the Teensy USB serial
(which survives the unplug — separate cable), the firmware responded to spec:
`fault → MPC_STALE`, `guard_mode → DISABLED`, `output = 0` (gated off); the leg
**held in CLOSED_LOOP**. *Withdrawn hypothesis:* I first read the serial `s8 → s1`
(CLOSED_LOOP → IDLE) transition at ~58 s post-unplug as the ODrive's own
control-loop watchdog disarming the leg, and wrote it up as a finding ("the leg
holds only until the ~58 s watchdog"). The operator's physical observation overrode
it — *the leg held the whole time until they manually IDLE'd it via the GUI* — so
the `s1` was the GUI action, not a watchdog. The simpler, correct result stands: on
a sustained link loss the firmware gates output and the leg holds (odrv autonomous).
This is the canonical "abandon the hypothesis when the next data point — here the
operator's eyes — doesn't fit it."

**Methodology finding — do NOT physically unplug the USB-Ethernet dongle for the
link-drop test.** The unplug triggered the Linux kernel's `unregister_netdevice:
waiting for eth0 to become free` refcount-leak hang (`Usage count = 39` — the many
UDP sockets bound to the dongle's interface at the moment of unplug:
`teensy_bridge_node`, the bench-driver runs, the read-only probes). The leak does
not drain on its own and the dongle does not re-enumerate cleanly — it needs a
reboot. The leg stayed safe throughout (gated, held), but the J↔T link was lost for
the rest of the session, blocking sub-test B + the recovery + the undervoltage
observe. **Use `sudo ip link set eth0 down` … `up` instead** — identical link-drop
semantics (UDP stops → `MPC_STALE`), no netdev teardown, instant clean restore. The
CAN3 unplug (sub-test B) is unaffected (CAN3 is not a netdev with this issue).

*Confirmed + accepted as a known weakness (operator, follow-up testing).* The
operator independently retried recovery paths and confirmed the conclusion: a
**physical** unplug of the USB↔Ethernet adapter is **unrecoverable without a Jetson
reboot** (the `unregister_netdevice` refcount leak does not drain, the dongle does
not cleanly re-enumerate). This is accepted as a documented operational weakness
rather than chased further, on the rationale that the USB↔Ethernet connector sits in
a physically safe location and is never disturbed during normal operation — so the
real-world trigger (an accidental yank of *that* connector) effectively never fires.
The mitigation for *testing* link drops stands (use `ip link down/up`); the physical
unplug is simply out of scope to recover from in software.

**U3-iv desk-side tooling — built + committed; flash + run pending eth0.** With the
hardware blocked, the U3-iv (real-MPC residual + motion-onset) tooling was built
off-hardware:
- *Recording + scaling (confirmed with the operator):* `mpc_20260330_165611.csv` —
  axis-0 range [0.079, 3.052] rev, peak 3.18 rev/s. It already fits the bench leg's
  ≈3.4 rev physical ceiling (the firmware `STROKE_MAX_REV[0] ≈ 3.90` is the
  *production* value and would NOT protect the shorter bench leg), so **no scaling
  is applied**; a safety scaler (`scale_to_bench`: a 3.30 rev position ceiling,
  0.10 below the ≈3.4 rev physical limit; + a 3.5 rev/s velocity guard, below the
  4.0 rev/s `ODRIVE_LEG_VEL_LIMIT_RPS` the bring-up sets) is a no-op here but bounds
  any recording. The 7.94 rev/s recordings were
  rejected — they'd saturate the velocity limit → tracking deviation → abort.
- *`RecordedThrowSource`* (`replay_setpoint.py`, mirrors `SyntheticKnotSource`):
  approach → settle → replay, so the throw onset breaks away from a true rest
  (clean to measure); an optional dependency-injected `torque_ff_fn` for the later
  D9 onset A/B. 19 unit tests + an offline trajectory preview. Commit `a277274`.
- *`LegCmd` uplink (Path B — the operator chose the direct on-Teensy residual):* the
  telemetry uplinks only encoder pos, not the Teensy's *commanded* float32 interp
  output, so the on-Teensy residual could not be measured on the wire. Added an
  **additive `LegCmd` message** (MsgType 0x88, T2J, 100 Hz) carrying
  `axes[i].target_pos_rev/target_vel_rps` — the float32 ladder output, written every
  interp tick for all legs regardless of the output gate. *Why an additive message,
  not growing `Telemetry`:* additive keeps every existing frame byte-unchanged, so
  **no `PROTOCOL_VERSION` bump** — the version is a hard `decode_frame` gate, and
  bumping it forces every consumer to redeploy in lockstep, whereas an old consumer
  simply ignores the unknown 0x88 type. `pio` green; cross-language consistency test
  green. **Not yet flashed** (USB flash deferred to the U3-iv run, to avoid
  perturbing the operator's concurrent `eth0` recovery — a Teensy reboot changes the
  network endpoint). Commit `b06699f`. The bench driver gained `--replay` (uses
  `RecordedThrowSource`, captures `LegCmd` into a `cmd_teensy_rev` CSV column = the
  residual datum).

## U3-iii powered fault-replay — sub-test B + recovery (2026-06-25)

With `eth0` restored (the dongle re-seated + Jetson rebooted — the physical-unplug
hang is now an *accepted weakness*, above), the powered tail resumed. The `LegCmd`
firmware (`b06699f`) was **flashed over USB and validated on hardware**: `TELEMETRY`
+ the new `LEG_CMD` frame both decode cleanly at 100 Hz. A pre-arm diagnostic read
caught the leg **unpowered** (`bus_V = 0.0 V`, `active_errors = 0x201` =
`INITIALIZING | DC_BUS_UNDER_VOLTAGE`) — the canonical unpowered-bench signature
([[feedback_uniform_undervoltage_benign]]); the errors **self-cleared the instant
the DC bus came up**. The bench supply only reaches **21 V** (vs the production
~48 V), which is electrically safe for this gentle test — the leg ODrive's
`dc_bus_undervoltage_trip_level` is 10.5 V — but back-EMF-marginal for the
higher-speed U3-iv throw, so U3-iv waits on the 48 V PSU.

**Sub-test B (CAN3 drop → deferred-stow recovery) — PASS.** Leg armed via
`--close-loop` to a clean hold at 0.30 rev (`--observe` mode, sole wire authority,
e-stop in hand). Operator unplugged CAN3, waited ~5 s, replugged. Captured on the
always-on telemetry CSV (`setpoint_bench_ax0_observe_20260625_133918.csv`):

| t (s) | Event | Reading |
|------:|-------|---------|
| ~15.83 | CAN3 unplugged | — |
| 15.98 | `MOTOR_FB_STALE` (fault 7) | +0.15 s — recoverable feedback-staleness intermediate |
| 17.78 | **`CAN_BUS_DOWN` (fault 6)** | ~2.0 s after the last leg heartbeat — the `CAN_HEARTBEAT_TIMEOUT_US` clock (`fault_step` measures `now − last_heartbeat_us`), which is ≈ +1.95 s vs the operator's ~15.83 s unplug estimate ✓; driver cedes authority (flags=0, streaming off) |
| 17.8 → 22.8 | **leg holds** at 0.30 rev | odrv0 autonomous CLOSED_LOOP @ 21 V — no runaway, no drift |
| ~22.78 | CAN3 replugged → fault **6 → 0** | fresh leg heartbeats → `all_present_legs_fresh()` → deferred stow fires |
| 22.85 → 23.08 | **deferred-stow descent** 0.30 → 0.006 rev | **peak \|vel\| = 2.472 rev/s ≤ 2.5 cap** ✓; ~0.23 s |
| end | leg **IDLE** at off-pose (`STOW_OFF_POSE_REV = 0.0`) | de-energized, fault NONE, errors 0x0 ✓ |

This is the **Phase-8 deferred-stow safety inversion validated on hardware** — the
stow armed at CAN-loss detection and executed only on *confirmed* reconnect, which
is exactly the path U1's present-scoped `all_present_legs_fresh` unblocked for a
subset-populated bus (single bench leg). The **undervoltage observe** is covered by
the natural unpowered→powered transition above (UV error appeared then cleared),
so a deliberate armed-UV sag was judged unnecessary risk for the first cutover.

### Discussion — three things worth keeping

1. **The stow velocity profile is ramp-to-cap-and-hold, not a symmetric triangle.**
   I pre-registered an analytic expectation of ~1.7 rev/s peak for a 0.30 rev
   descent (assuming a triangular accel/decel profile at `STOW_ACCEL_RPS2`). The
   *measured* peak was **2.472 rev/s — essentially the `GENTLE_MOVE_VEL_LIMIT_RPS`
   = 2.5 cap**. Reading `leg_interp.cpp` after the fact confirmed why: the stow
   ramps `s_stow_speed` up at `STOW_ACCEL_RPS2` and **holds it at the cap** until
   within `STOW_DONE_EPS_REV` of the target, then stops — there is no symmetric
   decel ramp. So even a short descent reaches the clamp. This is the empirical-
   probe discipline paying off: the analytic was wrong, the measurement was right,
   and the safety property (descent ≤ 2.5 rev/s) is *confirmed by data*, not by a
   model that happened to also be conservative.

2. **The deferred stow is host-independent.** On the *first* attempt the operator's
   replug landed just after the driver's `--duration` expired (the conversational
   unplug→confirm→replug round-trip outran the window). The stow still ran — the
   firmware descended the leg and IDLE'd it **with no bench driver and no host
   authority alive** (verified by a post-hoc live diagnostic read: leg at ~0 rev,
   IDLE, fault NONE). This is correct, desirable autonomy: the §6 stow latch lives
   in the 10 Hz `fault_step`, independent of the Jetson. It also explains a subtle
   state difference — when the host *is* alive (run 2), the firmware leaves the leg
   in CLOSED_LOOP holding at the off-pose (the driver's `finally` then IDLEs it);
   when the host is *dead* (run 1), the firmware IDLEs the leg itself.

3. **Coordination latency, not the test, was the failure mode of run 1.** The fix
   was procedural: hand the operator the full unplug→pause→replug sequence to run
   at their own pace under a generous `--duration` (150 s), rather than gating each
   physical step on a conversational confirm. Run 2 captured the complete
   fault → hold → reconnect → descent → IDLE arc in one CSV.

## U3-iv real-MPC residual + D9 motion-onset (2026-06-25)

The U3-iv run measures the two quantities that gate the decision **D9** (port the
friction-FF into the Teensy ISR, or accept its loss on the β path): the **float32
interp residual** and the **motion-onset penalty** of dropping friction-FF.
Tooling: the `LegCmd` uplink (`b06699f`), the friction-FF injection
(`friction_ff_torque_nm` + `--friction-ff`, `930f70e`), and — added mid-run — the
**time-stretch** (`time_stretch` + `--replay-stretch`/`--max-dev`/`--vel-limit`,
`489b495`). All powered, operator-gated, e-stop in hand, bench driver sole wire
authority.

**The full-speed throw is untrackable on the bench leg — current-limited at the
onset.** Replaying `mpc_20260330_165611.csv` (axis 0, peak 3.18 rev/s, ~75 rev/s²
onset) at 1× **aborted at the throw onset, twice**: deviation 0.305 rev (default
0.30 belt), then — after raising the belt to 0.5 and `vel_limit` to 10 — 0.544 rev.
The decisive datum: at the second abort the leg hit **`iq = 9.04 A` (its 10 A
current limit)** and *still* could not accelerate to follow the onset. So the
binding constraint is **onset torque/acceleration, not velocity** (the leg never
approached even 4 rev/s; `vel_limit` was a red herring). The recording is the
real-robot *commanded* `cmd_ext` — it does not prove the real leg tracked that
onset tightly, and the bench's 10 A limit caps the achievable onset accel below the
command's demand. The firmware **lead clamp (`MAX_LEAD_REV` = 0.15) saturated** the
instant the throw started (`cmd_teensy` pinned to `enc + 0.15`), which both starves
the catch-up *and* corrupts the residual (a clamped `cmd_teensy` is no longer the
pure interp output).

**Reframe → time-stretch (operator chose this over pushing the hardware).** The two
U3-iv measurements do **not** require a full-speed throw: (a) the **float32
residual is speed-independent** — it is float32-vs-float64 arithmetic on ~3 rev
position values, identical however slowly the same positions are traversed; (b) the
**onset penalty is a low-velocity stiction breakaway**, measurable from any rest. So
`time_stretch` resamples the position trajectory to play 3× slower (shape preserved;
per-frame velocity **and** the knot velocity-steps scale 1/3 — *not* 1/9: a linear
resample keeps the velocity steps discrete at the knots, so the per-frame demand
scales 1/k, verified empirically 75→38→25 rev/s² at 1×/2×/3×). At 3× the peak is
**1.06 rev/s** — well inside the ~2.5 rev/s the leg already demonstrated.

**Pass A (friction-FF ON, 3×) and pass B (friction-FF OFF, 3×) both tracked cleanly
to 3.05 rev** (`setpoint_bench_ax0_replay_20260625_161732.csv` and `…_173811.csv`).
The lead clamp stayed unsaturated through the whole trajectory (pass A: 1/1999
samples touched 0.15, and that one was the t=0 interp-ladder seeding artifact), so
`cmd_teensy` is the pure float32 interp output.

**Float32 residual — `max|float32 − float64| = 5.5e-7 rev` (≈ 0.039 µm).** Computed
software-side (cubic Hermite on the 3× knots, 500 Hz subticks, both precisions) —
the clean, timing-independent way to isolate the float32 arithmetic effect. Worst
case is at the *largest* position (~2.68 rev), as expected (float32 ulp ≈ 3.6e-7 rev
at 3 rev). **Within the xref/Phase-7 "done-when" 1e-6 rev threshold (`hermite_xref/xref.py`)
and — the pre-registered D9 bar — ~4 orders of magnitude inside tracking noise (~mm).** The hardware `cmd_teensy` *corroborates* (it commanded the
full trajectory, range [0.076, 3.052], lead unsaturated) but cannot *isolate* 5.5e-7
directly — any hardware↔offline timing jitter (~1e-3 rev at 1 rev/s) swamps it, so
the precision number is software and the hardware confirms sanity at the
tracking-noise level. **D9 criterion 1: MET.**

**Motion-onset penalty — null (below ~25 ms resolution + run-to-run noise).** Both
passes break away from rest within the *same* 25 ms grid step (onset at t≈4.025,
breakaway at t≈4.05–4.075); FF-OFF if anything moved *slightly earlier* (+18.4 vs
+7.4 mrev at 4.075), so there is **no systematic onset delay** from dropping
friction-FF. **Why (load-bearing):** the smooth-gate FF is `gate(v) = 1 −
exp(−(|v|/0.05)²)`, which is **≈0 at v ≈ 0** — deliberately suppressed at the
breakaway moment (the PR 2.1 limit-cycle fix). The `iq` traces confirm the FF *is*
applied (pass A pulls 4.7/6.9 A as it starts vs pass B's 1.6/1.1 A) but contributes
~nothing at the v≈0 breakaway itself, so breakaway timing is unchanged. **D9
criterion 2: MET** (penalty ≪ 40 ms).

### D9 decision — accept the friction-FF loss (do NOT port it to the Teensy ISR)

Both criteria are met and the onset null is *robust*, not lucky: the gate that
makes friction-FF safe on platform (suppress at v≈0) is exactly what makes its loss
free at breakaway. The float32 interp is precise to sub-micron. So the β path
(40 Hz knots → Teensy 500 Hz Hermite, friction-FF dropped) is validated, and **U4**
(the production α→β switch, which *is* the friction-FF-drop) is green-lit.

### Discussion — corrections the data forced

This stage repeatedly contradicted an a-priori expectation; each correction is
logged because the next reader (likely AI) will otherwise re-derive the wrong one:
- **Pre-flight scaling checked bounds, not trackability.** `scale_to_bench` gates
  position ceiling + peak chord velocity — neither of which is the binding
  constraint. The leg aborted on *onset acceleration* (current-limited), a
  constraint the pre-flight never modelled. Trackability ≠ in-bounds.
- **`vel_limit` was a red herring.** Raising it 4→10 rps changed nothing (the leg
  never approached 4 rps in the binding phase); the limiter was current/torque.
  Empirical `iq = 9.04 A` settled it — *measure the binding variable, don't assume
  it*.
- **Time-stretch accel scales 1/k, not 1/k².** Continuous time-dilation gives
  1/k², but a *linear resample* keeps velocity steps discrete at the knots, so the
  per-frame demand scales 1/k. The offline preview (75→38→25) caught the wrong
  exponent before it reached a docstring claim.
- **The float32 residual cannot be measured from hardware.** `cmd_teensy` was built
  (`LegCmd`) to read the interp output directly — but a 5.5e-7 rev effect sits far
  below the ~1e-3 rev hardware↔offline timing-jitter floor even at 1 rev/s. The
  residual is intrinsically a *software* float32-vs-float64 measurement; the
  hardware uplink's real value is the **tracking/lead-clamp** picture (which is what
  exposed the untrackable-throw finding), not the precision number.
- **The onset null is physics, not noise.** It would be easy to dismiss "0 ms
  penalty" as too-coarse resolution; the gate-suppression mechanism (and the `iq`
  evidence that FF *is* applied yet doesn't move breakaway) makes it a real,
  explained null.

## U4 — design locked, ready to implement (next session)

> **U4 landed 2026-06-25** — implemented in a fresh session per this plan. See
> [[2026-06-25-phase11-u4-production-cutover]] for the as-built record (the pump
> rewrite, the bumpless parity test, the `/teensy` rename, the
> bypass-vs-pass-through discussion, and the U5 hand-off). The locked design
> below is preserved as the pre-implementation spec.

D9 green-lit U4 (the production α→β switch); the design was settled with the
operator on 2026-06-25 and U4 deferred to a fresh session (it is a safety-critical
production rewrite whose correctness hinges on a convention detail that wants
full attention, not the tail of the long U3/D9 session). Everything below is the
locked design + the code map so the next session starts fast.

**Decisions (operator, 2026-06-25):**
1. **Bypass motor_guard for the legs.** The β `SetpointPump` reads the 40 Hz MPC
   command stream (`:5557`, `TOPIC_MPC_CMD`) directly; motor_guard leaves the leg
   path entirely. Leg safety becomes **MPC (coupled workspace) + the Teensy fault
   machine** (per-leg stroke/deviation/MPC-staleness-E-STOP/deferred-stow — all
   U3-validated). motor_guard's `:5556` output simply goes unconsumed; removing it
   from `jugglebot_launch.py` is a deployment follow-up (U5/cleanup), not required
   for the U4 code switch (ZMQ PUB/SUB tolerates the extra idle subscriber).
2. **Do the `/teensy` rename in U4.** Promote the leg/hand `/teensy/*` topics +
   services to their production names (drop the `/teensy/` prefix) — they were only
   namespaced to coexist with `can_node`, which is already gone (plan line 1375).
   Recover the exact production names from `can_node`'s git history; grep ALL
   consumers first (GUI, other nodes) — this is a wide ripple.

**THE safety-critical subtlety — the α→β switch must be bumpless.** The new pump
must reproduce motor_guard's *exact* knot derivation (`motor_guard.py:541–603`),
which mixes conventions:
- **u0** = `msg['motor_rev']` (ODrive convention, 0=STOW, includes the stow
  offset) if present, else `ext_mm × geom.mm_to_rev` (the unit-test/sim fallback).
- **u1** = `msg['cmd_next_mm'] × geom.mm_to_rev`; **u2** = `cmd_next2_mm ×
  geom.mm_to_rev` (extension convention, NO stow offset). Absent → clear the
  corresponding `has_u1`/`has_u2` flag (do NOT send NaN — firmware D4).
- **v0** = `msg['vel_mm_s']` → motor rev/s via
  `conversions.leg_velocities_to_motor_velocities` (cf. `hermite_xref/xref.py:143`).
- **torque_ff = zeros** (the friction-FF drop; D9).

Get this wrong and either the Teensy Hermite diverges from the xref-validated
float64 reference, or — worse — the leg *jumps by the stow offset* the instant the
source flips from α (`motor_rev`) to β. So the pump now needs the **geometry**
(`mm_to_rev`) injected, where today it is pure. The xref tool
(`tools/probes/teensy_link_profiling/hermite_xref/xref.py`, already <1e-6 rev vs
motor_guard) is the regression check: the β knots feed the same interpolator it
exercises, so re-running it after the rewrite confirms parity.

**Code map (verified 2026-06-25):**
- `controller/teensy_link/setpoint_pump.py` — `SetpointPump.build(telem, …)`
  currently consumes motor_guard's `:5556` telemetry (`leg_pos/leg_vel/leg_torques`,
  `flags=0`). Rewrite to consume the `:5557` MPC dict per the convention above,
  setting `flags |= has_u1 | has_u2`. Keep the `JB_OP_MAX_POSITION_STEP_REV`
  per-step gate + NaN/Inf reject.
- `ros_ws/.../teensy_bridge_node.py` — `_setpoint_loop`/`_process_setpoint`
  (~784–823) SUB on `:5556`; switch to `:5557` (`TOPIC_MPC_CMD`). Pump instantiated
  ~417. Keep the `mpc_active` / `enable_setpoint_output` (default false) gate — U4
  arms nothing; U5 flips the gate on the powered six-leg rig.
- `make_mpc_command` schema: `ros_ws/.../motion/ipc.py:167+` (fields above).
- `/teensy/*` names: `teensy_bridge_node.py` ~301–366 (topics `/teensy/robot_state`,
  `/teensy/hand_telemetry`, `/teensy/link_status`, `/teensy/profile`; services
  `/teensy/clear_errors`, `/teensy/reboot_odrives`, `/teensy/encoder_search`,
  `/teensy/home`, `/teensy/odrive_command`, `/teensy/set_motor_vel_curr_limits`).
- Tests to rewrite: `tests/teensy_link/test_setpoint_pump.py` (MPC-dict input now),
  `tests/ros/test_teensy_bridge_node_setpoint.py`.

## Open Questions / Next (U3 tail — operator-gated, powered)

Stages (i) + (ii) + the link-drop **sub-test A** + **sub-test B (CAN3 drop)** +
the undervoltage observe + **U3-iv (residual + onset, D9 decided)** all done
(above). The `motor_guard` config-path bug is **fixed** (`c584767`). **U3 is
complete; D9 = accept the friction-FF loss.** Remaining, each gated by explicit
operator final-say + e-stop:

3. **Sub-test A link-drop *recovery* — optional crumb.** The gating half is
   confirmed (prior session); the recovery half (link restored → `MPC_STALE`
   self-clears, leg resumes tracking) was never observed because the physical-unplug
   method killed `eth0`. It's a minor confirmation (MPC_STALE-not-latched is already
   covered in code + unit tests); if done, use `sudo ip link set eth0 down … up`
   while armed in `--observe` mode — **never** the physical dongle unplug
   ([[project_eth0_usb_dongle_unplug_weakness]]).
4. **U4 — production α→β switch — DONE (2026-06-25).** D9 decided (accept the
   friction-FF loss) unblocked U4: the production setpoint path now emits β knots
   (40 Hz MPC knots → Teensy 500 Hz Hermite), the friction-FF is dropped, and the
   `/teensy` rename landed. Desk-side, arms nothing
   (`enable_setpoint_output=false`). Full record:
   [[2026-06-25-phase11-u4-production-cutover]] (commits `cb0d158`, `50fc8fe`).
   **U5** (powered six-leg validation + decommission) is cleared to start.

Also open: **Finding C** — the deferred deterministic pre-arm sole-authority guard
(refuse to arm unless the firmware reads `LINK_LOST`); **Finding A** (the rare
feedback freeze) root cause. The `motor_guard` config-path bug is **fixed**
(`c584767`). **Rollback note (corrected):** `can_node` is now reference-only (the
Jetson no longer sees CAN directly), so the cutover abort is **e-stop +
`mpc_active=0` disarm + power-down**, *not* a `<10 min` socketcan swap-back — the
bench driver's instant disarm (Ctrl-C / fault) gates firmware output off (≈ one
10 Hz fault step, ~100–200 ms — not "instant"). *(The disarm-to-output-gate latency
was not separately characterised in U3-iv — that run measured the float32 residual +
onset, not the gate latency — so the ~100–200 ms estimate stands; carry it to U5 for
a direct measurement.)*
