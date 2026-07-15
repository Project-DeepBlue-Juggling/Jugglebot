# Session runbook — first arming of the leg torque feedforward (gravity)

**Status:** NOT YET RUN. This is the procedure for turning `dynamics.torque_ff_enabled`
on for the first time on real hardware.

**Operator runs every robot-actuating command.** Claude preps commands + PASS/ABORT
criteria and verifies read-only telemetry.

---

## What is being armed

An **open-loop torque** injected into each leg's ODrive, alongside the position/velocity
cascade, at 500 Hz. It is the gravity load the platform imposes on each leg
(**0.013–0.041 Nm/leg**, mean ~0.023 Nm at the active pose ⇒ **0.2–0.7 A**), computed from
the planned pose and decomposed through the Jacobian.

What it is **not**: it is not friction FF (that is a separate, velocity-signed model that
is not part of this feature), and it does not include the platform-inertia or
reflected-rotor terms (`torque_ff_platform_inertia` ships `false` — see the YAML).

**Expected visible effect on a stationary platform: none.** The gravity FF (0.2–0.7 A) is
*smaller than the Coulomb friction floor* (1.09 A). It cannot make a held platform move.
If arming it makes the platform visibly jump, **that is a fault, not the feature** — abort.

## The three things that could go wrong, and what guards each

| Failure | Consequence | Guard |
|---|---|---|
| **Sign inverted** | FF helps gravity pull the platform down; position loop fights 2× the load; sag, buzz, possible MAX_DEVIATION | `test_gravity_ff_sign_holds_platform_up` + the reconstruction test. **S3 below is the hardware confirmation.** |
| **Magnitude 10× / units slip** | FF ≥ 0.55 Nm saturates the ODrive current clamp; the position loop loses *all* authority and the leg runs open-loop until MAX_DEVIATION E-STOPs it mid-motion | **Two software layers** (since 2026-07-14): (1) `SetpointPump` clamp at `torque_ff_max_nm` = **0.15 Nm** (≈2.6 A; 0.1451 wire-Nm) — binds first on the production path; (2) the can-bridge **firmware ingest backstop** at `torque_ff_firmware_clamp_wire_nm` = **0.25 wire-Nm** (≈4.5 A; clamps, never rejects — surfaced per leg as `torque_clamp_mask` on `/link_status`). ⚠️ Layer 2 **requires a can-bridge reflash**; on a pre-2026-07-14 firmware it is dormant and the pump clamp is the only one (int16 saturation at ±3.2767 wire-Nm = 59 A is not a guard) |
| **Integrator double-count at arm** | The velocity integrator already carries gravity from the activate hold; a step FF briefly commands ~2× gravity → upward kick on every leg | `SetpointPump` **ramp**: FF = 0 on the first accepted frame, linear to full over `torque_ff_ramp_s` = 2.0 s (80 accepted knots). Restarts on `reset()` (link loss / re-arm). |

---

## S0 — Pre-flight (no motion, no power to the legs needed for 0a–0c)

**0a. Confirm the drives still carry the `torque_constant` the wire scale assumes.**
This is the single unguarded assumption in the design: nothing in the software ever reads
`torque_constant` back off a drive, and the whole Kt prescale (×0.9673 since the 2026-07-15 Kt measurement; was ×0.8835) is built on the
drives holding ODrive's default `8.27/150 = 0.055133`. If someone has ever "fixed" a drive
to 0.0624 with odrivetool, the FF under-delivers by 11.7% **and the shipping velocity loop
is already detuned by 11.7%** (independently of this feature).

```
# odrivetool, per leg drive (repeat for all six)
odrv0.axis0.config.motor.torque_constant        # EXPECT 0.055133331567049
odrv0.axis0.config.can.input_torque_scale       # EXPECT 10000  (NOT ODrive's default 1000)
odrv0.axis0.controller.config.vel_gain          # EXPECT 0.20
odrv0.axis0.controller.config.vel_integrator_gain  # EXPECT 0.32
odrv0.axis0.controller.config.pos_gain          # EXPECT 40.0
```

- **PASS**: all six drives report exactly those values.
- **ABORT**: any drive differs. In particular `input_torque_scale = 1000` on any drive
  means that leg would receive **10× the intended feedforward torque**. Do not proceed.

**0b. Confirm firmware version.** The can-bridge must be v3-protocol (commit ≥ `5daf53e`)
and the ODrives on 0.6.11 — already validated 2026-06-29, re-confirm only if drives were
swapped.

**0c. Confirm the feature is currently OFF and the suite is green.**
```
grep -n "torque_ff_enabled" config/hardware_config.yaml     # EXPECT: false
source ~/Desktop/PDJ_venv/venv/bin/activate && python -m pytest tests/ -q
```
- **PASS**: `torque_ff_enabled: false`, suite green.

---

## S1 — Baseline WITHOUT the feedforward (the A side of the A/B)

Run the robot exactly as today (flag still off). Record, so S4/S5 have something to compare
against:

1. Home → activate → hold at the active pose for 30 s.
2. Capture, from `robot_state` / the GUI: **per-leg `iq_measured` at hold** (this is the
   current the integrator is spending to carry gravity — it is the number the FF is
   supposed to take over), and the **hold position noise** (`act_std`, µm).
3. Run one **S4-class stroke battery** (the standard bench move set) and record
   `track_err_rms` and, critically, **error at arrival**.

- **PASS**: hold `act_std` in the usual ~20 µm band; arrival error ≪ 1 mm (today: 0.054 mm
  median). Note the per-leg hold `iq_measured` — call it `iq_hold_baseline`.
- **ABORT**: anything already anomalous. Do not layer a new feedforward on a sick baseline.

---

## S2 — Arm the feature (software only, no motion)

```
# 1. Edit config/hardware_config.yaml:
#      dynamics.torque_ff_enabled: true
#    (leave torque_ff_gravity: true, torque_ff_platform_inertia: false,
#     torque_ff_max_nm: 0.15, torque_ff_ramp_s: 2.0 exactly as shipped)

python config/generate_config.py
source ~/Desktop/PDJ_venv/venv/bin/activate && python -m pytest tests/ -q

cd ros_ws && colcon build --packages-select jugglebot && source install/setup.bash
```

**`colcon build` is mandatory** — the launch runs the INSTALLED copy of
`hardware_config.py`, and a YAML edit without a rebuild silently does nothing (this exact
trap cost a session on 2026-07-12).

- **PASS**: `test_shipped_config_has_the_feature_off` **fails** (it is designed to — it is
  the tripwire proving the flag really moved), everything else green. Re-run with
  `--deselect tests/motion/test_leg_torque_ff.py::test_shipped_config_has_the_feature_off`
  to confirm nothing *else* broke.
- **NO FIRMWARE REFLASH IS NEEDED to arm the feature.** The Teensy already forwards
  `torque_ff`; the flag flip is Jetson-side only.
- **The firmware ingest backstop is a separate question.** Since 2026-07-14 the
  firmware source carries a second clamp layer (`leg_interp.cpp` ingest clamp at
  `torque_ff_firmware_clamp_wire_nm` = 0.25 wire-Nm + the per-leg `torque_clamp_mask`
  heartbeat/`/link_status` telemetry). **That layer only exists on the wire after a
  can-bridge reflash** — until the bridge is reflashed the pump clamp (0.1451 wire-Nm)
  remains the only clamp, exactly as originally validated, and `torque_clamp_mask`
  reads 0 on `/link_status` (the flags bits are simply never set). Flash-A of this
  firmware is safe either before or after this session: the backstop is strictly
  wider than the pump clamp, so it never binds on a healthy chain.

**Confirm the arming banner.** On launch, `teensy_bridge_node` must log:
```
LEG TORQUE FEEDFORWARD IS ENABLED (clamp ±0.15 Nm true, wire scale 0.967251, ramp 80 frames ≈ 2.0 s)
```
- **ABORT if this line is absent** — you are running a stale install.

---

## S3 — THE SIGN CHECK (the one that matters). Platform on the bench, ready to E-STOP.

> **2026-07-15 (ARMING CONTRACT)**: under the default `auto_arm:=true` the wire
> arms **automatically on ACTIVE entry** — the FF ramp you are about to watch
> starts at that automatic arm, i.e. right after `activate` completes. For a
> deliberate, operator-timed arm edge (recommended for this first-arming
> session), launch with `auto_arm:=false` and arm manually via
> `ros2 service call /set_setpoint_output std_srvs/srv/SetBool "{data: true}"`.

Arm and hold at the active pose. **Do not command any motion.** Watch the platform and the
per-leg `iq_measured` through the 2 s ramp.

**What PASS looks like:**
- The platform **does not move** (the FF is below the friction floor — it cannot move it).
- Per-leg `iq_measured` at hold **falls, or stays flat**, relative to `iq_hold_baseline`.
  The FF is taking over some of the load the integrator was carrying; the integrator
  unwinds by the same amount, so the *total* current is roughly unchanged and its
  *composition* shifts. A modest drop is the ideal signature.
- Nothing latches. `act_std` stays in its usual band.

**What ABORT looks like — hit E-STOP immediately:**
- The platform **sags, drops, or buzzes** during the ramp.
- Per-leg `iq_measured` at hold **rises by roughly 2× the FF** (i.e. ≈ +0.2–0.7 A per leg,
  in the direction that *adds* to what the integrator was already doing). That is the
  signature of an **inverted sign**: the FF is pushing down, the integrator is fighting it,
  and the two are now summing instead of trading.
- Any `MAX_DEVIATION` or overspeed E-STOP.

> If S3 aborts on an inverted sign, do **not** patch a minus sign into the pump or the
> firmware. The Jetson emits extension-positive Nm and the firmware's `leg_sign` does the
> inversion — exactly as it does for position and vel_ff, which are known-good. A sign
> inversion here means one of the three sign carriers changed (`-W_gravity`,
> `solve(J.T, W)`, the spool radius). Find which; the unit tests pin all three.

---

## S4 — Slow motion, one axis

Command a **slow** z-only move (e.g. 170 → 190 → 170 mm over several seconds), well inside
the session limits. Watch `iq_measured` peaks and listen.

- **PASS**: motion is smooth; no new audible buzz vs S1; peak `iq` never approaches the
  10 A `current_soft_max`; no faults.
- **ABORT**: any new vibration, any `iq` excursion beyond ~5 A, any latch.

---

## S5 — The S4-class stroke battery (the A/B payoff)

Re-run the exact battery from S1 and compare.

- **PASS**: **error at arrival still ≤ 1 mm** (the operator's spec; today 0.054 mm median).
  Tracking error is *no worse* than the S1 baseline. Hold `act_std` is *no worse*.
- **This is the honest expectation**: on the **unloaded/static** metrics the gravity FF
  should change almost nothing — it is smaller than friction, and the arrival error is
  already 5× inside spec. **The FF is not expected to improve the arrival number.** Its
  value is that the integrator no longer has to carry gravity, which matters under
  *dynamic* load and is where the remaining margin lives.
- **ABORT / REVERT** if: arrival error degrades, any limit cycle appears (especially in the
  5–6 Hz band — the 2026-05-08 friction-FF limit cycle lived there), or hold quality
  degrades.

**Reverting is one line**: `torque_ff_enabled: false` → `generate_config.py` → `colcon build`.
No reflash, no gain change, no re-validation of the velocity loop — that is the entire point
of the design.

---

## Explicitly out of scope for this session

- **Do NOT set `torque_ff_platform_inertia: true`.** The can-bridge holds the last
  `torque_ff` undecayed through a stale-link window, so an acceleration-proportional term
  would keep pushing at full magnitude while the commanded velocity decays to zero. That
  needs a firmware change (zero `cmd_tor` in the extrapolation path, as the recovery ramp
  already does at `leg_interp.cpp:479`) before it is safe.
- **`run_mpc.py` with the flag on is now SAFE from the inertia hazard** (fixed 2026-07-14,
  after the adversarial review): `HardwarePlant` reads the per-term config flags, so with
  `torque_ff_platform_inertia: false` the MPC path publishes **gravity only** (reflected
  rotor was already skipped — `skip_reflected_inertia=True` in `HardwarePlant.set_pose`; an
  earlier revision of this doc wrongly said it was included). The acceleration-proportional
  term cannot reach the wire from any producer while that flag is false. MPC remains
  dormant on this branch regardless; this session validates the trajectory path only.
- **Do NOT change `torque_constant` on the drives.** See the WHY block in
  `config/hardware_config.yaml`. It would silently detune the validated velocity loop.
