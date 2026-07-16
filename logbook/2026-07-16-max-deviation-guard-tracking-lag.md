---
title: The MAX_DEVIATION guard latched on legitimate velocity-loop lag, not runaway — guard raised 0.5→1.0 rev, leg vel_limit 4.0→6.0 rev/s, per-leg latch attribution wired
type: investigation
date: 2026-07-16
status: in-progress
phase: "MVP trajectory bringup — S4 limit ramp (MAX_DEVIATION guard + vel_limit headroom)"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.h
  - config/generated/hardware_config.py
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/gui/js/state-minimap.js
  - controller/teensy_link/replay_setpoint.py
  - tests/hardware/bench_leg_sysid.py
  - tests/firmware/native/test_fault_machine.cpp
  - tests/ros/test_teensy_bridge_node_read.py
  - tools/probes/gui_synthetic_stack.py
  - tests/hardware/mvp_bench_runbook.md
  - tests/hardware/session_phase4_ramp.md
commits:
subsystem:
  - can
  - config
  - ros
  - gui
tags:
  - safety
  - dynamics
  - performance
---

# The MAX_DEVIATION guard latched on legitimate velocity-loop lag, not runaway

## Summary

The operator ran the S4 limit ramp on 2026-07-16 afternoon. It **PASSed** at the
100 / 660 / 10500 (vel / acc / jerk, mm-units) step, then **latched the Teensy
MAX_DEVIATION guard three times** at the vel = 200 mm/s step — bags
`2026-07-16_13-17-58` (@528.371 s and @621.567 s) and `2026-07-16_13-34-58`
(@72.502 s), always **leg 1 (0-indexed) first**, frozen trip deviations
**−0.5196 / −0.5603 / −0.5515 rev**, always at the **onset of the same
asymmetric sweep**; raising jerk 8000 → 10500 made **no difference**.

A six-agent parallel investigation + adversarial cross-check established that these trips
were **legitimate velocity-loop lag under coordinated-move load, not runaway**:
the guard compares the RAW streamed 40 Hz knot `u0` against the encoder **PRE
lead-clamp**, so it counts exactly the command-space lag the lead clamp
deliberately tolerates. The drives were never current-railed (peak `iq_setpoint`
8.73 A of a 10 A soft max) nor velocity-railed (peak encoder vel 2.23 of 4.0
rev/s); `active_errors` was 0 through all three trip windows.

Operator-approved response (all four): **(1)** raise `MAX_DEVIATION_REV` 0.5 →
1.0 rev (firmware); **(2)** raise the ODrive leg `vel_limit` 4.0 → 6.0 rev/s for
lead-clamp catch-up headroom (runtime CAN push, no reflash); **(3)** wire per-leg
latch attribution to the bridge log + `/link_status` + the GUI minimap; **(4)**
recalibrate the runbook ABORT rules (0.1 rev applies at HOLDS only; during moves
deviation may run to ~0.6 rev but MUST collapse at arrival). `MAX_LEAD` stays
0.10 — deliberately unchanged.

## Symptoms

- S4 limit ramp: `(vel, acc, jerk) = (100, 660, 10500)` completed clean.
- The vel = 200 mm/s step latched `fault_state = MAX_DEVIATION` three times,
  each on the first move of the same wide asymmetric sweep. Leg output was
  suppressed on each latch; recovery required `/clear_errors`.
- Deterministic: same leg (1, 0-indexed) first every time; same trip magnitude
  band (0.52–0.56 rev); onset-locked. Jerk 8000 vs 10500 was not a factor.

## Diagnosis

**What the guard measures.** The MAX_DEVIATION check runs at `FAULT_TASK_HZ`
(10 Hz) in `fault_machine.cpp:367-371` and compares `interp_base_pos` (the RAW
streamed knot `u0`, via `s_base_pos` — `leg_interp.cpp:612/252/215`) against the
encoder. This is **PRE lead-clamp**. The EXECUTED command is bounded
independently to `enc ± MAX_LEAD` (0.10 rev) by the lead clamp
(`leg_interp.cpp:421-430`), so what the guard was reading is the gap between the
commanded ramp and where the leg actually is — command-space lag, not physical
excursion. Across all three trips the executed command never exceeded enc ± 0.10
rev.

**The scaling law (the decisive evidence).** Peak `|deviation|` is **superlinear
in realized leg velocity** — `v^1.19` pooled over all legs, `v^1.39` for leg 1:

| realized leg vel (mm/s) | peak \|dev\| (rev) | latched? |
|---|---|---|
| 35–50 | 0.079 | no |
| 75–110 | 0.170 | no |
| 110–140 | 0.276 | no |
| 140–170 | 0.408 | no |
| 170–200 | 0.544 | **6/6 move records** |

*(6/6: six per-move records landed in the 170–200 band across both bags — each of
the three latch events spans two records in the per-move extraction, the tripped
move plus its latch window. Every record in the band belongs to a latch event.)*

The implied lag time-constant **grows 112 → 208 ms** across that range — it is
*not* a fixed transport delay (the real transport delay is 6–15 ms). A
deficit-integral check closes the loop quantitatively: the integral of
`(v_cmd − v_enc)` from move onset to trip is **−0.594 rev** against the observed
**−0.516 rev** deviation accumulated over the same window (leg-1 live deviation
at the last 10 Hz sample before the crossing; the frozen trip snapshot then read
−0.5196).

**The drives were never at any rail.** Peak `iq_setpoint` was **8.73 A** of the
10 A soft max, with **zero** pre-trip samples above 9.5 A; peak encoder velocity
was **2.23 rev/s** of the 4.0 rev/s `vel_limit` (0.00 samples ≥ 95%); ODrive
`active_errors` read 0 throughout all three trip windows.

### Hypotheses

| # | Hypothesis | Verdict | Evidence |
|---|---|---|---|
| H1 | Fixed transport delay | **REFUTED** | 12–33× too small; the implied τ is non-constant (112 → 208 ms) |
| H2 | ODrive `vel_limit` saturation | **REFUTED** | peak encoder vel 2.2 of 4.0 rev/s; 0.00 samples ≥ 95% |
| H3 | Accel-phase velocity-loop lag under coordinated load | **SUPPORTED (root)** | superlinear scaling law + deficit-integral closure |
| H4 | Lead-clamp limit cycle (the 2026-07-10 stutter) | **REFUTED** | monotone build, not oscillatory; v3 keeps `vel_ff` |
| H5 | Current-rail / stall | **REFUTED** | 8.7 A peak setpoint, nowhere near the 10 A soft max |
| H6 | Stroke clamp | **REFUTED** | trips occur mid-stroke |
| H7 | Command discontinuity | **REFUTED** | max knot step 0.081 rev = 40 Hz × commanded v (continuous) |

**Mechanism.** The velocity loop drags a **coordinated-move reflected load**
(~5–20× the bench single-leg `J_eff` ≈ 3e-4 kg·m² — that bench value describes
the *unloaded* leg only). During accel-away the velocity deficit integrates into
position deviation. Once the deviation exceeds `MAX_LEAD` (0.10 rev) the lead
clamp — by design — removes the position-P authority and tracking rides `vel_ff`
alone; the gap peaks at approximately `τ_eff · v_peak` and crossed the old
0.5-rev threshold at ~186 mm/s realized velocity. The gravity feedforward is
irrelevant here: the morning FF-on/off bags show **identical** deviation.

## Discussion

*(Written before the Fix section, per the CLAUDE.md rule — several hypotheses
were withdrawn mid-investigation and a non-obvious inertia reconciliation was
accepted.)*

**(a) Hypotheses withdrawn mid-investigation.** The orchestrator's initial
`vel_limit`-saturation framing (H2) and the theory agent's corollary — that the
*encoder* velocity is capped at `vel_limit` while the clamp is engaged — both
died against the measured encoder velocities. The velocity *setpoint* saturates;
the *encoder* does not. Peak measured encoder velocity was 2.2 of 4.0 rev/s, so
saturation cannot be the mechanism. These retractions are recorded under
Withdrawn claims below.

**(b) Reconciling "inertial FF is negligible" with "accel FF is the right
lever."** The 2026-07-13 bench plant-ID measured `J_eff ≈ J_rotor` for the
*unloaded single leg*, which is TRUE — but it does **not** describe
coordinated-move dynamics, where the data-implied reflected inertia is
1–3e-3+ kg·m². This is the reconciliation: gravity/inertial FF is genuinely
negligible at the bench `J` (the "inertial FF is negligible" conclusion stands),
*and* accel FF is the correct lever at the coordinated-move `J` (~2–6 A). Any
future accel-FF sizing MUST use a **measured coordinated-move inertia**, not the
bench single-leg value — that is the next chapter, not this one.

**(c) Why raise the guard instead of "fixing" tracking first.** The guard was
sized to a gentler command envelope; the deviation it tripped on is command-space
lag the lead clamp deliberately tolerates; and the S4 ramp is blocked *today*.
Tracking improvement (accel FF) is real and worth doing, but it must not **gate**
the ramp. Raising the guard to match the measured legitimate envelope is the
correct decoupling: the physical excursion is bounded independently (see the Fix
rationale) so the safety cost is small and bounded.

**(d) Knot-phase lead is counterproductive here.** Plan Phase 4b's knot-phase
lead advances `u0`, which would *grow* the `u0 − enc` quantity this guard
watches. It is the wrong tool for this problem — do not reach for it here.

**(e) The mid-session battery edit is not a confound.** The operator edited
`traj_ramp_battery.py` at 13:23 to widen the move amplitudes beyond their labels.
This was checked and ruled out: both the PASS and the FAIL batteries are
post-edit, and the scaling law is expressed in **realized** leg velocity, not the
commanded label — so the widened moves land on the same curve.

## Fix

Two independent deployment halves (see Verification / Outcome for the operator
pickup on each).

**(A) Guard raise — firmware, requires reflash.**
`ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h`:
- `MAX_DEVIATION_REV` **0.5f → 1.0f**.
- `FW_VERSION` **1 → 2** — a purely human-facing identity marker so the flashed
  build is identifiable. Nothing reads `FW_VERSION` at runtime — its sole
  consumer is the boot Serial banner (`Teensy_code_canbridge.ino:405`);
  `version_check.cpp` is unrelated (it caches the ODrive axes' firmware
  versions off CAN, and the UDP protocol version is checked in frame decode).
  The bump has **no runtime or handshake effect**.
- `leg_interp.cpp` comments updated to reflect the new `vel_limit` inequality;
  `MAX_LEAD_REV` stays **0.10f** (unchanged).

**(B) Leg `vel_limit` raise — config → runtime CAN push, no reflash.**
`config/hardware_config.yaml` `jugglebot_odrive_defaults.leg_vel_limit_rps`
**4.0 → 6.0**, regenerated (`python config/generate_config.py`) into
`config/generated/hardware_config.{h,py}` (`LEG_VEL_LIMIT_RPS = 6.0f`) and the
consumer copies, and into `ros_ws/src/jugglebot/jugglebot/hardware_config.py`
(`ODRIVE_LEG_VEL_LIMIT_RPS = 6.0`). It is **pushed to each leg ODrive over CAN at
runtime** — no reflash, no ODrive-NVM edit. The ODrive NVM snapshot
`config/ODrive config Files/odrive_pro_leg_config.json` was deliberately NOT
touched; the runtime push is authoritative over the flashed NVM once homing runs.

Push site (line numbers current as of this entry's commit — this same commit's
observability change, part (C) below, grew the file +44 lines):
`teensy_bridge_node._run_configure()`
(`teensy_bridge_node.py:3009`) calls
`self.teensy_set_vel_curr_limits(axis, hw.ODRIVE_LEG_VEL_LIMIT_RPS,
hw.ODRIVE_LEG_CURR_LIMIT_A)` at `teensy_bridge_node.py:3035`. `_run_configure`
fires from three places: after every successful `/home` (`_do_home`,
`:3453` — the operator's "set after every homing"); the `/configure` service
(`:3474`); and after every `/activate` (`:3693`). So the **earliest effective
point is the first homing of the session**. The current limit pushed alongside is
`hw.ODRIVE_LEG_CURR_LIMIT_A = 10.0` (YAML `leg_curr_limit_a`, **unchanged**).

Conversion basis: `GEOM_MM_TO_REV ≈ 0.01418332` rev/mm ⇒ 6.0 rev/s ≈ **423 mm/s**
and the 280 mm/s trajectory ceiling ≈ 3.97 rev/s. The session trajectory ceiling
(280 mm/s) is a deliberately conservative *trajectory* cap and is unchanged — only
the *drive* limit moved.

**Fix rationale.**
- **Guard 1.0 rev** is a command-source sanity check re-sized to the *measured
  legitimate envelope*. Physical excursion is bounded independently by the lead
  clamp (`enc ± 0.10`) + the stroke clamp + the ODrive position clip, so raising
  the guard adds **zero** physical excursion for command-side faults. The
  accepted cost is the **encoder-side-runaway detect distance: 35 → 70 mm** — a
  corner where 0.5 rev already exceeded top-of-stroke headroom. The firmware
  overspeed guard `MAX_MOTOR_VEL_RPS = 16.5` rev/s still bounds any true
  encoder-side runaway and is unchanged.
- **`vel_limit` 6.0 rev/s** is catch-up headroom. The v2-bug constraint
  `Kp·MAX_LEAD ≤ vel_limit` becomes `40 × 0.10 = 4.0 ≤ 6.0` — a healthy
  inequality (was `4.0 = 4.0`, exactly at the limit). With the P-term sitting 2.0
  rev/s below the limit, `vel_ff` regains genuine catch-up authority under the
  clamp instead of being fully clipped. The bench envelope is ~48 rev/s (legs
  proven to 3.4 m/s), so 6.0 rev/s ≈ 423 mm/s stays far under it; the 16.5 rev/s
  overspeed guard is unaffected.

**(C) Observability — per-leg latch attribution.** Attribution is
**firmware-ground-truth only**: Python never re-thresholds `MAX_DEVIATION_REV`;
it reads the firmware's frozen `max_dev_leg` / `max_dev_value` snapshot from
`HeartbeatT2J`. That is precisely why today's 0.5 → 1.0 change needs **zero**
Python change — the attribution auto-tracks whatever threshold the firmware trips
at.
- `teensy_bridge_node._guard_fault_leg_hint()` previously read only the ODrive
  diagnostic, which empirically returned `''` at all three real latches
  (`active_errors == 0`); it now **falls back to the heartbeat snapshot**.
- `/link_status` gains a `guard_fault_leg` KeyValue: the culprit leg number as a
  string while a MAX_DEVIATION latch is ACTIVE, else `''`. It is a direct field
  for rosbags/GUI (does not re-derive gating) and reads `''` after
  `/clear_errors` even though the raw `max_dev_leg` persists as "last latch since
  boot".
- `ros_ws/gui/js/state-minimap.js` guard-latch tooltip now shows
  `(MAX_DEVIATION, leg N)`; the file's ground-truth anchors were re-verified
  against the grown bridge (the audit caught that the old `:3062-3079` anchor
  had rotted well before this diff) and now cite `_svc_clear_errors`
  (`teensy_bridge_node.py:3316-3368`) and `_arm_setpoint_output` (`:1654`) by
  name.

New log-line formats (sign convention: trip snapshot `:+.3f`, live vector
`:+.2f`):
- Edge (MAX_DEVIATION): `Teensy guard FAULT LATCHED: fault_state=MAX_DEVIATION
  (leg 1, dev=-0.552 rev at trip) live_dev=[-0.55,-0.31,+0.34,+0.33,-0.12,+0.41]
  rev — leg output is now SUPPRESSED and every leg command (incl. DEACTIVATE,
  which returns ERR_BUS_DOWN) will be refused. Recover with: ros2 service call
  /clear_errors std_srvs/srv/Trigger`
- Edge (other faults, unchanged): `…fault_state=ODRIVE_FATAL (leg 3) — …` (no
  `live_dev` vector; leg hint from ODrive diag).
- Persistent reminder (every `_GUARD_LATCH_REPEAT_S = 5.0` s): `TEENSY GUARD
  LATCHED (MAX_DEVIATION) (leg 1, dev=-0.552 rev at trip) — leg output
  suppressed; CLEAR_ERRORS required` (now carries the leg hint for ALL fault
  types; previously had none).

Supporting ripple (constants that reference the changed limits, updated in
lockstep): `controller/teensy_link/replay_setpoint.py` (the ≤ `vel_limit`
tracking-guard doc), `tests/hardware/bench_leg_sysid.py`
(`BRIDGE_MAX_DEVIATION_REV 0.5 → 1.0`; the bench measurement-step `vel_cap`
default deliberately STAYS 4.0), `tests/firmware/native/test_fault_machine.cpp`
and `tests/ros/test_teensy_bridge_node_read.py` (guard-threshold + attribution
tests), and `tools/probes/gui_synthetic_stack.py` (the new `guard_fault_leg`
field). The runbook ABORT-rule recalibration lands in
`tests/hardware/mvp_bench_runbook.md` and `tests/hardware/session_phase4_ramp.md`
(coordinated with the runbook writer).

## Verification

- Scoped suite (`python -m pytest tests/motion/ tests/teensy_link/
  tests/firmware/ -q`, run 2026-07-16): **1018 passed in 110.58 s**.
- Attribution tests (audit re-run, 2026-07-16): `python -m pytest
  tests/ros/test_teensy_bridge_node_read.py -q` → **33 passed in 6.23 s**;
  `python -m pytest tests/firmware/ -q` → **165 passed in 14.43 s** (the
  native fault-machine binary recompiles and re-verifies the new 1.0 rev
  threshold, stimuli re-sized 0.9 → 1.2 rev).
- Firmware built clean (`pio run -e teensy41`, 2026-07-16): SUCCESS — Flash 3.2%,
  RAM 27.2%. (Build only — flashing is the operator's step; see Outcome.)
- An `/audit --unstaged` round before this commit (verdict: no blocking
  findings) caught: three GUI ground-truth anchors pointing at the wrong
  function (rotted before this diff, "bumped" onto `_run_activate`'s docstring
  — re-pointed to `_svc_clear_errors` by name), nine stale `0.5 rev` / `4.0
  vel_limit` claims across bridge/trajectory/bench/docs comments, the
  normative methodology plan still stating the `Kp·MAX_LEAD = vel_limit`
  EQUALITY as a contract (restated as the inequality with the 2026-07-16
  rationale), `docs/can_bridge/safety.md`'s fault table (0.5 → 1.0, plus a
  pre-existing stale 0.15 lead-clamp figure), and four factual wobbles in this
  entry (agent count, "6/6" units, the −0.516 measurement instant, the
  `version_check.cpp` misattribution) — all fixed pre-commit.
- Full suite, pre-audit (`pytest tests/ -q`, run 2026-07-16): **2811 passed,
  1 xfailed in 596.35 s**. Post-audit-fix re-run — the pre-commit gate
  (`pytest tests/ -q`, run 2026-07-16): **2811 passed, 1 xfailed in 772.17 s**.

## Outcome

The guard is re-sized to the measured legitimate envelope, `vel_ff` regains
catch-up authority under the lead clamp, and the culprit leg is now named on the
bridge log, `/link_status`, and the GUI minimap. **Deployment is two independent
halves and neither has reached hardware yet:**

- **(A) Guard 0.5 → 1.0** lives in the compiled firmware and takes effect **only
  after the operator reflashes the can-bridge Teensy**
  (`pio run -e teensy41 -t upload`). Until reflashed the guard stays at 0.5 and
  will keep latching at ~190 mm/s.
- **(B) `vel_limit` 4.0 → 6.0** is a runtime CAN push — **no reflash**. Operator
  next-session pickup: `colcon build --packages-select jugglebot` **then**
  relaunch. A bare relaunch of a stale install keeps pushing 4.0, because the
  launch runs the *installed* copy of `hardware_config.py`, not the source tree
  (known project gotcha). After the colcon build + relaunch, the normal cold-start
  (BOOT → HOMING) auto-pushes 6.0 at the first homing.

Once both land, the S4 vel = 200 mm/s step should complete — the deviation will
still build to ~0.55 rev at onset (that is legitimate command-space lag) but now
sits well inside the 1.0-rev guard and, with `vel_ff` catch-up authority
restored, should collapse at arrival.

## Withdrawn claims

- [2026-07-16] Initial framing: the trips are ODrive `vel_limit` **saturation**
  (H2).
  WITHDRAWN: the measured peak encoder velocity was 2.23 of 4.0 rev/s with 0.00
  samples ≥ 95% of the limit through all three trip windows — the encoder was
  never near the rail.
  Superseded by: H3 (accel-phase velocity-loop lag under coordinated load) in the
  Diagnosis hypotheses table.
- [2026-07-16] Theory-agent corollary: while the lead clamp is engaged the
  *encoder* velocity is **capped at `vel_limit`**.
  WITHDRAWN: only the velocity *setpoint* saturates; the encoder velocity is a
  free measured quantity and stayed at 2.2 rev/s. The clamp bounds the command,
  not the plant.
  Superseded by: the deficit-integral closure (−0.594 rev computed vs −0.516 rev
  observed) in the Diagnosis section.

## Open Questions

- **Accel feedforward is the next chapter, not this one.** Sizing it MUST use a
  **measured coordinated-move inertia** (data-implied 1–3e-3+ kg·m²), NOT the
  bench single-leg `J_eff ≈ 3e-4`. See the FF-before-feedback methodology in
  `plans/active/leg-gain-tuning-methodology.md`.
- **Hardware validation pending** on both deployment halves (reflash for A;
  colcon build + relaunch for B), then a re-run of the S4 vel = 200 step to
  confirm the ramp clears and the deviation collapses at arrival.
- The legacy Jetson-side 500 Hz guard (`motor_guard.py:87`) keeps its own
  `MAX_DEVIATION_REV = 0.5` — deliberately NOT raised here. It belongs to the
  MPC path (dormant on this branch) and has seen none of this data; the
  intentional divergence is documented at the firmware constant's definition.
  Decide whether it tracks 1.0 or stays at 0.5 when the MPC path revives.
  (The `max_dev_leg` / `max_dev_value` wire telemetry this entry's
  observability consumes is NOT new — it has been flashed since firmware v3,
  2026-07-10 `5daf53e`; only the guard threshold + `FW_VERSION` change in the
  pending reflash, which also arms the dormant `torque_ff` ingest clamp from
  `10de03c`.)

## Related

- `logbook/2026-07-10-s4-stutter-guard-forensics-recovery-stack.md` — the v2
  lead-clamp bug + the v3 firmware (`MAX_LEAD` 0.15 → 0.10, `vel_ff` kept at
  clamp engage) whose constraint this entry re-balances.
- `logbook/2026-07-16-gravity-ff-armed.md` — the morning gravity-FF session
  whose FF-on/off bags confirm gravity FF is irrelevant to this deviation.
- `plans/active/leg-gain-tuning-methodology.md` — FF-before-feedback; the
  next-chapter accel-FF pointer.
- Rosbags: `~/Desktop/rosbags/2026-07-16_13-17-58` (trips @528.371 s, @621.567 s),
  `~/Desktop/rosbags/2026-07-16_13-34-58` (trip @72.502 s).
- Session/runbook: `tests/hardware/session_phase4_ramp.md`,
  `tests/hardware/mvp_bench_runbook.md` (recalibrated ABORT rules).
- The per-agent analysis artifacts live in the session scratchpad on volatile
  `/tmp` — the durable numbers are captured IN this entry.
