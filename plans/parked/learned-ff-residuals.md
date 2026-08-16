---
title: Learned Feedforward Residuals — Batch ILC on the Leg Torque Channel
created: 2026-07-24
status: parked   # 2026-08-15 — gates G-A (bridge-temporal closure + latency monitor), G-B (accel-FF concluded; that plan is itself parked) and G-C (stale-hold decay flashed) are all open; no implementation before they clear, per this plan's own rule.
related_logbook:
  - 2026-07-18-teensy-uptime-tracking-degradation.md
  - 2026-05-08-friction-ff-platform-limit-cycle.md
  - 2026-07-13-leg-plant-id-and-the-units-bug.md
related_config:
  - config/hardware_config.yaml → dynamics.torque_ff_learned (proposed, default false)
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/emitter.py::KnotEmitter.frame
  - ros_ws/src/jugglebot/jugglebot/motion/torque_ff.py::LegTorqueFeedforward
  - controller/teensy_link/setpoint_pump.py::SetpointPump
---

# Learned Feedforward Residuals — Batch ILC on the Leg Torque Channel

**Status:** PROPOSED — design note. Phase 0 (the residual telemetry channel) is
unblocked and independently valuable to three consumers; Phases 1–3 are gated
on three external arcs (see § Prerequisite gates). No implementation starts
before the gates clear and the Phase-1 repeatability decision passes.

## Recommendation (headline)

Juggling is repetitive: the same throw, catch, and reload trajectories execute
over and over. Iterative learning control (ILC) exploits exactly that — record
the per-leg tracking residual of a repeated trajectory, fold a filtered,
clamped fraction of it into the feedforward for the *next* execution, and
converge toward near-perfect tracking of that specific trajectory with **zero
new feedback loop in the hot path**. The two closed tuning arcs point here:
the leg gain hunt concluded tracking is a *feedforward* problem (gains are for
disturbance rejection; `plans/active/leg-gain-tuning-methodology.md`), and the
model-based FF chain (gravity shipped, platform-inertia planned in
`plans/parked/accel-ff-inertia.md`) can only capture what the model contains.
What it structurally cannot capture — the ±1.5 A/leg friction/hysteresis
band, pose-dependent non-inertial load ("half the measured heaviness"),
per-leg asymmetries (leg 2 heavy in x, leg 0 in y) — is *repeatable* along a
repeated trajectory, which is precisely the component ILC learns.

**What to build:** a batch (between-executions, offline) ILC that learns
per-trajectory, per-leg **torque trim profiles** on the 25 ms knot grid,
applied through the existing `torque_Nm` channel under the existing pump
clamp. **What not to build:** any per-tick feedback, any reference warping,
any online/continuous adaptation (v1 learns only in the offline batch step,
never during a session).

## Design constraints (load-bearing)

1. **Torque channel only — never the reference, never the velocity knots.**
   The wire has exactly one channel that leaves the executed position path
   untouched: `torque_ff` (pure ODrive current feedforward). The position
   knots u0/u1/u2 *and* the velocity v0 all shape the Teensy Hermite's
   executed path (`setpoint_pump.py` — v0 is the Hermite start-knot
   velocity; u2 sets the endpoint velocity v1 = (u2 − u1)/T),
   so learning on any of them would warp the gate-validated reference and
   make the `MAX_DEVIATION` guard partially self-referential. On the torque
   channel the commanded position stays the plan's analytic quintic: the
   K-contracts, the feasibility gate, and the guard's u0−enc semantics are
   all untouched by construction.
2. **Learn between executions; apply at plan install; sample per knot.** The
   correction updates offline from recorded repetitions, attaches to a plan
   at install time, and is sampled by `KnotEmitter.frame` as a lookup — no
   measured-state dependence ever enters the 40 Hz path. Firmware already
   holds `cmd_tor` ZOH per knot (`leg_interp.cpp:403`), so a knot-grid
   profile introduces no new high-frequency class beyond the existing
   staircase the gravity/inertia terms already ride.
3. **The trim is a trim, not a second actuator.** A load-time budget clamp
   `TRIM_MAX` bounds the learned term to the headroom under the pump clamp:
   pump 0.15 TRUE Nm minus p95 combined model demand at the target dynamics
   (arithmetic at 3000 mm/s² rigid+gravity = 0.129 → ≈0.02 Nm headroom with
   the inertia term enabled; ≈0.11 Nm if that term ships dormant). The exact
   value is fixed by Phase-1 arithmetic, not chosen here. The pump clamp
   itself (`setpoint_pump.py`, first-binding layer, ceiling 0.30) remains the
   single wire enforcement point, unchanged.
4. **Profiles are keyed, and a key miss means zero trim.** Key = trajectory
   identity hash (constructor kind, quantized target pose, duration, limits
   tier, shaper config) **plus** a model-config hash (torque-FF flags, sizing
   constants, Kt). Consequences by construction: a retime-model re-enable
   changes durations → keys miss → safe no-trim; any model change (e.g.
   enabling `torque_ff_platform_inertia`) invalidates all profiles → safe
   no-trim until re-learned. The safe default is always the current shipped
   behaviour.
5. **Deployed profiles are versioned artifacts**, stored under
   `ros_ws/src/jugglebot/resources/ilc/` (precedent:
   `resources/throw_affine_correction.json`), loaded at `trajectory_node`
   startup, attached at install.
6. **OFF path byte-identical.** Gate flag `dynamics.torque_ff_learned`
   (default false, same pattern as `torque_ff_platform_inertia`): when off,
   no profile attaches and the emitted frames are byte-identical to today's.

## Prerequisite gates (all three must clear before Phase 1)

- **G-A — CLEARED 2026-08-15. The 2026-07-18 uptime-lag investigation is closed**
  (reboot isolation experiment run S1 2026-08-12; root cause = the vendored
  FlexCAN_T4 `_available` RX-ring leak; fix = FW 14, validated at 5.8 h and 15.2 h
  of continuous uptime; the alarmed continuous latency monitor landed in the same
  change-set — `logbook/2026-08-15-fw14-validated-arc-closed.md`). Note the
  corollary below is now cheap to satisfy: uptime is no longer a lag predictor, so
  the learner's healthy-threshold refusal should be re-keyed off `latency_monitor`
  / `leak` rather than off `uptime_ms` when Phase 1 starts. ILC's
  premise is a *repeatable* plant; a transport delay drifting 10→240 ms with
  Teensy uptime is non-repeatable error that learning would chase forever —
  and worse, bake into deployed profiles. Corollary, enforced in tooling:
  the learner refuses bags whose `uptime_ms` exceeds the healthy threshold.
- **G-B — The accel-FF arc is concluded** (shipped enabled *or* dormant —
  either is fine; what matters is the model baseline is FROZEN before
  learning starts). The learned profile is defined as the residual *after*
  the model; the model-config hash in the profile key enforces this
  mechanically.
- **G-C — Firmware stale-hold torque decay is flashed** (accel-FF flash
  bundle T1.1, `leg_interp.cpp:403`). Identical hazard: a held learned trim
  pushing at full magnitude through a stale-link window while `cmd_vel`
  decays is the same open-loop divergence the inertia term is blocked on.
  No new firmware work — the same flash sitting covers both arcs.

## Phase summary

| Phase | Content | Gate | Status |
|---|---|---|---|
| 0 | Residual telemetry channel (identity tag + offline extractor) | reproduces the 2026-07-18 lag table | UNBLOCKED |
| 1 | Offline prototype + repeatability decision | heavy-leg inter-repetition ρ ≥ 0.6, else NULL-exit | gated G-A/G-B/G-C |
| 2 | Production wiring, ship dormant | full suite + tripwire tests | after 1 |
| 3 | Hardware A/B, batch learning between runs | pre-registered criterion + abort signatures | operator |

## Phase 0 — Residual telemetry channel *(unblocked, independently valuable)*

Everything needed is already bagged: `/leg_setpoint_echo` (accepted u0,
motor revs), `/robot_state` (measured `pos_estimate`, 100 Hz),
`/trajectory/diagnostics` (`move_seq` per-move boundary,
trajectory_node.py:381), `/link_status` (`uptime_ms`). Two gaps close here:

- **T0.1 Identity tag.** Publish the trajectory-identity hash and
  model-config hash on `trajectory/diagnostics` (KeyValue, alongside
  `move_seq`) so bags carry plan identity, not just segmentation.
- **T0.2 Extractor.** `tools/probes/residual_extract.py`: rosbag → per-move,
  per-leg residual profiles (u0−enc resampled to the 25 ms knot grid) +
  cross-correlation lag, keyed by (move_seq, identity hash, uptime_ms).
  Outputs to `temp/probes/`. Committed with a README row per the
  reusable-probe rule — this is a recurring harness, not a one-off.
- **T0.3 Output schema** documented in this section on implementation — it
  is the shared spec three consumers read: the accel-FF A/B (accel-phase
  windows and full profiles from the same extraction), this plan's learner,
  and the latency monitoring that closes the 2026-07-18 investigation.
- **Gate 0:** the extractor reproduces the 2026-07-18 seven-bag lag table
  (10/40/160/130/250/240/230 ms) within grid resolution — validation against
  already-trusted forensic results.

## Phase 1 — Offline prototype + repeatability decision *(no hardware, no wiring)*

Consumes ≥2 same-session repetitions of the 22-move lateral battery on a
healthy (post-G-A) plant — the first post-fix sessions produce this
automatically once Phase 0's tooling exists.

- **T1.1 Repeatability measurement.** Inter-repetition correlation ρ of
  residual profiles, per leg per move. **Pre-registered decision gate:
  heavy-leg ρ ≥ 0.6 proceeds; below it the plan NULL-exits** — the residual
  is not dominated by repeatable disturbance, nothing ships, and the
  residual question escalates to the velocity-loop-bandwidth / replanner
  track (frozen gains, out of scope here). A null is a legitimate,
  publishable outcome — not a failure to be rescued.
- **T1.2 Update-law prototype** (offline, on recorded repetitions):
  r_{k+1} = clamp(r_k + γ·Φ(e_k)) with γ ≤ 0.3; Φ = zero-phase low-pass
  (batch learning permits non-causal filtering — an advantage online ILC
  does not have) plus an optional constant lead; the e→τ scaling derives
  from the 2026-07-13 plant ID (Kt 0.0570). Validate by held-out prediction:
  the learned profile must predict a ≥30%-class heavy-leg deviation
  reduction on a repetition it was not trained on.
- **Gate 1:** a sizing memo fixing γ, Φ, TRIM_MAX (headroom arithmetic per
  design constraint 3), and the key quantization. Operator checkpoint before
  any wiring.

## Phase 2 — Production wiring *(software only, ship dormant)*

`dynamics.torque_ff_learned: false` + codegen; profile store/loader; attach
at install; `KnotEmitter.frame` adds the sampled trim into `torque_ff`
before `make_mpc_command` (emitter.py:106-118 region). Tests: pump-acceptance
invariant (frames with trim are accepted by a real `SetpointPump`),
clamp-budget property (model + trim ≤ pump clamp at battery dynamics),
byte-identical-OFF, key-miss ⇒ exact zeros, and a
`test_shipped_config_has_the_feature_*` tripwire so the shipped state is a
deliberate, logged act. Full suite + ci-deep with the (date, command,
result) triple.

## Phase 3 — Hardware A/B *(operator-run, batch learning between runs)*

Protocol mirrors the accel-FF A/B (matched approach history, consistent
supply/warmth, `traj_ramp_battery.py` read-only). Learning pass k: run the
battery → offline update → re-run. **Pre-registered success criterion:**
≥30% heavy-leg accel-phase peak-deviation reduction by k ≤ 3, with act_std
non-increasing on every leg. **Abort signatures:** (i)–(iv) of the accel-FF
protocol verbatim, plus (v) any repetition whose deviation *increases* vs
the previous → freeze learning, revert to last-good profile; (vi) any
trim-budget saturation → stop and resize offline. Ship decision: enabled
with frozen profiles, or dormant.

## Verification

- `pytest tests/ -q` after every `*.py`/`*.yaml` change and as the
  pre-commit gate; config edits follow edit → `generate_config.py` → stage →
  test → commit.
- Each phase closes with a logbook entry carrying a real Discussion section;
  test-count claims cite the (date, command, result) triple.
- The Phase-3 A/B is the load-bearing hardware verification; criteria and
  aborts are pre-registered above before the first run.

## Out of scope

- Reference warping and velocity-knot learning (design constraint 1).
- Per-tick feedback of any kind; online/continuous adaptation (v1 is batch).
- Follower/SpaceMouse trajectories — non-repetitive by nature; no key ever
  matches, which is the correct behaviour.
- Catch trajectories with per-throw varying targets: BB scatter breaks exact
  keying. The natural first juggling consumer is
  `plans/active/single-ball-toss.md` Tier 8a (toss-at-position — fixed
  nominated targets, genuinely repeated). Revisit catch-side learning only
  with evidence from that tier.
- Leg gains (FROZEN per `plans/active/leg-gain-tuning-methodology.md`); the
  hand chain; the dormant MPC path.

## Risks (ranked)

1. **Limit-cycle class (2026-05-08 canonical).** Mitigations: the trim is a
   low-passed profile on the knot grid (no discontinuities by construction),
   rides the shared `torque_ff_ramp_s` wire ramp, and the A/B carries the
   act_std / 5 Hz spectral abort signatures.
2. **Learning from a contaminated plant.** Mitigations: gate G-A; the
   learner's uptime_ms refusal; profiles record source-bag uptime.
3. **Model drift silently invalidating profiles.** Mitigation: the
   model-config hash in the key — profiles miss (zero trim) rather than
   apply stale corrections.
4. **A growing trim masking real plant degradation.** Inverted into an
   invariant: re-learned profile magnitude must shrink or hold
   session-over-session; growth is a plant-drift alarm that triggers
   investigation, never a bigger trim.

## Relation to the replanner direction

Orthogonal and complementary: ILC targets *tracking quality on repeated
primitives* with zero architecture change; the event-triggered replanner
(memory: foundation-first, 2026-06-27) targets planning optimality and
compute headroom. A Phase-1 null here *strengthens* the replanner case by
localizing the residual to velocity-loop bandwidth.
