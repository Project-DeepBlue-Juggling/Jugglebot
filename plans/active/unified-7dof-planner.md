---
title: Unified whole-cycle 7-DoF motion planner (Scope B)
created: 2026-08-29
status: proposed   # 2026-08-29 — owner-commissioned (Scope B chosen over carry-only Scope A the same day);
                   # promote to active when the Phase 0 probe results are recorded. The original MP-M2 gate
                   # DISSOLVED later the same day: the owner halted toss-multi-catch-pose at the pre-M2
                   # boundary (stop clean, no reverts) — see § 1 relationship table.
owner: Harrison
related_plan: toss-multi-catch-pose.md
related_code:
  - sim/juggle_planner/juggle_planner.py::plan_cup_cycle
  - sim/juggle_tilt.py::realize_tilted
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/planner.py
  - teensy_link/setpoint_pump.py::SetpointPump
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
---

# Unified whole-cycle 7-DoF motion planner (Scope B)

## 1. Context

### The problem

The platform (6-DoF Stewart pose) and the hand (1-DoF slider, ODrive node 6) are
planned and commanded by two disjoint systems that share only wall-clock
deadlines, never a trajectory or an objective:

- The platform path is pose-space quintic plans (`motion/trajectory/planner.py`)
  emitted as 40 Hz leg-space knots (`emitter.py`, `JB_TRAJ_KNOT_DT_S = 0.025`)
  over ZMQ :5557 → UDP `Setpoint` (six-wide, `PROTOCOL_VERSION 5`) → the
  can-bridge Teensy's 500 Hz cubic-Hermite interpolator (`leg_interp.cpp`,
  Mode 1) → six `set_input_pos` frames per 2 ms tick, guarded by
  `MAX_DEVIATION_REV` and the stroke clamp.
- The hand is a **one-shot scalar dispatch**: the Jetson picks `event_vel`,
  ships 8 opaque bytes on the `HAND_TRAJ_CMD` RPC, the can-bridge forwards them
  verbatim on CAN 0x6D0 (`hand_ops.cpp`), and the **Platform Teensy** builds the
  entire stroke from closed form (`Teensy_code_platform/Trajectory.h`) and
  streams 500 Hz `set_input_pos` (0x0CC) itself. `hand_stroke.py` is a
  host-side read-only mirror of that timing.

Consequences, all measured or documented in-tree:

1. **Ball-frame smoothness is inexpressible.** "Smooth from the ball's
   perspective" means keeping apparent gravity (`g − a_cup`) aligned with the
   cup axis and bounded; that couples platform tilt to the translational
   acceleration of platform-plus-hand and cannot be stated across two planners.
   The only approximation is the `LeanShaper` heuristic — which the now-halted
   `toss-multi-catch-pose.md` M2 *would have* turned OFF on the pre-tilt move
   for cadence (a measured 0.28–0.35 s cost), trading seating quality for
   speed because the split architecture offers no third option.
2. **The carry exists physically but is planned as nothing.** After a CAUGHT
   the sequencer's terminal is `ACTION_STAY`; the next cycle's POSITIONING
   `go_to_pose` carries the seated ball to the next pre-tilt pose while the
   hand is commanded nothing at all (`toss_session.py` — the catch stroke ends
   at 0 rev, a kind-0 throw starts at 0 rev). A fast DoF idles through every
   re-pose.
3. **The phase machine is a single-ball architecture.** `toss_sequencer.py`
   commands one subsystem per phase with zero-velocity endpoints between
   phases. Multi-ball (2–3 balls in one hand) requires overlapping per-ball
   timelines — a continuous, event-*constrained* trajectory, not phases.
   `bb-online-juggle-tilt-rearchitecture.md:340-341` names the root cause
   directly: "the harnesses stitch hand-rolled quintic phases instead of using
   the codebase's own whole-cycle `plan_cup_cycle`".

### What this plan builds

One planner that treats the robot as a single 7-channel entity. Per cycle it
produces **one cup trajectory** covering throw → flight-side re-pose (carry of
the next ball) → catch, with the ball-frame constraints as hard constraints
(release free-fall `a_cup == g`, detach cone `cross(a_cup − g, cup_axis) == 0`,
catch velocity matching), banking free during the carry, and decomposes it into
a synchronized 7-channel knot stream: 6 legs + the hand, all interpolated by
the same 500 Hz firmware lane under the same guard architecture.

Owner decisions recorded 2026-08-29 (conversation, this plan's commissioning):

- **Scope B** — the planner owns the whole cycle including the launch
  mechanism (cup free-fall release executed jointly by platform+slider), not
  carry-only. Re-validation of the toss surface is accepted as the cost.
- **Exclusive hand mastery**: end state is Jetson → can-bridge → hand ODrive,
  with the Platform Teensy stroke engine decommissioned. During migration a
  firmware `hand_source` latch makes dual mastery structurally impossible
  (§ 2.4); decommission is the final phase, not the first.
- **The z = 170 mm centroid pin stays the default.** The planner supports the
  platform centroid deviating from z = 170 (`unified_z_float_enabled`), but it
  ships **off**; when off, `realize` pins z exactly as today.
- **Sequencing**: **before** the `critical-point-ilc.md` build ladder; MP was
  halted at the pre-M2 boundary the same day (see below), so nothing else
  gates the start. Rationale: ILC's `e_model` is by
  design a call into the production planning chain
  (`tests/hardware/ilc_fit_lib.py` design constraint 1); Scope B replaces that
  chain's launch mechanism, so a corpus and sensitivities learned on the
  stroke-engine plant would be invalidated. ILC re-targets the unified chain
  and re-captures its corpus after Phase 5 (§ 6, ILC hand-off).

Architectural resolutions recorded 2026-08-29 (owner, multiple-choice review —
each is binding on the phases below):

1. **Catch runway**: deceleration runway is a hard planner constraint (the
   catch knot must leave `≥ v_cup²/(2·a_hand_max)` of slider stroke below it,
   plus margin), and the **sim acceptance authority is the kinematic-capture
   gate** (toss_gate style); MuJoCo contact runs advisory-only; the hardware
   ladder is the seating authority. This resolves the Rung-3 blocker by
   construction (§ 4, Phase 1).
2. **MP wind-down**: `toss-multi-catch-pose` stops clean at the pre-M2
   boundary, no reverts; ring-specific landed work stays unexercised; the plan
   archives as superseded by this one (relationship table below).
3. **Planner API**: event-timeline — the planner takes an ordered list of
   throw/catch events over a horizon window; v1 callers pass exactly one
   throw + one catch (§ 4, Phase 1).
4. **Replan policy**: plan at cycle commit + bounded catch-side replans on
   tracker updates; the next throw's boundary conditions stay fixed (§ 4,
   Phase 4). No receding-horizon loop.
5. **Wire velocities**: the v6 `Setpoint` carries exact segment-end
   velocities (`v1`, all 7 channels) behind a `HAS_V1` flag — decided now
   because the version bump is already being paid and a retrofit would cost a
   second lockstep flash (§ 2.3).
6. **Phase 6 trigger**: the stroke-engine retirement fires when the Phase 5
   UH ladder completes — no separate parity campaign; the ladder's own
   acceptance (including T-H6 outcome parity at the matched low tier) is the
   evidence base. Rollback images stay named artifacts (§ 4, Phase 6).

### Relationship to existing plans

| Plan | Relationship |
|---|---|
| `toss-multi-catch-pose.md` | **SUPERSEDED OUTRIGHT (owner, 2026-08-29): MP halted at the pre-M2 boundary** — stop clean, no reverts; general-correctness fixes from the 2026-08-29 waves stay (they serve every legacy sitting until Phase 6), no ring-specific deliverable ever landed — M0–M6 never ran (verified 2026-08-30, see the MP plan's Archival note). Archived as superseded 2026-08-30. Its § 7 constant-beat lever and its M3/M4 machinery are provided natively here; MP-M5's carry-seating observation re-homes to Phase 5 (T-H5); the § 7 unlock conditions (aim-authority re-derivation vs `MAX_TILT_DEG = 12°`) transfer to Phase 5 acceptance. |
| `mvp-trajectory-bringup.md` | This plan becomes the vehicle for its Phase 8/9 stretch goals (self-toss loop, two-ball). |
| `bb-online-juggle-tilt-rearchitecture.md` Rung 3 | Sim-side twin. Its P2 attempt (2026-07-03, ~35 probes) BLOCKED on "the slam is the seat's runway" — every cleanup of the whole-cycle sim catch dropped MAKE to 0/12. **Phase 1 must answer that failure in sim before any hardware phase starts** (§ 4, Phase 1 acceptance). |
| `critical-point-ilc.md` | Sequenced after this plan (owner). Its Phases 0–2 substrate (records, miner, gates) survives; the fit corpus, `SIGMA_E`, cell keys and the speed-authority band do not — re-capture on the unified plant. |
| `toss-pipelined-preamble.md` | Coexists. Shipped (`toss_pipeline_enabled` default true); the legacy pipelined path remains the fallback mode throughout. |
| `leg-bus-frame-drops.md` | Interaction: a 7th frame per 2 ms tick takes bridge TX ~3000 → ~3500 fps (~56.5 % → ~65 % streaming utilisation), intensifying the mechanism that plan indicts (ODrive-side TX suppression beating against the tick-quantised burst). Its workstream-B A/B (drop rate vs bridge TX rate) runs in Phase 0 here; if drops scale with TX rate, the source fix sequences before Phase 3 hardware streaming. **Answered 2026-08-30: they do not — the deficit is smaller at 7 frames/tick than at 6, and utilisation lands at 62.2 %, not 65 %. No prerequisite on Phase 3** (see Phase 0's Outcome). |

## 2. Architecture

### 2.1 Current

```
reload_coordinator ── toss_session/toss_sequencer FSM (phases, single ball)
      │ go_to_pose / timed_target / catch/dynamic_target        │ SetHandTrajCmd(event_vel, wall_time_ms)
      ▼                                                          ▼
trajectory_node                                          teensy_bridge_node
  planner.build_* → validate → TrajectoryPlan (6-DoF)      HAND_TRAJ_CMD RPC
  40 Hz emitter: state_at(τ) → IK ×3 → 6-wide knots               │
      │ ZMQ :5557 (msgpack 'mpc_cmd')                             │
      ▼                                                          ▼
teensy_bridge_node: SetpointPump → UDP Setpoint v5 (156 B)   can-bridge: hand_ops
      ▼                                                       preamble + 0x6D0 passthrough
can-bridge Teensy: 500 Hz Hermite ×6 → 6× set_input_pos          ▼
  [MAX_DEVIATION_REV, stroke clamp, lead clamp]           Platform Teensy: closed-form
      ▼                                                    stroke → 500 Hz 0x0CC
  leg ODrives (nodes 0–5)                                        ▼
                                                           hand ODrive (node 6)
```

Two masters for the hand bus traffic; two planners; one shared wall clock.

### 2.2 Proposed

```
unified cycle orchestrator (param-gated, JB_OP_UNIFIED_CYCLE_ENABLED)
      │  per cycle, OFF the emitter thread:
      │  (1) cup_cycle.plan(...)      — convex QP over cup Cartesian jerk (25 ms grid)
      │  (2) cup_realize.tilt_schedule — banking from apparent gravity, endpoint-pinned
      │  (3) cup_realize.decompose    — pose_6dof (z pinned to 170 unless z-float) + slider_rev
      │  (4) validate_cycle           — existing 6-DoF gates + new hand gates
      ▼
trajectory_node: CyclePlan (7-channel, one clock) installed like any plan
  40 Hz emitter: legs as today + hand_rev/hand_vel/hand lookahead keys
      │ ZMQ :5557
      ▼
teensy_bridge_node: SetpointPump (7-channel) → UDP Setpoint v6 (208 B, HAS_HAND/HAS_V1 flags)
      ▼
can-bridge Teensy FW 17: 500 Hz Hermite ×7 → 6× leg set_input_pos + 1× hand set_input_pos (0x0CC)
  [legs: unchanged guards]  [hand: MAX_DEVIATION_HAND_REV, clip [0, 10.8] rev, hand lead clamp]
  hand_source latch: LEGACY ⊕ STREAMED  — HAND_TRAJ_CMD refused while STREAMED, hand
  channel discarded (counted) while LEGACY.  Dual mastery structurally impossible.
      ▼
leg ODrives (0–5) + hand ODrive (6)

Platform Teensy (end state, FW 4): stroke engine + 0x6D0 decode + 0x0C9 hand-encoder
cache RETIRED.  RETAINED: 0x6E0 cold-start state + FW identity, SCL3300 inclinometer
(the only tilt sensor), time-sync slave.
```

### 2.3 New/changed message formats

**UDP `Setpoint` v6** (`config/generate_udp_protocol.py`): all six f32 arrays
widen 6 → 7 (index 6 = hand, ODrive-convention absolute rev, **no sign flip**,
hand wire scales 100/100 vs leg 1000/10000 — `odrive_protocol.h::
encode_leg_setpoint` already handles axis 6 correctly), **plus a new `v1`
f32[7] array — the exact velocity at the u1 knot — behind flag bit 3
`HAS_V1`** (owner, 2026-08-29). With `v1` carried, the 500 Hz Hermite
reconstruction is exact for piecewise-cubic plans on every channel; the
firmware's `v1 = (u2 − u1)/SEG_T` forward difference remains the fallback
when the flag is clear. `SETPOINT_SIZE` 156 → 208 (49 f32 + u32 + u64). New
flag bit 2 `HAS_HAND`; when clear the firmware ignores index 6 and emits no
hand frame. **This is an incompatible wire change ⇒
`PROTOCOL_VERSION` 5 → 6 ⇒ total link darkness against any board not flashed
in lockstep** (`decode_frame` hard-rejects on version). The additive-MsgType
alternative (a parallel `HandSetpoint` frame) is rejected: two frames per knot
would need cross-frame atomic latching in the ISR staging path, introducing a
torn-latch class the single-frame widening removes by construction.

**ZMQ `mpc_cmd` dict** (`motion/ipc.py::make_mpc_command`): leg arrays stay
6-wide (so `motor_guard.py`'s `shape == (6,)` asserts and
`controller/hardware_plant.py` stay untouched and decodable). New optional keys:

```python
{
  # ... existing 6-wide keys unchanged ...
  'vel_next_mm_s':     [...],   # 6-wide leg extension rates at the u1 knot → v1[0:6]
  'hand_rev':          9.9594,  # ODrive absolute rev, u0 for the hand lane
  'hand_vel_rps':      -12.4,   # rev/s, v0
  'hand_next_rev':     9.71,    # u1
  'hand_next2_rev':    9.32,    # u2
  'hand_next_vel_rps': -9.8,    # rev/s at the u1 knot → v1[6]
}
```

Hand keys absent ⇒ pump clears `HAS_HAND`; `vel_next` keys absent ⇒ pump
clears `HAS_V1` ⇒ behaviour identical to today. `HAS_V1` requires every
active channel's v1 (`vel_next_mm_s` always; `hand_next_vel_rps` whenever
the hand keys are present) — a partial set **rejects the frame** loudly, per
the NaN-reject precedent in T-U7. When emitting a `CyclePlan` (unified mode)
the emitter supplies leg `vel_next_mm_s` from a `twist_to_leg_velocities`
call at τ+dt on the Jacobian `_ik(pose1)` already computes — one extra
matrix-vector product per tick; legacy plans emit no `vel_next` keys, so
`HAS_V1` stays clear and the firmware fallback is the flown path until
Phase 4.

### 2.4 The `hand_source` interlock (firmware)

A latched mode on the can-bridge: `LEGACY_STROKE` (boot default) |
`STREAMED`. Switchable only via a new **additive** RPC (`HAND_SOURCE_SET`,
new MsgType — additive frames need no version bump), accepted only while
`!mpc_active` and the hand is inside `HAND_SETTLE_BAND_REV` of a rest
position. While `STREAMED`: `hand_ops::hand_traj_cmd` returns a new
`ERR_HAND_SOURCE` and the interp emits the hand frame. While `LEGACY`: the
interp discards Setpoint index 6 (counted on a visible counter) and 0x6D0
forwarding works as today. The latch state rides `HeartbeatT2J` flags so
`/link_status` can display it. This makes the migration period safe: both code
paths exist, but never both masters.

### 2.5 What stays the same

The transport and safety framework: 40 Hz knots → single-slot ISR staging →
500 Hz Hermite → tick-quantised TX; `MAX_DEVIATION` guard architecture and its
CLEAR_ERRORS-only latch release; leg homing and hand homing (`leg_homing.cpp`,
axis 6 already homes on the shared machine); the hand ball-sensor poller
(bridge-only, unchanged); the legacy sequencer/session stack as the fallback
mode; ball tracking, possession verdicts, `outcome_detail` discipline.

## 3. Implementation Phase Summary

| Phase | Scope | Status | Date | Risk | Validates |
|-------|-------|--------|------|------|-----------|
| 0 | Probes + recorded decisions: QP solver runtime on Jetson 3.8, Hermite stroke-reconstruction fidelity, bus headroom with a 7th frame (+ leg-bus-frame-drops A/B), hand guard constants derivation | COMPLETE | 2026-08-30 | Low | In-process QP is feasible; 7-frame bus budget is safe |
| 1 | Planner core port (pure Python): `cup_cycle` QP, tilt schedule, `realize` generalisation, `CyclePlan`, `validate_cycle`, sim parity + MuJoCo whole-cycle gate | COMPLETE | 2026-09-01 | Low (software only) | Ball-frame constraints hold; the Rung-3 "runway" failure is answered in sim |
| 2 | Wire v6 + host 7-channel path: codegen, `SetpointPump`, emitter, `make_mpc_command`, tests; firmware-absent safe | NOT STARTED | | Medium | Codec, per-channel step gates, backward-compatible producers |
| 3 | Can-bridge FW 17: 7th interp lane, hand guards, `hand_source` interlock, dispatch; lockstep flash + bench ladder | NOT STARTED | | High | Hand streaming safety envelope on real hardware |
| 4 | Jetson unified-cycle mode: orchestrator, node wiring, plan-derived announcements/suppression, outcome vocabulary; end-to-end sim gate | NOT STARTED | | Medium | Whole cycle through the production stack in sim |
| 5 | Hardware ladder: streamed hold → banked carry (ball seated) → planned catch → planned throw (low tier) → full cycles → two-pose constant beat | NOT STARTED | | High | Ball-smooth carry and the planned launch on hardware |
| 6 | Exclusivity + close-out: Platform Teensy FW 4 stroke retirement, host RPC retirement, contract doc, ILC hand-off, docs | NOT STARTED | | Medium | Single-master end state |

Phase 0 runs while this plan is `proposed` — its recorded results are the
promotion gate (the MP-M2 gate dissolved 2026-08-29 with the MP halt).
Phases 1–2 are software-only. Phase 3 onward requires `active`.

## 4. Implementation Phases (detailed)

### Phase 0: Probes and recorded decisions — COMPLETE 2026-08-30

**Probes** (one-off drivers in `/tmp/probe_*.py`; anything reusable promotes to
`tools/probes/` per `tools/probes/README.md`):

1. **QP solver probe.** `plan_cup_cycle`'s problem is a convex QP once
   `detach_axis` is treated as the fixed parameter it already is: dynamics are
   an exact linear triple integrator in the 3×n jerk decision (n = 25 at the
   demo period), the objective is quadratic, and every hard constraint —
   release free-fall, detach cross-product, interpolated catch position, jerk
   boxes, workspace boxes — is linear. Prototype an equality-constrained KKT
   solve (dense `numpy.linalg.solve`, ~75 variables + ~20–40 equalities) with
   post-hoc box checks and a bounded active-set or duration-stretch fallback.
   Targets on this Jetson under system Python 3.8: **< 10 ms steady-state**,
   trajectory parity with the CasADi/IPOPT reference (measured 0.23–0.27 s on
   this Jetson) within 1 mm / 10 mm/s over the demo cycle set. Escape hatch if
   parity or runtime fails: CasADi in a separate venv process bridged over ZMQ
   (the `run_mpc.py` pattern) — record the decision either way.
2. **Hermite stroke-fidelity probe.** The planner's 0.025 s jerk grid aligns
   1:1 with the wire knot grid, so the cup plan is piecewise-cubic on exactly
   the firmware's segment boundaries, and the v1-carriage decision (§ 2.3)
   makes reconstruction exact by design. The probe therefore (a) verifies
   the v1-exact reconstruction end-to-end against the closed-form stroke at
   `event_vel` 3–7 m/s (expected: exact to float precision), and (b) records
   the legacy `v1 = (u2 − u1)/SEG_T` forward-difference error for the record
   (the `HAS_V1`-clear fallback path). No wire-design go/no-go remains here.
   (Measured 2026-08-30: exact to float precision only on knot-aligned cubics —
   the planner's own output; ≤ 3.25 mm worst-case over grid phase on the legacy
   closed-form stroke, vs ≤ 11.84 mm for the fallback. See the Phase 0 entry's
   scope split.)
3. **Bus headroom + frame-drop A/B.** On the bench, a `BENCH`-guarded firmware
   variant emits a 7th (hand, current-position-hold) frame in the burst.
   Measure: `PROFILE` `can1_util_x100`, `tx_deferred`/`tx_q_hwm` (must stay 0),
   `[cantx]` per-class census, and the `leg-bus-frame-drops` per-axis
   encoder-frame drop rate at 6 vs 7 frames/tick. Predicted: ~3500 fps, ~65 %
   streaming utilisation, design-bound pending frames 8 → 9 (≤ 16 mailboxes,
   not re-opened). **If the drop rate scales with TX rate, the
   `leg-bus-frame-drops` source fix sequences before Phase 3.** Re-read
   `lib/FlexCAN_T4/PROVENANCE.md` § P3/P4 against the widened burst and record
   the verdict.
4. **Hand guard constants.** Derive and record: `MAX_DEVIATION_HAND_REV`
   (velocity-aware — at 221 rev/s peak stroke speed the **measured** 10–15.9 ms
   end-to-end telemetry latency alone produces **2.2–3.5 rev** of apparent
   deviation; the earlier "~1 rev" figure here was unsourced and implies 4.5 ms
   — see the Phase 0 entry's *Withdrawn claims*. The 10 Hz fault-task guard
   needs either a velocity-compensated bound or a bound sized to worst honest
   lag), `MAX_LEAD_HAND_REV` (the legs' 0.10 rev is meaningless at hand
   speeds), the planner caps `JB_TRAJ_HAND_VEL_LIMIT_RPS` (provisional 250;
   ceiling: 7 m/s × 31.617 rev/m ≈ 221 rev/s) and
   `JB_TRAJ_HAND_ACC_LIMIT_RPS2` (provisional 3500, under the measured
   4178–4333 rev/s² axis ceiling in
   `ros_ws/docs/hand_command_continuity.md`).
   **Signed off 2026-08-30 (owner):** the provisional single caps become
   **two-tier** (limit/ceiling, the leg pattern) —
   `hand_vel_limit_rps` **200** / `hand_vel_ceiling_rps` **300** (200 is +12 %
   over the C-HAND-3 certified 178.23 rev/s peak; the provisional 250 left a
   40 % invisible band, and the planner cap is the only practical hand
   overspeed guard today), and `hand_acc_limit_rps2` **3500** /
   `hand_acc_ceiling_rps2` **3900**, under the **C-HAND-2 authority bound
   3925.5 rev/s²** (that bound binds, not the measured 4178–4333 ceiling).
   Plus a coupled fifth constant this list missed, `HAND_VELFF_LIMIT_RPS`
   **300**, and a new `fault_machine` hand overspeed guard at ~1.15× ceiling.

**Dependencies:** none. **Deliverable:** a short logbook entry recording the
four decisions with the (date, command, result) triples.

**Outcome (2026-08-30) — all four probes closed; probe 3 flown on the bench.**

- **1. QP solver: IN-PROCESS numpy QP, a Goldfarb–Idnani dual active set.**
  Parity vs the CasADi/IPOPT reference **1.91e-4 mm / 2.47e-3 mm/s** worst case
  over 6 cases (bars 1 mm / 10 mm/s), **p50 5.577 / p90 6.367 ms on n=40**
  under system Python 3.8 — the escape hatch is **not taken**, and IPOPT was
  additionally caught **falsely refusing a feasible cycle** the QP solves in one
  iteration.
- **2. Hermite: v1-exact is float-exact (≤ 3.6e-15 rev) on knot-aligned
  piecewise cubics** — the planner's own output — and **≤ 3.25 mm on the legacy
  closed-form stroke vs ≤ 11.84 mm** for the forward-difference fallback.
- **3. Bus headroom: FLOWN 2026-08-30 — comfortable, and the drop rate does not
  follow TX rate.** Rows 11–14 on one boot (bag
  `~/Desktop/rosbags/2026-08-30_15-59-52/`): `can1_tx` **3150 → 3650 fps,
  exactly +500**, twice, so the 7th frame demonstrably flowed;
  `can1_util_pct` **56.7 → 62.2 %** (below the pinned ~65 %, well under ~70 %);
  `defer jb` increment **0** and `txq jb` **0** every arm; `leak_* ≡ 0`,
  `can3_errors` all-zero, `interp_deadline_misses` 0 with `interp_max_jitter_us`
  peaking at 2 µs in every arm, `latency_monitor` OK ×3358. **Decision rule answers
  NO:** per-axis encoder deficit **−0.76 ± 5.68 at 6 frames/tick** vs
  **−0.28 ± 3.73 at 7** (battery-moving windows only: −1.06 ± 7.06 with 3
  episodes vs −0.02 ± 0.25 with 0), smaller at the higher rate and inside the
  A-vs-A′ within-boot spread — so the `leg-bus-frame-drops` source fix **does
  not sequence before Phase 3**. Caveats recorded in the runbook: only three of
  four batteries are in the bag (arm A quiescent), and this plant runs ~5×
  cleaner than the 2026-08-15 reference, so the +16 % (6→7 frames/tick;
  3150→3650 fps) arm had little dynamic range. **Arms C (drives quiet) and D (250 Hz) NOT flown** — C deferred by the
  operator, D's `INTERP_RATE_HZ` 250 companion build never authorised; row 16's
  larger lever is now the informed follow-up. The FlexCAN P3/P4 desk verdict
  against the widened burst is clean, and FW 16 is now aboard.
- **4. Hand guards owner-signed:** `MAX_DEVIATION_HAND_REV` **2.5 rev**
  velocity-compensated with the residual computed in the 500 Hz interp tick,
  `MAX_LEAD_HAND_REV` **2.0 rev** freshness-aware, `HAND_VELFF_LIMIT_RPS`
  **300**, caps **200/300 rev/s** and **3500/3900 rev/s²**, and a
  `fault_machine` hand overspeed guard to be added.

**Phase 1's dependencies (decisions 1 and 2) are satisfied — Phase 1 is cleared
to start.** Probe 3's bench numbers are now recorded, so the promotion gate
(`proposed` → `active`) has its evidence and awaits only the owner's flip;
Phase 3 carries **no** `leg-bus-frame-drops` prerequisite. The canonical records
are
[`logbook/2026-08-30-unified-7dof-planner-phase0-probes.md`](../../logbook/2026-08-30-unified-7dof-planner-phase0-probes.md)
(desk probes) and
[`logbook/2026-08-30-unified7-bus-headroom-sitting.md`](../../logbook/2026-08-30-unified7-bus-headroom-sitting.md)
(the bench sitting), with the full arm table in
[`tests/hardware/session_unified7_bus_headroom.md`](../../tests/hardware/session_unified7_bus_headroom.md) § Results;
commits carry the `Logbook-Entry:` trailer, so no SHAs are recorded here.

### Phase 1: Planner core port (pure Python, no hardware) — COMPLETE 2026-09-01

**New files** (all `ros_ws/src/jugglebot/jugglebot/motion/trajectory/`, pure
Python, numpy-only, `from __future__ import annotations`, Python 3.8):

- `cup_cycle.py` — the QP port of `sim/juggle_planner/juggle_planner.py::
  plan_cup_cycle` per the Phase 0 decision, with an **event-timeline API**
  (owner, 2026-08-29): `plan_window(events, state0, cfg)` takes an ordered
  list of throw/catch events (ball id, time, site, velocity/target) over a
  horizon window — v1 callers always pass exactly one throw + one catch, so
  parity with `plan_cup_cycle` is asserted on that case and 3-ball later
  becomes data, not an interface rewrite. Same trajectory outputs
  (`CupCyclePlan`: pos/vel/acc knots + jerk + `catch_k` + `takeoff_vel`),
  same constraint set plus one addition: **the catch-runway hard constraint**
  (owner, 2026-08-29) — the catch knot must leave
  `≥ v_cup²/(2·a_hand_max)` of slider stroke below it, plus margin, so
  deceleration room is planned rather than obtained by ceiling overshoot.
  Warm-startable from the previous cycle. **The Phase 0 decision binds the port
  to a Goldfarb–Idnani dual active set** — the naive KKT-plus-violated-boxes
  loop this section's earlier wording suggested was shown to *cycle*, returning
  silently wrong trajectories (working sets to rank 37/53, `cond(K)` ~1e23,
  answers 124 m off); see the Phase 0 logbook entry.
- `cup_realize.py` — stage 2 + 3. `tilt_schedule(cup_plan, receive_tilt,
  throw_tilt)`: banking from apparent gravity — the cup axis tracks
  `normalize(g − a_cup(t))`, saturated at `tilt_geometry.MAX_TILT_DEG` (12°),
  rate-limited, endpoint-pinned to the receive tilt at catch and the throw
  tilt (= `detach_axis`) at release. `decompose(cup_plan, tilts)`: the
  generalisation of `sim/juggle_tilt.py::realize_tilted` — per knot,
  `pose_6dof = [centroid_xy, z, rx, ry, 0]` + `slider_rev`, using the
  height-invariant rotation centre (`CUP_TILT_CENTER_Z_MM = 744.3`) lever-arm
  model. **z behaviour:** `unified_z_float_enabled` false (default) pins
  z = `JB_OP_DEFAULT_ACTIVE_Z_MM` exactly as today; true lets z absorb up to
  `unified_z_band_mm` of the vertical stroke when the slider saturates,
  gated by `validate`'s leg-workspace checks rather than the pin.
- `cycle_plan.py` — `CyclePlan`: a 7-channel plan object sharing one clock;
  `state_at(t)` returns the 6-DoF pose state (TrajectoryPlan-compatible, so
  the emitter and `_install` machinery reuse it) plus `hand_at(t) →
  (rev, rev_s)`. Piecewise-cubic on the 0.025 s grid by construction.
- `feasibility.py` (modified) — `validate_cycle(cycle_plan, limits, geom)`:
  runs the existing 6-DoF gates on the pose track, plus new hand gates:
  slider stroke `[0, JB_OP_HAND_CATCH_PRIME_REV]` operating /
  `[0, GEOM_HAND_MOTOR_HARD_STOP_REVS]` hard, hand vel/acc caps, per-knot
  hand step bound. New codes `HAND_STROKE`, `HAND_LIMIT_VEL`, `HAND_LIMIT_ACC`
  join the existing vocabulary (`outcome_detail.bound_msg` discipline).

**Config** (`config/hardware_config.yaml` → `python config/generate_config.py`):
`jugglebot_operational.unified_cycle_enabled` (false),
`trajectory_op.hand_vel_limit_rps`, `trajectory_op.hand_acc_limit_rps2`,
`trajectory_op.unified_z_float_enabled` (false),
`trajectory_op.unified_z_band_mm`.

**Sim validation** (the phase gate): a MuJoCo whole-cycle harness derived from
`sim/juggle_online.py` runs the ported planner through
`realize`-equivalent decomposition with **banking enabled** (the sim's level-
platform `realize_tilted` pins z and rz; the port must reproduce its output
bit-comparably at zero banking, then improve on it with banking on).
**Acceptance (owner, 2026-08-29 — resolves the Rung-3 collision):** the sim
authority is the **toss_gate-style kinematic capture model**. The whole-cycle
set must pass the kinematic-capture gate (`core_clean`-equivalent bands) with
the runway constraint active and no ceiling-overshoot slam anywhere in the
planned trajectory; **MuJoCo contact runs advisory-only** (reported, never
gating). Grounds: `sim/toss_gate.py` already deliberately avoids MuJoCo
contact as the low-fidelity element, hardware catches are already smooth
(bench fact), and the Rung-3 P2 failure was a fragile equilibrium of exactly
that contact model. The hardware ladder (Phase 5) is the seating authority.

**Dependencies:** Phase 0 decisions 1 and 2.

**Outcome (2026-09-01) — all four work packages landed; the sim phase gate
PASSES.** Canonical record:
[`logbook/2026-09-01-unified-7dof-planner-phase1-planner-core.md`](../../logbook/2026-09-01-unified-7dof-planner-phase1-planner-core.md).

- **WP1 `cup_cycle.py`** — the Goldfarb–Idnani port with the event-timeline
  `plan_window` (one-throw-one-catch bit-identical to the legacy signature) and
  the catch-runway constraint (in-QP linear bound + an analytic `CATCH_RUNWAY`
  gate that is a **refusal** on a descending catch, since the catch-position
  equality leaves the touch-down height no freedom). Parity **1.906e-4 mm /
  2.471e-3 mm/s** over 6 committed fixtures; 30-point T-U1 grid worst release
  `‖a−g‖` **6.0e-14**, detach cross **3.0e-15**, catch **2.4e-11 mm**. A
  **second silent-wrongness class** was found and closed — a knife-edge
  infeasible 0.40 s cycle terminated with every inequality satisfied and
  equality residual **121**, so every solve now verifies feasibility ≤ 1e-7 and
  refuses otherwise.
- **WP2 `cup_realize.py` + `cycle_plan.py`** — apparent-gravity banking (2-norm
  cap + rate limit; a per-axis-rate bug leaking **12.124°** past the 12° cap was
  caught by the T-U3 cap assertion) and the 744.3 mm lever decomposition with
  `CUP_Z_BASE_MM` generalised to `base(z)`, reducing to the literal bit-exactly
  at the pin. **Zero-banking parity vs `realize_tilted` asserted at
  `max|Δ| == 0.0`.** `CyclePlan` honours the `TrajectoryPlan` `state_at`
  contract; the zero-segments blindness and the terminal-hold cliff are
  documented and pinned.
- **WP3 config + `validate_cycle`** — the six keys above landed with regenerated
  artifacts (**CONFIG FRESH, 14 artifacts**), and `validate_cycle` (+452 lines)
  samples the knot grid directly with `HAND_STROKE` / `HAND_LIMIT_VEL` /
  `HAND_LIMIT_ACC` each probe-confirmed deterministic before pinning. **Hand
  extrema are closed-form, not sampled** — knot-grid FD under-measures accel
  **38 %** (2763.5 vs 4432.5 rev/s²) and would false-accept against the 3500
  cap. `SetTrajectoryLimits.srv` is deliberately **not** widened (interfaces
  rebuild → Phase 2/4).
- **WP4 the phase gate** (`sim/cycle_gate.py` + 17 unmarked tests) — **PASS**
  (2026-09-01, `python sim/cycle_gate.py`, 10.5 s): **11 points**, capture
  distance **0.000 mm** everywhere, zero-banking parity **0.0**, banking beats
  level on apparent-gravity misalignment **11/11**, slam-free, worst runway
  margin **106 mm**. MuJoCo contact is **advisory-only at 4/11 makes** — the
  predicted Rung-3 P2 contact-model signature, never gating. Two planner fixes
  were required: accel-bounded banking (`tilt_accel_limit_rad_s2` 5.326 rad/s²
  by convexity; the projection sweep was measured and rejected) and cfg-gated
  cup accel boxes shipped **off** with a recorded negative result (the box does
  not buy flight time).
- **HEADLINE for tier planning: the maximum plannable flight is 0.80 s** under
  the 3500 rev/s² hand cap with z pinned (release z 0.86 m → 3256 rev/s²;
  0.85 s → 3678; ≥ 1.05 s refuses; above 0.88 m the pre-launch dip no longer
  fits). **The lever is release height, not a planner knob.**

**Four owner decisions are recorded and OPEN** (entry § Open Questions): (1) the
**shipped-limit verdict** — every gate cycle reads `LIMIT_JERK` at the shipped
leg jerk 30000 for a structural reason (the z-launch jerk leaks
`sin(tilt) × 744.3 mm` into centroid xy; 0° → 17005, 3.5° → 28970, 4° → 33339,
12° → 107815 mm/s³, mesh-converged), so the gate runs catch-capable session
limits (250/3000/150000) and reports `shipped_limit_verdicts` beside them —
raise the shipped limits, or have unified mode always ride session limits;
(2) the 0.9 s flight advisory band (4371 > 3500, with a test that fails if it
ever fits); (3) the smoothing accel cap yielding to the endpoint pins on short
cycles (2/30 grid cases ~6 % over); (4) `v_match` deferred mirroring
`toss_gate` (uniform 0.316 = 1 − `catch_slider_vel_ratio`, by design).

**Phase 2 is cleared to start (software-only).** Phases 1–2 are permitted while
this plan is `proposed`; **Phase 3+ requires `active`.**

### Phase 2: Wire v6 + host 7-channel path — NOT STARTED

**Modified files:** `config/generate_udp_protocol.py` (Setpoint 6 → 7 plus
the `v1` f32[7] array, `HAS_HAND` bit 2, `HAS_V1` bit 3,
`SETPOINT_SIZE` 208, `PROTOCOL_VERSION` 5 → 6) + regenerated
`config/generated/udp_protocol.py` / `.h`; `teensy_link/setpoint_pump.py`
(7-channel: per-channel `max_step_rev` — leg 0.3 rev, hand from Phase 0;
`_finite_vec` widths; hand keys → index 6; `torque_ff[6] = 0`; the `v1`
array filled from `vel_next_mm_s` + `hand_next_vel_rps` with `HAS_V1`
cleared when absent);
`motion/ipc.py::make_mpc_command` (the five optional hand keys +
`vel_next_mm_s`, § 2.3);
`motion/trajectory/emitter.py` (sample `CyclePlan.hand_at` at τ, τ+dt, τ+2dt
when the plan carries a hand track; emit the hand keys); `trajectory_node.py`
(the per-knot step backstop covers the hand channel with its own bound);
`teensy_link/synthetic_setpoint.py` + `replay_setpoint.py` (channel-count
plumbing); `tools/probes/traj_stream_probe.py`; the full test set named in
§ 5.

**Critical details:** leg arrays in the ZMQ dict stay 6-wide —
`motor_guard.py` and `controller/hardware_plant.py` are untouched.
`hardware_config.h` regeneration rides the same commit. The
`extra_script.py` incremental-build hazard from the 4 → 5 bump applies:
Phase 3's firmware build must be clean, not incremental. After this phase the
host cannot talk to an FW ≤ 16 board (version darkness is loud and
fail-closed); the phase is therefore **committed but not deployed** to the
robot until Phase 3's flash sitting.

**Dependencies:** Phase 1 (for `CyclePlan`; the pump/emitter changes
themselves only need the key names).

### Phase 3: Can-bridge FW 17 — 7th interp lane, hand guards, interlock — NOT STARTED

**Modified files** (`ros_ws/src/jugglebot/Teensy_code_canbridge/`):
`leg_interp.cpp/.h` (Staging and all per-channel state arrays 6 → 7; latch,
isfinite loop, all three ladder modes, recovery slew, `interp_reset`,
`interp_base_pos` bounds; Mode 1 uses the transmitted `v1` when `HAS_V1`,
falling back to the `(u2 − u1)/SEG_T` forward difference when clear; hand
lane gated on `HAS_HAND` and `hand_source == STREAMED`; TX emits the 7th
`set_input_pos` — 0x0CC — in the same burst); `canbridge_config.h` (`MAX_DEVIATION_HAND_REV`,
`MAX_LEAD_HAND_REV`, hand stroke clamp bounds `[0.0, HAND_MOTOR_MAX_POSITION]`,
`FW_VERSION` 17); `fault_machine.cpp` (hand joins the guard loops with its own
constants — the leg `MAX_MOTOR_VEL_RPS = 16.5` overspeed number must NOT be
applied to the hand; same latch/snapshot/CLEAR_ERRORS machinery);
`hand_ops.cpp` (+ `ERR_HAND_SOURCE`); the new `HAND_SOURCE_SET` RPC
(additive MsgType) and its host plumbing in `teensy_link/` +
`teensy_bridge_node.py`; `can_buses.cpp` (`HAND_CMD_ECHO` goes silent under
SRX_DIS once the bridge masters the hand — re-source `hand_telemetry`
`pos_cmd/vel_ff_cmd/tor_ff_cmd` from `axes[6].target_*`); hand axis
CLOSED_LOOP + POSITION/PASSTHROUGH arming folded into the existing
MPC-arming contract when `hand_source == STREAMED` (the `hand_ops` preamble
is the current owner of that transition).

**Bench validation** (E-stop ready, no ball, run before any robot sitting):
the T-H1..T-H4 ladder in § 5. The flash event is **lockstep**: FW 17 + the
Phase 2 host checkout in one sitting (also discharges the pending FW 16
poller-cadence flash — 16's behavioural content is carried forward into 17).
`EXPECTED_BRIDGE_FW_VERSION` → 17 in the same commit.

**Dependencies:** Phase 2; Phase 0 probes 3 and 4; the `leg-bus-frame-drops`
verdict (§ 1, prerequisite row) — **satisfied 2026-08-30, no source fix
required first**.

### Phase 4: Jetson unified-cycle mode — NOT STARTED

**New file:** `motion/unified_cycle.py` — the pure-Python per-cycle
orchestrator (no ROS imports): given session goals (throw target, flight
time, catch site, beat period) and the measured state, call
`cup_cycle` → `cup_realize` → `validate_cycle`, produce the `CyclePlan` +
plan-derived release/catch metadata. Planning runs **off the emitter thread**
(the determinism rule: no solve, no blocking I/O in the 40 Hz loop) —
per-cycle, in the coordinator's service context, budget ≤ 50 ms.

**Replan policy (owner, 2026-08-29): plan at commit + bounded catch-side
replans.** On a tracker landing update, only the catch-side tail of the
committed cycle re-plans, installed through the existing C2 continuity /
`_install` machinery (the dynamic-target pattern today); the next throw's
boundary conditions stay fixed so the beat holds. Replan count is bounded and
every output re-validates through `validate_cycle`. No receding-horizon loop.

**Modified files:** `reload_coordinator_node.py`
(`_execute_toss_continuous` routes to the unified path when
`JB_OP_UNIFIED_CYCLE_ENABLED` and the goal opts in — the
`JB_OP_TOSS_PIPELINE_ENABLED` ships-false precedent; `toss_session` keeps
owning accounting); `trajectory_node.py` (install `CyclePlan` through the
existing `_install` continuity machinery; the deferred-reach and
`arm_catch`/reach-envelope plumbing driven consistently from the plan);
`catch_coordinator_node.py` (the reactive hand-arm block at the
announcement path is **gated OFF** under unified mode — the plan already
contains the catch stroke; its `SetHandTrajCmd` dispatch is the single
legacy hand write on that path); announcements still published with
plan-derived fields (`build_announcement_fields` equivalent) so the tracker,
possession and suppression consumers are unchanged; the `hand_stroke.py`
timing consumers (`stroke_clear_time`, `required_arm_lead_s`) get
plan-derived twins under unified mode. New outcomes minted through
`outcome_detail` (`REJECTED_CYCLE_INFEASIBLE(<validate_cycle code>: …)` etc.).
Session choreography carries the 2026-08-28 owner directive re-homed from the
superseded MP plan's Q-2: a survived MISS must not `go_home` — hold the pose,
wait for the ball environment to settle, then resume.

**Sim gate:** a unified-mode variant of `sim/toss_gate.py` running the
production chain end-to-end (planner → emitter → real `SetpointPump` →
firmware-mirror interpolation) for the single-toss cycle set; acceptance
mirrors the existing gate's `core_clean` bands.

**Dependencies:** Phases 1–2 (software); Phase 3 only for hardware use.

### Phase 5: Hardware ladder — NOT STARTED

Each rung is a separate sitting with the full-suite pre-hardware rule
(`./run_tests.sh --full`) and E-stop discipline; rungs UH-1..UH-7 map onto
§ 5's T-H1..T-H7. Order is deliberately carry-first even under Scope B:
UH-1 streamed hold → UH-2 slow streamed strokes (no ball) → UH-3 **banked
carry with a seated ball** (the by-eye cup watch inherited from MP-M5) →
UH-4 planned catch of a tossed ball → UH-5 planned throw at low tier
(h ≤ 0.5 m) → UH-6 full planned cycles → UH-7 two-pose ring at constant
beat. The aimed rungs inherit the superseded MP plan's § 7 unlock: re-derive
the aim-authority window against `tilt_geometry.MAX_TILT_DEG` (12°) before
UH-7 flies aimed at the constant beat. The legacy mode remains one `hand_source` switch away
at every rung.

**Dependencies:** Phases 3–4; owner present (operator runs actuating
commands).

### Phase 6: Exclusivity + close-out — NOT STARTED

Platform Teensy **FW 4**: retire the 0x6D0 decode, stroke engine
(`Trajectory.h` generators) and the 0x0C9 hand-encoder sniff; **retain** the
0x6E0 cold-start state + FW identity (bump `PLATFORM_FW_VERSION_EXPECTED`
3 → 4, contract `ros_ws/docs/platform_fw_version.md`), the SCL3300
inclinometer path, and time-sync. Flash via **Arduino IDE only** (the pio
image is CAN-MUTE). Host: retire `set_hand_traj_cmd`; `smooth_move_hand`
re-implements as a planned single-channel move through the streamed lane.
`hand_source` default flips to STREAMED; `hand_ops` 0x6D0 forwarding and the
legacy gate retire at the next bridge FW bump. Close-out: a
`ros_ws/docs/unified_cycle_contract.md` normative doc (the invariant set, one
enforcement point per invariant, the tests that pin them), logbook entries,
ILC hand-off note (what re-captures), archive review.

**Trigger (owner, 2026-08-29):** Phase 6 fires when the Phase 5 UH ladder
completes (completion = the final rung, UH-7 two-pose constant beat,
accepted) — no separate parity campaign; the ladder's own acceptance
(including T-H6's outcome parity at the matched low tier) is the evidence
base. The FW 3 Platform image and FW 16 bridge image remain named rollback
artifacts after retirement.

## 5. Testing Plan

### Unit (offline, no hardware)

**T-U1..T-U5 LANDED 2026-09-01** (Phase 1) — `tests/motion/test_cup_cycle.py`
(T-U1, T-U2), `tests/motion/test_cup_realize.py` (T-U3, T-U4),
`tests/motion/test_validate_cycle.py` (T-U5), with
`tests/motion/test_cycle_plan.py` and `tests/sim/test_cycle_gate.py` alongside
them. T-U6..T-U9 remain for Phases 2–3.

- **T-U1** ✅ **LANDED** (`tests/motion/test_cup_cycle.py`) `cup_cycle`
  constraint satisfaction: for a grid of cycle
  parameters (period 0.4–1.2 s, flight 0.6–1.0 s, displaced catch sites),
  assert release `‖a_cup(T) − g‖ < 1e-6`, detach collinearity
  `‖cross(a − g, axis)‖ < 1e-6` over `n_detach` knots, catch position error
  < 0.1 mm, jerk/workspace boxes respected, and the **catch-runway
  constraint** holds (slider headroom below the catch knot ≥ the planned
  deceleration distance for the arrival speed). Pass/fail: all constraints on
  every grid point that the reference solver also solves.
- **T-U2** ✅ **LANDED** (`tests/motion/test_cup_cycle.py`, fixtures
  `tools/probes/data/cup_cycle_qp_refs.npz` via
  `tools/probes/capture_cup_cycle_refs.py`) QP-vs-IPOPT parity: trajectory max
  deviation < 1 mm / 10 mm/s on
  the recorded Phase 0 cycle set (reference trajectories captured as fixtures;
  CasADi is not installed in the test env).
- **T-U3** ✅ **LANDED** (`tests/motion/test_cup_realize.py`) `tilt_schedule`:
  apparent-gravity alignment within saturation,
  12° cap never exceeded, endpoints equal receive/throw tilt exactly, rate
  limit respected; zero-banking mode reproduces `realize_tilted` output
  bit-comparably (ports `tests/sim/test_juggle_tilt.py`'s parity pattern).
- **T-U4** ✅ **LANDED** (`tests/motion/test_cup_realize.py`) `decompose` z-pin:
  with `unified_z_float_enabled` false, every
  emitted pose has `z == JB_OP_DEFAULT_ACTIVE_Z_MM` exactly; with it true, z
  stays within `unified_z_band_mm` and `validate` gates the excursion.
- **T-U5** ✅ **LANDED** (`tests/motion/test_validate_cycle.py`)
  `validate_cycle` hand gates: drive `HAND_STROKE` /
  `HAND_LIMIT_VEL` / `HAND_LIMIT_ACC` each with a minimal violating plan
  (empirical-probe rule: confirm each code deterministically before
  asserting it); NaN cup input ⇒ `UNREACHABLE`; refusal strings satisfy
  `outcome_detail.base_outcome` round-trip.
- **T-U6** Setpoint v6 codec: pack/unpack round-trip 7-wide including `v1`,
  `HAS_HAND` and `HAS_V1` flag semantics, `SETPOINT_SIZE == 208`, version-5
  frames rejected (extends `tests/teensy_link/test_protocol_codec.py`).
- **T-U7** `SetpointPump` 7-channel: hand keys absent ⇒ flag clear and legs
  byte-identical to a v5-era build of the same command (regression fixture);
  per-channel step gates (leg 0.3, hand bound) reject independently; NaN in
  any hand key rejects the frame; a partial v1 key set (one required
  `vel_next` key present without the other) rejects the frame.
- **T-U8** Emitter: `CyclePlan` sampling emits hand keys with the same
  lookahead discipline as legs; plans without a hand track emit none (legacy
  plans byte-identical output — regression fixture against current emitter).
- **T-U9** Firmware natives (`tests/firmware/native/test_leg_interp.cpp` +
  `test_hermite_xref.py`): 7-lane Hermite parity with the Python reference;
  both Mode 1 velocity rules pinned (`HAS_V1` transmitted-exact and the
  forward-difference fallback); hand lane inert when `HAS_HAND` clear;
  Mode 2/3 extrapolation on the hand lane decays like a leg.

### Integration (real processes, no actuators)

- **T-I1** :5557 end-to-end: `trajectory_node` (unified plan installed) →
  `teensy_bridge_node` with a loopback UDP sink; assert 7-channel frames at
  40 Hz, `HAS_HAND` set only during hand-active segments.
- **T-I2** Unified-mode sim gate (`sim/toss_gate.py` variant): production
  chain end-to-end per § 4 Phase 4 under the **kinematic capture model**
  (MuJoCo contact advisory-only, per the 2026-08-29 resolution); acceptance
  `core_clean ≥ 9/10` on the band the legacy gate pins, plus the two-pose
  constant-beat cycle set.
- **T-I3** Interlock choreography (mocked-ROS): under unified mode the
  reactive hand-arm dispatch is provably never called (spy on the
  `SetHandTrajCmd` client); under legacy mode byte-identical behaviour to
  today (mocked-ROS tests are blind to real choreography — the hardware twin
  is T-H4).
- **T-I4** Solver budget on-target: `unified_cycle` plan+validate wall time
  ≤ 50 ms p99 on the Jetson under concurrent suite load (`serial`-marked
  only if it measures wall-clock against a threshold).

### Hardware (bench first, E-stop ready)

- **T-H1** Streamed hand hold: `hand_source=STREAMED`, stream the current
  hand position for 10 min. Pass: zero motion, zero deviation-guard trips,
  `tx_deferred == 0`, `leg-bus-frame-drops` episode rate not elevated vs the
  6-frame baseline.
- **T-H2** Slow streamed stroke (no ball): 0.5 rev/s triangle over 2 rev;
  then Phase-0-probe-2 replay of a 3 m/s stroke. Pass: tracking error
  matches the Hermite-fidelity prediction; hand deviation guard armed and
  quiet.
- **T-H3** Guard trips (deliberate): command a step past the hand step gate
  host-side (must be refused by the pump); inject a deviation via a held
  rotor (operator) — `MAX_DEVIATION_HAND` E-stops, latches, releases only on
  CLEAR_ERRORS with the recovery slew.
- **T-H4** Interlock on hardware: `HAND_TRAJ_CMD` while STREAMED ⇒
  `ERR_HAND_SOURCE` (visible in `hand_traj_acks`); Setpoint hand channel
  while LEGACY ⇒ discarded + counted; source switch refused while
  `mpc_active`.
- **T-H5** Banked carry, seated ball (UH-3): two-pose re-pose with the ball
  in the cup, banking on, lean shaper off; by-eye cup watch + bag. Pass: no
  visible ball disturbance at a re-pose duration ≤ the legacy `GoToPose`
  duration for the same re-pose at `lean_gain = 0` (computable offline from
  `planner.build_move`; the MP plan measured the tilt-only case at the
  0.20 s jerk floor).
- **T-H6** Planned catch, then planned throw at h ≤ 0.5 m (UH-4/5): outcome
  parity with the legacy tier at the same site before any tier ramp; release
  velocity error vs plan < 5 % (bag-measured).
- **T-H7** Endurance: 30 min of unified cycles at a low tier. Pass:
  `latency_monitor` quiet, leak counters 0, no guard trips, drop-episode
  rate at baseline.

### Regression

- **T-R1** Legacy byte-exactness: with `unified_cycle_enabled` false and no
  hand keys, the leg wire bytes are identical to pre-change fixtures (pump
  and emitter fixture tests above are the enforcement).
- **T-R2** Version skew is loud: v5 host vs v6 decode (and inverse) rejects
  every frame; `BRIDGE_FW_CHECK` advisory names 17.
- **T-R3** The full legacy toss battery (`tests/ros/test_toss_*`,
  `tests/sim/test_toss_gate.py`) passes unchanged in legacy mode after every
  phase — the fallback stays flyable until Phase 6.

## 6. Notes for Collaborators

### Safety-critical invariants

| Invariant | Where | Consequence if wrong |
|---|---|---|
| Hand has **no sign flip** and wire scales 100/100 (legs: negate + 1000/10000) | `odrive_protocol.h::encode_leg_setpoint` (already correct for axis 6) | Sign/scale error commands a full-speed hand excursion |
| Hand setpoint clip `[0, 10.8]` rev is the metal | `canbridge_config.h::HAND_MOTOR_MAX_POSITION` (FW 15's entire delta) | Commands past metal |
| Leg guard constants must NOT be applied to the hand (`MAX_DEVIATION_REV 1.0`, `MAX_LEAD_REV 0.10`, `MAX_MOTOR_VEL_RPS 16.5` are leg numbers; hand `vel_limit` is 1000 rev/s) | `fault_machine.cpp`, `canbridge_config.h` | Either dead guard or constant false trips |
| `SEGMENT_T_S == JB_TRAJ_KNOT_DT_S == 0.025` (comment-enforced, both ends) | `canbridge_config.h:148`, `hardware_config.py:175` | Distorted velocity profile on all 7 channels |
| `PROTOCOL_VERSION` mismatch = total link darkness, by design | `udp_protocol.py:12` / `udp_protocol.h:12` | A non-lockstep flash strands the robot (loudly, fail-closed) |
| Planning never runs on the emitter thread (owner determinism rule, 2026-08-10) | `motion/unified_cycle.py` call sites | 40 Hz jitter → lead-clamp engagement → commanded stops |
| `TEENSY_TRAJ_HAND_STROKE_M` (0.355, throw-profile basis) ≠ `GEOM_HAND_STROKE_MM` (344.75, physical) — never substitute one for the other | `hardware_config.yaml:1117-1141` note | Silent geometry error in the slider decomposition |
| The z=170 pin has TWO chains: `JB_OP_DEFAULT_ACTIVE_Z_MM` consumers AND sim-side `Z_ACTIVE_MM` literals (`sim/juggle_tilt.py:58`, `sim/juggle_online.py:92`, `sim/gate_common.py:25`, plus `toss_sequencer.py:615`'s pinned local) | § 2.2 of the exploration; grep before touching | z-float toggle that misses a chain ships a contradiction |
| `toss_workspace_xy_mm` was DELETED 2026-08-29 — the reach-feasibility gate is the sole lateral authority | `logbook/2026-08-29-displacement-caps-removed.md` | Referencing the dead key resurrects a retired policy |
| Platform Teensy flash is Arduino IDE only (pio image is CAN-MUTE) | memory / bench facts | A pio flash silently kills the cold-start + inclinometer paths |

### Architecture decisions (root causes, not authority)

- **Why one widened frame, not an additive hand frame:** the ISR latches one
  staging slot atomically under PRIMASK; two frames per knot would need
  cross-frame latch coherence, creating a torn-knot failure class (hand and
  legs from different knots in one tick) that single-frame widening makes
  structurally impossible. The version bump's cost is one lockstep flash.
- **Why the QP reformulation:** with `detach_axis` a parameter the problem is
  convex with linear constraints; IPOPT's 0.23 s on this Jetson is 9× the
  knot period and CasADi is absent from the ROS interpreter. A dense KKT
  solve is both faster and dependency-free. The ZMQ-bridged CasADi process is
  the recorded escape hatch, not the default, because it adds a process, a
  failure mode, and serialization latency to every cycle.
- **Why `hand_source` is a firmware latch, not a host convention:** the
  2026-08-09/-08-13 arcs showed host-side conventions cannot prevent two
  writers on one CAN id; only the single TX owner (the bridge) can make dual
  mastery structurally impossible.
- **Why carry-first on the hardware ladder even under Scope B:** the carry is
  the lowest-consequence first contact for a streamed hand (ball seated,
  moderate accelerations, `MAX_DEVIATION` armed) and it isolates the new
  transport from the new launch mechanism, so a failure attributes cleanly.
- **Why before ILC:** ILC's `e_model` must call the production chain; learn
  on the plant that will exist, not the one being replaced.
- **Why `v1` rides the v6 frame:** the version bump is paid once; retrofitting
  exact velocities later would cost a second incompatible wire change and
  lockstep flash. Behind `HAS_V1` the fallback (forward difference) stays
  testable and the legacy emitter's semantics are preserved until a producer
  opts in.
- **Why the kinematic capture model is the sim authority:** MuJoCo contact is
  the documented low-fidelity element (`sim/toss_gate.py` avoids it
  deliberately), hardware catches are already smooth, and the Rung-3 P2
  failure was an equilibrium of that contact model — gating on it optimises
  the wrong plant.
- **Why the event-timeline API:** the 3-ball endgame needs overlapping
  per-ball events; taking a list now (while always passing one throw + one
  catch) costs one signature and saves rewriting every caller later.

### Startup/shutdown ordering

Unchanged from today for legacy mode. Unified mode adds: `hand_source`
switch only while disarmed and settled; hand CLOSED_LOOP arming rides the
existing MPC-arming contract; on any fault the hand lane E-stops with the
legs (single guard machine, single CLEAR_ERRORS release).

### Files affected (summary)

| Action | Files |
|---|---|
| Create | `motion/trajectory/cup_cycle.py`, `cup_realize.py`, `cycle_plan.py`, `motion/unified_cycle.py`, `ros_ws/docs/unified_cycle_contract.md`, tests (`tests/motion/test_cup_cycle.py`, `test_cup_realize.py`, `test_cycle_plan.py`, `tests/ros/test_unified_cycle_node.py`, sim-gate variant) |
| Modify | `config/generate_udp_protocol.py` + generated (`udp_protocol.py/.h`), `config/hardware_config.yaml` + generated config artifacts, `teensy_link/{setpoint_pump,synthetic_setpoint,replay_setpoint}.py`, `motion/ipc.py`, `motion/trajectory/{emitter,feasibility}.py`, `trajectory_node.py`, `teensy_bridge_node.py`, `reload_coordinator_node.py`, `catch_coordinator_node.py`, `Teensy_code_canbridge/{leg_interp.*,canbridge_config.h,fault_machine.cpp,hand_ops.cpp,can_buses.cpp,rpc.cpp}`, existing pump/emitter/codec/interp tests |
| Retire (Phase 6) | `Teensy_code_platform` stroke engine + 0x6D0 decode + 0x0C9 sniff (FW 4), `set_hand_traj_cmd` host path, `hand_ops` 0x6D0 forwarding |

### Rollback plan

Phases 0–2 and 4 are software behind ships-false flags — rollback is the
flag. Phase 3 rollback: reflash FW 16 + check out the pre-v6 host (version
darkness makes a half-rollback loud, not silent); `hand_source` boots LEGACY
so a rolled-back host on FW 17 still flies the legacy path. Phase 6 is the
point of no return for the stroke engine — the FW 3 Platform image and the
FW 16 bridge image are retained as named artifacts until the owner declares
the unified path proven across a full session ladder.
