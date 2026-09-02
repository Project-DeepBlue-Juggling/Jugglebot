---
title: "Unified 7-DoF planner Phase 2 — the wire goes seven wide: v6 Setpoint with exact knot velocities, a reject-don't-guess hand pump, and a host that can no longer talk to old firmware (by design)"
type: investigation
date: 2026-09-02
status: resolved
phase: "unified-7dof-planner — Phase 2"
related_plan: unified-7dof-planner.md
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/jugglebot/motion/ipc.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/emitter.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - teensy_link/replay_setpoint.py
  - teensy_link/setpoint_pump.py
  - teensy_link/synthetic_setpoint.py
  - tests/firmware/test_hermite_xref.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/hardware/bench_leg_sysid.py
  - tests/hardware/kt_bench_test.py
  - tests/hardware/teensy_guard_validation.py
  - tests/motion/data/v5_emitter_frame_fixtures.json
  - tests/motion/test_leg_torque_ff.py
  - tests/motion/test_trajectory_emitter.py
  - tests/ros/test_teensy_bridge_node_setpoint.py
  - tests/ros/test_trajectory_node.py
  - tests/teensy_link/data/v5_pump_wire_fixtures.json
  - tests/teensy_link/test_protocol_codec.py
  - tests/teensy_link/test_replay_setpoint.py
  - tests/teensy_link/test_setpoint_pump.py
  - tests/teensy_link/test_synthetic_setpoint.py
  - tests/teensy_link/test_v5_wire_regression.py
  - tools/probes/README.md
  - tools/probes/capture_v5_wire_fixtures.py
  - tools/probes/teensy_link_profiling/jetson/setpoint_stub.py
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - tools/probes/traj_stream_probe.py
  - plans/active/unified-7dof-planner.md
  - plans/active/INDEX.md
  - logbook/2026-09-02-unified-7dof-planner-phase2-wire-v6.md
  - logbook/INDEX.md
subsystem:
  - can
  - motion
  - ros
  - config
  - tools
tags:
  - IPC
  - safety
  - testing
  - docs
---

# Unified 7-DoF planner Phase 2 — wire v6 + host 7-channel path

## Summary

Phase 2 of [`plans/active/unified-7dof-planner.md`](../plans/active/unified-7dof-planner.md):
the v6 Setpoint wire and the 7-channel host path, firmware-absent safe, and
**COMMITTED BUT NOT DEPLOYED** — after this commit the host cannot talk to any
FW ≤ 16 board (`PROTOCOL_VERSION` 5 → 6; the total link darkness is **loud and
fail-closed by design**). Deployment waits for Phase 3's lockstep flash sitting,
and that firmware build must be **CLEAN, not incremental** (the `extra_script.py`
hazard from the 4 → 5 bump applies).

**The wire:** all six f32 arrays widen 6 → 7 — index 6 is the hand, in **raw
ODrive rev with no sign flip** (CAN scaling stays firmware-side); a new v1
f32[7] exact-knot-velocity array after `torque_ff`; `SETPOINT_SIZE` 156 → 208;
flags `HAS_HAND` 0x4 and `HAS_V1` 0x8. ZMQ leg arrays stay 6-wide; six new
optional `mpc_cmd` keys carry the hand and the exact velocities.

## Process

**Fixture-first, enforced structurally.** A dedicated agent captured pre-change
fixtures at HEAD `2aaaae1` *before any edit*: 6 cases / 13 pump v5 frames plus
8 emitter `ipc._pack` msgpack blobs, double-run byte-identical, committed under
`tests/teensy_link/data/` + `tests/motion/data/` — a **deliberate exception** to
`tools/probes/README.md`'s data-cut location convention, recorded there. The
capture script `tools/probes/capture_v5_wire_fixtures.py` **refuses to run
post-v6** (`PROTOCOL_VERSION == 5` guard + a frozen `mm_to_rev` drift assert) —
a historical recipe by design, not a regeneration path.

Then implementation, then a 4-lens adversarial review: 14 findings → 7 distinct
→ 6 fixed, 1 carried to Phase 3. Per-finding verification refuted one MAJOR
framing — the "unpinned 0.80 relation" was already behaviourally pinned at
`tests/motion/test_validate_cycle.py::test_hand_step_bound_refuses_without_tripping_the_velocity_cap`;
the finding was retained only as dead-assert cleanup. **Owner directive taken
mid-phase, applying to future phases: 2 review lenses, no per-finding verify
stage, one audit per phase.**

## Discussion

### One derivation chain for the hand step gate

The pump gate is `hand_vel_limit_rps × knot_dt` = **5.0 rev** (shipped
200 × 0.025); `validate_cycle`'s bound is 0.80 × the same product = **4.0 rev**
(`feasibility.py:1231`) — exactly the legs' 20 % validate-below-pump margin
(0.24 vs 0.3). **Rejected alternative:** pump gate = 4.0, equal to the validate
bound — it zeroes the documented *"a pump reject can't happen on a validated
plan"* margin invariant. (Cites Phase 0 Decision 4.) Post-review,
`teensy_bridge_node` passes the config-chain value at **both** pump construction
sites — the review's one real MAJOR: the live gate silently rode the module
default, behaviourally invisible precisely because default == chain value, which
is why a direct ctor-pin test was added rather than trusting the coincidence.

### Reject-over-fallback for the hand

The hand key set is all-or-nothing; NaN in any hand key **rejects the frame
loudly** — a deliberate contrast with the legs' u1/u2 flag-clear fallback. The
hand requires the full Mode-1 knot set (`HAS_U1`+`HAS_U2`); `HAS_V1` requires
that set AND every active channel's v1 (a partial set rejects); hand keys
without a position command reject (pre-review this was silently skipped). Root
cause, not policy: on a 221 rev/s axis an undefined wire state or a silent drop
is a **step-command hazard**, and the pump is the single host-side enforcement
point. Degrading gracefully is the legs' luxury.

### Hand-gap re-baseline

An accepted hand-absent frame **clears the step-gate baseline** — the first hand
frame after a gap is a first frame, accepted. Gating across the gap would
compare against a position the plan legitimately left; the firmware
`MAX_DEVIATION_HAND` guard (Phase 3) is the complementary layer. **CARRIED TO
PHASE 3** (review finding 7): the bench ladder needs an explicit gap-re-entry
case *before* the observe-first deviation guard arms, and the
`HAS_HAND`-falling-edge-while-frames-continue firmware behaviour (decay, never
hold-at-last-command from up to 200 rev/s) must be stated normatively in
Phase 3.

### Emitter: one extra matrix-vector product, and delete-not-None

The emitter detects `plan.hand_at` (the `CyclePlan` contract) and samples
τ / τ+dt / τ+2dt. `vel_next_mm_s` comes from **one extra matrix-vector product
on the J1 that `_ik(pose1)` already computed** and previously discarded — no
extra IK solve. `make_mpc_command`'s new kwargs are absent-when-None and
**DELETE stale keys under `out=` reuse** — deliberately different from the
legacy write-None semantics, because a stale hand value surviving into a legacy
frame would silently arm `HAS_HAND`; reused-dict-equals-fresh-dict is pinned
byte-exactly.

### Smaller calls, so they read as decisions

- `/leg_setpoint_echo` deliberately stays 6-wide (GUI topic contract; a hand
  echo is Phase 3/4 surface). `SetTrajectoryLimits.srv` still not widened —
  Phase 1's decision carried: session hand limits are YAML-set until the
  interfaces rebuild.
- `synthetic_setpoint` `DEFAULT_MAX_STEP_REV` 0.15 → 0.10, matching the
  verified firmware `MAX_LEAD_REV = 0.10f` (`canbridge_config.h:236`); the pin
  test parses the header text. `replay_setpoint`'s 0.15 is left as
  documented-historical.
- Three stale duplicated `PROTOCOL_VERSION == 5` pins removed from
  `test_protocol_codec.py` per that file's own one-constant-one-pin rule; the
  xlang freeze pins updated 5 → 6 via the test's intentional-change mechanism.
- **One atomic commit**, not codegen-first: the widened generated codec breaks
  6-wide pump packing, so a codegen-only commit is red by construction.

## Verification

- Baseline: (2026-09-01, `./run_tests.sh`, **6313 passed / 4 skipped, 263.64 s
  parallel + empty serial, total 276 s — PASS**) at HEAD `2aaaae1`.
- Codegen: (2026-09-01, `python config/generate_udp_protocol.py`, **5 artifacts
  regenerated; Setpoint 208 B payload / 218 B frame**).
- Implementation tiers (2026-09-01):
  (`python -m pytest tests/teensy_link/ -q`, **291 passed in 7.33 s**);
  (`python -m pytest tests/firmware/ -q`, **406 passed in 207.06 s**);
  (`python -m pytest tests/motion/ -q`, **2128 passed / 3 skipped in 327.39 s**);
  (`python -m pytest tests/ros/ -q`, **2626 passed / 1 skipped in 343.71 s** —
  the MPC-chain removal `c07310c` took `tests/ros/` to 2622/1 skipped (its
  entry's triple); this phase's new node tests bring it to 2626).
- Review-fix wave (2026-09-02):
  (`python -m pytest tests/teensy_link/ -q`, **296 passed in 7.57 s**);
  (`python -m pytest tests/motion/test_validate_cycle.py
  tests/motion/test_trajectory_emitter.py -q`, **40 passed in 2.44 s**);
  (`python -m pytest tests/ros/test_teensy_bridge_node_setpoint.py
  tests/ros/test_trajectory_node.py -q`, **181 passed in 15.52 s**);
  (`python -m pytest tests/firmware/test_udp_protocol_xlang.py
  tests/firmware/test_config_drift.py -q`, **65 passed in 2.35 s**).
- Post-docs gate: (2026-09-02, `./run_tests.sh`, **6382 passed / 4 skipped in
  255.47 s parallel + empty serial phase, total 267 s — PASS**).
- Pre-commit gate, after the audit fixes (the `/audit --unstaged` pass: 1
  WARNING + 4 NOTES, all doc/one-line, applied): (2026-09-02, `./run_tests.sh`,
  **6382 passed / 4 skipped in 256.74 s parallel + empty serial phase — PASS**).
  `--full` deliberately NOT run: the phase brief's owner directive scopes
  `--full` to changes under `controller/` or `sim/`, and this diff touches
  neither.

## Outcome

**Phase 2 is COMPLETE — committed, not deployed.** The v6 codec, the 7-channel
pump, the emitter hand path and the version-skew loud-reject are all landed and
pinned; the leg wire bytes are proven identical to the pre-change fixtures with
no hand keys present. The robot stays on v5/FW ≤ 16 until Phase 3's lockstep
flash sitting (clean firmware build; `EXPECTED_BRIDGE_FW_VERSION` is still 16 —
the `BRIDGE_FW_CHECK names 17` half of T-R2 is Phase 3's).

Carried to Phase 3: review finding 7 (the gap-re-entry bench case and the
normative `HAS_HAND` falling-edge decay statement) and the hand deviation/lead
firmware guards. Carried to Phase 4: the interfaces rebuild that would widen
`SetTrajectoryLimits.srv` — which then obligates re-deriving the bridge pump's
`max_step_hand_rev` from the live session limit (or re-constructing the pump on
`set_limits`), so the three-layer 5.0/4.0 chain (pump gate, node backstop,
`validate_cycle`) moves together instead of splitting static-vs-live.
