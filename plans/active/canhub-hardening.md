---
title: Can-hub Teensy firmware/UDP pipeline — hardening pass (Fable-5 fresh-eyes review)
created: 2026-07-02
status: active
last_updated: 2026-07-02
related_plan:
  - canbridge-foundation-coldstart-parity.md
related_review:
  # Full 8-area Fable-5 review + adversarial verdicts + synthesis (durable run journal):
  - ~/.claude/projects/-home-jetson-Desktop-Jugglebot/464186d7-ba4a-4f04-b124-0f6646ec869d/subagents/workflows/wf_f446b2a9-aae
related_code:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/
  - controller/teensy_link/
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/firmware/
---

# Can-hub pipeline hardening pass

## Context

After the canbridge-foundation cold-start-parity plan landed (Phase 4 + the reboot
bugfix + hardware validations, 2026-07-02), a fresh **Fable-5 multi-agent review**
(8 area-reviewers → adversarial verification of every HIGH → synthesis; run
`wf_f446b2a9-aae`) assessed the whole can-hub Teensy firmware/UDP pipeline for
robustness / test-coverage / elegance / quality. This plan tracks the resulting
hardening pass. Goal: **truly harden the safety-critical substrate before autonomous
juggling**, then archive the parity plan.

**Verdict (Fable 5):** *"Production-quality core, pre-hardening edges."* Above typical
first-pass firmware (single-enforcement-point fault machine, disciplined PRIMASK/
seqlock concurrency, single-sourced protocol, a native harness compiling REAL firmware
TUs, 472+ green tests; happy paths hardware-validated through Phase 5). But **6
independently-confirmed HIGH findings cluster exactly where the powered sittings could
not reach** — the last line of defense and rare paths. The pass is warranted and
well-bounded: ~half the items are host/test-side and safely auto-appliable; the
firmware items consolidate into 1–2 flash cycles + one powered re-validation sitting.

## Cross-cutting themes

1. **Ported safety semantics diverged from motor_guard/can_node at the parity edges**
   (E-STOP latch, disarm-while-CLOSED_LOOP predicate, link-loss visibility on
   robot_state, clear_disarm_reasons, hand-axis scope) — restore parity or write the
   deliberate divergence into the matrix + header; never leave it silent.
2. **One time-base root cause drives an entire watchdog failure class (fw AND host):**
   `now_wall_us()` is steppable yet every staleness interval runs on it. Contract:
   monotonic clock for ALL interval arithmetic; wall clock ONLY for wire-bound
   timestamps. Fix the class once, not 14 patches.
3. **Single-layer trust at every rare-path boundary** (setpoint thread dies on one bad
   frame; firmware ingests wire floats with no isfinite; seq discarded; STATE_WRITE
   persists unvalidated floats; RPC retry assumes a firmware dedup that doesn't exist;
   SetpointPump.reset() never wired) — add the second layer, host-side first.
4. **Test coverage inversely correlated with consequence at the firmware layer:** the
   code that drives legs into hardstops (leg_homing/activate/deactivate.cpp) + rpc.cpp
   dispatch + the C++ framing are never compiled by any test; goldens miss the
   divergent regions.
5. **Concurrency discipline 95% consistent — the misses are the safety-relevant 5%**
   (Ethernet.loop() outside NetLock; last_heartbeat_us non-atomic; interp_begin_stow
   barrier; ISR priority AT the syscall ceiling; no UDP RX drain budget).
6. **Documentation drift small in volume, concentrated where AI sessions trust it**
   (CAN2→CAN3, "nothing has run on a Teensy yet" README, inverted timeout docstring,
   false "C++ can't be compiled" xref claims, unfulfilled serialization promise).

## Decisions

- **E-STOP latch = `motor_guard` semantics (LOCKED, operator 2026-07-02).** The
  firmware latches `guard_mode==ESTOP` on MPC_STALE / MOTOR_OVERSPEED / MAX_DEVIATION
  and holds output gated off until an explicit operator clear (mirrors
  `motor_guard._trigger_estop`; the natural clear hook is `fault_notify_clear_errors()`).
  Drives item [13] + the guard half of [17]. Record the rationale in the fault_machine
  header when it lands.
- **Hand-axis scope in fault-eval / clear / stow-IDLE (OPEN).** Decide at the Tier-2
  firmware bundle (item [17]): extend to NUM_AXES for JUGGLEBOT_AXES parity, or
  document the deliberate legs-only scope + add matrix rows. (Standing operator note:
  hand gaps default to real regressions, not off-scope.)

## Recommended changes (21, tiered)

### Tier 1 — implement-now (host / test / doc; low-risk, pytest-gated, NO re-flash)

| # | Area | Risk | Item |
|---|---|---|---|
| 1 | host-teensy-link | low | Exception-contain the production setpoint thread (reject-not-raise in build(); wrap _process_setpoint; catch OSError on send) |
| 2 | host-bridge-node | low | **Surface UDP-link loss on robot_state** (OR link_lost into has_fatal_can_error + error string) — dropped can_node parity ⚠ observable behaviour change |
| 3 | tests / fw-motion | low | Compile leg_homing/activate/deactivate.cpp into the native harness; retire the tautological Python transcriptions |
| 4 | host-teensy-link | low | Wire SetpointPump.reset() at confirmed-reconnect + mpc_active re-enable |
| 5 | host-teensy-link / fw-relay | low | Stop auto-retrying non-idempotent RPCs host-side; fix the false idempotence docstring; check pending.event after retry |
| 6 | host-bridge-node | low | Fix the cross-axis disarm-while-CLOSED_LOOP predicate ⚠ observable behaviour change |
| 7 | host-bridge-node | medium | Restore hand vel/curr-limit topic parity (apply + cache + fix stale comments) ⚠ observable behaviour change |
| 8 | tests / fw-config | low | Native/golden expansion: rpc.cpp dispatch+codec, C++ UDP framing, guard E-STOP & stow-mutual-exclusion, dead-bus golden column, BB-throw & ODrive-encoder byte goldens, DeferredStowLatch first_seen |
| 9 | fw-config / fw-infra | low | PROTOCOL_VERSION freeze test + codegen lints + Python MAX_PAYLOAD parity + pin firmware library versions |
| 10 | host-bridge-node | low | Bridge-node robustness cluster: async UDP-reconnect re-read, relay STATE_READ/WRITE serialization lock, bounded cone catch-event queue |
| 11 | host / tests | low | Host bundle: ActivateMonitor requires CLOSED_LOOP, scale_to_bench floor>ceiling raise, capped socket drain + typed CrcError, rpc_args METHOD += DEACTIVATE, stroke literals from config |
| 12 | cross-cutting | low | Documentation truth sweep (CAN2→CAN3 in the generator, README refresh, stale citations, stroke-min comment, harness-scope docs, UV-flag note, DEGRADED marker, dead task-table constants) |

### Tier 2 — surface-first (firmware; re-flash + ONE powered re-validation sitting)

| # | Area | Risk | Item |
|---|---|---|---|
| 13 | fw-fault/motion | high | **Latch guard E-STOP** until explicit clear (motor_guard semantics — DECIDED) |
| 14 | fw-fault/motion/infra (+host) | high | **Monotonic micros64() for ALL interval/staleness arithmetic**; wall clock only for wire timestamps (triple-corroborated) |
| 15 | fw-infra | low | NetLock around the lwIP pump (Ethernet.loop()) + UDP RX drain budget |
| 16 | fw-motion/relay/config | medium | Flash-A: command-gate & trust-boundary bundle (MPC↔cold-start mutual exclusion, HAND_TRAJ_CMD↔homing interlock, homing presence check, STATE_WRITE float validation, setpoint isfinite + seq guard, notify-after-gate ordering, AXIS_ALL send-result accum) |
| 17 | fw-infra/fault | medium | Flash-B: concurrency & parity residues (atomic last_heartbeat_us, interp_begin_stow PRIMASK barrier, ISR priority 32→16, clear_disarm_reasons mirror, **hand-axis scope decision**) |
| 18 | host-teensy-link | medium | Close the HomingMonitor silent-abort false-success class (uplink HomingResult or orchestrator fault_state cross-check); fix the inverted timeout docstring now |
| 19 | tests | low | Commit the hardware-validation procedures: 500 Hz deadline/jitter PASS/ABORT gate + ISR/stow-re-arm soak probe in tools/probes/ |
| 20 | fw-relay/motion/fault | medium | Small firmware follow-ups to fold into the NEXT firmware phase (version-sweep re-query, REBOOT-during-stow interlock, stow-gate fatal/estop abort-or-document, SETTLE_STOP_US from config, HOME via axis-6 policy table, RESULT_BUF_CAP static_assert, encode_frame nullptr guard, cmd-result ring drop policy) |

### Tier 3 — defer

| # | Area | Item |
|---|---|---|
| 21 | elegance (host+fw) | Structural refactors: teensy_bridge_node split, activate/deactivate dedup, observer _MonitorBase, SPSC ring/template consolidation, setpoint-loop poll-rate, FakeTeensy port API — after hardening / when next touching these files |

## Coverage gaps (17 — most closable host-side, folded into items 3/8/9/11/19)

1. Cold-start move firmware (leg_homing/activate/deactivate.cpp) never compiled by any test.
2. Guard E-STOP paths (MPC_STALE, MOTOR_OVERSPEED) + deferred-stow↔cold-start mutual exclusion + budget-preservation-on-down-bus — zero native coverage.
3. rpc.cpp dispatch()/send_axis_frame (the (method,axis) enforcement point) + request codec bounds — never compiled.
4. Hand-written C++ UDP framing (decode/encode_frame/crc16) — the network trust boundary — never executed by a test.
5. Firmware dead-bus no-op-clear asserted only on the Python mirror (no bus_down golden column).
6. Clock-step robustness (divergent wall vs mono across a fault_step) — untested; required to land + protect the monotonic-clock fix.
7. BB throw encoder byte-parity xref promised by the generator does not exist.
8. odrive_protocol.h C++ encoders never byte-compared cross-language.
9. Setpoint ingress firmware-side (truncated payload silent-drop, stale-seq accept, post-clear stale-disarm tick).
10. DeferredStowLatch mirror first_seen diverges from firmware; cold-start divergent region has no golden.
11. RPC reliability paths (retry-then-success, stray/late/dup responses, res_len truncation, stale-response-for-old-req_id).
12. Bridge-node loopback gaps (mid-run heartbeat death → fault flags; malformed setpoint → thread survives; disarm predicate; hand-limit topic; reentrant relay-read/STATE_WRITE).
13. 500 Hz deadline/jitter counters instrumented but read by no procedure.
14. ISR/FreeRTOS concurrency has no committed hardware soak recipe.
15. time_base step/slew/stale-gate + the three SPSC rings + health_of() — zero host coverage.
16. PROTOCOL_VERSION layout freeze absent (one violation shipped in 0935c63); codegen lacks enum-uniqueness/msg_type-membership/payload-budget asserts.
17. Test-input literals (stroke bounds) hand-copied, can go stale against canbridge_config.h.

## Execution sequence

1. **Tier 1 now** — host/test/doc, pytest-gated, in sub-batches (host-robustness →
   bridge-node → tests → docs), each its own commit + logbook. Surface [2]/[6]/[7]
   (observable robot-behaviour changes) for operator eyeball before landing.
2. **Tier 2** — consolidate the firmware items into 1–2 flash cycles (Flash-A gates/
   trust-boundaries, Flash-B concurrency/parity), land the E-STOP latch [13] +
   monotonic clock [14] + NetLock [15], then ONE powered re-validation sitting
   (cold-start + deferred-stow + a clock-step/boot-order check). Hand-axis scope
   decided here.
3. **Tier 3** — defer structural refactors.
4. **Then** `/archive-plan canbridge-foundation-coldstart-parity` (its residuals are
   discharged; this hardening plan carries the forward work) and move to autonomous
   movement.

## Testing plan

- Every Tier-1 sub-batch: `pytest tests/ -q` green (cite the date/command/result
  triple), the order-flaky allocation tests confirmed isolated.
- Tier 2: `pio run` green + the native harness (now compiling the cold-start TUs) +
  the golden conformance + the new clock-step test, then the powered re-validation.
- New coverage lands with the item that motivates it (gaps → items 3/8/9/11/19).
