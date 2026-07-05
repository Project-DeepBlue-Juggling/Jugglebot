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

#### Tier-1 progress (2026-07-02 → 2026-07-05)

Logbook: `logbook/2026-07-02-canhub-hardening-tier1.md`. The original host/test/doc
batches landed in five commits (each `pytest tests/ -q` green — triples in the
logbook Verification section); the 2026-07-05 close-out then added item-8 completion
(`6f20fea`, `db4ddf8`), the typed CrcError (`2cf3a07`), and the doc-truth sweep.
*(The `#`/Row numbers below are this plan's 1–21 item numbers; `gap N` are the
coverage-gap numbers. The commit messages additionally carry the Fable-5
review-finding numbers, which are a SEPARATE scheme and do NOT match these — e.g.
review-finding `[14]` = the native/golden work here = plan item 8, whereas plan
item 14 is the pending Tier-2 monotonic clock.)*

| Row | Item | Status | Commit |
|-----|------|--------|--------|
| 1 | setpoint-thread containment | ✅ done | `f8397c7` (A) |
| 2 | link-loss → `has_fatal_can_error` (observable) | ✅ done | `c7425e9` (B) |
| 3 | compile cold-start `.cpp` + retire tautological xrefs | ✅ done | `431ea89` |
| 4 | `SetpointPump.reset()` wiring | ✅ done | `f8397c7` (A) |
| 5 | non-idempotent RPC no-retry + docstring + retry-race | ✅ done | `f8397c7` (A) |
| 6 | cross-axis disarm predicate (observable) | ✅ done | `c7425e9` (B) |
| 7 | hand vel/curr-limit topic parity (observable) | ✅ done | `c7425e9` (B) |
| 8 | native/golden expansion | ✅ done — C++ framing (gap 4) `8821451`; rpc.cpp dispatch (gap 3) + odrive (gap 8) + BB-throw (gap 7) byte xrefs `6f20fea`; dead-bus golden (gap 5) + DeferredStowLatch `first_seen` cold-start golden (gap 10) `db4ddf8`. Guard-E-STOP (gap 2) + clock-step (gap 6) landed WITH Tier-2 (they pin the item-13 latch / item-14 monotonic clock, per the Tier-2 logbook). | `8821451` `6f20fea` `db4ddf8` |
| 9 | PROTOCOL_VERSION + layout freeze, lints, lib pins | ✅ done | `37a81d4` |
| 10 | bridge-node robustness cluster (async re-read, relay lock, cone bound) | ✅ done | `c7425e9` (B) |
| 11 | host bundle (ActivateMonitor CL, scale floor>ceiling, capped drain, rpc_args DEACTIVATE, stroke literals) | ✅ done — typed `CrcError` at the `decode_frame` codegen source now closes the last residual | `f8397c7` (A) + `2cf3a07` (CrcError) |
| 12 | documentation truth sweep | ◐ host/doc side done — CAN2→CAN3 in the generator `1634634`; firmware `README.md` (flashed+validated status, archived-HANDOFF link, pinned-lib table, `pio` build) + native-harness `README.md` (compiled-TU list, archived-plan future-tense, 500 Hz validated) refreshed. **Deferred to Phase 2** (firmware source, CAN3-owned): dead task-table constants + UV-flag note in `canbridge_config.h`/`fault_machine.*`, and the `LinkState::DEGRADED` codegen marker (a regen would touch the delivered firmware header). `synthetic_setpoint.py` stroke-floor comment verified accurate (matches config). | — |

**Observable changes** — plan items 2 / 6 / 7 — have hardware-confirmation checks in
the logbook (read-only operator eyeball). **Tier-1 host/test/doc scope is now
closed** (logbook `2026-07-02-canhub-hardening-tier1.md` → resolved): item 8 (native/
golden) and item 11 (CrcError) landed, and item 12's host/doc sweep is done. The
only item-12 residue is firmware-resident doc comments, folded into the Phase-2
firmware cycle (item 20 territory) — see the item-12 row.

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

#### Tier-2 progress (2026-07-04) — FLASHED + HARDWARE-VALIDATED

Logbook: `logbook/2026-07-02-canhub-hardening-tier2.md`. Six items landed as compiled,
native-tested firmware; an adversarial review over the whole diff caught + fixed two
real regressions; `pio run -e teensy41` green; the can-hub Teensy was **flashed**.

| # | Item | Status | Commit |
|---|------|--------|--------|
| 13 | guard E-STOP latch (motor_guard semantics) | ✅ flashed | `0ad3d25` |
| 14 | monotonic clock for all interval/staleness arithmetic | ✅ flashed | `138aa11` |
| 15 | NetLock lwIP pump + UDP RX drain budget | ✅ flashed | `b562825` |
| 16 | Flash-A command-gate & trust-boundary bundle (7) | ✅ flashed | `c8ba247` |
| 17 | Flash-B concurrency & parity residues (5) | ✅ flashed | `83ac938` |
| 18 | HomingMonitor: **[18B]** inverted-docstring fix DONE (`6fe1ec9`); **[18A]** false-success uplink DEFERRED (wire-protocol change; own cycle after this flash) | ◐ partial | — |
| — | adversarial-review fixes (seq-guard restart brick + stow gravity-drop + residues) | ✅ flashed | `192e6af` |

**Hand-axis fault-eval → NUM_AXES ([17] §5) is an OBSERVABLE change** (a hand fault now
E-STOPs the legs).

**Powered validation PASSED 2026-07-04** — all 7 checks pass (CHECK 2 seq-guard
operator-confirmed PASS). Results table + tooling in the logbook. **Item 19 DONE** —
the interactive validation-checklist runners are committed
(`tools/probes/canhub_tier2_hw_validation.py` + the MPC-free zero-motion
`tests/hardware/teensy_guard_validation.py`), and the automated 500 Hz deadline/jitter
PASS/ABORT soak gate now lands as `tools/probes/canhub_500hz_deadline_gate.py`
(read-only PROFILE-frame ingestion — asserts the deadline-miss delta == 0 + worst-window
jitter ≤ 500 us over a soak; `--self-test`-verified decision logic; the ISR/stow-re-arm
soak is the same gate over a long `--duration` while the operator induces a
CAN-loss→reconnect deferred stow). Carried out one open
item — a **marginal
CAN3 bus** (`err` climbing, `tec`→255, sticky `flt=BUSOFF`, decorative to the staleness
gate) → own bench-diagnosis cycle. **Remaining:** item 20 (small fw follow-ups) + [18A]
(HomingResult uplink) + the CAN3 investigation + `health_of` bus-off wiring, then
`/archive-plan canbridge-foundation-coldstart-parity`.

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
13. 500 Hz deadline/jitter counters instrumented but read by no procedure. ✅ closed by item 19 (`tools/probes/canhub_500hz_deadline_gate.py`).
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
