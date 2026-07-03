---
title: Can-hub hardening Tier-2 — firmware safety/concurrency/parity pass + flash (Fable-5 review)
type: feature
date: 2026-07-02
status: awaiting-hardware-validation
phase: "Tier-2"
related_plan: canhub-hardening.md
related_entries:
  - 2026-07-02-canhub-hardening-tier1
  - 2026-07-02-canbridge-phase4-orchestrator-wiring
  - 2026-05-19-can-loss-fault-response-safety-inversion
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_link.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hand_ops.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platform_relay.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_homing.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_deactivate.cpp
  - controller/teensy_link/client.py
  - controller/teensy_link/homing.py
commits:
  - 0ad3d25
  - 138aa11
  - b562825
  - c8ba247
  - 83ac938
  - 6fe1ec9
  - 192e6af
subsystem:
  - firmware
  - can
  - controller
  - testing
tags:
  - hardening
  - firmware
  - e-stop
  - concurrency
  - monotonic-clock
  - fable-5
  - flash
---

## Summary

Tier-2 of the can-hub hardening (plan `canhub-hardening.md`) — the **firmware** half of
the Fable-5 review: the safety/concurrency/parity edges the powered sittings could not
reach. Six items landed as compiled, native-tested firmware, then an **adversarial
review** over the whole diff (which caught two real regressions), a `pio` compile-check,
and a **flash** of the can-hub Teensy (2026-07-03). Awaiting one powered re-validation
sitting.

| Item | What | Commit |
|------|------|--------|
| **[13]** | Guard E-STOP **latches** until an operator clear (motor_guard semantics) | `0ad3d25` |
| **[14]** | **Monotonic `micros64()`** for ALL interval/staleness arithmetic; wall clock for wire timestamps only | `138aa11` |
| **[15]** | **NetLock** around the lwIP pump + a bounded UDP RX drain | `b562825` |
| **[16]** | **Flash-A** command-gate/trust-boundary bundle (7 sub-items) | `c8ba247` |
| **[17]** | **Flash-B** concurrency/parity residues (5 sub-items) | `83ac938` |
| **[18B]** | Inverted HomingMonitor timeout docstring (host doc) | `6fe1ec9` |
| — | Adversarial-review fixes (2 regressions + residues) | `192e6af` |

**[18A]** (uplink `HomingResult` in a Diagnostic pad byte to close the silent-abort
false-success class) is **deferred**: it is a wire-protocol change that ripples into the
Tier-1 [15] layout-freeze test + codegen and is a detection improvement, not a safety
gate — better validated as its own cycle after this flash.

## Context

Tier-1 (host/test/doc, `logbook/2026-07-02-canhub-hardening-tier1.md`) closed the
host-side edges. Tier-2 is the firmware behaviour changes the Fable-5 review flagged as
HIGH/medium, clustering at the last line of defence: the E-STOP latch, the one time-base
root cause (steppable `now_wall_us()` feeding every staleness interval), the lwIP
concurrency hazard, the command-gate trust boundaries, and the concurrency/parity
residues. Each requires a flash, so they were consolidated into one flash cycle with a
single powered re-validation to follow.

## Changes

*(Per-item detail is in each commit body; the highlights + the non-obvious decisions
are here. Bracket numbers `[N]` are plan item numbers.)*

- **[13] E-STOP latch.** `evaluate_guard()` recomputed a stack-local `estop` every 10 Hz
  tick with no persistence, so the instant a transient cleared (vel dropped, a fresh
  setpoint arrived, deviation shrank) guard_mode reverted to ENABLED and the 500 Hz ISR
  silently resumed streaming — an unacknowledged auto-recovery from E-STOP. Added a
  sticky `s_estop_latched` (+ the reason frozen at first trip) consumed at the three
  downstream sites, released ONLY by `fault_notify_clear_errors()` — deliberately NOT in
  `clear_error_flags()` (the internal soft-reset/UV auto-retry also calls that, and an
  ODrive bounce must not silently clear a guard E-STOP). `fb_stale` stays out of the
  latch (recoverable).
- **[14] Monotonic clock.** One root cause: `now_wall_us() = micros64() + offset`, and
  `set_wall_anchor()` STEPS the offset — so every interval `(now_wall_after − stamp_before)`
  jumps by the step. A forward step jerks the 500 Hz trajectory phase; a backward step
  underflows uint64 → permanent spurious staleness (spurious E-STOP/stow) or masks a real
  MPC_STALE. Converted every interval stamp+read PAIR to `micros64()` across ~8 firmware
  files + host `client.py`, split the 3 dual-use bus-RX sites (health stamp→mono, ring
  `t_bridge_us`→wall), and left wire timestamps on wall. Native clock-step test proves a
  ±5 s wall step leaves fault/guard/stow/trajectory bit-identical while a mono advance
  still trips staleness.
- **[15] NetLock + drain budget.** `Ethernet.loop()` (the NO_SYS=1 lwIP pump) ran OUTSIDE
  any NetLock while prio-3 TX tasks held it → pbuf/PCB corruption. Wrapped the pump in its
  own tight NetLock scope; bounded `drain_socket` to 8 frames/wake (drop-deferred, not
  dropped) so a UDP flood can't starve the prio-3 fault machine. Real native test with a
  fake QNEthernet/recursive-mutex shim asserts lock-coverage (regression-witness confirmed).
- **[16] Flash-A** (7 sub-items): ISR-level MPC↔cold-start mutual exclusion (zero-latency);
  hand-traj↔homing interlock; homing presence check; STATE_WRITE isfinite; setpoint
  isfinite + **wrap-safe seq guard**; notify-after-gate (a down-bus clear no longer refills
  the soft-reset budget); AXIS_ALL send-result accumulation.
- **[17] Flash-B** (5 sub-items): atomic `last_heartbeat_us`; `interp_begin_stow` PRIMASK
  barrier; **ISR priority 32→16** (audited FreeRTOS-free); `clear_disarm_reasons` mirror;
  hand-axis fault-eval → NUM_AXES (an OBSERVABLE change — a hand fault now E-STOPs the legs).

## Discussion

*(Non-negotiable here: a review withdrew nothing of the design but caught two real
regressions I shipped, and several items involved non-obvious tradeoffs a future reader
won't infer from the code.)*

### The adversarial review earned its keep — two regressions caught before the flash

Before flashing safety-critical firmware I ran a 5-dimension adversarial review (find →
adversarially refute each finding → synthesize; 12 agents, 0 findings refuted). It
surfaced two real regressions I had introduced in Flash-A, neither caught by the native
harness because both are cross-session/cross-component:

1. **HIGH — the seq-guard bricked MPC control after ~half of restarts.** The Flash-A
   setpoint seq guard `(int16_t)(seq − s_last_sp_seq) <= 0 → drop` treated the wire seq as
   an in-session monotonic counter. But `s_last_sp_seq` persists for the whole Teensy
   uptime (`interp_reset()` has NO runtime caller, and the Jetson-5V-powered can-bridge
   OUTLIVES `run_mpc` — memory `project_canbridge_power_topology`), while the host resets
   its SHARED `_tx_seq_stream` to 0 on every `TeensyLinkClient` construction. So after a
   restart with a persisted high-water in [1,32767] (~50% of shutdowns) every setpoint was
   dropped as "stale" for up to ~13 min — and because dropped frames don't advance
   `s_last_setpoint_us`, the MPC_STALE guard E-STOP (the new [13] latch!) fired and
   re-latched through every CLEAR_ERRORS. The two hardening items composed into a
   self-healing-only-by-power-cycle brick. Fix: re-baseline before the seq guard — if the
   last accepted setpoint is already older than `MPC_CMD_STALENESS_US`, the prior stream is
   dead (restart or a >250 ms gap), so forget the high-water. A live 40 Hz stream (~25 ms
   gaps) never crosses the bound, so it never false-resets within a session. **Lesson: a
   per-item native test can't catch a bug that lives in the interaction between a firmware
   static's lifetime and a host counter's lifetime — the adversarial review's
   cross-component lens is exactly what found it.**
2. **MEDIUM — an unarmed gravity drop.** Flash-A's `!coldstart` gate was mis-applied to
   the deferred-STOW descent TX (not just the MPC ladder) while `s_stow_complete` stayed
   unconditional — so a DEACTIVATE (operator safing reflex) landing in the ~2-3 s auto-stow
   window suppressed the descent frames but still let the watchdog IDLE all six raised legs
   on a virtual completion → disarm-from-raised → gravity drop. Fix, made symmetric: drop
   `!coldstart` from the stow TX (the stow is a self-contained safety descent, not the MPC
   stream) AND reject a cold-start request while `fault_stow_pending()` (the reverse of the
   existing stow-begin-vs-move guard). **Lesson: `climb one level` — the item-16 map said
   "add `!coldstart` to BOTH TX guards"; the stow TX was the wrong place, and the fix was to
   recognise the mutual-exclusion CONTRACT (a motion source and the stow can never
   co-drive) and enforce it symmetrically at both request and TX, not patch one site.**

The residues (a bare `bb_state.last_heartbeat_us` the item-17 sweep missed → atomic; two
comment inaccuracies about volatility and can_node parity) were fixed too. Three NIT
verdicts (s_phase non-volatile) were left as-is: the verifier confirmed they are safe
under the shipping `-O2`/no-LTO build (non-inline cross-TU calls are the reload barrier).

### Why the E-STOP latch clears on one step, not motor_guard's two

`motor_guard` needs an explicit disable THEN enable to clear. The can-bridge re-enables
automatically once `fault_notify_clear_errors()` fires AND `s_mpc_active`/link/no-fatal-can
still hold — a deliberate one-step difference, because `mpc_active` (the Jetson heartbeat
bit) IS the Jetson's "guard ENABLED" signal, so the operator's re-arm is implicit in the
stream resuming. Recorded in the fault_machine header so it reads as intentional.

### The hand-axis fault-eval extension is the one observable robot-behaviour change here

Extending `evaluate_errors` to NUM_AXES (can_node parity) means a hand active-error or
hand disarm-while-CLOSED_LOOP now escalates to `s_fatal_error` → guard ESTOP → legs safed.
The watchdog/stow stay legs-only (the hand is never stowed). This is intended whole-robot
fatal semantics, but it is a new trigger an operator must not be surprised by — flagged in
the hardware checklist below.

## Verification

*(All per-item counts below are `pytest tests/ -q`, run 2026-07-03, on the pinned
Jetson stack.)*

- **[13]** `pytest tests/ -q` 2026-07-03: 2036 passed + known-flaky alloc test (passes
  isolated), 1 xfailed. Fault golden UNAFFECTED (evaluate_guard is not emitted).
- **[14]** 2037 passed, 1 xfailed. Grep-audited: every remaining `now_wall_us()` is
  wire-bound; no unpaired conversion. Golden unchanged.
- **[15]** 2038 passed, 1 xfailed. New `test_udp_link` (+50 assertions, lock-coverage
  regression-witnessed).
- **[16]** 2038 passed, 1 xfailed. +native tests in leg_interp/hand_ops/platform_relay/
  leg_homing/activate/deactivate/rpc_dispatch.
- **[17]** 2041 passed, 1 xfailed. Golden byte-identical (no regen). Concurrency lint for
  the items the single-threaded harness can't prove.
- **Review fixes** (`192e6af`) 2040 passed + the known-flaky alloc test (passes isolated),
  1 xfailed. +seq-guard gap-reset + stow-pending reject tests.
- **`pio run -e teensy41`**: SUCCESS (222528 text / 35520 data / 106880 bss — fits the
  Teensy 4.1; only the pre-existing benign `-fno-rtti`-on-C warnings).
- **Flash** (`pio run -t upload`, 2026-07-03): SUCCESS — soft-reboot, 258 KB programmed,
  Teensy re-enumerated + `/dev/ttyACM0` live.
- Order-flaky `test_t3b_h4_on_post_solve_allocates_within_budget` / `test_hot_loop_allocation_contract`
  confirmed isolated (memory `project_hot_loop_alloc_test_flaky`), not regressions.

## Hardware validation checklist (operator runs; Claude verifies read-only)

The native harness proves DECISION LOGIC only — the concurrency/timing/500 Hz behaviours
are on-hardware-replay gaps. Run these in one powered sitting. **Observable behaviour
changes to expect (not faults): (a) the guard E-STOP now LATCHES until CLEAR_ERRORS; (b) a
HAND fault now E-STOPs the legs; (c) link-loss reads fatal on robot_state; (d) cross-axis
disarm reads fatal.**

1. **E-STOP latch [13]** — arm (mpc_active=1), induce a guard E-STOP (e.g. an MPC-stream
   gap → MPC_STALE, or briefly exceed the overspeed limit). Confirm guard_mode stays ESTOP
   + output gated **after the condition clears** (pre-fix it auto-recovered). Then
   CLEAR_ERRORS → clean re-enable. Confirm clearing THROUGH a live condition re-latches.
2. **Monotonic clock / clock-step [14]** — the crux is a wall-clock step must NOT perturb
   control. Boot the Teensy with `teensy_bridge_node` DOWN, then bring it up (forces a wall
   anchor step) while `run_mpc` streams — confirm NO spurious MPC_STALE/LINK_LOST/stow and
   NO trajectory jump on the legs. Also: **restart `run_mpc` and confirm the MPC stream is
   accepted immediately** (the review-fixed seq-guard gap-reset — pre-fix this bricked
   control for minutes after ~half of restarts).
3. **NetLock / RX flood [15]** — from the Jetson, UDP-flood the STREAM/RPC ports while
   `run_mpc` streams 40 Hz. Confirm: no hardfault (scheduler LED stays alive), `[diag]
   drain_cap` climbs under flood and reads 0 at nominal, interp `deadline_misses`/`max_jitter`
   stay 0, heartbeat RTT bounded.
4. **MPC↔cold-start mutual exclusion [16]** — with mpc_active=1, issue a HOME/ACTIVATE/
   DEACTIVATE → expect ERR_REJECTED; and during a cold-start move confirm the 40 Hz stream
   does not co-drive (no leg fight/jerk). Issue a DEACTIVATE during an in-progress deferred
   stow → expect ERR_REJECTED (review fix; no gravity drop).
5. **ISR priority / stow barrier [17 items 2-3]** — over a CAN-loss→reconnect deferred stow,
   confirm a smooth profiled descent (no position jump from a torn stow base) and interp
   jitter unchanged/improved vs before.
6. **Hand-axis fault-eval [17 item 5]** — induce a hand ODrive fault (active-error or
   disarm-while-CLOSED_LOOP) and confirm it now E-STOPs the legs (guard ESTOP, output
   gated). Confirm a stale hand HEARTBEAT alone does NOT arm the leg watchdog/stow.
7. **Cold-start regression sweep** — a normal BOOT→HOMING→IDLE + a levelling cycle +
   activate/deactivate, to confirm no Tier-2 change regressed the Phase-4 cold-start parity.

## Deferred / follow-up

- **[18A]** HomingMonitor false-success uplink (firmware+protocol+host) — a wire-format
  change; its own cycle after this flash.
- **[15] contract-grade follow-up** — a shared `net_lock.h` guarding `Ethernet.loop()` +
  `linkState()` at the source (one enforcement point).
- **s_phase volatile** — add if LTO is ever enabled (safe as-is under `-O2`/no-LTO).

## Related

- Plan: `plans/active/canhub-hardening.md` (Tier-2 rows 13-20).
- Tier-1: `logbook/2026-07-02-canhub-hardening-tier1.md`.
- Adversarial review run journal: `~/.claude/projects/-home-jetson-Desktop-Jugglebot/464186d7-ba4a-4f04-b124-0f6646ec869d/subagents/workflows/wf_1f5084aa-0d9`.
- The deferred-stow inversion this preserves: `logbook/2026-05-19-can-loss-fault-response-safety-inversion.md`.
