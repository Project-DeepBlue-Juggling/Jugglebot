---
title: Can-bridge foundation Phase 0 — native firmware-test harness (growable HAL) + one codegen-allocation pass + RpcMethod/flags-bit lint
type: feature
date: 2026-06-29
status: resolved
phase: "0"
related_plan: canbridge-foundation-coldstart-parity.md
related_entries:
  - 2026-06-27-can-node-teensy-parity-audit
  - 2026-05-19-can-loss-fault-response-safety-inversion
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.h
  - tests/firmware/native/ (new harness — shims, fake_hal, doctest, build.py, two test binaries, golden, README)
  - tests/firmware/test_native_firmware.py
  - tests/firmware/test_rpc_dispatch_lint.py
  - tests/firmware/test_fault_logic.py
commits:
  - 69c1eaf
  - 5f07ad5
subsystem:
  - can
  - ros
  - testing
tags:
  - safety
  - firmware
  - testing
  - codegen
---

# Can-bridge foundation Phase 0 — native firmware-test harness + codegen-allocation pass

## Summary

The inaugural phase of `canbridge-foundation-coldstart-parity.md` (a 7-phase plan
to restore the robot's pre-CAN-offload behaviour on the can-bridge architecture).
Phase 0 ships **no behaviour change** — it lands the test/codegen substrate every
later phase leans on. Two deliverables:

1. **A native firmware-test harness** (`tests/firmware/native/`) that compiles the
   safety-critical firmware (`fault_machine.cpp`, `leg_interp.cpp`) on the build
   **host** and tests the **binary** — closing the standing risk the 2026-06-27
   parity audit flagged (§5): the most safety-critical ported code had *no compiled
   test*, only three hand-synced Python transcriptions, so a C++ edit that diverged
   from the mirrors passed the whole suite.

2. **One coordinated codegen-allocation pass** in `config/generate_udp_protocol.py`
   reserving every new UDP wire id the later phases need (5 RpcMethods + 2 T2J
   MsgTypes), generating the previously prose-only `HeartbeatT2J.flags` bits 0-3 as
   a `HeartbeatT2JFlags` enum, plus a **RpcMethod dispatch lint** so parallel
   sessions can't collide on ids or leave a method un-dispatched.

Verified: full suite **1883 → 1881 passed, 1 xfailed** — a **net −2 pytest tests**
that is *not* a coverage loss: the 19-test hand-maintained Python transcription in
`test_fault_logic.py` was retired in favour of 17 pytest tests (golden conformance +
lint + the two native-binary runners) that drive **122 compiled doctest assertions**
+ the firmware-anchored golden. Firmware `pio run` green; the harness **proven to
catch divergence** (a temporary `<`→`<=` edit to the soft-reset cap failed 4
compiled assertions + the golden-honesty guard, then reverted).

## Motivation

The parity audit's reassuring half — the CAN-loss deferred-stow inversion, the
soft-reset bounce-loop limiter, undervoltage gating — is exactly the code whose
correctness rested on *manual* three-way agreement (`fault_machine.cpp` ==
`tests/firmware/test_fault_logic.py` == `controller/teensy_link/fault_logic.py`).
The audit named this the highest-leverage place a future regression could hide
(§5). Every later phase of this plan edits safety-adjacent firmware (the reboot
watchdog-suppression latch, the axis-6 allow-table, the relay gate), so a compiled
test that fails on a real C++ divergence is the prerequisite for touching that code
with confidence. Separately, the audit and the 7-seam design surfaced **seams
independently claiming the same wire ids** — allocating them all in one Phase-0
pass (with a lint) removes a class of parallel-session merge hazards before the
work fans out.

## Design

### Native harness (Option B: per-module binaries that `#include` the real `.cpp`)

The harness compiles the unmodified firmware behind two ~10-line shim headers
(`hal_shims/Arduino.h`: IntervalTimer stub, PRIMASK intrinsics as no-ops,
micros/millis; `hal_shims/arduino_freertos.h`: pulls Arduino.h, satisfies
`freertos_shim.h`) plus a ~90-line **fake HAL** (`fake_hal.cpp`): a controllable
clock, a settable `udp_last_rx_us`, the `*_active` cold-start predicates, and a
**recording** `can_jugglebot_send` so a test asserts exactly which CAN3 frames the
real logic emits. The fake HAL is built to **grow** — it carries an inbound-CAN3
injection hook (`fake_inject_can3_rx`), unused in Phase 0, that Phases 1/3 will use
for relay reply-correlation and the Get_Version handshake.

Each test binary `#include`s the `.cpp` under test to reach its file-statics, and
links the shared objects (`axis_state.o`, `ball_butler_state.o`, `fake_hal.o`).
Driven by one hash-cached `build.py` and one pytest wrapper
(`test_native_firmware.py`, `skipif` no g++) so `pytest tests/ -q` stays the single
gate. Framework: **doctest** (one vendored MIT header, zero lib build). Static
isolation: a public `interp_reset()` (added to `leg_interp.cpp`; also useful
on-target for re-arm) + `fault_machine_init()` + `fake_reset()` between cases.

The compiled assertions exercise: the soft-reset bounce-loop limiter, UV gating +
the uniform-UV-benign distinction, the deferred-stow **5 invariants** (incl.
never-command-a-dead-bus via the recording HAL and the terminal-IDLE completion),
the Phase-11 present-axis freshness scoping (the single-leg-rig reconnect dead-lock
fix), the recoverable fb-staleness suppression + present-scoped MAX_DEVIATION, and
the interp lead/stroke/present-axis clamps + Hermite/Taylor/decay modes.

### Firmware-anchored golden conformance (so the *host mirror* can't drift either)

`test_fault_machine --emit-golden` dumps the subset of scenarios that the Jetson
host mirror `fault_logic.py` models (`FaultEvaluator`/`DeferredStowLatch`),
recording the **real firmware's** verdict. Two committed-golden guards then keep
all three layers in lockstep — the coupling the manual transcription maintained by
hand, now enforced by tests:

* `test_native_firmware.py` regenerates the golden and asserts it equals the
  committed `native/fault_golden.json` (firmware drift ⇒ fail, on the Jetson);
* `test_fault_logic.py` (repurposed) replays the committed golden through
  `fault_logic.py` and asserts agreement (host-mirror drift ⇒ fail, **everywhere**
  — no compiler needed).

This **retires** the hand-maintained transcription that used to live in
`test_fault_logic.py` (`FaultMirror`/`StowMirror` + the present-axis/fb-stale
re-derivations): the REAL firmware is now the authority, and the present-axis /
fb-stale / MAX_DEVIATION edge cases moved to compiled assertions.
`tests/teensy_link/test_fault_logic_mirror.py` is **kept** (hand-scenario tests for
the same classes, incl. the live `LinkLossLatch`) — complementary to the golden.

### Codegen-allocation pass + lint

One pass reserves the ids verbatim per the plan: RpcMethods `GET_AXIS_VERSIONS`
0x0050, `TILT_READ` 0x0051, `STATE_READ` 0x0052, `STATE_WRITE` 0x0053,
`HAND_TRAJ_CMD` 0x0054; T2J MsgTypes `PLATFORM_FRAME` 0x89, `HAND_CMD_ECHO` 0x8A
(filling the 0x89-0x8F gap below `RPC_RESPONSE` 0x90). The reserved methods get
`ERR_NOT_IMPL` **stub** dispatch cases in `rpc.cpp` (mirroring the `ENCODER_SEARCH`
stub) so the firmware builds and the lint passes; later phases replace the stubs.
The `HeartbeatT2J.flags` bits 0-3 become a generated `HeartbeatT2JFlags` enum (u32;
the enum-emit was generalised from u8/u16 to any width), and the firmware producer
(`Teensy_code_canbridge.ino`) now references the named constants instead of magic
`0x1u..0x8u`. The new `test_rpc_dispatch_lint.py` asserts every `RpcMethod` has a
dispatch case (no method falls to the generic `ERR_UNKNOWN_METHOD` default) and no
`case RpcMethod::X` references a non-existent id (no orphan) — the class of drift
that produced the audit's dead constants.

## Implementation

Two firmware edits, both safe no-ops on the running robot: `interp_reset()` (a new
public reset of the interp file-statics; the firmware build strips it as unused —
binary size byte-identical to baseline) and the `.ino` flags rewire to named
constants (binary size byte-identical). The codegen pass regenerated five delivered
artifacts; `python config/generate_config.py` produced no change (no YAML touched).

Empirical head-start reproduced first (throwaway probe under `/tmp`, not committed):
g++ 9.4 `-std=c++17` compiles the two TUs unchanged behind the shims + fake HAL and
the real `fault_step()` drives the soft-reset limiter correctly (6 clear frames on
attempt #1, ODRIVE_FATAL after the budget); deterministic (identical output +
identical binary hash across runs). The harness build uses `-O2 -Wall
-Wno-unused-function` with exceptions/RTTI **enabled** (doctest needs them; the
firmware TUs neither throw nor use RTTI, so behaviour is unchanged).

## Verification

(date, command, result triples — re-runnable from the artefact alone)

- **Inaugural baseline** (no code changed at this point): `git log -1` = `2ef75d8`;
  `pytest tests/ -q` (run 2026-06-29) = **1883 passed, 1 xfailed in 462.50 s**
  (consistent with the plan's effective baseline 1883/1; the load-flake
  `test_mpc_time_pathologies` passed this run); `pio run` (can-bridge firmware,
  2026-06-29) = **SUCCESS, 5.45 s**, `firmware.elf` text 220480 B.
- **Native harness, direct** (2026-06-29): `./temp/firmware_native/test_fault_machine`
  = **98 assertions / 6 cases pass**; `./temp/firmware_native/test_leg_interp`
  = **24 assertions / 7 cases pass**.
- **Divergence-catch proof** (2026-06-29): temporary `<`→`<=` on the soft-reset cap
  (`fault_machine.cpp:137`) → `test_fault_machine` **4 assertions fail** (the
  `soft_reset_one_shot` scenario) AND `test_committed_golden_matches_live_firmware`
  + `test_native_fault_machine_binary_passes` fail in pytest. Reverted (clean — no
  residual `git diff`); native tests green again (98/98).
- **Firmware + host-mirror subset** (`pytest tests/firmware/ tests/teensy_link/ -q`,
  2026-06-29) = **265 passed in 4.88 s**.
- **Full suite** (`pytest tests/ -q`, run 2026-06-29) = **1881 passed, 1 xfailed in
  444.25 s** — 0 failed. Net **−2** vs the 1883 baseline, fully accounted for and
  **not a regression**: the old `test_fault_logic.py` collected **19** tests (the
  retired `FaultMirror`/`StowMirror` transcription + present-axis/fb-stale
  re-derivations); the new `test_fault_logic.py` (11, golden conformance) +
  `test_native_firmware.py` (3) + `test_rpc_dispatch_lint.py` (3) collect **17**.
  19 → 17 = −2. The retired transcription's coverage moved into the **122 compiled
  doctest assertions** (98 fault-machine + 24 leg-interp) that run inside
  `test_native_firmware.py`'s two binary-runner tests, plus the golden conformance.
- **Firmware build, post-edits** (`pio run`, 2026-06-29) = **SUCCESS, 5.52 s**,
  text 220480 B (byte-identical to baseline → the firmware edits are no-ops).
- **Codegen determinism** (2026-06-29): re-running `generate_config.py` +
  `generate_udp_protocol.py` produced **no new working-tree changes** (byte-identical
  output); only `udp_protocol.*` differ from `HEAD` (the intentional id allocation).

## Discussion

CLAUDE.md makes the Discussion non-negotiable here: a non-obvious tradeoff was
accepted (logic-only scope), the chosen harness approach beat reasonable
alternatives for reasons not inferable from the code, and one deliberate deviation
from the plan's stated design was made.

### Why doctest + `#include`-the-`.cpp`, over the alternatives

**doctest vs GoogleTest** — doctest is a single MIT header with zero library build;
GoogleTest needs a one-time CMake build of the lib + a per-host lib cache to manage.
For a harness whose whole point is "in the normal `pytest tests/ -q` gate, on the
Jetson, with no extra setup," the single-header path keeps the build hash-cacheable
and the dependency a vendored file with a recorded version/licence. The gmock
ergonomics GoogleTest buys are not needed: the seam here is a fake HAL of leaf
functions, not interface mocking.

**`#include`-the-`.cpp` vs a public test bridge** — the safety logic lives in
file-statics (`s_soft_reset_attempts`, `s_stow_pending`, …) with no accessors.
Adding test-only accessors to production firmware would pollute it; `#include`-ing
the `.cpp` into the test TU reaches the statics with zero firmware change. The cost
is the one-TU constraint (a `.cpp` is never both `#include`d and linked as an
object in the same binary, or its symbols double-define).

**Deviation from the plan's `LINKS leg_interp.o`** — the plan specified
`test_fault_machine.cpp #includes fault_machine.cpp and LINKS leg_interp.o`. Under
that design `test_fault_machine` cannot call the **static** `interp_isr()`, so it
cannot complete a stow — and deferred-stow **invariant 5** (terminal IDLE on stow
completion) is a *fault-machine* branch (`fault_machine.cpp:200-207`) that only
fires when `interp_stow_complete()` is true. I `#include` **both**
`fault_machine.cpp` and `leg_interp.cpp` instead (verified ODR-clean: the two TUs
share no symbol names), so the binary drives `interp_isr()` to completion and
asserts the IDLE-all-six + latch-clear as a **compiled** assertion. This is a
strict superset of the plan's coverage; the one-`#include`-per-binary rule is still
respected (each `.cpp` is `#include`d once, neither also linked as an object).

### What the harness validated that the mirror could not — and one mental-model catch

Writing the assertions surfaced a real value-of-compiling moment: my hand-expected
`soft_reset_attempts=1` for the `uv_only_recovers` scenario was **wrong**. The real
`fault_step()` (and `fault_logic.py`, independently) leaves the budget at **0** —
the UV-recovery path's *second* `clear_errors_can()` resets the budget after the
soft-reset path already incremented it. The firmware matched its mirror exactly
(soft_reset=0, clear_calls=2); my mental model didn't. This is precisely the class
of error a compiled-against-ground-truth harness exists to catch, and it is captured
in the golden (so it can't silently regress). Not a firmware bug — surfaced and
confirmed against `fault_logic.py` before fixing the *test's* expectation.

### The accepted tradeoff: logic-only scope

The harness is single-threaded on a fake clock. It does **not** cover FreeRTOS/ISR
concurrency (the deferred-stow re-arm race, the PRIMASK atomic publish, ISR priority
vs the syscall ceiling) or the 500 Hz deadline — those remain on-hardware-replay
gaps (parity item #1 still UNVALIDATED). It also asserts interp **behaviour**
(clamps fired, modes transitioned, descent converged), **not** bit-exact float
equality: aarch64/x86 host float is true IEEE-32 (closer to the Teensy FPU than the
float64 Python mirror), but the float64 numerical role stays with
`test_hermite_xref.py`. This scope is stated in the harness README so a green run is
not over-trusted. The trade is deliberate: the highest-leverage regression risk the
audit named was *decision-logic* divergence under hand transcription, and that is
exactly what this closes.

### Codegen: reserve-all-now + lint, not mint-on-demand

The 7-seam design found seams independently claiming the same ids. Reserving all of
them in one pass — even the ones whose real dispatch is phases away — trades a
handful of `ERR_NOT_IMPL` stubs now for the removal of a whole class of
parallel-session collisions. The lint makes "reserved but un-dispatched" a test
failure rather than a runtime `ERR_UNKNOWN_METHOD`, so a future session that adds a
wire id without a firmware case is told in pytest, not on the bench.

## Open questions / next steps

- **Phase 1 is cleared to start** (Platform-Teensy relay seam): the wire ids are
  reserved, the growable HAL has the inbound-CAN3 injection hook ready for
  reply-correlation, and the axis-6 allow-table can be added against a compiled
  fault/dispatch test.
- The `HeartbeatT2JFlags` enum is wired at the firmware **producer**; the Jetson
  **consumer** (`teensy_bridge_node.py:_T2J_FLAG_*` magic constants) still
  duplicates the bit values. Adopting the generated enum there is a clean no-op a
  future phase touching that node's heartbeat path should fold in (not done here to
  keep Phase 0 to the test/codegen substrate).
- The three-way-agreement rule still holds for any firmware fault/interp edit until
  the native harness fully subsumes the mirrors; the golden conformance now makes
  the *host-mirror* half of that automatic.

## Related

- Plan: [`plans/active/canbridge-foundation-coldstart-parity.md`](../plans/active/canbridge-foundation-coldstart-parity.md) — Phase 0 detail + the codegen-allocation table.
- [2026-06-27-can-node-teensy-parity-audit.md](2026-06-27-can-node-teensy-parity-audit.md) — §5 (no compiled-firmware test) is the risk this closes; the headline cold-start regression is what Phases 1-6 fix.
- [2026-05-19-can-loss-fault-response-safety-inversion.md](2026-05-19-can-loss-fault-response-safety-inversion.md) — the deferred-stow 5 invariants the harness now asserts as compiled tests.
- Harness usage + scope: [`tests/firmware/native/README.md`](../tests/firmware/native/README.md).
