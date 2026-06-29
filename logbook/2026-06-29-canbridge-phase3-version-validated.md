---
title: Can-bridge foundation Phase 3 — firmware Get_Version sweep + Jetson-validated firmware_validated (+ reconnect-residual close)
type: feature
date: 2026-06-29
status: resolved
phase: "3"
related_plan: canbridge-foundation-coldstart-parity.md
related_entries:
  - 2026-06-29-canbridge-phase2-coldstart-relay-state
  - 2026-06-29-canbridge-phase1-platform-relay-seam
  - 2026-06-29-canbridge-phase0-native-harness
  - 2026-06-27-can-node-teensy-parity-audit
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - controller/teensy_link/protocol.py
  - controller/teensy_link/rpc_args.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/version_check.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/version_check.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/firmware/native/test_version_check.cpp
  - tests/firmware/native/build.py
  - tests/firmware/test_native_firmware.py
  - tests/firmware/test_rpc_dispatch_lint.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/ros/test_teensy_bridge_node_version.py
  - tests/ros/test_teensy_bridge_node_coldstart.py
  - tests/ros/test_teensy_bridge_node_read.py
  - tests/teensy_link/test_rpc_args.py
  - tools/probes/canbridge_version_probe.py
commits:
  - bf400b7
  - 35f48f1
  - TBD-probe
subsystem:
  - can
  - ros
  - firmware
  - testing
tags:
  - cold-start
  - firmware-version
  - validate-group
  - parity
  - reconnect-residual
---

# Can-bridge foundation Phase 3 — Get_Version handshake + firmware_validated

## Summary

Phase 3 of `canbridge-foundation-coldstart-parity.md` restores can_node's
firmware-version handshake on the can-bridge, replacing the hardcoded
`firmware_validated=False` that wedged the orchestrator's BOOT at
`state_machine.py:232` until the 30 s `BOOT_TIMEOUT_S` → FAULT. The firmware now
sweeps `Get_Version` across the present Jugglebot axes (one frame per cold-start
monitor tick, bus-paced) and caches the raw 8-byte replies; the bridge pulls them
once via a new `GET_AXIS_VERSIONS` RPC (a bridge-**local** cache read — no CAN3
round-trip) and runs the **existing tested** `MotorStateTracker.validate_group`
against `EXPECTED_HW_VERSIONS`, latching `firmware_validated`. A version mismatch
keeps `firmware_validated=False` **and** appends the mismatch string to
`robot_state.error`, force-FAULTing the orchestrator (exact can_node parity for a
wrong-firmware ODrive). **Version semantics stay entirely in tested Python — the
firmware parses nothing.**

Because landing a real `firmware_validated` un-gates the orchestrator's `is_homed`
skip (`state_machine.py:228-235`) for the first time, Phase 3 also **closes the
Phase-2 reconnect-trigger residual** (its mandated precondition): a CAN3-bus-health
(`bus1_health` → OK from a degraded state) reconnect re-trigger that re-reads the
Platform Teensy's cold-start state **conservatively** (retry, then `is_homed=False`
on failure) — so a Jugglebot power-cycle that the UDP-watchdog never sees can no
longer leave a stale `is_homed=True` against a de-referenced robot.

## Motivation

The 2026-06-27 parity audit found the bridge fed the orchestrator a constant
`firmware_validated=False`. With the production launch co-starting
`orchestrator_node` + the bridge, BOOT (`state_machine.py:232`) waits on
`firmware_validated`; constant `False` means it never proceeds and FAULTs on the
boot timeout — the automated cold-start is dead. can_node instead queried each
ODrive's version once heartbeats arrived (`_handle_heartbeat:325-329` →
`_send_next_version_query:341-349`), validated the group when complete
(`_handle_get_version:487-495`), and set `fatal_error` + an error string on a
mismatch — the behaviour parity requires.

The Phase-2 logbook §"The reconnect-trigger residual" flagged that the Phase-2
`is_homed` consumption becomes dangerous the moment `firmware_validated` goes live:
the UDP-watchdog reconnect re-read never fires for a CAN3-only drop (Jugglebot
disconnected, Jetson + can-bridge stay powered — the power topology,
`project_canbridge_power_topology`), so a stale `is_homed=True` could survive and
let the orchestrator skip homing on a de-referenced robot. The audit re-targeted
this from "a Phase-4 probe" to a **hard Phase-3 precondition** — closed here.

## Design

### Firmware: the Get_Version sweep + raw-version cache (`version_check.{h,cpp}`)

A new module owns a per-axis raw-version cache (`s_version_raw[NUM_AXES][8]`), a
received bitmask, and a query-sent bitmask. `version_check_step()` runs in the
**cold-start monitor task** (`task_homing`, `HOMING_RATE_HZ`=100 — *not* the
safety-critical fault task) alongside homing/activate/deactivate: it sends **one**
`Get_Version` per tick to the next present-but-unqueried Jugglebot axis (present =
`axes[i].heartbeat_seen`), gated on `jugglebot_commands_allowed()`, and idles once
swept. `can_buses.cpp` `decode_into_cache` gains a `get_version` case that calls
`version_record(axis, d)` (the 8-byte reply passes the existing `len<8` drop; the
empty request never round-trips with `SRX_DIS`). The `GET_AXIS_VERSIONS` RPC
(`rpc.cpp`) returns `version_fill_blob()` synchronously in the RPC response.

### Wire: `ResultAxisVersions` codegen struct (mask + axis-major raw)

`GET_AXIS_VERSIONS` takes no args and returns a new codegen struct
`ResultAxisVersions { u8 received_mask; u8 raw[56]; }` (= `1 + NUM_AXES*8` = 57 B,
≤ the 64 B RPC result buffer) **synchronously in the RPC response** — unlike the
Phase-1 relay reads (`TILT_READ`/`STATE_READ`) which trigger an async
`PLATFORM_FRAME` over CAN3. The versions are a bridge-local cache, so no CAN3
round-trip is needed on the pull. This is the first RpcArg struct with an array
field, so the codegen's RpcArg Python emitter was extended to splat arrays on pack
and regroup them on unpack (mirroring the MESSAGES path); the C++ POD path already
handled arrays.

### Host: pull → decode → validate_group → latch (`_version_check_poll`, 1 Hz)

A 1 Hz timer pulls `GET_AXIS_VERSIONS`, decodes the set-bit axes via
`rpc_args.decode_axis_versions_result` + `jugglebot.can.odrive.decode_get_version`,
records each into a `MotorStateTracker` (reused **only** for `record_version` +
`validate_group`), and once `all_jugglebot_versions_received()` runs
`validate_group(JUGGLEBOT_AXES, "Jugglebot")`. It latches:
`firmware_validated=True` on a clean match, or `_firmware_mismatch_error=<str>` on
a mismatch (sticky). The poll no-ops once resolved, and runs **off** the publish
path (the RPC is a cheap UDP round-trip reading a bridge-local cache). The publish
path sources `msg.firmware_validated` from the latch, prepends the mismatch string
to `msg.error`, and ORs the mismatch into `has_fatal_odrive_error` — exact
can_node:1085 / 491 ordering and flags.

### The reconnect-residual close (`_health_check` CAN3-bus-health edge)

`_health_check` (1 Hz) gains a `bus1_health` (CAN3 core bus) edge detector: a
transition **to OK from a degraded state (WARN/BUS_OFF)** fires
`_read_cold_start_state_conservative('can3_reconnect')` — the boot read's
retry-then-`is_homed=False` fallback, generalised. The UDP-watchdog reconnect
(Phase 2) stays **keep-stale** on failure (a link blip is not a power loss); the
CAN3 path is **conservative** (a CAN3 recovery implies the Platform Teensy may have
power-cycled). `UNKNOWN→OK` (the boot first-connection) is **excluded** so it
cannot clobber a good `__init__` boot read.

## Implementation

- **Codegen** (`generate_udp_protocol.py`): added `ResultAxisVersions`; extended
  the RpcArg Python emitter to handle array fields; regenerated `udp_protocol.h`/
  `.py` + the delivered copies + `docs/teensy-udp-protocol.md`; re-exported
  `ResultAxisVersions` via `controller/teensy_link/protocol.py`.
- **Firmware**: new `version_check.{h,cpp}`; `can_buses.cpp` `get_version` decode
  case → `version_record`; `rpc.cpp` `GET_AXIS_VERSIONS` → `version_fill_blob`
  (the reserved ERR_NOT_IMPL stub now carries only `HAND_TRAJ_CMD`), result-buffer
  cap named `RESULT_BUF_CAP`; `.ino` adds `version_check_init()` + `version_check_step()`.
- **Host** (`teensy_bridge_node.py`): `_versions`/`_firmware_validated`/
  `_firmware_mismatch_error` state; `_version_check_poll` timer; publish-path
  sourcing of `firmware_validated` + mismatch error/flag; `_read_cold_start_state_
  conservative` (extracted from the boot read); CAN3-bus-health edge in `_health_check`;
  `_last_bus1_health` edge tracker.
- **Host helper** (`rpc_args.py`): `encode_axis_versions_result` /
  `decode_axis_versions_result` (the firmware blob mirror).

## Verification

- **Native firmware harness** (`test_version_check.cpp`, compiled host-side via the
  Phase-0 growable HAL using the `fake_inject_can3_rx` hook): the Get_Version sweep
  (one frame/tick, present-axis only, bus-gated), `version_record` + the
  `ResultAxisVersions` blob fill, and the buffer-cap guard — **5 cases / 37
  assertions PASS** (`python tests/firmware/native/build.py` + direct binary run,
  2026-06-29).
- **`pio run`** (can-bridge firmware, 2026-06-29): **SUCCESS** (text 220480→221504,
  +1 KB for the version module); `config/generate_udp_protocol.py` deterministic.
- **Full suite** (`pytest tests/ -q`, run 2026-06-29, three times incl. the
  post-audit-fix run): each run **1919 passed, 1 xfailed, 2 failed** (453 s / 481 s /
  457.99 s). EVERY failure across all three runs is an allocation-budget / timing
  test that PASSES IN ISOLATION — `test_hot_loop_allocation_contract` (all 3 runs,
  the documented order-flake `project_hot_loop_alloc_test_flaky`),
  `test_t3b_h4_on_post_solve_allocates_within_budget` (both idle runs — same
  allocation-budget class, order-sensitive), and `test_enabled_sends_setpoint_frame`
  (only the contended run — a 2 s-timeout-under-load miss). Verified passing
  isolated (2026-06-29): hot-loop 3/3 (15.9 s), the mpc-allocation test 1/1
  (7.3 s), all three together 14/14 (23.9 s), and the affected bridge files 53/53
  after the audit fixes. **None touch the Phase-3 code** — firmware version
  validation + cold-start reconnect share ZERO code with the MPC hot loop /
  allocation path / setpoint pump; the failures are the order-sensitive
  allocation-budget measurement being perturbed by the ~13 added tests that collect
  before `tests/sim/` (an allocation REGRESSION would fail isolated too — these do
  not). **Effective: 1919 passed + 1 xfailed, net +13 over the clean baseline**
  (this session `pytest tests/ -q` = 1908 passed, 1 xfailed, 0 failed in 456.58 s).
  New tests: `test_teensy_bridge_node_version.py` (6),
  `test_teensy_bridge_node_coldstart.py` (+3 CAN3-health reconnect),
  `test_rpc_args.py` (+1), `test_udp_protocol_xlang.py` (+1),
  `test_rpc_dispatch_lint.py` (+1 `test_get_axis_versions_is_implemented`),
  `test_native_firmware.py` (+1 version-check binary), and the rewritten
  `test_firmware_validated_conservative_false`.

## Discussion

### The firmware_validated latch policy: mismatch → active FAULT, cannot-validate → passive BOOT-timeout

The root failure modes a wrong-firmware ODrive prevents are: (a) running leg
PID/feedforward against an ODrive whose control loop or sign conventions differ
from what the Jugglebot config assumes — the dangerous-motion class CLAUDE.md warns
about; (b) silently operating on an un-validatable stack during a host/firmware
version skew. can_node's resolution was a **single determination once all versions
arrive**: mismatch → `fatal_error=True` + an appended error string; all-match →
`firmware_validated=True`; never-arrive → stays False and BOOT times out.

I mirrored this exactly rather than inventing an active "cannot-validate" FAULT
(the rejected option from the design questions), for two concrete reasons. First, a
genuine host/firmware skew (host has Phase 3, firmware is older and answers
`ERR_UNKNOWN_METHOD`/`ERR_NOT_IMPL`) already FAULTs **passively** via the existing
30 s `BOOT_TIMEOUT_S` — adding an active error there is redundant and diverges from
can_node. Second, an active FAULT on "cannot validate yet" risks FAULTing during
the **transient pre-sweep window**: at cold boot the bridge may pull before the
firmware has finished its ≤7-frame sweep, so an early pull returns an incomplete
blob; treating that as fatal would be a false positive. The poll therefore retries
(cheap, no-op once resolved) and lets BOOT_TIMEOUT govern the give-up — which is
precisely can_node's asynchronous behaviour. The mismatch path *is* active (the
string in `robot_state.error` force-FAULTs the orchestrator at
`orchestrator_node.py:137-139`) because a real mismatch is a hard configuration
error, not a transient.

### The reconnect-residual close, and the WARN-vs-BUS_OFF firmware nuance

The Phase-2 residual is the load-bearing safety item here, because Phase 3 is what
makes the stale-`is_homed=True` hole *reachable* (it un-gates the skip). The plan
named the trigger "`bus1_health` BUS_OFF→OK", but the firmware's `health_of()`
(`can_buses.cpp`) only emits `UNKNOWN`/`WARN`/`OK` today — `BUS_OFF` is a documented
FlexCAN-register TODO. So the **implementable** trigger is "→OK from a degraded
state", which under current firmware is `WARN→OK` (CAN3 heartbeats go stale when
Jugglebot drops → `WARN`; resume on reconnect → `OK`); `BUS_OFF→OK` is subsumed for
free when the register read lands. I excluded `UNKNOWN→OK` deliberately: `UNKNOWN`
is the boot "no frames seen yet" state, not a loss, and firing the conservative
re-read there could clobber a **good** `__init__` boot read with the
`is_homed=False` fallback on a transient read failure — a needless re-home. The
test `test_can3_health_unknown_to_ok_does_not_fire` pins that exclusion.

The **asymmetry** between the two reconnect paths is the whole point and is itself
a contract: the UDP-watchdog reconnect *keeps* the stale cache on a failed read
(can_node passive last-known parity — a Jetson↔Teensy link blip leaves the Platform
Teensy and its references powered, so a re-home would be wrong), while the CAN3
path *clears* to `is_homed=False` on a failed read (a CAN3 recovery implies the
Platform Teensy — on Jugglebot's 12 V logic, sharing the ODrive supply — may have
power-cycled and forgotten). `test_can3_health_recovery_conservative_on_read_failure`
and the pre-existing `test_refresh_failure_keeps_cached_value` pin both halves.

### Synchronous RPC result vs the PLATFORM_FRAME relay path

`GET_AXIS_VERSIONS` returns the blob **synchronously** in the RPC response, unlike
the Phase-1 relay reads which trigger an async `PLATFORM_FRAME` over CAN3. This is
correct because the versions live in a **bridge-local** cache (filled by the
firmware's own sweep), so there is no Platform-Teensy round-trip to await — the RPC
dispatcher already supports a result blob (`result[64]`/`res_len`), which no method
used before. Using the relay's async path here would have invented a CAN3
round-trip that does not exist.

### Why reuse MotorStateTracker.validate_group (zero firmware version semantics)

`validate_group` (`motor_state.py:194-229`) is already tested and is the exact
can_node validation (per-axis hw check + intra-group fw consistency). Putting it on
the Jetson keeps version *policy* in tested Python and the firmware a dumb byte
cache — so a policy change (new expected versions, a looser fw rule) never needs a
firmware reflash. Instantiating a whole `MotorStateTracker` only for its version
methods is slightly heavy but reuses the exact tested code rather than
re-implementing the group logic.

### The audit-caught MEDIUM: the conservative re-read must not block the publish

A pre-commit `/audit --staged` (2026-06-29; APPROVE WITH MINOR FIXES — no HIGH)
flagged that the CAN3 conservative re-read, as first written, ran **synchronously**
in the 1 Hz `_health_check` callback. Because `_health_check`, the 100 Hz
`_publish_robot_state`, and `_version_check_poll` all share the node's default
`MutuallyExclusiveCallbackGroup`, the re-read's bounded retries + `time.sleep`s +
relay round-trips (~1.9 s worst case) would **stall the `robot_state` stream for up
to ~1.9 s** on a CAN3-recovery edge — wider than the ~0.5 s Phase-2 UDP-reconnect
read it built on, and enough to perturb the orchestrator's staleness handling /
the GUI. **Fixed in-session** (`_read_cold_start_state_conservative_async`): the
re-read is dispatched to a short-lived daemon thread guarded by
`_cold_start_reread_inflight` (one re-read in flight — the recovery edge is
one-shot), so the 1 Hz timer returns immediately. The thread touches only the
cold-start cache under `self._lock`, and the relay RPC / `_await_platform_reply`
primitives are already thread-safe (the RX thread feeds them). The CAN3-reconnect
tests now synchronise on the in-flight flag. Two LOW findings: the version poll's
RPC (same shared group) is bounded to a short `timeout=0.3, retries=1` (it is a
sub-ms local-cache read normally; this caps the worst case on a degraded link); and
`has_fatal_odrive_error` stays sticky-True after a version mismatch even across a
`CLEAR_ERRORS` (a deliberate divergence from can_node's clearable `fatal_error` —
a wrong-firmware ODrive is a hard config error that must stay fatal until the
firmware is fixed + the bridge restarted; the plan's "latched OR-term" instruction;
both behaviours still force-FAULT via the sticky error string). Phase 4's holistic
`ReentrantCallbackGroup` restructure subsumes the shared-group root cause; this
in-session fix removes the acute ~1.9 s stall now.

## Open questions / next steps

- **Bench probe — PASSED on hardware (2026-06-29, `tools/probes/canbridge_version_probe.py`,
  motor power OFF + CAN3 connected, Phase-3 firmware flashed via `pio run -t upload`):**
  all 7 Jugglebot axes (6 legs + hand) report **hw 4.4.58** (matching
  `EXPECTED_HW_VERSIONS`) and a consistent **fw 0.6.11**; the inlined
  `validate_group` mirror returns clean → `firmware_validated` would latch True on
  the production bridge. This exercised the WHOLE firmware path end-to-end —
  Get_Version sweep → per-axis cache → `GET_AXIS_VERSIONS` RPC → host decode →
  validate. The expected-version config is confirmed correct; no Phase-4
  cold-start FAULT from a stale version tuple.
- **BUS_OFF firmware TODO:** when `can_buses.cpp` `health_of()` gains the FlexCAN
  ESR1 register read, `BUS_OFF→OK` joins `WARN→OK` automatically (no host change).
- **Phase 4** wires the orchestrator action/services and the
  `ReentrantCallbackGroup`; Phase 4 depends on 1, 2, 3, **and 5**.

## Related

- Plan: `plans/active/canbridge-foundation-coldstart-parity.md` (Phase 3 + the
  PRECONDITION block).
- Phase 2: `2026-06-29-canbridge-phase2-coldstart-relay-state.md` §"The
  reconnect-trigger residual" (the hole this entry closes).
- Phase 1: `2026-06-29-canbridge-phase1-platform-relay-seam.md` (the relay seam +
  FakeTeensy harness precedent).
- Phase 0: `2026-06-29-canbridge-phase0-native-harness.md` (the growable HAL +
  `fake_inject_can3_rx` hook + the `GET_AXIS_VERSIONS=0x0050` reservation).
- `project_canbridge_power_topology` (why CAN3 health, not the UDP link, signals a
  reference loss).
