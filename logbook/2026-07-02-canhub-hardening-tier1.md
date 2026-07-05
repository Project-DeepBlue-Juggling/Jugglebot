---
title: Can-hub hardening Tier-1 — host/test/doc robustness pass (Fable-5 fresh-eyes review)
type: feature
date: 2026-07-02
status: resolved
phase: "Tier-1"
related_plan: canhub-hardening.md
related_entries:
  - 2026-07-02-canbridge-phase4-orchestrator-wiring
  - 2026-07-02-canbridge-reboot-encoder-search-clear
  - 2026-06-27-can-node-teensy-parity-audit
files_changed:
  - controller/teensy_link/setpoint_pump.py
  - controller/teensy_link/rpc.py
  - controller/teensy_link/rpc_args.py
  - controller/teensy_link/activate.py
  - controller/teensy_link/replay_setpoint.py
  - controller/teensy_link/client.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/platformio.ini
  - tests/firmware/native/coldstart_hal.cpp
  - tests/firmware/native/test_leg_activate.cpp
  - tests/firmware/native/test_leg_deactivate.cpp
  - tests/firmware/native/test_leg_homing.cpp
  - tests/firmware/native/test_udp_framing.cpp
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/firmware/test_firmware_build_pins.py
  # Tier-1 close-out (2026-07-05): typed CrcError + doc truth sweep
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.py
  - controller/teensy_link/protocol.py
  - tests/teensy_link/test_protocol_codec.py
  - tests/teensy_link/test_client.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/README.md
  - tests/firmware/native/README.md
commits:
  - f8397c7
  - c7425e9
  - 37a81d4
  - 431ea89
  - 8821451
  - 6f20fea   # item 8 — rpc dispatch + odrive/BB byte xrefs
  - db4ddf8   # item 8 — dead-bus + first_seen goldens
  - 2cf3a07   # item 11 — typed CrcError
subsystem:
  - controller
  - ros
  - can
  - testing
tags:
  - hardening
  - cold-start
  - protocol
  - test-coverage
  - fable-5
---

## Summary

Tier-1 of the can-hub hardening pass (plan `canhub-hardening.md`), driven by a
fresh **Fable-5 multi-agent review** of the whole can-hub Teensy firmware/UDP
pipeline (8 area-reviewers → adversarial verification of every HIGH → synthesis).
Tier-1 is the **host / test / doc** half: low-risk, pytest-gated, **NO re-flash**.
The firmware behaviour-change items (E-STOP latch, monotonic clock, NetLock, trust
boundaries) are Tier-2 and deferred to a flash cycle + a powered re-validation.

> **Numbering:** bracket tags `[N]` in this entry are the plan's **1–21 item
> numbers** (`canhub-hardening.md`); `gap N` are the plan's coverage-gap numbers.
> The commit *messages* additionally carry the Fable-5 **review-finding** numbers,
> a SEPARATE scheme that does NOT match the plan item numbers (e.g. review-finding
> `[14]` = the native/golden work = plan **item 8**; plan **item 14** is the pending
> Tier-2 monotonic clock). Where a commit is named its SHA is cited directly.

Landed this cycle, each its own commit:

- **A — host `teensy_link` robustness** (`f8397c7`): exception-contain the
  production setpoint thread (reject-not-raise in `SetpointPump.build()`; wrap
  `_process_setpoint`; OSError-guard the send); stop auto-retrying **non-idempotent**
  RPCs host-side + fix the false idempotence docstring + the retry-race; wire
  `SetpointPump.reset()` at reconnect + the mpc_active 0→1 edge; host bundle
  (`ActivateMonitor` requires CLOSED_LOOP, `scale_to_bench` raises on floor>ceiling,
  capped socket drain, `rpc_args` METHOD += DEACTIVATE).
- **B — `teensy_bridge_node` robustness** (`c7425e9`): surface UDP-link loss as
  `has_fatal_can_error` (dropped `can_node` parity); fix the cross-axis
  disarm-while-CLOSED_LOOP predicate; restore hand vel/curr-limit topic parity;
  bounded cone catch-event queue; a re-entrant relay serialization lock closing the
  STATE_WRITE lost-update; async cold-start re-reads off the 1 Hz timer.
- **C — test-harness + codegen coverage**: PROTOCOL_VERSION + wire-layout freeze +
  codegen lints + firmware lib pins (`37a81d4`, item 9); compile
  `leg_homing/activate/deactivate.cpp` into the native harness + retire the
  tautological Python xrefs (`431ea89`, item 3); execute the C++ UDP framing codec
  natively (`8821451`, item 8 / gap 4 — the first slice of the native/golden expansion).

**Observable robot-behaviour changes** (need operator eyeball on hardware): B's
link-loss→fatal ([2]), cross-axis disarm ([6]), hand-limit topic ([7]). The
hardware-confirmation checks are in **Remaining → Hardware confirmation** below.

## Context

The canbridge-foundation cold-start-parity plan had just landed (Phase 4 +
reboot-bugfix + the powered validations, `2026-07-02`). Fable-5's verdict:
*"Production-quality core, pre-hardening edges."* — 6 independently-confirmed HIGH
findings clustering exactly where the powered sittings could not reach (the last
line of defence + rare paths). The review's six cross-cutting themes (ported
safety-semantic divergences at the parity edges; one time-base root cause driving a
whole watchdog class; single-layer trust at every rare-path boundary; test coverage
inversely correlated with consequence at the firmware layer; the safety-relevant 5%
of concurrency misses; small-but-concentrated doc drift) map onto the 21 tiered
items. Tier-1 closes the host/test/doc subset before autonomous juggling.

## Changes

### A — host `teensy_link` robustness (`f8397c7`)

The setpoint downlink is the production leg path. Three single-layer-trust gaps:
`build()` could **raise** on a malformed frame (killing the 40 Hz thread while
mpc_active stayed 1 — a one-frame problem became a restart-only outage); a send
`OSError` (ENETUNREACH before the peer is up) was unguarded; and one bad frame
anywhere in `_process_setpoint` escaped to the loop. Fix: `build()` **rejects, never
raises** (bad `_finite_vec` clears the flag, top-level backstop → `frames_rejected++`);
`_process_setpoint` is fully exception-contained with an inner OSError guard on the
send. Complementary: `NON_IDEMPOTENT_METHODS` (HOME/ACTIVATE/DEACTIVATE/REBOOT/
BB_THROW/HAND_TRAJ_CMD) force `retries=0` in `RpcClient.call()` — the firmware has
**no RPC dedup**, so a lost RESPONSE must not re-dispatch a physical move; the false
"idempotent" docstring is fixed and the retry-race (`got` flag + post-retry
`pending.event` check) closed. `SetpointPump.reset()` is now wired at a confirmed
reconnect and the mpc_active 0→1 edge (a stale `_prev_pos` from a prior pose would
otherwise wedge the per-step gate forever). Host bundle: `ActivateMonitor` DONE now
requires `axis_state == CLOSED_LOOP` (an abort-to-IDLE-at-target must not read as
success); `scale_to_bench` raises on an unusable floor>ceiling bench config;
`LinkStats.drain_capped` + a `_MAX_DRAIN_PER_WAKE=256` bound on `_drain_socket`;
`rpc_args.RpcMethod.DEACTIVATE: ArgAxisOnly`.

### B — `teensy_bridge_node` robustness (`c7425e9`)

- **Link-loss → fatal ([2], observable).** `has_fatal_can_error` now ORs in the
  `LinkLossLatch`, with a distinct operator-facing error string ("Teensy link lost
  (UDP) …"). Without it, a dead Jetson↔Teensy link left the orchestrator (whose
  ONLY health input is `robot_state`) consuming frozen motor states stamped with
  fresh clock times — `can_node` coupled 2 s silence → fatal_can, and that coupling
  was dropped in the port.
- **Cross-axis disarm ([6], observable).** Fatal is now `(any leg disarmed) AND
  (any leg in CLOSED_LOOP)` — the `fault_logic` / `can_node` parity. The prior
  per-leg conjunction (the *same* leg disarmed AND CLOSED_LOOP) essentially never
  fired (a disarmed ODrive leaves CLOSED_LOOP almost immediately), so the common
  real event — one leg drops torque while the others hold — slipped through exactly
  the OR-term meant to un-mask it.
- **Hand vel/curr limits ([7], observable).** `set_motor_vel_curr_limits` now
  applies the hand ODrive (axis 6, bridge-owned on CAN3) and caches the values so a
  later `_run_configure` re-applies the operator's update — reversing a can_node-
  parity regression.
- **Bounded cone queue + relay lock + async re-reads (item 10).** The cone catch-event
  queue is bounded to 4000 (drop-oldest). The relay lock is an **RLock** held across
  the *whole* read-modify-write in `_write_is_homed`/`_write_level_state`/
  `_clear_cold_start_state_on_reboot` (each nested `relay_write_robot_state`
  re-acquires it), closing the STATE_WRITE lost-update. Both cold-start re-read
  triggers now dispatch off the 1 Hz `_health_check` timer via a shared
  `_dispatch_cold_start_reread` + inflight guard.

### C — test-harness + codegen coverage

- **Item 9 — codegen (`37a81d4`).** PROTOCOL_VERSION frozen across generator + Python +
  C++; a **wire-layout freeze** (sha256 over every message/arg field layout, framed
  MsgType values, framing constants) forces a layout change to either bump the
  version or re-pin deliberately — the guard the `0935c63` layout-without-bump
  regression would have tripped. Codegen lints (enum value/name uniqueness, msg_type
  membership + uniqueness, payload/arg budget, enum-value-fits-width, MAX_PAYLOAD
  parity). Firmware lib pins in `platformio.ini` (teensy@5.1.0, freertos-teensy
  @commit 7e6ae6a, QNEthernet@0.35.0 — the exact versions the flashed firmware was
  built against) + a text-parse guard.
- **Item 3 — cold-start native drivers (`431ea89`).** The code that drives legs into
  hardstops (`leg_homing/activate/deactivate.cpp`) was never compiled by any test.
  Three new native doctest drivers (+130 byte-parity assertions) compile the real
  `.cpp` and assert the emitted CAN frames byte-for-byte; the tautological Python
  xref transcriptions are retired in favour of them (cross-file + config-sanity pins
  kept). A separate self-contained fake HAL (`coldstart_hal.cpp`) resolves the ODR +
  gate differences.
- **Item 8 / gap 4 — C++ framing (`8821451`).** The hand-written C++ framing codec (the
  network trust boundary) is now EXECUTED by a native test — canonical CRC anchor,
  round-trip, and every rejection.

## Discussion

*(Written before the Verification section, per the philosophy — the non-obvious
decisions where a future reader wouldn't infer the "why" from the code.)*

### The relay lock must be an RLock spanning the whole RMW, not a plain Lock on the wire write

The naive read of item 10 ("serialize the relay round-trips") is a `threading.Lock`
inside each relay method. That serializes the two STATE_WRITE **wire writes** but
does **not** close the lost-update: `_write_is_homed` and `_write_level_state` each do
read-cache → wire-write → update-cache, and if both read the same stale cache before
either writes, the second wire write lands last and clobbers the first's field on the
Platform Teensy (a homing write erased by a concurrent levelling write, or vice
versa). Closing it requires holding the lock across the **entire** RMW — but the RMW
helpers call `relay_write_robot_state`, which also takes the lock, so a plain Lock
would self-deadlock. Hence an **RLock**, acquired at the top of each RMW helper and
re-acquired (harmlessly) inside `relay_write_robot_state`. The serialization test
proves the whole RMW is under the lock by blocking inside the wire write and
confirming a second thread cannot acquire it until the RMW completes.

### The two cold-start re-reads share ONE inflight guard — and the CAN3 caller preserves its edge

Making the UDP-reconnect re-read async (item 10, the async-reread sub-part) created an interaction: the
CONSERVATIVE (CAN3-recovery, is_homed=False on failure) and the KEEP-STALE
(UDP-reconnect) reads could now run concurrently and race the cache — where the
weaker keep-stale read could land an is_homed=True write LAST and resurrect a stale
reference after a power-cycle. A **single** inflight guard prevents that race
(at most one re-read of either kind at a time). The cost is a same-tick collision:
if the UDP read wins the guard, the CAN3 conservative read is skipped for that tick —
so `_dispatch_cold_start_reread` returns whether it started, and the CAN3 caller only
consumes its bus-health recovery edge (`_last_bus1_health = OK`) when it actually
dispatched. Otherwise it leaves the edge DEGRADED so the stronger conservative read
re-fires next tick — never permanently preempted by the weaker path. The asymmetry
(UDP blip KEEPS stale; CAN3 recovery goes conservative) is the documented Phase-3
safety invariant: a Jetson↔Teensy blip leaves the Platform Teensy powered (references
intact), but a CAN3-health recovery implies the shared ODrive supply may have cycled.

### The cold-start drivers get a *separate* fake HAL, not an edit to fake_hal

Each cold-start module DEFINES its own `*_active()` and reads a bus/fault gate
(`can_buses_stats().jugglebot_health` + `fault_can_bus_down()` + `fault_guard_mode()`)
that the fault/interp TUs don't. A driver that `#include`s `leg_activate.cpp` would
ODR-clash on `activate_active()` against `fake_hal.o`'s fake, and `fake_hal.o` is
compiled once + shared, so it can't be per-binary macro-guarded. Editing `fake_hal`
to split the predicates out risked the 5 existing native binaries (`fault_machine.cpp`
DEFINES `fault_can_bus_down`/`fault_guard_mode`, so those can't move to a shared
fake either). The clean resolution: a **self-contained** `coldstart_hal.cpp` linked
ONLY by the cold-start drivers (never alongside `fake_hal.o`), with each driver
supplying its two SIBLING `*_active()` inline. A ~30-line recording-TX duplication was
the accepted cost of decoupling — zero risk to the existing binaries. The three
modules could NOT share one TU: they use identical file-static names (`s_phase`,
`s_start_req`, `s_result`), so `#include`ing all three collides.

### Retiring the xrefs is a coverage *upgrade*, not a reduction

The pytest count drops ~13 from retiring the tautological transcriptions (net −10
after the +3 native driver wrappers), which looks like a regression. It isn't: those
tests were `hw.X == hw.X`
self-comparisons (the mirror returned the same constant it compared against) or
`firmware == cannode` checks where both sides were the same Python formula. The
native drivers assert the REAL emitted frames byte-for-byte (and the homing driver
runs the real **float32** EMA — closer to the Teensy FPU than the retired float64
Python). A Fable-5 fresh-eyes analysis classified every xref assertion; what a
single-TU native test cannot see was KEPT: the cross-file firmware-tol-vs-Jetson-
observer pin, the DEACTIVATE RpcMethod/rpc_args wire parity, and the pure-Python
config-sanity pins (which still run when g++ is absent and the native harness skips).

### Lib pinning is behaviour-preserving reproducibility, not an upgrade

`platformio.ini` pinned to the versions **already installed** (the flashed binary's
provenance), so the next `pio run` on this Jetson is unchanged — the pin only
constrains future resolves and other machines. A floating `platform = teensy` / bare
git URL / bare lib name would silently pull a newer toolchain/library and change the
binary under a validated flash with no local edit.

## Verification

- **A** (`pytest tests/ -q`, run 2026-07-02): **2002 passed, 1 xfailed in 456.30 s**.
- **B** (`pytest tests/ -q`, run 2026-07-03): **2013 passed, 1 xfailed in 461.72 s**
  (+11 tests/ros: link-loss fatal, cross-axis disarm ×2, hand-limit ×2, cone bound,
  relay RLock ×2, mpc_active edge reset, `_process_setpoint` containment ×2).
- **Item 9** (`pytest tests/ -q`, run 2026-07-03): **2024 passed, 1 xfailed in 461.25 s**
  (+11 codegen/build-pin tests).
- **Item 3** (`pytest tests/ -q`, run 2026-07-03): **2014 passed, 1 xfailed in 459.00 s**
  (net −10: +3 native driver wrappers, −13 retired xref tautologies; the real
  coverage is +130 byte-parity doctest assertions across the 3 compiled binaries).
- **Item 8 / gap 4** (`pytest tests/ -q`, run 2026-07-03): **2015 passed, 1 xfailed in
  464.94 s** (+1 native framing wrapper, +26 doctest assertions).
- **Close-out** (`pytest tests/ -q`, run 2026-07-05): **2049 passed, 1 xfailed in
  646.86 s** — covers the typed-CrcError tests (+2 codec / +1 client, `2cf3a07`) and
  the already-in-tree item-8 native/golden tests (`6f20fea`, `db4ddf8`, each green at
  its own landing). The doc-only close-out edits (READMEs, plan, this entry) have no
  test surface. Order-flaky alloc tests confirmed isolated, as before.
- Native harness (`python tests/firmware/native/build.py`): all 9 binaries build +
  pass on the Jetson g++; the new cold-start drivers = 10+7+9 cases / 130 assertions,
  framing = 5 cases / 26 assertions.
- Order-flaky allocation tests (`test_hot_loop_allocation_contract`,
  `test_t3b_h4_on_post_solve_allocates_within_budget`) confirmed isolated (memory
  `project_hot_loop_alloc_test_flaky`), not regressions.

## Close-out (2026-07-05)

Tier-1's host/test/doc scope is **complete** — status flipped to `resolved`. The
three items outstanding at the last update all landed:

- **Item 8 — native/golden expansion — DONE.** rpc.cpp `dispatch()` + odrive /
  BB-throw byte xrefs (gaps 3/7/8, `6f20fea`); dead-bus golden column + DeferredStowLatch
  `first_seen` cold-start golden (gaps 5/10, `db4ddf8`). Guard-E-STOP + clock-step
  natives (gaps 2/6) landed **with Tier-2** as recommended — they pin the item-13
  E-STOP latch / item-14 monotonic clock, which now exist (see
  `2026-07-02-canhub-hardening-tier2.md`).
- **Row 11 residual — typed `CrcError` — DONE** (`2cf3a07`). A `CrcError(ValueError)`
  emitted at the `decode_frame` codegen source (`config/generate_udp_protocol.py`),
  regenerated, and caught before `ValueError` in `client.py`'s RX path — so
  `crc_errors` vs `decode_errors` no longer key off a fragile `"CRC" in str(e)`
  substring match. +2 codec / +1 client test.
- **Row 12 — documentation truth sweep — host/doc side DONE.** CAN2→CAN3 in the
  generator (`1634634`, pre-close-out); firmware `README.md` (status WIP→flashed+
  hardware-validated, archived-HANDOFF link, greiman→tsandmann lib table + "pinned in
  platformio.ini", `pio` build/flash) and native-harness `README.md` (the real 10-TU
  compile list, archived-plan future-tense, 500 Hz "still UNVALIDATED"→validated
  2026-07-04) refreshed. **Firmware-resident doc comments are deferred to the Phase-2
  firmware cycle**: dead task-table constants + the UV-flag note live in
  `canbridge_config.h` / `fault_machine.*` (owned by the concurrent CAN3 session), and
  the `LinkState::DEGRADED` marker fix would regenerate the delivered firmware header —
  all better landed alongside the item-20 firmware commit. `synthetic_setpoint.py`'s
  stroke-floor comment (0.0709 rev) was verified accurate against config
  (`STROKE_MIN_REV[0]` = 0.070917) — not stale, no change.

### Hardware confirmation (the three observable behaviour changes)

These change what `robot_state` reports / what the hand does, so they want an
operator eyeball (read-only observation — no new actuation beyond a normal sitting):

1. **[2] link-loss → fatal.** With the stack up and telemetry flowing, drop the
   Jetson↔Teensy UDP link (`ip link set <iface> down` on the teensy-link NIC, per
   memory `project_eth0_usb_dongle_unplug_weakness` — do NOT physically unplug the
   dongle). Confirm `robot_state.has_fatal_can_error` goes True and
   `robot_state.error` contains "Teensy link lost (UDP) …", and the orchestrator
   FAULTs. Restore the link → recovers.
2. **[6] cross-axis disarm.** During a powered hold, if one leg drops torque
   (disarms) while others hold CLOSED_LOOP, confirm `robot_state.has_fatal_odrive_error`
   goes True (previously it would have missed this). Opportunistic — observe if it
   occurs; do not force a disarm.
3. **[7] hand vel/curr limits.** Publish a `set_motor_vel_curr_limits` with non-zero
   `hand_vel_limit`/`hand_curr_limit` and confirm the hand ODrive (axis 6) accepts
   the update (and a subsequent `/configure` re-applies it, not the config default).

## Related

- Plan: `plans/active/canhub-hardening.md` (the 21-item tiered plan; Tier-1 tracked here).
- Fable-5 review run journal: `~/.claude/projects/-home-jetson-Desktop-Jugglebot/464186d7-ba4a-4f04-b124-0f6646ec869d/subagents/workflows/wf_f446b2a9-aae`.
- Predecessor: `2026-07-02-canbridge-phase4-orchestrator-wiring` (the parity plan this hardens).
