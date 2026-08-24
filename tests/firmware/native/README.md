# Native firmware test harness

Compiles the **real** safety-critical can-bridge firmware on the build host and
tests the **binary**, so a C++ divergence from the safety logic fails
`pytest tests/ -q` — instead of passing silently because only a hand-maintained
Python transcription was checked (the standing risk flagged in
`logbook/2026-06-27-can-node-teensy-parity-audit.md` §5).

It compiles the safety-critical firmware TUs directly — `fault_machine.cpp`,
`leg_interp.cpp`, `platform_relay.cpp`, `version_check.cpp`, `hand_ops.cpp`,
`rpc.cpp` dispatch, `udp_link.cpp`, and the three cold-start move modules
(`leg_homing/activate/deactivate.cpp`) — plus the generated C++ UDP framing codec
and the ODrive / Ball-Butler CAN codecs, and drives them via the HAL seam below
(including the inbound-CAN3 injection hook in `fake_hal`). The relay / version /
cold-start paths that once lived on a `canbridge-foundation-coldstart-parity` TODO
here are now compiled (that plan is archived); see "Build recipe" for the
per-binary TU map.

## Run it

It is wired into the normal suite — no separate command:

```bash
pytest tests/firmware/test_native_firmware.py -q     # build + run + golden check
./run_tests.sh                                       # the full gate (incl. the above)
```

`pytest tests/firmware/test_fault_logic.py -q` runs the golden conformance for the
Jetson host mirror and needs **no compiler** (it replays the committed golden).

Build/run the binaries directly (handy when iterating):

```bash
python tests/firmware/native/build.py            # build both (hash-cached)
./temp/firmware_native/test_fault_machine        # doctest output
./temp/firmware_native/test_leg_interp
python tests/firmware/native/build.py --force    # ignore the cache, rebuild
```

## Build recipe

* Compiler: `g++ -std=c++17 -O2 -Wall -Wno-unused-function` (exceptions/RTTI
  enabled for doctest; the firmware TUs neither throw nor use RTTI, so this does
  not change their behaviour). Fixed flags ⇒ reproducible (same sources → same
  binary).
* Includes: `-I hal_shims -I . -I <Teensy_code_canbridge>`.
* **No `.cpp` is ever both `#include`d and linked as an object in the same
  binary** (the rule that avoids ODR clashes):
  * `test_fault_machine.cpp` `#include`s **both** `fault_machine.cpp` and
    `leg_interp.cpp` — the two share no symbol names, so one TU is ODR-clean — so
    it can drive the `static interp_isr()` to complete a stow and assert the
    fault machine's terminal-IDLE handling (deferred-stow invariant 5) as a
    *compiled* assertion.
  * `test_leg_interp.cpp` `#include`s `leg_interp.cpp` only.
  * `test_platform_relay.cpp` / `test_version_check.cpp` / `test_hand_ops.cpp`
    `#include` their one module (`platform_relay.cpp` / `version_check.cpp` /
    `hand_ops.cpp`).
  * `test_gpio_poll.cpp` `#include`s **both** `gpio_poll.cpp` and
    `version_check.cpp` (same no-shared-symbols argument as `test_fault_machine`):
    the ball-sensor poller's `Get_Version` gate reads the version cache, and
    `version_record()` is the only honest way to seed it.
  * `test_leg_activate.cpp` / `test_leg_deactivate.cpp` / `test_leg_homing.cpp`
    each `#include` ONE cold-start module `.cpp` (defining its own `*_active()`) and
    supply the two SIBLING predicates inline — so they link `coldstart_hal.o`
    (NOT `fake_hal.o`, which would ODR-clash on the module's own `*_active()` and
    lacks the `can_buses_stats()`/`fault_*` gate these modules read). This closes
    the standing gap that the code driving legs into hardstops
    (`leg_homing/activate/deactivate.cpp`) was **never compiled by any test**
    (Fable-5 hardening [6]).
  * `test_rpc_dispatch.cpp` `#include`s `rpc.cpp` (the `(method,axis)` enforcement
    point); `test_udp_link.cpp` `#include`s `udp_link.cpp` behind a fake QNEthernet /
    recursive-mutex shim. `test_udp_framing.cpp`, `test_odrive_protocol.cpp`, and
    `test_ball_butler_protocol.cpp` compile the generated C++ framing codec and the
    ODrive / Ball-Butler CAN codecs against their headers (no extra firmware `.cpp`
    object). `build.py` builds 15 binaries in all.
  * Shared objects compiled once and linked as needed: `axis_state.o`,
    `ball_butler_state.o`, and either `fake_hal.o` (fault/interp/relay/version/hand)
    or `coldstart_hal.o` (the three cold-start move drivers).
* Hash-cached: `build.py` stamps a SHA over every harness file + every firmware
  `.cpp/.h/.ino` + the flags + the g++ version. Unchanged sources ⇒ zero
  recompile. Artifacts live under `temp/firmware_native/` (gitignored).
* g++-gated: `build.have_gpp()` lets the pytest wrapper **skip** (not fail) on a
  host without a compiler. The Jetson run is authoritative.

## The HAL seam (`fake_hal.cpp` / `fake_hal.h`)

The compiled TUs reach a few leaf symbols from other translation units we do not
compile on the host; `fake_hal.cpp` defines them with a controllable clock and a
**recording** CAN3 TX, and the two `hal_shims/` headers stand in for the Teensy
core:

| Symbol the firmware needs | Real owner (off-target) | Host stand-in |
|---|---|---|
| `now_wall_us()`, `micros64()` | `time_base.cpp` | controllable clock (`fake_set_clock`/`fake_advance`) |
| `udp_last_rx_us()` | `udp_link.cpp` | settable (`fake_set_udp_last_rx_us`) |
| `homing_active()` / `activate_active()` / `deactivate_active()` | `leg_homing/activate/deactivate.cpp` | settable predicates |
| `can_jugglebot_send()` | `can_buses.cpp` | **recording** vector (`fake_sent_*`) — the never-command-a-dead-bus check reads this |
| `axes[]`, `bb_state` | `axis_state.cpp`, `ball_butler_state.cpp` | the **real** objects (those `.cpp` are compiled + linked) |
| `IntervalTimer`, PRIMASK intrinsics, `micros/millis` | Teensy core `<Arduino.h>` / freertos port | `hal_shims/Arduino.h` + `arduino_freertos.h` |
| (growable) inbound CAN3 frame | `can_buses.cpp` RX decode | `fake_inject_can3_rx()` FIFO — unused in Phase 0; for Phase 1/3 reply-correlation |

Reset between cases: `fake_reset()` + `fault_machine_init()` + `interp_reset()`
clear all file-scope state so test ordering can neither fabricate nor mask a
result.

### The cold-start HAL seam (`coldstart_hal.cpp` / `coldstart_hal.h`)

The three cold-start move modules read a DIFFERENT leaf set than the fault/interp
TUs — their bus/fault gate is `jugglebot_commands_allowed()` +
`fault_can_bus_down()` + `fault_guard_mode()`, and each DEFINES its own `*_active()` (which would ODR-clash with `fake_hal.o`'s
fake predicates). So they get a SEPARATE, self-contained fake HAL, linked ONLY by
the cold-start drivers, never alongside `fake_hal.o`:

| Symbol | Host stand-in (`coldstart_hal`) |
|---|---|
| `micros64()` / `now_wall_us()` | controllable clock (`cs_advance`/`cs_set_mono`) |
| `can_buses_stats().jugglebot_health` | settable (`cs_set_jugglebot_health`) — ALSO moves the command gate to the matching verdict, see below |
| `jugglebot_commands_allowed()` | settable (`cs_set_commands_allowed`); defaults to follow `cs_set_jugglebot_health` |
| `fault_can_bus_down()` / `fault_guard_mode()` | settable (`cs_set_can_bus_down` / `cs_set_guard_estop`) |
| `can_jugglebot_send()` | **recording** vector (`cs_sent_*`) + fail injection (`cs_set_send_fail_index`) |
| the two SIBLING `*_active()` | inline in each driver (the real one comes from the `#include`d module) |

Reset between cases: `cs_reset()` + the module's `*_init()`.

`cs_set_jugglebot_health()` deliberately moves BOTH knobs (health WARN/BUS_OFF ⇒
commands refused). Before 2026-07-29 the three gates re-derived the bus term from
the health enum, so tests drove them with that setter alone; they now call
`jugglebot_commands_allowed()`, and without the coupling those cases would pass
against a gate nobody set. The coupling is a **default, not an invariant** —
production runs two classifiers that agree on every cell except
passive-but-not-yet-sustained (health reports WARN, the gate still allows), and a
test modelling that cell calls `cs_set_commands_allowed()` *after* the health
setter to force the divergence.

## Golden vectors (`fault_golden.json`)

`test_fault_machine --emit-golden <path>` dumps the firmware-anchored conformance
vectors (the subset of scenarios the Jetson mirror `teensy_link/
fault_logic.py` models). Regenerate after an *intended* fault-machine change:

```bash
python tests/firmware/native/build.py --golden tests/firmware/native/fault_golden.json
```

Two guards keep the three layers in lockstep (the coupling the manual
transcription used to maintain by hand):

* `test_native_firmware.py::test_committed_golden_matches_live_firmware` — a fresh
  emission must equal the committed golden (firmware drift ⇒ fail, on the Jetson).
* `test_fault_logic.py` — `fault_logic.py` must reproduce the committed golden
  (host-mirror drift ⇒ fail, everywhere — no compiler needed).

## Scope (read this before trusting a green run)

The harness validates **decision logic only** — single-threaded, on a fake clock.
It does **not** cover, and these remain on-hardware-replay gaps:

* FreeRTOS/ISR **concurrency** (the deferred-stow re-arm race, the PRIMASK atomic
  publish, ISR priority vs the syscall ceiling);
* the **500 Hz deadline** / interp jitter — single-threaded on a fake clock here,
  so not exercised by this native harness. On hardware the interp `deadline_misses`
  / `max_jitter` counters read clean through the 2026-07-04 Tier-2 flood +
  deferred-stow checks (`logbook/2026-07-02-canhub-hardening-tier2.md`, checks 3 &
  5), and a dedicated automated PASS/ABORT soak gate over the PROFILE frame lives at
  `tools/probes/archived/canhub_500hz_deadline_gate.py` (item 19; archived 2026-08-15, arc closed);
* float32-vs-float64 numerical residue — host float is true IEEE-32 (closer to the
  Teensy FPU than the float64 Python mirror), and these tests assert **behaviour**
  (clamps fired, modes transitioned, descent converged), not bit-exact equality.
  The float64 numerical xref stays in `tests/firmware/test_hermite_xref.py`.

## Vendored dependency

`doctest.h` — doctest v2.4.11, single-header C++ test framework, MIT licensed
(© 2016-2023 Viktor Kirilov, <https://github.com/doctest/doctest>). Vendored
verbatim; do not hand-edit.
