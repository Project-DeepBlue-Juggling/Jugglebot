---
title: Can-bridge foundation Phase 6 — reboot-in-progress watchdog latch + bus-transmittable (SYNCH) clear/reboot gate + hand in AXIS_ALL
type: feature
date: 2026-06-30
status: resolved
phase: "6"
related_plan: canbridge-foundation-coldstart-parity.md
related_entries:
  - 2026-06-29-canbridge-phase3-version-validated
  - 2026-06-29-canbridge-phase2-coldstart-relay-state
  - 2026-06-29-canbridge-phase1-platform-relay-seam
  - 2026-06-29-canbridge-phase0-native-harness
  - 2026-06-27-can-node-teensy-parity-audit
  - 2026-05-19-can-loss-fault-response-safety-inversion
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - controller/teensy_link/fault_logic.py
  - tests/firmware/native/test_fault_machine.cpp
  - tests/firmware/native/test_platform_relay.cpp
  - tests/firmware/native/fault_golden.json
  - tests/firmware/test_fault_logic.py
  - tests/firmware/test_rpc_dispatch_lint.py
  - tools/probes/canbridge_reboot_latch_probe.py
commits:
  - 3d390f4
  - TBD-logbook-plan-index
subsystem:
  - can
  - firmware
  - safety
  - testing
tags:
  - cold-start
  - fault-response
  - reboot
  - watchdog
  - deferred-stow
  - bus-health
  - parity
  - contract-gap
---

# Can-bridge foundation Phase 6 — robust clear_errors + reboot-in-progress latch

## Summary

Phase 6 (the **final** phase of `canbridge-foundation-coldstart-parity.md`) closes
the two `clear_errors`/`reboot` HIGH gaps from the 2026-06-27 parity audit, both of
which touch the safety-critical fault state machine:

1. **Reboot-in-progress watchdog-suppression latch** (fault machine). A
   `REBOOT_ODRIVES` RPC now arms `fault_notify_reboot_started()`, which suppresses
   the CAN3 CAN-loss detector for a bounded window so a deliberate ODrive-reboot's
   heartbeat silence is not read as a real CAN loss (which would falsely arm the
   2026-05-19 deferred stow on reconnect). Armed **only** by the reboot RPC, so a
   spontaneous CAN loss is completely unaffected — the deferred-stow inversion is
   preserved exactly. This **completes the `REBOOT_ODRIVES` shared hook** (plan
   architecture: "complete only after Phases 2 + 6"; Phase 2 landed the host
   `STATE_WRITE` `is_homed`/level/pose clear = step 2; Phase 6 adds the firmware
   watchdog-suppression latch = step 1).
2. **Bus-transmittable (ESR1.SYNCH) gate** for the operator recovery one-shots
   `CLEAR_ERRORS`/`REBOOT_ODRIVES`. These now gate on the *live* electrical
   bus-lock (`jugglebot_bus_transmittable()` = `s_jugglebot_rxh.synced`) instead of
   heartbeat-staleness (`jugglebot_commands_allowed()`), so a recovery clear reaches
   a just-repowered bus that is electrically alive but has not heartbeated yet (the
   audit's "carve clear/reboot out of the bus-health gate" recommendation). Every
   other op keeps the staleness gate.
3. **Hand (axis 6) in the AXIS_ALL loops.** The `CLEAR_ERRORS`/`REBOOT_ODRIVES`
   `AXIS_ALL` loops now iterate `i < NUM_AXES` (legs + hand), matching `can_node`'s
   `JUGGLEBOT_AXES` (audit rows 21/40); the old `i < NUM_LEGS` silently dropped the
   hand.

The fault-machine change lands **three-way in lockstep**: `fault_machine.cpp` +
`controller/teensy_link/fault_logic.py` (`DeferredStowLatch` reboot port) + the
native harness (`tests/firmware/native/test_fault_machine.cpp` + the deliberately
regenerated `fault_golden.json` replayed by `tests/firmware/test_fault_logic.py`).

## Motivation

The 2026-06-27 parity audit flagged two known HIGHs
(`logbook/2026-06-27-can-node-teensy-parity-audit.md` §§2–3):

- **Robust `clear_errors`** — `CLEAR_ERRORS`/`REBOOT` return `ERR_BUS_DOWN` when
  `health_of` reads WARN from heartbeat staleness; a *just-repowered* bus (RX
  timestamp still stale before the first decode) then blocks the very clear needed
  for recovery (the 2026-06-27 stale-UV-trips-`/activate` incident). `can_node`
  never gated the clear.
- **`reboot_odrives` dropped two side-effects** — no watchdog/heartbeat suppression
  (so a reboot's heartbeat silence falsely trips the CAN-loss deferred-stow
  machinery) and no `is_homed` invalidation. `can_node._reboot_odrives_steps`
  suppressed heartbeat tracking during the reboot window precisely so the watchdog
  would not read the deliberate silence as a CAN loss (`can_node.py:1556`); the
  bridge reboot path dropped that. (The `is_homed` invalidation is Phase 2's
  `STATE_WRITE` clear — already landed.)

## Design

### Reboot-in-progress latch (`fault_machine.cpp`)

Three file-statics: `s_reboot_in_progress` (armed), `s_reboot_deadline_us` (64-bit,
accessed via `atomic_read/write_u64` — the `can_buses.cpp:577` tear hazard),
`s_reboot_saw_stale` (the legs went stale since arming — the release gate).

- **Arm** (`fault_notify_reboot_started()`, called from the `REBOOT_ODRIVES`
  dispatch **after** the bus gate passes so a refused reboot never blinds the
  detector): set the deadline `= now + REBOOT_WATCHDOG_SUPPRESS_US`, clear
  `saw_stale`, publish `s_reboot_in_progress = true` **last**.
- **Release** (in `watchdog_and_stow`, BEFORE the detection so the AND sees the
  released state): `(saw_stale && all_present_legs_fresh()) || now >= deadline`.
  The common case is fresh-after-stale (the ODrives came back — far tighter than
  the deadline); the deadline is the failure-case backstop.
- **Detection** (`fault_machine.cpp:218`) ANDs `!s_reboot_in_progress`.
- **`REBOOT_WATCHDOG_SUPPRESS_US = 6.0 s`** (`canbridge_config.h`), sized to the
  *measured* reboot latency (below) + margin — **not** the untuned ~10 s `can_node`
  assumed.

The `saw_stale` gate is essential: a latch armed while the legs are still fresh
(the reboot frames have not yet silenced them) must not release on that same
freshness — only a fresh reading *after* the legs have actually gone stale proves
the reboot completed. Without it, the latch would release one tick after arming and
re-expose the very false loss it exists to suppress.

### Bus-transmittable gate (`can_buses.cpp` + `rpc.cpp`)

`jugglebot_bus_transmittable()` returns the *live, non-sticky* `ESR1.SYNCH` bit
(`s_jugglebot_rxh.synced`, maintained every service tick in `service_bus`). A single
gate chokepoint `gate_allows(method)` in `rpc.cpp` maps the method → gate basis via
the header-inline `method_gates_on_bus_transmittable()` policy predicate (`rpc.h`):
`CLEAR_ERRORS`/`REBOOT_ODRIVES` → `jugglebot_bus_transmittable()`; everything else →
`jugglebot_commands_allowed()`. Both `send_axis_frame()` (per-axis) and the
`AXIS_ALL` loops route through `gate_allows`, so there is exactly one enforcement
point.

### Hand in AXIS_ALL

The `AXIS_ALL` clear/reboot loops iterate `i < NUM_AXES` (7 = legs + hand);
`encode_clear_errors(6)`/`encode_reboot(6)` build the hand node frame, and the
Phase-1 allow-table already permits `CLEAR_ERRORS`/`REBOOT_ODRIVES` on axis 6.

## Implementation

Three-way + gate, all landed together (see files_changed). Firmware: the latch in
`fault_machine.{h,cpp}` + the `REBOOT_WATCHDOG_SUPPRESS_US` constant in
`canbridge_config.h`; `jugglebot_bus_transmittable()` in `can_buses.{h,cpp}`;
`gate_allows` + the two AXIS_ALL cases + latch-arm in `rpc.cpp`; the policy
predicate in `rpc.h`. Host mirror: `DeferredStowLatch` gains `reboot_in_progress` +
`notify_reboot_started()` + the `step(..., reboot_start, deadline_pass)` release
logic (the clock abstracted the same way the latch already abstracts it into
`stale`/`fresh`). Tests: the native harness gains 3 reboot golden scenarios (driven
through the compiled `fault_step()`), 3 explicit reboot `TEST_CASE`s, and the gate
policy predicate test in `test_platform_relay.cpp`; the golden was regenerated
deliberately (`build.py --golden`) and `test_fault_logic.py` replays the new fields;
`test_rpc_dispatch_lint.py` gains three source-introspection wiring guards
(clear/reboot gate on SYNCH not staleness; AXIS_ALL includes the hand; reboot arms
the latch, clear does not).

No codegen change (no new wire ids — the gate is firmware-only; the policy predicate
is header-inline). No host `teensy_bridge_node` change (the latch is armed
firmware-side in the `REBOOT_ODRIVES` dispatch; the host still just sends the RPC).

## Verification

Per the CLAUDE.md (date, command, result) triple rule:

- **Baseline** (2026-06-30): `git HEAD = cec27df`; `pytest tests/ -q` → **1920
  passed, 1 failed, 1 xfailed in 457.63 s** — the 1 failure is the documented
  order-flaky `test_hot_loop_allocation_contract`, confirmed
  (`pytest tests/sim/test_hot_loop_allocation_contract.py -q`, 2026-06-30) **3
  passed in 15.98 s** isolated (`project_hot_loop_alloc_test_flaky`); `pio run`
  **SUCCESS, 5.06 s**.
- **Native harness** (`temp/firmware_native/test_fault_machine`, 2026-06-30):
  **9 cases / 136 assertions pass** (was 6/98 — the 3 reboot cases + the reboot
  golden scenarios). `test_platform_relay`: **6 cases / 57 assertions pass** (was
  5/46 — the gate policy predicate). `pytest tests/firmware/ tests/teensy_link/ -q`
  (2026-06-30) **282 passed in 4.99 s** (golden conformance + golden honesty + the
  new lint guards).
- **Divergence-catch proven** (2026-06-30): stripping `&& !s_reboot_in_progress`
  from the detection failed **exactly** the 3 reboot `TEST_CASE`s (11 assertions);
  restoring → 9/9, 136/136. So the new tests genuinely exercise the suppression
  logic, not a tautology.
- **Full suite** (`pytest tests/ -q`, 2026-07-01): **1927 passed, 1 xfailed in
  452.99 s** — fully green. The pass count is **+7** vs the 1920-pass baseline: **6
  new tests** (3 `test_rpc_dispatch_lint` wiring guards + 3 reboot golden-conformance
  params in `test_fault_logic.py`) **plus** the documented order-flaky
  `test_hot_loop_allocation_contract` flipping green this run (it failed the baseline
  run, passes isolated). Collected tests: +6. (Re-run after the audit-fix adds one
  more `test_rpc_dispatch_lint` assertion — same test count.)
- **Firmware** (`pio run`, 2026-06-30): **SUCCESS, 5.27 s**. Codegen deterministic
  (regenerate → no diff).
- **Bench probes** (2026-06-30, `tools/probes/canbridge_reboot_latch_probe.py`,
  motor power OFF, Phase-3 firmware flashed, `REBOOT_ODRIVES` operator-authorised):
  - **Reboot→first-leg-heartbeat latency** (= the latch release condition, read over
    UDP as the `CAN_BUS_DOWN` clear time): **2.57 / 3.54 / 2.27 s** across 3 runs →
    `REBOOT_WATCHDOG_SUPPRESS_US = 6 s` (measured worst 3.5 s + ~2.5 s margin).
  - **False CAN-loss confirmed** (Phase-3 firmware): the reboot trips `CAN_BUS_DOWN`
    at onset ~2.0–2.3 s — so the latch is genuinely needed.
  - **Deadlock premise did NOT reproduce**: with all 7 ODrives (legs + hand)
    rebooted and silent, `bus1`(CAN3) `health` **never went WARN** and every
    `CLEAR_ERRORS` returned **OK** — the Platform Teensy keeps CAN3 RX fresh (see
    Discussion).
- **"After" probe — Phase-6 firmware flashed** (2026-07-01, `pio run -t upload`
  soft-reboot flash SUCCESS, then the same probe): a controlled before/after with
  identical hardware — the only variable changed is the firmware (the latch).
  - **Latch suppresses the false loss ON HARDWARE**: `CAN_BUS_DOWN` was **never
    observed** during the reboot window (vs the Phase-3 onset at ~2.0–2.3 s in all
    3 "before" runs). The ODrive reboot time is a hardware constant (the legs were
    silent 2.3–3.5 s, same as before), so the watchdog would have tripped at 2 s
    were it not suppressed — the latch is doing it.
  - **SYNCH register read confirmed correct (PROBE 1, end-to-end)**: every
    `CLEAR_ERRORS`/`REBOOT_ODRIVES` returned **OK**, so `gate_allows()` →
    `jugglebot_bus_transmittable()` → `s_jugglebot_rxh.synced` returned **true** on
    the alive bus. Had the ESR1 read returned garbage/0, the gate would have refused
    with `ERR_BUS_DOWN` — the RPC status is a direct observable of the register read
    (no serial console needed). The gate's SYNCH=0 *refusal* path (a truly dead bus)
    is unforced here (needs a full CAN3 teardown) — see Open Questions.

## Discussion

CLAUDE.md makes the Discussion non-negotiable here on all three counts: a hypothesis
was withdrawn mid-investigation, a non-obvious tradeoff was accepted, and the chosen
approach beat reasonable alternatives for reasons not inferable from the code.

### 1. The deadlock premise did not survive the bench probe — and why the gate change still ships

The plan's stated root cause for the gate change was: *"a motor-bus power cycle
leaves the bus SYNCED (the Platform Teensy still heartbeats), so the recovery clear
must be allowed — but the current staleness gate REFUSES it."* The probe **falsified
the "refuses it" half** on the current bench: I rebooted **all 7** ODrives (legs +
hand) so the entire core bus went silent, and `bus1`(CAN3) `= jugglebot_health`
**never went WARN** — every `CLEAR_ERRORS` returned OK. The reason: the **Platform
Teensy keeps CAN3 RX fresh** on its own (it transmits frequently enough that
`s_jugglebot_last_rx_us` never crosses the 2 s timeout), so `health_of` stays OK
even with every ODrive silent. Two staleness signals that I had conflated turn out
to be independent: the **watchdog** uses per-leg `any_leg_heartbeat_stale()` (which
*does* trip on a reboot → the false `CAN_BUS_DOWN` is real), while the **command
gate** uses bus-level RX (`s_jugglebot_last_rx_us`, which the Platform Teensy keeps
fresh).

Per "push back when evidence contradicts a hypothesis," I abandoned the
"reproduced-deadlock" framing. Reading the audit, the *actual* deadlock is the
**just-repowered-bus transient**: after a full CAN3 power-cycle, `health_of` reads
WARN from the **stale RX timestamp before the first heartbeat is decoded**, blocking
the recovery clear. My ODrive-only reboot can't reach that (the Platform Teensy
never dropped); reproducing it needs a full CAN3 power-cycle (invasive — not done
this session). The SYNCH gate is nonetheless the correct fix for that latent
transient — `ESR1.SYNCH → 1` the instant the bus is electrically alive, *before* any
frame decodes, so the gate allows the recovery clear exactly when staleness would
still refuse it. It is also the audit's own recommendation ("carve clear/reboot out
of the bus-health gate", the `axis_state.h:84` "operator one-shots stay ungated"
principle), it does not regress current behaviour (SYNCH stays 1 during a partial
outage, same as today), and it is strictly safer than the alternatives (below). The
operator confirmed shipping it with this honest framing (design-intent + latent
transient), not as a bench-reproduced deadlock fix.

### 2. Why SYNCH over blanket-ungate, and why NOT fault_conf

- **vs blanket-ungate** (`can_node`'s behaviour — never gated the clear). Blanket
  fixes the deadlock but loses the "never blind-clear a truly dead bus" guard: a
  ≤N-frame clear/reboot to a bus with no ACKer climbs the FlexCAN TEC toward bus-off
  (the same failure the cone-absent gate prevents on CAN2). SYNCH keeps that guard
  (SYNCH=0 on a bus-off / unsynced bus still refuses) while allowing the recovery
  case — strictly safer on both axes, *provided* the register read is sound.
- **vs `fault_conf` (ESR1.FLTCONF)**. The plan floated "and/or `fault_conf < 2`".
  Rejected: the recorded `fault_conf` in `BusRxHealth` is **sticky-max** (only ever
  climbs, never resets until reboot — `can_buses.cpp:443`), so gating on it would
  *permanently* refuse clear/reboot after the first bus-off event, defeating the
  recovery it is meant to enable. A node in bus-off has SYNCH=0 anyway, so the live
  SYNCH bit already subsumes the "not bus-off" check for a *live* gate. SYNCH-only.

### 3. Window tightness = blind-spot, softened by the fresh-after-stale release

The window is the blind-spot for a real CAN loss that *coincides* with a deliberate
reboot: during it, detection is suppressed. Two things bound the cost. (a) The
**fresh-after-stale release** means a *normal* reboot releases the latch the instant
the legs return (~2.3–3.5 s measured), regardless of the window — so the window only
bounds the *failure* case (legs never return). (b) During that failure-case window
the ODrives **autonomously hold** their last setpoint (the 2026-05-19 inversion), so
the blind-spot delays the eventual *safe stow*, it never collapses the platform.
That safety asymmetry (a too-tight window causes a benign spurious stow on a slow
reboot; a too-loose window merely delays a safe stow) argued for a *slightly*
generous 6 s over the measured 3.5 s — but far under `can_node`'s untuned ~10 s and
the plan-rejected ~15 s. The whole point of PROBE 2 was to replace the untuned
assumption with the measured value.

### 4. How the deferred-stow inversion is preserved (proven, not asserted)

The latch is armed **only** by `fault_notify_reboot_started()` from the
`REBOOT_ODRIVES` dispatch. A spontaneous CAN loss never calls it, so
`s_reboot_in_progress` stays false and the detection AND (`&& !s_reboot_in_progress`)
is a no-op — detection fires exactly as in the 2026-05-19 design. This is proven by
the native `TEST_CASE("reboot latch: a SPONTANEOUS CAN loss is UNAFFECTED ...")`
(drives a loss without arming the latch, asserts `fault_can_bus_down()` +
`fault_stow_pending()` + `fault_state()==CAN_BUS_DOWN`) and the
`reboot_latch_does_not_touch_spontaneous_loss` golden scenario. The latch also never
touches an *already-armed* stow (it gates only new detection), so a spontaneous loss
that armed the stow *before* a reboot still executes on reconnect.

### 5. Test seam: header predicate + isolation test (the axis-6 precedent), not a behavioural dispatch include

The dispatch-gate mirror is realised as (a) a **header-inline pure predicate**
`method_gates_on_bus_transmittable()` pinned directly by the native harness — exactly
how the analogous `hand_axis6_permitted` / `is_platform_reply_id` predicates are
tested — plus (b) **source-introspection lints** in `test_rpc_dispatch_lint.py` that
assert the `CLEAR_ERRORS`/`REBOOT_ODRIVES` cases route through `gate_allows` (never
`jugglebot_commands_allowed` directly), loop `i < NUM_AXES`, and arm the latch. The
alternative — a native test that `#include`s `rpc.cpp` and drives `dispatch()`
behaviourally — was rejected: it needs ~15 leaf-symbol fakes (Relay / version /
homing / activate / deactivate / BB / udp), a real maintenance/divergence surface,
for coverage the axis-6 gate has never had (its wiring is verified by review + the
bench probe, same as here). Operator-confirmed.

### 6. Citation-drift correction (Phase-3 audit precedent)

The plan's Phase-6 line citations had drifted (confirmed against ground truth before
coding): ESR1.SYNCH is `can_buses.cpp:462-466` (not 382-407); FLTCONF→`fault_conf`
is `:437-443`; the staleness-≠-bus-down TODO is `:567` (not `:500`); the atomic
64-bit helpers are `time_base.h:33-43` (the `:510-516` block is TX serialisation).
The watchdog-detection `:178` and the AXIS_ALL `i < NUM_LEGS` citations were correct.

## Open Questions

- **Full-CAN3-repower deadlock reproduction.** The staleness-refuses-clear deadlock
  needs a full CAN3 power-cycle (Platform Teensy included) to manifest on the bench;
  this session validated the reboot **latch** on hardware but not the gate
  **deadlock** (Discussion §1). The SYNCH gate is a latent-transient / design-intent
  fix; a full-repower bench test would confirm it end-to-end.
- **SYNCH gate — the REFUSAL path is unforced.** The *allow* path is confirmed on
  hardware (the "after" probe: `CLEAR_ERRORS`/`REBOOT` returned OK → the SYNCH read
  returns true on the alive bus, PROBE 1 end-to-end). The *refusal* path (SYNCH=0 on
  a genuinely dead/bus-off CAN3 → `ERR_BUS_DOWN`) is not forced here (it needs a full
  CAN3 teardown); it inherits confidence from the same `ESR1` access the
  Phase-1-validated `fault_conf`/`tec_max` diagnostics use. A full-CAN3-teardown test
  would exercise it (and the §1 deadlock) together.
- **Cross-task `saw_stale`/`deadline` concurrency.** `fault_notify_reboot_started`
  (UDP-RX task) and `watchdog_and_stow` (fault task) race on `s_reboot_saw_stale`
  and `s_reboot_deadline_us`; the deadline is atomic and the double-reboot race on
  `saw_stale` is benign (a re-reboot re-silences the legs → no spurious early
  release). This is the documented "harness validates decision logic, not
  FreeRTOS/ISR concurrency" gap.

## Related

- [canbridge-foundation-coldstart-parity.md](../plans/active/canbridge-foundation-coldstart-parity.md)
  — the plan; Phase 6 is the final phase (4 still needs 5; 6 does not gate 4/5).
- [2026-06-27-can-node-teensy-parity-audit.md](2026-06-27-can-node-teensy-parity-audit.md)
  — the two HIGHs this phase closes (§§2–3).
- [2026-05-19-can-loss-fault-response-safety-inversion.md](2026-05-19-can-loss-fault-response-safety-inversion.md)
  — the deferred-stow inversion the reboot latch must not (and does not) regress.
- [2026-06-29-canbridge-phase0-native-harness.md](2026-06-29-canbridge-phase0-native-harness.md)
  — the compiled harness + golden-conformance pattern reused here.
- `tools/probes/canbridge_reboot_latch_probe.py` — the reusable reboot-latency +
  gate-condition bench probe; outputs to `temp/probes/`.
