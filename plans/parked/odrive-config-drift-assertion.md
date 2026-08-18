---
title: ODrive config-drift assertion — an automatic control for NVM drift
created: 2026-08-18
status: parked   # 2026-08-18 — the assumed transport does not exist. A CAN/SDO read
                 # has NO response path to the Jetson (rpc.cpp:249-257,
                 # can_buses.cpp:248-262) and the hand is refused SDO_READ outright
                 # (udp_protocol.h:593-608), so nothing can be built as specified
                 # without a can-bridge firmware change + flash, which is unauthorised.
                 # USB is ruled out on evidence (drives are not on USB at run time),
                 # so CAN3 is the only permanent path and firmware is the only route
                 # to an AUTOMATIC control. Parked on ONE owner decision: is automatic
                 # drift detection worth a firmware chapter + flash, or is the manual
                 # Option C tool enough? Resume at Phase 0.
related_logbook:
  - 2026-08-10-hand-drive-braking-clamp-diagnosis.md
  - 2026-08-18-hand-torque-clamp-removed.md
related_config:
  - config/ODrive config Files/odrive_pro_leg_config.json
  - config/ODrive config Files/odrive_pro_hand_config.json
  - config/protocol_config.yaml → endpoints.odrive_pro_0_6_11
related_code:
  - tools/odrive_fleet_reflash.py::flatten_config
  - tools/odrive_fleet_reflash.py::diff_flat
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py::_version_check_poll
---

# ODrive config-drift assertion

## Context

### The failure this exists to close

On 2026-08-18 the hand ODrive's `axis0.config.torque_soft_min` was found sitting
at `-0.055133331567049026` N·m. At the hand's `torque_constant` of
`0.00551333324983716` that is exactly **−10.00 A**, asymmetric against a
`torque_soft_max` of `+0.5`. It truncated the FW-2 post-throw decel feedforward
on every throw above ~0.38 m and produced a post-throw dip on 54/54 measured
throws, costing a multi-day diagnosis arc.

The value's provenance is now confirmed against the snapshots rather than
inferred: `config/ODrive config Files/odrive_pro_leg_config.json` declares
`axis0.config.motor.torque_constant = 0.055133331567049026` — the **legs'**
`torque_constant`, bit-identical, ten times the hand's `Kt`. A cross-drive paste
into a clamp field is the only reading that fits.

### Why a contract and not a fix

Three properties made the drift survivable for weeks, and each is a property of
a *class*, not of this instance:

1. **Config drift is recurring on this machine.** `tools/odrive_fleet_reflash.py`
   exists because "this machine's ODrives occasionally need a config reset (cause
   unknown — one cured the 2026-08-10 leg-0 `SPINOUT_DETECTED`)". A reset reverts
   every tuned value, so divergence between drive NVM and the committed snapshots
   is an expected recurring event.
2. **The one drive that drifted is the one drive nothing verified.** That tool's
   step 5 verify covers the six legs; its docstring states the hand "is
   discovered, recorded, and then left strictly alone".
3. **A runbook step is not a control.** Bench row **H7.0c** in
   `tests/hardware/session_anomaly_fixes.md` specifies exactly this check on
   exactly this register, names `-0.0551` as the abort value, and says it takes
   30 s. It went unrun for weeks.

Point 3 is the reason the deliverable must be automatic. Adding a second manual
probe would reproduce the failure it is meant to prevent.

### THE BLOCKING FINDING — the assumed transport does not exist

The design as originally scoped reads the registers over CAN/SDO through the
existing `teensy_bridge_node.teensy_sdo_read` → `RpcMethod.SDO_READ` path. **That
path cannot return a value to the Jetson, and cannot address the hand at all.**
Two independent blockers, both firmware-resident, both pinned by existing tests:

**B1 — an SDO read has no response path to the Jetson.**
`ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp:249-257` dispatches
`SDO_READ` by sending the `RxSdo` frame and returning `send_axis_frame`'s
`RpcStatus`; `res_len` stays 0 from the top of `dispatch`. The comment states it
outright: *"The TxSdo reply has NO return path to this RPC — nothing correlates
it back to the caller."* The reply arrives on CAN and is decoded at
`can_buses.cpp:248-262`, where the `ODriveCmd::TxSdo` case consumes it **only**
when `axis == HAND_AXIS` **and** `endpoint_id ==
EndpointId::odrive_pro_0_6_11::get_gpio_states`, handing it to `gpio_poll_record`.
Every other TxSdo reply is dropped. No uplink `MsgType` carries an arbitrary SDO
value. `teensy_link/rpc_args.py:136` already says "response returns async on
TxSdo" — async to nowhere the Jetson can observe.

**B2 — the hand is refused `SDO_READ` before the frame leaves the Teensy.**
`udp_protocol.h:593-608`'s `hand_axis6_permitted` allow-table omits `SDO_READ`,
so `rpc.cpp:111-119`'s `send_axis_frame` returns `ERR_REJECTED` for axis 6. This
is pinned in three places — `tests/firmware/native/test_rpc_dispatch.cpp:187-190`
("SDO_READ is NOT in the hand allow-table → ERR_REJECTED, nothing sent"),
`tests/firmware/native/test_platform_relay.cpp:284`, and
`tests/firmware/test_hand_axis6_allow.py:52`. For the one drive that actually
drifted, the request does not reach the bus.

No Jetson-side change works around either. Both live in can-bridge firmware.

### Why "write the firmware but do not flash it" is not a safe half-step

`tests/firmware/test_bridge_fw_version_xref.py:76-83` pins
`canbridge_config.h::FW_VERSION` equal to `rpc_args.EXPECTED_BRIDGE_FW_VERSION`,
and `teensy_bridge_node` runs an install-skew self-check against the
`BRIDGE_IDENTITY` frame. Writing firmware without flashing it therefore arms a
live skew alarm against the running FW 14 board. `clapboard-can3-integration.md`
already carries exactly this hazard on another branch (FW 15 in source, never
flashed, `EXPECTED_BRIDGE_FW_VERSION` bumped to 15), and a second instance of it
on `mvp-trajectory-bringup` would collide with that one.

### The wrong-endpoint hazard is a CAN/SDO artifact, not a universal one

ODrive endpoint IDs are firmware-build-specific and a wrong id **answers
plausibly** — `version_check.h:21-27` records that endpoint 700 on a Pro 0.6.11
returns `encoder_estimator1.status`, "a live-looking sensor that never changes,
with no timeout to diagnose it". `config/protocol_config.yaml`'s `endpoints:`
block is already keyed by `(board, fw)` for exactly this reason and carries the
warning "A consumer MUST verify Get_Version before use". It presently holds
**one** id for `odrive_pro_0_6_11` (`get_gpio_states: 726`); the ~18 ids this
work needs do not exist in the repo, are not derivable, and must be extracted
from the ODrive 0.6.11 release's `flat_endpoints.json` — which is not in the tree
either.

**Over USB none of this applies.** The `odrive` package resolves properties by
name through the device's own descriptor, which is why
`tools/odrive_fleet_reflash.py` reads and writes 324 config leaves without ever
mentioning an endpoint id. The hardest single constraint on this design is a
property of the CAN/SDO transport alone — but § "USB is not a runtime transport"
below establishes that USB is not available to pay for that relief.

### USB is not a runtime transport on this robot

The obvious way around the endpoint hazard is to read over USB, as
`tools/odrive_fleet_reflash.py` and bench row H7.0c both do. It does not work,
and the reason is physical rather than architectural: **the drives are not on USB
during normal operation.**

- The topology is CAN-only. `docs/can_bridge/index.md:23-35` draws the whole
  system as Jetson ⇄ (UDP over Ethernet, static /30) ⇄ can-bridge Teensy ⇄
  **CAN3 → 6 leg ODrives + hand ODrive + platform Teensy**. There is no USB leg
  in the diagram, and `docs/can_bridge/index.md:62-64` records that "the
  can-bridge is the only CAN path that exists now".
- USB presence is an operator precondition, not an ambient fact.
  `tools/odrive_fleet_reflash.py:89-91` lists "All seven drives powered and on
  USB" among the **preconditions** the operator must satisfy, and the tool then
  *waits* for them (`:560`, `DISCOVERY_TIMEOUT_S = 30.0`) and warns when the hand
  is "unplugged or unpowered" (`:433-436`).
- It is routinely absent. `logbook/2026-08-12-odrive-fleet-reflash-tool.md:98-99`
  records "No ODrives were on USB this session (`lsusb -d 1209:` empty)" on an
  ordinary working day. The same command run on this box on 2026-08-18 exits 1
  with no output: the only devices enumerated are hubs, the Realtek USB-Ethernet
  adapter that carries the can-bridge link, a Bluetooth radio and one Teensy.
- Nothing in the ROS stack has ever used it. The only imports of the real
  `odrive` package are `tools/odrive_fleet_reflash.py:486-489` (lazy, inside
  `OdriveApi.__init__`) and its test. `jugglebot.can.odrive`, imported by
  `teensy_bridge_node` and `orchestrator_node`, is a **name collision** — it is a
  pure CAN frame codec. `odrive` appears in no requirements file and no
  `package.xml`.

A USB-based check would therefore score UNKNOWN on every launch, which is not a
control. This is the finding that collapses the option space in Phase 0.

### The comparison rule: quantise, do not tolerate

A fuzzy tolerance is the wrong instrument, and the reason is measurable. Every
value in both snapshots was produced by `odrivetool backup-config` reading a
float32 register and widening it, so it round-trips **exactly**: `-1.5`, `0.5`,
`-0.055133331567049026`, `0.00551333324983716`, `50.0` and `-8.0` all satisfy
`float32(x) == x`. `tools/odrive_fleet_reflash.py::values_equal` documents this
and compares exactly, noting that "a tolerance would hide a failed write".

The exception is a **hand-edited** snapshot. The 2026-08-18 fix writes
`torque_soft_max: 0.7` and `torque_soft_min: -0.7` by hand, and
`float32(0.7) = 0.699999988079071 ≠ 0.7`. An exact compare against a drive
holding the float32 value would report a false DRIFT on the very register this
work exists to watch.

The rule that survives both cases is therefore **quantise the expected value to
float32, then compare exactly** (`struct.unpack('<f', struct.pack('<f', want))`).
It absorbs hand-editing without opening a band that could mask real drift, and it
is transport-independent. A declared tolerance will not be used.

### Snapshot shapes differ

`odrive_pro_leg_config.json` is **flat** (dotted keys); `odrive_pro_hand_config.json`
is **nested**. Both flatten to 324 leaves.
`tools/odrive_fleet_reflash.py::flatten_config` already accepts either shape and
is the function to reuse.

## Architecture

### The contract, in three parts

| Part | Artifact |
|------|----------|
| Normative document | `ros_ws/docs/odrive_config_contract.md` — the invariant, the register set with per-register failure mode, the verdict vocabulary, and the escalation policy |
| Single enforcement point | one pure-Python module, `ros_ws/src/jugglebot/jugglebot/motion/odrive_config_check.py`, holding the registry, the snapshot loader, the float32-quantised comparison and the verdict; the ROS node calls it once and publishes the row |
| Test | `tests/motion/test_odrive_config_check.py` — fails if the invariant is violated, including a case that reproduces the 2026-08-18 clamp value |

The module takes an injected `read(axis, register) -> float | None` callable, so
the transport is a parameter and the verdict logic is unit-testable with no ROS,
no CAN and no USB. That boundary is what lets Phase 0's transport decision land
without rewriting anything above it.

### Verdict vocabulary — UNKNOWN is not PASS

Three verdicts, and the third is the load-bearing one:

- **PASS** — every declared register read, and every value matched.
- **DRIFT** — a register read and disagreed. Loud: `ERROR` log + a latched
  `/link_status` row. **Warn-and-latch, never refuse.**
- **UNKNOWN** — the check could not be trusted. Any of: the drive's reported
  firmware does not match the build the registry is keyed to; a register did not
  read; the transport was unavailable. Rendered distinctly from PASS everywhere.

An unverifiable check reporting PASS manufactures false confidence about a
safety-relevant register set, which is strictly worse than no check. UNKNOWN
exists so that outcome is unreachable.

### The firmware gate uses data the system already has

The registry is keyed to an ODrive firmware build. The gate reads the per-axis
`(fw_major, fw_minor, fw_rev)` triple that `teensy_bridge_node._version_check_poll`
(line 3466) already decodes from the `GET_AXIS_VERSIONS` bridge-local cache into
`MotorStateTracker`. No new firmware, no new frame, no extra CAN round-trip. If
an axis's triple is absent or does not match the registry's key, that axis scores
UNKNOWN and **no register read is issued for it** — the refusal precedes the
request, as in `gpio_poll.cpp:145-162`.

### Escalation is one line

The verdict-to-action mapping is a single policy table in the enforcement point,
mapping each verdict to `(log_level, latch, block_launch: bool)` with
`block_launch=False` for every verdict today. Escalating DRIFT to a refusal is
that one field. The normative document says so, and says why it is False now: this
is a new instrument, its false-positive modes (endpoint drift, unit mismatch,
hand-edited snapshots) are real, and a launch that refuses to start is a worse
failure than one that complains.

### Register set

Eighteen registers per drive. Chosen by reachable failure mode, not by
importance-in-the-abstract.

| Register | Failure mode if drifted |
|----------|------------------------|
| `axis0.config.torque_soft_min` / `torque_soft_max` | The 2026-08-18 incident. Silently truncates commanded torque; symptom is misattributed to physics |
| `axis0.config.motor.torque_constant` | Rescales commanded torque **and** stealth-detunes the velocity loop (−11.7 % measured). Also the field whose leg value produced the incident, so a cross-drive paste is detectable here |
| `axis0.config.motor.current_soft_max` / `current_hard_max` | The ceilings bench row H7.5 gates against (45 A vs a 50 A limit). A silent reduction truncates authority exactly as the clamp did; a silent raise removes a thermal guard |
| `config.dc_max_negative_current`, `config.max_regen_current` | A **second, independent** way to produce the 2026-08-18 signature: throw decel dumps energy to the bus, and a wrong regen sink either trips overvoltage or clamps braking |
| `axis0.controller.config.vel_limit`, `vel_limit_tolerance`, `enable_vel_limit`, `enable_torque_mode_vel_limit` | A velocity ceiling applied during torque-mode braking truncates authority. `enable_torque_mode_vel_limit` is a boolean whose flip is invisible in every telemetry channel |
| `axis0.controller.config.pos_gain`, `vel_gain`, `vel_integrator_gain` | Leg gains are **FROZEN** (`project_closed_arcs`). Included *because* they are frozen: this is what makes "frozen" enforceable rather than aspirational — a silent change invalidates every downstream tuning conclusion |
| `axis0.config.can.input_torque_scale` | 10000 (leg) / 100 (hand), and it must match the firmware's `LEG_TOR_SCALE` / `HAND_TOR_SCALE`. A mismatch scales every torque feedforward by 100×, silently |
| `axis0.config.can.node_id` | Axis identity. A drift misaddresses the whole fleet; it is also the recovery key `odrive_fleet_reflash.py` records before erasing |
| `axis0.config.motor.pole_pairs` | Wrong commutation |
| `axis0.trap_traj.config.vel_limit` | Governs the ACTIVATE / DEACTIVATE platform rise and lower — an actuating motion |

**Excluded, with reasons:**

- **Per-drive calibration outputs** — `commutation_mapper.config.index_offset`,
  `motor.phase_resistance`, `motor.phase_inductance`. These are "flashed verbatim
  from the snapshot and are NOT preserved per drive (owner's call, 2026-08-12:
  they are regenerated by calibration)". They are *expected* to differ per drive,
  so asserting them yields a permanent false DRIFT on all seven — the fastest way
  to make the instrument ignored. This exclusion is the single most important one.
- **Encoder wiring** (`INC_ENCODER0` vs `ONBOARD_ENCODER0`, `spi_encoder*`,
  `rs485_encoder_group*`) — legitimately differs leg vs hand, and a mismatch fails
  loudly at commutation or homing rather than silently. Candidate for a later
  phase; `check_target_is_leg_config` already guards the flash direction.
- **Housekeeping** (`enable_uart_a`, `usb_cdc_protocol`, `startup_motor_calibration`,
  `watchdog_timeout`, `step_gpio_pin`, …) — no run-time-reachable safety failure
  mode on this robot, and ~300 such leaves would dominate any round-trip budget.
- **`inf`-valued bus limits** (`I_bus_soft_max/min`, `P_bus_soft_max/min`,
  the hand's `config.dc_max_positive_current`) — a finite value appearing there
  *is* real signal, but it is second-order against the 18 above and costs
  round-trips. Deferred to Phase 4, not dismissed.
- **Anticogging** (`anticogging.max_torque`) — a torque bound and therefore
  arguably in family, but anticogging's role on this robot is unestablished.
  Deferred to Phase 4 rather than asserted on a guess.

### Round-trip budget and placement

18 registers × 7 drives = **≤ 126 reads**, one in flight at a time.

On the CAN/SDO transport that is 126 bus round-trips, which **must** complete
before the 500 Hz setpoint stream starts and must never overlap it:
`leg-bus-frame-drops.md` documents per-axis leg-bus encoder frame
drops gated by the 500 Hz streaming load (~10 % of streaming windows against
0/232 idle windows), so SDO traffic during streaming would worsen a known open
defect. Paced at one read per 20 ms the sweep is ~2.5 s, which sits inside the
cold-start window that already budgets ~1.9 s for the Platform state read.

On the USB transport the reads do not touch CAN at all and the constraint is
vacuous — a further argument for Phase 0 resolving transport before anything else.

## Implementation Phase Summary

| Phase | Deliverable | Gate |
|-------|-------------|------|
| **0** | **Transport decision** (owner) — Option A, B or C below | THE parked gate |
| 1 | Pure-Python enforcement point + registry + snapshot loader + float32-quantised comparison + verdict, transport injected | Phase 0 |
| 2 | Normative document `ros_ws/docs/odrive_config_contract.md` | Phase 1 |
| 3 | Delivery surface — under Option A the transport adapter, the `/link_status` row and the node wiring; under Option C a `tools/odrive_config_check.py` CLI | Phases 0–2 |
| 4 | Second-tier registers (`inf` bus limits, anticogging, encoder wiring) | Phase 3 shipped and quiet |

Phases 1 and 2 are transport-independent and could in principle start first.
They deliberately do not: a comparison layer with no live reader is the runbook
failure of § Context point 3 wearing a different hat, and shipping one would
create the appearance of a control where none exists.

## Implementation Phases

### Phase 0 — Transport decision (the gate)

Three options. Each is stated with what it costs, not with a recommendation
attached to authority.

**Option A — can-bridge firmware: add an SDO response cache.**
Follows the `version_check.h` sweep-and-cache precedent exactly: record
`(axis, endpoint, raw u32)` in the `can_buses.cpp` `TxSdo` case alongside the
existing `gpio_poll` consumption, expose it through a new `GET_SDO_CACHE` RPC
that reads the bridge-local cache with no CAN round-trip on the pull, and add
`SDO_READ` to `hand_axis6_permitted` (updating the three tests that pin its
absence). Costs: a new RPC method is a wire change, so `PROTOCOL_VERSION` and
`FW_VERSION` both move and the Jetson checkout and the flash must move together;
an operator flash is required; the ~18 endpoint ids must first be extracted from
the ODrive 0.6.11 `flat_endpoints.json` and added to `config/protocol_config.yaml`;
and the full wrong-endpoint hazard applies, so the firmware gate of § Architecture
is mandatory. This is the option the original scope assumed, and it is the most
expensive of the three.

**Option B — USB, via the `odrive` package. RULED OUT on evidence.**
Technically the cheapest — no firmware, no flash, no endpoint ids, no
wrong-endpoint hazard, no CAN traffic, and `flatten_config` / `diff_flat` /
`values_equal` reusable as-is. It fails on a physical fact, not a design one: the
drives are not on USB during operation (§ Context, "USB is not a runtime
transport"), so every launch would score UNKNOWN. Recorded here so a future
session does not re-derive it as the obvious cheap answer. It becomes viable only
if the robot's wiring changes to put all seven drives on a permanent USB
connection to the Jetson — a hardware decision, not a software one, and one that
would also need the reverse of `CONFLICTING_PATTERNS`
(`tools/odrive_fleet_reflash.py:218-227`) thought through, since `odrivetool` and
the ODrive GUI hold an exclusive USB claim the operator uses regularly.

**Option C — a manual pre-flight, honestly labelled.**
Keep the check human, but move it from a 30-second row buried in a bench document
to a single-command tool (`tools/odrive_config_check.py`) the operator runs during
a maintenance sitting when the drives are already on USB — the same sitting
`odrive_fleet_reflash.py` runs in. Phases 1, 2 and 4 land unchanged; only Phase 3's
adapter and the `/link_status` row are replaced by a CLI. Cheap, no firmware, and
strictly better than H7.0c because it checks all seven drives and every declared
register rather than one register on one drive. **It is explicitly not the control
the owner asked for** — it is a runbook step, and § Context point 3 is the record
of what happens to those. Its honest role is as an interim while Option A is
scheduled, or as the permanent answer if Option A is judged not worth a firmware
chapter.

Wiring Option C into `tools/nightly_suite.sh` was considered and rejected: the
drives are neither powered nor on USB at 04:00, so the nightly would report
UNKNOWN every night, and a channel that always says UNKNOWN is ignored exactly as
H7.0c was.

**What the decision turns on.** The option space is narrower than it looks. CAN3
is the only permanent path from the Jetson to the drives, so **Option A is the
only option that produces an automatic launch-time control** — and it costs a
firmware chapter, a coordinated `PROTOCOL_VERSION` / `FW_VERSION` bump, an
operator flash, and an endpoint registry that must be extracted from
`flat_endpoints.json` and is subject to the wrong-endpoint hazard in full. Option
C delivers most of the comparison value for a fraction of the cost but remains a
manual step.

The decision is therefore: **is automatic drift detection worth a can-bridge
firmware chapter and a flash?** That is an owner call on cost and risk appetite,
not a technical unknown, and it is the gate this plan is parked behind.

### Phase 1 — The enforcement point

`ros_ws/src/jugglebot/jugglebot/motion/odrive_config_check.py`, pure Python, no
ROS imports, `from __future__ import annotations`:

- `REGISTERS` — the 18 entries, each carrying the flattened snapshot path, the
  per-drive-kind applicability (leg / hand), and the failure mode as a docstring
  line so the register set is self-documenting at its single definition site.
- `REGISTRY_FW` — the ODrive firmware triple the registry is keyed to.
- `load_snapshot(path)` — reads a snapshot and flattens it, accepting both the
  flat and nested shapes.
- `expected_f32(value)` — the float32 quantisation of § Context.
- `compare(axis, expected, read) -> AxisVerdict` — issues no read for an axis
  whose firmware triple is absent or mismatched, returning UNKNOWN with the
  reason; otherwise reads each register and classifies.
- `sweep(axes, fw_by_axis, read) -> FleetVerdict` — bounds the total reads at
  `len(REGISTERS) × len(axes)` and carries the worst verdict across the fleet.
- `POLICY` — the one-line escalation table.

### Phase 2 — The normative document

`ros_ws/docs/odrive_config_contract.md`: the invariant ("the live drives' declared
safety registers match the committed snapshots, or the mismatch is loud"), the
register table with failure modes, the exclusions and why, the verdict vocabulary
with UNKNOWN's rationale, the float32-quantisation rule with the `0.7` worked
example, the round-trip bound, and the escalation policy including why
`block_launch` is False today and what changing it would take.

### Phase 3 — The delivery surface

The adapter implements `read(axis, register) -> float | None` for whichever
transport Phase 0 selected. The node calls `sweep()` once during cold start,
before the setpoint stream, and publishes one latched `/link_status` group
alongside the existing `firmware_validated` precedent:

| key | value |
|-----|-------|
| `odrive_config_verdict` | `PASS` \| `DRIFT` \| `UNKNOWN` |
| `odrive_config_drift_count` | number of registers that disagreed |
| `odrive_config_unknown_count` | number of axes scored UNKNOWN |
| `odrive_config_first_drift` | `axis<N>.<register>: live=<x> want=<y>` — the first row, so the log line is actionable without a second query |
| `odrive_config_registry_fw` | the firmware triple the registry is keyed to |

DRIFT logs at `ERROR` once, with every offending row. The latch is sticky for the
session, matching `firmware_validated`.

### Phase 4 — Second-tier registers

Only once Phase 3 has run quiet across several sessions: the `inf` bus limits,
anticogging bounds, and encoder wiring. Adding these before the first tier is
trusted risks a false DRIFT training the operator to ignore the row.

## Testing Plan

`tests/motion/test_odrive_config_check.py`, parallel-safe (`tmp_path` for
snapshots, no fixed paths, no ports, no markers):

1. **The regression that names the incident.** A synthetic hand drive reading
   `torque_soft_min = -0.055133331567049026` against the corrected snapshot scores
   DRIFT, and the offending row names the register. This is the test that would
   have caught 2026-08-18.
2. **UNKNOWN is never PASS.** An axis whose firmware triple mismatches `REGISTRY_FW`
   scores UNKNOWN, the fleet verdict is UNKNOWN, and **the read callable is never
   invoked for that axis** — asserted by a spy, because the refusal preceding the
   request is the whole point.
3. **A read returning `None` scores UNKNOWN, not PASS.**
4. **Float32 quantisation.** A snapshot carrying hand-typed `0.7` matches a drive
   holding `0.699999988079071`; a drive holding `0.69` does not.
5. **Round-trip bound.** A full clean sweep issues exactly
   `len(REGISTERS) × len(axes)` reads and no more.
6. **Warn-and-latch.** DRIFT leaves `block_launch` False for every verdict in
   `POLICY` — the test that makes escalation a deliberate act rather than a drift.
7. **Both snapshot shapes.** `load_snapshot` returns the same 324 leaves for the
   flat leg file and the nested hand file.
8. **Snapshot cross-check.** Every `REGISTERS` path resolves in the real committed
   snapshots — so a register renamed by an ODrive upgrade fails a test rather than
   silently scoring UNKNOWN forever.

## Notes for Collaborators

- **Do not build this on `teensy_sdo_read`.** It is wired to a real RPC and it
  encodes correct bytes, which makes it look usable; it returns no value and it
  cannot address the hand. § Context B1/B2 has the citations.
- **Do not weaken UNKNOWN to PASS** to make a bring-up session quiet. The whole
  instrument is worth less than nothing if it can report all-clear on a register
  set it did not actually read.
- **Do not add a fuzzy tolerance.** Quantise the expected value to float32 and
  compare exactly; § Context has the measurements showing why the tolerance would
  only ever hide drift.
- **The calibration-output exclusion is load-bearing.** Asserting
  `index_offset` / `phase_resistance` / `phase_inductance` produces a permanent
  false DRIFT on all seven drives.
- Bench row **H7.0c** in `tests/hardware/session_anomaly_fixes.md` stays as the
  manual fallback until Phase 3 ships, and should be retired in the same change
  that lands it — two controls for one register, one of them unrun, is how the
  first one stopped being trusted.
