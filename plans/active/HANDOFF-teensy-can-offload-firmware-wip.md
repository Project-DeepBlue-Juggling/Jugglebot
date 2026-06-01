---
title: Teensy CAN Offload — Firmware WIP Handoff
created: 2026-06-01
status: active
branch: teensy-can-offload-firmware-wip
parent_plan: teensy-can-offload.md
---

# Teensy CAN Offload — Firmware WIP Handoff

This document tracks an **autonomous, hardware-free** implementation pass over
Phases 2–8 of [`teensy-can-offload.md`](teensy-can-offload.md), plus the
profiling/instrumentation tooling. It is the entry point for the human review
that must precede any hardware bring-up.

**Nothing here has touched hardware.** Every "done" below means "implemented and
verified to the extent possible without the bench" — see
[Needs hardware validation](#needs-hardware-validation).

## What this branch adds (new files only)

- `ros_ws/src/jugglebot/Teensy_code_legbridge/` — the new Teensy 4.1 "leg
  bridge" firmware (FreeRTOS + QNEthernet + dual FlexCAN). Mirrors the layout of
  the existing `Teensy_code/` (platform Teensy 4.0). **The existing
  `Teensy_code/` is untouched.**
- `tools/probes/teensy_link_profiling/` — offline validation + profiling tools:
  - `hermite_xref/` — bit-for-bit cross-check of the C++-targeted interpolator
    port against the real `motor_guard.py`.
  - `jetson/` — Jetson-side diagnostic UDP consumer (CSV + matplotlib).
- `config/generate_udp_protocol.py` + `config/udp_protocol_spec.py` —
  single-source generator for the UDP wire protocol (emits the C++ header, the
  Python module, and the markdown spec).
- `docs/teensy-udp-protocol.md` — the generated protocol reference.

**No existing production code was modified.** Planned Jetson-side changes
(`can_node.py` → UDP bridge, disabling `bus.broadcast_time()`) are Phases 10/13
of the parent plan and are documented under
[Planned production-side changes](#planned-production-side-changes-not-yet-made)
rather than applied.

## Phase status

| Phase | Title | Status | Notes |
|------:|-------|:------:|-------|
| 2 | FreeRTOS skeleton + Ethernet bring-up | ✅ | Scaffold + QNEthernet static IP + net/heartbeat/diag tasks |
| 3 | UDP framing layer | ✅ | Fixed-length frames + CRC-16/CCITT-FALSE; cross-lang tested |
| 4 | UDP protocol contract finalised | ✅ | Single-source generator → C++/Py/md; 23 xlang tests pass |
| 5 | CAN1 time-sync master + CAN2 ODrive protocol | ✅ | code only; ODrive port byte-validated vs odrive.py (22 tests) |
| 6 | Per-axis state cache + telemetry uplink | ✅ | cache + 100 Hz telem + on-change diag |
| 7 | Hermite/Taylor interpolator port | ✅ | C++ + xref: 0.0 rev divergence vs motor_guard (synthetic + recorded) |
| 8 | Fault state machine + watchdog/deferred-stow | ✅ | invariants ported; logic spec'd by tests; bench-replay pending |
| — | Profiling / instrumentation tools | ✅ | firmware PROFILE frame + Jetson consumer (CSV+plots) + stub client |

Legend: ✅ implemented · ⚠️ partial · ❌ skipped · ⏳ not started.

### Verification summary

The firmware C++ is **not compiled** here (no Teensy toolchain), so correctness
rests on (a) cross-language/cross-reference tests against the production Python
ground truth, and (b) an adversarial review pass. New tests (all in
`tests/firmware/`, runnable with no ROS2/hardware):

- `test_udp_protocol_xlang.py` (23) — C++↔Python protocol byte-consistency + CRC.
- `test_odrive_protocol_xref.py` (22) — ODrive encoders/decoders byte-validated vs `odrive.py`/`bus.py`.
- `test_hermite_xref.py` (3) — interpolator **0.0 rev** vs the real `MotorGuard` (synthetic + recorded) + stroke-bound match.
- `test_fault_logic.py` (8) — executable spec for the fault-determination + deferred-stow invariants.

Authoritative gate: `pytest tests/ -q` (run 2026-06-02): **1572 passed, 1 xfailed
in 441.70 s** (the 1 xfailed is the pre-existing inherited permanent xfail,
unrelated). The 56 new firmware tests are included.

Phases 9 (encoder-search/homing), 10 (Jetson bridge), 11–13 (cutover,
decommission) are **out of scope** for this hardware-free pass — they require
the bench or modify production code. See parent plan. The RPC envelope already
reserves `ENCODER_SEARCH` / `HOME` methods (they return `ERR_NOT_IMPL`), and
`SDO_READ`/`SDO_WRITE` are wired (the SDO response decode for encoder-search
feedback lands with Phase 9), so Phase 9 slots in without a protocol change.

## Decisions made autonomously

### D1 — Fixed-length typed frames, NOT COBS (Phase 3)

**Decision:** Each UDP datagram is one fixed-layout, typed frame:
`[Header(8)][payload][crc16(2)]`, little-endian. Header = magic(u16) + version(u8)
+ msg_type(u8) + seq(u16) + length(u16).

**Alternatives:** COBS-delimited framing (the plan listed "COBS or fixed-length, TBD").

**Rationale (root cause, not appeal-to-plan):** COBS solves *message framing on a
byte-stream transport* (serial/TCP) that has no inherent record boundaries. UDP
datagrams **already** carry exactly one message boundary per recv, so COBS adds a
byte-stuffing encode/decode scan for zero benefit here. A fixed per-type layout is
(a) zero-allocation and (b) constant-time to parse — both mandatory for the hard
real-time firmware, where a variable-length scan would inject timing jitter into the
RX path that competes with the 500 Hz interp tick. We keep a `length` field so future
variable-payload types (RPC blobs) are supported without a format change.

### D2 — CRC-16/CCITT-FALSE (Phase 3)

**Decision:** poly `0x1021`, init `0xFFFF`, no reflection, no final XOR. Computed over
the whole frame except the trailing CRC. Verified against the canonical check value
(`crc("123456789") == 0x29B1`) in both languages.

**Rationale:** Standard, well-specified, table-free bit-serial form is trivial to make
byte-identical in C++ and Python (the cross-lang test enforces this). Good Hamming
distance for the short frames here. CRC-32 would be overkill for ≤166-byte frames and
cost more cycles on the M7.

### D3 — Single-source protocol generator, separate from `generate_config.py` (Phase 4)

**Decision:** `config/generate_udp_protocol.py` holds the SPEC and emits the C++ header,
the Python module, and the markdown doc. It does **not** extend the existing
`generate_config.py`.

**Alternatives:** Fold UDP-protocol codegen into `generate_config.py` (the plan
suggested "consider extending" it).

**Rationale:** `generate_config.py` is a large, load-bearing, CAN/geometry-focused file
on the critical path of every config change. The UDP protocol is a distinct concern with
a different shape (packed structs + framing helpers, not flat constants). A separate
generator keeps each tool single-purpose, avoids risk to the existing config pipeline,
and is independently testable. Byte-for-byte C++/Python consistency is guaranteed by
construction (one SPEC) and enforced by `tests/firmware/test_udp_protocol_xlang.py`
(committed C++/Py must equal a fresh generation).

### D4 — Wire carries motor-rev space, not mm/pose (Phase 4)

**Decision:** `Setpoint` carries `u0/u1/u2/v0/accel` in **motor-rev / rev·s⁻¹ / rev·s⁻²**
(Jugglebot convention, positive = extension) and `torque_ff` in Nm — exactly what
`motor_guard`'s interpolator consumes internally. The Jetson bridge performs the same
mm→rev / pose conversions `motor_guard` does today.

**Rationale:** Keeps the Teensy interpolator byte-identical to `motor_guard`'s rev-space
math (the Phase 7 xref proves <1e-6 rev match). The mm/pose workspace check stays on the
Jetson per the parent plan ("motor_guard non-interpolation responsibilities stay").
`u1`/`u2` presence is signalled by explicit flag bits, **not** NaN sentinels — avoids
NaN-in-float ambiguity on the wire.

### D5 — Interpolator ports the FULL ladder incl. lead + stroke clamp (Phase 7 — see note)

**Decision:** The Teensy `leg_interp` ports `motor_guard.py:870-1049` *including* the
`MAX_LEAD_REV` lead-clamp and the per-leg stroke clamp.

**Tension in the parent plan:** The plan's "What stays unchanged" says the `MAX_LEAD_REV`
lead-clamp "stays inside motor_guard on the Jetson", yet Phase 7's scope is "port
motor_guard.py:870-1049" — a range that *contains* both clamps.

**Resolution + rationale:** Port the full ladder. The clamps are cheap and safety-critical;
running them on the device that actually emits the 500 Hz CAN setpoints is strictly safer
(defence-in-depth at the point of command generation). The Jetson-side `motor_guard`
lead-clamp then becomes redundant defence-in-depth, to be reconciled when `motor_guard` is
slimmed at the Phase 10 bridge work. Recorded so a reviewer can challenge it.

### D6 — Stroke-clamp bounds embedded as constants, not codegen (Phase 7)

**Decision:** Per-leg `STROKE_MIN_REV[6]` / `STROKE_MAX_REV[6]` are embedded in
`legbridge_config.h`, captured 2026-06-01 from the live `MotorGuard`
(`_stroke_min_rev`/`_stroke_max_rev` = `WorkspaceLimits.from_geometry` hard limits ×
per-leg `GEOM_MM_TO_REV`).

**Rationale:** These are derived from workspace limits not currently exported by the
codegen. Embedding + a TODO(Phase 10) to hoist them into codegen is lower-risk than
modifying the generated `hardware_config` pipeline now. The xref harness asserts the
embedded constants equal the running guard's, so drift is caught.

### D7 — CAN RX decoded in the FlexCAN callback, not via an ISR→queue→task hop (Phase 5)

**Decision:** `can_buses` decodes incoming ODrive frames straight into the cache in
the `onReceive` callback (pumped by the CAN-RX task's `canX.events()`), not by
enqueuing raw frames to a task as the plan's table sketches.

**Rationale:** This is the proven platform-Teensy idiom (`Teensy_code.ino` `canSniff`).
The decode is a few µs of `memcpy` + seqlock write — bounded and fast — so a queue hop
adds latency and a copy for no benefit. Failure mode it avoids: a backed-up RX queue
under a fault storm decoupling the cache from reality. The cache writes are
seqlock-guarded so the interp ISR's reads are torn-read-free.

### D8 — RPC method arg layouts co-located in `rpc.h`, not in the generator (Phase 5)

**Decision:** The per-method RPC argument structs (`ArgAxisState`, `ArgVelCurr`, …)
live as packed structs in `rpc.h`, not in `config/generate_udp_protocol.py`.

**Rationale:** The consumer of these layouts — the Jetson UDP bridge — is Phase 10
(out of scope tonight) and does not exist yet, so there is no second language to keep
in sync *now*. Co-locating them with the dispatcher keeps them readable and reviewable.
**Follow-up:** hoist into the generator at Phase 10 when the Jetson bridge becomes the
second consumer (same single-source discipline as the frame layouts).

### D9 — Friction feedforward NOT ported to the interpolator (Phase 7)

**Decision:** `leg_interp` ports the position/velocity ladder + clamps but passes
`torque_ff` through unchanged; it does **not** port `motor_guard._compute_friction_ff_Nm`
(the Stribeck-with-smooth-gate friction model).

**Rationale:** Friction FF is a separate additive *torque* term that does not affect the
commanded position or velocity — which is what the ladder produces and what the xref
validates (0.0 rev). Porting it requires the `FrictionFFParams` (a tuned bench fit with
a load-bearing smooth-gate that prevented a 5 Hz limit cycle — see
`logbook/2026-05-08-friction-ff-platform-limit-cycle.md`) and its own validation. It is
a well-scoped **follow-on port**, flagged so the torque path isn't silently lossy on the
bench. Until then the Teensy emits MPC-supplied `torque_ff` only (gravity+inertia).

### D10 — Interpolator runs in a hardware IntervalTimer ISR above the FreeRTOS ceiling (Phase 7)

**Decision:** The 500 Hz tick is a bare `IntervalTimer` ISR (`leg_interp.cpp`), set to a
priority above `configMAX_SYSCALL_INTERRUPT_PRIORITY`, computing AND transmitting to CAN2
directly in the ISR. It makes no FreeRTOS calls; setpoint hand-off uses a plain volatile
pending flag.

**Rationale:** The entire point of moving off Linux is non-negotiable 500 Hz determinism
(plan §"Task layout": "the interp task is the highest priority… nothing else can preempt
it"). An ISR above the RTOS syscall ceiling literally cannot be delayed by any RTOS
critical section — stronger than a high-priority *task*, which still yields to critical
sections and scheduler latency. **Bench-validation items** (handoff list): the exact
priority value vs the syscall ceiling, and FlexCAN mailbox writes from ISR context.

### D11 — Watchdog consolidated into the 10 Hz fault task (Phase 8)

**Decision:** The CAN2 heartbeat watchdog runs inside `fault_step()` (10 Hz), not as a
separate 1 Hz task with a socketcan restore generator (can_node's
`_watchdog_check` + `attempt_restore_steps`).

**Rationale:** can_node's restore generator re-initialises the *socketcan device* across
ticks (a Linux-stack operation that can block). The Teensy's FlexCAN peripheral never
goes away — "restore" is simply observing leg heartbeats resume, a passive check. So the
generator/restore machinery has no analog here; folding the watchdog into the 10 Hz fault
evaluation (faster than can_node's 1 Hz, strictly safer) keeps all fault logic at one
enforcement point. The "confirmed reconnect = fresh leg heartbeats" signal replaces
can_node's "attempt_restore_steps rx+tx OK".

### D12 — Teensy-side deferred-stow execution = velocity-limited descent in the ISR (Phase 8)

**Decision:** The deferred stow is executed on-Teensy as a velocity-limited per-leg
descent to the off pose (stroke min) in the 500 Hz ISR (`interp_begin_stow`), at
`JB_OP_GENTLE_MOVE_VEL_LIMIT_RPS`, then IDLE — the analog of can_node's
`_gently_move_to_setpoint(0.0, deactivating=True)`.

**Rationale:** The hard-won invariant (logbook 2026-05-19) is *never command a dead bus;
stow only on confirmed reconnect*. The Teensy now owns CAN, so it owns the latch and the
descent. The descent is velocity-limited (the safety-critical property; on_shutdown uses
the same limit) but NOT yet jerk-limited like the Jetson's quintic — a simplification.
In the cutover, the Jetson bridge may *also* arm a stow; the two compose idempotently
(both drive to the same off pose). **Follow-up:** reconcile Teensy-vs-Jetson stow
ownership and add jerk-limiting at Phase 10/11. **Bench-validation:** replay every
logbook fault scenario (soft-reset bounce, CAN-loss safety inversion, undervoltage) and
confirm the descent + IDLE handoff (plan Phase 8 "done when" + Risk note).

## Adversarial review (post-implementation, 2026-06-02)

Because the firmware cannot be compiled here, a 6-dimension adversarial review
(29 agents: one skeptical reviewer per port dimension vs ground truth, each finding
independently verified) was run over the C++ ports. 23 raw findings → 21 confirmed
real; 2 correctly refuted (`lroundf` half-away-from-zero vs Python `round()` half-to-
even — benign; an alleged E-STOP label-priority inversion — not real). **All 21 fixed
or explicitly accepted** in commit `<review-fixes>`:

- **Blocking — compile error:** `.ino` referenced `JbUdp::HEARTBEAT_J2T_SIZE` but the
  generator's `_screaming` emitted `HEARTBEAT_J2_T_SIZE` (split the digit→capital
  boundary). Fixed the generator to split only on lowercase→capital (names now
  `HEARTBEAT_J2T_SIZE` etc.) AND switched the `.ino` check to `sizeof(...Payload)`.
- **Blocking — dropped safety state:** `FaultState::MAX_DEVIATION` was never assigned.
  Added the incoming-command-vs-encoder deviation E-STOP (motor_guard.py:539-551).
- **High — concurrency (500 Hz path):** torn 64-bit reads / RMW races between the interp
  ISR (above the FreeRTOS syscall ceiling) and tasks. Added IRQ-guarded atomic-64
  accessors and applied them to `micros64`, `s_wall_offset_us`, `s_last_setpoint_us`,
  `s_last_rx_us`, `g_last_jetson_hb_us`; made the setpoint staging publish atomic
  (IRQ-guarded copy + flag = the barrier the seqlock would give); moved the UDP TX
  seq-increment + encode-into-shared-buffer + transmit inside one `NetLock`.
- **Medium:** soft-reset budget no longer consumed on a no-op clear (bus down); deferred
  stow now descends to the true off pose 0.0 (was stroke-min) with an accel-limited ramp
  (no startup velocity step) and uses `STOW_DONE_EPS_REV`; `decode_into_cache` drops
  truncated (<8-byte) CAN frames (the omitted `_check_len`).
- **Accepted (low, latent):** `clip_position` treats a non-leg/non-hand axis as the hand
  rather than rejecting it — unreachable (only legs 0-5 call it) and the fallback is safe;
  left as-is, noted here.

After the fixes: `pytest tests/firmware/ -q` → 56 passed; the interpolator xref still
shows **0.0 rev** divergence.

## Needs hardware validation

Everything compiled-by-inspection only — no Teensy toolchain in this environment.
The C++ has **not been compiled**. First bench step is a clean build (resolve any
library-API drift) before behavioural testing.

- **Build the firmware.** Confirm it compiles against the pinned FreeRTOS_TEENSY4
  + QNEthernet + FlexCAN_T4 versions. Fix any API drift (esp. the FreeRTOS umbrella
  header — `freertos_shim.h`; greiman vs tsandmann fork).
- **Phase 2/3:** LED blinks 1 Hz (scheduler alive); `ping 192.168.42.2` works;
  `ping -i 0.01 -c 1000` latency < ~2 ms; UDP echo round-trips; serial shows clean
  task scheduling. The QNEthernet PHY brings the Jetson adapter link lights up.
- **lwIP threading model.** `udp_link.cpp` serialises QNEthernet with a recursive
  mutex (RX in `net` task, TX from any task). Confirm this holds under load, or
  switch to a single net-task + TX-queue design (noted in the README). lwIP is not
  reentrant — this is the highest-risk integration point.
- **FreeRTOS heap sizing.** Total task stacks ≈ 25 KB; confirm `configTOTAL_HEAP_SIZE`
  is large enough and `vTaskStartScheduler()` does not return (the fatal path blinks
  fast).
- **Phase 5 — time-sync master (0x7DD).** Sniff CAN1; confirm 100 Hz frames in the exact
  `pack('<II', sec, usec)` format, and that all three slaves (platform 4.0, BB, cone)
  still report `time_synced == true` with behaviourally-correct timing. Confirm the
  Jetson stops broadcasting on CAN (its `bus.broadcast_time()` disabled — Phase 5
  production change, not yet made; see below).
- **Phase 5 — ODrive cycle.** Drive one bench ODrive on CAN2 through IDLE → CLOSED_LOOP →
  position commands → IDLE; confirm all telemetry parses. The encoders are byte-validated
  vs `odrive.py` offline, but on-wire confirmation is still required.
- **Phase 5 — IntervalTimer ISR priority.** Confirm `s_timer.priority(32)` sits above
  `configMAX_SYSCALL_INTERRUPT_PRIORITY` so the interp ISR is non-maskable by FreeRTOS,
  and that FlexCAN `write()` from ISR context is safe under load.
- **Phase 7 — float32 divergence.** The algorithm matches `motor_guard` to 0.0 rev in
  float64 (xref). Measure the float32-on-hardware residual against a recorded throw; the
  plan accepts "within a tolerance that doesn't affect tracking" if it isn't bit-exact.
- **Phase 7 — friction FF gap.** Torque path currently omits the Stribeck friction FF
  (decision D9). Either port it or confirm bench tracking is acceptable without it.
- **Phase 8 — replay every logbook fault scenario.** The fault logic is spec'd by
  `tests/firmware/test_fault_logic.py` (a Python mirror) but the C++↔can_node fidelity
  and the real timing need bench replay: soft-reset bounce-loop cap, the CAN-loss
  safety-inversion (legs stay CLOSED_LOOP during the down window; profiled stow on
  confirmed reconnect; re-arm on mid-descent re-drop), and undervoltage recovery. The
  plan's Phase 8 "done when" requires reproducing `can_node` test coverage on hardware.
- **Phase 8 — interp ISR ↔ fault task race on the output gate / stow flags.** The fault
  task (FreeRTOS, prio 3) writes `s_output_enabled` / stow flags that the interp ISR
  (above the syscall ceiling) reads. Single-word volatiles are atomic on M7, but confirm
  no multi-field tearing matters (the stow uses a single `s_stow_active` flag + a
  separately-captured `s_stow_pos[]`; verify the begin/consume ordering on the bench).

## Blocked — needs human input

_(genuine blockers — none so far)_

## Planned production-side changes (not yet made)

Per the hard constraint, **no existing production code was modified**. The cutover
will require these — documented here, to be made in their own commits/sessions:

- **Disable the Jetson time-sync master (Phase 5).** Remove/guard the
  `self.create_timer(ts_period, self.bus.broadcast_time)` callsite and the
  `bus.broadcast_time` keep-alive entry in `can_node.py.__init__`; the leg-bridge
  Teensy now owns 0x7DD. Also fix the stale slave-list docstring in `bus.py:108`
  (omits the cone Teensy) — flagged in the parent plan's follow-ups.
- **Jetson UDP time-of-day responder (Phase 5).** A small UDP responder on
  `PORT_RPC` answering `RpcMethod::TIME_OF_DAY_QUERY` with `CLOCK_REALTIME` µs
  (the master's wall-clock anchor). New code; the Teensy is the client.
- **`can_node.py` → UDP bridge (Phase 10).** Replace CAN encode/poll paths with the
  UDP protocol; re-expose identical ROS2 topics/services; reimplement the watchdog
  as a Teensy-link monitor. Hoist the RPC method arg layouts (decision D8) into the
  generator at this point (the bridge is the second consumer).
- **Move `JB_OP_MAX_POSITION_STEP_REV` per-step gate into the UDP sender (Phase 10).**
- **Decommission socketcan (Phase 13).** Remove `python-can`, delete `bus.py` and
  dead branches.

## Build instructions

Full detail in [`Teensy_code_legbridge/README.md`](../../ros_ws/src/jugglebot/Teensy_code_legbridge/README.md).
Summary: Teensy 4.1, Teensyduino 1.59+ (Arduino IDE 2.x or PlatformIO `teensy`),
600 MHz, Optimize "Faster", USB type Serial. Libraries: FreeRTOS_TEENSY4 (greiman),
QNEthernet (ssilverman), FlexCAN_T4 (bundled). Regenerate shared headers with
`python config/generate_config.py` and `python config/generate_udp_protocol.py`
before building.

## Recommended order of human review

Review in dependency order — the contract first, then the modules that build on it,
then the safety-critical ports, then the tools:

1. **Protocol contract** — `docs/teensy-udp-protocol.md` + `config/generate_udp_protocol.py`.
   This is the spine; everything else encodes/decodes against it. Check the frame
   layouts and the framing decision (D1/D2). `tests/firmware/test_udp_protocol_xlang.py`
   enforces C++↔Python consistency.
2. **Config + cache** — `legbridge_config.h` (pins, task table, the 1:1-ported control
   constants) and `axis_state.h`. Sanity-check the pin map and FreeRTOS priorities against
   your board.
3. **ODrive port** — `odrive_protocol.h` vs `odrive.py`. Byte-validated by
   `tests/firmware/test_odrive_protocol_xref.py`, but eyeball `encode_leg_setpoint`
   (the sign/scale/clip chain) — it commands the legs.
4. **Interpolator (safety-critical)** — `leg_interp.cpp` vs `teensy_interp.py` vs
   `motor_guard.py`. The xref proves 0.0 rev; confirm the C++↔Python transcription by eye
   and decide on the float32 residual + the friction-FF gap (D9).
5. **Fault state machine (safety-critical)** — `fault_machine.cpp` vs `can_node.py:386-483`
   / `:1443-1530` and the 2026-05-19 logbook. This is the most subtle port; the deferred-
   stow invariants must hold. `tests/firmware/test_fault_logic.py` is the executable spec.
   **Plan a bench replay of every logbook fault scenario before trusting it on hardware.**
6. **CAN + time-sync + RPC + telemetry** — `can_buses.cpp`, `time_sync_master.cpp`,
   `rpc.cpp`, `telemetry.cpp`. Confirm the 0x7DD payload matches `bus.py` and the RPC arg
   layouts (D8).
7. **Scaffold** — `Teensy_code_legbridge.ino` (task creation, wiring) + `net_ethernet.cpp`
   / `udp_link.cpp` (the lwIP threading model, D-handoff).
8. **Tools** — `tools/probes/teensy_link_profiling/` (xref harness + Jetson consumer + stub).

Then: build it (no toolchain here — first bench step), and work the
[Needs hardware validation](#needs-hardware-validation) list.
