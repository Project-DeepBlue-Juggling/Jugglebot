---
title: "FW 23: axis-silence watchdog replaces the heartbeat-only CAN_BUS_DOWN predicate"
type: bugfix
date: 2026-09-15
status: done
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/axis_state.h (new `AxisState::last_rx_us` — any-frame liveness stamp)
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp (stamp `last_rx_us` above the per-command switch; new per-axis `s_hb_frames[]` census)
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h (`CanRxHealth::hb_frames[NUM_AXES]`)
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp (`any_present_leg_silent`/`all_present_legs_alive` read `last_rx_us`; `first_silent_leg`; CAN_BUS_DOWN trip latch; `fault_hb_stale_mask`)
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.h (three latch getters + the mask getter)
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h (CAN_HEARTBEAT_TIMEOUT_US → CAN_AXIS_SILENCE_TIMEOUT_US, value unchanged; new CAN_HEARTBEAT_STALE_US = 0.5 s; FW_VERSION 22 → 23)
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino (fill the three latch fields + flags bits 16-22)
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp (CacheDiag `hb_frames` fill + width static_assert)
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp (renamed predicate in the reboot-latch comment)
  - config/generate_udp_protocol.py (PROTOCOL_VERSION 8 → 9; HeartbeatT2J +can_fault_leg/age_ms/count; HeartbeatT2JFlags HB_STALE_MASK + HEARTBEAT_HB_STALE_SHIFT; CacheDiag +hb_frames[7]; FaultState CAN_BUS_DOWN doc string)
  - config/generated/udp_protocol.h, config/generated/udp_protocol.py, docs/teensy-udp-protocol.md, ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h, tools/probes/teensy_link_profiling/jetson/udp_protocol.py (regenerated)
  - tests/firmware/native/test_fault_machine.cpp (stamp_hb/stamp_rx_only helpers + 5 FW 23 cases)
  - tests/firmware/native/test_platform_relay.cpp (renamed constant)
  - tests/firmware/test_udp_protocol_xlang.py (PROTOCOL_VERSION + wire-layout re-pin)
  - tests/firmware/test_bridge_fw_version_xref.py (22→23 bump-history clause)
  - teensy_link/rpc_args.py (EXPECTED_BRIDGE_FW_VERSION 22 → 23)
  - tests/teensy_link/test_protocol_codec.py, tests/teensy_link/test_v5_wire_regression.py (v9 size/version pins)
  - docs/can_bridge/safety.md, ros_ws/docs/can-node-teensy-parity.md, plans/active/leg-gain-tuning-methodology.md (renamed constant/predicates)
---

## What / Why

At the 2026-09-15 R3 sitting `fault_state = CAN_BUS_DOWN` tripped twice and
self-cleared inside one 10 Hz fault tick each time, stowing the robot, while the
bus was demonstrably healthy: 0 CAN wire errors, ring leak 0, RX ~1950 frames/s
uninterrupted, and every axis's encoder stream fresh (worst per-axis age 95 ms).
Forensics in the scratchpad report; the mechanism is that the fatal predicate
(`fault_machine.cpp` `any_leg_heartbeat_stale`) read **only**
`AxisState::last_heartbeat_us` against a 2.0 s timeout, and the legs are
configured `heartbeat_msg_rate_ms = 100`. So the trip needed **20 consecutive
lost frames of the single sparsest stream an ODrive emits** — 10 of the ~272
frames/s it broadcasts — and an ODrive *drops* rather than queues a cyclic frame
that finds no free TX mailbox, so under the 62 % streaming bus load the 10 Hz
stream is by far the most exposed to a long run of consecutive losses. Bus-side
arbitration priority cannot rescue a frame that never entered a mailbox.

Worse, the event was unattributable: nothing on the wire carried per-leg
heartbeat age, and `fault_machine.cpp` latched nothing for this fault (unlike
`max_dev_leg` for MAX_DEVIATION). A safety watchdog that stows the machine,
self-clears in ≤100 ms and leaves no record of *which* leg or *how stale* is a
guard nobody can debug — the owner's call was to fix the proxy measurement, keep
reporting heartbeat dropouts, and stop letting them stow the robot.

## Discussion

**The failure classes, and how the new predicate treats each.** The fault's job
is to answer one question — *has an actuator stopped answering?* — and the old
predicate answered a different one.

1. **Whole bus dark** (cable out, transceiver dead, bridge bus wedged). Every
   leg goes silent on every stream simultaneously. Still trips, at the same 2.0 s,
   and is now simply the all-legs instance of one predicate rather than a
   separate case. The 2026-06-24 bench cutover measured this at ≈ +1.95 s after
   the unplug; nothing about that changes.
2. **One ODrive dead / wedged / unpowered.** That node stops emitting
   *everything*. Still trips — and deliberately so: the platform is a closed
   kinematic chain, five legs streaming against a dead sixth is the hazard, so a
   single silent leg must still suppress output and arm the stow. Now the trip
   also *names* the leg and its silence age.
3. **One leg's heartbeat dropped under load** (the 2026-09-15 event). No longer
   a fault at all. The leg is alive on the wire — its encoder, iq, error, temp
   and vbus frames keep arriving — so `last_rx_us` stays fresh and the machine
   keeps flying. The dropout is instead *reported*, on two new surfaces: the
   live `HB_STALE_MASK` (HeartbeatT2J flags bits 16-22) at a deliberately
   sharper 0.5 s, and a per-axis `hb_frames[7]` census on CacheDiag beside the
   existing `enc_frames`.

The contract shape is the point: **one invariant** (a present leg that has sent
no frame of any kind for 2.0 s is silent), **one enforcement point**
(`any_present_leg_silent`, with `first_silent_leg` for the latch and
`all_present_legs_alive` as its exact complement), and the liveness stamp written
*above* the per-command switch in `decode_into_cache` so no frame type added to
the decode later can silently miss it. That placement is what stops the class
from recurring: the old defect was structurally "the predicate depends on one
arbitrary stream", and the fix makes it depend on the union by construction.

**What was ruled out.**

*Widening `CAN_AXIS_SILENCE_TIMEOUT_US`.* The cheap move, and wrong on this
evidence: the watchdog fired on a condition that was genuinely true — 20
heartbeats really were lost. Widening it trades a false stow for a slower
response to a real bus loss and leaves the proxy just as bad, one factor of two
further out. The defect is the *measurement*, not the threshold.

*Raising the leg `heartbeat_msg_rate_ms` 100 → 20.* Gets 5× the evidence in the
same 2.0 s window, for +280 frames/s (≈ +3.6 % bus load) on a bus already at
62 % — i.e. it adds load to the exact mechanism (TX mailbox contention) that
caused the dropouts, and it is an ODrive-config change that a `config reset`
silently reverts. Reading the streams the legs *already* emit costs zero bus
load and zero configuration state.

*Excluding the hand, or requiring all legs silent before tripping.* The
watchdog stays legs-only (`NUM_LEGS`) exactly as before — there is nothing to
stow on the hand, and a stale hand heartbeat must not arm the leg stow. And
"all legs silent" would have suppressed the 2026-09-15 false trip too, but at
the cost of going blind to class 2, which is the more dangerous of the three.

**Tradeoff accepted.** The fatal predicate is now *less* sensitive to an ODrive
that keeps broadcasting telemetry while its control loop has stopped responding
to commands — a node that streams but does not act would keep `last_rx_us`
fresh. That gap was already covered, and is covered better, by the layers built
for it: `MOTOR_FB_STALENESS_US` (0.15 s on `pos_timestamp_us`) suppresses output
on a stale encoder, the `MAX_DEVIATION` guard latches on commanded-vs-encoder
divergence, and `evaluate_errors` E-STOPs on any active error or
disarm-while-CLOSED_LOOP. Heartbeat *content* (`axis_state`,
`procedure_result`) still reaches `evaluate_errors` and
`all_axis_heartbeats_ok`; only the *timeout* moved.

**One rename that is not cosmetic.** `all_axis_heartbeats_ok`
(HeartbeatT2J flags bit 2) is left heartbeat-based at 2.0 s. It means what its
name says, but it is no longer the same question as "the bus is up", and a
consumer must stop reading it as one — `HB_STALE_MASK` is the sharper per-axis
form. Called out here because that flag is the kind of thing a future session
reaches for when the fatal predicate's answer is wanted.

## Fix

* `AxisState::last_rx_us` (`axis_state.h`), stamped with `atomic_write_u64` in
  `decode_into_cache` immediately after the axis/length validity guards and
  **before** the per-command switch, so heartbeat, get_error, encoder, iq,
  temps, vbus and version all count as liveness. Stray/garbled frames are
  excluded by those two guards, so bus junk cannot forge liveness.
* `CAN_HEARTBEAT_TIMEOUT_US` → `CAN_AXIS_SILENCE_TIMEOUT_US` (2 000 000 µs,
  unchanged) across firmware, native tests and docs;
  `any_leg_heartbeat_stale`/`all_present_legs_fresh` →
  `any_present_leg_silent`/`all_present_legs_alive`, both now reading
  `last_rx_us`. The reboot-in-progress suppression latch rides the same
  predicate, as before. `FaultState::CAN_BUS_DOWN` keeps its wire value; its
  generator doc string now says what it actually measures.
* New `CAN_HEARTBEAT_STALE_US = 500 000` (5 lost heartbeats) drives
  `fault_hb_stale_mask()` → HeartbeatT2J flags bits 16-22. Report-only; nothing
  in the firmware reads it. It lives in `fault_machine.cpp`, not the `.ino`,
  so the native harness can compile and assert it — the heartbeat uplink is not
  natively compilable, and an untested producer is how the pre-FW-23 per-axis
  staleness ended up computed-and-discarded.
* Trip latch: `can_fault_leg` (u8, 0xFF = none), `can_fault_age_ms` (u16,
  saturating) and `can_fault_count` (u16, saturating) on HeartbeatT2J, written
  once on the trip edge and **not** cleared by the self-clear.
  `fault_notify_clear_errors` releases leg/age but keeps the count.
* `hb_frames[7]` on CacheDiag, incremented in the heartbeat decode case beside
  the existing `s_enc_frames` precedent.
* PROTOCOL_VERSION 8 → 9 (HeartbeatT2J 121 → 126 B, CacheDiag 129 → 157 B — both
  grow, so the bump is mandatory and darkness against an FW ≤ 22 board is the
  intended failure); FW_VERSION 22 → 23.

## Verification

* Native firmware harness (`python tests/firmware/native/build.py &&
  temp/firmware_native/test_fault_machine`, run 2026-09-15): **23 test cases,
  431 assertions, all pass** (18 cases / 408 assertions before). The five new
  cases are the `FW 23: …` block in
  `tests/firmware/native/test_fault_machine.cpp`: (a) a leg whose heartbeat is
  dead while its frame stream keeps arriving does NOT trip — 6 s walked in
  100 ms steps, three times the 2.0 s window, with the fault and the stow latch
  asserted clear on every tick and the dropout asserted *reported* in
  `HB_STALE_MASK`; (b) one wholly silent leg DOES trip and latches
  `can_fault_leg == 4`, age 2300-2400 ms, count 1; (c) the latch survives the
  self-clear, re-points and re-counts on a second trip, and `CLEAR_ERRORS`
  releases leg/age but keeps the count; (d) whole-bus darkness latches leg 0;
  (e) `HB_STALE_MASK` is per-axis, includes the hand, is silent at 400 ms, fires
  at 600 ms, never faults, and fits its 7 wire bits.
* The committed `tests/firmware/native/fault_golden.json` is **unchanged**, and
  `tests/firmware/test_fault_logic.py` (the host-mirror conformance replay) is
  green without edits. Every pre-existing scenario stamps both clocks through the
  new `stamp_hb()` helper, so the golden is the regression proof that the
  predicate swap is behaviour-preserving wherever the two predicates agree —
  which is everywhere the only bus traffic is heartbeats.
* `pytest tests/firmware -q`, run 2026-09-15: **258 passed, 1 skipped in
  13.60 s**.
* `pytest tests/teensy_link -q`, run 2026-09-15: **392 passed in 11.46 s** (7
  wire-size/version pins moved to v9: HeartbeatT2J 121 → 126 B, CacheDiag
  129 → 157 B).
* `pytest tests/ros -q`, run 2026-09-15: **2848 passed, 4 skipped in 391.23 s**
  — the existing bridge decode is unaffected (it reads HeartbeatT2J by field
  name off the generated dataclass, so the inserted fields are simply unread
  until the host follow-on lands).
* `pytest tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py
  -q`, run 2026-09-15: **83 passed in 0.50 s**.
* `pio run -e teensy41` (BUILD ONLY, from
  `ros_ws/src/jugglebot/Teensy_code_canbridge/`, run 2026-09-15): **SUCCESS in
  10.89 s**, text 245 056 / data 35 520 / bss 111 072 B. Log:
  `temp/logs/fw23_pio.log`. **NOT FLASHED** — the operator flashes.
* `python config/generate_udp_protocol.py` + `python config/generate_config.py`,
  run 2026-09-15: five generated artifacts rewritten
  (`config/generated/udp_protocol.{h,py}`, `docs/teensy-udp-protocol.md`, the
  firmware's delivered `udp_protocol.h`,
  `tools/probes/teensy_link_profiling/jetson/udp_protocol.py`);
  `generate_config.py` produced no diff.
* NOT run: `./run_tests.sh` (the full gate). Parallel sessions are mid-edit on
  `motion/skills/`, `skill_node.py` and `rosbridge_websocket_lean.py` in this
  worktree, so a whole-tree run would measure their work in progress. The gate
  belongs to the commit that lands this.

(2026-09-15, `./run_tests.sh --full`, log `temp/logs/r3_followups_full2_20260915.log`, the combined tree of the four same-day units): **PASS — parallel 6146 passed, 9 skipped, 2 xfailed in 286.47 s; serial 6 passed in 18.79 s.**

## Host follow-on (NOT in this change)

The Jetson bridge must decode and surface the new wire surface before the next
powered sitting, per the forensics report's condition (1):

* `HeartbeatT2J.can_fault_leg` / `can_fault_age_ms` / `can_fault_count` →
  `/link_status` rows, and the `Teensy guard FAULT LATCHED` ERROR line must name
  the leg and age instead of being silent about both.
* `HeartbeatT2JFlags.HB_STALE_MASK` (bits 16-22, `HEARTBEAT_HB_STALE_SHIFT`) →
  per-axis `/link_status` rows for legs 0-5 as well as BB 7/8, which is the only
  place the load-gated dropout becomes visible while the robot is still flying.
* `CacheDiagPayload.hb_frames[7]` → the cache-diag consumer beside `enc_frames`.
* The CAN_BUS_DOWN ERROR line currently tells the operator to run
  `/clear_errors`, which is wrong for a self-clearing fault.
* Pins: can-bridge **FW 23**, **PROTOCOL_VERSION 9**.
  `teensy_link/rpc_args.EXPECTED_BRIDGE_FW_VERSION` is already 23 here (the
  firmware xref test pins the pair); the board is UNFLASHED, so the live link is
  dark against this tree until the operator runs
  `pio run -e teensy41 -t upload`.

## Host side (same day)

Every item this entry's "Host follow-on" section listed is now wired in
`ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py`. The wire decode itself
needed no codec change: `HeartbeatT2J.can_fault_leg/_age_ms/_count` and
`CacheDiag.hb_frames[7]` already decode by field name off the generated
dataclass (same mechanism as `max_dev_leg` / `enc_frames`), so this unit's
codec work was tests, not code — `teensy_link/protocol.py` only needed
`HEARTBEAT_HB_STALE_SHIFT` added to its re-export list (`HEARTBEAT_TORQUE_CLAMP_SHIFT`'s
precedent).

* **`/link_status`** gains `can_fault_leg` (int, 255 = none), `can_fault_age_ms`,
  `can_fault_count`, `hb_stale_mask` (the raw 7-bit int) and `hb_stale_axes` (a
  readable `"0,2,6"` / `"-"` list) — mirroring the `max_dev_leg` /
  `torque_clamp_mask` KeyValue style. A new `diag_hb_stale_legs` key surfaces
  per-axis `DiagnosticPayload.flags` bit 0 for legs 0-5 (freshness-gated the
  same way `_log_odrive_errors` already is), which is the leg equivalent of the
  BB 7/8 gate in `_bb_motor_states_for_robot_state`.
* **The CAN_BUS_DOWN edge ERROR line is now factual.** It no longer tells the
  operator to run `/clear_errors` — there is nothing to clear — and instead
  names the leg (or "leg unknown" if `can_fault_leg == 0xFF`), the age and the
  trip count from the latch, and says the firmware self-clears and stows.
  Every other fault state (E-STOP, ODRIVE_FATAL, MAX_DEVIATION, …) is
  untouched, including the "Recover with: /clear_errors" wording — those are
  still latched-until-cleared. The "fault cleared" INFO line gets a
  CAN_BUS_DOWN-specific branch too, saying the firmware stowed and that no
  CLEAR_ERRORS was needed.
* **A throttled WARN** (≤ 1/5 s) fires while `HB_STALE_MASK` is nonzero, names
  the stale axes, and says plainly this is diagnostic-only.
* **`CacheDiag.hb_frames`** rides `/cache_diag` as `hb_frames_0..6`, right
  beside `enc_frames_0..6` (same raw/cumulative/differenced-by-consumer
  contract) — this one wasn't in the original follow-on's host-code list but
  is the direct "beside enc_frames" instrument that list called for.
* **`ALL_AXIS_HEARTBEATS_OK` consumer audit**: grepped `teensy_link/`,
  `ros_ws/src/jugglebot/jugglebot/` and `tests/` for
  `ALL_AXIS_HEARTBEATS_OK`/`all_axis_heartbeats_ok`. The only host hit is
  `_T2J_FLAG_ALL_AXIS_HB_OK = 0x4` in `teensy_bridge_node.py` — defined,
  never referenced anywhere else in that file. No live consumer reads it as
  the fatal predicate (or at all); nothing needed fixing.

Verification: `pytest tests/teensy_link tests/firmware/test_udp_protocol_xlang.py
tests/ros/test_teensy_bridge_node_*.py -q`, run 2026-09-15: **869 passed** (48
in `test_protocol_codec.py`, including the new CAN_BUS_DOWN trip-latch,
HB_STALE_MASK and `hb_frames` round-trip tests). `./run_tests.sh` not run —
parallel sessions are mid-edit on `motion/skills/`/`skill_node.py` in this
worktree (same reasoning as the firmware-side entry above).
