#pragma once
// =============================================================================
//  version_check.h — firmware Get_Version sweep + raw-version cache
// =============================================================================
//  Restores can_node's firmware-version handshake (can_node._handle_heartbeat
//  :325-329 + _send_next_version_query :341-349 + _handle_get_version :474-495)
//  on the can-bridge, so the orchestrator's BOOT no longer wedges on a
//  hardcoded firmware_validated=False (state_machine.py:232).
//
//  Split of responsibility (the validation POLICY stays in tested Python,
//  motor_state.validate_group):
//    * FIRMWARE (here): once a Jugglebot axis has heartbeated, send it ONE
//      Get_Version (one frame per cold-start-monitor tick → bus-paced, ≤7 frames
//      total), cache the raw 8-byte reply, and expose the whole cache + a
//      received bitmask via the GET_AXIS_VERSIONS RPC. This TU still parses and
//      compares nothing.
//    * AMENDMENT (2026-07-29, hand ball sensor): the firmware now performs
//      EXACTLY ONE version compare, in gpio_poll.cpp — the cached fw triple for
//      the hand axis vs JBBallDetect::EXPECTED_FW, gating the ball-sensor poller
//      (and reading the cache through version_raw_copy below). Root cause, not
//      convenience: ODrive endpoint ids are firmware-build-specific, and the
//      wrong-build id ANSWERS PLAUSIBLY (700 on a Pro 0.6.11 is
//      encoder_estimator1.status — a live-looking sensor that never changes,
//      with no timeout to diagnose it), so the refusal must happen BEFORE the
//      RxSdo leaves the Teensy. Only firmware can do that. Everything else —
//      the expected-version registry, the hw check, the pass/fail latch — stays
//      in Python.
//    * JETSON (teensy_bridge_node): pull the blob ONCE via GET_AXIS_VERSIONS,
//      decode the set-bit axes (jugglebot.can.odrive.decode_get_version) and run
//      MotorStateTracker.validate_group against EXPECTED_HW_VERSIONS, latching
//      firmware_validated / the mismatch string.
//
//  Why a SWEEP + cache (not a per-pull CAN3 round-trip): the Jetson's pull is a
//  cheap UDP RPC that reads a bridge-LOCAL cache — no CAN3 round-trip on the
//  pull. The firmware fills that cache asynchronously as the ODrives heartbeat,
//  exactly as can_node queued + drained its Get_Version timer.
//
//  Concurrency / determinacy: version_check_step() runs in the cold-start MONITOR
//  task (task_homing, HOMING_RATE_HZ) — NOT the safety-critical fault task. No
//  blocking, no unbounded loops, at most one CAN3 TX per tick. version_record()
//  runs in the CAN3 RX decode context; version_fill_blob() in the net/RPC
//  context. Each axis's 8 bytes are written once (versions are constant) and the
//  received bit is set AFTER the bytes (memory barrier), so a reader that sees a
//  set bit always sees valid bytes — single-byte volatile mask accesses are
//  atomic on Cortex-M7, so no seqlock is needed (cf. axis_state.h).
// =============================================================================

#include <cstdint>

namespace CanBridge {

// Init / re-arm: clears the received + query-sent bitmasks. Called once from
// setup() (file-statics already boot zeroed; this is also a test-isolation seam).
void version_check_init();

// Cold-start monitor tick (HOMING_RATE_HZ, never an ISR). Sends ONE Get_Version
// to the next present-but-unqueried Jugglebot axis (one frame/tick → bus-paced),
// gated on jugglebot_commands_allowed(). No-op once every present axis is queried.
void version_check_step();

// CAN3 RX decode seam: cache a Get_Version reply's raw 8 bytes for `axis` and set
// its received bit. Idempotent (versions are constant). Called from
// can_buses.cpp decode_into_cache's get_version case.
void version_record(uint8_t axis, const uint8_t* d8);

// GET_AXIS_VERSIONS RPC result: pack ResultAxisVersions (received_mask + the raw
// 8-byte payload per axis, axis-major) into `out`. Returns the byte count, or 0
// if `cap` is too small. Reads the cache; never touches CAN3.
uint16_t version_fill_blob(uint8_t* out, uint16_t cap);

// Copy one axis's cached raw 8-byte Get_Version reply into `out8`. Returns false
// (leaving `out8` untouched) when that axis's reply has not been received — the
// received bit IS the validity gate. The caller does the decoding; see the
// amendment note above for gpio_poll's one legitimate compare.
bool version_raw_copy(uint8_t axis, uint8_t* out8);

// Diagnostics / test accessors.
uint8_t version_received_mask();    // bit i ⇒ axis i's Get_Version reply cached
uint8_t version_query_sent_mask();  // bit i ⇒ Get_Version sent to axis i

// ── Ball Butler ODrives (CAN1 axes 7-8) ─────────────────────────────────────
//  The BB twin of everything above, restoring the half of can_node's BOOT
//  firmware check that commit 5875531 dropped. That cutover moved BB from
//  can_node (USB-CAN, now physically removed) to teensy_bridge_node and
//  deferred BB ODrive validation to a "phase B ... by decoding axes 7+8 on CAN1
//  and surfacing the result via teensy_bridge_node (a new T2J flag or RPC)".
//  The RX decode landed; this is the version half, arriving as the new RPC that
//  message anticipated.
//
//  SEPARATE STATE, SEPARATE BLOB, SEPARATE SWEEP — not a widening of the
//  Jugglebot arrays above:
//    * the wire blob is a fixed NUM_AXES*8 array, so growing it would be an
//      INCOMPATIBLE change (PROTOCOL_VERSION bump + lockstep flash) to add a
//      display row; a new method is additive and costs neither.
//    * the two sweeps ride different buses with different presence gates
//      (jugglebot_commands_allowed() vs can_bb_tx's own partner_recent), and a
//      dead BB must never stall the Jugglebot sweep or vice versa.
//  Concurrency/determinacy is identical: bb_version_check_step() runs in the
//  cold-start MONITOR task, at most one CAN1 TX per tick; bb_version_record()
//  runs in the CAN1 RX decode context; the received bit is published AFTER the
//  bytes (same barrier), so a set bit always implies valid bytes.

// Init / re-arm: clears the BB received + query-sent bitmasks.
void bb_version_check_init();

// Cold-start monitor tick. Sends ONE Get_Version to the next present-but-
// unqueried BB axis on CAN1 (one frame/tick → bus-paced), then re-queries one
// unreceived axis per interval until every present axis has replied. can_bb_tx
// carries its own partner-presence gate, so an absent Ball Butler is a no-op
// rather than an un-ACKed TX climbing the FlexCAN TEC.
void bb_version_check_step();

// CAN1 RX decode seam: cache a BB Get_Version reply's raw 8 bytes for absolute
// node id `axis` (7 or 8) and set its received bit. Idempotent. Called from
// can_buses.cpp decode_bb_odrive's get_version case.
void bb_version_record(uint8_t axis, const uint8_t* d8);

// GET_BB_AXIS_VERSIONS RPC result: pack ResultBbAxisVersions (received_mask +
// the raw 8-byte payload per BB axis, axis-major from BB_FIRST_NODE) into
// `out`. Returns the byte count, or 0 if `cap` is too small. Never touches CAN1.
uint16_t bb_version_fill_blob(uint8_t* out, uint16_t cap);

// Diagnostics / test accessors. Bit i ⇒ the i-th BB axis (BB_FIRST_NODE + i).
uint8_t bb_version_received_mask();
uint8_t bb_version_query_sent_mask();

}  // namespace CanBridge
