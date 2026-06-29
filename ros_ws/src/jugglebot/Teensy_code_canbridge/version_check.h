#pragma once
// =============================================================================
//  version_check.h — Phase 3 firmware Get_Version sweep + raw-version cache
// =============================================================================
//  Restores can_node's firmware-version handshake (can_node._handle_heartbeat
//  :325-329 + _send_next_version_query :341-349 + _handle_get_version :474-495)
//  on the can-bridge, so the orchestrator's BOOT no longer wedges on a
//  hardcoded firmware_validated=False (state_machine.py:232).
//
//  Split of responsibility (ZERO version SEMANTICS in firmware — the validation
//  policy stays in tested Python, motor_state.validate_group):
//    * FIRMWARE (here): once a Jugglebot axis has heartbeated, send it ONE
//      Get_Version (one frame per cold-start-monitor tick → bus-paced, ≤7 frames
//      total), cache the raw 8-byte reply, and expose the whole cache + a
//      received bitmask via the GET_AXIS_VERSIONS RPC. The firmware never parses
//      or compares versions.
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

// Diagnostics / test accessors.
uint8_t version_received_mask();    // bit i ⇒ axis i's Get_Version reply cached
uint8_t version_query_sent_mask();  // bit i ⇒ Get_Version sent to axis i

}  // namespace CanBridge
