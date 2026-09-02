#pragma once
// =============================================================================
//  hand_source.h — the hand-mastery interlock latch (unified-7dof FW 17)
// =============================================================================
//  plans/active/unified-7dof-planner.md § 2.4. ONE latched mode decides which
//  master may command the hand ODrive (axis 6):
//
//    LEGACY_STROKE (boot default) — the Platform Teensy stroke engine masters
//      the hand: HAND_TRAJ_CMD forwards on 0x6D0 as today, and the 500 Hz
//      interp DISCARDS Setpoint index 6 (counted, visible on [hand7]).
//    STREAMED — the can-bridge masters the hand: the interp emits the 7th
//      set_input_pos in the leg burst, and hand_ops::hand_traj_cmd REFUSES
//      with ERR_HAND_SOURCE.
//
//  WHY A FIRMWARE LATCH, NOT A HOST CONVENTION (root cause, plan § 6): the
//  2026-08-09/-08-13 arcs showed host-side conventions cannot prevent two
//  writers on one CAN id; only the single TX owner (this bridge) can make dual
//  mastery structurally impossible. Both code paths exist during the migration,
//  but never both masters.
//
//  SWITCHING is via the additive HAND_SOURCE_SET RPC only, and it is gated:
//  accepted only while !mpc_active AND the hand is settled at a rest position
//  (fresh axis-6 telemetry, |pos - rest| <= HAND_SETTLE_BAND_REV for rest in
//  {retract 0.0, catch-prime 9.9594}, |vel| <= HAND_SOURCE_SETTLE_VEL_RPS).
//  The gate closes the mid-motion-mastery-swap class: a switch during a stroke
//  would hand a moving axis to a master whose command timeline starts cold.
//
//  The latch survives CLEAR_ERRORS (it is a MODE, not a fault) and rides
//  HeartbeatT2J flags bit 6 (HAND_SOURCE_STREAMED) so /link_status displays it.
//
//  CONCURRENCY: the latch byte is written only from the RPC/net task
//  (hand_source_request) and read by the 500 Hz interp ISR, hand_ops (net
//  task), the fault task and the heartbeat task — a single naturally-aligned
//  volatile byte, atomic on Cortex-M7 (the s_output_enabled idiom).
// =============================================================================

#include <cstdint>

namespace CanBridge {

namespace HandSource {
constexpr uint8_t LEGACY_STROKE = 0u;   // boot default
constexpr uint8_t STREAMED      = 1u;
}

// Current latch value (HandSource::*). Boot default LEGACY_STROKE.
uint8_t hand_source();
inline bool hand_source_is(uint8_t v) { return hand_source() == v; }
bool hand_source_streamed();

// The gated switch — the HAND_SOURCE_SET RPC's single enforcement point.
// `mpc_active_now` is passed by the caller (rpc.cpp reads fault_mpc_active())
// so this TU stays free of fault_machine linkage (native-harness seam).
// Returns a JbUdp::RpcStatus value: OK (latched, or already there),
// ERR_BAD_ARGS (unknown source value), ERR_REJECTED (mpc_active, axis-6
// telemetry missing/stale, or the hand is not settled at a rest position).
uint16_t hand_source_request(uint8_t source, bool mpc_active_now);

// Test-isolation seam (native harness): return the latch to its power-on value.
// Production never calls it — a reboot is the only other path back to LEGACY.
void hand_source_reset();

}  // namespace CanBridge
