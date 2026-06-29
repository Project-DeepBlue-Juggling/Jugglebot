#pragma once
// =============================================================================
//  fake_hal.h — control surface for the native firmware test harness's fake HAL
// =============================================================================
//  The compiled safety-critical TUs (fault_machine.cpp, leg_interp.cpp) reach a
//  handful of LEAF symbols from OTHER translation units we do not compile on the
//  host (can_buses.cpp, time_base.cpp, udp_link.cpp, leg_homing/activate/
//  deactivate.cpp). fake_hal.cpp DEFINES those leaf symbols with a controllable
//  clock and a recording CAN3 TX so a test can drive the real decision logic and
//  observe every CAN frame it would emit. This header is the test-side control
//  surface (clock setters, the recorded-frame accessor, the predicate setters,
//  and the GROWABLE inbound-CAN3 injection hook).
//
//  GROWABLE: the inbound-CAN3-frame injection hook is present from Phase 0 even
//  though nothing consumes it yet — Phases 1/3 compile the relay/version decode
//  path into the harness and use it for reply-correlation + the Get_Version
//  handshake tests. Adding the seam now keeps those later phases additive.
//
//  Scope: this validates DECISION LOGIC, not FreeRTOS/ISR concurrency or 500 Hz
//  timing (those remain on-hardware-replay gaps). See README.md.
// =============================================================================

#include <cstdint>
#include <cstddef>

namespace CanBridge {

// One recorded CAN3 transmit (what the firmware would have put on the Jugglebot
// core bus). Layout mirrors ODrive::CanFrame so a test can decode id/buf.
struct SentFrame {
  uint32_t id;
  uint8_t  len;
  uint8_t  buf[8];
};

// ── Whole-harness reset (call between test cases, with fault_machine_init() and
//    interp_reset()) so file-scope fake state can neither fabricate nor mask a
//    result via test ordering. ─────────────────────────────────────────────────
void fake_reset();

// ── Controllable clock (drives now_wall_us() and micros64()) ─────────────────
void     fake_set_clock(uint64_t wall_us, uint64_t mono_us);
void     fake_advance(uint64_t delta_us);          // advance both clocks together
uint64_t fake_wall_us();
uint64_t fake_mono_us();

// ── Jetson UDP link freshness (drives udp_last_rx_us()) ──────────────────────
void fake_set_udp_last_rx_us(uint64_t t_us);

// ── Cold-start move predicates (drive homing/activate/deactivate_active()) ───
void fake_set_homing(bool active);
void fake_set_activate(bool active);
void fake_set_deactivate(bool active);

// ── Recording CAN3 TX (drives can_jugglebot_send()) ──────────────────────────
size_t            fake_sent_count();
const SentFrame&  fake_sent_at(size_t i);
void              fake_clear_sent();
// Convenience: count recorded frames whose ODrive command id (arb & 0x1F) matches.
size_t            fake_sent_count_cmd(uint8_t cmd_id);

// ── GROWABLE inbound-CAN3 injection hook (Phase 0: unused; Phases 1/3 use it) ─
//  A test pushes a frame the bridge's CAN3 RX would have received; the future
//  relay/version decode TU drains it via fake_can3_rx_pop(). Phase 0 only
//  exercises push/pending so the seam is proven to compile and round-trip.
void   fake_inject_can3_rx(uint32_t id, const uint8_t* data, uint8_t len);
bool   fake_can3_rx_pop(uint32_t& id, uint8_t* data, uint8_t& len);  // consumer side
size_t fake_can3_rx_pending();

}  // namespace CanBridge
