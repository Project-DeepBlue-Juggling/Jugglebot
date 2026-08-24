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
//  GROWABLE: the inbound-CAN3-frame injection hook is present from the harness's
//  first version even though nothing consumes it yet — the relay/version decode
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
  uint8_t  cls;   // the TxCls the caller declared (2026-08-24 tri-state accounting).
                  // Recorded so a test can assert not just WHAT went on the bus but
                  // which per-caller deferral bucket the firmware charged it to —
                  // the classification is a per-caller RULING, so a silent drift to
                  // the wrong bucket would mis-attribute exactly the pressure the
                  // census exists to attribute.
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

// ── Bridge wall-clock anchor (drives time_synced()) ──────────────────────────
//  Defaults to TRUE on fake_reset() — an anchored bridge is the steady state, and
//  a test that cares about the un-anchored case sets it false explicitly.
void fake_set_time_synced(bool synced);

// ── Cold-start move predicates (drive homing/activate/deactivate_active()) ───
void fake_set_homing(bool active);
void fake_set_activate(bool active);
void fake_set_deactivate(bool active);

// ── CAN3 command gate (drives jugglebot_commands_allowed()) ──────────────────
//  Defaults to TRUE on fake_reset() so a relay/leg send is recorded unless a test
//  explicitly down-gates the bus (the ERR_BUS_DOWN fail-fast path).
void fake_set_commands_allowed(bool allowed);

// ── Recording CAN3 TX (drives can_jugglebot_send()) ──────────────────────────
size_t            fake_sent_count();
const SentFrame&  fake_sent_at(size_t i);
void              fake_clear_sent();
// Convenience: count recorded frames whose ODrive command id (arb & 0x1F) matches.
size_t            fake_sent_count_cmd(uint8_t cmd_id);

// ── Send-outcome injection (drives can_jugglebot_tx()'s TxResult) ────────────
//  Fail the Nth can_jugglebot_tx ATTEMPT after this call (0-based): it returns
//  TxResult::FAILED and does NOT record the frame. FAILED is the bus-partner
//  presence gate refusing — the ONE outcome in which no frame reaches the wire — so
//  not recording it is the faithful model. -1 (the fake_reset default) = never.
//  Used by the hand_ops preamble-abort test: a failed preamble send must abort the
//  traj TX, so the 0x6D0 frame is never sent.
void              fake_set_send_fail_index(int attempt_index);

//  DEFER the Nth attempt (0-based): it returns TxResult::DEFERRED and IS RECORDED.
//  That asymmetry with the fail hook is the whole point of the tri-state — a
//  deferred frame went into the software txBuffer and transmits in order, so a
//  faithful fake must put it on the recorded bus. -1 (the default) = never.
void              fake_set_send_defer_index(int attempt_index);
//  Defer EVERY send (a saturated-mailbox bus). Composes with the fail index: a
//  FAILED attempt still fails.
void              fake_set_send_defer_all(bool defer_all);
//  How many recorded sends carried each TxCls, and the class of the most recent
//  send ATTEMPT (recorded or not). Both reset with fake_reset().
size_t            fake_sent_count_cls(uint8_t cls);
int               fake_last_tx_class();   // -1 ⇒ nothing attempted since the reset

// ── NOT HERE: leg_interp symbols (interp_last_tick_us & friends) ─────────────
//  A seam for a leg_interp symbol looks like it belongs in this file and MUST NOT
//  go here. test_fault_machine and test_leg_interp both #include the REAL
//  leg_interp.cpp *and* link fake_hal.o, so any leg_interp definition added here is
//  a duplicate-symbol link failure in those two binaries — the ODR rule stated at
//  build.py:16-24. Stub such symbols in the DRIVER that needs them instead
//  (test_hand_ops.cpp does this for interp_last_tick_us; test_udp_link.cpp does it
//  for micros64/net_ethernet_service). The link error is loud, so this note exists
//  to save the diagnosis, not to prevent a silent bug.

// ── GROWABLE inbound-CAN3 injection hook (initially unused; the relay/version tests use it) ─
//  A test pushes a frame the bridge's CAN3 RX would have received; the future
//  relay/version decode TU drains it via fake_can3_rx_pop(). The harness initially only
//  exercises push/pending so the seam is proven to compile and round-trip.
void   fake_inject_can3_rx(uint32_t id, const uint8_t* data, uint8_t len);
bool   fake_can3_rx_pop(uint32_t& id, uint8_t* data, uint8_t& len);  // consumer side
size_t fake_can3_rx_pending();

}  // namespace CanBridge
