#pragma once
// =============================================================================
//  coldstart_hal.h — control surface for the cold-start-move test fake HAL
// =============================================================================
//  The three cold-start move TUs (leg_homing.cpp / leg_activate.cpp /
//  leg_deactivate.cpp) reach a DIFFERENT set of leaf symbols than the fault/interp
//  TUs the main fake_hal serves: their bus/fault gate reads
//  can_buses_stats().jugglebot_health + fault_can_bus_down() + fault_guard_mode()
//  (NOT jugglebot_commands_allowed()), and each DEFINES its own *_active() (which
//  would ODR-clash with fake_hal's fake predicates). So they get this SEPARATE,
//  self-contained fake — linked ONLY by the cold-start driver binaries, never
//  alongside fake_hal.o — plus each driver supplies the two SIBLING *_active()
//  predicates inline (the one it excludes is the real module under test).
//
//  Scope: DECISION LOGIC only (request validation, the SETUP/COMMAND frame
//  emission, abort-to-IDLE), not FreeRTOS/ISR concurrency or 500 Hz timing.
// =============================================================================

#include <cstdint>
#include <cstddef>

namespace CanBridge {

// One recorded CAN3 transmit (mirrors fake_hal::SentFrame + ODrive::CanFrame).
struct CsSentFrame {
  uint32_t id;
  uint8_t  len;
  uint8_t  buf[8];
};

// Whole-fake reset — call at the top of every TEST_CASE (with the module's
// *_init()) so file-scope state cannot leak across cases. Defaults: bus HEALTHY
// (jugglebot_health = OK), no CAN-down, guard ENABLED (not E-STOP), commands
// allowed, no send failures, clock 0, recording cleared.
void cs_reset();

// ── Controllable clock (drives micros64() / now_wall_us()) ───────────────────
void cs_advance(uint64_t delta_us);
void cs_set_mono(uint64_t mono_us);

// ── Bus / fault gate (drive *_allowed() in the cold-start modules) ───────────
void cs_set_jugglebot_health(uint8_t health);   // JbUdp::BusHealth → can_buses_stats()
void cs_set_can_bus_down(bool down);            // → fault_can_bus_down()
void cs_set_guard_estop(bool estop);            // → fault_guard_mode()==ESTOP
void cs_set_commands_allowed(bool allowed);     // → jugglebot_commands_allowed()

// ── Recording CAN3 TX (drives can_jugglebot_send()) ──────────────────────────
size_t             cs_sent_count();
const CsSentFrame& cs_sent_at(size_t i);
void               cs_clear_sent();
// Fail the Nth can_jugglebot_send ATTEMPT after this call (0-based); -1 = never.
// A failed send returns false and is NOT recorded (never reached the bus).
void               cs_set_send_fail_index(int attempt_index);

}  // namespace CanBridge
