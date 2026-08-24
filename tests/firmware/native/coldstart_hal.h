#pragma once
// =============================================================================
//  coldstart_hal.h — control surface for the cold-start-move test fake HAL
// =============================================================================
//  The three cold-start move TUs (leg_homing.cpp / leg_activate.cpp /
//  leg_deactivate.cpp) reach a DIFFERENT set of leaf symbols than the fault/interp
//  TUs the main fake_hal serves: their bus/fault gate reads
//  jugglebot_commands_allowed() + fault_can_bus_down() + fault_guard_mode(), and
//  each DEFINES its own *_active() (which would ODR-clash with fake_hal's fake
//  predicates).
//
//  (Until 2026-07-29 those three gates each re-derived the bus term from
//  can_buses_stats().jugglebot_health instead of calling the shared predicate —
//  three private copies that kept refusing on a transient error-passive blip after
//  the shared gate had stopped. They now call jugglebot_commands_allowed(); see
//  classify_command_gate in can_buses.h. can_buses_stats() is still faked here
//  because the modules read other CanStats fields.) So they get this SEPARATE,
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
// Sets can_buses_stats().jugglebot_health AND, as a default, moves
// jugglebot_commands_allowed() to the matching verdict (WARN/BUS_OFF ⇒ refused).
// Call cs_set_commands_allowed() AFTER this one to model the single cell where
// production disagrees: passive-but-not-yet-sustained reports WARN while the
// command gate still allows (classify_command_gate, can_buses.h).
void cs_set_jugglebot_health(uint8_t health);   // JbUdp::BusHealth → can_buses_stats()
void cs_set_can_bus_down(bool down);            // → fault_can_bus_down()
void cs_set_guard_estop(bool estop);            // → fault_guard_mode()==ESTOP
void cs_set_commands_allowed(bool allowed);     // → jugglebot_commands_allowed()
void cs_set_mpc_active(bool active);            // → fault_mpc_active() (MPC-stream interlock)
void cs_set_stow_pending(bool pending);        // → fault_stow_pending() (deferred-stow interlock, review fix)

// ── Recording CAN3 TX (drives can_jugglebot_tx()) ────────────────────────────
size_t             cs_sent_count();
const CsSentFrame& cs_sent_at(size_t i);
void               cs_clear_sent();
// Fail the Nth can_jugglebot_tx ATTEMPT after this call (0-based); -1 = never.
// FAILED models the bus-partner presence gate refusing: nothing is queued, so the
// frame is NOT recorded and the bool wrapper the cold-start ladders use reads false.
void               cs_set_send_fail_index(int attempt_index);
// DEFER the Nth attempt (0-based): TxResult::DEFERRED, and the frame IS recorded —
// it went into the software txBuffer and transmits in order, so the ladders' bool
// wrapper reads TRUE and a ladder must NOT abort on it. -1 (the default) = never.
void               cs_set_send_defer_index(int attempt_index);

}  // namespace CanBridge
