// =============================================================================
//  coldstart_hal.cpp — self-contained fake HAL for the cold-start move drivers
// =============================================================================
//  Defines the leaf symbols leg_homing/activate/deactivate.cpp need from TUs we
//  do not compile on the host (can_buses.cpp, time_base.cpp, fault_machine.cpp):
//  a controllable clock, a settable CAN3 bus-health + fault gate, and a RECORDING
//  can_jugglebot_send so a test asserts exactly which CAN3 frames the real ladder
//  emitted. It deliberately does NOT define the *_active() predicates (each driver
//  supplies its two siblings inline; the real one comes from the #included module)
//  and is NEVER linked alongside fake_hal.o. See coldstart_hal.h.
// =============================================================================

#include "coldstart_hal.h"

#include <vector>

#include "odrive_protocol.h"   // ODrive::CanFrame (return type of can_jugglebot_send)
#include "can_buses.h"         // CanStats + declares can_jugglebot_send/can_buses_stats/commands_allowed
#include "time_base.h"         // declares micros64 / now_wall_us
#include "fault_machine.h"     // declares fault_can_bus_down / fault_guard_mode
#include "udp_protocol.h"      // JbUdp::BusHealth / GuardMode

namespace CanBridge {

// ── State ────────────────────────────────────────────────────────────────────
static uint64_t g_mono_us = 0;
static uint8_t  g_health = JbUdp::BusHealth::OK;    // default: bus healthy → gate open
static bool     g_can_bus_down = false;
static bool     g_guard_estop = false;
static bool     g_commands_allowed = true;
static bool     g_mpc_active = false;              // MPC-stream interlock (reject cold-start ops while the MPC drives the legs); default: not driving
static bool     g_stow_pending = false;            // deferred-stow interlock (review fix); default: no stow
static std::vector<CsSentFrame> g_sent;
static int g_send_fail_index = -1;
static int g_send_attempts   = 0;

// ── Reset ────────────────────────────────────────────────────────────────────
void cs_reset() {
  g_mono_us = 0;
  g_health = JbUdp::BusHealth::OK;
  g_can_bus_down = false;
  g_guard_estop = false;
  g_commands_allowed = true;
  g_mpc_active = false;
  g_stow_pending = false;
  g_send_fail_index = -1;
  g_send_attempts = 0;
  g_sent.clear();
}

// ── Clock ────────────────────────────────────────────────────────────────────
void cs_advance(uint64_t delta_us) { g_mono_us += delta_us; }
void cs_set_mono(uint64_t mono_us) { g_mono_us = mono_us; }
uint64_t micros64()    { return g_mono_us; }   // HAL: time_base.h
uint64_t now_wall_us() { return g_mono_us; }   // HAL: time_base.h (cold-start uses only micros64)

// ── Bus / fault gate ─────────────────────────────────────────────────────────
// Setting the reported health ALSO moves the command gate to the verdict the
// real firmware would reach for that bus state. Before 2026-07-29 the cold-start
// gates re-derived their bus term from can_buses_stats().jugglebot_health, so a
// test could drive them with this setter alone; they now call the shared
// jugglebot_commands_allowed() (one enforcement point — classify_command_gate in
// can_buses.h). Coupling the two knobs here keeps those tests honest instead of
// silently passing against a gate nobody set.
//
// The coupling is a DEFAULT, not an invariant: production has two classifiers
// that agree on every cell except passive-but-not-yet-sustained, where health
// reports WARN while the gate still allows. A test modelling that cell calls
// cs_set_commands_allowed() AFTER this setter to force the disagreement.
void cs_set_jugglebot_health(uint8_t health) {
  g_health = health;
  g_commands_allowed = (health != JbUdp::BusHealth::WARN &&
                        health != JbUdp::BusHealth::BUS_OFF);
}
void cs_set_can_bus_down(bool down)          { g_can_bus_down = down; }
void cs_set_guard_estop(bool estop)          { g_guard_estop = estop; }
void cs_set_commands_allowed(bool allowed)   { g_commands_allowed = allowed; }
void cs_set_mpc_active(bool active)          { g_mpc_active = active; }
void cs_set_stow_pending(bool pending)       { g_stow_pending = pending; }

CanStats can_buses_stats() {                  // HAL: can_buses.h
  CanStats s{};
  s.jugglebot_health = g_health;
  return s;
}
bool jugglebot_commands_allowed() { return g_commands_allowed; }   // HAL: can_buses.h
bool fault_can_bus_down() { return g_can_bus_down; }               // HAL: fault_machine.h
uint8_t fault_guard_mode() {                                       // HAL: fault_machine.h
  return g_guard_estop ? JbUdp::GuardMode::ESTOP : JbUdp::GuardMode::ENABLED;
}
bool fault_mpc_active() { return g_mpc_active; }                    // HAL: fault_machine.h (MPC-stream interlock)
bool fault_stow_pending() { return g_stow_pending; }                // HAL: fault_machine.h (deferred-stow interlock)

// ── Recording CAN3 TX ────────────────────────────────────────────────────────
void cs_set_send_fail_index(int attempt_index) {
  g_send_fail_index = attempt_index;
  g_send_attempts = 0;
}
bool can_jugglebot_send(const ODrive::CanFrame& f) {   // HAL: can_buses.h
  const int attempt = g_send_attempts++;
  if (attempt == g_send_fail_index) return false;   // TX-enqueue failure — not recorded
  CsSentFrame s;
  s.id = f.id;
  s.len = f.len;
  for (int i = 0; i < 8; ++i) s.buf[i] = f.buf[i];
  g_sent.push_back(s);
  return true;
}
size_t             cs_sent_count() { return g_sent.size(); }
const CsSentFrame& cs_sent_at(size_t i) { return g_sent.at(i); }
void               cs_clear_sent() { g_sent.clear(); }

}  // namespace CanBridge
