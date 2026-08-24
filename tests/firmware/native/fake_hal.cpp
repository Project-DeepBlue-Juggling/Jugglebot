// =============================================================================
//  fake_hal.cpp — fake HAL for the native firmware test harness
// =============================================================================
//  Defines the leaf symbols fault_machine.cpp / leg_interp.cpp need from TUs we
//  do not compile on the host: a controllable clock (now_wall_us / micros64),
//  the Jetson-link freshness (udp_last_rx_us), the cold-start move predicates
//  (homing/activate/deactivate_active), and a RECORDING can_jugglebot_send so a
//  test can assert exactly which CAN3 frames the real logic emitted (the
//  never-command-a-dead-bus invariant is checked directly against this record).
//
//  See fake_hal.h for the test-side control surface and README.md for the seam.
// =============================================================================

#include "fake_hal.h"

#include <deque>
#include <vector>

#include "odrive_protocol.h"   // ODrive::CanFrame (return type of can_jugglebot_send)
#include "can_buses.h"         // declares can_jugglebot_send (we define it here)
#include "time_base.h"         // declares now_wall_us / micros64 (we define them here)
#include "udp_link.h"          // declares udp_last_rx_us (we define it here)

namespace CanBridge {

// ── State ────────────────────────────────────────────────────────────────────
static uint64_t g_wall_us = 0;
static uint64_t g_mono_us = 0;
static uint64_t g_udp_last_rx_us = 0;
static bool     g_time_synced = true;   // bridge wall anchor present (steady state)
static bool     g_homing = false, g_activate = false, g_deactivate = false;
static bool     g_commands_allowed = true;   // CAN3 gate; default open (see fake_hal.h)
static std::vector<SentFrame>      g_sent;
static std::deque<SentFrame>       g_can3_rx;   // inbound injection FIFO (growable hook)
static int  g_send_fail_index  = -1;   // can_jugglebot_tx attempt (0-based) that FAILS; -1 = never
static int  g_send_defer_index = -1;   // attempt (0-based) that DEFERS (and is recorded); -1 = never
static bool g_send_defer_all   = false;
static int  g_send_attempts    = 0;    // attempts since the last reset / index set
static int  g_last_tx_class    = -1;

// ── Reset ────────────────────────────────────────────────────────────────────
void fake_reset() {
  g_wall_us = 0; g_mono_us = 0; g_udp_last_rx_us = 0;
  g_time_synced = true;
  g_homing = g_activate = g_deactivate = false;
  g_commands_allowed = true;
  g_send_fail_index = -1;
  g_send_defer_index = -1;
  g_send_defer_all = false;
  g_send_attempts = 0;
  g_last_tx_class = -1;
  g_sent.clear();
  g_can3_rx.clear();
}

// ── Clock ────────────────────────────────────────────────────────────────────
void     fake_set_clock(uint64_t wall_us, uint64_t mono_us) { g_wall_us = wall_us; g_mono_us = mono_us; }
void     fake_advance(uint64_t delta_us) { g_wall_us += delta_us; g_mono_us += delta_us; }
uint64_t fake_wall_us() { return g_wall_us; }
uint64_t fake_mono_us() { return g_mono_us; }

// HAL: time_base.h
uint64_t now_wall_us() { return g_wall_us; }
uint64_t micros64()    { return g_mono_us; }

void fake_set_time_synced(bool synced) { g_time_synced = synced; }
bool time_synced() { return g_time_synced; }   // HAL: time_base.h

// ── Jetson link freshness ────────────────────────────────────────────────────
void     fake_set_udp_last_rx_us(uint64_t t_us) { g_udp_last_rx_us = t_us; }
uint64_t udp_last_rx_us() { return g_udp_last_rx_us; }   // HAL: udp_link.h

// ── Cold-start move predicates ───────────────────────────────────────────────
void fake_set_homing(bool active)     { g_homing = active; }
void fake_set_activate(bool active)   { g_activate = active; }
void fake_set_deactivate(bool active) { g_deactivate = active; }
bool homing_active()     { return g_homing; }       // HAL: leg_homing.h
bool activate_active()   { return g_activate; }     // HAL: leg_activate.h
bool deactivate_active() { return g_deactivate; }   // HAL: leg_deactivate.h

// ── CAN3 command gate ────────────────────────────────────────────────────────
void fake_set_commands_allowed(bool allowed) { g_commands_allowed = allowed; }
bool jugglebot_commands_allowed() { return g_commands_allowed; }   // HAL: can_buses.h

// Fail the Nth send ATTEMPT after this call (0-based); -1 = never. Resets the
// attempt counter so the index is relative to sends made after the setter.
void fake_set_send_fail_index(int attempt_index) {
  g_send_fail_index = attempt_index;
  g_send_attempts = 0;
}
void fake_set_send_defer_index(int attempt_index) {
  g_send_defer_index = attempt_index;
  g_send_attempts = 0;
}
void fake_set_send_defer_all(bool defer_all) { g_send_defer_all = defer_all; }

// ── Recording CAN3 TX ────────────────────────────────────────────────────────
TxResult can_jugglebot_tx(const ODrive::CanFrame& f, uint8_t cls) {   // HAL: can_buses.h
  const int attempt = g_send_attempts++;
  g_last_tx_class = (int)cls;
  // FAILED models the bus-partner presence gate refusing: nothing was queued and
  // nothing reaches the wire, so the frame is NOT recorded.
  if (attempt == g_send_fail_index) return TxResult::FAILED;
  SentFrame s;
  s.id = f.id;
  s.len = f.len;
  for (int i = 0; i < 8; ++i) s.buf[i] = f.buf[i];
  s.cls = cls;
  g_sent.push_back(s);
  // DEFERRED is recorded: the frame is in the software txBuffer and transmits in
  // order. A fake that withheld it here would encode the very error the tri-state
  // exists to correct.
  if (g_send_defer_all || attempt == g_send_defer_index) return TxResult::DEFERRED;
  return TxResult::MAILBOX;
}
int fake_last_tx_class() { return g_last_tx_class; }
size_t fake_sent_count_cls(uint8_t cls) {
  size_t n = 0;
  for (const auto& s : g_sent) if (s.cls == cls) ++n;
  return n;
}
size_t           fake_sent_count() { return g_sent.size(); }
const SentFrame& fake_sent_at(size_t i) { return g_sent.at(i); }
void             fake_clear_sent() { g_sent.clear(); }
size_t fake_sent_count_cmd(uint8_t cmd_id) {
  size_t n = 0;
  for (const auto& s : g_sent) if ((s.id & 0x1Fu) == cmd_id) ++n;
  return n;
}

// ── GROWABLE inbound-CAN3 injection hook (not yet consumed by any decode path) ─
void fake_inject_can3_rx(uint32_t id, const uint8_t* data, uint8_t len) {
  SentFrame r; r.id = id; r.len = len; r.cls = TxCls::OTHER;
  for (int i = 0; i < 8; ++i) r.buf[i] = (i < len && data) ? data[i] : 0;
  g_can3_rx.push_back(r);
}
bool fake_can3_rx_pop(uint32_t& id, uint8_t* data, uint8_t& len) {
  if (g_can3_rx.empty()) return false;
  const SentFrame r = g_can3_rx.front(); g_can3_rx.pop_front();
  id = r.id; len = r.len;
  if (data) for (int i = 0; i < 8; ++i) data[i] = r.buf[i];
  return true;
}
size_t fake_can3_rx_pending() { return g_can3_rx.size(); }

}  // namespace CanBridge

// ── Arduino time fns (declared by the Arduino.h shim; unused by the TUs) ─────
unsigned long micros() { return 0; }
unsigned long millis() { return 0; }
