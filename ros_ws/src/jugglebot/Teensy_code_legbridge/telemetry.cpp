// =============================================================================
//  telemetry.cpp — motor-state uplink + on-change diagnostics
// =============================================================================
#include "telemetry.h"

#include <cstring>
#include <cmath>
#include "legbridge_config.h"
#include "udp_protocol.h"
#include "udp_link.h"
#include "axis_state.h"
#include "time_base.h"

namespace LegBridge {

// ── On-change thresholds (only emit a Diagnostic when one is exceeded) ────────
static constexpr float DIAG_IQ_THRESH_A     = 0.5f;
static constexpr float DIAG_TEMP_THRESH_C    = 1.0f;
static constexpr float DIAG_VOLT_THRESH_V    = 0.5f;
static constexpr uint32_t DIAG_FORCE_PERIOD_US = 1000000u;   // 1 Hz per-axis refresh

// Last-published diagnostic snapshot, per axis, for change detection.
struct DiagBaseline {
  uint32_t active_errors, disarm_reason;
  uint8_t  axis_state, ctrl_mode, input_mode, flags;
  float    iq_setpoint, temp_fet, temp_motor, bus_voltage;
  uint64_t last_sent_us;
  bool     ever_sent;
};
static DiagBaseline s_base[NUM_AXES];

void telemetry_init() {
  for (uint8_t i = 0; i < NUM_AXES; ++i) s_base[i] = DiagBaseline{};
}

// ── 100 Hz motor-state frame ──────────────────────────────────────────────────
static void send_telemetry() {
  JbUdp::TelemetryPayload t{};
  t.t_teensy_us = now_wall_us();
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    float pos, vel; uint64_t ts;
    snapshot_pos_vel(axes[i], pos, vel, ts);
    t.pos_rev[i] = pos;
    t.vel_rps[i] = vel;
  }
  udp_send_stream(JbUdp::MsgType::TELEMETRY, (const uint8_t*)&t, sizeof(t));
}

// ── Per-axis diagnostic (on-change or 1 Hz) ───────────────────────────────────
static bool diag_changed(const AxisState& a, const DiagBaseline& b) {
  if (!b.ever_sent) return true;
  if (a.active_errors != b.active_errors) return true;
  if (a.disarm_reason != b.disarm_reason) return true;
  if (a.axis_state != b.axis_state) return true;
  if (a.controller_mode != b.ctrl_mode) return true;
  if (a.input_mode != b.input_mode) return true;
  if (fabsf(a.iq_setpoint - b.iq_setpoint) > DIAG_IQ_THRESH_A) return true;
  if (fabsf(a.temp_fet   - b.temp_fet)   > DIAG_TEMP_THRESH_C) return true;
  if (fabsf(a.temp_motor - b.temp_motor) > DIAG_TEMP_THRESH_C) return true;
  if (fabsf(a.bus_voltage - b.bus_voltage) > DIAG_VOLT_THRESH_V) return true;
  return false;
}

static void send_diag(uint8_t axis) {
  AxisState& a = axes[axis];
  const uint64_t now = now_wall_us();
  const bool stale = a.heartbeat_seen &&
                     (now - a.last_heartbeat_us > CAN_HEARTBEAT_TIMEOUT_US);

  JbUdp::DiagnosticPayload d{};
  d.axis_id       = axis;
  d.axis_state    = a.axis_state;
  d.ctrl_mode     = a.controller_mode;
  d.input_mode    = a.input_mode;
  d.flags         = stale ? 0x1u : 0x0u;
  d.active_errors = a.active_errors;
  d.disarm_reason = a.disarm_reason;
  d.iq_setpoint   = a.iq_setpoint;
  d.iq_measured   = a.iq_measured;
  d.temp_fet      = a.temp_fet;
  d.temp_motor    = a.temp_motor;
  d.bus_voltage   = a.bus_voltage;
  udp_send_stream(JbUdp::MsgType::DIAGNOSTIC, (const uint8_t*)&d, sizeof(d));

  DiagBaseline& b = s_base[axis];
  b.active_errors = a.active_errors; b.disarm_reason = a.disarm_reason;
  b.axis_state = a.axis_state; b.ctrl_mode = a.controller_mode; b.input_mode = a.input_mode;
  b.iq_setpoint = a.iq_setpoint; b.temp_fet = a.temp_fet;
  b.temp_motor = a.temp_motor; b.bus_voltage = a.bus_voltage;
  b.flags = d.flags;
  b.last_sent_us = now;
  b.ever_sent = true;
}

void telemetry_step() {
  send_telemetry();

  // Stagger the 1 Hz forced refresh across axes so the forced frames never
  // burst together; changed axes are sent immediately regardless of slot.
  static uint32_t tick = 0;
  const uint32_t slot = tick % TELEM_RATE_HZ;              // 0..99, ticks within the second
  const uint32_t slots_per_axis = TELEM_RATE_HZ / NUM_AXES;  // ~14
  const uint64_t now = now_wall_us();
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    const bool due_forced =
        (now - s_base[i].last_sent_us >= DIAG_FORCE_PERIOD_US) &&
        (slot == (uint32_t)i * slots_per_axis);
    if (diag_changed(axes[i], s_base[i]) || due_forced) {
      send_diag(i);
    }
  }
  ++tick;
}

}  // namespace LegBridge
