// =============================================================================
//  can_buses.cpp — dual FlexCAN_T4 + RX decode + TX
// =============================================================================
#include "can_buses.h"

#include <FlexCAN_T4.h>
#include "canbridge_config.h"
#include "protocol_config.h"
#include "axis_state.h"
#include "time_base.h"
#include "odrive_protocol.h"

namespace CanBridge {

// CAN1 (pins 22/23) shared; CAN2 (pins 1/0) leg bus.
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
static FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

static volatile uint32_t s_can1_rx = 0, s_can1_tx = 0, s_can2_rx = 0, s_can2_tx = 0;
static volatile uint64_t s_can1_last_rx_us = 0, s_can2_last_rx_us = 0;

// Decode an ODrive frame into the per-axis cache. Used by both buses.
static void decode_into_cache(const CAN_message_t& msg) {
  const uint8_t axis = ODrive::axis_of(msg.id);
  const uint8_t cmd  = ODrive::cmd_of(msg.id);
  if (axis >= NUM_AXES) return;            // not a leg/hand axis we cache
  if (msg.len < 8) return;                 // drop truncated frames (mirrors odrive.py _check_len;
                                           // every ODrive telemetry frame we decode is DLC 8)
  AxisState& a = axes[axis];
  const uint8_t* d = msg.buf;

  switch (cmd) {
    case ODriveCmd::heartbeat_message: {
      auto h = ODrive::decode_heartbeat(d);
      a.axis_state       = h.state;
      a.procedure_result = h.procedure_result;
      a.trajectory_done  = h.trajectory_done;
      a.last_heartbeat_us = now_wall_us();
      a.heartbeat_seen   = true;
      a.heartbeat_stale  = false;
      break;
    }
    case ODriveCmd::get_error: {
      auto e = ODrive::decode_error(d);
      a.active_errors = e.active_errors;
      a.disarm_reason = e.disarm_reason;
      break;
    }
    case ODriveCmd::get_encoder_estimate: {
      auto p = ODrive::decode_encoder_estimate(d);   // (pos, vel) in ODrive convention
      // Sign-flip legs to the Jugglebot convention (positive = extension),
      // mirroring can_node._handle_encoder's _leg_sign.
      const float pos = ODrive::leg_sign(axis, p.a);
      const float vel = ODrive::leg_sign(axis, p.b);
      write_pos_vel(a, pos, vel, now_wall_us());
      break;
    }
    case ODriveCmd::get_iq: {
      auto q = ODrive::decode_iq(d);
      a.iq_setpoint = q.a; a.iq_measured = q.b;
      break;
    }
    case ODriveCmd::get_temps: {
      auto t = ODrive::decode_temps(d);
      a.temp_fet = t.a; a.temp_motor = t.b;
      break;
    }
    case ODriveCmd::get_bus_voltage_current: {
      auto v = ODrive::decode_bus_voltage_current(d);
      a.bus_voltage = v.a; a.bus_current = v.b;
      break;
    }
    // get_version / TxSdo handled elsewhere (encoder-search Phase 9). Ignore here.
    default:
      break;
  }
}

static void on_can1_rx(const CAN_message_t& msg) {
  s_can1_rx++;
  s_can1_last_rx_us = now_wall_us();
  // On the shared bus we only care about hand-axis (node 6) telemetry for the
  // uplink; everything else (BB, cone, platform, our own 0x7DD) is ignored.
  if (ODrive::axis_of(msg.id) == HAND_AXIS) decode_into_cache(msg);
}

static void on_can2_rx(const CAN_message_t& msg) {
  s_can2_rx++;
  s_can2_last_rx_us = now_wall_us();
  decode_into_cache(msg);                  // leg axes 0..5
}

void can_buses_init() {
  can1.begin();
  can1.setBaudRate(CAN_BITRATE);
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(on_can1_rx);

  can2.begin();
  can2.setBaudRate(CAN_BITRATE);
  can2.setMaxMB(16);
  can2.enableFIFO();
  can2.enableFIFOInterrupt();
  can2.onReceive(on_can2_rx);
}

void can_buses_service() {
  can1.events();
  can2.events();
}

static bool send_on(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& bus,
                    const ODrive::CanFrame& f) {
  CAN_message_t m;
  m.id = f.id;
  m.len = f.len;
  m.flags.extended = 0;
  memcpy(m.buf, f.buf, 8);
  return bus.write(m) > 0;
}
// (CAN2 has a distinct template type, so a small overload rather than a template.)
static bool send_on(FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>& bus,
                    const ODrive::CanFrame& f) {
  CAN_message_t m;
  m.id = f.id;
  m.len = f.len;
  m.flags.extended = 0;
  memcpy(m.buf, f.buf, 8);
  return bus.write(m) > 0;
}

bool can1_send(const ODrive::CanFrame& f) {
  const bool ok = send_on(can1, f);
  if (ok) s_can1_tx++;
  return ok;
}

bool can2_send(const ODrive::CanFrame& f) {
  const bool ok = send_on(can2, f);
  if (ok) s_can2_tx++;
  return ok;
}

static uint8_t health_of(uint64_t last_rx_us) {
  if (last_rx_us == 0) return JbUdp::BusHealth::UNKNOWN;
  // OK if we've seen a frame within the CAN heartbeat window.
  // TODO(bench): read the FlexCAN error/bus-off registers for WARN/BUS_OFF.
  if (now_wall_us() - last_rx_us > CAN_HEARTBEAT_TIMEOUT_US) return JbUdp::BusHealth::WARN;
  return JbUdp::BusHealth::OK;
}

CanStats can_buses_stats() {
  CanStats s;
  s.can1_rx = s_can1_rx; s.can1_tx = s_can1_tx;
  s.can2_rx = s_can2_rx; s.can2_tx = s_can2_tx;
  s.can1_health = health_of(s_can1_last_rx_us);
  s.can2_health = health_of(s_can2_last_rx_us);
  return s;
}

}  // namespace CanBridge
