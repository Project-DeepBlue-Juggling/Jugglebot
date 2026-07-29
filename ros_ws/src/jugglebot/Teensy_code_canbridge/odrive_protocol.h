#pragma once
// =============================================================================
//  odrive_protocol.h — ODrive CAN encode/decode (C++ port of odrive.py)
// =============================================================================
//  Direct, byte-for-byte port of
//    ros_ws/src/jugglebot/jugglebot/can/odrive.py
//  Arbitration ID layout: arb_id = (node_id << 5) | command_id.
//  All multi-byte fields are little-endian, matching Python struct '<' and the
//  ODrive wire format; Cortex-M7 is native LE so memcpy of float/int matches.
//
//  Decoupled from FlexCAN: encoders fill a neutral CanFrame POD (like odrive.py
//  returns a can.Message); can_buses converts CanFrame ↔ CAN_message_t on TX/RX.
//  This keeps the protocol unit-checkable and matches the Python separation.
//
//  Convention: positions/velocities crossing this boundary are in the JUGGLEBOT
//  convention (positive = extension). encode_leg_setpoint() applies the ODrive
//  sign flip + int16 scaling exactly as can_node._send_position_target does.
// =============================================================================

#include <cstdint>
#include <cstring>
#include <cmath>
#include "protocol_config.h"
#include "canbridge_config.h"

namespace CanBridge {
namespace ODrive {

// ── Neutral CAN frame POD ─────────────────────────────────────────────────────
struct CanFrame {
  uint32_t id  = 0;
  uint8_t  len = 8;
  uint8_t  buf[8] = {0};
};

constexpr uint8_t NODE_ID_SHIFT = 5;
constexpr uint8_t CMD_ID_MASK   = 0x1F;

inline uint32_t arb_id(uint8_t axis, uint8_t cmd) {
  return ((uint32_t)axis << NODE_ID_SHIFT) | cmd;
}
inline uint8_t axis_of(uint32_t arb)  { return (uint8_t)(arb >> NODE_ID_SHIFT); }
inline uint8_t cmd_of(uint32_t arb)   { return (uint8_t)(arb & CMD_ID_MASK); }

inline bool is_leg(uint8_t axis) { return axis < NUM_LEGS; }

// ODrive convention: negative = extension; Jugglebot: positive = extension.
// Centralised sign flip (mirrors odrive._leg_sign / can_node._leg_sign).
inline float leg_sign(uint8_t axis, float v) { return is_leg(axis) ? -v : v; }

// ── Error bitmasks (odrive.py ERROR_CODES) ───────────────────────────────────
constexpr uint32_t ERR_DC_BUS_UNDER_VOLTAGE = 512u;

// ── Encoders (fill `out`) ─────────────────────────────────────────────────────
inline CanFrame encode_get_version(uint8_t axis) {
  CanFrame f; f.id = arb_id(axis, ODriveCmd::get_version); f.len = 0; return f;
}

inline CanFrame encode_set_state(uint8_t axis, uint32_t state) {
  CanFrame f; f.id = arb_id(axis, ODriveCmd::set_requested_state);
  memcpy(&f.buf[0], &state, 4);    // bytes 4..7 already zero
  return f;
}

inline CanFrame encode_set_controller_mode(uint8_t axis, uint32_t ctrl, uint32_t input) {
  CanFrame f; f.id = arb_id(axis, ODriveCmd::set_controller_mode);
  memcpy(&f.buf[0], &ctrl, 4); memcpy(&f.buf[4], &input, 4);
  return f;
}

// pack('<fhh', position, vel_ff_int16, torque_ff_int16)
inline CanFrame encode_set_input_pos(uint8_t axis, float position,
                                     int16_t vel_ff, int16_t torque_ff) {
  CanFrame f; f.id = arb_id(axis, ODriveCmd::set_input_pos);
  memcpy(&f.buf[0], &position, 4);
  memcpy(&f.buf[4], &vel_ff, 2);
  memcpy(&f.buf[6], &torque_ff, 2);
  return f;
}

inline CanFrame encode_set_input_vel(uint8_t axis, float vel, float torque_ff) {
  CanFrame f; f.id = arb_id(axis, ODriveCmd::set_input_vel);
  memcpy(&f.buf[0], &vel, 4); memcpy(&f.buf[4], &torque_ff, 4);
  return f;
}

inline CanFrame encode_set_input_torque(uint8_t axis, float torque) {
  CanFrame f; f.id = arb_id(axis, ODriveCmd::set_input_torque);
  memcpy(&f.buf[0], &torque, 4);
  return f;
}

inline CanFrame encode_set_vel_curr_limits(uint8_t axis, float vel_limit, float curr_limit) {
  CanFrame f; f.id = arb_id(axis, ODriveCmd::set_vel_curr_limits);
  memcpy(&f.buf[0], &vel_limit, 4); memcpy(&f.buf[4], &curr_limit, 4);
  return f;
}

// pack('<f', vel_limit) + 4 zero bytes (odrive.encode_set_traj_vel_limit)
inline CanFrame encode_set_traj_vel_limit(uint8_t axis, float vel_limit) {
  CanFrame f; f.id = arb_id(axis, ODriveCmd::set_traj_vel_limit);
  memcpy(&f.buf[0], &vel_limit, 4);   // bytes 4..7 already zero
  return f;
}

// pack('<ff', acc_limit, dec_limit) (odrive.encode_set_traj_acc_limits)
inline CanFrame encode_set_traj_acc_limits(uint8_t axis, float acc_limit, float dec_limit) {
  CanFrame f; f.id = arb_id(axis, ODriveCmd::set_traj_acc_limits);
  memcpy(&f.buf[0], &acc_limit, 4); memcpy(&f.buf[4], &dec_limit, 4);
  return f;
}

inline CanFrame encode_set_absolute_position(uint8_t axis, float position) {
  CanFrame f; f.id = arb_id(axis, ODriveCmd::set_absolute_position);
  memcpy(&f.buf[0], &position, 4);
  return f;
}

inline CanFrame encode_set_pos_gain(uint8_t axis, float pos_gain) {
  CanFrame f; f.id = arb_id(axis, ODriveCmd::set_pos_gain);
  memcpy(&f.buf[0], &pos_gain, 4);
  return f;
}

inline CanFrame encode_set_vel_gains(uint8_t axis, float vel_gain, float vel_int_gain) {
  CanFrame f; f.id = arb_id(axis, ODriveCmd::set_vel_gains);
  memcpy(&f.buf[0], &vel_gain, 4); memcpy(&f.buf[4], &vel_int_gain, 4);
  return f;
}

inline CanFrame encode_clear_errors(uint8_t axis) {
  CanFrame f; f.id = arb_id(axis, ODriveCmd::clear_errors); return f;  // 8 zero bytes
}

inline CanFrame encode_reboot(uint8_t axis) {
  CanFrame f; f.id = arb_id(axis, ODriveCmd::reboot_odrives); return f;
}

// pack('<BHBf', opcode, endpoint_id, 0, value)
inline CanFrame encode_sdo(uint8_t axis, uint8_t opcode, uint16_t endpoint_id, float value) {
  CanFrame f; f.id = arb_id(axis, ODriveCmd::RxSdo);
  f.buf[0] = opcode;
  memcpy(&f.buf[1], &endpoint_id, 2);
  f.buf[3] = 0;
  memcpy(&f.buf[4], &value, 4);
  return f;
}
inline CanFrame encode_sdo_read(uint8_t axis, uint16_t endpoint_id) {
  return encode_sdo(axis, SDO::OPCODE_READ, endpoint_id, 0.0f);
}
inline CanFrame encode_sdo_write(uint8_t axis, uint16_t endpoint_id, float value) {
  return encode_sdo(axis, SDO::OPCODE_WRITE, endpoint_id, value);
}

// ── clip_position (odrive.clip_position) ──────────────────────────────────────
// Clamp a Jugglebot setpoint (pre-inversion) to [0, max]. Caller negates for legs.
inline float clip_position(uint8_t axis, float setpoint) {
  const float max_pos = is_leg(axis) ? LEG_MOTOR_MAX_POSITION : HAND_MOTOR_MAX_POSITION;
  if (setpoint < 0.0f) return 0.0f;
  if (setpoint > max_pos) return max_pos;
  return setpoint;
}

// ── Full leg/hand setpoint TX (port of can_node._send_position_target) ────────
//  clip → leg_sign → int16 scale (vel*scale, tor*scale) → clamp int16 → encode.
inline CanFrame encode_leg_setpoint(uint8_t axis, float setpoint_rev,
                                    float vel_ff_rps, float torque_ff_Nm) {
  float pos = clip_position(axis, setpoint_rev);
  pos       = leg_sign(axis, pos);
  float vff = leg_sign(axis, vel_ff_rps);
  float tff = leg_sign(axis, torque_ff_Nm);
  float vscale, tscale;
  if (is_leg(axis)) { vscale = LEG_VEL_SCALE;  tscale = LEG_TOR_SCALE; }
  else              { vscale = HAND_VEL_SCALE; tscale = HAND_TOR_SCALE; }
  long vi = lroundf(vff * vscale);
  long ti = lroundf(tff * tscale);
  if (vi < -32768) vi = -32768; else if (vi > 32767) vi = 32767;
  if (ti < -32768) ti = -32768; else if (ti > 32767) ti = 32767;
  return encode_set_input_pos(axis, pos, (int16_t)vi, (int16_t)ti);
}

// ── Decoders (data = 8-byte CAN payload) ──────────────────────────────────────
struct Heartbeat { uint8_t state; uint8_t procedure_result; bool trajectory_done; };
inline Heartbeat decode_heartbeat(const uint8_t* d) {
  return Heartbeat{ d[4], d[5], (bool)(d[6] & 0x01) };
}

struct ErrorFrame { uint32_t active_errors; uint32_t disarm_reason; };
inline ErrorFrame decode_error(const uint8_t* d) {
  ErrorFrame e; memcpy(&e.active_errors, &d[0], 4); memcpy(&e.disarm_reason, &d[4], 4); return e;
}

struct FloatPair { float a; float b; };
inline FloatPair decode_encoder_estimate(const uint8_t* d) {  // (pos, vel)
  FloatPair p; memcpy(&p.a, &d[0], 4); memcpy(&p.b, &d[4], 4); return p;
}
inline FloatPair decode_iq(const uint8_t* d)   { return decode_encoder_estimate(d); }  // (sp, meas)
inline FloatPair decode_temps(const uint8_t* d){ return decode_encoder_estimate(d); }  // (fet, motor)
inline FloatPair decode_bus_voltage_current(const uint8_t* d) { return decode_encoder_estimate(d); }

// '<BHB' header + the 4-byte payload decoded as a UINT32. The `_u32` suffix
// stays even though this is now the only C++ SDO decoder: the Python side's
// odrive.decode_sdo_response unpacks float32, and a same-name/different-
// semantics pair across the two languages is a worse trap than a long name.
// (A float32 variant existed here until 2026-07-29 and was removed as dead —
// no C++ caller ever had one; get_gpio_states returns a bitmask, which a
// float32 unpack would reinterpret.)
struct SdoResponseU32 { uint16_t endpoint_id; uint32_t value; };
inline SdoResponseU32 decode_sdo_response_u32(const uint8_t* d) {
  SdoResponseU32 r; memcpy(&r.endpoint_id, &d[1], 2); memcpy(&r.value, &d[4], 4); return r;
}

struct Version {
  uint8_t proto_ver, hw_product, hw_ver, hw_variant, fw_major, fw_minor, fw_rev, fw_unreleased;
};
inline Version decode_get_version(const uint8_t* d) {
  return Version{ d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7] };
}

}  // namespace ODrive
}  // namespace CanBridge
