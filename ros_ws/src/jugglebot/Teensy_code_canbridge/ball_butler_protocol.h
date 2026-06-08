#pragma once
// =============================================================================
//  ball_butler_protocol.h — Ball Butler command CAN frame encoders
// =============================================================================
//  Pure-header byte-for-byte port of jugglebot.can.ball_butler.encode_throw_
//  command + encode_state_command. The RPC dispatch in rpc.cpp validates input
//  ranges and returns ERR_BAD_ARGS for malformed throws BEFORE calling these
//  encoders — the encoders themselves are pure builders.
//
//  Wire format (8 bytes, little-endian) for THROW_CMD (0x7D0):
//    Bytes 0-1: yaw   (int16,  π/32768 rad/LSB)
//    Bytes 2-3: pitch (uint16, π/65536 rad/LSB)
//    Bytes 4-5: speed (uint16, 0.1 mm/s/LSB; speed_mps * 10000)
//    Bytes 6-7: throw_time (uint16, lower 16 bits of epoch ms = wall ms & 0xFFFF)
//
//  RELOAD/RESET/CALIBRATE_LOC are payloadless commands distinguished by ID.
//
//  Range constants below mirror the Python validation (ball_butler.py:98-105)
//  and are reused by the RPC dispatch's ERR_BAD_ARGS check.
// =============================================================================

#include <cstdint>
#include <cstring>
#include "odrive_protocol.h"   // CanFrame
#include "protocol_config.h"   // BallButlerCanId::*
#include "time_base.h"         // now_wall_us()

namespace CanBridge {
namespace BallButler {

// ── Input ranges (mirror of ball_butler.py:98-105) ───────────────────────────
constexpr float kPi          = 3.14159265358979323846f;
constexpr float kHalfPi      = kPi * 0.5f;

constexpr float YAW_MIN_RAD   = -kPi;
constexpr float YAW_MAX_RAD   =  kPi;        // exclusive upper bound in Python
constexpr float PITCH_MIN_RAD =  0.0f;
constexpr float PITCH_MAX_RAD =  kHalfPi;
constexpr float SPEED_MIN_MPS =  0.0f;
constexpr float SPEED_MAX_MPS =  6.5535f;
constexpr float DELAY_MIN_S   =  0.0f;
constexpr float DELAY_MAX_S   =  65.535f;

// Scale factors for the wire encoding (constexpr, no macro dependency).
constexpr float YAW_SCALE   = 32768.0f / kPi;
constexpr float PITCH_SCALE = 65536.0f / kPi;
constexpr float SPEED_SCALE = 10000.0f;

// True iff the throw arguments are inside the Python-validated ranges.
// Used by the rpc.cpp BB_THROW dispatch to short-circuit with ERR_BAD_ARGS
// before constructing the frame. Mirrors ball_butler.encode_throw_command's
// math.isfinite + range checks.
inline bool throw_args_valid(float yaw_rad, float pitch_rad,
                             float speed_mps, float delay_s) {
  // NaN/Inf fail the < / >= comparisons, matching math.isfinite() rejection.
  if (!(yaw_rad   >= YAW_MIN_RAD   && yaw_rad   <  YAW_MAX_RAD))   return false;
  if (!(pitch_rad >= PITCH_MIN_RAD && pitch_rad <= PITCH_MAX_RAD)) return false;
  if (!(speed_mps >= SPEED_MIN_MPS && speed_mps <= SPEED_MAX_MPS)) return false;
  if (!(delay_s   >= DELAY_MIN_S   && delay_s   <= DELAY_MAX_S))   return false;
  return true;
}

// ── encode_throw (mirror of ball_butler.py:77 encode_throw_command) ──────────
// Pre-condition: throw_args_valid(...) returned true. Builds the 8-byte CAN
// frame with the wire-format scaling. Truncation-toward-zero matches Python's
// int() cast on both positive and negative yaw values.
inline ODrive::CanFrame encode_throw(float yaw_rad, float pitch_rad,
                                     float speed_mps, float delay_s) {
  ODrive::CanFrame f;
  f.id  = BallButlerCanId::THROW_CMD;
  f.len = 8;

  const int16_t  yaw_raw   = (int16_t)(yaw_rad   * YAW_SCALE);
  const uint16_t pitch_raw = (uint16_t)(pitch_rad * PITCH_SCALE);
  const uint16_t speed_raw = (uint16_t)(speed_mps * SPEED_SCALE);

  // throw_time = (wall_ms + delay_ms) & 0xFFFF. Mirrors ball_butler.py:110:
  //   throw_time_ms = int((time.time() + delay_s) * 1000.0) & 0xFFFF
  // Requires time-sync (now_wall_us == micros64() before the first anchor),
  // but the BB slave only consumes the low 16 bits modulo so the sync is
  // about phase agreement, not absolute time.
  const uint64_t throw_time_ms_64 =
      (now_wall_us() / 1000ULL) + (uint64_t)(delay_s * 1000.0f);
  const uint16_t throw_time_raw = (uint16_t)(throw_time_ms_64 & 0xFFFFULL);

  memcpy(&f.buf[0], &yaw_raw,        2);
  memcpy(&f.buf[2], &pitch_raw,      2);
  memcpy(&f.buf[4], &speed_raw,      2);
  memcpy(&f.buf[6], &throw_time_raw, 2);
  return f;
}

// ── Payloadless state commands (mirror of ball_butler.py:122 encode_state_command)
inline ODrive::CanFrame encode_reload() {
  ODrive::CanFrame f; f.id = BallButlerCanId::RELOAD_CMD;        f.len = 0; return f;
}
inline ODrive::CanFrame encode_reset() {
  ODrive::CanFrame f; f.id = BallButlerCanId::RESET_CMD;         f.len = 0; return f;
}
inline ODrive::CanFrame encode_calibrate_loc() {
  ODrive::CanFrame f; f.id = BallButlerCanId::CALIBRATE_LOC_CMD; f.len = 0; return f;
}

}  // namespace BallButler
}  // namespace CanBridge
