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
//  ABSOLUTE RELEASE (PROTOCOL_VERSION 10, FW 29, 2026-10-12): the Jetson sends
//  the release instant itself (ArgBbThrow.release_wall_us, on the shared wall
//  clock), computed ONCE from the same `now` its ThrowAnnouncement is stamped
//  with. Until FW 28 the RPC carried a RELATIVE delay that this header added to
//  now_wall_us() ON RECEIPT, so every millisecond of Jetson dispatch latency
//  (DDS action goal, executor scheduling under load, UDP) moved the release by
//  the same millisecond — the 14-22 ms sd release jitter of the R5 sittings
//  (logbook/2026-10-12-skill-stack-r5-bb-feed-no-drift-release-jitter.md).
//  Dispatch latency now spends lead margin only. check_release() is the ONE
//  enforcement point for the lead bounds; BB's firmware is unchanged (it always
//  decoded bytes 6-7 as an absolute epoch-ms).
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
#include "udp_protocol.h"      // JbUdp::BB_RELEASE_{MIN,MAX}_LEAD_US

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

// Release-lead bounds at bridge receipt (protocol constants — see
// generate_udp_protocol.py for the derivation: MIN = BB wind-up + schedule
// margin, MAX = inside BB's ±32.768 s low-16-bit epoch-ms reconstruction).
constexpr int64_t RELEASE_MIN_LEAD_US = (int64_t)JbUdp::BB_RELEASE_MIN_LEAD_US;
constexpr int64_t RELEASE_MAX_LEAD_US = (int64_t)JbUdp::BB_RELEASE_MAX_LEAD_US;
static_assert(RELEASE_MAX_LEAD_US < 32768000LL,
              "BB decodes bytes 6-7 within ±32.768 s of its now — a longer lead wraps into the past");

// Scale factors for the wire encoding (constexpr, no macro dependency).
constexpr float YAW_SCALE   = 32768.0f / kPi;
constexpr float PITCH_SCALE = 65536.0f / kPi;
constexpr float SPEED_SCALE = 10000.0f;

// True iff the throw arguments are inside the Python-validated ranges.
// Used by the rpc.cpp BB_THROW dispatch to short-circuit with ERR_BAD_ARGS
// before constructing the frame. Mirrors ball_butler.encode_throw_command's
// math.isfinite + range checks.
inline bool throw_args_valid(float yaw_rad, float pitch_rad,
                             float speed_mps) {
  // NaN/Inf fail the < / >= comparisons, matching math.isfinite() rejection.
  if (!(yaw_rad   >= YAW_MIN_RAD   && yaw_rad   <  YAW_MAX_RAD))   return false;
  if (!(pitch_rad >= PITCH_MIN_RAD && pitch_rad <= PITCH_MAX_RAD)) return false;
  if (!(speed_mps >= SPEED_MIN_MPS && speed_mps <= SPEED_MAX_MPS)) return false;
  return true;
}

// ── check_release — the absolute-release lead gate (FW 29) ──────────────────
// Lead = release_wall_us − now_us (signed; computed in int64 so a release in the
// past is negative, never a huge unsigned wrap). Writes the lead (saturated to
// int32 for the RPC result) and returns the verdict. Order matters only for the
// name: <= 0 is PAST (the instant is gone — a "throw now" would be a silent
// re-time), then TOO_SOON (BB cannot meet it), then TOO_FAR (BB's 16-bit decode
// would wrap it into the past).
enum class ReleaseVerdict : uint8_t { OK = 0, PAST = 1, TOO_SOON = 2, TOO_FAR = 3 };

inline ReleaseVerdict check_release(uint64_t release_wall_us, uint64_t now_us,
                                    int32_t& lead_us_out) {
  const int64_t lead = (int64_t)release_wall_us - (int64_t)now_us;
  lead_us_out = (lead > (int64_t)INT32_MAX) ? INT32_MAX
              : (lead < (int64_t)INT32_MIN) ? INT32_MIN : (int32_t)lead;
  if (lead <= 0)                   return ReleaseVerdict::PAST;
  if (lead < RELEASE_MIN_LEAD_US)  return ReleaseVerdict::TOO_SOON;
  if (lead > RELEASE_MAX_LEAD_US)  return ReleaseVerdict::TOO_FAR;
  return ReleaseVerdict::OK;
}

// ── encode_throw (mirror of ball_butler.py:77 encode_throw_command) ──────────
// Pre-condition: throw_args_valid(...) returned true (and, for a throw,
// check_release(...) returned OK). Builds the 8-byte CAN frame with the
// wire-format scaling. Truncation-toward-zero matches Python's int() cast on
// both positive and negative yaw values. Pure: the release instant is an input,
// never read from the clock here.
inline ODrive::CanFrame encode_throw(float yaw_rad, float pitch_rad,
                                     float speed_mps, uint64_t release_wall_us) {
  ODrive::CanFrame f;
  f.id  = BallButlerCanId::THROW_CMD;
  f.len = 8;

  const int16_t  yaw_raw   = (int16_t)(yaw_rad   * YAW_SCALE);
  const uint16_t pitch_raw = (uint16_t)(pitch_rad * PITCH_SCALE);
  const uint16_t speed_raw = (uint16_t)(speed_mps * SPEED_SCALE);

  // throw_time = release epoch ms & 0xFFFF (mirrors ball_butler.py
  // encode_throw_command_at). BB rebuilds the upper bits from its own synced
  // wall clock (nearest instant within ±32.768 s), so the sync is about phase
  // agreement, not absolute time.
  const uint64_t throw_time_ms_64 = release_wall_us / 1000ULL;
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
