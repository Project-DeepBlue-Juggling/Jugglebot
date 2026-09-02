// =============================================================================
//  hand_source.cpp — the hand-mastery interlock latch (unified-7dof FW 17)
// =============================================================================
//  Contract + rationale in hand_source.h. This TU deliberately links against
//  only axis_state (the axis-6 telemetry the settle gate reads) and the clock
//  (micros64), so the native harness can drive it with the standard
//  axis_state.o + fake_hal.o pair and no stubs.
// =============================================================================
#include "hand_source.h"

#include <cmath>               // fabsf
#include "canbridge_config.h"  // HAND_SETTLE_BAND_REV / rest positions / staleness
#include "udp_protocol.h"      // JbUdp::RpcStatus
#include "axis_state.h"        // hand_axis() telemetry
#include "time_base.h"         // micros64 / atomic_read_u64

namespace CanBridge {

// Written by hand_source_request (RPC/net task) + hand_source_reset (tests);
// read by the interp ISR, hand_ops, fault task and heartbeat task. Single
// naturally-aligned byte → atomic loads/stores on Cortex-M7; volatile so no
// reader caches it across a switch.
static volatile uint8_t s_source = HandSource::LEGACY_STROKE;

uint8_t hand_source()          { return s_source; }
bool    hand_source_streamed() { return s_source == HandSource::STREAMED; }
void    hand_source_reset()    { s_source = HandSource::LEGACY_STROKE; }

// The hand is "settled at a rest position" iff it sits inside
// HAND_SETTLE_BAND_REV of a known rest and is not moving. Two rests are
// recognised (the two places the hand legitimately parks between motions):
//   * retract  — JBOp::HAND_RETRACT_REV (0.0). The band's LOWER edge is taken
//     from Homing::HAND_ABS_POS_REV (-0.1): a freshly-homed hand rests AT the
//     homing reference, 0.1 rev below retract, and must count as settled —
//     measuring 0.10 rev from 0.0 alone would put it exactly on the boundary
//     and make the gate a float-equality coin flip.
//   * catch-prime — JBOp::HAND_CATCH_PRIME_REV (9.9594), where the hand waits
//     between catches. Accepting it makes a mid-session STREAMED → LEGACY
//     rollback possible without first commanding a streamed move to 0.
static bool hand_settled_at_rest(float pos, float vel) {
  if (fabsf(vel) > HAND_SOURCE_SETTLE_VEL_RPS) return false;
  const bool at_retract =
      (pos >= Homing::HAND_ABS_POS_REV - HAND_SETTLE_BAND_REV) &&
      (pos <= JBOp::HAND_RETRACT_REV + HAND_SETTLE_BAND_REV);
  const bool at_prime =
      fabsf(pos - JBOp::HAND_CATCH_PRIME_REV) <= HAND_SETTLE_BAND_REV;
  return at_retract || at_prime;
}

uint16_t hand_source_request(uint8_t source, bool mpc_active_now) {
  using namespace JbUdp;
  if (source != HandSource::LEGACY_STROKE && source != HandSource::STREAMED)
    return RpcStatus::ERR_BAD_ARGS;
  // Idempotent: re-asserting the current mode is a no-op OK (so a host retry
  // after a lost response cannot wedge on the gates it already passed).
  if (source == s_source) return RpcStatus::OK;
  // Never swap mastery while the setpoint stream is armed — the interp may be
  // mid-motion on the hand lane (STREAMED) or a stroke may be in flight
  // (LEGACY), and either master's timeline would start cold on a moving axis.
  if (mpc_active_now) return RpcStatus::ERR_REJECTED;

  // Settle gate needs REAL, FRESH axis-6 telemetry: a snapshot of the hot
  // pos/vel/timestamp triple (seqlock — the CAN RX decode writes it at up to
  // 100 Hz), then a freshness check so a dark bus's stale cache cannot bless a
  // switch. pos_timestamp_us == 0 ⇒ never seen ⇒ refuse (0.0 rev is a real,
  // reachable, WRONG position — the bench7 unseen-skip lesson).
  AxisState& h = hand_axis();
  if (!h.heartbeat_seen) return RpcStatus::ERR_REJECTED;
  float pos, vel; uint64_t ts;
  snapshot_pos_vel(h, pos, vel, ts);
  if (ts == 0) return RpcStatus::ERR_REJECTED;
  const uint64_t now = micros64();   // interval clock — never the steppable wall
  if (now > ts && (now - ts) > MOTOR_FB_STALENESS_US) return RpcStatus::ERR_REJECTED;
  if (!hand_settled_at_rest(pos, vel)) return RpcStatus::ERR_REJECTED;

  s_source = source;   // single-byte store — atomic publish
  return RpcStatus::OK;
}

}  // namespace CanBridge
