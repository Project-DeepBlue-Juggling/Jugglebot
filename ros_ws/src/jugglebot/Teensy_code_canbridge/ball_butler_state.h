#pragma once
// =============================================================================
//  ball_butler_state.h — Ball Butler heartbeat cache (decoded fields)
// =============================================================================
//  The can-bridge's mirror of the Ball Butler heartbeat (CAN1 ID 0x7D1):
//  decoded state byte + state_data + ball_in_hand bit + yaw/pitch/hand decoded
//  to floats. Populated by the CAN1 RX decode (see can_buses.cpp), read by the
//  [bb] task_diag print and by the upstream HeartbeatT2J emitter.
//
//  Concurrency: the heartbeat decode writes 6+ correlated fields. We use the
//  same seqlock-style retry pattern as axis_state.h's pos/vel triple so a
//  reader takes a torn-read-free snapshot of the full BB state without a
//  mutex. The CAN1 RX callback is the sole writer (FlexCAN ISR context); the
//  task_diag / heartbeat-emitter tasks are readers.
//
//  Convention: yaw/pitch in degrees, hand position in mm — matches the wire
//  decode in jugglebot.can.ball_butler.BallButlerHeartbeat (HEARTBEAT_*_RES_*
//  applied at decode time).
// =============================================================================

#include <cstdint>
#include "canbridge_config.h"
#include "protocol_config.h"

namespace CanBridge {

struct BallButlerState {
  // ── Decoded heartbeat fields (written by CAN1 RX decode) ─────────────────
  volatile bool     ball_in_hand  = false;   // heartbeat byte 0 bit 0
  volatile uint8_t  state         = 0;       // BallButlerState::* (BOOT/IDLE/…)
  volatile uint8_t  state_data    = 0;       // error code when state == ERROR, else 0
  volatile float    yaw_deg       = 0.0f;
  volatile float    pitch_deg     = 0.0f;
  volatile float    hand_mm       = 0.0f;

  // ── Freshness ────────────────────────────────────────────────────────────
  // last_heartbeat_us + heartbeat_seen written by the RX decode; heartbeat_stale
  // is set by a freshness check elsewhere (e.g. task_diag) against
  // BallButler::HEARTBEAT_TIMEOUT_MS.
  volatile uint64_t last_heartbeat_us = 0;
  volatile bool     heartbeat_seen    = false;
  volatile bool     heartbeat_stale   = false;

  // ── Seqlock for torn-read-free snapshot of the decoded fields ────────────
  // Bumped odd before a write, even after. Readers retry while odd or changed.
  volatile uint32_t seq = 0;
};

// Singleton; defined in ball_butler_state.cpp.
extern BallButlerState bb_state;

// Snapshot returned by snapshot_bb(): the fields a reader cares about, copied
// out atomically so no consumer sees a torn write across them.
struct BallButlerSnapshot {
  bool     ball_in_hand;
  uint8_t  state;
  uint8_t  state_data;
  float    yaw_deg;
  float    pitch_deg;
  float    hand_mm;
  uint64_t last_heartbeat_us;
  bool     heartbeat_seen;
  bool     heartbeat_stale;
};

// Writer: update the decoded state atomically. Call from the CAN1 RX decode
// when a heartbeat frame arrives. Bumps seq odd → writes → bumps seq even.
inline void write_bb_heartbeat(BallButlerState& s,
                               bool ball_in_hand,
                               uint8_t state, uint8_t state_data,
                               float yaw_deg, float pitch_deg, float hand_mm,
                               uint64_t t_us) {
  s.seq = s.seq + 1;          // odd → write in progress
  asm volatile("" ::: "memory");
  s.ball_in_hand      = ball_in_hand;
  s.state             = state;
  s.state_data        = state_data;
  s.yaw_deg           = yaw_deg;
  s.pitch_deg         = pitch_deg;
  s.hand_mm           = hand_mm;
  s.last_heartbeat_us = t_us;
  s.heartbeat_seen    = true;
  s.heartbeat_stale   = false;  // mirror axis decode's stale-clear (axis_state.cpp:55)
  asm volatile("" ::: "memory");
  s.seq = s.seq + 1;          // even → done
}

// Reader: consistent snapshot of the decoded BB state. Retries internally if
// the writer was mid-update.
inline void snapshot_bb(const BallButlerState& s, BallButlerSnapshot& out) {
  uint32_t s0, s1;
  do {
    s0 = s.seq;
    out.ball_in_hand      = s.ball_in_hand;
    out.state             = s.state;
    out.state_data        = s.state_data;
    out.yaw_deg           = s.yaw_deg;
    out.pitch_deg         = s.pitch_deg;
    out.hand_mm           = s.hand_mm;
    out.last_heartbeat_us = s.last_heartbeat_us;
    out.heartbeat_seen    = s.heartbeat_seen;
    out.heartbeat_stale   = s.heartbeat_stale;
    asm volatile("" ::: "memory");
    s1 = s.seq;
  } while ((s0 & 1) || (s0 != s1));   // retry if writer was mid-update
}

}  // namespace CanBridge
