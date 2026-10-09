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

// ── BB stamped yaw estimate (CAN1 YAW_ESTIMATE 0x7D8, FW 28) ────────────────
// Written by the CAN1 RX decode once per BB frame (~150 Hz, one per fresh
// YawAxis sample); read by telemetry.cpp send_bb_estimates() at 100 Hz. Same
// seqlock idiom as BallButlerState. All times are bridge micros64() (monotonic):
// the sample's age at emit is (now_mono - rx_mono_us) + age_at_tx_us, with no
// wall-clock term, so a time-sync slew cannot move it.
struct BbYawEstimateCache {
  volatile float    yaw_deg      = 0.0f;   // BB-local, unwrapped (deg)
  volatile float    yaw_vel_dps  = 0.0f;   // deg/s, decoded from the int16
  volatile uint32_t age_at_tx_us = 0;      // BB sample -> BB TX (us, BB monotonic)
  volatile uint64_t rx_mono_us   = 0;      // bridge micros64() at RX decode
  volatile uint32_t frames       = 0;      // frames decoded this boot
  volatile bool     seen         = false;
  volatile uint32_t seq          = 0;
};

extern BbYawEstimateCache bb_yaw;

struct BbYawEstimateSnapshot {
  float    yaw_deg;
  float    yaw_vel_dps;
  uint32_t age_at_tx_us;
  uint64_t rx_mono_us;
  uint32_t frames;
  bool     seen;
};

inline void write_bb_yaw(BbYawEstimateCache& c, float yaw_deg, float yaw_vel_dps,
                         uint32_t age_at_tx_us, uint64_t rx_mono_us) {
  c.seq = c.seq + 1;          // odd → write in progress
  asm volatile("" ::: "memory");
  c.yaw_deg      = yaw_deg;
  c.yaw_vel_dps  = yaw_vel_dps;
  c.age_at_tx_us = age_at_tx_us;
  c.rx_mono_us   = rx_mono_us;
  c.frames       = c.frames + 1;
  c.seen         = true;
  asm volatile("" ::: "memory");
  c.seq = c.seq + 1;          // even → done
}

inline void snapshot_bb_yaw(const BbYawEstimateCache& c, BbYawEstimateSnapshot& out) {
  uint32_t s0, s1;
  do {
    s0 = c.seq;
    out.yaw_deg      = c.yaw_deg;
    out.yaw_vel_dps  = c.yaw_vel_dps;
    out.age_at_tx_us = c.age_at_tx_us;
    out.rx_mono_us   = c.rx_mono_us;
    out.frames       = c.frames;
    out.seen         = c.seen;
    asm volatile("" ::: "memory");
    s1 = c.seq;
  } while ((s0 & 1) || (s0 != s1));
}

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
