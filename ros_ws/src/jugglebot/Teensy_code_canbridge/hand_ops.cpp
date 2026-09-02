// =============================================================================
//  hand_ops.cpp — Hand trajectory / smooth-move conduit
// =============================================================================
//  Port of can_node._send_hand_traj_cmd / _smooth_move_hand (can_node.py:1626-1661)
//  minus the payload construction (which stays HOST-side, byte-identical to
//  can_node — see hand_ops.h / config/generate_udp_protocol.py ArgHandTraj). Here
//  the firmware only: gates, sends the ready-preamble, and forwards the 8 bytes on
//  the firmware-owned 0x6D0 id.
// =============================================================================
#include "hand_ops.h"

#include <cstring>            // memcpy
#include "canbridge_config.h"  // HAND_AXIS
#include "protocol_config.h"   // ODriveState / ODriveControlMode / ODriveInputMode / PlatformCanId
#include "odrive_protocol.h"   // ODrive::encode_set_state / encode_set_controller_mode / CanFrame
#include "can_buses.h"         // can_jugglebot_tx / TxCls, jugglebot_commands_allowed
#include "leg_homing.h"        // homing_active (HAND_TRAJ_CMD ↔ homing interlock)
#include "leg_interp.h"        // interp_last_tick_us (the phase-stamp reference)
#include "hand_source.h"       // hand_source_streamed (the § 2.4 mastery latch, FW 17)
#include "time_base.h"         // micros64 (interval clock — the phase is an INTERVAL)

namespace CanBridge {

// Per-stage outcome counters — contract (incl. the single-writer argument for
// the missing PRIMASK) in hand_ops.h.
static HandOpsCounters s_counters{};

// Interp-phase stamp ring — contract (pre-registered read, benign-race semantics)
// in hand_ops.h. Console-only; nothing here reaches the wire.
static HandPhaseRing s_phase{};

HandOpsCounters hand_ops_counters() { return s_counters; }

HandPhaseRing hand_phase_ring() { return s_phase; }

void hand_ops_counters_reset() {
  s_counters = HandOpsCounters{};
  s_phase = HandPhaseRing{};
}

// Record one dispatch's (phase, outcome). The COUNT is incremented last, after the
// sample slot is fully written: the reader keys off `n` to decide which slots are
// real, so storing the payload before the count is what keeps the benign race
// benign in the common case. In source order the payload lands first; nothing
// forces the compiler to keep that order across the two plain stores and nothing
// needs it to — an out-of-order `n` costs one garbled console sample and cannot
// reach control flow.
static inline void phase_push(uint16_t phase_us, uint8_t outcome) {
  s_phase.v[s_phase.n % HAND_PHASE_RING_LEN] = HandPhaseSample{phase_us, outcome};
  s_phase.n++;
}

namespace HandOps {

uint16_t hand_traj_cmd(const JbUdp::RpcArgs::ArgHandTraj& a) {
  using namespace JbUdp;   // RpcStatus::* (JbUdp::RpcStatus is a namespace, not a type)
  // Counted BEFORE any gate, so it is the honest denominator: OK is derived by
  // subtracting the five failure counters, and a gate that refuses every call
  // still shows up as traffic rather than as silence.
  s_counters.calls++;
  // Interp-phase stamp, taken HERE — before any gate, alongside `calls`, and before
  // the first CAN send — so every dispatch is stamped at the same point in the
  // function and the OK vs FAIL phase distributions are directly comparable. Two
  // u64 reads plus a subtract, on the RPC task; NOTHING is added to the 500 Hz ISR
  // path (interp_last_tick_us only READS the stamp the ISR already keeps).
  //
  // Reads the TICK FIRST so an ISR landing between the two samples cannot make the
  // subtraction go negative. Written as one expression the two calls are only
  // indeterminately sequenced, and GCC in fact emits micros64() first — so a tick
  // arriving between them yields a tick_us LATER than the micros64() sample and the
  // u64 subtraction underflows, surfacing as phase_us ≈ 65480-65535: a value this
  // ring's own contract says is impossible, i.e. a self-inflicted refutation of the
  // phase model. Sequencing it explicitly makes the ordering a source-level fact
  // rather than a compiler accident.
  const uint64_t tick_us  = interp_last_tick_us();
  const uint16_t phase_us = (uint16_t)(micros64() - tick_us);
  // ── hand_source interlock (§ 2.4, FW 17) — the FIRST gate ───────────────────
  // While the bridge masters the hand (STREAMED), the legacy stroke conduit is
  // structurally refused: a 0x6D0 dispatch would put the Platform Teensy's
  // 500 Hz stroke on the wire AGAINST the interp's own 500 Hz hand frames —
  // exactly the two-masters-one-CAN-id class the latch exists to make
  // impossible. ERR_HAND_SOURCE (not ERR_REJECTED) so the host ack NAMES the
  // mode as the reason; visible in hand_traj_acks (T-H4's observable).
  if (hand_source_streamed()) {
    s_counters.rej_source++;
    phase_push(phase_us, HAND_PHASE_REJ_SOURCE);
    return RpcStatus::ERR_HAND_SOURCE;
  }
  // HAND_TRAJ_CMD ↔ homing interlock, checked before any CAN send: a hand
  // catch-trajectory must not fire while the SAME firmware state machine is mid-
  // homing (axis 6 homes with the shared move-to-hardstop ladder). A concurrent
  // traj would fight the move-to-hardstop and corrupt the just-defined
  // HAND_ABS_POS_REV reference. Reject before the preamble reaches CAN3.
  if (homing_active()) {
    s_counters.rej_homing++;
    phase_push(phase_us, HAND_PHASE_REJ_HOMING);
    return RpcStatus::ERR_REJECTED;
  }
  // Gate like a leg motion command: a confirmed-stale/dead CAN3 must withhold the
  // trajectory. HAND_TRAJ_CMD is NOT a recovery one-shot, so it keeps the
  // heartbeat-staleness gate (jugglebot_commands_allowed) — NOT the SYNCH
  // bus-transmittable carve-out that CLEAR_ERRORS/REBOOT_ODRIVES use.
  if (!jugglebot_commands_allowed()) {
    s_counters.bus_down++;
    phase_push(phase_us, HAND_PHASE_BUS_DOWN);
    return RpcStatus::ERR_BUS_DOWN;
  }

  // Preamble — bring the hand ODrive (axis 6) to CLOSED_LOOP + POSITION/PASSTHROUGH
  // so the Platform Teensy's ensuing trajectory setpoints land. can_node issued
  // this in BOTH _send_hand_traj_cmd and _smooth_move_hand (the dropped precondition
  // the audit flagged, row 37). ABORT the traj TX if either preamble frame fails to
  // enqueue — running a trajectory against a hand that is not in CLOSED_LOOP/
  // PASSTHROUGH would fault or silently no-op the move.
  // TxCls::HAND (2026-08-24): DEFERRED = SENT, and the ack says so. Reading a
  // deferral as failure here is what produced the ERR_TIMEOUT epidemic — a
  // dispatch the bench then observed transmitting, acked as a timeout (the
  // "lying ack"). Only TxResult::FAILED aborts now. NOTHING here re-dispatches:
  // the hand ladders and _MAX_ARM_DISPATCHES cap stay as they are, deliberately.
  if (!tx_reached_the_wire(can_jugglebot_tx(
          ODrive::encode_set_state(HAND_AXIS, ODriveState::CLOSED_LOOP), TxCls::HAND))) {
    s_counters.pre1_fail++;
    phase_push(phase_us, HAND_PHASE_PRE1);
    return RpcStatus::ERR_TIMEOUT;
  }
  if (!tx_reached_the_wire(can_jugglebot_tx(
          ODrive::encode_set_controller_mode(
              HAND_AXIS, ODriveControlMode::POSITION, ODriveInputMode::PASSTHROUGH),
          TxCls::HAND))) {
    s_counters.pre2_fail++;
    phase_push(phase_us, HAND_PHASE_PRE2);
    return RpcStatus::ERR_TIMEOUT;
  }

  // Forward the host-built 8-byte payload on the FIRMWARE-OWNED 0x6D0 id. The Jetson
  // supplies the payload (incl. the absolute wall_time_ms deadline baked in); the
  // firmware owns the arbitration id + dlc and does NOT re-stamp — it never sees the
  // deadline as anything but opaque bytes.
  ODrive::CanFrame f;
  f.id  = PlatformCanId::TRAJ_CMD;   // 0x6D0
  f.len = 8;
  memcpy(f.buf, a.payload, 8);
  if (!tx_reached_the_wire(can_jugglebot_tx(f, TxCls::HAND))) {
    s_counters.traj_fail++;
    phase_push(phase_us, HAND_PHASE_TRAJ);
    return RpcStatus::ERR_TIMEOUT;
  }
  phase_push(phase_us, HAND_PHASE_OK);
  return RpcStatus::OK;
}

}  // namespace HandOps
}  // namespace CanBridge
