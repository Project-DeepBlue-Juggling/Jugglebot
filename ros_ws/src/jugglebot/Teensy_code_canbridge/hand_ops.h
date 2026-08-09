// =============================================================================
//  hand_ops.h — Hand trajectory / smooth-move conduit
// =============================================================================
//  The HAND_TRAJ_CMD RPC path. Keeps rpc.cpp thin (mirrors platform_relay.cpp):
//  set_hand_traj_cmd AND smooth_move_hand both ride this one RPC, discriminated by
//  byte 0 of the host-built payload (0/1/2 = catch-traj type, 3 = smooth-move).
//
//  The Jetson builds the EXACT 8-byte 0x6D0 PLATFORM_TRAJ_CMD payload (byte-
//  identical to can_node._send_hand_traj_cmd / _smooth_move_hand, including the
//  ABSOLUTE wall_time_ms deadline); the firmware attaches the firmware-owned 0x6D0
//  arbitration id + dlc and forwards it — never a Jetson-supplied raw frame
//  (least-privilege, same principle as STATE_WRITE re-encoding 0x6E0). Because the
//  firmware only ever sees opaque payload bytes it CANNOT re-stamp the deadline; an
//  absolute deadline is immune to Jetson→bridge→CAN3 transit jitter (the Platform
//  Teensy fires when its synced clock reaches the deadline — the temporal-accuracy
//  contract, see the BallButler 2026-06-18 arc).
// =============================================================================
#pragma once

#include <cstdint>
#include "udp_protocol.h"   // JbUdp::RpcArgs::ArgHandTraj, JbUdp::RpcStatus

namespace CanBridge {

// ── Per-stage outcome attribution (2026-08-02) ──────────────────────────────
//  hand_traj_cmd has FIVE distinct failure exits and THREE of them return the
//  same bare ERR_TIMEOUT, so on the wire they are indistinguishable — the
//  2026-08-01 recount could establish THAT the arm ack fails at ~59 % but not
//  WHICH send refused. These counters split them. They are CUMULATIVE SINCE
//  BOOT and ride the additive BridgeTxDiag uplink (telemetry.cpp).
//
//  `calls` is incremented at function ENTRY, before any gate, so the success
//  count is derivable without a sixth counter:
//      OK = calls - rej_homing - bus_down - pre1_fail - pre2_fail - traj_fail
//
//  SINGLE WRITER, no masking: hand_traj_cmd has exactly one caller — rpc.cpp's
//  RpcMethod::HAND_TRAJ_CMD case, on the RPC task. This is NOT the ISR+task
//  writer class that forces the PRIMASK idiom on can_buses.cpp's TX counters,
//  and adding a mask here would buy nothing. If a second caller ever appears —
//  above all one reachable from interp_isr — this comment is the thing that has
//  to be revisited, not just the increments.
struct HandOpsCounters {
  uint32_t calls;        // invocations, counted at entry before any gate
  uint32_t rej_homing;   // ERR_REJECTED: the homing interlock refused
  uint32_t bus_down;     // ERR_BUS_DOWN: jugglebot_commands_allowed() refused
  uint32_t pre1_fail;    // ERR_TIMEOUT at send #1 (set_state CLOSED_LOOP)
  uint32_t pre2_fail;    // ERR_TIMEOUT at send #2 (set_controller_mode)
  uint32_t traj_fail;    // ERR_TIMEOUT at send #3 (the 0x6D0 traj frame)
};

// Snapshot of the counters above. Read from the telemetry task (a torn read
// across fields is harmless for a 1 Hz cumulative instrument, same discipline
// as can_buses' snapshot_bus).
HandOpsCounters hand_ops_counters();

// ── Hand-dispatch interp-phase stamp (2026-08-09) — CONSOLE-ONLY DIAGNOSTIC ──
//  The 2026-08-09 bench proved the ERR_TIMEOUTs are TX-mailbox congestion, and the
//  entry's addendum § A3 explains the *rate* (37.5 %, not the naive ~6 %) with
//  PHASE-LOCKED DISPATCH QUANTISATION: task_net wakes only on 1 kHz FreeRTOS ticks,
//  and the 1 kHz SysTick and the 500 Hz interp PIT are exactly commensurate off one
//  24 MHz crystal, so a dispatch can occupy only TWO phases mod the 2 ms interp
//  period. One link in that chain (PERCLK's clock source) is INFERENCE, not read
//  from source — so the verdict ships with its own falsification test rather than
//  being believed because the fix went green.
//
//  PRE-REGISTERED READ (do not re-derive after seeing the data):
//    * phase_us clustered at two values ⇒ the phase model is CONFIRMED and
//      setMaxMB(24) is the right shape of fix;
//    * phase_us spread ~uniformly over 0-2000 ⇒ the verdict is REFUTED and the
//      mailbox-occupancy story re-opens.
//
//  NOT ON THE WIRE. This is a USB-serial `[handphase]` line off task_diag; no
//  MsgType, no payload change, no PROTOCOL_VERSION bump. A diagnostic that answers
//  one question on one bench session does not earn a permanent wire field.
struct HandPhaseSample {
  uint16_t phase_us;   // micros64() - interp_last_tick_us() at ENTRY: 0..~2000 within
                       // the 2 ms interp cycle. Stamped BEFORE the gates so EVERY call
                       // is stamped, including the ones that never reach a CAN send —
                       // an outcome-conditional stamp could not show that OK and FAIL
                       // occupy different phases, which is the whole comparison.
                       // Truncating cast: an interp stall > 65 ms wraps this, and a
                       // stamp taken before the first interp tick is meaningless.
  uint8_t  outcome;    // HandPhaseOutcome below
};

// Outcome codes. Deliberately a SEPARATE enumeration from RpcStatus: three of the
// five failure exits return the identical bare ERR_TIMEOUT, which is the whole
// reason the per-stage counters exist, so a status code cannot label a sample.
enum HandPhaseOutcome : uint8_t {
  HAND_PHASE_OK         = 0,
  HAND_PHASE_REJ_HOMING = 1,
  HAND_PHASE_BUS_DOWN   = 2,
  HAND_PHASE_PRE1       = 3,
  HAND_PHASE_PRE2       = 4,
  HAND_PHASE_TRAJ       = 5,
};

// Ring depth. 8 is one 1 Hz diag tick's worth at the bench ladder's ≥2 s dispatch
// spacing with a wide margin; the console print reports the total push count too, so
// an overrun is visible (+N > 8) rather than silent.
constexpr uint8_t HAND_PHASE_RING_LEN = 8;

struct HandPhaseRing {
  HandPhaseSample v[HAND_PHASE_RING_LEN];
  uint32_t        n;   // total pushes since boot; newest sample is v[(n-1) % LEN]
};

// Snapshot the ring. SINGLE WRITER (hand_traj_cmd on task_net, exactly like the
// counters above); read by task_diag and by the native tests. The race is BENIGN and
// bounded by construction: task_diag can copy a slot mid-write and print one garbled
// sample, or see `n` advance past a slot it already copied. Both are cosmetic on a
// 1 Hz console line at ≤0.5 Hz dispatch rates, and neither can affect control flow —
// nothing reads this ring except a printf. Masking IRQs to make a debug print tidy
// would add an IRQ-off window on the leg-setpoint path to buy nothing.
HandPhaseRing hand_phase_ring();

// Console label for an outcome code. Header-inline so the .ino's diag print needs no
// new link dependency (the same reason is_platform_reply_id lives in can_buses.h).
inline const char* hand_phase_outcome_name(uint8_t o) {
  switch (o) {
    case HAND_PHASE_OK:         return "OK";
    case HAND_PHASE_REJ_HOMING: return "rej_homing";
    case HAND_PHASE_BUS_DOWN:   return "bus_down";
    case HAND_PHASE_PRE1:       return "pre1";
    case HAND_PHASE_PRE2:       return "pre2";
    case HAND_PHASE_TRAJ:       return "traj";
    default:                    return "?";
  }
}

// Zero the counters AND the phase ring. Production never calls this (the file-statics
// boot zeroed); it is the test-isolation seam that keeps the native cases
// order-independent, exactly like version_check_init(). The ring is folded in here
// rather than given its own reset so a test can never isolate one instrument and
// silently inherit the other's leftovers.
void hand_ops_counters_reset();

namespace HandOps {

// HAND_TRAJ_CMD dispatch. Gated on jugglebot_commands_allowed() (a motion command,
// not a recovery one-shot — it keeps the heartbeat-staleness gate, NOT the SYNCH
// carve-out): a confirmed-stale/dead bus withholds the traj, like a leg setpoint.
// Sends the CLOSED_LOOP + POSITION/PASSTHROUGH preamble to the hand ODrive (axis
// 6) so the ensuing Platform-Teensy trajectory setpoints land, then forwards the
// payload on the firmware-owned 0x6D0 id. ABORTS the traj TX (no 0x6D0 frame) if a
// preamble send fails. Returns an RpcStatus.
//
// Every exit — including OK — is tallied into HandOpsCounters above. A send that
// "fails" here means can_jugglebot_send() returned false, which on a live bus
// means FlexCAN's write() deferred the frame into the software TX queue rather
// than dropping it (can_buses.h); the status is still ERR_TIMEOUT because the
// firmware cannot know whether the deferred frame will make its deadline.
uint16_t hand_traj_cmd(const JbUdp::RpcArgs::ArgHandTraj& a);

}  // namespace HandOps
}  // namespace CanBridge
