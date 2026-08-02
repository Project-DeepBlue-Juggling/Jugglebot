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

// Zero the counters. Production never calls this (the file-statics boot zeroed);
// it is the test-isolation seam that keeps the native cases order-independent,
// exactly like version_check_init().
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
