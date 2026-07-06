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
namespace HandOps {

// HAND_TRAJ_CMD dispatch. Gated on jugglebot_commands_allowed() (a motion command,
// not a recovery one-shot — it keeps the heartbeat-staleness gate, NOT the SYNCH
// carve-out): a confirmed-stale/dead bus withholds the traj, like a leg setpoint.
// Sends the CLOSED_LOOP + POSITION/PASSTHROUGH preamble to the hand ODrive (axis
// 6) so the ensuing Platform-Teensy trajectory setpoints land, then forwards the
// payload on the firmware-owned 0x6D0 id. ABORTS the traj TX (no 0x6D0 frame) if a
// preamble send fails. Returns an RpcStatus.
uint16_t hand_traj_cmd(const JbUdp::RpcArgs::ArgHandTraj& a);

}  // namespace HandOps
}  // namespace CanBridge
