// =============================================================================
//  hand_ops.cpp — Hand trajectory / smooth-move conduit (Phase 5)
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
#include "can_buses.h"         // can_jugglebot_send, jugglebot_commands_allowed

namespace CanBridge {
namespace HandOps {

uint16_t hand_traj_cmd(const JbUdp::RpcArgs::ArgHandTraj& a) {
  using namespace JbUdp;   // RpcStatus::* (JbUdp::RpcStatus is a namespace, not a type)
  // Gate like a leg motion command: a confirmed-stale/dead CAN3 must withhold the
  // trajectory. HAND_TRAJ_CMD is NOT a recovery one-shot, so it keeps the
  // heartbeat-staleness gate (jugglebot_commands_allowed) — NOT the SYNCH
  // bus-transmittable carve-out that CLEAR_ERRORS/REBOOT_ODRIVES use (Phase 6).
  if (!jugglebot_commands_allowed()) return RpcStatus::ERR_BUS_DOWN;

  // Preamble — bring the hand ODrive (axis 6) to CLOSED_LOOP + POSITION/PASSTHROUGH
  // so the Platform Teensy's ensuing trajectory setpoints land. can_node issued
  // this in BOTH _send_hand_traj_cmd and _smooth_move_hand (the dropped precondition
  // the audit flagged, row 37). ABORT the traj TX if either preamble frame fails to
  // enqueue — running a trajectory against a hand that is not in CLOSED_LOOP/
  // PASSTHROUGH would fault or silently no-op the move.
  if (!can_jugglebot_send(ODrive::encode_set_state(HAND_AXIS, ODriveState::CLOSED_LOOP)))
    return RpcStatus::ERR_TIMEOUT;
  if (!can_jugglebot_send(ODrive::encode_set_controller_mode(
          HAND_AXIS, ODriveControlMode::POSITION, ODriveInputMode::PASSTHROUGH)))
    return RpcStatus::ERR_TIMEOUT;

  // Forward the host-built 8-byte payload on the FIRMWARE-OWNED 0x6D0 id. The Jetson
  // supplies the payload (incl. the absolute wall_time_ms deadline baked in); the
  // firmware owns the arbitration id + dlc and does NOT re-stamp — it never sees the
  // deadline as anything but opaque bytes.
  ODrive::CanFrame f;
  f.id  = PlatformCanId::TRAJ_CMD;   // 0x6D0
  f.len = 8;
  memcpy(f.buf, a.payload, 8);
  return can_jugglebot_send(f) ? RpcStatus::OK : RpcStatus::ERR_TIMEOUT;
}

}  // namespace HandOps
}  // namespace CanBridge
