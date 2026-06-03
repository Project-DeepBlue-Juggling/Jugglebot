#pragma once
// =============================================================================
//  can_buses.h — dual FlexCAN_T4 setup, RX decode into the cache, and TX
// =============================================================================
//  CAN1 = shared bus (hand ODrive telemetry, platform Teensy, BB, cone). We are
//         the time-sync MASTER here (broadcast 0x7DD) and we observe hand-axis
//         telemetry for the uplink. Other CAN1 traffic is ignored.
//  CAN2 = private leg bus (6 leg ODrives). All leg telemetry + commands.
//
//  RX is dispatched from canX.events() (pumped by the CAN RX task) via onReceive
//  callbacks — the proven platform-Teensy idiom (Teensy_code.ino canSniff) — and
//  decoded straight into the per-axis cache (fast, bounded; no queue hop). This
//  deviates from the plan's "FlexCAN ISR + queue → RX task" purely for lower
//  latency and fewer moving parts; the decode is microseconds and the cache
//  writes are seqlock-guarded.
// =============================================================================

#include <cstdint>
#include "odrive_protocol.h"
#include "udp_protocol.h"   // JbUdp::BusHealth

namespace CanBridge {

void can_buses_init();      // begin both buses, register RX callbacks
void can_buses_service();   // pump can1.events() + can2.events(); call from CAN RX task

bool can1_send(const ODrive::CanFrame& f);   // shared bus TX (time-sync broadcast)
bool can2_send(const ODrive::CanFrame& f);   // leg bus TX (setpoints, RPC-driven cmds)

struct CanStats {
  uint32_t can1_rx, can1_tx, can2_rx, can2_tx;
  uint8_t  can1_health;   // JbUdp::BusHealth
  uint8_t  can2_health;
};
CanStats can_buses_stats();

}  // namespace CanBridge
