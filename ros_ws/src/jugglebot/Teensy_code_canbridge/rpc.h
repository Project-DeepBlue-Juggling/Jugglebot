#pragma once
// =============================================================================
//  rpc.h — RPC envelope helpers + server dispatcher (RPC port, PORT_RPC)
// =============================================================================
//  RPC framing (JbUdp::RpcRequest / RpcResponse fixed heads + variable blob):
//    request  = [method u16][req_id u16][arg_len u16][pad u16][args...]
//    response = [method u16][req_id u16][status u16][res_len u16][result...]
//
//  The Teensy is the RPC SERVER for Jetson-initiated ODrive control (set state,
//  controller mode, gains, limits, clear errors, reboot, SDO). It is the RPC
//  CLIENT for exactly one method — TIME_OF_DAY_QUERY — the time-sync master's
//  wall-clock bootstrap/drift query (handled in time_sync_master).
//
//  Method arg layouts (defined as packed structs below) are a per-method
//  sub-contract. They are NOT yet in the single-source generator because the
//  consumer (Jetson UDP bridge) is Phase 10 / out of scope; co-located here for
//  now, to be hoisted into config/generate_udp_protocol.py at Phase 10.
//  See handoff "Decisions made autonomously".
// =============================================================================

#include <cstdint>
#include <cstring>
#include "udp_protocol.h"

namespace CanBridge {
namespace Rpc {

// ── Envelope (de)serialisation ────────────────────────────────────────────────
constexpr uint16_t REQ_HEAD = JbUdp::RPC_REQUEST_SIZE;    // 8
constexpr uint16_t RESP_HEAD = JbUdp::RPC_RESPONSE_SIZE;  // 8

// Build a request payload into `out` (cap out_cap). Returns payload length or 0.
uint16_t pack_request(uint16_t method, uint16_t req_id,
                      const uint8_t* args, uint16_t arg_len,
                      uint8_t* out, uint16_t out_cap);

// Parse a request payload. Returns true + fills outs (args points into payload).
bool parse_request(const uint8_t* payload, uint16_t len,
                   uint16_t* method, uint16_t* req_id,
                   const uint8_t** args, uint16_t* arg_len);

uint16_t pack_response(uint16_t method, uint16_t req_id, uint16_t status,
                       const uint8_t* result, uint16_t res_len,
                       uint8_t* out, uint16_t out_cap);

bool parse_response(const uint8_t* payload, uint16_t len,
                    uint16_t* method, uint16_t* req_id, uint16_t* status,
                    const uint8_t** result, uint16_t* res_len);

// ── Per-method arg layouts ────────────────────────────────────────────────────
// HOISTED into the single-source generator at Phase 10b (firmware handoff D8):
// the packed structs now live in JbUdp::RpcArgs (config/generate_udp_protocol.py
// → udp_protocol.h), since the Jetson UDP bridge became the second consumer.
// We consume them here so rpc.cpp's dispatch is unchanged. The generated
// `static_assert(sizeof(...))`s guard size drift. Jetson-side consumer:
// controller/teensy_link/rpc_args.py; round-trip test: tests/teensy_link/test_rpc_args.py.
using JbUdp::RpcArgs::ArgAxisState;
using JbUdp::RpcArgs::ArgControllerMode;
using JbUdp::RpcArgs::ArgVelCurr;
using JbUdp::RpcArgs::ArgPosGain;
using JbUdp::RpcArgs::ArgVelGains;
using JbUdp::RpcArgs::ArgAbsPosition;
using JbUdp::RpcArgs::ArgAxisOnly;
using JbUdp::RpcArgs::ArgSdoRead;
using JbUdp::RpcArgs::ArgSdoWrite;
using JbUdp::RpcArgs::ResultTimeOfDay;
using JbUdp::RpcArgs::ArgBbThrow;
using JbUdp::RpcArgs::ArgRobotState;   // STATE_WRITE (Platform-Teensy relay — Phase 1)
using JbUdp::RpcArgs::ArgHandTraj;     // HAND_TRAJ_CMD (hand traj + smooth-move — Phase 5)

using JbUdp::RpcArgs::AXIS_ALL;   // broadcast to all axes (CLEAR_ERRORS/REBOOT)

// Phase 6 gate-basis policy: which operator RPCs gate on the bus-TRANSMITTABLE signal
// (live ESR1.SYNCH, jugglebot_bus_transmittable()) instead of the default heartbeat-
// staleness gate (jugglebot_commands_allowed()). CLEAR_ERRORS and REBOOT_ODRIVES are
// non-motion recovery one-shots that MUST reach the ODrives on a just-repowered /
// motor-cycled bus that is electrically alive but has not heartbeated yet (the
// 2026-06-27 deadlock); every other op keeps the staleness gate (a stale bus means a
// dropped setpoint/config, which SHOULD be withheld). Pure + header-inline so the
// native harness pins the policy directly — a future edit that drops a method from
// this set (re-gating clear/reboot back onto staleness) fails the test. rpc.cpp's
// single gate chokepoint (gate_allows) routes on this predicate.
inline bool method_gates_on_bus_transmittable(uint16_t method) {
  return method == JbUdp::RpcMethod::CLEAR_ERRORS ||
         method == JbUdp::RpcMethod::REBOOT_ODRIVES;
}

// Register the server dispatcher with udp_link (call during setup).
void rpc_server_init();

}  // namespace Rpc
}  // namespace CanBridge
