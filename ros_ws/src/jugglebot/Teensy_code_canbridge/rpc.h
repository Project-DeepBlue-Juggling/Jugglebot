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

using JbUdp::RpcArgs::AXIS_ALL;   // broadcast to all legs (CLEAR_ERRORS/REBOOT)

// Register the server dispatcher with udp_link (call during setup).
void rpc_server_init();

}  // namespace Rpc
}  // namespace CanBridge
