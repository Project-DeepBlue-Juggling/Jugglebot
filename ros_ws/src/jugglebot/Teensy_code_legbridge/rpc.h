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

namespace LegBridge {
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

// ── Per-method arg layouts (packed; little-endian) ────────────────────────────
#pragma pack(push, 1)
struct ArgAxisState     { uint8_t axis; uint32_t state; };                 // SET_AXIS_STATE
struct ArgControllerMode{ uint8_t axis; uint32_t ctrl; uint32_t input; };  // SET_CONTROLLER_MODE
struct ArgVelCurr       { uint8_t axis; float vel_limit; float curr_limit; };
struct ArgPosGain       { uint8_t axis; float pos_gain; };
struct ArgVelGains      { uint8_t axis; float vel_gain; float vel_int_gain; };
struct ArgAbsPosition   { uint8_t axis; float position; };
struct ArgAxisOnly      { uint8_t axis; };                                 // CLEAR_ERRORS / REBOOT
struct ArgSdoRead       { uint8_t axis; uint16_t endpoint; };
struct ArgSdoWrite      { uint8_t axis; uint16_t endpoint; float value; };
struct ResultTimeOfDay  { uint64_t jetson_wall_us; };                      // TIME_OF_DAY_QUERY result
#pragma pack(pop)

constexpr uint8_t AXIS_ALL = 0xFF;   // broadcast to all legs (CLEAR_ERRORS/REBOOT)

// Register the server dispatcher with udp_link (call during setup).
void rpc_server_init();

}  // namespace Rpc
}  // namespace LegBridge
