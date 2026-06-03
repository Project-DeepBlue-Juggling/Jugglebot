// =============================================================================
//  rpc.cpp — RPC envelope helpers + server dispatcher
// =============================================================================
#include "rpc.h"

#include "udp_protocol.h"
#include "udp_link.h"
#include "canbridge_config.h"
#include "odrive_protocol.h"
#include "can_buses.h"
#include "fault_machine.h"

namespace CanBridge {
namespace Rpc {

// ── Envelope ──────────────────────────────────────────────────────────────────
uint16_t pack_request(uint16_t method, uint16_t req_id,
                      const uint8_t* args, uint16_t arg_len,
                      uint8_t* out, uint16_t out_cap) {
  if ((uint32_t)REQ_HEAD + arg_len > out_cap) return 0;
  memcpy(out + 0, &method, 2);
  memcpy(out + 2, &req_id, 2);
  memcpy(out + 4, &arg_len, 2);
  const uint16_t pad = 0; memcpy(out + 6, &pad, 2);
  if (arg_len && args) memcpy(out + REQ_HEAD, args, arg_len);
  return REQ_HEAD + arg_len;
}

bool parse_request(const uint8_t* payload, uint16_t len,
                   uint16_t* method, uint16_t* req_id,
                   const uint8_t** args, uint16_t* arg_len) {
  if (len < REQ_HEAD) return false;
  memcpy(method, payload + 0, 2);
  memcpy(req_id, payload + 2, 2);
  memcpy(arg_len, payload + 4, 2);
  if ((uint32_t)REQ_HEAD + *arg_len > len) return false;
  *args = payload + REQ_HEAD;
  return true;
}

uint16_t pack_response(uint16_t method, uint16_t req_id, uint16_t status,
                       const uint8_t* result, uint16_t res_len,
                       uint8_t* out, uint16_t out_cap) {
  if ((uint32_t)RESP_HEAD + res_len > out_cap) return 0;
  memcpy(out + 0, &method, 2);
  memcpy(out + 2, &req_id, 2);
  memcpy(out + 4, &status, 2);
  memcpy(out + 6, &res_len, 2);
  if (res_len && result) memcpy(out + RESP_HEAD, result, res_len);
  return RESP_HEAD + res_len;
}

bool parse_response(const uint8_t* payload, uint16_t len,
                    uint16_t* method, uint16_t* req_id, uint16_t* status,
                    const uint8_t** result, uint16_t* res_len) {
  if (len < RESP_HEAD) return false;
  memcpy(method, payload + 0, 2);
  memcpy(req_id, payload + 2, 2);
  memcpy(status, payload + 4, 2);
  memcpy(res_len, payload + 6, 2);
  if ((uint32_t)RESP_HEAD + *res_len > len) return false;
  *result = payload + RESP_HEAD;
  return true;
}

// ── Server dispatch ───────────────────────────────────────────────────────────

// "Never command a confirmed-dead bus." Reject CAN-bound RPCs when CAN2 is in a
// WARN/BUS_OFF state (the proxy for fatal_can_error until Phase 8 wires the
// authoritative fault-machine predicate). OK/UNKNOWN both allow commands so the
// initial bring-up sequence (set CLOSED_LOOP before telemetry warms) works.
static bool can2_commands_allowed() {
  const uint8_t h = can_buses_stats().can2_health;
  return h != JbUdp::BusHealth::WARN && h != JbUdp::BusHealth::BUS_OFF;
}

// Send an ODrive frame to a leg axis (0..5). Hand (axis 6) is owned by the
// platform Teensy in this architecture → reject. Returns an RpcStatus.
static uint16_t send_leg_frame(uint8_t axis, const ODrive::CanFrame& f) {
  if (axis == HAND_AXIS) return JbUdp::RpcStatus::ERR_REJECTED;  // platform Teensy owns the hand
  if (axis >= NUM_LEGS)  return JbUdp::RpcStatus::ERR_BAD_ARGS;
  if (!can2_commands_allowed()) return JbUdp::RpcStatus::ERR_BUS_DOWN;
  return can2_send(f) ? JbUdp::RpcStatus::OK : JbUdp::RpcStatus::ERR_TIMEOUT;
}

template <typename Arg>
static bool take(const uint8_t* args, uint16_t arg_len, Arg& out) {
  if (arg_len < sizeof(Arg)) return false;
  memcpy(&out, args, sizeof(Arg));
  return true;
}

static uint16_t dispatch(uint16_t method, const uint8_t* args, uint16_t arg_len,
                         uint8_t* result, uint16_t& res_len) {
  res_len = 0;
  using namespace JbUdp;
  switch (method) {
    case RpcMethod::NOP:
      return RpcStatus::OK;

    case RpcMethod::SET_AXIS_STATE: {
      ArgAxisState a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return send_leg_frame(a.axis, ODrive::encode_set_state(a.axis, a.state));
    }
    case RpcMethod::SET_CONTROLLER_MODE: {
      ArgControllerMode a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return send_leg_frame(a.axis, ODrive::encode_set_controller_mode(a.axis, a.ctrl, a.input));
    }
    case RpcMethod::SET_VEL_CURR_LIMITS: {
      ArgVelCurr a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return send_leg_frame(a.axis, ODrive::encode_set_vel_curr_limits(a.axis, a.vel_limit, a.curr_limit));
    }
    case RpcMethod::SET_POS_GAIN: {
      ArgPosGain a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return send_leg_frame(a.axis, ODrive::encode_set_pos_gain(a.axis, a.pos_gain));
    }
    case RpcMethod::SET_VEL_GAINS: {
      ArgVelGains a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return send_leg_frame(a.axis, ODrive::encode_set_vel_gains(a.axis, a.vel_gain, a.vel_int_gain));
    }
    case RpcMethod::SET_ABSOLUTE_POSITION: {
      ArgAbsPosition a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return send_leg_frame(a.axis, ODrive::encode_set_absolute_position(a.axis, a.position));
    }
    case RpcMethod::CLEAR_ERRORS: {
      ArgAxisOnly a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      // Operator clear refills the soft-reset auto-retry budget (clear_error_flags).
      fault_notify_clear_errors();
      if (a.axis == AXIS_ALL) {
        if (!can2_commands_allowed()) return RpcStatus::ERR_BUS_DOWN;
        for (uint8_t i = 0; i < NUM_LEGS; ++i) can2_send(ODrive::encode_clear_errors(i));
        return RpcStatus::OK;
      }
      return send_leg_frame(a.axis, ODrive::encode_clear_errors(a.axis));
    }
    case RpcMethod::REBOOT_ODRIVES: {
      ArgAxisOnly a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      if (a.axis == AXIS_ALL) {
        if (!can2_commands_allowed()) return RpcStatus::ERR_BUS_DOWN;
        for (uint8_t i = 0; i < NUM_LEGS; ++i) can2_send(ODrive::encode_reboot(i));
        return RpcStatus::OK;
      }
      return send_leg_frame(a.axis, ODrive::encode_reboot(a.axis));
    }
    case RpcMethod::SDO_READ: {
      ArgSdoRead a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      // Response value returns async on TxSdo (Phase 9 encoder-search consumes it).
      return send_leg_frame(a.axis, ODrive::encode_sdo_read(a.axis, a.endpoint));
    }
    case RpcMethod::SDO_WRITE: {
      ArgSdoWrite a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return send_leg_frame(a.axis, ODrive::encode_sdo_write(a.axis, a.endpoint, a.value));
    }
    case RpcMethod::ENCODER_SEARCH:
    case RpcMethod::HOME:
      return RpcStatus::ERR_NOT_IMPL;   // Phase 9

    case RpcMethod::TIME_OF_DAY_QUERY:
      // The Teensy is the CLIENT for this method; it should never receive it as
      // a server request. Reject defensively.
      return RpcStatus::ERR_UNKNOWN_METHOD;

    default:
      return RpcStatus::ERR_UNKNOWN_METHOD;
  }
}

static void on_request(uint16_t /*seq*/, const uint8_t* payload, uint16_t len) {
  uint16_t method, req_id, arg_len;
  const uint8_t* args;
  if (!parse_request(payload, len, &method, &req_id, &args, &arg_len)) return;

  uint8_t result[64];
  uint16_t res_len = 0;
  const uint16_t status = dispatch(method, args, arg_len, result, res_len);

  uint8_t out[JbUdp::MAX_PAYLOAD];
  const uint16_t n = pack_response(method, req_id, status,
                                   res_len ? result : nullptr, res_len,
                                   out, sizeof(out));
  if (n) udp_send_rpc(JbUdp::MsgType::RPC_RESPONSE, out, n);
}

void rpc_server_init() {
  udp_on_rpc_request(on_request);
}

}  // namespace Rpc
}  // namespace CanBridge
