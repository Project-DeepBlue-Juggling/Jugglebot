// =============================================================================
//  rpc.cpp — RPC envelope helpers + server dispatcher
// =============================================================================
#include "rpc.h"

#include "udp_protocol.h"
#include "udp_link.h"
#include "canbridge_config.h"
#include "odrive_protocol.h"
#include "ball_butler_protocol.h"
#include "ball_butler_state.h"
#include "can_buses.h"
#include "fault_machine.h"
#include "leg_homing.h"
#include "leg_activate.h"
#include "leg_deactivate.h"
#include "platform_relay.h"
#include "version_check.h"

namespace CanBridge {
namespace Rpc {

// RPC response result-blob buffer capacity (the `result[]` in on_request). The
// only method that returns a result blob today is GET_AXIS_VERSIONS
// (ResultAxisVersions = 57 B); keep this ≥ the largest result blob.
static constexpr uint16_t RESULT_BUF_CAP = 64;

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

// jugglebot_commands_allowed() ("never command a confirmed-dead bus") now lives in
// can_buses.cpp — it is shared with platform_relay.cpp (the relay reads/writes gate
// on the same CAN3-health predicate). See can_buses.h.

// Send an ODrive frame to a Jugglebot axis. Legs (0..5) always pass the (method,
// axis) check; the HAND ODrive (axis 6) is on CAN3 too and is forwarded ONLY for
// the narrow allow-table (JbUdp::hand_axis6_permitted — set/mode/gains/limits/
// clear/reboot/home/abs-pos), replacing the old blanket axis==HAND_AXIS reject so
// hand homing + the hand command surface work (Phase 1). Every permitted op — leg
// or hand — is still gated on jugglebot_commands_allowed(): the hand is GATED like
// a leg, never ungated. Returns an RpcStatus.
static uint16_t send_axis_frame(uint16_t method, uint8_t axis, const ODrive::CanFrame& f) {
  if (axis == HAND_AXIS) {
    if (!JbUdp::hand_axis6_permitted(method)) return JbUdp::RpcStatus::ERR_REJECTED;
  } else if (axis >= NUM_LEGS) {
    return JbUdp::RpcStatus::ERR_BAD_ARGS;
  }
  if (!jugglebot_commands_allowed()) return JbUdp::RpcStatus::ERR_BUS_DOWN;
  return can_jugglebot_send(f) ? JbUdp::RpcStatus::OK : JbUdp::RpcStatus::ERR_TIMEOUT;
}

// Ball Butler presence gate. CAN1 carries no other partner; with BB unpowered
// or unplugged, an un-ACKed TX would climb the FlexCAN TEC and eventually drop
// the bus to bus-off (same failure mode the cone-absent gate prevents on CAN2 —
// HANDOFF-firmware-three-bus D2, but for CAN1). bb_present() requires a fresh
// heartbeat; the bridge-level bb/calibrate handler translates ERR_BUS_DOWN to a
// silent-success for the homing flow (matching can_node._svc_bb_calibrate).
static bool bb_present() {
  // Single-byte volatile reads are atomic on Cortex-M7 — no seqlock needed for
  // this two-bool freshness check. (The decoded yaw/pitch/hand fields require
  // snapshot_bb() for cross-field coherence; here we only sample state metadata.)
  return bb_state.heartbeat_seen && !bb_state.heartbeat_stale;
}

// Send a BB frame to CAN1, gated on BB presence (above). Returns RpcStatus.
static uint16_t send_bb_frame(const ODrive::CanFrame& f) {
  if (!bb_present()) return JbUdp::RpcStatus::ERR_BUS_DOWN;
  return can_bb_send(f) ? JbUdp::RpcStatus::OK : JbUdp::RpcStatus::ERR_TIMEOUT;
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
      return send_axis_frame(method, a.axis, ODrive::encode_set_state(a.axis, a.state));
    }
    case RpcMethod::SET_CONTROLLER_MODE: {
      ArgControllerMode a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return send_axis_frame(method, a.axis, ODrive::encode_set_controller_mode(a.axis, a.ctrl, a.input));
    }
    case RpcMethod::SET_VEL_CURR_LIMITS: {
      ArgVelCurr a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return send_axis_frame(method, a.axis, ODrive::encode_set_vel_curr_limits(a.axis, a.vel_limit, a.curr_limit));
    }
    case RpcMethod::SET_POS_GAIN: {
      ArgPosGain a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return send_axis_frame(method, a.axis, ODrive::encode_set_pos_gain(a.axis, a.pos_gain));
    }
    case RpcMethod::SET_VEL_GAINS: {
      ArgVelGains a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return send_axis_frame(method, a.axis, ODrive::encode_set_vel_gains(a.axis, a.vel_gain, a.vel_int_gain));
    }
    case RpcMethod::SET_ABSOLUTE_POSITION: {
      ArgAbsPosition a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return send_axis_frame(method, a.axis, ODrive::encode_set_absolute_position(a.axis, a.position));
    }
    case RpcMethod::CLEAR_ERRORS: {
      ArgAxisOnly a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      // Operator clear refills the soft-reset auto-retry budget (clear_error_flags).
      fault_notify_clear_errors();
      if (a.axis == AXIS_ALL) {
        if (!jugglebot_commands_allowed()) return RpcStatus::ERR_BUS_DOWN;
        for (uint8_t i = 0; i < NUM_LEGS; ++i) can_jugglebot_send(ODrive::encode_clear_errors(i));
        return RpcStatus::OK;
      }
      return send_axis_frame(method, a.axis, ODrive::encode_clear_errors(a.axis));
    }
    case RpcMethod::REBOOT_ODRIVES: {
      ArgAxisOnly a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      if (a.axis == AXIS_ALL) {
        if (!jugglebot_commands_allowed()) return RpcStatus::ERR_BUS_DOWN;
        for (uint8_t i = 0; i < NUM_LEGS; ++i) can_jugglebot_send(ODrive::encode_reboot(i));
        return RpcStatus::OK;
      }
      return send_axis_frame(method, a.axis, ODrive::encode_reboot(a.axis));
    }
    case RpcMethod::SDO_READ: {
      ArgSdoRead a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      // Response value returns async on TxSdo (Phase 9 encoder-search consumes it).
      return send_axis_frame(method, a.axis, ODrive::encode_sdo_read(a.axis, a.endpoint));
    }
    case RpcMethod::SDO_WRITE: {
      ArgSdoWrite a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return send_axis_frame(method, a.axis, ODrive::encode_sdo_write(a.axis, a.endpoint, a.value));
    }
    case RpcMethod::ENCODER_SEARCH:
      // Encoder index search stays Jetson-orchestrated over SET_AXIS_STATE
      // (Phase 9a) — an ODrive-autonomous state needs no firmware move. The RPC
      // remains stubbed; see teensy_bridge_node._run_encoder_search.
      return RpcStatus::ERR_NOT_IMPL;
    case RpcMethod::HOME: {
      // Phase 9b: fire-and-monitor. Validate + latch a start, return immediately
      // (the move takes ~seconds; the net task must not block). The homing task
      // runs the velocity-limited move-to-hardstop; the Jetson observes
      // completion via telemetry (axis_state → IDLE, pos → home ref).
      ArgAxisOnly a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return homing_request(a.axis);
    }
    case RpcMethod::ACTIVATE: {
      // Phase 11 U5: fire-and-monitor TRAP_TRAJ move to the active pose. Validate
      // + latch a start, return immediately (the ODrive runs the trajectory; the
      // Jetson observes completion via telemetry). axis == AXIS_ALL activates
      // every present leg in parallel (even platform rise).
      ArgAxisOnly a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return activate_request(a.axis);
    }
    case RpcMethod::DEACTIVATE: {
      // Phase 11 U5: fire-and-monitor TRAP_TRAJ controlled lower to the STOW pose,
      // then IDLE. Validate + latch a start, return immediately (the ODrive runs
      // the descent; the Jetson observes the CLOSED_LOOP → IDLE completion via
      // telemetry). axis == AXIS_ALL deactivates every present leg in parallel
      // (even platform lowering).
      ArgAxisOnly a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return deactivate_request(a.axis);
    }

    // ── Platform-Teensy relay (Phase 1) ──────────────────────────────────────
    // Typed write RPCs that re-establish the Jetson↔Platform-Teensy conduit over
    // CAN3. TILT_READ/STATE_READ only TRIGGER a reply (the Platform answers on the
    // same id; on_jugglebot_rx forwards it verbatim as a PLATFORM_FRAME the host
    // correlates). STATE_WRITE carries the whole RobotState; the firmware encodes
    // the 0x6E0 frame (least-privilege — no Jetson-supplied raw frame). All gate on
    // jugglebot_commands_allowed() inside platform_relay (fail-fast ERR_BUS_DOWN).
    case RpcMethod::TILT_READ:
      return Relay::tilt_read();
    case RpcMethod::STATE_READ:
      return Relay::state_read();
    case RpcMethod::STATE_WRITE: {
      ArgRobotState a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return Relay::state_write(a);
    }

    // ── Firmware version pull (Phase 3) ──────────────────────────────────────
    // Synchronous: returns the bridge-LOCAL Get_Version cache (filled by the
    // cold-start version sweep, version_check.cpp) in the RPC response — NO CAN3
    // round-trip and NO PLATFORM_FRAME (unlike the relay reads). The Jetson
    // decodes the set-bit axes and runs validate_group; ZERO version semantics
    // here. version_fill_blob caps to RESULT_BUF_CAP defensively.
    case RpcMethod::GET_AXIS_VERSIONS:
      res_len = version_fill_blob(result, RESULT_BUF_CAP);
      return res_len ? RpcStatus::OK : RpcStatus::ERR_BAD_ARGS;

    // ── Reserved (canbridge-foundation-coldstart-parity Phase 0) ─────────────
    // Wire ids were allocated up-front (one codegen pass) so parallel sessions
    // cannot mint colliding ids. Each remaining method's real dispatch lands in
    // its owning phase; until then it answers ERR_NOT_IMPL (mirrors the
    // ENCODER_SEARCH stub above) so the firmware builds AND the RpcMethod
    // dispatch lint (tests/firmware/test_rpc_dispatch_lint.py) passes — every
    // method has a case. (GET_AXIS_VERSIONS [Phase 3] is now implemented above.)
    case RpcMethod::HAND_TRAJ_CMD:       // Phase 5 (hand traj + smooth-move)
      return RpcStatus::ERR_NOT_IMPL;

    // ── Ball Butler (CAN1) — typed commands ──────────────────────────────
    // Each gated on BB presence to prevent the un-ACKed-TX bus-off failure
    // mode (analogous to the CAN2 cone-absent gate). Bridge node
    // bb/calibrate translates ERR_BUS_DOWN to silent-success to preserve
    // can_node's homing semantics.
    case RpcMethod::BB_THROW: {
      ArgBbThrow a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      // Range-check BEFORE building the frame — refuses a malformed throw
      // before CAN1 sees a byte (HANDOFF-firmware-three-bus D2). Mirrors
      // ball_butler.py:98-105 ValueError raises.
      if (!BallButler::throw_args_valid(a.yaw_rad, a.pitch_rad,
                                        a.speed_mps, a.delay_s))
        return RpcStatus::ERR_BAD_ARGS;
      return send_bb_frame(BallButler::encode_throw(
          a.yaw_rad, a.pitch_rad, a.speed_mps, a.delay_s));
    }
    case RpcMethod::BB_RELOAD:
      return send_bb_frame(BallButler::encode_reload());
    case RpcMethod::BB_RESET:
      return send_bb_frame(BallButler::encode_reset());
    case RpcMethod::BB_CALIBRATE_LOC:
      return send_bb_frame(BallButler::encode_calibrate_loc());

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

  uint8_t result[RESULT_BUF_CAP];
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
