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
#include "hand_ops.h"
#include "hand_source.h"   // hand_source_request (HAND_SOURCE_SET, FW 17)

namespace CanBridge {
namespace Rpc {

// RPC response result-blob buffer capacity (the `result[]` in on_request). The
// only method that returns a result blob today is GET_AXIS_VERSIONS
// (ResultAxisVersions = 57 B); keep this ≥ the largest result blob.
static constexpr uint16_t RESULT_BUF_CAP = 64;
// Build-time guard: fail the build if the largest RPC result blob outgrows the buffer, so a
// future growth of ResultAxisVersions (or a new result-bearing method) can never
// silently truncate in version_fill_blob / dispatch instead of being caught here.
static_assert(RESULT_BUF_CAP >= sizeof(JbUdp::RpcArgs::ResultAxisVersions),
              "RESULT_BUF_CAP too small for the largest RPC result blob");

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

// The single CAN3 gate chokepoint. CLEAR_ERRORS / REBOOT_ODRIVES are
// non-motion recovery one-shots that gate on the LIVE bus-transmittable signal
// (jugglebot_bus_transmittable() = ESR1.SYNCH) so a recovery clear reaches a
// just-repowered bus that is electrically alive but not yet heartbeating (the
// 2026-06-27 deadlock); every other op keeps the heartbeat-staleness gate
// (jugglebot_commands_allowed) — a stale bus SHOULD withhold a setpoint/config.
// Both send_axis_frame() (per-axis) and the AXIS_ALL loops route through here, so the
// method→gate-basis mapping has exactly one enforcement point. The policy predicate is
// header-inline in rpc.h (method_gates_on_bus_transmittable) and pinned by the harness.
static bool gate_allows(uint16_t method) {
  return method_gates_on_bus_transmittable(method) ? jugglebot_bus_transmittable()
                                                   : jugglebot_commands_allowed();
}

// Send an ODrive frame to a Jugglebot axis. Legs (0..5) always pass the (method,
// axis) check; the HAND ODrive (axis 6) is on CAN3 too and is forwarded ONLY for
// the narrow allow-table (JbUdp::hand_axis6_permitted — set/mode/gains/limits/
// clear/reboot/home/abs-pos), replacing the old blanket axis==HAND_AXIS reject so
// hand homing + the hand command surface work. Every permitted op — leg
// or hand — is still gated via gate_allows(): the hand is GATED like a leg, never
// ungated. Returns an RpcStatus.
static uint16_t send_axis_frame(uint16_t method, uint8_t axis, const ODrive::CanFrame& f) {
  if (axis == HAND_AXIS) {
    if (!JbUdp::hand_axis6_permitted(method)) return JbUdp::RpcStatus::ERR_REJECTED;
  } else if (axis >= NUM_LEGS) {
    return JbUdp::RpcStatus::ERR_BAD_ARGS;
  }
  if (!gate_allows(method)) return JbUdp::RpcStatus::ERR_BUS_DOWN;
  // TxCls (2026-08-24) derived from the SAME predicate that picks the gate basis,
  // so the classification has one enforcement point and no call site can drift:
  // CLEAR_ERRORS / REBOOT_ODRIVES are the recovery one-shots and ride the
  // dedicated safety-deferral counter; everything else is ordinary RPC. DEFERRED
  // = SENT, so the ack is OK — the frame is queued and transmits in order.
  // TxResult::FAILED (no bus partner) is the only ERR_TIMEOUT, which is exactly
  // the true-failure retry basis the callers below already act on.
  const uint8_t cls = method_gates_on_bus_transmittable(method) ? TxCls::SAFETY : TxCls::RPC;
  return tx_reached_the_wire(can_jugglebot_tx(f, cls))
             ? JbUdp::RpcStatus::OK : JbUdp::RpcStatus::ERR_TIMEOUT;
}

// Ball Butler presence gate. CAN1 carries no other partner; with BB unpowered
// or unplugged, an un-ACKed TX would climb the FlexCAN TEC and eventually drop
// the bus to bus-off (same failure mode the cone-absent gate prevents on CAN2 —
// but for CAN1). bb_present() requires a fresh
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
  return tx_reached_the_wire(can_bb_tx(f, TxCls::RPC))
             ? JbUdp::RpcStatus::OK : JbUdp::RpcStatus::ERR_TIMEOUT;
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
      if (a.axis == AXIS_ALL) {
        // Gate on the bus-transmittable (SYNCH) signal, not staleness, so a
        // recovery clear reaches a just-repowered bus. AXIS_ALL loops legs + hand
        // (i < NUM_AXES) — can_node JUGGLEBOT_AXES parity (the audit's dropped-hand row).
        if (!gate_allows(method)) return RpcStatus::ERR_BUS_DOWN;
        // Send FIRST, notify only if at least one frame was actually enqueued (
        // a clear that cannot reach the bus must NOT refill the soft-reset auto-retry
        // budget or release the guard E-STOP latch). The SYNCH gate above passes on a
        // fully dead-but-idle-recessive bus, and the bus-partner presence gate
        // (2026-07-05) then refuses every send in the parked state — so notify-before-
        // send would fire the side-effects with nothing on the wire. any (not all-ok)
        // is the notify condition: a PARTIAL enqueue still reached the bus. Accumulate
        // the per-axis result: partial failure surfaces as ERR_TIMEOUT; the
        // send is the LEFT operand of &&/|=, so every axis is attempted regardless.
        bool ok = true, any = false;
        for (uint8_t i = 0; i < NUM_AXES; ++i) {
          // TxCls::SAFETY. `any`/`ok` keep their exact meaning: a DEFERRED clear
          // IS on its way to the bus, so it refills the budget and releases the
          // guard latch — which is what the "must not fire the side-effects with
          // nothing on the wire" rule was always trying to express.
          const bool s = tx_reached_the_wire(
              can_jugglebot_tx(ODrive::encode_clear_errors(i), TxCls::SAFETY));
          any |= s; ok = s && ok;
        }
        if (any) fault_notify_clear_errors();
        return ok ? RpcStatus::OK : RpcStatus::ERR_TIMEOUT;
      }
      // Per-axis: refill the budget only if the frame actually went out (gate passed +
      // enqueue OK) — the same after-the-gate discipline as the AXIS_ALL path.
      const uint16_t st = send_axis_frame(method, a.axis, ODrive::encode_clear_errors(a.axis));
      if (st == RpcStatus::OK) fault_notify_clear_errors();
      return st;
    }
    case RpcMethod::REBOOT_ODRIVES: {
      ArgAxisOnly a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      // Never REBOOT during a deferred stow (pending OR in-progress). A
      // reboot mid-descent disarms the raised legs → gravity drop; the same hazard
      // class as the DEACTIVATE-during-stow reject. Mirrors the cold-start requests'
      // fault_stow_pending() interlock (homing/activate/deactivate_request). Rejects
      // every axis (incl. a hand-only reboot) for a simple, conservative gate — a
      // stow completes in ~2-3 s, after which REBOOT is available again.
      if (fault_stow_pending()) return RpcStatus::ERR_REJECTED;
      if (a.axis == AXIS_ALL) {
        // Bus-transmittable gate; AXIS_ALL loops legs + hand (i < NUM_AXES —
        // JUGGLEBOT_AXES parity). Arm the watchdog-suppression latch only ONCE the
        // reboot is actually going out (after the gate passes), so a refused reboot
        // does not blind the detector against a real, coincident loss.
        if (!gate_allows(method)) return RpcStatus::ERR_BUS_DOWN;
        // Send FIRST, arm the watchdog-suppression latch only if at least one reboot
        // frame was actually enqueued — a refused broadcast (SYNCH gate passes on a
        // dead-but-idle bus; the presence gate then refuses every send) must not blind
        // the CAN-loss detector for 6 s with nothing on the wire. any (not all-ok):
        // a partial broadcast still reboots some ODrives, so the coming silence is
        // deliberate and the latch must arm. Accumulate the per-axis result:
        // partial failure surfaces as ERR_TIMEOUT; every axis is still attempted.
        bool ok = true, any = false;
        for (uint8_t i = 0; i < NUM_AXES; ++i) {
          const bool s = tx_reached_the_wire(
              can_jugglebot_tx(ODrive::encode_reboot(i), TxCls::SAFETY));   // TxCls::SAFETY
          any |= s; ok = s && ok;
        }
        if (any) fault_notify_reboot_started();
        return ok ? RpcStatus::OK : RpcStatus::ERR_TIMEOUT;
      }
      // Per-axis: send via the shared gate; arm the latch only if the reboot went out
      // AND it targets a LEG. The CAN-loss detector watches leg heartbeats only
      // (any_leg_heartbeat_stale), so a hand-only reboot (axis 6) can never false-trip
      // it — arming for it would blind LEG-loss detection for the full window with zero
      // benefit (saw_stale never sets → no fresh-after-stale release). AXIS_ALL arms in
      // its own branch (it reboots the legs too).
      const uint16_t st = send_axis_frame(method, a.axis, ODrive::encode_reboot(a.axis));
      if (st == RpcStatus::OK && a.axis < NUM_LEGS) fault_notify_reboot_started();
      return st;
    }
    case RpcMethod::SDO_READ: {
      ArgSdoRead a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      // The TxSdo reply has NO return path to this RPC — nothing correlates it
      // back to the caller, and no encoder-search consumer has ever existed. The
      // bridge's only TxSdo consumer is gpio_poll's hand ball-sensor poll (axis 6,
      // get_gpio_states), which this RPC cannot reach anyway: hand_axis6_permitted
      // rejects SDO_READ for axis 6.
      return send_axis_frame(method, a.axis, ODrive::encode_sdo_read(a.axis, a.endpoint));
    }
    case RpcMethod::SDO_WRITE: {
      ArgSdoWrite a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return send_axis_frame(method, a.axis, ODrive::encode_sdo_write(a.axis, a.endpoint, a.value));
    }
    case RpcMethod::ENCODER_SEARCH:
      // Encoder index search stays Jetson-orchestrated over SET_AXIS_STATE
      // — an ODrive-autonomous state needs no firmware move. The RPC
      // remains stubbed; see teensy_bridge_node._run_encoder_search.
      return RpcStatus::ERR_NOT_IMPL;
    case RpcMethod::HOME: {
      // Fire-and-monitor. Validate + latch a start, return immediately
      // (the move takes ~seconds; the net task must not block). The homing task
      // runs the velocity-limited move-to-hardstop; the Jetson observes
      // completion via telemetry (axis_state → IDLE, pos → home ref).
      //
      // Axis-6 policy: HOME does NOT route through send_axis_frame, so its
      // per-axis gate is NOT the hand_axis6_permitted allow-table — it is owned
      // wholly by homing_request(), the single enforcement point for the HOME path.
      // homing_request accepts legs 0..5 AND the hand (axis 6, homed with the
      // Homing::HAND_* params), rejects AXIS_ALL / out-of-range, and
      // presence/bus/concurrency-gates. This is consistent with the allow-table,
      // which also permits HOME for the hand (hand_axis6_permitted(HOME) == true);
      // the two never diverge because homing_request is the only path HOME takes.
      ArgAxisOnly a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return homing_request(a.axis);
    }
    case RpcMethod::ACTIVATE: {
      // Fire-and-monitor TRAP_TRAJ move to the active pose. Validate
      // + latch a start, return immediately (the ODrive runs the trajectory; the
      // Jetson observes completion via telemetry). axis == AXIS_ALL activates
      // every present leg in parallel (even platform rise).
      ArgAxisOnly a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return activate_request(a.axis);
    }
    case RpcMethod::DEACTIVATE: {
      // Fire-and-monitor TRAP_TRAJ controlled lower to the STOW pose,
      // then IDLE. Validate + latch a start, return immediately (the ODrive runs
      // the descent; the Jetson observes the CLOSED_LOOP → IDLE completion via
      // telemetry). axis == AXIS_ALL deactivates every present leg in parallel
      // (even platform lowering).
      ArgAxisOnly a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return deactivate_request(a.axis);
    }

    // ── Platform-Teensy relay ──────────────────────────────────────
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

    // ── Firmware version pull ──────────────────────────────────────
    // Synchronous: returns the bridge-LOCAL Get_Version cache (filled by the
    // cold-start version sweep, version_check.cpp) in the RPC response — NO CAN3
    // round-trip and NO PLATFORM_FRAME (unlike the relay reads). The Jetson
    // decodes the set-bit axes and runs validate_group; ZERO version semantics
    // here. version_fill_blob caps to RESULT_BUF_CAP defensively.
    case RpcMethod::GET_AXIS_VERSIONS:
      res_len = version_fill_blob(result, RESULT_BUF_CAP);
      return res_len ? RpcStatus::OK : RpcStatus::ERR_BAD_ARGS;

    // ── Hand trajectory / smooth-move ──────────────────────────────
    // set_hand_traj_cmd + smooth_move_hand both ride this one RPC. The host builds
    // the exact 8-byte 0x6D0 payload (byte-0 discriminator: 0/1/2 = catch-traj type,
    // 3 = smooth-move, byte-identical to can_node); hand_ops sends the CLOSED_LOOP +
    // POSITION/PASSTHROUGH preamble then forwards it on the firmware-owned 0x6D0 id,
    // aborting if a preamble send fails. This is the last of the reserved
    // ERR_NOT_IMPL stubs to land — no reserved methods remain (the dispatch lint's
    // reserved list is now empty; every id has a real case).
    case RpcMethod::HAND_TRAJ_CMD: {
      ArgHandTraj a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return HandOps::hand_traj_cmd(a);
    }

    // ── Hand-mastery latch (unified-7dof FW 17, plan § 2.4) ────────────────
    // ADDITIVE method — no PROTOCOL_VERSION bump (the LegCmd/HandSensor
    // precedent: an FW ≤ 16 board answers ERR_UNKNOWN_METHOD, loudly).
    // Bridge-LOCAL: no CAN frame is sent, so no bus gate — the acceptance
    // gates live in hand_source_request (single enforcement point): value
    // valid, !mpc_active (passed from the fault machine here so hand_source.cpp
    // stays fault-machine-free for the native harness), and the hand settled at
    // a rest position on FRESH axis-6 telemetry. Boot default LEGACY_STROKE;
    // only a reboot or this RPC moves the latch.
    case RpcMethod::HAND_SOURCE_SET: {
      ArgHandSource a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      return hand_source_request(a.source, fault_mpc_active());
    }

    // ── Ball Butler (CAN1) — typed commands ──────────────────────────────
    // Each gated on BB presence to prevent the un-ACKed-TX bus-off failure
    // mode (analogous to the CAN2 cone-absent gate). Bridge node
    // bb/calibrate translates ERR_BUS_DOWN to silent-success to preserve
    // can_node's homing semantics.
    case RpcMethod::BB_THROW: {
      ArgBbThrow a; if (!take(args, arg_len, a)) return RpcStatus::ERR_BAD_ARGS;
      // Range-check BEFORE building the frame — refuses a malformed throw
      // before CAN1 sees a byte. Mirrors
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
