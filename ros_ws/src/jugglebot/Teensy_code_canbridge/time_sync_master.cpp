// =============================================================================
//  time_sync_master.cpp — 0x7DD broadcast + time-of-day RPC client
// =============================================================================
#include "time_sync_master.h"

#include <cstring>
#include "canbridge_config.h"
#include "protocol_config.h"
#include "udp_protocol.h"
#include "udp_link.h"
#include "rpc.h"
#include "odrive_protocol.h"
#include "can_buses.h"
#include "time_base.h"

namespace CanBridge {

// ── Time-of-day client state ──────────────────────────────────────────────────
static uint16_t s_req_id = 0;
static uint16_t s_pending_req = 0;
static bool     s_pending = false;
static uint64_t s_send_us = 0;          // micros64() at last TOD request
static uint64_t s_next_query_us = 0;    // when to send the next TOD request

// ── 0x7DD broadcast (matches bus.py broadcast_time: pack('<II', sec, usec)) ───
static void broadcast_0x7dd() {
  if (!time_synced()) return;           // no valid wall-clock yet → stay silent
  const uint64_t w = now_wall_us();
  const uint32_t sec  = (uint32_t)(w / 1000000ULL);
  const uint32_t usec = (uint32_t)(w % 1000000ULL);
  ODrive::CanFrame f;
  f.id = SharedCanId::TIME_SYNC;        // 0x7DD
  f.len = 8;
  memcpy(&f.buf[0], &sec, 4);           // little-endian, matches '<II'
  memcpy(&f.buf[4], &usec, 4);
  can_bb_send(f);                       // C3 fans this out to cone + jugglebot too
}

// ── Time-of-day query (RPC client) ────────────────────────────────────────────
static void send_tod_query() {
  uint8_t payload[Rpc::REQ_HEAD];       // no args
  const uint16_t n = Rpc::pack_request(JbUdp::RpcMethod::TIME_OF_DAY_QUERY,
                                       ++s_req_id, nullptr, 0,
                                       payload, sizeof(payload));
  if (!n) return;
  s_pending_req = s_req_id;
  s_pending = true;
  s_send_us = micros64();
  udp_send_rpc(JbUdp::MsgType::RPC_REQUEST, payload, n);
}

static void on_tod_response(uint16_t /*seq*/, const uint8_t* payload, uint16_t len) {
  uint16_t method, req_id, status, res_len;
  const uint8_t* result;
  if (!Rpc::parse_response(payload, len, &method, &req_id, &status, &result, &res_len)) return;
  if (method != JbUdp::RpcMethod::TIME_OF_DAY_QUERY) return;
  if (!s_pending || req_id != s_pending_req) return;
  if (status != JbUdp::RpcStatus::OK || res_len < sizeof(Rpc::ResultTimeOfDay)) return;

  Rpc::ResultTimeOfDay r;
  memcpy(&r, result, sizeof(r));

  // NTP-style: the Jetson stamped its wall-clock when it processed our request,
  // ~one-way-delay ago. Estimate one-way ≈ RTT/2 and advance the anchor by it so
  // now_wall_us() ≈ the Jetson's current wall-clock at receipt.
  const uint32_t rtt = (uint32_t)(micros64() - s_send_us);
  set_wall_anchor(r.jetson_wall_us + rtt / 2);
  udp_note_rtt(rtt);
  s_pending = false;
}

void time_sync_master_init() {
  udp_on_rpc_response(on_tod_response);
}

void time_sync_step() {
  broadcast_0x7dd();

  const uint64_t now = micros64();
  if (now >= s_next_query_us) {
    send_tod_query();
    // Fast retry (500 ms) until the first anchor lands, then slow drift re-sync.
    const uint64_t interval_us = time_synced()
        ? (uint64_t)TIMEOFDAY_RESYNC_MS * 1000ULL
        : 500000ULL;
    s_next_query_us = now + interval_us;
  }
}

}  // namespace CanBridge
