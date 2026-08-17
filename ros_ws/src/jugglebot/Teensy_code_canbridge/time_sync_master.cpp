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
#include "leg_interp.h"   // the 500 Hz occupancy census carried on CLOCK_DIAG

namespace CanBridge {

// ── Time-of-day client state ──────────────────────────────────────────────────
static uint16_t s_req_id = 0;
static uint16_t s_pending_req = 0;
static bool     s_pending = false;
static uint64_t s_send_us = 0;          // micros64() at last TOD request
static uint64_t s_next_query_us = 0;    // when to send the next TOD request
// RTT of the exchange that produced the newest accepted anchor (CLOCK_DIAG).
// Written by on_tod_response (net task), read by clock_diag_step (tsync task).
static volatile uint32_t s_last_anchor_rtt_us = 0;

// ── 0x7DD broadcast (matches bus.py broadcast_time: pack('<II', sec, usec)) ───
// Fan out the SAME frame on all three subsystem buses (ADR-0013): BB on the bb
// bus, the cone (or an electronic clapboard) on the cone bus, the platform Teensy
// + ODrives on the jugglebot bus. Buses are named by ROLE — since 2026-07-31 the
// jugglebot and cone roles sit on the CAN2 and CAN3 controllers respectively
// (can_buses.cpp:18-44). Frame ID, payload, and cadence are identical to the
// Jetson-as-master era, so every slave's IIR filter is unaffected. The three
// TXes are independent and partial-failure-tolerant: a false return on one bus
// (e.g. the cone-absent gate withholding the cone bus, or a
// transient mailbox-full) must NOT block the other two, so the cone-disconnect
// case can never starve BB or Jugglebot time-sync.
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
  // Independent TX per bus — call all three regardless of individual results so
  // one bus failing (cone absent, mailbox full) never blocks the others.
  can_bb_send(f);
  can_cone_send(f);
  can_jugglebot_send(f);
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
  // Pair the RTT with the anchor it produced, for CLOCK_DIAG. Stored BEFORE
  // set_wall_anchor deliberately: the emitter (task_time_sync) decides there is
  // something new to send by observing the anchor's bumped anchor_seq, so
  // publishing the RTT first makes "new seq visible ⇒ its RTT already stored" a
  // happens-before rather than a race with a two-instruction window. Reading
  // udp_last_rtt_us() at emit time instead would have been right only by
  // cadence accident (anchors are 30 s apart, the emit follows within 10 ms) —
  // true today, and silently wrong the moment the resync interval changes.
  // Single naturally-aligned word ⇒ the store and the cross-task load are atomic.
  s_last_anchor_rtt_us = rtt;
  set_wall_anchor(r.jetson_wall_us + rtt / 2);
  udp_note_rtt(rtt);
  s_pending = false;
}

// ── CLOCK_DIAG (0x8F) emitter — FW 11 ─────────────────────────────────────────
//  One frame per ACCEPTED anchor: the clock-discipline sample time_base captured,
//  plus the 500 Hz interp occupancy differenced over the window since the last
//  emit. Cadence therefore follows the anchor cadence — ~30 s in steady state,
//  500 ms during the pre-first-anchor fast retry.
//
//  WHY THIS RUNS HERE AND NOT IN on_tod_response. on_tod_response is a udp_link
//  RX handler, dispatched from inside drain_socket's bounded drain loop on the
//  net task — the same loop that carries every SETPOINT. udp_link.h's own
//  contract for handlers is "fast / non-blocking", and a udp_send_stream() there
//  would build and transmit a frame under NetLock from inside the RX drain
//  (recursive, so not a deadlock, but it lengthens the path the 40 Hz setpoint
//  stream shares). Deferring to task_time_sync costs at most one 100 Hz tick
//  (<= 10 ms) of emit latency against a 30 s cadence, and costs the SAMPLE
//  nothing at all: t_local_us was stamped at the anchor, so the delay cannot
//  move a timestamp — only the frame's arrival, which no consumer times from.
static uint32_t s_diag_last_seq     = 0;   // anchor_seq of the last sample emitted
static uint32_t s_diag_prev_ticks   = 0;   // interp census snapshots at that emit
static uint32_t s_diag_prev_recover = 0;
static uint32_t s_diag_prev_extrap  = 0;

static void clock_diag_step() {
  ClockAnchorSample s;
  if (!clock_last_anchor(&s)) return;         // no anchor has landed yet
  if (s.anchor_seq == s_diag_last_seq) return; // already emitted this one
  s_diag_last_seq = s.anchor_seq;

  // Read the census ONCE and difference against the previous emit's snapshot.
  // Unsigned subtraction is wrap-correct, so the u32 rollover at ~99 days of
  // uptime needs no special case (and this arc is precisely about long uptimes).
  // Numerators FIRST, denominator LAST: the 500 Hz ISR can fire between these
  // reads, and with the denominator freshest an interleaved tick lands in
  // interp_ticks while the numerators miss it — the window's ratio can only
  // round DOWN, never report extrap_ticks > interp_ticks (>100 % duty).
  const uint32_t recover = interp_recover_slew_ticks();
  const uint32_t extrap  = interp_extrap_ticks();
  const uint32_t ticks   = interp_tick_count();

  JbUdp::ClockDiagPayload p{};
  p.t_local_us         = s.t_local_us;
  p.jetson_wall_us     = s.jetson_wall_us;
  p.dt_local_us        = s.dt_local_us;
  p.rtt_us             = s_last_anchor_rtt_us;   // the RTT of THIS anchor's exchange (see on_tod_response)
  p.err_us             = s.err_us;
  p.freq_ppb           = s.freq_ppb;
  p.anchor_seq         = s.anchor_seq;
  p.interp_ticks       = ticks   - s_diag_prev_ticks;
  p.recover_slew_ticks = recover - s_diag_prev_recover;
  p.extrap_ticks       = extrap  - s_diag_prev_extrap;
  p.flags              = s.flags;

  // Advance the snapshots even if the send fails: the counters are a census of
  // WHAT HAPPENED, and holding them back on a dropped frame would silently fold
  // two windows into one and manufacture an occupancy spike. A lost frame is
  // already detectable — anchor_seq skips.
  s_diag_prev_ticks   = ticks;
  s_diag_prev_recover = recover;
  s_diag_prev_extrap  = extrap;

  udp_send_stream(JbUdp::MsgType::CLOCK_DIAG, (const uint8_t*)&p, sizeof(p));
}

void time_sync_master_init() {
  udp_on_rpc_response(on_tod_response);
  // Baseline the occupancy census here rather than at zero, so the first
  // window is "since init" instead of "since power-on" — otherwise the first
  // CLOCK_DIAG's denominator silently includes whatever ran during setup().
  // Same read order as clock_diag_step (numerators first, denominator last).
  s_diag_prev_recover = interp_recover_slew_ticks();
  s_diag_prev_extrap  = interp_extrap_ticks();
  s_diag_prev_ticks   = interp_tick_count();
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

  clock_diag_step();   // emits only on the tick after a NEW anchor is accepted
}

}  // namespace CanBridge
