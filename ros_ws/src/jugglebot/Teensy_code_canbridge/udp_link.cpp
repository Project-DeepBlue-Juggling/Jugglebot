// =============================================================================
//  udp_link.cpp — UDP framing / dispatch / TX implementation
// =============================================================================
#include "udp_link.h"

#include <QNEthernet.h>
#include "freertos_shim.h"
#include "canbridge_config.h"
#include "udp_protocol.h"
#include "net_ethernet.h"
#include "time_base.h"

using namespace qindesign::network;

namespace CanBridge {

// ── Sockets + peer ────────────────────────────────────────────────────────────
static EthernetUDP s_stream;
static EthernetUDP s_rpc;
static const IPAddress kJetsonIP(JETSON_IP[0], JETSON_IP[1], JETSON_IP[2], JETSON_IP[3]);

// ── lwIP serialisation ────────────────────────────────────────────────────────
static SemaphoreHandle_t s_net_mtx = nullptr;
struct NetLock {
  NetLock()  { if (s_net_mtx) xSemaphoreTakeRecursive(s_net_mtx, portMAX_DELAY); }
  ~NetLock() { if (s_net_mtx) xSemaphoreGiveRecursive(s_net_mtx); }
};

// ── Handlers ──────────────────────────────────────────────────────────────────
static FrameHandler s_h_setpoint = nullptr;
static FrameHandler s_h_heartbeat = nullptr;
static FrameHandler s_h_rpc = nullptr;
static FrameHandler s_h_rpc_resp = nullptr;

void udp_on_setpoint(FrameHandler h)      { s_h_setpoint = h; }
void udp_on_heartbeat_j2t(FrameHandler h) { s_h_heartbeat = h; }
void udp_on_rpc_request(FrameHandler h)   { s_h_rpc = h; }
void udp_on_rpc_response(FrameHandler h)  { s_h_rpc_resp = h; }

// ── Sequence counters + stats ─────────────────────────────────────────────────
static uint16_t s_seq_stream_tx = 0;
static uint16_t s_seq_rpc_tx = 0;
static bool     s_have_last_dn_seq = false;
static uint16_t s_last_dn_seq = 0;
static UdpStats s_stats = {0, 0, 0, 0};
static volatile uint32_t s_last_rtt_us = 0;
static volatile uint32_t s_rtt_jitter_us = 0;
static volatile uint64_t s_last_rx_us = 0;

// RX scratch (sized to the max frame; one buffer, RX is single-task).
static uint8_t s_rx_buf[JbUdp::MAX_FRAME];
static uint8_t s_tx_buf[JbUdp::MAX_FRAME];

void udp_link_init() {
  if (!s_net_mtx) s_net_mtx = xSemaphoreCreateRecursiveMutex();
  NetLock lk;
  s_stream.begin(JbUdp::PORT_STREAM);
  s_rpc.begin(JbUdp::PORT_RPC);
}

static void track_downlink_seq(uint16_t seq) {
  if (s_have_last_dn_seq) {
    const uint16_t expected = (uint16_t)(s_last_dn_seq + 1);
    if (seq != expected) s_stats.seq_gaps++;
  }
  s_last_dn_seq = seq;
  s_have_last_dn_seq = true;
}

// Drain one socket; dispatch valid frames. `is_rpc` selects the channel.
static void drain_socket(EthernetUDP& sock, bool is_rpc) {
  for (;;) {
    int n;
    {
      NetLock lk;
      n = sock.parsePacket();
      if (n <= 0) return;
      if (n > (int)sizeof(s_rx_buf)) { sock.read((uint8_t*)nullptr, 0); s_stats.crc_errors++; continue; }
      sock.read(s_rx_buf, n);
    }
    JbUdp::Header hdr;
    const uint8_t* payload = nullptr;
    if (!JbUdp::decode_frame(s_rx_buf, (uint16_t)n, &hdr, &payload)) {
      s_stats.crc_errors++;
      continue;
    }
    s_stats.rx_frames++;
    atomic_write_u64(&s_last_rx_us, micros64());   // 64-bit monotonic (item 14); read as an interval by the link-health watchdog

    if (!is_rpc) track_downlink_seq(hdr.seq);

    switch (hdr.msg_type) {
      case JbUdp::MsgType::SETPOINT:
        if (s_h_setpoint) s_h_setpoint(hdr.seq, payload, hdr.length);
        break;
      case JbUdp::MsgType::HEARTBEAT_J2T:
        if (s_h_heartbeat) s_h_heartbeat(hdr.seq, payload, hdr.length);
        break;
      case JbUdp::MsgType::RPC_REQUEST:
        if (s_h_rpc) s_h_rpc(hdr.seq, payload, hdr.length);
        break;
      case JbUdp::MsgType::RPC_RESPONSE:
        if (s_h_rpc_resp) s_h_rpc_resp(hdr.seq, payload, hdr.length);
        break;
      default:
        // Phase 3 echo path: any otherwise-unhandled valid frame is echoed back
        // to the sender verbatim, exercising the framing+CRC round-trip end to
        // end (the UDP-echo stress test). Recognised typed frames never reach here.
        {
          NetLock lk;
          sock.beginPacket(sock.remoteIP(), sock.remotePort());
          sock.write(s_rx_buf, n);
          sock.endPacket();
          s_stats.tx_frames++;
        }
        break;
    }
  }
}

void udp_link_service() {
  net_ethernet_service();            // pump lwIP (RX/timeouts)
  drain_socket(s_stream, /*is_rpc=*/false);
  drain_socket(s_rpc,    /*is_rpc=*/true);
}

static bool send_to(EthernetUDP& sock, uint16_t port, uint8_t msg_type,
                    uint16_t* seq_ctr, const uint8_t* payload, uint16_t len) {
  // Take the lock FIRST: the seq increment, the encode into the SHARED s_tx_buf,
  // and the transmit must be one atomic step. Multiple tasks (telemetry,
  // heartbeat, RPC) call this concurrently; encoding into the shared buffer or
  // racing the seq++ outside the lock corrupts frames / loses sequence numbers.
  NetLock lk;
  const uint16_t seq = (*seq_ctr)++;
  const uint16_t total = JbUdp::encode_frame(msg_type, seq, payload, len,
                                             s_tx_buf, sizeof(s_tx_buf));
  if (total == 0) return false;
  if (!sock.beginPacket(kJetsonIP, port)) return false;
  sock.write(s_tx_buf, total);
  const bool ok = sock.endPacket();
  if (ok) s_stats.tx_frames++;
  return ok;
}

bool udp_send_stream(uint8_t msg_type, const uint8_t* payload, uint16_t len) {
  return send_to(s_stream, JbUdp::PORT_STREAM, msg_type, &s_seq_stream_tx, payload, len);
}

bool udp_send_rpc(uint8_t msg_type, const uint8_t* payload, uint16_t len) {
  return send_to(s_rpc, JbUdp::PORT_RPC, msg_type, &s_seq_rpc_tx, payload, len);
}

UdpStats udp_link_stats()      { return s_stats; }
uint32_t udp_last_rtt_us()     { return s_last_rtt_us; }
uint32_t udp_rtt_jitter_us()   { return s_rtt_jitter_us; }
uint64_t udp_last_rx_us()      { return atomic_read_u64(&s_last_rx_us); }

void udp_note_rtt(uint32_t rtt_us) {
  // EMA jitter estimate around the RTT mean (alpha = 1/4).
  if (s_last_rtt_us != 0) {
    const int32_t d = (int32_t)rtt_us - (int32_t)s_last_rtt_us;
    const uint32_t adiff = (uint32_t)(d < 0 ? -d : d);
    s_rtt_jitter_us = s_rtt_jitter_us - (s_rtt_jitter_us >> 2) + (adiff >> 2);
  }
  s_last_rtt_us = rtt_us;
}

}  // namespace CanBridge
